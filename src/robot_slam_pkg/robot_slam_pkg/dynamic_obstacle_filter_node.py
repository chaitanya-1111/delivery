#!/usr/bin/env python3
"""
dynamic_obstacle_filter_node.py

PRODUCTION ROLE:
  During mapping, if a staff member or customer walks through the scan
  plane, their body registers as a solid obstacle in the occupancy grid.
  Over multiple passes, moving obstacles leave "smear artifacts" in the
  map — phantom walls in the middle of corridors.

  This node detects scan points that appear inconsistently across
  consecutive scans (they appear then disappear = moving object) and
  removes them BEFORE they reach slam_toolbox.

  PIPELINE:
    /scan (from Step 1 filter)
        ↓ dynamic_obstacle_filter_node (this node)
        ↓
    /scan_filtered_dynamic  → slam_toolbox (clean, static-only)

  DETECTION ALGORITHM: Temporal Consistency Voting
    For each angular bin, keep the last N range measurements.
    If a measurement is an outlier vs the median of its window,
    mark it as dynamic (temporary) and replace with max_range (no return).
    This is robust, runs at scan frequency, uses no ML.

  PARAMETERS:
    window_size:          how many scans to compare (default 5)
    consistency_threshold: how different a point must be to be flagged
    enabled:              can be disabled in localization mode
                          (less important when not building the map)

TOPICS SUBSCRIBED:
  /scan                    (sensor_msgs/LaserScan) — filtered scan from Step 1

TOPICS PUBLISHED:
  /scan_for_slam           (sensor_msgs/LaserScan) — dynamic-obstacle-free scan
  /slam/dynamic_mask       (sensor_msgs/LaserScan) — shows which points were removed
  /slam/dynamic_point_count (std_msgs/Int32)        — how many points removed this scan
"""

import collections
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32


class DynamicObstacleFilterNode(Node):

    def __init__(self):
        super().__init__('dynamic_obstacle_filter_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('window_size', 5)
        self.declare_parameter('consistency_threshold', 0.3)  # meters
        self.declare_parameter('enabled', True)
        self.declare_parameter('min_dynamic_count', 3)   # bins must agree

        self._window       = self.get_parameter('window_size').value
        self._threshold    = self.get_parameter('consistency_threshold').value
        self._enabled      = self.get_parameter('enabled').value
        self._min_dyn      = self.get_parameter('min_dynamic_count').value

        # ── QoS ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ── Subscriptions ─────────────────────────────────────────
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, sensor_qos)

        # ── Publishers ────────────────────────────────────────────
        # /scan_for_slam: pass this to slam_toolbox instead of /scan
        self._clean_pub = self.create_publisher(
            LaserScan, '/scan_for_slam', sensor_qos)

        self._mask_pub  = self.create_publisher(
            LaserScan, '/slam/dynamic_mask', sensor_qos)

        self._count_pub = self.create_publisher(
            Int32, '/slam/dynamic_point_count', 10)

        # ── State: ring buffer of recent scans per angular bin ────
        # _history[bin_index] = deque of recent range values
        # Populated on first scan (we don't know num_bins until then)
        self._history: list = None
        self._num_bins: int  = 0

        self._total_filtered: int = 0
        self._scan_count: int = 0

        if not self._enabled:
            self.get_logger().info(
                'DynamicObstacleFilterNode: DISABLED. '
                'Passing /scan through to /scan_for_slam unchanged.'
            )
        else:
            self.get_logger().info(
                f'DynamicObstacleFilterNode started. '
                f'window={self._window}, threshold={self._threshold}m. '
                f'Filtering moving people from SLAM scan.'
            )

    def _on_scan(self, msg: LaserScan):
        """
        Process each incoming scan.
        If disabled, pass through unchanged.
        If enabled, remove dynamic points.
        """
        self._scan_count += 1

        if not self._enabled:
            # Pass through directly
            self._clean_pub.publish(msg)
            return

        # ── Initialize history buffer on first scan ───────────────
        num_bins = len(msg.ranges)
        if self._history is None or self._num_bins != num_bins:
            self._num_bins = num_bins
            self._history = [
                collections.deque(maxlen=self._window)
                for _ in range(num_bins)
            ]
            self.get_logger().info(
                f'Initialized dynamic filter: {num_bins} angular bins, '
                f'window={self._window} scans.'
            )

        ranges      = list(msg.ranges)
        max_range   = msg.range_max
        dynamic_mask = [False] * num_bins

        # ── Add current scan to history ───────────────────────────
        for i, r in enumerate(ranges):
            # Store only valid (finite, in-range) measurements
            if math.isfinite(r) and msg.range_min <= r <= max_range:
                self._history[i].append(r)
            # Don't add NaN/Inf to history (would corrupt median)

        # ── Dynamic detection: temporal consistency voting ─────────
        # For each bin, compute the median of its recent history.
        # If the current reading deviates from the median by more
        # than threshold AND the history is full (enough evidence),
        # mark as dynamic.
        removed_count = 0

        for i in range(num_bins):
            hist = self._history[i]
            if len(hist) < max(2, self._window // 2):
                # Not enough history yet — can't make a determination
                continue

            current = ranges[i]
            if not (math.isfinite(current) and
                    msg.range_min <= current <= max_range):
                # Already invalid, skip
                continue

            # Compute median of history (excluding current reading)
            hist_list = sorted(hist)[:-1] if len(hist) > 1 else list(hist)
            if not hist_list:
                continue

            median = hist_list[len(hist_list) // 2]

            # Is the current reading significantly different from median?
            if abs(current - median) > self._threshold:
                # This point is inconsistent with recent history = dynamic
                dynamic_mask[i] = True
                removed_count  += 1

        self._total_filtered += removed_count

        # ── Build clean scan (remove dynamic points) ───────────────
        clean_ranges = list(ranges)
        mask_ranges  = [max_range] * num_bins  # visualization: max = no point

        for i in range(num_bins):
            if dynamic_mask[i]:
                clean_ranges[i] = float('nan')  # remove from SLAM
                mask_ranges[i]  = ranges[i]     # show what was removed

        # ── Publish clean scan ─────────────────────────────────────
        clean_msg        = LaserScan()
        clean_msg.header = msg.header
        clean_msg.angle_min         = msg.angle_min
        clean_msg.angle_max         = msg.angle_max
        clean_msg.angle_increment   = msg.angle_increment
        clean_msg.time_increment    = msg.time_increment
        clean_msg.scan_time         = msg.scan_time
        clean_msg.range_min         = msg.range_min
        clean_msg.range_max         = msg.range_max
        clean_msg.ranges            = clean_ranges
        clean_msg.intensities       = msg.intensities

        self._clean_pub.publish(clean_msg)

        # ── Publish mask (what was removed) ───────────────────────
        mask_msg        = LaserScan()
        mask_msg.header = msg.header
        mask_msg.angle_min       = msg.angle_min
        mask_msg.angle_max       = msg.angle_max
        mask_msg.angle_increment = msg.angle_increment
        mask_msg.time_increment  = msg.time_increment
        mask_msg.scan_time       = msg.scan_time
        mask_msg.range_min       = msg.range_min
        mask_msg.range_max       = msg.range_max
        mask_msg.ranges          = mask_ranges
        self._mask_pub.publish(mask_msg)

        # ── Publish count ──────────────────────────────────────────
        cnt_msg = Int32()
        cnt_msg.data = removed_count
        self._count_pub.publish(cnt_msg)

        # ── Log significant filtering events ──────────────────────
        if removed_count > 30:  # > 30 bins filtered = likely a person
            self.get_logger().info(
                f'Dynamic obstacle detected: {removed_count} points filtered '
                f'from scan (likely a person in the scan plane). '
                f'Total filtered this session: {self._total_filtered}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down. Total dynamic points filtered: '
            f'{node._total_filtered} across {node._scan_count} scans.'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
