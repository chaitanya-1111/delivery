#!/usr/bin/env python3
"""
lidar_diagnostics_node.py

PRODUCTION ROLE:
  Continuously monitors the RPLidar and publishes structured diagnostic
  data to /diagnostics. This integrates with ROS 2's standard diagnostic
  framework, allowing monitoring tools (rqt_robot_monitor, Foxglove, etc.)
  to display lidar health in dashboards.

  If the lidar stops publishing (USB disconnect, driver crash, hardware
  failure), this node raises a diagnostic ERROR immediately so operators
  can respond.

  It also computes and logs:
    - Actual scan frequency (should be ~8-10 Hz for C1/M2)
    - Points per scan (should be ~360-720 per scan)
    - Percentage of valid (non-NaN) points
    - Min/max range statistics

TOPICS SUBSCRIBED:
  /scan_raw   (sensor_msgs/LaserScan) — raw scan from rplidar_ros driver
  /scan       (sensor_msgs/LaserScan) — filtered scan (for comparison)

TOPICS PUBLISHED:
  /diagnostics (diagnostic_msgs/DiagnosticArray) — standard ROS diagnostics
  /lidar/health (std_msgs/String)                — simple: OK / WARN / ERROR
  /lidar/stats  (std_msgs/String JSON)           — frequency, valid%, etc.
"""

import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class LidarDiagnosticsNode(Node):

    # ── Thresholds ────────────────────────────────────────────────
    # Scan frequency: RPLidar C1/M2 at "Standard" mode = ~7-10 Hz
    FREQ_MIN_HZ       = 5.0    # below this = WARN (motor may be slowing)
    FREQ_CRITICAL_HZ  = 2.0    # below this = ERROR (lidar failing)
    FREQ_TARGET_HZ    = 8.0    # expected nominal frequency

    # Valid point ratio: what % of scan points should be valid (non-NaN)
    # In a typical restaurant: 60-95% valid (some open angles hit far walls)
    VALID_PCT_WARN    = 0.30   # below 30% valid = WARN (too much open space or noise)

    # Timeout: if no scan received for this long, declare ERROR
    TIMEOUT_SEC       = 3.0

    def __init__(self):
        super().__init__('lidar_diagnostics_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('expected_frequency', self.FREQ_TARGET_HZ)
        self.declare_parameter('timeout_sec', self.TIMEOUT_SEC)
        self.declare_parameter('hardware_id', 'RPLidar C1/M2')

        self._expected_freq = self.get_parameter('expected_frequency').value
        self._timeout       = self.get_parameter('timeout_sec').value
        self._hw_id         = self.get_parameter('hardware_id').value

        # ── QoS: sensor data profile ──────────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ── Subscriptions ─────────────────────────────────────────
        self._raw_sub = self.create_subscription(
            LaserScan, '/scan_raw', self._on_raw_scan, sensor_qos)
        self._filtered_sub = self.create_subscription(
            LaserScan, '/scan', self._on_filtered_scan, sensor_qos)

        # ── Publishers ────────────────────────────────────────────
        self._diag_pub   = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._health_pub = self.create_publisher(String, '/lidar/health', 10)
        self._stats_pub  = self.create_publisher(String, '/lidar/stats', 10)

        # ── State ──────────────────────────────────────────────────
        # Rolling window of scan timestamps for frequency calculation
        # Keep last 20 scan timestamps → smooth frequency estimate
        self._raw_timestamps: deque = deque(maxlen=20)
        self._filtered_timestamps: deque = deque(maxlen=20)

        self._last_raw_scan: LaserScan = None
        self._last_scan_time: float = None
        self._scan_count: int = 0

        # ── Diagnostics timer: run at 2Hz ─────────────────────────
        self._diag_timer = self.create_timer(0.5, self._publish_diagnostics)

        # ── Stats timer: run at 1Hz ───────────────────────────────
        self._stats_timer = self.create_timer(1.0, self._publish_stats)

        self.get_logger().info(
            f'LidarDiagnosticsNode started. '
            f'Monitoring /scan_raw and /scan. '
            f'Expected frequency: {self._expected_freq:.1f} Hz'
        )

    # ── Scan Callbacks ────────────────────────────────────────────
    def _on_raw_scan(self, msg: LaserScan):
        """Record raw scan arrival for frequency computation."""
        now = time.monotonic()
        self._raw_timestamps.append(now)
        self._last_raw_scan = msg
        self._last_scan_time = now
        self._scan_count += 1

    def _on_filtered_scan(self, msg: LaserScan):
        """Record filtered scan arrival."""
        self._filtered_timestamps.append(time.monotonic())

    # ── Frequency Calculation ─────────────────────────────────────
    def _compute_frequency(self, timestamps: deque) -> float:
        """
        Compute actual scan frequency from a window of timestamps.

        Method: (N-1 intervals) / (total time span)
        This is more accurate than averaging individual intervals.
        """
        if len(timestamps) < 2:
            return 0.0
        elapsed = timestamps[-1] - timestamps[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(timestamps) - 1) / elapsed

    # ── Scan Analysis ─────────────────────────────────────────────
    def _analyze_scan(self, scan: LaserScan) -> dict:
        """
        Compute quality statistics for a single scan.

        Returns dict with:
          total_points: total number of range readings
          valid_points: non-NaN, non-Inf readings in [min_range, max_range]
          valid_pct:    valid_points / total_points
          min_range:    closest valid return (meters)
          max_range:    farthest valid return (meters)
          mean_range:   average range of valid points
        """
        ranges = scan.ranges
        total = len(ranges)
        valid_vals = []

        for r in ranges:
            if (math.isfinite(r) and
                    scan.range_min <= r <= scan.range_max):
                valid_vals.append(r)

        valid_count = len(valid_vals)
        valid_pct   = valid_count / total if total > 0 else 0.0

        stats = {
            'total_points' : total,
            'valid_points' : valid_count,
            'valid_pct'    : round(valid_pct, 3),
            'min_range'    : round(min(valid_vals), 3) if valid_vals else None,
            'max_range'    : round(max(valid_vals), 3) if valid_vals else None,
            'mean_range'   : round(sum(valid_vals)/len(valid_vals), 3) if valid_vals else None,
        }
        return stats

    # ── Diagnostics Publisher (2Hz) ───────────────────────────────
    def _publish_diagnostics(self):
        """
        Build and publish a DiagnosticArray message.

        Levels:
          DiagnosticStatus.OK    (0) → everything nominal
          DiagnosticStatus.WARN  (1) → degraded but functional
          DiagnosticStatus.ERROR (2) → lidar not usable, intervention needed
          DiagnosticStatus.STALE (3) → no data received
        """
        now = time.monotonic()
        status = DiagnosticStatus()
        status.name = 'RPLidar'
        status.hardware_id = self._hw_id

        # ── Check: is lidar even publishing? ──────────────────────
        if self._last_scan_time is None:
            status.level   = DiagnosticStatus.STALE
            status.message = 'No scan received yet. Is rplidar_ros running?'
            self.get_logger().warn('Lidar: no scan received yet!')

        elif (now - self._last_scan_time) > self._timeout:
            elapsed = now - self._last_scan_time
            status.level   = DiagnosticStatus.ERROR
            status.message = f'Lidar TIMEOUT: no scan for {elapsed:.1f}s! Check USB connection.'
            self.get_logger().error(
                f'LIDAR TIMEOUT: no scan for {elapsed:.1f}s. '
                f'Check: ls /dev/ttyUSB* or ls /dev/rplidar'
            )

        else:
            # ── Compute metrics ───────────────────────────────────
            raw_freq    = self._compute_frequency(self._raw_timestamps)
            filt_freq   = self._compute_frequency(self._filtered_timestamps)
            scan_stats  = self._analyze_scan(self._last_raw_scan) if self._last_raw_scan else {}

            # Build key-value pairs for diagnostic dashboard
            kv = []
            kv.append(KeyValue(key='Scan frequency (raw)',
                               value=f'{raw_freq:.2f} Hz'))
            kv.append(KeyValue(key='Scan frequency (filtered)',
                               value=f'{filt_freq:.2f} Hz'))
            kv.append(KeyValue(key='Total scans received',
                               value=str(self._scan_count)))
            if scan_stats:
                kv.append(KeyValue(key='Points per scan',
                                   value=str(scan_stats['total_points'])))
                kv.append(KeyValue(key='Valid points',
                                   value=f"{scan_stats['valid_points']} "
                                         f"({scan_stats['valid_pct']*100:.1f}%)"))
                if scan_stats['min_range'] is not None:
                    kv.append(KeyValue(key='Range (min/mean/max)',
                                       value=f"{scan_stats['min_range']:.2f}m / "
                                             f"{scan_stats['mean_range']:.2f}m / "
                                             f"{scan_stats['max_range']:.2f}m"))

            status.values = kv

            # ── Determine health level ─────────────────────────────
            if raw_freq < self.FREQ_CRITICAL_HZ:
                status.level   = DiagnosticStatus.ERROR
                status.message = (
                    f'Lidar frequency critically low: {raw_freq:.2f} Hz '
                    f'(min: {self.FREQ_CRITICAL_HZ} Hz). Motor may be failing.'
                )
                self.get_logger().error(status.message)

            elif raw_freq < self.FREQ_MIN_HZ:
                status.level   = DiagnosticStatus.WARN
                status.message = (
                    f'Lidar frequency low: {raw_freq:.2f} Hz '
                    f'(expected: {self._expected_freq:.1f} Hz)'
                )
                self.get_logger().warn(status.message)

            elif scan_stats.get('valid_pct', 1.0) < self.VALID_PCT_WARN:
                status.level   = DiagnosticStatus.WARN
                status.message = (
                    f"Low valid point ratio: {scan_stats['valid_pct']*100:.1f}%. "
                    f'Check for glass walls, mirrors, or obstructions near lidar.'
                )
                self.get_logger().warn(status.message)

            else:
                status.level   = DiagnosticStatus.OK
                status.message = (
                    f'OK — {raw_freq:.1f} Hz, '
                    f"{scan_stats.get('valid_points', '?')} valid pts/scan"
                )

        # ── Publish DiagnosticArray ───────────────────────────────
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self._diag_pub.publish(arr)

        # ── Publish simple health string ──────────────────────────
        level_map = {
            DiagnosticStatus.OK:    'OK',
            DiagnosticStatus.WARN:  'WARN',
            DiagnosticStatus.ERROR: 'ERROR',
            DiagnosticStatus.STALE: 'STALE',
        }
        health_msg = String()
        health_msg.data = level_map.get(status.level, 'UNKNOWN')
        self._health_pub.publish(health_msg)

    # ── Stats Publisher (1Hz) ─────────────────────────────────────
    def _publish_stats(self):
        """Publish detailed scan statistics as JSON for monitoring tools."""
        if self._last_raw_scan is None:
            return

        raw_freq   = self._compute_frequency(self._raw_timestamps)
        filt_freq  = self._compute_frequency(self._filtered_timestamps)
        scan_stats = self._analyze_scan(self._last_raw_scan)

        stats = {
            'frequency_raw_hz'      : round(raw_freq,  2),
            'frequency_filtered_hz' : round(filt_freq, 2),
            'total_scans'           : self._scan_count,
            **scan_stats,
        }

        msg = String()
        msg.data = json.dumps(stats)
        self._stats_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
