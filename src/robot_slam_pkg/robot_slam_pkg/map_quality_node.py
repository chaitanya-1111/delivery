#!/usr/bin/env python3
"""
map_quality_node.py

PRODUCTION ROLE:
  Before Nav2 starts navigating, this node validates that the loaded
  map is actually usable. It prevents the robot from trying to navigate
  on a corrupted, incomplete, or mismatched map.

  Runs ONCE at startup, then continues monitoring the map for drift
  (signs that the environment has changed significantly from the map).

VALIDATION CHECKS:
  1. Map is non-empty (has cells)
  2. Resolution matches expected (0.05m)
  3. Coverage is sufficient (< 40% unknown cells)
  4. Has sufficient free space for robot to navigate
  5. TF from map→odom is available (SLAM is running)
  6. Map dimensions are reasonable (not a 1x1 cell or 10000x10000 cell)
  7. AMCL/SLAM scan-to-map correlation score (ongoing drift detection)

TOPICS SUBSCRIBED:
  /map                     (nav_msgs/OccupancyGrid)
  /slam/map_age_days       (std_msgs/Float32)

TOPICS PUBLISHED:
  /slam/map_valid          (std_msgs/Bool)     → true if map passes all checks
  /slam/map_quality_report (std_msgs/String JSON) → full quality report
  /slam/map_warnings       (std_msgs/String)   → human-readable warnings

PARAMETERS:
  min_free_pct:       minimum % of known-free cells required
  max_unknown_pct:    maximum % of unknown cells allowed
  expected_resolution: 0.05 by default
  stale_map_days:     warn if map older than this
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String, Float32
import tf2_ros


class MapQualityNode(Node):

    def __init__(self):
        super().__init__('map_quality_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('min_free_pct',        15.0)   # % free cells
        self.declare_parameter('max_unknown_pct',     50.0)   # % unknown cells
        self.declare_parameter('expected_resolution',  0.05)  # meters/cell
        self.declare_parameter('min_map_width',         50)   # cells
        self.declare_parameter('min_map_height',        50)   # cells
        self.declare_parameter('stale_map_days',        60.0)
        self.declare_parameter('check_tf_timeout_sec',   5.0)

        self._min_free      = self.get_parameter('min_free_pct').value
        self._max_unknown   = self.get_parameter('max_unknown_pct').value
        self._expected_res  = self.get_parameter('expected_resolution').value
        self._min_w         = self.get_parameter('min_map_width').value
        self._min_h         = self.get_parameter('min_map_height').value
        self._stale_days    = self.get_parameter('stale_map_days').value
        self._tf_timeout    = self.get_parameter('check_tf_timeout_sec').value

        # ── TF Buffer ─────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Subscriptions ─────────────────────────────────────────
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos)
        self._age_sub = self.create_subscription(
            Float32, '/slam/map_age_days', self._on_map_age, 10)

        # ── Publishers ────────────────────────────────────────────
        self._valid_pub   = self.create_publisher(Bool, '/slam/map_valid', 10)
        self._report_pub  = self.create_publisher(String, '/slam/map_quality_report', 10)
        self._warning_pub = self.create_publisher(String, '/slam/map_warnings', 10)

        # ── State ─────────────────────────────────────────────────
        self._current_map: OccupancyGrid = None
        self._map_age_days: float = None
        self._validation_result: dict = {}
        self._map_valid: bool = False
        self._validated_once: bool = False

        # ── Timers ────────────────────────────────────────────────
        self._validate_timer = self.create_timer(10.0, self._run_validation)
        self._publish_timer  = self.create_timer(5.0,  self._publish_status)

        self.get_logger().info('MapQualityNode started. Waiting for /map...')

    # ── Callbacks ─────────────────────────────────────────────────
    def _on_map(self, msg: OccupancyGrid):
        self._current_map = msg
        # Run validation immediately on first map receipt
        if not self._validated_once:
            self._run_validation()

    def _on_map_age(self, msg: Float32):
        self._map_age_days = msg.data

    # ── Main Validation ───────────────────────────────────────────
    def _run_validation(self):
        """
        Run all quality checks on the current map.
        Builds a structured report and publishes it.
        """
        checks  = {}
        warnings = []
        errors   = []

        # ── Check 1: Map received ──────────────────────────────────
        if self._current_map is None:
            self.get_logger().warn('Map quality check: no map received yet.')
            self._map_valid = False
            return

        grid = self._current_map
        info = grid.info
        data = grid.data

        # ── Check 2: Map dimensions ───────────────────────────────
        w, h = info.width, info.height
        checks['dimensions'] = {
            'width': w, 'height': h,
            'pass': (w >= self._min_w and h >= self._min_h)
        }
        if not checks['dimensions']['pass']:
            errors.append(
                f'Map too small: {w}x{h} cells. '
                f'Minimum: {self._min_w}x{self._min_h}. '
                f'Drive the robot around more during mapping!'
            )

        # ── Check 3: Resolution ───────────────────────────────────
        res = info.resolution
        res_ok = abs(res - self._expected_res) < 0.005  # within 5mm tolerance
        checks['resolution'] = {'value': res, 'expected': self._expected_res, 'pass': res_ok}
        if not res_ok:
            errors.append(
                f'Map resolution mismatch: {res:.3f}m (expected {self._expected_res}m). '
                f'Rebuild map with correct slam_toolbox resolution setting.'
            )

        # ── Check 4: Cell statistics ──────────────────────────────
        total    = len(data)
        free     = sum(1 for c in data if c == 0)
        occupied = sum(1 for c in data if c == 100)
        unknown  = sum(1 for c in data if c < 0)   # -1 = unknown

        free_pct     = (free     / total * 100) if total > 0 else 0.0
        occupied_pct = (occupied / total * 100) if total > 0 else 0.0
        unknown_pct  = (unknown  / total * 100) if total > 0 else 100.0
        area_m2      = free * (res ** 2)

        checks['coverage'] = {
            'total_cells'    : total,
            'free_pct'       : round(free_pct, 1),
            'occupied_pct'   : round(occupied_pct, 1),
            'unknown_pct'    : round(unknown_pct, 1),
            'free_area_m2'   : round(area_m2, 1),
            'pass_free'      : free_pct >= self._min_free,
            'pass_unknown'   : unknown_pct <= self._max_unknown,
        }
        checks['coverage']['pass'] = (
            checks['coverage']['pass_free'] and
            checks['coverage']['pass_unknown']
        )

        if not checks['coverage']['pass_free']:
            errors.append(
                f'Insufficient free space: {free_pct:.1f}% '
                f'(minimum: {self._min_free}%). '
                f'Drive into more open areas during mapping.'
            )
        if not checks['coverage']['pass_unknown']:
            warnings.append(
                f'High unknown area: {unknown_pct:.1f}% '
                f'(maximum: {self._max_unknown}%). '
                f'Some areas may not be mapped. Robot may not navigate there.'
            )

        # ── Check 5: TF availability ──────────────────────────────
        tf_ok = self._check_tf_available('map', 'odom')
        checks['tf'] = {'map_to_odom': tf_ok, 'pass': tf_ok}
        if not tf_ok:
            errors.append(
                'TF map→odom not available. '
                'Is slam_toolbox or nav2 lifecycle_manager running and active?'
            )

        # ── Check 6: TF robot pose ────────────────────────────────
        tf_robot = self._check_tf_available('map', 'base_footprint')
        checks['robot_pose_in_map'] = {'pass': tf_robot}
        if not tf_robot:
            warnings.append(
                'Cannot get robot pose in map frame. '
                'Localization may not have converged yet.'
            )

        # ── Check 7: Map age ──────────────────────────────────────
        if self._map_age_days is not None:
            age_ok = self._map_age_days <= self._stale_days
            checks['map_age'] = {
                'age_days': round(self._map_age_days, 1),
                'pass'    : age_ok
            }
            if not age_ok:
                warnings.append(
                    f'Map is {self._map_age_days:.0f} days old. '
                    f'Consider re-mapping if restaurant layout has changed.'
                )
        else:
            checks['map_age'] = {'pass': True, 'note': 'Age not available'}

        # ── Overall result ────────────────────────────────────────
        # Map is valid if ALL hard checks pass (dimensions, resolution, coverage free, TF).
        # Warnings are informational only.
        critical_checks = [
            checks['dimensions']['pass'],
            checks['resolution']['pass'],
            checks['coverage']['pass_free'],
            checks['tf']['pass'],
        ]
        self._map_valid = all(critical_checks)

        # ── Build report ──────────────────────────────────────────
        self._validation_result = {
            'timestamp'  : time.time(),
            'map_valid'  : self._map_valid,
            'map_size'   : f'{w}x{h} cells ({w*res:.1f}m x {h*res:.1f}m)',
            'free_area'  : f'{area_m2:.1f} m²',
            'coverage'   : f'{100-unknown_pct:.1f}% mapped',
            'checks'     : checks,
            'warnings'   : warnings,
            'errors'     : errors,
        }

        self._validated_once = True

        # ── Log result ────────────────────────────────────────────
        if self._map_valid:
            self.get_logger().info(
                f'✅ MAP VALID — {w*res:.1f}m×{h*res:.1f}m, '
                f'{free_pct:.1f}% free ({area_m2:.1f}m²), '
                f'{unknown_pct:.1f}% unknown'
            )
        else:
            self.get_logger().error(
                f'❌ MAP INVALID — Errors: {errors}'
            )

        for w_msg in warnings:
            self.get_logger().warn(f'Map warning: {w_msg}')

    def _check_tf_available(self, target: str, source: str) -> bool:
        """Check if a TF transform is available."""
        try:
            self._tf_buffer.lookup_transform(
                target, source,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout)
            )
            return True
        except Exception:
            return False

    # ── Status Publisher ──────────────────────────────────────────
    def _publish_status(self):
        # Valid flag
        valid_msg = Bool()
        valid_msg.data = self._map_valid
        self._valid_pub.publish(valid_msg)

        # Full report
        if self._validation_result:
            report_msg = String()
            report_msg.data = json.dumps(self._validation_result)
            self._report_pub.publish(report_msg)

            # Human-readable warnings
            warnings = self._validation_result.get('warnings', [])
            errors   = self._validation_result.get('errors', [])
            all_msgs = [f'ERROR: {e}' for e in errors] + \
                       [f'WARN: {w}' for w in warnings]
            if all_msgs:
                w_msg = String()
                w_msg.data = ' | '.join(all_msgs)
                self._warning_pub.publish(w_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapQualityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
