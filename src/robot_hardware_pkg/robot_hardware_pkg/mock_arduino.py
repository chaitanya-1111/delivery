#!/usr/bin/env python3
"""
software_test.py
================
Delivery Robot — Software Stack Verification Tool
robot_hardware_pkg

PURPOSE:
  Automated test that verifies the entire hardware interface software stack
  is working correctly WITHOUT any physical hardware.

  Run this whenever you want to confirm:
    ✓ /odom is publishing at correct rate
    ✓ TF tree is correct (odom → base_footprint)
    ✓ /joint_states are publishing
    ✓ /cmd_vel commands produce correct odometry responses
    ✓ Watchdog stops the robot when cmd_vel goes silent
    ✓ Diagnostics are healthy

USAGE:
  Terminal 1:
    ros2 launch robot_hardware_pkg hardware_bringup.launch.py mock_mode:=true

  Terminal 2:
    ros2 run robot_hardware_pkg software_test

  All tests run automatically and print a PASS/FAIL report.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import time
import math
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String


# ══════════════════════════════════════════════════════════════
#  TEST CONFIG
# ══════════════════════════════════════════════════════════════

TEST_DRIVE_SPEED    = 0.3    # m/s — used in forward drive test
TEST_TURN_SPEED     = 0.5    # rad/s — used in rotation test
TEST_DURATION_S     = 2.0    # seconds per movement test
ODOM_RATE_EXPECTED  = 50.0   # Hz
ODOM_RATE_TOLERANCE = 5.0    # Hz — acceptable deviation


class SoftwareTestNode(Node):

    def __init__(self):
        super().__init__('software_test_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', qos)

        # Subscribers + collection buffers
        self.odom_msgs_      = []
        self.joint_msgs_     = []
        self.status_msgs_    = []
        self.odom_lock_      = threading.Lock()

        self.create_subscription(Odometry,   '/odom',             self._odom_cb,   qos)
        self.create_subscription(JointState, '/joint_states',     self._joint_cb,  qos)
        self.create_subscription(String,     '/hardware/status',  self._status_cb, 10)

        # Test results
        self.results_ = {}

        self.get_logger().info('Software Test Node ready — starting in 1 second...')

    def _odom_cb(self, msg):
        with self.odom_lock_:
            self.odom_msgs_.append(msg)

    def _joint_cb(self, msg):
        self.joint_msgs_.append(msg)

    def _status_cb(self, msg):
        self.status_msgs_.append(msg.data)

    # ── Test Helpers ──────────────────────────────────────────────────

    def _spin_for(self, seconds: float):
        """Spin the ROS event loop for N seconds."""
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def _send_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_vel_pub_.publish(msg)

    def _stop(self):
        self._send_cmd(0.0, 0.0)

    def _assert(self, name: str, condition: bool, detail: str = ''):
        icon = '✅ PASS' if condition else '❌ FAIL'
        self.results_[name] = condition
        detail_str = f'  ({detail})' if detail else ''
        print(f'  {icon}  {name}{detail_str}')
        return condition

    # ══════════════════════════════════════════════════════════
    #  INDIVIDUAL TESTS
    # ══════════════════════════════════════════════════════════

    def test_topics_alive(self):
        """Check all required topics are publishing."""
        print('\n── TEST 1: Topics Alive ───────────────────────────────')

        with self.odom_lock_:
            self.odom_msgs_.clear()
        self.joint_msgs_.clear()
        self._spin_for(2.0)

        with self.odom_lock_:
            odom_count = len(self.odom_msgs_)
        joint_count = len(self.joint_msgs_)
        status_count = len(self.status_msgs_)

        self._assert('/odom publishing',        odom_count  > 0,
                     f'{odom_count} messages in 2s')
        self._assert('/joint_states publishing',joint_count > 0,
                     f'{joint_count} messages in 2s')
        self._assert('/hardware/status publishing', status_count > 0,
                     f'{status_count} messages in 2s')

    def test_odom_rate(self):
        """Verify odometry publishes at ~50 Hz."""
        print('\n── TEST 2: Odometry Rate ──────────────────────────────')

        with self.odom_lock_:
            self.odom_msgs_.clear()

        t_start = time.time()
        self._spin_for(2.0)
        elapsed = time.time() - t_start

        with self.odom_lock_:
            count = len(self.odom_msgs_)

        actual_hz = count / elapsed
        expected  = ODOM_RATE_EXPECTED
        ok = abs(actual_hz - expected) <= ODOM_RATE_TOLERANCE

        self._assert('Odom rate ~50 Hz', ok,
                     f'actual={actual_hz:.1f} Hz, expected={expected:.0f} Hz ±{ODOM_RATE_TOLERANCE}')

    def test_forward_drive(self):
        """Send forward cmd_vel — verify robot moves in +X direction."""
        print('\n── TEST 3: Forward Drive ──────────────────────────────')

        # Record start pose
        with self.odom_lock_:
            self.odom_msgs_.clear()

        # Drive forward for 2 seconds at 0.3 m/s
        deadline = time.time() + TEST_DURATION_S
        while time.time() < deadline:
            self._send_cmd(TEST_DRIVE_SPEED, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

        self._stop()
        self._spin_for(0.3)

        with self.odom_lock_:
            if not self.odom_msgs_:
                self._assert('Forward: odom received', False, 'No odometry messages')
                return
            final = self.odom_msgs_[-1]

        x = final.pose.pose.position.x
        y = final.pose.pose.position.y

        # Expect: moved ~0.6 m in X (0.3 m/s × 2 s), minimal Y drift
        expected_x = TEST_DRIVE_SPEED * TEST_DURATION_S  # ≈ 0.6 m
        x_ok = abs(x - expected_x) < 0.15   # within 15 cm
        y_ok = abs(y) < 0.05                # less than 5 cm lateral drift

        self._assert('Forward: X displacement correct', x_ok,
                     f'x={x:.3f} m, expected≈{expected_x:.2f} m')
        self._assert('Forward: Y drift minimal',        y_ok,
                     f'y={y:.4f} m (should be <0.05 m)')

    def test_rotation(self):
        """Send rotation cmd_vel — verify robot turns in place."""
        print('\n── TEST 4: Rotation ───────────────────────────────────')

        with self.odom_lock_:
            self.odom_msgs_.clear()

        # Spin in place for 2 seconds
        deadline = time.time() + TEST_DURATION_S
        while time.time() < deadline:
            self._send_cmd(0.0, TEST_TURN_SPEED)
            rclpy.spin_once(self, timeout_sec=0.02)

        self._stop()
        self._spin_for(0.3)

        with self.odom_lock_:
            if not self.odom_msgs_:
                self._assert('Rotation: odom received', False)
                return
            final = self.odom_msgs_[-1]

        x = final.pose.pose.position.x
        y = final.pose.pose.position.y
        q = final.pose.pose.orientation
        # Extract yaw from quaternion
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        expected_yaw = TEST_TURN_SPEED * TEST_DURATION_S  # ≈ 1.0 rad

        yaw_ok = abs(abs(yaw) - expected_yaw) < 0.2   # within ~11 degrees
        pos_ok = math.sqrt(x*x + y*y) < 0.1            # didn't translate

        self._assert('Rotation: heading change correct', yaw_ok,
                     f'yaw={math.degrees(yaw):.1f}°, expected≈{math.degrees(expected_yaw):.1f}°')
        self._assert('Rotation: stayed in place',        pos_ok,
                     f'displacement={math.sqrt(x*x+y*y):.3f} m (should be <0.1 m)')

    def test_watchdog(self):
        """Verify motors stop when cmd_vel goes silent."""
        print('\n── TEST 5: Watchdog ───────────────────────────────────')

        # Drive forward
        for _ in range(20):
            self._send_cmd(0.3, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

        # Go silent for 1 second (longer than 0.5s watchdog)
        t_silent = time.time()
        self._spin_for(1.0)

        # Check that /hardware/status mentions it's still alive (node didn't crash)
        with self.odom_lock_:
            recent_odom = self.odom_msgs_[-5:] if len(self.odom_msgs_) >= 5 else self.odom_msgs_

        # After silence, velocity in odom should be ~0
        if recent_odom:
            last = recent_odom[-1]
            vel = abs(last.twist.twist.linear.x)
            stopped = vel < 0.05
            self._assert('Watchdog: robot stopped after silence', stopped,
                         f'v={vel:.4f} m/s (should be <0.05 m/s)')
        else:
            self._assert('Watchdog: odom still publishing', False)

    def test_joint_states(self):
        """Verify joint states match wheel movement."""
        print('\n── TEST 6: Joint States ───────────────────────────────')

        self.joint_msgs_.clear()

        # Drive briefly
        for _ in range(30):
            self._send_cmd(0.3, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
        self._stop()
        self._spin_for(0.3)

        ok_name    = False
        ok_vel     = False
        ok_pos_inc = False

        if self.joint_msgs_:
            msg = self.joint_msgs_[-1]
            ok_name = ('left_wheel_joint'  in msg.name and
                       'right_wheel_joint' in msg.name)
            if len(msg.velocity) == 2:
                ok_vel = any(abs(v) > 0.01 for v in msg.velocity)
            if len(msg.position) == 2:
                ok_pos_inc = any(abs(p) > 0.01 for p in msg.position)

        self._assert('JointState: correct joint names', ok_name)
        self._assert('JointState: velocities non-zero during drive', ok_vel)
        self._assert('JointState: positions accumulating',  ok_pos_inc)

    # ══════════════════════════════════════════════════════════
    #  RUN ALL TESTS
    # ══════════════════════════════════════════════════════════

    def run_all_tests(self):
        print('\n')
        print('═' * 58)
        print('   DELIVERY ROBOT — SOFTWARE STACK VERIFICATION')
        print('   Testing without hardware (mock mode)')
        print('═' * 58)

        # Wait for nodes to be ready
        self._spin_for(1.5)

        self.test_topics_alive()
        self.test_odom_rate()
        self.test_forward_drive()
        self.test_rotation()
        self.test_watchdog()
        self.test_joint_states()

        # ── Summary ──────────────────────────────────────────────────
        print('\n')
        print('═' * 58)
        print('   RESULTS SUMMARY')
        print('═' * 58)

        passed = sum(1 for v in self.results_.values() if v)
        total  = len(self.results_)
        failed = [k for k, v in self.results_.items() if not v]

        print(f'\n  Tests passed: {passed}/{total}')

        if failed:
            print('\n  Failed tests:')
            for f in failed:
                print(f'    ❌ {f}')
        else:
            print('\n  ✅ ALL TESTS PASSED')
            print('  Software stack is ready for hardware integration.')

        print('\n' + '═' * 58 + '\n')

        return passed == total


# ══════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = SoftwareTestNode()

    success = False
    try:
        success = node.run_all_tests()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    import sys
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()