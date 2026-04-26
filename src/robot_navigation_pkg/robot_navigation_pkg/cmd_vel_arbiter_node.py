#!/usr/bin/env python3
"""
cmd_vel_arbiter_node.py

PRODUCTION ROLE:
  In a production robot, multiple sources want to send /cmd_vel:
    1. Nav2 controller_server  → autonomous navigation
    2. Safety supervisor (Step 5) → emergency stop / speed limiting
    3. Manual teleop           → operator override for maintenance

  Without arbitration, these would conflict — last writer wins,
  which is dangerous. The arbiter enforces strict priority:

    PRIORITY 1 (highest): E-STOP  → zero velocity, always wins
    PRIORITY 2:           TELEOP  → operator override (maintenance)
    PRIORITY 3 (default): NAV2   → autonomous navigation

  This node sits between Nav2 and the hardware:
    Nav2:    publishes /cmd_vel_nav
    Teleop:  publishes /cmd_vel_teleop
    E-stop:  publishes /navigation/estop (Bool)

    Arbiter reads all three, applies priority, outputs /cmd_vel
    which robot_hardware_pkg consumes.

  WHY THIS MATTERS:
    During a delivery, if a staff member grabs the joystick to move
    the robot out of the way, it should override autonomy immediately.
    When they let go (teleop silent for >1s), autonomy resumes.

SUBSCRIPTIONS:
  /cmd_vel_nav      (geometry_msgs/Twist) — Nav2 controller output
  /cmd_vel_teleop   (geometry_msgs/Twist) — manual operator input
  /navigation/estop (std_msgs/Bool)       — true = full stop
  /navigation/speed_limit (std_msgs/Float32) — scale [0.0, 1.0]

PUBLICATIONS:
  /cmd_vel           (geometry_msgs/Twist) — to robot_hardware_pkg
  /navigation/source (std_msgs/String)     — which source is active: NAV/TELEOP/ESTOP
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String

ZERO_TWIST = Twist()  # all zeros = full stop


class CmdVelArbiterNode(Node):

    SOURCE_NAV    = 'NAV'
    SOURCE_TELEOP = 'TELEOP'
    SOURCE_ESTOP  = 'ESTOP'

    def __init__(self):
        super().__init__('cmd_vel_arbiter_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('teleop_timeout_sec', 1.0)
        self.declare_parameter('nav_timeout_sec',    0.5)
        self.declare_parameter('publish_rate_hz',   20.0)

        self._teleop_timeout = self.get_parameter('teleop_timeout_sec').value
        self._nav_timeout    = self.get_parameter('nav_timeout_sec').value
        self._pub_rate       = self.get_parameter('publish_rate_hz').value

        # ── QoS ──────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ── Subscriptions ─────────────────────────────────────────
        self._nav_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self._on_nav_cmd, sensor_qos)

        self._teleop_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self._on_teleop_cmd, sensor_qos)

        self._estop_sub = self.create_subscription(
            Bool, '/navigation/estop', self._on_estop, reliable_qos)

        self._speed_limit_sub = self.create_subscription(
            Float32, '/navigation/speed_limit', self._on_speed_limit, reliable_qos)

        # ── Publishers ────────────────────────────────────────────
        self._cmdvel_pub = self.create_publisher(Twist, '/cmd_vel', sensor_qos)
        self._source_pub = self.create_publisher(String, '/navigation/source', 10)

        # ── State ─────────────────────────────────────────────────
        self._nav_cmd: Twist       = ZERO_TWIST
        self._teleop_cmd: Twist    = ZERO_TWIST
        self._estop: bool          = False
        self._speed_limit: float   = 1.0     # 1.0 = full speed, 0.0 = stopped
        self._active_source: str   = self.SOURCE_NAV

        self._last_nav_time: float    = None
        self._last_teleop_time: float = None

        # ── Output timer ──────────────────────────────────────────
        period = 1.0 / self._pub_rate
        self._pub_timer = self.create_timer(period, self._arbitrate_and_publish)

        self.get_logger().info(
            f'CmdVelArbiterNode started at {self._pub_rate:.0f} Hz.\n'
            f'  Priority: ESTOP > TELEOP (timeout={self._teleop_timeout}s) '
            f'> NAV (timeout={self._nav_timeout}s)\n'
            f'  Nav2 output   → /cmd_vel_nav\n'
            f'  Teleop output → /cmd_vel_teleop\n'
            f'  Final output  → /cmd_vel'
        )

    # ── Input Callbacks ───────────────────────────────────────────
    def _on_nav_cmd(self, msg: Twist):
        self._nav_cmd = msg
        self._last_nav_time = time.monotonic()

    def _on_teleop_cmd(self, msg: Twist):
        self._teleop_cmd = msg
        self._last_teleop_time = time.monotonic()

    def _on_estop(self, msg: Bool):
        if msg.data != self._estop:
            self._estop = msg.data
            if self._estop:
                self.get_logger().warn(
                    'E-STOP ENGAGED! All motion stopped. '
                    'Publish /navigation/estop:=false to resume.')
            else:
                self.get_logger().info('E-STOP cleared. Resuming normal operation.')

    def _on_speed_limit(self, msg: Float32):
        new_limit = max(0.0, min(1.0, msg.data))
        if abs(new_limit - self._speed_limit) > 0.05:
            self.get_logger().info(
                f'Speed limit changed: {self._speed_limit:.2f} → {new_limit:.2f}')
        self._speed_limit = new_limit

    # ── Arbitration Logic ─────────────────────────────────────────
    def _arbitrate_and_publish(self):
        """
        Apply priority rules and publish the winning cmd_vel.

        Called at publish_rate_hz (20Hz by default).
        Always publishes something — if no source is active, publishes ZERO.
        """
        now = time.monotonic()
        output = ZERO_TWIST
        source = self.SOURCE_NAV

        # ── PRIORITY 1: E-STOP (always wins) ─────────────────────
        if self._estop:
            output = ZERO_TWIST
            source = self.SOURCE_ESTOP

        # ── PRIORITY 2: TELEOP (operator override) ────────────────
        elif (self._last_teleop_time is not None and
              (now - self._last_teleop_time) < self._teleop_timeout):
            # Teleop is fresh — operator has control
            output = self._teleop_cmd
            source = self.SOURCE_TELEOP

        # ── PRIORITY 3: NAV2 (autonomous navigation) ──────────────
        elif (self._last_nav_time is not None and
              (now - self._last_nav_time) < self._nav_timeout):
            # Nav2 is sending fresh commands
            output = self._nav_cmd
            source = self.SOURCE_NAV

        # ── NO ACTIVE SOURCE → zero velocity ──────────────────────
        else:
            output = ZERO_TWIST
            source = self.SOURCE_NAV   # nav is default mode even when idle

        # ── Apply speed limit ─────────────────────────────────────
        if self._speed_limit < 1.0 and source != self.SOURCE_ESTOP:
            output = self._apply_speed_limit(output, self._speed_limit)

        # ── Publish ───────────────────────────────────────────────
        self._cmdvel_pub.publish(output)

        # ── Publish source (for monitoring) ──────────────────────
        if source != self._active_source:
            self.get_logger().info(
                f'cmd_vel source changed: {self._active_source} → {source}')
            self._active_source = source

        source_msg = String()
        source_msg.data = source
        self._source_pub.publish(source_msg)

    def _apply_speed_limit(self, twist: Twist, limit: float) -> Twist:
        """
        Scale all velocity components by limit factor [0.0, 1.0].
        Used when safety_supervisor (Step 5) requests reduced speed
        e.g., near obstacles or in crowded areas.
        """
        out = Twist()
        out.linear.x  = twist.linear.x  * limit
        out.linear.y  = twist.linear.y  * limit
        out.linear.z  = twist.linear.z  * limit
        out.angular.x = twist.angular.x * limit
        out.angular.y = twist.angular.y * limit
        out.angular.z = twist.angular.z * limit
        return out


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
