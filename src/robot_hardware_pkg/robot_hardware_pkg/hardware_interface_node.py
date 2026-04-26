#!/usr/bin/env python3
"""
hardware_interface_node.py
==========================
Robot Hardware Interface — Production Node
Delivery Robot v1.0

WHAT THIS NODE DOES:
  Bridge between ROS2 Nav2 stack and physical robot hardware.

  Direction 1 (Command):
    /cmd_vel Twist → differential drive kinematics → PWM values → Arduino → Motors

  Direction 2 (Feedback):
    Arduino encoders → serial ticks → dead reckoning math → /odom + /tf + /joint_states

MOCK MODE:
  Run with --ros-args -p mock_mode:=true
  Simulates Arduino responses so you can test ALL software WITHOUT hardware.
  The robot will drive in a virtual space. Odometry, TF, Nav2 all work normally.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import serial
import threading
import math
import time

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster

# ══════════════════════════════════════════════════════════════
#  ROBOT PHYSICAL CONSTANTS  (match your hardware exactly)
# ══════════════════════════════════════════════════════════════

WHEEL_DIAMETER      = 0.20
WHEEL_RADIUS        = WHEEL_DIAMETER / 2.0
WHEEL_SEPARATION    = 0.60
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER   # 0.6283 m

ENCODER_PPR         = 13
QUADRATURE_MULT     = 4
GEAR_RATIO          = 71
ENCODER_CPR         = ENCODER_PPR * QUADRATURE_MULT * GEAR_RATIO  # 3692

METERS_PER_TICK     = WHEEL_CIRCUMFERENCE / ENCODER_CPR  # 0.0001702 m/tick
TICKS_PER_METER     = ENCODER_CPR / WHEEL_CIRCUMFERENCE  # 5878 ticks/m

MAX_MOTOR_RPM       = 60.0
MAX_LINEAR_VEL      = (MAX_MOTOR_RPM / 60.0) * WHEEL_CIRCUMFERENCE  # 0.628 m/s
MAX_PWM             = 255
PWM_DEADBAND        = 40

# ══════════════════════════════════════════════════════════════
#  SERIAL CONFIG
# ══════════════════════════════════════════════════════════════

SERIAL_PORT         = '/dev/ttyUSB0'
SERIAL_BAUD         = 115200
SERIAL_TIMEOUT      = 1.0

# ══════════════════════════════════════════════════════════════
#  TIMING
# ══════════════════════════════════════════════════════════════

ODOM_RATE_HZ        = 50.0
CMD_VEL_TIMEOUT_S   = 0.5
DIAGNOSTICS_RATE_HZ = 1.0


# ══════════════════════════════════════════════════════════════
#  MOCK ARDUINO SIMULATOR
#  Runs inside the node when mock_mode=True.
#  Simulates realistic encoder ticks based on the last cmd_vel.
# ══════════════════════════════════════════════════════════════

class MockArduino:
    """
    Simulates what the Arduino firmware would do.

    When the node sends "M 120 118\n", the real Arduino drives motors
    and accumulates encoder ticks. This mock class does the same math
    in software, so the rest of the node behaves identically.

    Physics model:
      PWM → wheel speed (linear mapping, no motor dynamics)
      wheel speed × time → distance
      distance / meters_per_tick → ticks
    """

    def __init__(self, logger):
        self.logger        = logger
        self.left_ticks    = 0
        self.right_ticks   = 0
        self.left_pwm      = 0
        self.right_pwm     = 0
        self.last_update   = time.time()
        self.lock          = threading.Lock()
        self.logger.info('[MOCK] MockArduino simulator active — no hardware needed')

    def set_motors(self, left_pwm: int, right_pwm: int):
        self._step()   # integrate motion up to now before changing speed
        with self.lock:
            self.left_pwm  = left_pwm
            self.right_pwm = right_pwm

    def get_ticks(self):
        self._step()
        with self.lock:
            return self.left_ticks, self.right_ticks

    def _step(self):
        """Integrate motor commands → encoder ticks over elapsed time."""
        now = time.time()
        dt  = now - self.last_update
        self.last_update = now

        if dt <= 0:
            return

        with self.lock:
            # Convert PWM → wheel speed (m/s)
            left_vel  = (self.left_pwm  / MAX_PWM) * MAX_LINEAR_VEL
            right_vel = (self.right_pwm / MAX_PWM) * MAX_LINEAR_VEL

            # Distance traveled this step
            d_left  = left_vel  * dt
            d_right = right_vel * dt

            # Add small Gaussian noise to simulate real encoder behavior
            import random
            noise_scale = 0.0001  # ≈ 0.1 mm — very small, realistic
            d_left  += random.gauss(0, noise_scale)
            d_right += random.gauss(0, noise_scale)

            # Convert distance → ticks
            self.left_ticks  += int(d_left  / METERS_PER_TICK)
            self.right_ticks += int(d_right / METERS_PER_TICK)


# ══════════════════════════════════════════════════════════════
#  HARDWARE INTERFACE NODE
# ══════════════════════════════════════════════════════════════

class HardwareInterfaceNode(Node):

    def __init__(self):
        super().__init__('hardware_interface_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('mock_mode',      False)
        self.declare_parameter('serial_port',    SERIAL_PORT)
        self.declare_parameter('serial_baud',    SERIAL_BAUD)
        self.declare_parameter('pwm_deadband',   PWM_DEADBAND)
        self.declare_parameter('base_frame_id',  'base_footprint')
        self.declare_parameter('odom_frame_id',  'odom')
        self.declare_parameter('publish_tf',     True)

        self.mock_mode_     = self.get_parameter('mock_mode').value
        self.serial_port_   = self.get_parameter('serial_port').value
        self.serial_baud_   = self.get_parameter('serial_baud').value
        self.pwm_deadband_  = self.get_parameter('pwm_deadband').value
        self.base_frame_id_ = self.get_parameter('base_frame_id').value
        self.odom_frame_id_ = self.get_parameter('odom_frame_id').value
        self.publish_tf_    = self.get_parameter('publish_tf').value

        # ── Pose state ────────────────────────────────────────────────
        self.x_              = 0.0
        self.y_              = 0.0
        self.theta_          = 0.0
        self.left_wheel_ang_ = 0.0
        self.right_wheel_ang_= 0.0

        # ── Encoder state ─────────────────────────────────────────────
        self.prev_left_ticks_  = 0
        self.prev_right_ticks_ = 0
        self.cur_left_ticks_   = 0
        self.cur_right_ticks_  = 0
        self.ticks_lock_       = threading.Lock()

        # ── Command state ─────────────────────────────────────────────
        self.last_cmd_time_ = time.time()
        self.cmd_lock_      = threading.Lock()

        # ── Diagnostics counters ──────────────────────────────────────
        self.serial_errors_     = 0
        self.odom_publishes_    = 0
        self.cmd_vel_received_  = 0
        self.node_start_time_   = time.time()

        # ── Hardware backend ──────────────────────────────────────────
        if self.mock_mode_:
            self.get_logger().warn('=' * 55)
            self.get_logger().warn('  MOCK MODE ACTIVE — No hardware required')
            self.get_logger().warn('  All software stack will run normally.')
            self.get_logger().warn('  Encoder ticks are simulated from cmd_vel.')
            self.get_logger().warn('=' * 55)
            self.mock_ = MockArduino(self.get_logger())
            self.serial_ = None
        else:
            self.mock_ = None
            self.serial_ = self._connect_serial()
            # Start serial reader thread only in real mode
            threading.Thread(
                target=self._serial_reader_thread, daemon=True).start()

        # ── ROS Publishers ────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.odom_pub_       = self.create_publisher(Odometry,      '/odom',               qos)
        self.joint_pub_      = self.create_publisher(JointState,    '/joint_states',        qos)
        self.status_pub_     = self.create_publisher(String,        '/hardware/status',     10)
        self.diag_pub_       = self.create_publisher(DiagnosticArray, '/hardware/diagnostics', 10)

        self.tf_broadcaster_ = TransformBroadcaster(self)

        # ── ROS Subscribers ───────────────────────────────────────────
        self.cmd_vel_sub_ = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, qos)

        # ── Timers ────────────────────────────────────────────────────
        self.odom_timer_  = self.create_timer(1.0 / ODOM_RATE_HZ,        self._odom_callback)
        self.watch_timer_ = self.create_timer(0.1,                         self._watchdog_callback)
        self.diag_timer_  = self.create_timer(1.0 / DIAGNOSTICS_RATE_HZ,  self._diagnostics_callback)

        self.get_logger().info('Hardware Interface Node READY')
        self._publish_status('READY')

    # ══════════════════════════════════════════════════════════
    #  SERIAL CONNECTION
    # ══════════════════════════════════════════════════════════

    def _connect_serial(self):
        while True:
            try:
                ser = serial.Serial(self.serial_port_, self.serial_baud_, timeout=SERIAL_TIMEOUT)
                self.get_logger().info(f'Waiting for Arduino on {self.serial_port_}...')
                deadline = time.time() + 5.0
                while time.time() < deadline:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line == 'READY':
                        self.get_logger().info('Arduino connected and ready.')
                        return ser
                self.get_logger().warn('No READY signal — retrying...')
                ser.close()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e} — retry in 2s')
                self.serial_errors_ += 1
                time.sleep(2.0)

    # ══════════════════════════════════════════════════════════
    #  SERIAL READER THREAD  (real hardware only)
    # ══════════════════════════════════════════════════════════

    def _serial_reader_thread(self):
        """
        Background thread — reads "E left right" lines from Arduino.
        Runs independently so serial blocking never stalls ROS callbacks.
        """
        while rclpy.ok():
            try:
                raw = self.serial_.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if line.startswith('E '):
                    parts = line.split()
                    if len(parts) == 3:
                        with self.ticks_lock_:
                            self.cur_left_ticks_  = int(parts[1])
                            self.cur_right_ticks_ = int(parts[2])
            except (serial.SerialException, ValueError):
                self.serial_errors_ += 1
                time.sleep(0.05)

    # ══════════════════════════════════════════════════════════
    #  CMD_VEL CALLBACK
    # ══════════════════════════════════════════════════════════

    def _cmd_vel_callback(self, msg: Twist):
        """
        Converts Nav2 Twist command → individual wheel PWM → Arduino.

        Differential drive kinematics:
          v_left  = v - (ω × wheel_separation / 2)
          v_right = v + (ω × wheel_separation / 2)
        """
        v     = msg.linear.x
        omega = msg.angular.z

        # Kinematics
        v_left  = v - (omega * WHEEL_SEPARATION / 2.0)
        v_right = v + (omega * WHEEL_SEPARATION / 2.0)

        # Clamp to max speed
        v_left  = max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, v_left))
        v_right = max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, v_right))

        # Scale to PWM
        left_pwm  = int((v_left  / MAX_LINEAR_VEL) * MAX_PWM)
        right_pwm = int((v_right / MAX_LINEAR_VEL) * MAX_PWM)

        # Deadband
        left_pwm  = self._apply_deadband(left_pwm)
        right_pwm = self._apply_deadband(right_pwm)

        # Send to hardware or mock
        self._drive(left_pwm, right_pwm)

        with self.cmd_lock_:
            self.last_cmd_time_   = time.time()
            self.cmd_vel_received_ += 1

    def _apply_deadband(self, pwm: int) -> int:
        return 0 if abs(pwm) < self.pwm_deadband_ else pwm

    def _drive(self, left_pwm: int, right_pwm: int):
        """Send motor command to real Arduino or mock simulator."""
        if self.mock_mode_:
            self.mock_.set_motors(left_pwm, right_pwm)
        else:
            try:
                self.serial_.write(f'M {left_pwm} {right_pwm}\n'.encode())
            except serial.SerialException as e:
                self.get_logger().warn(f'Motor write failed: {e}')
                self.serial_errors_ += 1

    # ══════════════════════════════════════════════════════════
    #  WATCHDOG
    # ══════════════════════════════════════════════════════════

    def _watchdog_callback(self):
        """Stop motors if /cmd_vel goes silent for CMD_VEL_TIMEOUT_S seconds."""
        with self.cmd_lock_:
            silent_for = time.time() - self.last_cmd_time_
        if silent_for > CMD_VEL_TIMEOUT_S:
            self._drive(0, 0)

    # ══════════════════════════════════════════════════════════
    #  ODOMETRY TIMER  (50 Hz)
    # ══════════════════════════════════════════════════════════

    def _odom_callback(self):
        """
        Dead reckoning odometry from encoder ticks.

        Every 20 ms:
          1. Get tick deltas since last cycle
          2. Convert ticks → wheel distances (meters)
          3. Compute robot center displacement + heading change
          4. Update (x, y, theta) pose
          5. Publish /odom, /tf, /joint_states
        """
        now = self.get_clock().now()
        dt  = 1.0 / ODOM_RATE_HZ

        # Read ticks
        if self.mock_mode_:
            left_ticks, right_ticks = self.mock_.get_ticks()
        else:
            with self.ticks_lock_:
                left_ticks  = self.cur_left_ticks_
                right_ticks = self.cur_right_ticks_

        # Deltas
        dl = left_ticks  - self.prev_left_ticks_
        dr = right_ticks - self.prev_right_ticks_
        self.prev_left_ticks_  = left_ticks
        self.prev_right_ticks_ = right_ticks

        # Ticks → meters
        d_left  = dl * METERS_PER_TICK
        d_right = dr * METERS_PER_TICK

        # Robot motion
        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / WHEEL_SEPARATION

        # Pose update
        self.x_     += d_center * math.cos(self.theta_)
        self.y_     += d_center * math.sin(self.theta_)
        self.theta_ += d_theta
        self.theta_  = math.atan2(math.sin(self.theta_), math.cos(self.theta_))

        # Velocities
        v_lin = d_center / dt
        v_ang = d_theta  / dt

        # Publish
        q = self._euler_to_quat(0.0, 0.0, self.theta_)
        self._pub_odom(now, q, v_lin, v_ang)
        if self.publish_tf_:
            self._pub_tf(now, q)
        self._pub_joints(now, d_left, d_right, dt)

        self.odom_publishes_ += 1

    # ══════════════════════════════════════════════════════════
    #  PUBLISH HELPERS
    # ══════════════════════════════════════════════════════════

    def _pub_odom(self, stamp, q, v_lin, v_ang):
        msg = Odometry()
        msg.header.stamp    = stamp.to_msg()
        msg.header.frame_id = self.odom_frame_id_
        msg.child_frame_id  = self.base_frame_id_

        msg.pose.pose.position.x    = self.x_
        msg.pose.pose.position.y    = self.y_
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Covariance: [x, y, z, roll, pitch, yaw] diagonal
        msg.pose.covariance[0]  = 0.001   # x
        msg.pose.covariance[7]  = 0.001   # y
        msg.pose.covariance[14] = 1e6     # z  (unused — planar)
        msg.pose.covariance[21] = 1e6     # roll (unused)
        msg.pose.covariance[28] = 1e6     # pitch (unused)
        msg.pose.covariance[35] = 0.01    # yaw

        msg.twist.twist.linear.x  = v_lin
        msg.twist.twist.angular.z = v_ang
        msg.twist.covariance[0]   = 0.001
        msg.twist.covariance[7]   = 0.001
        msg.twist.covariance[35]  = 0.01

        self.odom_pub_.publish(msg)

    def _pub_tf(self, stamp, q):
        t = TransformStamped()
        t.header.stamp    = stamp.to_msg()
        t.header.frame_id = self.odom_frame_id_
        t.child_frame_id  = self.base_frame_id_

        t.transform.translation.x = self.x_
        t.transform.translation.y = self.y_
        t.transform.translation.z = 0.0
        t.transform.rotation.x    = q[0]
        t.transform.rotation.y    = q[1]
        t.transform.rotation.z    = q[2]
        t.transform.rotation.w    = q[3]

        self.tf_broadcaster_.sendTransform(t)

    def _pub_joints(self, stamp, d_left, d_right, dt):
        self.left_wheel_ang_  += d_left  / WHEEL_RADIUS
        self.right_wheel_ang_ += d_right / WHEEL_RADIUS

        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.name         = ['left_wheel_joint', 'right_wheel_joint']
        js.position     = [self.left_wheel_ang_,  self.right_wheel_ang_]
        js.velocity     = [
            (d_left  / WHEEL_RADIUS) / dt,
            (d_right / WHEEL_RADIUS) / dt
        ]
        self.joint_pub_.publish(js)

    # ══════════════════════════════════════════════════════════
    #  DIAGNOSTICS  (1 Hz)
    # ══════════════════════════════════════════════════════════

    def _diagnostics_callback(self):
        """
        Publishes a machine-readable health report to /hardware/diagnostics.
        Also publishes a human-readable string to /hardware/status.

        Useful for monitoring dashboards and production health checks.
        """
        uptime = time.time() - self.node_start_time_

        # Build diagnostic message
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        hw_status = DiagnosticStatus()
        hw_status.name    = 'hardware_interface'
        hw_status.hardware_id = 'delivery_bot_v1'
        hw_status.level   = DiagnosticStatus.OK
        hw_status.message = 'MOCK MODE' if self.mock_mode_ else 'HARDWARE CONNECTED'

        hw_status.values = [
            KeyValue(key='mode',             value='mock' if self.mock_mode_ else 'hardware'),
            KeyValue(key='uptime_s',         value=f'{uptime:.1f}'),
            KeyValue(key='odom_publishes',   value=str(self.odom_publishes_)),
            KeyValue(key='cmd_vel_received', value=str(self.cmd_vel_received_)),
            KeyValue(key='serial_errors',    value=str(self.serial_errors_)),
            KeyValue(key='robot_x',          value=f'{self.x_:.4f}'),
            KeyValue(key='robot_y',          value=f'{self.y_:.4f}'),
            KeyValue(key='robot_theta_deg',  value=f'{math.degrees(self.theta_):.2f}'),
        ]

        if self.serial_errors_ > 10:
            hw_status.level   = DiagnosticStatus.WARN
            hw_status.message = f'High serial error count: {self.serial_errors_}'

        diag_array.status.append(hw_status)
        self.diag_pub_.publish(diag_array)

        # Human-readable status
        mode_tag = '[MOCK]' if self.mock_mode_ else '[REAL]'
        status_str = (
            f'{mode_tag} uptime={uptime:.0f}s | '
            f'pos=({self.x_:.2f}, {self.y_:.2f}) | '
            f'heading={math.degrees(self.theta_):.1f}° | '
            f'odom_hz={self.odom_publishes_/uptime:.1f} | '
            f'serial_errors={self.serial_errors_}'
        )
        self.status_pub_.publish(String(data=status_str))

    # ══════════════════════════════════════════════════════════
    #  HELPERS
    # ══════════════════════════════════════════════════════════

    def _euler_to_quat(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion (x, y, z, w)."""
        cy = math.cos(yaw   * 0.5)
        sy = math.sin(yaw   * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll  * 0.5)
        sr = math.sin(roll  * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _publish_status(self, msg: str):
        self.status_pub_.publish(String(data=msg))

    def destroy_node(self):
        self.get_logger().info('Shutting down — stopping motors')
        self._drive(0, 0)
        if self.serial_ and self.serial_.is_open:
            self.serial_.close()
        super().destroy_node()


# ══════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()