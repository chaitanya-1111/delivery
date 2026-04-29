#!/usr/bin/env python3
"""
lidar_tf_node.py

PRODUCTION ROLE:
  Publishes the static transform: base_link → laser.

  This tells the entire ROS system EXACTLY where the lidar is mounted
  on the robot body. It is the single most important calibration value
  in the whole navigation pipeline.

  WHY A DEDICATED NODE (vs using URDF + robot_state_publisher):
    - Use this node if you haven't integrated lidar.urdf.xacro yet
    - Or if you need runtime-adjustable TF without recompiling URDF
    - Or during initial setup/calibration before URDF is finalized

  If you ARE using robot_state_publisher with the URDF, disable this
  node (set publish_tf: false) to avoid duplicate TF broadcasts.

PARAMETERS (set in launch file or ros2 param set):
  x, y, z         : lidar position relative to base_link center (meters)
  roll, pitch, yaw : lidar orientation (radians)
  publish_tf       : bool — disable if using URDF/robot_state_publisher

PHYSICAL MEASUREMENT GUIDE:
  base_link is typically at the midpoint between the drive wheels,
  projected to the floor. Measure your lidar mount from there.

  Example for a typical tabletop delivery robot:
    x = 0.0   (centered left-right)
    y = 0.0   (centered front-back — adjust if lidar is offset)
    z = 0.18  (lidar is 18cm above floor = mounted on top of robot)
    roll, pitch, yaw = 0, 0, 0 (lidar faces same direction as robot)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math


class LidarTFNode(Node):

    def __init__(self):
        super().__init__('lidar_tf_node')

        # ── Parameters ────────────────────────────────────────────
        # PRODUCTION: Set these to your actual measured values!
        # These defaults assume lidar centered on robot, 18cm high.
        #
        # QUICK CALIBRATION CHECKLIST:
        #   1) Measure lidar center relative to base_link origin.
        #   2) Set x/y/z in meters (x forward, y left, z up).
        #   3) Set yaw in radians (0 forward, +pi/2 left, +pi backward).
        #   4) Verify with: ros2 run tf2_ros tf2_echo base_link laser
        #   5) If scan appears mirrored/rotated in RViz, revisit yaw first.
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame',  'laser')
        self.declare_parameter('x',     0.0)    # meters forward from base_link
        self.declare_parameter('y',     0.0)    # meters left from base_link
        self.declare_parameter('z',     0.18)   # meters up from base_link
        self.declare_parameter('roll',  0.0)    # radians
        self.declare_parameter('pitch', 0.0)    # radians
        self.declare_parameter('yaw',   0.0)    # radians (0=same direction as robot)
        self.declare_parameter('publish_tf', True)  # set false if using URDF

        self._parent = self.get_parameter('parent_frame').value
        self._child  = self.get_parameter('child_frame').value
        self._x      = self.get_parameter('x').value
        self._y      = self.get_parameter('y').value
        self._z      = self.get_parameter('z').value
        self._roll   = self.get_parameter('roll').value
        self._pitch  = self.get_parameter('pitch').value
        self._yaw    = self.get_parameter('yaw').value
        self._pub_tf = self.get_parameter('publish_tf').value

        if not self._pub_tf:
            self.get_logger().info(
                'publish_tf=false: TF will NOT be broadcast. '
                'Assuming robot_state_publisher handles it via URDF.'
            )
            return

        # ── Static TF Broadcaster ─────────────────────────────────
        # StaticTransformBroadcaster sends the TF once (or whenever called)
        # and it persists in the TF tree until the node dies.
        # This is correct for a fixed sensor that doesn't move.
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publish immediately on startup
        self._publish_static_tf()

        self.get_logger().info(
            f'Static TF published: {self._parent} → {self._child}\n'
            f'  Position: x={self._x:.3f}m, y={self._y:.3f}m, z={self._z:.3f}m\n'
            f'  Rotation: roll={math.degrees(self._roll):.1f}°, '
            f'pitch={math.degrees(self._pitch):.1f}°, '
            f'yaw={math.degrees(self._yaw):.1f}°\n'
            f'  IMPORTANT: Verify these match your physical robot measurements!\n'
            f'  Quick verify: ros2 run tf2_ros tf2_echo base_link laser'
        )

    def _publish_static_tf(self):
        """
        Broadcast the base_link → laser static transform.

        Uses a quaternion for rotation (required by ROS TF).
        We convert from roll/pitch/yaw Euler angles.
        """
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self._parent   # parent: base_link
        t.child_frame_id  = self._child    # child: laser

        # Translation (position)
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = self._z

        # Rotation: convert Euler angles to quaternion
        # Using the standard aerospace/robotics convention: roll→pitch→yaw
        q = self._euler_to_quaternion(self._roll, self._pitch, self._yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._tf_broadcaster.sendTransform(t)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
        """
        Convert roll/pitch/yaw Euler angles to quaternion (x, y, z, w).

        Uses ZYX convention (standard robotics: yaw applied first,
        then pitch, then roll).

        Args:
            roll:  rotation around X axis (radians)
            pitch: rotation around Y axis (radians)
            yaw:   rotation around Z axis (radians)

        Returns:
            (x, y, z, w) quaternion tuple
        """
        cy = math.cos(yaw   * 0.5)
        sy = math.sin(yaw   * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll  * 0.5)
        sr = math.sin(roll  * 0.5)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy

        return (x, y, z, w)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
