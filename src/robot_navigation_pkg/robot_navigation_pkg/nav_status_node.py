#!/usr/bin/env python3
"""
nav_status_node.py

ROLE IN THE SYSTEM:
  Monitors overall navigation health and publishes the robot's
  current position for other nodes (mission_manager, safety_supervisor).

  Think of it as the "dashboard" of the navigation system.

TOPICS SUBSCRIBED:
  /amcl_pose   (geometry_msgs/PoseWithCovarianceStamped)
    → Robot's estimated pose from AMCL (with uncertainty)

  /navigation/status  (std_msgs/String)
    → Status from nav_client_node

  /navigation/feedback (std_msgs/String)
    → Distance remaining etc.

TOPICS PUBLISHED:
  /robot/pose           (geometry_msgs/PoseStamped)
    → Clean current pose (no covariance) for other nodes to use

  /robot/pose_confidence (std_msgs/Float32)
    → 0.0 to 1.0: how confident AMCL is in the localization
    → Low confidence = robot is lost!

  /navigation/health    (std_msgs/String)
    → "HEALTHY" | "LOCALIZING" | "LOST" | "NAV_UNAVAILABLE"
"""

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# For checking if Nav2 topics are alive
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class NavStatusNode(Node):

    def __init__(self):
        super().__init__('nav_status_node')

        # QoS profile for sensor/pose data (best-effort, like lidar/cameras)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscriptions ────────────────────────────────────────

        # AMCL pose — this is what AMCL outputs after localization
        # PoseWithCovarianceStamped has covariance matrix (6x6)
        # Covariance tells us how UNCERTAIN the pose estimate is
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._on_amcl_pose,
            qos_profile=sensor_qos
        )

        self._nav_status_sub = self.create_subscription(
            String,
            '/navigation/status',
            self._on_nav_status,
            10
        )

        # ── Publishers ───────────────────────────────────────────
        self._pose_pub = self.create_publisher(
            PoseStamped,
            '/robot/pose',
            10
        )

        self._confidence_pub = self.create_publisher(
            Float32,
            '/robot/pose_confidence',
            10
        )

        self._health_pub = self.create_publisher(
            String,
            '/navigation/health',
            10
        )

        # ── State ────────────────────────────────────────────────
        self._last_pose: PoseStamped = None
        self._pose_confidence: float = 0.0
        self._nav_status: str = 'IDLE'
        self._last_amcl_time = None

        # Confidence thresholds
        # Covariance trace: sum of position variances (x² + y²)
        # < 0.1  = very confident
        # > 1.0  = very uncertain (robot is probably lost)
        self._CONF_HIGH   = 0.1
        self._CONF_LOW    = 1.0

        # Health check timer: 1Hz
        self._health_timer = self.create_timer(1.0, self._publish_health)

        self.get_logger().info('NavStatusNode started.')

    # ── AMCL Pose Handler ────────────────────────────────────────
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """
        AMCL publishes its best estimate of robot pose.
        The covariance matrix tells us how confident it is.

        covariance is a flat 36-element array (6x6 matrix):
          [0]  = var(x)       — uncertainty in x position
          [7]  = var(y)       — uncertainty in y position
          [35] = var(yaw)     — uncertainty in rotation

        Smaller variance = more confident = better localization.
        """
        # Extract clean pose (without covariance)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._last_pose = pose
        self._last_amcl_time = self.get_clock().now()

        # Compute confidence from covariance
        cov = msg.pose.covariance
        var_x   = cov[0]   # variance in x
        var_y   = cov[7]   # variance in y
        var_yaw = cov[35]  # variance in yaw

        # Trace = sum of main diagonal variances
        trace = var_x + var_y + var_yaw

        # Convert trace to 0-1 confidence (inverse relationship)
        # trace near 0 → confidence near 1.0 (very sure)
        # trace near 1.0+ → confidence near 0 (very unsure)
        if trace < self._CONF_HIGH:
            self._pose_confidence = 1.0
        elif trace > self._CONF_LOW:
            self._pose_confidence = 0.0
        else:
            self._pose_confidence = 1.0 - (trace - self._CONF_HIGH) / (self._CONF_LOW - self._CONF_HIGH)

        # Extract yaw for logging
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.get_logger().debug(
            f'AMCL pose: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) '
            f'yaw={math.degrees(yaw):.1f}° '
            f'confidence={self._pose_confidence:.2f} (trace={trace:.4f})'
        )

        # Publish clean pose
        self._pose_pub.publish(pose)

        # Publish confidence
        conf_msg = Float32()
        conf_msg.data = self._pose_confidence
        self._confidence_pub.publish(conf_msg)

    # ── Nav Status Handler ───────────────────────────────────────
    def _on_nav_status(self, msg: String):
        self._nav_status = msg.data

    # ── Health Publisher (1Hz) ───────────────────────────────────
    def _publish_health(self):
        """
        Determine and publish overall navigation health.

        Health states:
          HEALTHY         → Robot is localized and Nav2 is running fine
          LOCALIZING      → AMCL is still converging (first few seconds)
          LOST            → AMCL confidence is very low (robot is lost)
          NAV_UNAVAILABLE → No AMCL pose received at all (Nav2 not running?)
        """
        health = 'NAV_UNAVAILABLE'

        if self._last_amcl_time is not None:
            # Check how stale the AMCL pose is
            now = self.get_clock().now()
            age = (now - self._last_amcl_time).nanoseconds / 1e9

            if age > 3.0:
                # No AMCL update for 3+ seconds — something is wrong
                health = 'NAV_UNAVAILABLE'
                self.get_logger().warn('AMCL pose is stale! Is the lidar working?')
            elif self._pose_confidence >= 0.7:
                health = 'HEALTHY'
            elif self._pose_confidence >= 0.3:
                health = 'LOCALIZING'
                self.get_logger().info(
                    f'AMCL still localizing... confidence={self._pose_confidence:.2f}. '
                    'Drive the robot around a bit to help localization converge.'
                )
            else:
                health = 'LOST'
                self.get_logger().warn(
                    f'Robot appears LOST! AMCL confidence={self._pose_confidence:.2f}. '
                    'Use RViz to set initial pose estimate, or drive to a known location.'
                )

        health_msg = String()
        health_msg.data = health
        self._health_pub.publish(health_msg)

        if health == 'HEALTHY':
            if self._last_pose:
                x = self._last_pose.pose.position.x
                y = self._last_pose.pose.position.y
                self.get_logger().info(
                    f'Nav HEALTHY | pose=({x:.2f},{y:.2f}) | '
                    f'nav_status={self._nav_status} | '
                    f'confidence={self._pose_confidence:.2f}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = NavStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
