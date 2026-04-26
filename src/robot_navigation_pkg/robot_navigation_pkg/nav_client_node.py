#!/usr/bin/env python3
"""
nav_client_node.py

ROLE IN THE SYSTEM:
  This is the BRIDGE between your mission logic (Step 2) and the Nav2 stack.

  Mission Manager (Step 2) will publish a goal like:
    "Go to position (3.5, 2.1) on the map"

  This node receives that goal and:
    1. Converts it to a NavigateToPose action call
    2. Sends it to Nav2's bt_navigator
    3. Monitors progress (feedback every few seconds)
    4. Reports SUCCEEDED / FAILED / CANCELLED back

TOPICS SUBSCRIBED:
  /navigation/goal  (geometry_msgs/PoseStamped)
    → Receive a new navigation goal from the mission manager

  /navigation/cancel (std_msgs/Bool)
    → Cancel the current navigation goal

TOPICS PUBLISHED:
  /navigation/status  (std_msgs/String)
    → "IDLE" | "NAVIGATING" | "SUCCEEDED" | "FAILED" | "CANCELLED"

  /navigation/feedback (std_msgs/String)
    → JSON: {"distance_remaining": 1.5, "eta_seconds": 12.0}

ACTIONS USED:
  /navigate_to_pose  (nav2_msgs/action/NavigateToPose)
    → Nav2's main navigation action server
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

# Nav2 action message type
from nav2_msgs.action import NavigateToPose


class NavClientNode(Node):

    def __init__(self):
        super().__init__('nav_client_node')

        # ReentrantCallbackGroup allows action callbacks to run
        # concurrently with subscription callbacks
        self._cb_group = ReentrantCallbackGroup()

        # ── Action Client ────────────────────────────────────────
        # Connects to Nav2's bt_navigator action server
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self._cb_group
        )

        # ── Subscriptions ────────────────────────────────────────
        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/navigation/goal',
            self._on_goal_received,
            10,
            callback_group=self._cb_group
        )

        self._cancel_sub = self.create_subscription(
            Bool,
            '/navigation/cancel',
            self._on_cancel_received,
            10,
            callback_group=self._cb_group
        )

        # ── Publishers ───────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/navigation/status', 10)
        self._feedback_pub = self.create_publisher(String, '/navigation/feedback', 10)

        # ── State ────────────────────────────────────────────────
        self._current_goal_handle = None
        self._nav_state = 'IDLE'   # IDLE | NAVIGATING | SUCCEEDED | FAILED | CANCELLED

        # Publish status at 2Hz so other nodes can monitor it
        self._status_timer = self.create_timer(0.5, self._publish_status)

        self.get_logger().info('NavClientNode started. Waiting for navigation goals on /navigation/goal')

    # ── Status Publisher ─────────────────────────────────────────
    def _publish_status(self):
        msg = String()
        msg.data = self._nav_state
        self._status_pub.publish(msg)

    # ── Goal Received ────────────────────────────────────────────
    def _on_goal_received(self, pose: PoseStamped):
        """
        Called when mission_manager (Step 2) sends a new navigation target.

        If already navigating, cancel current goal first, then start new one.
        """
        self.get_logger().info(
            f'New goal received: x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}'
        )

        # Cancel any existing goal
        if self._current_goal_handle is not None:
            self.get_logger().info('Cancelling previous goal...')
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        # Wait for Nav2's action server to be available
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Nav2 /navigate_to_pose action server not available! '
                'Is navigation.launch.py running?'
            )
            self._nav_state = 'FAILED'
            return

        # Build the NavigateToPose goal
        # The frame_id MUST be 'map' (goals are in map coordinates)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Send goal asynchronously
        self.get_logger().info('Sending goal to Nav2...')
        self._nav_state = 'NAVIGATING'

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback
        )

        # When Nav2 accepts/rejects the goal, call _on_goal_response
        send_future.add_done_callback(self._on_goal_response)

    # ── Cancel Received ──────────────────────────────────────────
    def _on_cancel_received(self, msg: Bool):
        """Cancel the currently active navigation goal."""
        if msg.data and self._current_goal_handle is not None:
            self.get_logger().info('Navigation cancel requested.')
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
        else:
            self.get_logger().warn('Cancel received but no active goal.')

    # ── Action Callbacks ─────────────────────────────────────────
    def _on_goal_response(self, future):
        """Called when Nav2 responds to our goal submission."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED by Nav2!')
            self._nav_state = 'FAILED'
            self._current_goal_handle = None
            return

        self.get_logger().info('Goal ACCEPTED by Nav2. Robot is navigating...')
        self._current_goal_handle = goal_handle

        # When navigation completes (success or failure), call _on_nav_result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_feedback(self, feedback_msg):
        """
        Called periodically by Nav2 with navigation progress.

        feedback contains:
          - distance_remaining: meters left to goal
          - navigation_time: how long we've been navigating
          - estimated_time_remaining: ETA
          - number_of_recoveries: how many times robot got stuck
        """
        feedback = feedback_msg.feedback
        dist = feedback.distance_remaining

        # Estimate ETA (rough: assume 0.3 m/s average speed)
        eta = dist / 0.3 if dist > 0 else 0.0

        # Publish feedback as JSON for mission manager to consume
        feedback_data = {
            'distance_remaining': round(dist, 2),
            'eta_seconds': round(eta, 1),
            'recoveries': feedback.number_of_recoveries
        }

        feedback_pub_msg = String()
        feedback_pub_msg.data = json.dumps(feedback_data)
        self._feedback_pub.publish(feedback_pub_msg)

        self.get_logger().info(
            f'Navigation feedback: {dist:.2f}m remaining, ETA ~{eta:.0f}s, '
            f'recoveries={feedback.number_of_recoveries}'
        )

    def _on_nav_result(self, future):
        """Called when navigation completes (success or failure)."""
        result = future.result()

        # Nav2 result codes: 0=unknown, 1=accepted, 4=succeeded, 5=canceled, 6=aborted
        from action_msgs.msg import GoalStatus
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation SUCCEEDED! Robot arrived at goal.')
            self._nav_state = 'SUCCEEDED'

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation CANCELLED.')
            self._nav_state = 'CANCELLED'

        else:
            self.get_logger().error(f'Navigation FAILED! Status code: {status}')
            self._nav_state = 'FAILED'

        self._current_goal_handle = None

    def _on_cancel_done(self, future):
        """Called when goal cancellation is confirmed."""
        self.get_logger().info('Goal cancellation confirmed.')
        self._nav_state = 'CANCELLED'
        self._current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)

    node = NavClientNode()

    # MultiThreadedExecutor needed because we use ReentrantCallbackGroup
    # (action callbacks + subscription callbacks run concurrently)
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
