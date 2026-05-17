#!/usr/bin/env python3
"""
Simple keyboard teleop for the delivery robot in Gazebo.

Usage:
  python3 robot_teleop.py

Controls:
  w/a/s/d - Move forward/left/back/right
  q/e     - Rotate left/right
  spacebar - Stop
  x        - Exit
"""

import sys
import termios
import tty
import select
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class RobotTeleop(Node):
    def __init__(self):
        super().__init__('robot_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        
        print("\n" + "="*50)
        print("  🤖 Robot Teleop Controller")
        print("="*50)
        print("\nControls:")
        print("  W - Move forward")
        print("  A - Turn left")
        print("  S - Move backward")
        print("  D - Turn right")
        print("  Q - Strafe left")
        print("  E - Strafe right")
        print("  SPACE - Stop")
        print("  X - Exit")
        print("="*50 + "\n")

    def get_key(self):
        """Get a single key press from stdin."""
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                return key
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return None

    def publish_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Publish velocity command to /cmd_vel."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.publisher.publish(twist)

    def run(self):
        """Main control loop."""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key is None:
                    continue
                
                key = key.lower()
                
                if key == 'w':
                    print("→ Moving forward")
                    self.publish_velocity(linear_x=self.linear_speed)
                elif key == 's':
                    print("→ Moving backward")
                    self.publish_velocity(linear_x=-self.linear_speed)
                elif key == 'a':
                    print("→ Turning left")
                    self.publish_velocity(angular_z=self.angular_speed)
                elif key == 'd':
                    print("→ Turning right")
                    self.publish_velocity(angular_z=-self.angular_speed)
                elif key == 'q':
                    print("→ Strafing left")
                    self.publish_velocity(linear_y=self.linear_speed)
                elif key == 'e':
                    print("→ Strafing right")
                    self.publish_velocity(linear_y=-self.linear_speed)
                elif key == ' ':
                    print("→ Stopping")
                    self.publish_velocity()
                elif key == 'x':
                    print("\n✓ Exiting teleop")
                    self.publish_velocity()  # Stop the robot
                    break
                else:
                    continue
                    
        except KeyboardInterrupt:
            print("\n✓ Interrupted")
        finally:
            self.publish_velocity()  # Ensure robot stops


def main():
    rclpy.init()
    teleop = RobotTeleop()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
