#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import FaceTarget
import time

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        
        # Subscribe to the target calculated by the tracker
        self.sub_target = self.create_subscription(FaceTarget, '/face/target', self.target_callback, 10)
        
        # Current Angles (Start at center 90 degrees)
        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        
        # Movement Speed (Gain)
        self.Kp = 1.5
        
        self.get_logger().info("Servo Node Started (Simulation Mode)")
        self.get_logger().info("Hardware connection skipped for WSL/Testing")

    def target_callback(self, msg):
        if not msg.face_found:
            # If face is lost, just hold position (or return to center if you prefer)
            return
            
        # --- Logic ---
        # msg.norm_x is between -1.0 (Left) and +1.0 (Right)
        # If face is to the Left (-), we must subtract from angle to look left
        # If face is to the Right (+), we must add to angle to look right
        
        pan_step = -msg.norm_x * self.Kp  # Inverted logic often works best for servos
        tilt_step = -msg.norm_y * self.Kp
        
        # Update Angles safely (0 to 180 limits)
        self.pan_angle = max(0.0, min(180.0, self.pan_angle + pan_step))
        self.tilt_angle = max(0.0, min(180.0, self.tilt_angle + tilt_step))
        
        # Log the movement (This proves it works!)
        # Only log every few frames to avoid spamming too hard, or just log changes
        if abs(pan_step) > 0.1 or abs(tilt_step) > 0.1:
            self.get_logger().info(f"Target: ({msg.norm_x:.2f}, {msg.norm_y:.2f}) -> Servo: Pan={int(self.pan_angle)}° Tilt={int(self.tilt_angle)}°")

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
