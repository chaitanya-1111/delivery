#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # --- Configuration ---
        self.FRAME_RATE = 30.0
        self.WIDTH = 640
        self.HEIGHT = 480
        
        # --- Publisher ---
        # Best Effort QoS = If a frame is dropped, don't retry (standard for video)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.pub_image = self.create_publisher(Image, '/camera/image_raw', qos_profile)
        
        self.bridge = CvBridge()
        self.cap = None
        self.use_simulation = False
        
        # --- Hardware Check ---
        try:
            # Try opening Camera Index 0 (Default USB)
            self.cap = cv2.VideoCapture(0)
            
            # Check if it actually opened
            if not self.cap.isOpened():
                raise Exception("Could not open video device 0")
                
            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
            
            self.get_logger().info("📷 USB Camera Initialized!")
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ CAMERA ERROR: {e}")
            self.get_logger().warn("⚠️ SWITCHING TO SIMULATION MODE (Sending Test Pattern)")
            self.use_simulation = True

        # --- Timer ---
        self.timer = self.create_timer(1.0/self.FRAME_RATE, self.timer_callback)

    def timer_callback(self):
        if self.use_simulation:
            # Generate a fake image (Black background with moving text)
            frame = np.zeros((self.HEIGHT, self.WIDTH, 3), dtype=np.uint8)
            # Add dynamic text so you know it's alive
            import time
            cv2.putText(frame, f"SIMULATION CAM {time.time():.1f}", (50, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            # Read real camera
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture frame", throttle_duration_sec=5)
                return
        
        # Publish
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.pub_image.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Encoding Error: {e}")

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()