#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class Esp32CamNode(Node):
    def __init__(self):
        super().__init__('esp32_cam_node')
        
        # --- Configuration ---
        # REPLACE THIS with your ESP32's actual IP address!
        # Standard ESP32-CAM stream URL usually looks like this:
        self.stream_url = "http://192.168.1.100:81/stream" 
        
        # Publisher (Same topic as the old camera!)
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        self.get_logger().info(f"Attempting to connect to ESP32 at: {self.stream_url}")
        
        # Open the video stream
        self.cap = cv2.VideoCapture(self.stream_url)
        
        # Buffer size setup to reduce lag
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Timer: Check for frames continuously
        # 0.033s = ~30 FPS (It will go as fast as the ESP32 allows)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().warn("ESP32 Stream not accessible. Reconnecting...")
            self.cap.open(self.stream_url)
            time.sleep(1.0)
            return

        ret, frame = self.cap.read()
        
        if ret:
            # Resize if necessary to speed up processing
            # frame = cv2.resize(frame, (640, 480))
            
            # Convert OpenCV frame (BGR) to ROS Message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Dropped frame from ESP32")

def main(args=None):
    rclpy.init(args=args)
    node = Esp32CamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
