#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
import speech_recognition as sr
import threading
import sys
import select

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        
        # Subscribe to audio data (from microphone node)
        self.sub_audio = self.create_subscription(Int16MultiArray, '/audio/data', self.audio_callback, 10)
        
        # Publish recognized text (to Session Manager)
        self.pub_text = self.create_publisher(String, '/audio/stt_text', 10)
        
        self.get_logger().info("👂 STT Node Started")
        self.get_logger().info("⌨️  KEYBOARD OVERRIDE ACTIVE: Type in this terminal to 'speak' to the robot!")

        # Start the keyboard input thread (for WSL/Testing)
        self.input_thread = threading.Thread(target=self.keyboard_listener)
        self.input_thread.daemon = True
        self.input_thread.start()

    def audio_callback(self, msg):
        # In a real robot, we would buffer these bytes and send them to the API.
        # For now, we just acknowledge receipt to keep the graph happy.
        pass

    def keyboard_listener(self):
        """Allows the developer to type text to simulate speech"""
        while rclpy.ok():
            try:
                # specific check for non-blocking input on Linux
                i, o, e = select.select([sys.stdin], [], [], 1.0)
                if i:
                    text = sys.stdin.readline().strip()
                    if text:
                        msg = String()
                        msg.data = text
                        self.pub_text.publish(msg)
                        self.get_logger().info(f"📝 Manual Input Sent: '{text}'")
            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()