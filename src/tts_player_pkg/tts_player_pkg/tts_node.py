#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import pyttsx3
import threading
import time

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        
        # Subscribe to "what to say"
        self.sub_text = self.create_subscription(String, '/robot/speech', self.speak_callback, 10)
        
        # PUBLISHER: Tell the system when we are done speaking
        self.pub_done = self.create_publisher(Bool, '/audio/playback_done', 10)
        
        # --- Audio Engine Setup ---
        self.engine = None
        self.sim_mode = False
        
        try:
            self.engine = pyttsx3.init()
            self.engine.setProperty('rate', 150)
            self.engine.setProperty('volume', 1.0)
            # Test if engine actually works
            self.engine.getProperty('voices')
            self.get_logger().info("🎤 TTS Engine Initialized Successfully")
        except Exception as e:
            self.get_logger().warn(f"⚠️ TTS Hardware Failed: {e}")
            self.get_logger().warn("⚠️ SWITCHING TO SIMULATION MODE (Text-only logging)")
            self.sim_mode = True
        
        self.speech_lock = threading.Lock()
        self.get_logger().info("TTS Node Started")

    def speak_callback(self, msg):
        self.get_logger().info(f"🗣️ Robot Says: '{msg.data}'")
        
        # Run speech in a separate thread so it doesn't block ROS
        t = threading.Thread(target=self._speak_thread, args=(msg.data,))
        t.start()

    def _speak_thread(self, text):
        # 1. SIMULATION MODE (No Hardware)
        if self.sim_mode:
            duration = len(text) * 0.1
            time.sleep(duration)
            self._publish_done()
            return

        # 2. REAL HARDWARE MODE
        with self.speech_lock:
            try:
                if self.engine:
                    self.engine.say(text)
                    self.engine.runAndWait()
            except Exception as e:
                self.get_logger().error(f"TTS Playback Error: {e}")
            finally:
                self._publish_done()

    def _publish_done(self):
        # Notify Session Manager that speech is finished
        msg = Bool()
        msg.data = True
        self.pub_done.publish(msg)
        # self.get_logger().info("✅ Speech Complete Signal Sent")

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()