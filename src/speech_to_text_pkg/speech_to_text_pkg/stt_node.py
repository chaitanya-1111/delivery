#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
import speech_recognition as sr
import threading
import sys
import select
from array import array

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')

        # Subscribe to audio data (from microphone node)
        self.sub_audio = self.create_subscription(Int16MultiArray, '/audio/data', self.audio_callback, 10)

        # Publish recognized text (to Session Manager)
        self.pub_text = self.create_publisher(String, '/audio/stt_text', 10)

        self.recognizer = sr.Recognizer()
        self.audio_sample_rate = 16000
        self.audio_sample_width = 2
        self.audio_buffer = bytearray()
        self.buffer_lock = threading.Lock()
        self.transcribing = False
        self.min_buffer_bytes = self.audio_sample_rate * self.audio_sample_width
        self.max_buffer_bytes = self.audio_sample_rate * self.audio_sample_width * 5

        self.get_logger().info("👂 STT Node Started")
        self.get_logger().info("🎙️ Listening for microphone audio on /audio/data")
        self.get_logger().info("⌨️  KEYBOARD OVERRIDE ACTIVE: Type in this terminal to 'speak' to the robot!")

        # Start the keyboard input thread (for WSL/Testing)
        self.input_thread = threading.Thread(target=self.keyboard_listener)
        self.input_thread.daemon = True
        self.input_thread.start()

    def audio_callback(self, msg):
        if not msg.data:
            return

        try:
            chunk = array('h', msg.data).tobytes()
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to convert audio chunk: {e}")
            return

        with self.buffer_lock:
            self.audio_buffer.extend(chunk)
            if len(self.audio_buffer) > self.max_buffer_bytes:
                self.audio_buffer = self.audio_buffer[-self.max_buffer_bytes:]
            buffer_len = len(self.audio_buffer)

        if not self.transcribing and buffer_len >= self.min_buffer_bytes:
            thread = threading.Thread(target=self.process_audio_buffer)
            thread.daemon = True
            thread.start()

    def process_audio_buffer(self):
        self.transcribing = True
        with self.buffer_lock:
            audio_bytes = bytes(self.audio_buffer)
            self.audio_buffer.clear()

        audio_data = sr.AudioData(audio_bytes, self.audio_sample_rate, self.audio_sample_width)
        try:
            text = self.recognizer.recognize_google(audio_data)
            self.publish_result(text)
        except sr.UnknownValueError:
            self.get_logger().info("🤖 STT could not understand audio")
        except sr.RequestError as e:
            self.get_logger().warn(f"⚠️ STT request failed: {e}")
        except Exception as e:
            self.get_logger().error(f"STT processing error: {e}")
        finally:
            self.transcribing = False

    def publish_result(self, text: str):
        if not text:
            return
        msg = String()
        msg.data = text
        self.pub_text.publish(msg)
        self.get_logger().info(f"🗣️ STT Recognized: '{text}'")

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
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()