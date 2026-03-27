#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray  # Standard ROS message for audio data
import pyaudio
import threading

class MicrophoneNode(Node):
    def __init__(self):
        super().__init__('microphone_node')
        
        # Publisher
        self.publisher_ = self.create_publisher(Int16MultiArray, '/audio/data', 10)
        
        # Audio Config
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.recording = False

        self.get_logger().info("Microphone Node Started")
        
        # Attempt to open stream (Simulated on WSL often fails, so we handle it)
        try:
            self.stream = self.p.open(format=self.FORMAT,
                                      channels=self.CHANNELS,
                                      rate=self.RATE,
                                      input=True,
                                      frames_per_buffer=self.CHUNK)
            self.recording = True
            self.thread = threading.Thread(target=self.record_loop)
            self.thread.start()
        except OSError:
            self.get_logger().warn("⚠️ NO MICROPHONE DETECTED (Common on WSL)")
            self.get_logger().warn("⚠️ SWITCHING TO SIMULATION MODE (Generating silent audio)")
            self.recording = False
            # Start a dummy timer to simulate data flow
            self.timer = self.create_timer(0.1, self.simulated_audio)

    def record_loop(self):
        while rclpy.ok() and self.recording:
            try:
                data = self.stream.read(self.CHUNK, exception_on_overflow=False)
                # Convert raw bytes to Int16 array
                audio_ints = list(memoryview(data).cast('h'))
                msg = Int16MultiArray()
                msg.data = audio_ints
                self.publisher_.publish(msg)
            except Exception:
                break

    def simulated_audio(self):
        # Publishes empty audio packets just to keep the topic alive
        msg = Int16MultiArray()
        msg.data = [0] * 512
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.recording = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()