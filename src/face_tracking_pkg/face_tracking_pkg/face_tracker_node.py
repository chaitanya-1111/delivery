
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import FaceBox, FaceTarget
from std_msgs.msg import Bool

class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker_node')
        
        # --- Configuration ---
        self.IMG_WIDTH = 640
        self.IMG_HEIGHT = 480
        self.CENTER_X = self.IMG_WIDTH // 2
        self.CENTER_Y = self.IMG_HEIGHT // 2
        
        # --- Topics ---
        self.sub_face = self.create_subscription(FaceBox, '/face/primary', self.face_callback, 10)
        self.pub_target = self.create_publisher(FaceTarget, '/face/target', 10)
        self.pub_lost = self.create_publisher(Bool, '/face/lost', 10)
        
        # --- State ---
        self.last_face_time = self.get_clock().now()
        self.TIMEOUT_SEC = 2.0 # If no face for 2s, center the head

        # Timer to check for lost face
        self.timer = self.create_timer(0.1, self.check_lost)
        self.get_logger().info("Face Tracker Node Started")

    def face_callback(self, msg):
        self.last_face_time = self.get_clock().now() 
        # Calculate center of the face
        face_cx = msg.x + (msg.w // 2)
        face_cy = msg.y + (msg.h // 2)
        
        # Normalize coordinates (-1.0 to +1.0)
        # -1.0 = Far Left/Top
        # +1.0 = Far Right/Bottom
        # 0.0  = Center
        norm_x = (face_cx - self.CENTER_X) / (self.IMG_WIDTH / 2)
        norm_y = (face_cy - self.CENTER_Y) / (self.IMG_HEIGHT / 2)
        
        # Create Target Message
        target = FaceTarget()
        target.cx = int(face_cx)
        target.cy = int(face_cy)
        target.norm_x = float(norm_x)
        target.norm_y = float(norm_y)
        target.face_found = True
        
        self.pub_target.publish(target)
        self.pub_lost.publish(Bool(data=False))

    def check_lost(self):
        # Check if face is lost
        time_diff = self.get_clock().now() - self.last_face_time
        if time_diff.nanoseconds > (self.TIMEOUT_SEC * 1e9):
            # Face Lost -> Send "Center" command (0,0)
            target = FaceTarget()
            target.norm_x = 0.0
            target.norm_y = 0.0
            target.face_found = False
            
            self.pub_target.publish(target)
            self.pub_lost.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

