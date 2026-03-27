#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from robot_interfaces.msg import FaceBox
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge
import os
from ament_index_python.packages import get_package_share_directory

class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector_node')
        self.CONFIDENCE_THRESHOLD = 0.6
        self.NMS_THRESHOLD = 0.3
        
        # Robust Model Path Finding
        try:
            package_share = get_package_share_directory('face_detection_pkg')
            model_path = os.path.join(package_share, 'models', 'face_detection_yunet_2023mar.onnx')
            if not os.path.exists(model_path):
                 # Fallback for dev environment (running from source)
                 model_path = os.path.abspath("face_detection_pkg/models/face_detection_yunet_2023mar.onnx")
        except:
            model_path = os.path.abspath("face_detection_pkg/models/face_detection_yunet_2023mar.onnx")

        self.get_logger().info(f"Loading YuNet model from: {model_path}")
        
        self.face_detector = None
        try:
            self.face_detector = cv2.FaceDetectorYN.create(
                model=model_path,
                config="",
                input_size=(320, 320),
                score_threshold=self.CONFIDENCE_THRESHOLD,
                nms_threshold=self.NMS_THRESHOLD,
                top_k=5000,
                backend_id=cv2.dnn.DNN_BACKEND_OPENCV,
                target_id=cv2.dnn.DNN_TARGET_CPU
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load YuNet: {e}")

        self.bridge = CvBridge()
        
        # Best Effort QoS for Video
        qos_video = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.sub_cam = self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_video)
        self.pub_face = self.create_publisher(FaceBox, '/face/primary', 10)
        self.pub_present = self.create_publisher(Bool, '/face/present', 10)
        self.get_logger().info("Face Detector Node (YuNet) Started")

    def image_callback(self, msg):
        if self.face_detector is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            self.face_detector.setInputSize((width, height))
            _, faces = self.face_detector.detect(cv_image)
            
            face_present = False
            if faces is not None:
                largest_face = None
                max_area = 0
                for face in faces:
                    x, y, w, h = map(int, face[0:4])
                    confidence = float(face[14])
                    area = w * h
                    if area > max_area:
                        max_area = area
                        largest_face = (x, y, w, h, confidence)
                if largest_face:
                    face_present = True
                    x, y, w, h, conf = largest_face
                    box_msg = FaceBox()
                    box_msg.x = x
                    box_msg.y = y
                    box_msg.w = w
                    box_msg.h = h
                    box_msg.confidence = conf
                    self.pub_face.publish(box_msg)
            self.pub_present.publish(Bool(data=face_present))
        except Exception as e:
            self.get_logger().error(f"Vision Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
