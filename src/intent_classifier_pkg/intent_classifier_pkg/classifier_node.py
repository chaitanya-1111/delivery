#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import Intent

class IntentClassifierNode(Node):
    def __init__(self):
        super().__init__('intent_classifier_node')

        # 1. Subscribe to STT (The Ear)
        # We use String because that's what your STT node publishes
        self.sub_speech = self.create_subscription(
            String,
            '/audio/stt_text',
            self.speech_callback,
            10
        )

        # 2. Publish Intent (To The Brain)
        # We use /nlu/intent because that's what Session Manager listens to
        self.pub_intent = self.create_publisher(
            Intent,
            '/nlu/intent',
            10
        )

        self.get_logger().info("🧠 Intent Classifier Node Started (Production v1)")

    def speech_callback(self, msg: String):
        text = msg.data.lower().strip()
        intent, confidence = self.classify(text)

        # Create the message
        intent_msg = Intent()
        intent_msg.intent_type = intent  # <--- UPDATED
        intent_msg.raw_text = text            # <--- UPDATED
        intent_msg.confidence = float(confidence)
        intent_msg.session_id = "current"   # Optional, can leave empty
        self.pub_intent.publish(intent_msg)

        # Log for debugging
        self.get_logger().info(
            f"📌 Intent: {intent} ({confidence:.2f}) | Text: '{text}'"
        )

    def classify(self, text: str):
        """
        Deterministic rule-based intent classifier.
        Production v1 (fast, reliable).
        """
        if not text:
            return "unknown", 0.0

        # --- Greeting ---
        if any(word in text for word in ["hello", "hi", "hey", "morning", "greetings"]):
            return "greeting", 0.9

        # --- Goodbye ---
        if any(word in text for word in ["bye", "goodbye", "see you", "later"]):
            return "goodbye", 0.9

        # --- Confirmation ---
        if any(word in text for word in ["yes", "yeah", "correct", "confirm", "sure"]):
            return "confirm", 0.85

        # --- Denial ---
        if any(word in text for word in ["no", "nope", "wrong", "deny", "not"]):
            return "deny", 0.85

        # --- Asking about order ---
        if any(word in text for word in ["order", "package", "delivery", "status", "pizza", "item"]):
            return "check_order", 0.85

        # --- Asking name ---
        if "name" in text or "who are you" in text or "identity" in text:
            return "query_identity", 0.85

        # --- Emergency Stop (Safety First) ---
        if any(word in text for word in ["stop", "halt", "freeze"]):
            return "emergency_stop", 1.0

        return "unknown", 0.3

def main(args=None):
    rclpy.init(args=args)
    node = IntentClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()