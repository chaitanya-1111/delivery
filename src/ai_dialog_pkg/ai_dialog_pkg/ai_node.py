#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import AIRequest, AIResponse

class AIDialogNode(Node):
    def __init__(self):
        super().__init__('ai_dialog_node')

        # Subscribe to requests from Session Manager
        self.sub_request = self.create_subscription(
            AIRequest,
            '/ai/request',
            self.request_callback,
            10
        )

        # Publish responses back to Session Manager
        self.pub_response = self.create_publisher(
            AIResponse,
            '/ai/response',
            10
        )

        self.get_logger().info("🧠 AI Dialog Node Online (Rule-Based Mode)")

    def request_callback(self, msg: AIRequest):
        self.get_logger().info(f"📨 Request: [{msg.mode}] '{msg.user_text}'")
        
        # Generate the text
        response_text = self.generate_response(msg)
        
        # Create response message
        resp = AIResponse()
        resp.text = response_text
        resp.end_session = False
        
        # Send it back
        self.pub_response.publish(resp)

    def generate_response(self, msg: AIRequest) -> str:
        """
        This is where the 'Intelligence' lives.
        Currently Rule-Based. Replace with LLM API call later.
        """
        mode = msg.mode
        user_text = msg.user_text.lower()

        # 🟢 GREETING
        if mode == "GREETING":
            return "Hello! I am your delivery robot. Please state your name or order number."

        # 🔵 DELIVERY (Authority Mode)
        elif mode == "DELIVERY":
            return f"Thank you. Your identity is verified. Accessing order for {msg.session_id}."

        # 🔴 GOODBYE
        elif mode == "GOODBYE":
            return "Goodbye! Have a wonderful day."

        # 🟡 TALKING (The Conversation)
        elif mode == "TALKING":
            if "name" in user_text:
                return "Nice to meet you. I am Robo-One."
            elif "weather" in user_text:
                return "I do not have weather sensors, but it feels nice today."
            elif "delivery" in user_text or "package" in user_text:
                return "I have a secure package on board. I need verification to open it."
            else:
                return f"I heard you say {msg.user_text}, but I am focused on delivery right now."

        return "I am ready."

def main(args=None):
    rclpy.init(args=args)
    node = AIDialogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()