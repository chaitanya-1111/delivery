#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from robot_interfaces.msg import FaceBox, AIRequest, AIResponse, Intent  # <--- Added AI messages and Intent
import time
from enum import Enum

class SessionState(Enum):
    IDLE = 0
    HUMAN_DETECTED = 1
    GREETING = 2
    TALKING = 3
    GOODBYE = 4

class SessionManagerNode(Node):
    def __init__(self):
        super().__init__('session_manager')

        # -------------------- PARAMETERS --------------------
        self.declare_parameter('timeout_limit', 8.0)
        self.declare_parameter('confirm_time', 1.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('goodbye_delay', 3.0)  # Time to wait after goodbye before resetting

        self.TIMEOUT_LIMIT = self.get_parameter('timeout_limit').value
        self.CONFIRM_TIME = self.get_parameter('confirm_time').value
        self.GOODBYE_DELAY = self.get_parameter('goodbye_delay').value
        rate = self.get_parameter('control_rate').value

        # -------------------- AI INTERFACE --------------------
        self.pub_ai_req = self.create_publisher(AIRequest, '/ai/request', 10)
        self.sub_ai_resp = self.create_subscription(AIResponse, '/ai/response', self.ai_response_cb, 10)

        # -------------------- SUBSCRIPTIONS --------------------
        self.create_subscription(FaceBox, '/face/primary', self.face_cb, 10)
        self.create_subscription(Bool, '/face/present', self.presence_cb, 10)
        # Listen to Intent Classifier instead of raw STT
        self.sub_intent = self.create_subscription(Intent, '/nlu/intent', self.intent_callback, 10)
        self.create_subscription(Bool, '/audio/playback_done', self.audio_done_cb, 10)

        # -------------------- PUBLISHERS --------------------
        self.pub_speech = self.create_publisher(String, '/robot/speech', 10)
        self.pub_state = self.create_publisher(String, '/session/state', 10)

        # -------------------- SESSION STATE --------------------
        self.state = SessionState.IDLE
        self.last_face_time = 0.0
        self.detect_start_time = None
        self.session_active = False
        
        # State Flags
        self.speaking = False
        self.greeting_sent = False
        # Locks the current face across callbacks so conversation doesn't jump.
        # `robot_interfaces/FaceBox` has no `tracking_id`, so we lock using a
        # stable key derived from bounding box fields.
        self.locked_face_id = None
        
        # Non-blocking Timers
        self.goodbye_start_time = None

        # -------------------- TIMER --------------------
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info("✅ Session Manager (AI-Powered) Started")

    # ======================================================
    # CALLBACKS
    # ======================================================

    def face_cb(self, msg: FaceBox):
        now = time.time()

        current_face_id = self._face_key(msg)

        # Lock face on first detection
        if self.locked_face_id is None:
            self.locked_face_id = current_face_id
            self.get_logger().info(f"🔒 Face locked: {self.locked_face_id}")

        # Ignore other faces to prevent "Conversation Jumping"
        if current_face_id != self.locked_face_id:
            return

        self.last_face_time = now

        # Start detection timer if this is the first time seeing face in IDLE
        if self.state == SessionState.IDLE and self.detect_start_time is None:
            self.detect_start_time = now

    def presence_cb(self, msg: Bool):
        if msg.data:
            self.last_face_time = time.time()

    def intent_callback(self, msg: Intent):
        # ---------------------------------------------------
        # 1. VOICE WAKE-UP LOGIC
        # ---------------------------------------------------
        # If we are IDLE but someone says "Hello", wake up!
        if self.state == SessionState.IDLE and msg.intent_type == "greeting":
            self.get_logger().info("🔊 Voice Wake-up Detected!")
            # Fake a face detection time so we don't immediately timeout
            self.last_face_time = time.time() 
            self.detect_start_time = time.time()
            self.session_active = True
            
            # Jump straight to Greeting logic
            self.transition(SessionState.GREETING)
            return

        # ---------------------------------------------------
        # 2. STANDARD CONVERSATION GUARD
        # ---------------------------------------------------
        # For all other commands (Order, Name, etc.), we must be in TALKING state
        if self.state != SessionState.TALKING:
            return

        self.get_logger().info(f"🧠 Processing Intent: {msg.intent_type}")

        # ---------------------------------------------------
        # 3. INTENT HANDLING
        # ---------------------------------------------------
        if msg.intent_type == "goodbye":
            self.transition(SessionState.GOODBYE)
            self.ask_ai("GOODBYE")

        elif msg.intent_type == "greeting":
            # If we are already talking and they say hello again, just chat
            self.ask_ai("GREETING")

        elif msg.intent_type == "check_order":
            self.ask_ai("DELIVERY", user_text=msg.raw_text)

        elif msg.intent_type == "query_identity":
            self.ask_ai("TALKING", user_text=msg.raw_text)

        elif msg.intent_type == "emergency_stop":
            self.say_audio("Emergency Stop Activated.")
            # TODO: Publish 0 velocity to wheels here
            
        else:
            self.ask_ai("TALKING", user_text=msg.raw_text)

    def audio_done_cb(self, msg: Bool):
        if msg.data:
            self.speaking = False
            self.get_logger().info("🔊 Audio finished")

    # 🆕 NEW CALLBACK: Handle what the AI tells us to say
    def ai_response_cb(self, msg: AIResponse):
        self.get_logger().info(f"🤖 AI says: '{msg.text}'")
        self.say_audio(msg.text)  # Forward to TTS

    # ======================================================
    # CORE LOGIC LOOP (Non-Blocking)
    # ======================================================

    def control_loop(self):
        now = time.time()

        # 1. GLOBAL TIMEOUT CHECK
        # If active session and face is gone for too long -> Trigger Goodbye
        if self.session_active and self.state != SessionState.GOODBYE:
            if (now - self.last_face_time) > self.TIMEOUT_LIMIT:
                self.get_logger().warn("⏳ Face timed out!")
                self.ask_ai("GOODBYE")  # ✅ AI generates goodbye message
                self.transition(SessionState.GOODBYE)
                return

        # 2. STATE MACHINE
        if self.state == SessionState.IDLE:
            # Transition happens in face_cb setting detect_start_time
            if self.detect_start_time is not None:
                self.transition(SessionState.HUMAN_DETECTED)

        elif self.state == SessionState.HUMAN_DETECTED:
            # Glitch Filter: If face lost during detection, reset
            if (now - self.last_face_time) > 1.0:
                self.get_logger().info("👻 Ghost detection (glitch). Resetting.")
                self.reset_session()
                self.state = SessionState.IDLE
                return

            # Confirm Presence: If face held for X seconds -> Start Session
            if (now - self.detect_start_time) >= self.CONFIRM_TIME:
                self.session_active = True
                self.transition(SessionState.GREETING)

        elif self.state == SessionState.GREETING:
            if not self.greeting_sent:
                self.ask_ai("GREETING")  # ✅ AI generates greeting
                self.greeting_sent = True
                self.transition(SessionState.TALKING)

        elif self.state == SessionState.TALKING:
            # Waiting for STT events (handled in stt_cb)
            pass

        elif self.state == SessionState.GOODBYE:
            # Initialize goodbye timer only once
            if self.goodbye_start_time is None:
                self.goodbye_start_time = now
            
            # Wait for speech to finish AND for delay to pass (Non-blocking wait)
            time_passed = now - self.goodbye_start_time
            if not self.speaking and time_passed > self.GOODBYE_DELAY:
                self.get_logger().info("🔄 Session Cycle Complete. Resetting.")
                self.reset_session()
                self.state = SessionState.IDLE

        self.publish_state()

    # ======================================================
    # HELPERS
    # ======================================================
    def _face_key(self, msg: FaceBox) -> str:
        """
        Create a stable identifier from FaceBox fields.

        The face box can jitter frame-to-frame, so we bucket the geometry to
        tolerate minor motion.
        """
        # Bucket sizes: tune if your camera produces larger/smaller jitter.
        bucket_xy = 40.0
        bucket_wh = 40.0

        cx = msg.x + (msg.w / 2.0)
        cy = msg.y + (msg.h / 2.0)

        return (
            f"cx{int(cx // bucket_xy)}_"
            f"cy{int(cy // bucket_xy)}_"
            f"w{int(msg.w // bucket_wh)}_"
            f"h{int(msg.h // bucket_wh)}"
        )

    # ✅ NEW WAY: Ask the AI Node what to say
    def ask_ai(self, mode, user_text=""):
        req = AIRequest()
        req.session_id = str(self.locked_face_id) if self.locked_face_id else "unknown"
        req.mode = mode
        req.user_text = user_text
        req.persona = "friendly"
        
        self.get_logger().info(f"📤 Asking AI: [{mode}] {user_text}")
        self.pub_ai_req.publish(req)

    # This sends text to the actual TTS hardware (The Mouth)
    def say_audio(self, text):
        if self.speaking:
            return
        msg = String()
        msg.data = text
        self.pub_speech.publish(msg)
        self.speaking = True

    def transition(self, new_state: SessionState):
        if self.state != new_state:
            self.get_logger().info(f"🔁 State: {self.state.name} → {new_state.name}")
        self.state = new_state
        
        # Start the goodbye timer freshly when entering GOODBYE
        if new_state == SessionState.GOODBYE:
            self.goodbye_start_time = None

    def reset_session(self):
        """Reset all memory to clean state for the next customer"""
        self.session_active = False
        self.detect_start_time = None
        self.last_face_time = 0.0
        self.locked_face_id = None
        self.greeting_sent = False
        self.speaking = False
        self.goodbye_start_time = None

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.pub_state.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SessionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()