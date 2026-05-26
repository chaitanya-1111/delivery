/*
 * ═══════════════════════════════════════════════════════════════════
 * ARDUINO MOTOR CONTROLLER FIRMWARE
 * Delivery Robot v1.0
 * ═══════════════════════════════════════════════════════════════════
 *
 * PURPOSE:
 *   Bridge between ROS2 (hardware_interface_node) and physical motors
 *   - Receive PWM commands from ROS2: "M <left_pwm> <right_pwm>\n"
 *   - Drive motors via H-bridge motor driver
 *   - Read encoder feedback from both wheels
 *   - Send encoder ticks back to ROS2: "E <left_ticks> <right_ticks>\n"
 *
 * PROTOCOL:
 *   INPUT:  M 120 118       (motor command: left_pwm=120, right_pwm=118)
 *   OUTPUT: E 5000 5050     (encoder feedback: left_ticks, right_ticks)
 *   STARTUP: READY          (sent on boot, signals readiness to ROS2)
 *
 * ═══════════════════════════════════════════════════════════════════
 */

// ═══════════════════════════════════════════════════════════════════
// PIN CONFIGURATION — CUSTOMIZE FOR YOUR HARDWARE
// ═══════════════════════════════════════════════════════════════════

// ── Motor A (LEFT WHEEL) ──────────────────────────────────────────
#define MOTOR_LEFT_PWM    5       // PWM pin for left motor speed (0-255)
#define MOTOR_LEFT_FWD    6       // Direction pin (HIGH=forward, LOW=reverse)
#define MOTOR_LEFT_REV    7       // Direction pin complement

// ── Motor B (RIGHT WHEEL) ─────────────────────────────────────────
#define MOTOR_RIGHT_PWM   9       // PWM pin for right motor speed (0-255)
#define MOTOR_RIGHT_FWD   8       // Direction pin (HIGH=forward, LOW=reverse)
#define MOTOR_RIGHT_REV   10      // Direction pin complement

// ── ENCODER A (LEFT WHEEL) ────────────────────────────────────────
// Most Arduino boards: interrupts on digital 2 and 3 (or 20, 21 on Mega)
#define ENCODER_LEFT_A    2       // Interrupt pin (INT0)
#define ENCODER_LEFT_B    4       // Quadrature pin (optional, for direction)

// ── ENCODER B (RIGHT WHEEL) ───────────────────────────────────────
#define ENCODER_RIGHT_A   3       // Interrupt pin (INT1)
#define ENCODER_RIGHT_B   11      // Quadrature pin (optional, for direction)

// ── SERIAL COMMUNICATION ──────────────────────────────────────────
#define SERIAL_BAUD       115200  // Match ROS2 setting
#define TELEMETRY_RATE_MS 100     // Send encoder ticks every 100ms

// ═══════════════════════════════════════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════════════════════════════════════

volatile long encoder_left_ticks = 0;
volatile long encoder_right_ticks = 0;

long last_left_ticks = 0;
long last_right_ticks = 0;

unsigned long last_telemetry_time = 0;

// Motor command buffer
int left_pwm = 0;
int right_pwm = 0;

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════

void setup() {
  // Initialize serial communication with ROS2
  Serial.begin(SERIAL_BAUD);
  delay(500);  // Wait for serial to stabilize
  
  // Signal that Arduino is ready
  Serial.println("READY");
  
  // ── Configure motor output pins ───────────────────────────────
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_REV, OUTPUT);
  
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_REV, OUTPUT);
  
  // Stop motors on startup
  stop_motors();
  
  // ── Configure encoder input pins ──────────────────────────────
  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);
  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);
  
  // ── Attach interrupt handlers ─────────────────────────────────
  // When encoder pin changes, count the tick
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A),  encoder_left_isr,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoder_right_isr, CHANGE);
  
  Serial.println("Motor controller initialized");
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════

void loop() {
  // ── Process incoming ROS2 commands ────────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("M ")) {
      // Motor command: "M left_pwm right_pwm"
      parse_motor_command(cmd);
    }
  }
  
  // ── Send telemetry periodically ───────────────────────────────
  unsigned long now = millis();
  if (now - last_telemetry_time >= TELEMETRY_RATE_MS) {
    send_telemetry();
    last_telemetry_time = now;
  }
}

// ═══════════════════════════════════════════════════════════════════
// MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════════

void set_motor_left(int pwm) {
  /*
   * pwm range: -255 to +255
   *   Positive = forward
   *   Negative = reverse
   *   0 = stop
   */
  pwm = constrain(pwm, -255, 255);
  left_pwm = pwm;
  
  if (pwm > 0) {
    // Forward
    digitalWrite(MOTOR_LEFT_FWD, HIGH);
    digitalWrite(MOTOR_LEFT_REV, LOW);
    analogWrite(MOTOR_LEFT_PWM, pwm);
  } 
  else if (pwm < 0) {
    // Reverse
    digitalWrite(MOTOR_LEFT_FWD, LOW);
    digitalWrite(MOTOR_LEFT_REV, HIGH);
    analogWrite(MOTOR_LEFT_PWM, -pwm);
  } 
  else {
    // Stop
    digitalWrite(MOTOR_LEFT_FWD, LOW);
    digitalWrite(MOTOR_LEFT_REV, LOW);
    analogWrite(MOTOR_LEFT_PWM, 0);
  }
}

void set_motor_right(int pwm) {
  /*
   * pwm range: -255 to +255
   *   Positive = forward
   *   Negative = reverse
   *   0 = stop
   */
  pwm = constrain(pwm, -255, 255);
  right_pwm = pwm;
  
  if (pwm > 0) {
    // Forward
    digitalWrite(MOTOR_RIGHT_FWD, HIGH);
    digitalWrite(MOTOR_RIGHT_REV, LOW);
    analogWrite(MOTOR_RIGHT_PWM, pwm);
  } 
  else if (pwm < 0) {
    // Reverse
    digitalWrite(MOTOR_RIGHT_FWD, LOW);
    digitalWrite(MOTOR_RIGHT_REV, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, -pwm);
  } 
  else {
    // Stop
    digitalWrite(MOTOR_RIGHT_FWD, LOW);
    digitalWrite(MOTOR_RIGHT_REV, LOW);
    analogWrite(MOTOR_RIGHT_PWM, 0);
  }
}

void stop_motors() {
  set_motor_left(0);
  set_motor_right(0);
}

// ═══════════════════════════════════════════════════════════════════
// COMMAND PARSING
// ═══════════════════════════════════════════════════════════════════

void parse_motor_command(String cmd) {
  /*
   * Parse: "M <left_pwm> <right_pwm>"
   * Example: "M 120 118"
   */
  
  // Remove "M " prefix
  cmd = cmd.substring(2);
  
  int space_pos = cmd.indexOf(' ');
  if (space_pos == -1) {
    return;  // Malformed command
  }
  
  String left_str = cmd.substring(0, space_pos);
  String right_str = cmd.substring(space_pos + 1);
  
  int left_pwm = left_str.toInt();
  int right_pwm = right_str.toInt();
  
  set_motor_left(left_pwm);
  set_motor_right(right_pwm);
}

// ═══════════════════════════════════════════════════════════════════
// ENCODER ISR (Interrupt Service Routines)
// ═══════════════════════════════════════════════════════════════════

void encoder_left_isr() {
  /*
   * Called every time ENCODER_LEFT_A changes (rising or falling edge).
   * 
   * SIMPLE MODE: Count every edge = 1 tick
   * 
   * ADVANCED MODE (if you want direction):
   *   Read both A and B, determine direction
   *   Then decide to increment or decrement
   * 
   * For now, simple mode: always increment
   * (ROS2 handles direction from motor PWM sign)
   */
  encoder_left_ticks++;
}

void encoder_right_isr() {
  encoder_right_ticks++;
}

// ═══════════════════════════════════════════════════════════════════
// TELEMETRY (Send encoder feedback to ROS2)
// ═══════════════════════════════════════════════════════════════════

void send_telemetry() {
  /*
   * Send: "E <left_ticks> <right_ticks>"
   * Example: "E 5000 5050"
   * 
   * ROS2 hardware_interface_node expects this format.
   * It reads the ticks and updates odometry.
   */
  
  // Disable interrupts briefly to avoid tick count changing mid-read
  noInterrupts();
  long left_ticks = encoder_left_ticks;
  long right_ticks = encoder_right_ticks;
  interrupts();
  
  // Format: "E <left> <right>"
  Serial.print("E ");
  Serial.print(left_ticks);
  Serial.print(" ");
  Serial.println(right_ticks);
}

// ═══════════════════════════════════════════════════════════════════
// OPTIONAL UTILITIES
// ═══════════════════════════════════════════════════════════════════

/*
 * EXAMPLE: Reset encoder counters
 * (You might want to call this if you need to zero odometry)
 * 
 * void reset_encoders() {
 *   noInterrupts();
 *   encoder_left_ticks = 0;
 *   encoder_right_ticks = 0;
 *   interrupts();
 * }
 */

/*
 * EXAMPLE: More sophisticated encoder reading with direction
 * If your encoder has A and B quadrature pins, you can determine
 * direction and count only in one direction:
 * 
 * void encoder_left_isr_quad() {
 *   int a = digitalRead(ENCODER_LEFT_A);
 *   int b = digitalRead(ENCODER_LEFT_B);
 *   if (a == b) {
 *     encoder_left_ticks++;      // Forward
 *   } else {
 *     encoder_left_ticks--;      // Backward
 *   }
 * }
 */

// ═══════════════════════════════════════════════════════════════════
// END OF TEMPLATE
// ═══════════════════════════════════════════════════════════════════
/*
 * 
 * INSTALLATION & TESTING:
 * 
 * 1. UPLOAD THIS CODE:
 *    - Open Arduino IDE
 *    - Paste this code
 *    - Select your board type (Uno, Mega, etc.)
 *    - Upload
 *    - Open Serial Monitor (115200 baud)
 *    - You should see "READY" printed
 * 
 * 2. TEST MANUALLY IN SERIAL MONITOR:
 *    Type: M 100 100
 *    Motors should spin forward
 *    You should see encoder ticks printed: E 1234 1234
 *    
 *    Type: M 0 0
 *    Motors stop
 *    
 *    Type: M -100 -100
 *    Motors go backward
 * 
 * 3. TEST WITH ROS2:
 *    ros2 launch robot_hardware_pkg hardware_bringup.launch.py
 *    ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}"
 *    Watch /odom and /tf to see the robot move
 * 
 * TROUBLESHOOTING:
 * 
 * Q: Motors not spinning?
 *    A: Check pin assignments match your H-bridge
 *    A: Verify power supply to motor driver
 *    A: Check PWM is actually changing (add Serial.print in loop)
 * 
 * Q: No "READY" on startup?
 *    A: Serial port might not be connected
 *    A: Baud rate mismatch (check SERIAL_BAUD)
 *    A: Arduino might not be uploaded
 * 
 * Q: Encoders not counting?
 *    A: Check encoder pins are on interrupt-capable digital pins
 *    A: Verify encoder is electrically connected (continuity)
 *    A: Add Serial.print in ISR to debug (careful, ISR should be fast)
 * 
 * Q: Robot odometry wrong?
 *    A: Calibrate WHEEL_DIAMETER in hardware_interface_node.py
 *    A: Check encoder PPR and gear ratio match your hardware
 *    A: Verify both encoders count at the same rate
 * 
 */
