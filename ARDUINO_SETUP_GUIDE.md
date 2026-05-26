# Arduino Motor Controller Setup Guide

## Quick Start

### 1. **Customize Pin Configuration**
Edit these lines in `arduino_motor_controller.ino` to match YOUR hardware:

```cpp
// Motor A (LEFT WHEEL)
#define MOTOR_LEFT_PWM    5       // PWM pin
#define MOTOR_LEFT_FWD    6       // Forward direction pin
#define MOTOR_LEFT_REV    7       // Reverse direction pin

// Motor B (RIGHT WHEEL)
#define MOTOR_RIGHT_PWM   9       // PWM pin
#define MOTOR_RIGHT_FWD   8       // Forward direction pin
#define MOTOR_RIGHT_REV   10      // Reverse direction pin

// ENCODERS
#define ENCODER_LEFT_A    2       // Must be interrupt pin (2 or 3 on Uno)
#define ENCODER_LEFT_B    4       // Quadrature pin
#define ENCODER_RIGHT_A   3       // Must be interrupt pin
#define ENCODER_RIGHT_B   11      // Quadrature pin
```

**❓ How to find your pins?**
- Check your Arduino board layout (printable on the board itself)
- PWM pins (marked with `~`): 3, 5, 6, 9, 10, 11 (Uno)
- Interrupt pins: 2, 3 (Uno) or 2, 3, 18, 19, 20, 21 (Mega)

### 2. **Upload to Arduino**
```bash
1. Open Arduino IDE
2. Copy-paste arduino_motor_controller.ino
3. Select Tools → Board → "Arduino Uno" (or your board)
4. Select Tools → Port → "/dev/ttyUSB0" (or your serial port)
5. Click Upload
```

### 3. **Verify It Works**
```bash
# Open Serial Monitor in Arduino IDE
# Select 115200 baud
# You should see: READY
```

---

## Hardware Wiring Checklist

### Motor Driver (H-Bridge)
| Signal | Arduino Pin | Motor Driver Pin | Purpose |
|--------|------------|------------------|---------|
| LEFT_PWM | 5 | IN1 | Left motor speed (0-255) |
| LEFT_FWD | 6 | Motor A positive | Forward direction |
| LEFT_REV | 7 | Motor A negative | Reverse direction |
| RIGHT_PWM | 9 | IN2 | Right motor speed (0-255) |
| RIGHT_FWD | 8 | Motor B positive | Forward direction |
| RIGHT_REV | 10 | Motor B negative | Reverse direction |
| GND | GND | GND | Common ground |

### Encoders
| Signal | Arduino Pin | Encoder Pin | Notes |
|--------|------------|-------------|-------|
| ENCODER_LEFT_A | 2 | Channel A | **Must be interrupt pin** |
| ENCODER_LEFT_B | 4 | Channel B | Quadrature feedback |
| ENCODER_RIGHT_A | 3 | Channel A | **Must be interrupt pin** |
| ENCODER_RIGHT_B | 11 | Channel B | Quadrature feedback |
| GND | GND | GND | Common ground |

### Power
- **Arduino**: 5V USB or external power supply
- **Motor Driver**: 12V (typical) from battery
- **Motors**: Connected to motor driver outputs
- **Encoders**: 5V from Arduino (pull-up resistors usually on encoder boards)

---

## Communication Protocol

### ROS2 → Arduino (Motor Commands)
```
Format: M <left_pwm> <right_pwm>\n
Range:  -255 to +255 (negative = reverse)
Example: M 120 118
         (Left motor at 120/255 speed, right at 118/255)
```

### Arduino → ROS2 (Encoder Feedback)
```
Format: E <left_ticks> <right_ticks>\n
Rate:   Every 100ms (adjustable with TELEMETRY_RATE_MS)
Example: E 5000 5050
         (Left wheel: 5000 total ticks, right: 5050)
```

### Startup Sequence
```
1. Arduino boots
2. Sends "READY\n"
3. ROS2 hardware_interface_node detects it
4. System starts exchanging M and E messages
```

---

## Testing Steps

### Test 1: Manual Serial Commands
```bash
# In Arduino IDE Serial Monitor (115200 baud):

# Command 1: Motors forward
M 100 100
# Expected: Motors spin, you see E messages with increasing tick counts

# Command 2: Motors stop
M 0 0
# Expected: Motors stop, tick counts stay constant

# Command 3: Left motor only
M 100 0
# Expected: Only left wheel spins, robot turns

# Command 4: Reverse
M -100 -100
# Expected: Motors spin backward
```

### Test 2: ROS2 Integration (after uploading)
```bash
# Terminal 1: Start hardware node in mock mode first to verify setup
ros2 launch robot_hardware_pkg hardware_bringup.launch.py mock_mode:=true

# Terminal 2: Publish a test command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}" --once

# Terminal 3: Monitor odometry
ros2 topic echo /odom
# Should see increasing position.x (forward motion)

# Terminal 4: Check if Arduino is connected
ls -la /dev/ttyUSB*
# Should exist (note the exact device path)
```

### Test 3: Real Hardware Test
```bash
# Terminal 1: Start with real hardware
ros2 launch robot_hardware_pkg hardware_bringup.launch.py \
  serial_port:=/dev/ttyUSB0 \
  mock_mode:=false

# Terminal 2: Watch diagnostics
ros2 topic echo /hardware/diagnostics

# Terminal 3: Send forward command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}" --once

# Physical test: Robot should move forward ~1-2 meters
```

---

## Troubleshooting

### Issue: "No READY message" or connection timeout

**Possible causes:**
1. Wrong USB port → Check `ls -la /dev/ttyUSB*`
2. Wrong baud rate → Verify `SERIAL_BAUD` (should be 115200)
3. Arduino not uploaded → Re-upload the sketch
4. USB cable issue → Try different cable

**Fix:**
```bash
# Find Arduino port
ls -la /dev/ttyUSB* /dev/ttyACM*

# Launch with correct port
ros2 launch robot_hardware_pkg hardware_bringup.launch.py serial_port:=/dev/ttyACM0
```

---

### Issue: Motors don't spin when commanded

**Possible causes:**
1. Wrong pin assignments → Check your H-bridge pinout
2. H-bridge not powered → Check 12V supply
3. Direction pins not set correctly → Test all combinations manually
4. PWM values too low → Try M 200 200

**Debug steps:**
1. Add `Serial.print()` in `set_motor_left()` to verify PWM values
2. Manually set pins with multimeter to test
3. Test motors directly with battery + switch (bypass Arduino)

**Example debug code:**
```cpp
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("M ")) {
      parse_motor_command(cmd);
      Serial.print("DEBUG: left_pwm=");
      Serial.print(left_pwm);
      Serial.print(" right_pwm=");
      Serial.println(right_pwm);
    }
  }
  // ... rest of loop
}
```

---

### Issue: Encoder ticks not incrementing

**Possible causes:**
1. Encoder not connected → Check continuity
2. Wrong interrupt pins → Pins 2 & 3 only on Uno
3. Encoder PPR mismatch → Check encoder datasheet

**Debug steps:**
```cpp
// Add to setup() temporarily
Serial.println("Encoder pin states:");
Serial.print("LEFT_A: ");
Serial.println(digitalRead(ENCODER_LEFT_A));
Serial.print("RIGHT_A: ");
Serial.println(digitalRead(ENCODER_RIGHT_A));

// Manually spin wheel and watch Serial Monitor
// Should see values change from 0 to 1
```

---

### Issue: Odometry wrong (robot shows different distance than actual)

**This is expected! You need to calibrate.**

Edit `hardware_interface_node.py` (lines 54-66):
```python
# Measure your actual wheel diameter (meters)
WHEEL_DIAMETER = 0.20      # Start with your measurement

# Measure distance between wheel centers (meters)
WHEEL_SEPARATION = 0.60

# From motor/encoder datasheet
ENCODER_PPR = 13           # Pulses per rotation
GEAR_RATIO = 71            # Motor gear reduction
```

**Calibration test:**
```bash
1. Run: ros2 launch robot_hardware_pkg hardware_bringup.launch.py
2. Publish: ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once
3. Measure actual distance robot moved (use measuring tape)
4. Check reported distance: ros2 topic echo /odom | grep position.x
5. If odom shows 2m but robot moved 1m → increase WHEEL_DIAMETER
6. If odom shows 1m but robot moved 2m → decrease WHEEL_DIAMETER
7. Repeat until they match
```

---

## Advanced: Quadrature Encoder Direction

If your encoder counts ticks in wrong direction, you have two options:

**Option 1: Reverse encoder in firmware** (simple)
```cpp
void encoder_left_isr() {
  encoder_left_ticks--;  // Changed from ++
}
```

**Option 2: Use quadrature pins** (advanced, more accurate)
```cpp
void encoder_left_isr_quad() {
  int a = digitalRead(ENCODER_LEFT_A);
  int b = digitalRead(ENCODER_LEFT_B);
  if (a == b) {
    encoder_left_ticks++;      // Forward
  } else {
    encoder_left_ticks--;      // Backward
  }
}

// Then in setup(), replace:
// attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoder_left_isr, CHANGE);
// with:
// attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoder_left_isr_quad, CHANGE);
```

---

## Summary

| Step | File | Action |
|------|------|--------|
| 1 | `arduino_motor_controller.ino` | Customize pin definitions |
| 2 | Arduino IDE | Upload sketch to board |
| 3 | Serial Monitor | Verify "READY" message |
| 4 | Hardware | Wire motors & encoders |
| 5 | ROS2 launch | `ros2 launch robot_hardware_pkg hardware_bringup.launch.py` |
| 6 | Calibration | Adjust WHEEL_DIAMETER in hardware_interface_node.py |

**You're ready to control motors! 🎯**
