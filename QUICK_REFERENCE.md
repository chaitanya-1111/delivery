# Quick Reference: Motor Command Flow

## The Complete Journey

```
ROS2 Nav2 System
    ↓ /cmd_vel (Twist: linear.x, angular.z)
CmdVelArbiter
    ↓ /cmd_vel (prioritized)
HardwareInterfaceNode
    ├─ Converts Twist → Differential Drive Math
    ├─ Twist to individual wheel velocities
    ├─ Velocities to PWM (0-255)
    └─ Sends Serial: "M <left_pwm> <right_pwm>\n"
    ↓ Serial @ 115200 baud
Arduino Microcontroller (this template)
    ├─ Receives: "M 120 118"
    ├─ Parses PWM values
    ├─ Sets pin HIGH/LOW for direction (FWD/REV)
    ├─ Writes PWM to motor driver
    └─ Returns: "E <left_ticks> <right_ticks>\n"
    ↓ Serial @ 115200 baud
HardwareInterfaceNode
    ├─ Parses encoder ticks
    ├─ Dead reckoning math (ticks → distance)
    ├─ Publishes /odom
    ├─ Publishes /tf
    └─ Updates odometry for Nav2
    ↓ /odom (position, velocity)
Nav2 Planner
    └─ Updates position, plans next path
```

---

## Arduino Template: 30-Second Setup

### 1. Edit Pin Definitions
```cpp
#define MOTOR_LEFT_PWM    5       // Your PWM pin
#define MOTOR_LEFT_FWD    6       // Forward pin
#define MOTOR_LEFT_REV    7       // Reverse pin
#define MOTOR_RIGHT_PWM   9
#define MOTOR_RIGHT_FWD   8
#define MOTOR_RIGHT_REV   10
#define ENCODER_LEFT_A    2       // MUST be interrupt pin
#define ENCODER_RIGHT_A   3       // MUST be interrupt pin
```

### 2. Upload to Arduino
- Paste code in Arduino IDE
- Select correct board & port
- Click Upload

### 3. Test in Serial Monitor (115200 baud)
```
Type: M 100 100
See:  E 1234 5678
```

---

## Serial Protocol Reference

### Incoming (ROS2 → Arduino)
```
M <left_pwm> <right_pwm>

Examples:
  M 100 100     = Both motors forward at 100/255 speed
  M 100 -100    = Left forward, right reverse = turn left
  M 0 0         = Stop
  M -50 -50     = Both motors backward
  M 150 75      = Arc forward (right side slower)

Range: -255 to +255 where:
  Positive = motor forward
  Negative = motor reverse
  0 = stop
```

### Outgoing (Arduino → ROS2)
```
E <left_ticks> <right_ticks>

Examples:
  E 0 0         = Just started, no motion
  E 5000 5050   = Both wheels moved, small difference
  E 5000 3000   = Right wheel slower = turning right
  E -1000 1000  = Reversing (negative counts)

Sent every 100ms (adjustable TELEMETRY_RATE_MS)
```

### Startup
```
READY

Sent exactly once when Arduino boots.
ROS2 waits for this before sending motor commands.
```

---

## Motor/Encoder Connections

### To H-Bridge Motor Driver
```
Arduino     H-Bridge    Function
Pin 5 (PWM) → IN1       Left motor speed
Pin 6       → Motor A+  Left forward
Pin 7       → Motor A-  Left reverse
Pin 9 (PWM) → IN2       Right motor speed
Pin 8       → Motor B+  Right forward
Pin 10      → Motor B-  Right reverse
GND         → GND       Common ground
```

### To Encoders
```
Arduino     Encoder     Function
Pin 2       → Channel A Left wheel feedback (INTERRUPT)
Pin 4       → Channel B Left quadrature
Pin 3       → Channel A Right wheel feedback (INTERRUPT)
Pin 11      → Channel B Right quadrature
GND         → GND       Common ground
```

---

## Interrupt Pins (Arduino Uno)
```
Interrupt 0 = Digital Pin 2
Interrupt 1 = Digital Pin 3

⚠️ You MUST use pins 2 or 3 for encoder inputs!
Other pins cannot trigger interrupts on Uno.

For Arduino Mega:
  Interrupt pins: 2, 3, 18, 19, 20, 21
```

---

## Calibration Parameters

Edit in `hardware_interface_node.py` (lines 54-66):

```python
WHEEL_DIAMETER = 0.20              # Meters (measure physical wheel)
WHEEL_SEPARATION = 0.60            # Meters (distance between wheel centers)
ENCODER_PPR = 13                   # Pulses per rotation (from datasheet)
QUADRATURE_MULT = 4                # Usually 4 for quadrature
GEAR_RATIO = 71                    # Motor gear reduction (from datasheet)
```

**Calibration test:**
```bash
ros2 launch robot_hardware_pkg hardware_bringup.launch.py
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once
# Manually measure distance, compare with /odom
```

---

## Common Issues & Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| No "READY" | Serial port wrong | Check `ls -la /dev/ttyUSB*` |
| Motors don't spin | Wrong pins | Verify pin assignments match H-bridge |
| Wrong direction | Motor reversed | Add `-` in set_motor function or swap FWD/REV pins |
| Encoders silent | Wrong interrupt pin | Must use pin 2 or 3 (Uno) |
| Odometry wrong | Calibration off | Measure wheel & tune WHEEL_DIAMETER |
| Robot turns left | Right wheel slower | Balance PWM values or check encoder |

---

## ROS2 Command Examples

```bash
# Start hardware with real Arduino
ros2 launch robot_hardware_pkg hardware_bringup.launch.py

# Or mock mode (no hardware needed)
ros2 launch robot_hardware_pkg hardware_bringup.launch.py mock_mode:=true

# Send forward command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}" --once

# Send turn left command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 1.0}}" --once

# Send turn right command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: -1.0}}" --once

# Send reverse command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: -0.3}}" --once

# Monitor odometry
ros2 topic echo /odom | grep "^      x:"

# Watch diagnostics
ros2 topic echo /hardware/diagnostics
```

---

## Files You Need

| File | Purpose |
|------|---------|
| `arduino_motor_controller.ino` | Arduino firmware (this template) |
| `ARDUINO_SETUP_GUIDE.md` | Detailed setup guide |
| `hardware_interface_node.py` | ROS2 side (already configured) |
| `cmd_vel_arbiter_node.py` | Command prioritizer (already configured) |

---

## Differential Drive Kinematics (what hardware_interface_node does)

```
Given:
  v = desired linear velocity (m/s)
  ω = desired angular velocity (rad/s)
  L = wheel separation (0.6m)

Compute per-wheel velocities:
  v_left  = v - (ω × L / 2)
  v_right = v + (ω × L / 2)

Then:
  left_pwm  = (v_left / MAX_VEL) × 255
  right_pwm = (v_right / MAX_VEL) × 255

Example: Move forward (v=0.5, ω=0)
  v_left = 0.5 - 0 = 0.5
  v_right = 0.5 + 0 = 0.5
  Both motors same speed ✓

Example: Turn left (v=0.3, ω=1.0)
  v_left = 0.3 - (1.0 × 0.6 / 2) = 0.0
  v_right = 0.3 + (1.0 × 0.6 / 2) = 0.6
  Left motor stops, right faster = turn ✓
```

---

## Performance Targets

| Metric | Target | How to Verify |
|--------|--------|---------------|
| Serial latency | <50ms | `ros2 run tf2_tools static_transform_publisher 0 0 0 0 0 0 base_link odom` |
| Odometry drift | <5% per 10m | Drive straight 10m, measure error |
| Motor response | <100ms | Pub cmd_vel, watch /odom update |
| Encoder accuracy | ±2% | Test with known distance |
| Turn radius | Within 5cm | Drive 1m circle, measure |

---

## Next Steps

1. ✅ Download `arduino_motor_controller.ino`
2. ✅ Edit pin definitions for your hardware
3. ✅ Upload to Arduino
4. ✅ Test with Serial Monitor
5. ✅ Wire motors & encoders
6. ✅ Launch ROS2 hardware node
7. ✅ Calibrate wheel parameters
8. ✅ Test with Nav2 or teleop

**Done! Your robot can now move! 🤖**
