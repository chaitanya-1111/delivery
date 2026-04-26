# Safety Supervisor Package

Hardware-level safety supervisor for velocity gating and emergency stop. This package implements the last line of defense: every velocity command passes through safety approval before reaching the hardware.

## Package Structure

```
safety_supervisor_pkg/
├── safety_supervisor_pkg/
│   ├── __init__.py
│   ├── safety_supervisor_node.py        (440 lines) — Main cmd_vel gating + safety logic
│   ├── hardware_watchdog_node.py        (328 lines) — Component heartbeat monitoring
│   └── safety_logger_node.py            (297 lines) — Safety event JSONL logging
├── config/
│   └── safety_config.yaml               — All zones, timeouts, policy flags
├── launch/
│   └── safety_bringup.launch.py         — Launch all safety components
├── package.xml                          — ROS 2 package manifest
├── setup.py                             — Python package setup
├── setup.cfg                            — Setup configuration
└── README.md                            — This file
```

## What This Package Does

```
Nav2/Mission  →  /cmd_vel      ⤐
Teleop        →  /cmd_vel_teleop ⤴→ [Safety Gate] → /cmd_vel_safe → Hardware

↓ Additional inputs:
  LIDAR/Proximity: Distance checking
  E-Stop: Hardware/software trigger
  Heartbeat: Component health
```

The package enforces **4 safety zones** based on obstacle distance:

| Zone | Range | Action |
|------|-------|--------|
| **FREE** | > 0.60m | Full speed (1.0 scaling) |
| **CAUTION** | 0.35–0.60m | Speed scaling (0.5 × command) |
| **CRITICAL** | 0.20–0.35m | Max 0.1 × command (very slow) |
| **EMERGENCY** | < 0.20m | **FULL STOP** (0.0) |

## Installation

### 1. Copy to workspace

```bash
cp -r safety_supervisor_pkg ~/delivery_bot_ws/src/
```

### 2. Install dependencies

```bash
cd ~/delivery_bot_ws
sudo rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build package

```bash
colcon build --symlink-install --packages-select safety_supervisor_pkg
source install/setup.bash
```

## Launch

Safety **must** start BEFORE navigation:

```bash
# Launch all safety components
ros2 launch safety_supervisor_pkg safety_bringup.launch.py

# Optional: Launch with custom namespace
ros2 launch safety_supervisor_pkg safety_bringup.launch.py namespace:=robot1

# Custom log level
ros2 launch safety_supervisor_pkg safety_bringup.launch.py log_level:=debug
```

## Nodes

### 1. Safety Supervisor Node

**Main executable**: `safety_supervisor_node`

Implements velocity gating logic:
- Subscribes to `/cmd_vel` (navigation) and `/cmd_vel_teleop` (operator)
- Applies safety checks: zone-based scaling, e-stop, sensor timeouts
- Publishes `/cmd_vel_safe` to hardware
- Publishes `/safety/status` (JSON state), `/safety/speed_scale`, diagnostics

**Key logic**:
```python
# 1. Check sensor health (LIDAR timeout, hardware heartbeat)
if not healthy:
    output = STOP

# 2. Apply e-stop
if estop_active:
    output = STOP

# 3. Apply proximity zone scaling
if distance < 0.20m:
    output = STOP  # Emergency
elif distance < 0.35m:
    output *= 0.1  # Critical
elif distance < 0.60m:
    output *= 0.5  # Caution
else:
    output *= 1.0  # Free

# 4. Publish gated command
publish(output)
```

### 2. Hardware Watchdog Node

**Main executable**: `hardware_watchdog_node`

Monitors component heartbeats:
- Tracks latency of motor controllers, LIDAR, IMU, power system
- Publishes `/system/health` (detailed JSON) and `/system/health_summary` (OK/DEGRADED/CRITICAL)
- Generates component-level diagnostics

**Components monitored**:
- `motor_controller_left` — Heartbeat from left motor
- `motor_controller_right` — Heartbeat from right motor
- `lidar_scanner` — LIDAR scan activity
- `imu_sensor` — IMU data stream
- `power_management` — Battery/power system
- `network_interface` — ROS network connectivity

### 3. Safety Logger Node

**Main executable**: `safety_logger_node`

Logs all safety events to JSONL:
- Subscribes to `/safety/event` for structured event logging
- Writes time-stamped JSONL records to `~/.ros/safety_logs/`
- Rotates logs when they exceed size limit
- Compresses old logs (optional)
- Publishes `/logging/stats`

**Log format** (JSONL):
```json
{"timestamp": 1704067200.5, "event": "ZONE_CHANGE_FREE_to_CAUTION", "severity": "INFO", "state": "NOMINAL", "zone": "CAUTION", "distance": 0.48, "speed_scale": 0.5}
{"timestamp": 1704067202.1, "event": "ESTOP_TRIGGERED", "severity": "CRITICAL", "state": "ESTOP_ACTIVE", "zone": "EMERGENCY", "distance": 0.15, "speed_scale": 0.0}
```

## Topics

### Input Topics

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Navigation | Velocity from Nav2 |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | Operator | Velocity from teleop |
| `/scan` | `sensor_msgs/LaserScan` | LIDAR | Proximity scan |
| `/safety/estop_request` | `std_msgs/Bool` | Operator/Safety | Trigger e-stop |
| `/system/health_summary` | `std_msgs/String` | Watchdog | Nav system health |
| `/hardware/heartbeat` | `std_msgs/Bool` | Hardware | HW alive signal |

### Output Topics

| Topic | Type | Published by | Description |
|-------|------|--------------|-------------|
| `/cmd_vel_safe` | `geometry_msgs/Twist` | **Safety Supervisor** | Gated velocity (to hardware) |
| `/safety/status` | `std_msgs/String` | **Safety Supervisor** | Full safety state (JSON) |
| `/safety/speed_scale` | `std_msgs/Float32` | **Safety Supervisor** | Current speed scale [0.0–1.0] |
| `/safety/event` | `std_msgs/String` | **Safety Supervisor** | Safety event (JSON) |
| `/safety/estop` | `std_msgs/Bool` | **Safety Supervisor** | E-stop state |
| `/system/health` | `std_msgs/String` | **Watchdog** | Detailed component health (JSON) |
| `/system/health_summary` | `std_msgs/String` | **Watchdog** | Summary: OK/DEGRADED/CRITICAL |
| `/system/component_status` | `std_msgs/String` | **Watchdog** | Per-component status (JSON) |
| `/logging/stats` | `std_msgs/String` | **Logger** | Log file statistics (JSON) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Multiple | ROS diagnostics (compatible with `rqt_robot_monitor`) |

## Testing

### Test E-Stop

Trigger software e-stop:

```bash
# Activate e-stop
ros2 topic pub /safety/estop_request std_msgs/Bool "{data: true}" --once

# Verify robot stops (cmd_vel_safe should be all zeros)
ros2 topic echo /cmd_vel_safe

# Release e-stop
ros2 topic pub /safety/estop_request std_msgs/Bool "{data: false}" --once
```

### Monitor Safety Status

```bash
# Full safety state
ros2 topic echo /safety/status

# Current speed scale
ros2 topic echo /safety/speed_scale

# System health
ros2 topic echo /system/health_summary

# Component status
ros2 topic echo /system/component_status

# View diagnostics in ROS 2 GUI
rqt_robot_monitor
```

### Simulate Distance Changes

Publish fake LIDAR data to test zone transitions:

```bash
# Publish max range (all clear)
ros2 publish sensor_msgs/LaserScan /scan --once \
  '{header: {frame_id: lidar}, angle_min: -3.14, angle_max: 3.14, angle_increment: 0.01, range_min: 0.1, range_max: 10.0, ranges: [5.0, 5.0, 5.0, ...]}'

# Publish close distance (emergency zone)
ros2 publish sensor_msgs/LaserScan /scan --once \
  '{header: {frame_id: lidar}, angle_min: -3.14, angle_max: 3.14, angle_increment: 0.01, range_min: 0.1, range_max: 10.0, ranges: [0.15, 0.15, 0.15, ...]}'
```

## Configuration

Edit `config/safety_config.yaml` to customize:

- **Zone distances**: Adjust thresholds for FREE/CAUTION/CRITICAL/EMERGENCY
- **Speed limits**: Change scaling factors per zone
- **Timeouts**: Set sensor and heartbeat timeouts
- **Policy flags**: Enable/disable specific safety checks
- **Logging**: Configure log directory, file size, retention

### Example customization:

```yaml
# Make system more conservative
safety_zones:
  free:
    min_distance: 0.80  # Increased from 0.60
  caution:
    min_distance: 0.50  # Increased from 0.35
    
speed_limits:
  caution_zone: 0.3  # More restrictive: 30% speed
  critical_zone: 0.05  # Very slow: 5% speed
```

## Monitoring Dashboard

The package publishes to `/diagnostics` for integration with ROS 2 tools:

```bash
# Monitor in terminal
ros2 topic echo /diagnostics

# Monitor in GUI
rqt_robot_monitor

# Monitor in rviz2
# Add Diagnostics display (in rviz2: Add → Diagnostics)
```

## Hardware Integration Checklist

✓ Safety supervisor is ready to gate velocity commands
⚠️ **TODO**: Update your `robot_hardware_pkg` to:

1. Subscribe to `/cmd_vel_safe` instead of `/cmd_vel`
2. Publish heartbeat to `/hardware/heartbeat` at ~10 Hz
3. Monitor `/safety/estop` for hardware e-stop control

**Example remapping in hardware launch**:

```python
Node(
    package='robot_hardware_pkg',
    executable='hardware_interface_node',
    remappings=[
        ('cmd_vel', '/cmd_vel_safe'),  # ← Subscribe to gated command
    ],
    ...
)
```

## Performance

- **Safety loop**: 20 Hz (50 ms latency)
- **Diagnostics**: 1 Hz
- **LIDAR processing**: Real-time with incoming scans (~10 Hz typical)
- **Memory**: ~50 MB (Python nodes)

## Troubleshooting

### System enters DEGRADED state

```bash
# Check what's timing out
ros2 topic echo /system/health

# Verify LIDAR publishing
ros2 topic echo /scan

# Check heartbeat sources
ros2 topic list | grep heartbeat
```

### Speed is stuck at 0

```bash
# Check e-stop state
ros2 topic echo /safety/estop

# Check current zone
ros2 topic echo /safety/status | jq .current_zone

# Check system health
ros2 topic echo /system/health_summary
```

### Logs not being recorded

```bash
# Check logger is running
ros2 node list | grep logger

# Check log directory
ls -lah ~/.ros/safety_logs/

# Check logger stats
ros2 topic echo /logging/stats
```

## Next Steps

- **Step 6**: `battery_manager_pkg` — Battery monitoring and automatic dock-return
- **Step 7**: `motion_controller_pkg` — Trajectory tracking and motion planning

## License

Apache 2.0

## Author

Chaitanya
