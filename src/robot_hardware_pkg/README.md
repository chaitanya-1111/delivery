# robot_hardware_pkg

**Build type:** `ament_python`

Bridges **ROS 2 navigation-style commands** to **on-robot hardware** (Arduino + encoders): differential drive from `/cmd_vel`, wheel odometry on `/odom`, TF, and `/joint_states`. Supports **mock mode** with no serial device for software-only testing.

## Executables

| Command | Module | Purpose |
|---------|--------|---------|
| `hardware_interface_node` | `hardware_interface_node.py` | Main bridge: Twist, serial, odometry, TF |
| `mock_arduino` | `mock_arduino.py` | Standalone mock serial peer (development) |

Run with `ros2 run robot_hardware_pkg <executable>`.

## Launch

```bash
ros2 launch robot_hardware_pkg hardware_bringup.launch.py
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `mock_mode` | `false` | No hardware; simulated motion |
| `serial_port` | `/dev/ttyUSB0` | Arduino device |
| `serial_baud` | `115200` | Serial speed |
| `pwm_deadband` | `40` | Motor PWM deadband |
| `use_rviz` | `false` | Start RViz2 |
| `base_frame_id` | `base_footprint` | Must match URDF |
| `odom_frame_id` | `odom` | Odometry frame |

## Behavior summary

- **Commands:** Subscribes to `/cmd_vel` (`geometry_msgs/Twist`), converts to wheel speeds / PWM, sends to Arduino over serial.
- **Feedback:** Reads encoder ticks, publishes `nav_msgs/Odometry` and TF (see node source for exact topic names and frame IDs).
- **Mock mode:** Publishes plausible odometry/TF without a physical robot.

Physical constants (wheel diameter, separation, encoder counts) are defined in `hardware_interface_node.py` — tune to match your mechanics.

## Dependencies

`rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`, `pyserial` (runtime), plus `robot_description_pkg` share path for URDF in launch.

## Related

Use together with Nav2 or teleop once `/cmd_vel` and `/odom` are stable. For visualization only, use `robot_description_pkg/view_robot.launch.py`.
