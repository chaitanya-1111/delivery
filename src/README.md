# ROS 2 packages (`src`)

This directory is the **source space** for `delivery_bot_ws`. Each subdirectory is an ament package.

## Build order note

`robot_interfaces` is a **CMake** package that generates Python and C++ message code. Other packages depend on it; after a clean clone, run `colcon build` from the workspace root so interfaces are generated before Python packages that `import robot_interfaces`.

## Package index

| Directory | Type | Summary |
|-----------|------|---------|
| [audio_input_pkg](audio_input_pkg/README.md) | `ament_python` | Microphone input publisher |
| [esp32_camera_pkg](esp32_camera_pkg/README.md) | `ament_python` | ESP32 camera stream publisher |
| [face_detection_pkg](face_detection_pkg/README.md) | `ament_python` | Face detection and face metadata topics |
| [intent_classifier_pkg](intent_classifier_pkg/README.md) | `ament_python` | STT text to intent classification |
| [mission_manager_pkg](mission_manager_pkg/README.md) | `ament_python` | Delivery mission FSM, queue, and mission logging |
| [robot_bringup_pkg](robot_bringup_pkg/README.md) | `ament_python` | High-level launch package for integrated bringup |
| [robot_description_pkg](robot_description_pkg/README.md) | `ament_cmake` | URDF/Xacro robot model, RViz, robot description assets |
| [robot_hardware_pkg](robot_hardware_pkg/README.md) | `ament_python` | `/cmd_vel` to motors + encoder odometry + diagnostics |
| [robot_interfaces](robot_interfaces/README.md) | `ament_cmake` | Custom message definitions used across the stack |
| [robot_lidar_pkg](robot_lidar_pkg/README.md) | `ament_python` | RPLidar bringup, TF mount calibration, watchdog, diagnostics |
| [robot_navigation_pkg](robot_navigation_pkg/README.md) | `ament_python` | Nav2 bringup helpers and navigation bridge nodes |
| [robot_slam_pkg](robot_slam_pkg/README.md) | `ament_python` | SLAM mapping/localization launch and map utilities |
| [safety_supervisor_pkg](safety_supervisor_pkg/README.md) | `ament_python` | Safety monitoring, watchdogs, and safety status reporting |
| [session_manager_pkg](session_manager_pkg/README.md) | `ament_python` | Session state management and conversation orchestration |
| [speech_to_text_pkg](speech_to_text_pkg/README.md) | `ament_python` | Speech-to-text and keyboard test input |
| [tts_player_pkg](tts_player_pkg/README.md) | `ament_python` | Text-to-speech playback node |

## Calibration code locations

If you want to calibrate the robot quickly, these are the main files:

- **Lidar mount (extrinsics):** `robot_lidar_pkg/robot_lidar_pkg/lidar_tf_node.py`
  - Tune `x`, `y`, `z`, `roll`, `pitch`, `yaw` for `base_link -> laser`.
  - Also pass these from launch args in `robot_lidar_pkg/launch/lidar_bringup.launch.py`.
- **Drive and odometry constants:** `robot_hardware_pkg/robot_hardware_pkg/hardware_interface_node.py`
  - Tune `WHEEL_DIAMETER`, `WHEEL_SEPARATION`, `ENCODER_PPR`, `GEAR_RATIO`.
  - These directly affect `/odom`, TF, and path tracking quality.
- **SLAM quality knobs:** `robot_slam_pkg/config/slam_toolbox_mapping.yaml` and `robot_slam_pkg/launch/slam_mapping.launch.py`
  - Use after lidar/odometry calibration is stable.

Recommended order: calibrate lidar TF first, then wheel/encoder odometry, then tune SLAM/Nav2.

You can practice teleop and compare `/odom` behavior in **Gazebo** before hardware; that does not auto-edit these files—you change constants manually after tests. See [GAZEBO_SETUP.md](../GAZEBO_SETUP.md) (simulation is not ML training in this workspace).

## Quick launch

```bash
# From workspace after sourcing install/setup.bash
ros2 launch robot_bringup_pkg perception.launch.py
```

For more context, see the [workspace README](../README.md).
