# ROS 2 packages (`src`)

This directory is the **source space** for `delivery_bot_ws`. Each subdirectory is an ament package.

## Build order note

`robot_interfaces` is a **CMake** package that generates Python and C++ message code. Other packages depend on it; after a clean clone, run `colcon build` from the workspace root so interfaces are generated before Python packages that `import robot_interfaces`.

## Package index

| Directory | Type | Summary |
|-----------|------|---------|
| [robot_interfaces](robot_interfaces/README.md) | `ament_cmake` | Custom `.msg` types: `Intent`, `AIRequest`, `AIResponse`, `SpeechText`, `SessionState`, `FaceBox`, `FaceTarget` |
| [robot_description_pkg](robot_description_pkg/README.md) | `ament_cmake` | URDF/Xacro, meshes, RViz, `view_robot.launch.py` |
| [robot_hardware_pkg](robot_hardware_pkg/README.md) | `ament_python` | Differential drive interface, odometry, optional mock Arduino |
| [robot_bringup_pkg](robot_bringup_pkg/README.md) | `ament_python` | High-level launch: perception + dialog stack |
| [robot_camera_pkg](robot_camera_pkg/README.md) | `ament_python` | USB camera → `sensor_msgs/Image` |
| [esp32_camera_pkg](esp32_camera_pkg/README.md) | `ament_python` | ESP32-CAM HTTP stream → `sensor_msgs/Image` |
| [audio_input_pkg](audio_input_pkg/README.md) | `ament_python` | PyAudio microphone → `std_msgs/Int16MultiArray` |
| [face_detection_pkg](face_detection_pkg/README.md) | `ament_python` | OpenCV YuNet → `FaceBox`, face presence |
| [face_tracking_pkg](face_tracking_pkg/README.md) | `ament_python` | Largest/primary face → `FaceTarget` |
| [servo_control_pkg](servo_control_pkg/README.md) | `ament_python` | `FaceTarget` → pan/tilt (simulation default) |
| [speech_to_text_pkg](speech_to_text_pkg/README.md) | `ament_python` | Audio subscription + keyboard “speech” for testing |
| [intent_classifier_pkg](intent_classifier_pkg/README.md) | `ament_python` | Text → `Intent` on `/nlu/intent` |
| [session_manager_pkg](session_manager_pkg/README.md) | `ament_python` | Session FSM, bridges vision, NLU, AI, TTS |
| [ai_dialog_pkg](ai_dialog_pkg/README.md) | `ament_python` | `/ai/request` → `/ai/response` (rule-based) |
| [tts_player_pkg](tts_player_pkg/README.md) | `ament_python` | `/robot/speech` → speech or timed simulation |
| [ai_bridge_pkg](ai_bridge_pkg/README.md) | `ament_python` | Reserved / future bridge (no executables) |

## Quick launch

```bash
# From workspace after sourcing install/setup.bash
ros2 launch robot_bringup_pkg perception.launch.py
```

For more context, see the [workspace README](../README.md).
