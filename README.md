# delivery_bot_ws

ROS 2 workspace for an autonomous **delivery robot** with perception (camera, face detection/tracking), conversational AI pipeline (STT → intent → session → dialog → TTS), optional ESP32 camera streaming, URDF visualization, and a hardware bridge for differential drive and odometry.

## Prerequisites

- **ROS 2** (Humble or Iron are typical; adjust distro name in commands if yours differs)
- **Python 3** with packages used by nodes (e.g. `opencv-python`, `pyaudio`, `pyttsx3`, `SpeechRecognition`, `pyserial`)
- **Colcon** build tools

System packages often include: `python3-opencv`, `ros-humble-cv-bridge`, `ros-humble-sensor-msgs`, etc., matching your ROS distribution.

## Workspace layout

| Path | Purpose |
|------|---------|
| `src/` | All ROS 2 packages (see [src/README.md](src/README.md)) |
| `build/`, `install/`, `log/` | Created by `colcon build` (not always in version control) |

## Build

From the workspace root (`delivery_bot_ws`):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/delivery_bot_ws   # or your clone path
colcon build --symlink-install
source install/setup.bash
```

Build `robot_interfaces` first if you see missing-interface errors (colcon usually orders dependencies correctly).

## Run

### Full perception + conversation stack

Starts microphone, ESP32 camera, face pipeline, servos (simulated), session manager, TTS, STT, AI dialog, and intent classifier:

```bash
source install/setup.bash
ros2 launch robot_bringup_pkg perception.launch.py
```

### Robot model in RViz

```bash
ros2 launch robot_description_pkg view_robot.launch.py
```

Optional: `gui:=false`, `rviz:=false`, `use_sim_time:=true` for headless or simulation clock.

### Hardware: motors, odometry, TF

```bash
ros2 launch robot_hardware_pkg hardware_bringup.launch.py
```

Common arguments: `mock_mode:=true`, `serial_port:=/dev/ttyACM0`, `use_rviz:=true`. See that package’s README for details.

## Architecture (high level)

1. **Vision**: `/camera/image_raw` → face detector → `/face/primary`, `/face/present` → tracker → `/face/target` → servo node (pan/tilt; simulation logs on WSL).
2. **Audio in**: `microphone_node` publishes `/audio/data`; STT also accepts **keyboard lines** as fake speech for testing.
3. **NLU**: `/audio/stt_text` → intent classifier → `/nlu/intent` (`robot_interfaces/Intent`).
4. **Session + AI**: Session manager drives state machine, publishes `/robot/speech` and `/ai/request`, listens to `/ai/response` and intents.
5. **Speech out**: TTS subscribes to `/robot/speech`, publishes `/audio/playback_done` when finished.

Custom message definitions live in **`robot_interfaces`**.

## Packages

| Package | Role |
|---------|------|
| `robot_interfaces` | `.msg` definitions shared across the stack |
| `robot_description_pkg` | URDF/Xacro, RViz config, `view_robot` launch |
| `robot_hardware_pkg` | `cmd_vel` ↔ Arduino, odometry, TF, mock mode |
| `robot_bringup_pkg` | `perception.launch.py` — integrated stack |
| `robot_camera_pkg` | USB OpenCV camera → `/camera/image_raw` |
| `esp32_camera_pkg` | MJPEG stream → `/camera/image_raw` |
| `audio_input_pkg` | Microphone → `/audio/data` |
| `face_detection_pkg` | YuNet ONNX face detection |
| `face_tracking_pkg` | Face centroid → normalized target |
| `servo_control_pkg` | Face target → pan/tilt (simulation by default) |
| `speech_to_text_pkg` | STT stub + keyboard override |
| `intent_classifier_pkg` | Rule-based intents from text |
| `session_manager_pkg` | Conversation state machine |
| `ai_dialog_pkg` | Rule-based AI responses (replaceable with LLM) |
| `tts_player_pkg` | pyttsx3 or simulated TTS |
| `ai_bridge_pkg` | Placeholder package (no nodes yet) |

Each package under `src/` has its own **README.md** with topics, executables, and dependencies.

## Troubleshooting

- **WSL**: Microphone and TTS often fall back to simulation; use STT keyboard input to drive the dialog pipeline.
- **ESP32 camera**: Set the stream URL inside `esp32_camera_pkg` (default IP in source is a placeholder).
- **Face model**: YuNet ONNX must be installed under `face_detection_pkg` share or source tree (see that package’s README).

## License

Package licenses vary (`TODO` in several `package.xml` files); see individual packages. `robot_description_pkg` declares MIT.
