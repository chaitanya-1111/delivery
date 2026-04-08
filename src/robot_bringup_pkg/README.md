# robot_bringup_pkg

**Build type:** `ament_python`

High-level **launch-only** package: starts the integrated **perception + conversation** stack used for interactive delivery-robot demos.

## Launch file

### `launch/perception.launch.py`

Starts these nodes in order:

1. `audio_input_pkg/microphone_node` — `/audio/data`
2. `esp32_camera_pkg/wifi_cam_node` — `/camera/image_raw` (ESP32 stream)
3. `face_detection_pkg/face_detector_node` — YuNet detection
4. `face_tracking_pkg/face_tracker_node` — face target for servos
5. `servo_control_pkg/servo_node` — pan/tilt (simulation-friendly)
6. `session_manager_pkg/session_node` — session FSM + AI/TTS orchestration
7. `tts_player_pkg/tts_node` — speech from `/robot/speech`
8. `speech_to_text_pkg/stt_node` — STT + keyboard override → `/audio/stt_text`
9. `ai_dialog_pkg/ai_node` — `/ai/request` ↔ `/ai/response`
10. `intent_classifier_pkg/classifier_node` — `/audio/stt_text` → `/nlu/intent`

## Usage

```bash
source install/setup.bash
ros2 launch robot_bringup_pkg perception.launch.py
```

Ensure ESP32 URL, microphone, and ONNX model paths are valid for your machine (see individual package READMEs). On WSL, many nodes degrade gracefully to simulation.

## Executables

None registered in `setup.py`; this package is intended for **launch** and share resources only.

## Dependencies

Implicitly requires all launched packages to be installed in the same workspace overlay.
