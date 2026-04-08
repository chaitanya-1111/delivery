# speech_to_text_pkg

**Build type:** `ament_python`

**Speech-to-text** node for the delivery robot. Subscribes to microphone chunks on `/audio/data` (placeholder callback) and exposes a **keyboard thread** so you can type utterances in the terminal—especially useful on **WSL** without a working mic pipeline.

## Executable

```bash
ros2 run speech_to_text_pkg stt_node
```

**Node name:** `stt_node`

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/audio/data` | `std_msgs/Int16MultiArray` |
| Publish | `/audio/stt_text` | `std_msgs/String` |

## Behavior

- **Keyboard:** Non-blocking read on stdin; each line published as `String` on `/audio/stt_text`.
- **Audio callback:** Currently no-op (reserved for streaming STT).

## Dependencies

`rclpy`, `robot_interfaces` (declared in `package.xml`; may be for future `SpeechText` use), `SpeechRecognition` (import in node).

## Related

Feeds **`intent_classifier_pkg`**, which listens on `/audio/stt_text`.
