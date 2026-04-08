# audio_input_pkg

**Build type:** `ament_python`

Captures **microphone audio** with PyAudio and publishes PCM chunks for downstream STT or logging.

## Executable

```bash
ros2 run audio_input_pkg microphone_node
```

**Node name:** `microphone_node`

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Publish | `/audio/data` | `std_msgs/Int16MultiArray` |

Audio is **mono**, **16 kHz**, **16-bit**, chunked (1024 samples per message in current implementation).

## Behavior

- Opens a real input stream when hardware is available.
- On failure (e.g. **no mic on WSL**), logs a warning and can run a **silent/simulated** timer so the topic still exists for the graph.

## Dependencies

`rclpy`, PyAudio (`python3-pyaudio`), `audio_common_msgs` / `audio_common-msgs` as declared in `package.xml`.

## Related

`speech_to_text_pkg` subscribes to `/audio/data` but currently relies heavily on **keyboard input** for testing; real STT integration would buffer `/audio/data` here.
