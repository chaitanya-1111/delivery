# tts_player_pkg

**Build type:** `ament_python`

**Text-to-speech** using **pyttsx3** when the engine initializes; otherwise **simulation mode** (sleep based on text length) so the session manager still receives **`/audio/playback_done`**.

## Executable

```bash
ros2 run tts_player_pkg tts_node
```

**Node name:** `tts_node`

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/robot/speech` | `std_msgs/String` |
| Publish | `/audio/playback_done` | `std_msgs/Bool` |

## Behavior

- Speaks in a **worker thread** to avoid blocking the ROS executor.
- On engine failure (common in minimal/headless environments), logs a warning and uses timed simulation.

## Dependencies

`rclpy`, `std_msgs`, `pyttsx3`.

## Related

Driven by **`session_manager_pkg`**; completion signal unblocks conversation flow.
