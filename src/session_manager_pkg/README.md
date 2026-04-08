# session_manager_pkg

**Build type:** `ament_python`

**Central session / conversation state machine** (“brain”): ties together **face presence**, **NLU intents**, **AI dialog**, and **TTS**. Publishes lines for the robot to speak and AI requests for the dialog node.

## Executable

```bash
ros2 run session_manager_pkg session_node
```

**Node name:** `session_manager`

## Topics

### Subscriptions

| Topic | Type | Role |
|-------|------|------|
| `/face/primary` | `robot_interfaces/FaceBox` | Track locked face |
| `/face/present` | `std_msgs/Bool` | Person presence |
| `/nlu/intent` | `robot_interfaces/Intent` | Classified user intent |
| `/ai/response` | `robot_interfaces/AIResponse` | Reply text + session hints |
| `/audio/playback_done` | `std_msgs/Bool` | TTS finished |

### Publications

| Topic | Type | Role |
|-------|------|------|
| `/ai/request` | `robot_interfaces/AIRequest` | Ask dialog node |
| `/robot/speech` | `std_msgs/String` | Text for TTS |
| `/session/state` | `std_msgs/String` | High-level state string |

## Parameters

Declared on the node (ROS parameters):

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `timeout_limit` | `8.0` | Seconds-related timeout |
| `confirm_time` | `1.0` | Confirmation timing |
| `control_rate` | `10.0` | Control loop rate (Hz) |
| `goodbye_delay` | `3.0` | Delay after goodbye before reset |

## State machine (conceptual)

Enum in code: `IDLE`, `HUMAN_DETECTED`, `GREETING`, `TALKING`, `GOODBYE`. Transitions depend on face callbacks, intents, AI responses, and audio-done events.

## Dependencies

`rclpy`, `std_msgs`, `robot_interfaces`.

## Related

Orchestrates **`ai_dialog_pkg`**, **`tts_player_pkg`**, **`face_detection_pkg`**, **`intent_classifier_pkg`**.
