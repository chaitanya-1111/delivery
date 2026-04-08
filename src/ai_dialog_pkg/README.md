# ai_dialog_pkg

**Build type:** `ament_python`

**AI dialog node** in **rule-based** mode: receives structured **`AIRequest`** messages and returns **`AIResponse`** text. Designed so `generate_response()` can later call an **LLM API** without changing the topic contract.

## Executable

```bash
ros2 run ai_dialog_pkg ai_node
```

**ROS node name:** `ai_dialog_node`

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/ai/request` | `robot_interfaces/AIRequest` |
| Publish | `/ai/response` | `robot_interfaces/AIResponse` |

## Modes (`AIRequest.mode`)

Handled strings include:

- **`GREETING`** — Opening delivery prompt
- **`DELIVERY`** — Verification / order handoff wording
- **`GOODBYE`** — Farewell
- **`TALKING`** — Small talk / keyword replies (name, weather, delivery keywords)

`persona` and `session_id` are available on the request for future personalization.

## Dependencies

`rclpy`, `std_msgs`, `robot_interfaces`.

## Related

Peer of **`session_manager_pkg`** on the `/ai/*` topics.
