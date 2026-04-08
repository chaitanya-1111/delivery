# intent_classifier_pkg

**Build type:** `ament_python`

**Rule-based intent classifier** (“production v1”): maps free text to a compact **`intent_type`** string and confidence, packaged as `robot_interfaces/Intent`.

## Executable

```bash
ros2 run intent_classifier_pkg classifier_node
```

**Node name:** `intent_classifier_node`

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/audio/stt_text` | `std_msgs/String` |
| Publish | `/nlu/intent` | `robot_interfaces/Intent` |

## Intent labels (from `classifier_node.py`)

| `intent_type` | Trigger idea |
|---------------|----------------|
| `greeting` | hello, hi, hey, morning, … |
| `goodbye` | bye, goodbye, see you, … |
| `confirm` | yes, yeah, correct, sure, … |
| `deny` | no, wrong, deny, … |
| `check_order` | order, package, delivery, pizza, … |
| `query_identity` | name, who are you, … |
| `emergency_stop` | stop, halt, freeze |
| `unknown` | default |

`Intent.session_id` may be set to a placeholder (e.g. `"current"`) until a proper ID service exists.

## Dependencies

`rclpy`, `std_msgs`, `robot_interfaces`.

## Related

Consumes **`speech_to_text_pkg`**; consumed by **`session_manager_pkg`**.
