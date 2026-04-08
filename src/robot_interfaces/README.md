# robot_interfaces

**Build type:** `ament_cmake` (ROS IDL / `rosidl`)

Custom message definitions for the delivery robot conversational and perception pipeline. Other packages depend on this package and must be built after it (or in one `colcon build` invocation).

## Messages

### `Intent.msg`

Structured output from the intent classifier (NLU).

| Field | Type | Notes |
|-------|------|--------|
| `session_id` | `string` | Session identifier |
| `intent_type` | `string` | e.g. `GREET`, `ORDER`, `HELP`, `CANCEL`, `GOODBYE`, `UNKNOWN` (see `intent_classifier_pkg` for actual strings) |
| `confidence` | `float32` | Classifier confidence |
| `raw_text` | `string` | Original user text |

### `AIRequest.msg`

Session manager → AI dialog: what to answer and in which mode.

| Field | Type | Notes |
|-------|------|--------|
| `session_id` | `string` | |
| `mode` | `string` | e.g. `GREETING`, `TALKING`, `DELIVERY`, `GOODBYE` |
| `user_text` | `string` | May be empty for proactive prompts |
| `persona` | `string` | e.g. friendly, formal, playful |

### `AIResponse.msg`

AI dialog → session manager: spoken reply and session hints.

| Field | Type | Notes |
|-------|------|--------|
| `text` | `string` | Text for TTS |
| `emotion` | `string` | Reserved (e.g. for animation) |
| `end_session` | `bool` | Whether the AI considers the conversation finished |

### `SpeechText.msg`

Reserved / optional STT-style message (fields: `text`, `confidence`, `is_final`). The current stack often uses `std_msgs/String` on `/audio/stt_text` instead; this message is available for future alignment.

### `SessionState.msg`

Reserved for richer session reporting (`state`, `person_present`, `person_id`, `conversation_turn`, `speaking`).

### `FaceBox.msg`

2D bounding box for a detected face.

| Field | Type |
|-------|------|
| `x`, `y`, `w`, `h` | `int32` |
| `confidence` | `float32` |

### `FaceTarget.msg`

Normalized target for head/servo control.

| Field | Type | Notes |
|-------|------|--------|
| `cx`, `cy` | `int32` | Pixel center |
| `norm_x`, `norm_y` | `float32` | Roughly −1…+1 from image center |
| `face_found` | `bool` | |

## Usage in code

**Python**

```python
from robot_interfaces.msg import Intent, AIRequest, AIResponse, FaceBox, FaceTarget
```

**C++**

```cpp
#include "robot_interfaces/msg/intent.hpp"
```

Regenerate by rebuilding this package if you edit `.msg` files.

## Files

- `msg/*.msg` — interface definitions
- `CMakeLists.txt` — `rosidl_generate_interfaces`

## Dependencies

Declared in `package.xml`: `ament_cmake`, `rosidl_default_generators`, runtime via `rosidl_default_runtime`.
