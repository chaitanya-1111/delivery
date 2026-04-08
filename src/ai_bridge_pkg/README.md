# ai_bridge_pkg

**Build type:** `ament_python`

**Placeholder package** for a future **AI bridge** (e.g. external LLM API, cloud STT, or non-ROS process integration). There are **no console scripts** registered in `setup.py` and no Python modules beyond the standard ament Python package scaffold.

## Current status

- `package.xml` / `setup.py` declare dependencies on `robot_interfaces` and `std_msgs` in preparation for future nodes.
- No launch files or nodes are shipped yet.

## Intended use (future)

When implemented, this package might:

- Host a node that translates between REST/WebSocket AI backends and ROS topics (`/ai/request`, `/ai/response`, or parallel services).
- Centralize API keys and rate limiting away from `ai_dialog_pkg`.

## Build

```bash
colcon build --packages-select ai_bridge_pkg
```

## Related

See **`ai_dialog_pkg`** for the current in-process dialog implementation.
