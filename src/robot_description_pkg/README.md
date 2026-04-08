# robot_description_pkg

**Build type:** `ament_cmake`

URDF/Xacro model, meshes, and RViz configuration for the **autonomous food delivery robot**. Used by `robot_state_publisher` and optional visualization.

## Contents

- **`urdf/`** — `robot.urdf.xacro` (main model)
- **`launch/view_robot.launch.py`** — Standalone visualization pipeline
- **`config/`** — e.g. `robot_view.rviz`
- Additional assets (meshes, etc.) as included by the xacro

## Launch: view robot

```bash
ros2 launch robot_description_pkg view_robot.launch.py
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `gui` | `true` | Use `joint_state_publisher_gui` (sliders); if `false`, uses non-GUI `joint_state_publisher` |
| `rviz` | `true` | Start RViz2 |
| `rviz_config` | package `config/robot_view.rviz` | Path to `.rviz` file |
| `model` | package `urdf/robot.urdf.xacro` | Xacro path |
| `use_sim_time` | `false` | Set `true` when driven by Gazebo or bag `/clock` |

### Nodes started

- `robot_state_publisher` — TF from URDF + `/joint_states`
- `joint_state_publisher` or `joint_state_publisher_gui`
- `rviz2` (optional)

## Integration with hardware

`robot_hardware_pkg` launch loads the same xacro for TF consistency. Keep frame names (e.g. `base_footprint`, `odom`) aligned between URDF and hardware node parameters.

## Dependencies

`robot_state_publisher`, `joint_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xacro`, `launch`, `launch_ros`.

## License

MIT (per `package.xml`).
