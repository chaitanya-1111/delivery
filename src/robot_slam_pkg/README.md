# robot_slam_pkg

**Build type:** `ament_python`

SLAM and map lifecycle package for mapping and localization phases.

## Launch files

- `slam_mapping.launch.py`
  - Mapping session: SLAM Toolbox + dynamic obstacle filtering + map manager + quality checks.
- `slam_localization.launch.py`
  - Localization mode against a previously saved map.

## Executables

| Command | Module | Purpose |
|---------|--------|---------|
| `dynamic_obstacle_filter_node` | `dynamic_obstacle_filter_node.py` | Removes moving-object noise before SLAM |
| `map_quality_node` | `map_quality_node.py` | Checks map quality metrics |
| `map_manager_node` | `map_manager_node.py` | Saves/version-controls maps |
| `mapping_session` | `scripts/mapping_session.py` | Mapping helper workflow |

## Usage

```bash
# Build a new map
ros2 launch robot_slam_pkg slam_mapping.launch.py

# Localize on existing map
ros2 launch robot_slam_pkg slam_localization.launch.py
```

## Calibration note

This package assumes lidar TF and wheel odometry are already calibrated.
If maps look warped or drifting, first calibrate:

- `robot_lidar_pkg/robot_lidar_pkg/lidar_tf_node.py`
- `robot_hardware_pkg/robot_hardware_pkg/hardware_interface_node.py`
