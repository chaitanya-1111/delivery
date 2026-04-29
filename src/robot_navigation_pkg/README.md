# robot_navigation_pkg

**Build type:** `ament_python`

Navigation package that launches Nav2 stack components and bridges mission-level goals to navigation actions.

## Executables

| Command | Module | Purpose |
|---------|--------|---------|
| `nav_client` | `nav_client_node.py` | Sends goals into Nav2 from project mission topics |
| `nav_status` | `nav_status_node.py` | Publishes navigation status/state |
| `cmd_vel_arbiter` | `cmd_vel_arbiter_node.py` | Arbitrates velocity command sources |

## Launch

```bash
ros2 launch robot_navigation_pkg navigation.launch.py map_file:=/absolute/path/to/map.yaml
```

### Common arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `map_file` | package default map path | Map YAML for localization/navigation |
| `params_file` | `config/nav2_params.yaml` | Nav2 configuration |
| `use_rviz` | `true` | Start RViz |
| `use_sim_time` | `false` | Use simulation clock |

## Dependencies

Requires working:

- `/scan` (from lidar package)
- `/odom` and TF `odom -> base_*` (from hardware package)
- Correct map from `robot_slam_pkg`
