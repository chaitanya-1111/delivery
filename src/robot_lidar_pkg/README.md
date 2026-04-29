# robot_lidar_pkg

**Build type:** `ament_python`

Bringup package for RPLidar with production helpers: static TF publishing, scan diagnostics, and watchdog monitoring.

## Executables

| Command | Module | Purpose |
|---------|--------|---------|
| `lidar_tf_node` | `lidar_tf_node.py` | Publishes `base_link -> laser` static TF (calibration-critical) |
| `lidar_diagnostics_node` | `lidar_diagnostics_node.py` | Scan rate/health diagnostics |
| `scan_watchdog_node` | `scan_watchdog_node.py` | Detects lidar stalls/disconnects and reports status |

Run with `ros2 run robot_lidar_pkg <executable>`.

## Launch

```bash
ros2 launch robot_lidar_pkg lidar_bringup.launch.py
```

### Calibration-related launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `lidar_x` | `0.0` | Lidar offset in meters forward from `base_link` |
| `lidar_y` | `0.0` | Lidar offset in meters left from `base_link` |
| `lidar_z` | `0.18` | Lidar height in meters from `base_link` |
| `lidar_yaw` | `0.0` | Lidar yaw in radians (`3.14159` if mounted backward) |

Example:

```bash
ros2 launch robot_lidar_pkg lidar_bringup.launch.py lidar_x:=0.02 lidar_z:=0.195 lidar_yaw:=0.0
```

## Where to calibrate

- `robot_lidar_pkg/robot_lidar_pkg/lidar_tf_node.py`
  - Main calibration parameters are `x`, `y`, `z`, `roll`, `pitch`, `yaw`.
- `robot_lidar_pkg/launch/lidar_bringup.launch.py`
  - Exposes calibration args so you can tune without editing code each run.

After tuning, verify:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link laser
```
