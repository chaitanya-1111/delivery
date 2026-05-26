# Gazebo Simulation - Quick Start Guide

## Why use Gazebo?

Gazebo is a **digital test bench** for this delivery robot—not a machine-learning trainer.

| What people sometimes call "train" | In this workspace |
|-----------------------------------|-------------------|
| ML / reinforcement learning (neural net learns in sim) | **No** — no gym, PyTorch, or training scripts |
| Test teleop, Nav2, SLAM safely without hardware | **Yes** |
| Calibrate wheels, lidar mount, Nav2, SLAM | **Yes** — you edit config files manually after observing behavior |

Reasons to use simulation here:

1. **No risk to hardware** — bad `/cmd_vel` or Nav2 goals cannot damage motors or furniture.
2. **Same ROS interfaces as the real robot** — Gazebo diff-drive uses `/cmd_vel` and `/odom` (see `src/robot_description_pkg/gazebo/gazebo_plugins.xacro`), matching `robot_hardware_pkg`.
3. **Validate URDF and sensors** — meshes, wheel separation, LiDAR pose, and plugin topics before deploy.
4. **Develop navigation faster** — optional SLAM/Nav2 against simulated `/scan` and `/odom`.
5. **Repeatable tests** — fixed spawn via `gazebo_bringup.launch.py` args (`x`, `y`, `yaw`).

Related guides: [ROBOT_MOVEMENT_GUIDE.md](ROBOT_MOVEMENT_GUIDE.md) (teleop), [src/README.md](src/README.md) (calibration file locations).

---

## What changes in your files?

**Default: nothing in `src/` changes automatically.** Launching Gazebo only starts processes and publishes topics in memory.

| Activity in Gazebo | Files that might change | Who changes them |
|--------------------|-------------------------|------------------|
| Drive with teleop / `ros2 topic pub` | None | — |
| Save a ROS bag | New `.db3` bag folder (often outside `src/`) | You run `ros2 bag record` |
| Build a map with SLAM | New `map.yaml` + `map.pgm` | SLAM toolbox save |
| Tune sim physics | `src/robot_description_pkg/gazebo/gazebo_plugins.xacro` | You edit manually |
| Tune real odometry | `src/robot_hardware_pkg/robot_hardware_pkg/hardware_interface_node.py` | You edit manually |
| Tune lidar mount | `src/robot_lidar_pkg/robot_lidar_pkg/lidar_tf_node.py` or launch args | You edit manually |
| Tune Nav2 | `src/robot_navigation_pkg/config/nav2_params.yaml` | You edit manually |
| Tune SLAM | `src/robot_slam_pkg/config/slam_toolbox_mapping.yaml` | You edit manually |
| Change robot shape | `src/robot_description_pkg/urdf/*.xacro`, `meshes/*.stl` | You edit + `colcon build` |

**Simulation setup files** (do not auto-update when you drive the robot):

- `src/robot_description_pkg/launch/gazebo_bringup.launch.py`
- `src/robot_description_pkg/gazebo/gazebo_plugins.xacro`
- `gazebo_launch.sh`, `robot_teleop.py`

**Runtime-only** (not source): `~/.gazebo/` cache, `build/`, `install/`, `log/`, optional bags/maps.

**ML training in Gazebo** is not part of this repo. Adding it would mean new packages (e.g. gym + ROS bridge) and checkpoint files—not automatic edits to existing navigation/URDF files.

---

## Build & Setup

### 1. Build the workspace
```bash
cd ~/delivery_bot_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Gazebo with the robot

**Option A: Full simulation (Gazebo + RViz)**
```bash
ros2 launch robot_description_pkg gazebo_bringup.launch.py
```

**Option B: Gazebo only (no GUI)**
```bash
ros2 launch robot_description_pkg gazebo_bringup.launch.py gui:=false rviz:=false
```

**Option C: Custom robot starting position**
```bash
ros2 launch robot_description_pkg gazebo_bringup.launch.py \
    x:=1.0 y:=2.0 z:=0.0 yaw:=1.57
```

---

## Control the Robot

### 1. Send velocity commands (manually)
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

**Explanation:**
- `linear.x`: forward/backward speed (m/s)
- `angular.z`: rotation speed (rad/s)

### 2. Use keyboard teleop (if available)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Check available topics
```bash
ros2 topic list
```

---

## Important Topics & Services

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Input | Velocity commands (what you send) |
| `/odom` | nav_msgs/Odometry | Output | Robot odometry from Gazebo |
| `/clock` | rosgraph_msgs/Clock | Output | Simulation time |
| `/scan` | sensor_msgs/LaserScan | Output | LiDAR data |
| `/camera/depth/image_raw` | sensor_msgs/Image | Output | Depth camera |

---

## Verify Gazebo Integration

### Check if robot is receiving commands
**Terminal 1:** Monitor odometry
```bash
ros2 topic echo /odom
```

**Terminal 2:** Send a command
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

You should see the `/odom` values updating!

---

## Troubleshooting

### Gazebo won't start
```bash
# Check if gazebo is installed
which gzserver gzclient

# If not installed:
sudo apt-get install gazebo gazebo-ros gazebo-ros-pkgs gazebo-ros-plugins-pkgs
```

### spawn_entity.py not found
```bash
sudo apt-get install ros-humble-gazebo-ros
# or
sudo apt-get install ros-iron-gazebo-ros
```

### Robot not moving when I send /cmd_vel
- Check that Gazebo simulator is running
- Verify `/cmd_vel` is being published: `ros2 topic echo /cmd_vel`
- Check Gazebo console for errors

---

## Next steps

- **Keyboard control:** see [ROBOT_MOVEMENT_GUIDE.md](ROBOT_MOVEMENT_GUIDE.md) (`python3 robot_teleop.py`).
- **Autonomous navigation:** after a saved map, use `robot_navigation_pkg` launches (see that package README).
- **Calibration on hardware:** follow the order in [src/README.md](src/README.md) (lidar TF → wheel odometry → SLAM/Nav2).
- **MoveIt! / manipulator planning:** not included; this robot is differential-drive mobile base only.
