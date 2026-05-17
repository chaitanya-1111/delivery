# Gazebo Simulation - Quick Start Guide

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

## Next Steps: Regarding "movlet"

Please clarify what you mean by "movlet":
- **MoveIt!** - Motion planning for manipulators/arms? (requires separate setup)
- **Custom teleop tool** - Want me to create a simple GUI controller?
- **Something else** - Let me know!

Once clarified, I can help you integrate it with the Gazebo simulation.
