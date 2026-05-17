# 🤖 Move Your Robot in Gazebo - Quick Start

## Step 1: Launch Gazebo with the Robot

**Open Terminal 1 and run:**
```bash
cd ~/delivery_bot_ws
source install/setup.bash
ros2 launch robot_description_pkg gazebo_bringup.launch.py
```

✅ You should see:
- Gazebo window opening with the robot
- RViz window (optional visualization)

---

## Step 2: Control the Robot

### **Option A: Keyboard Control (Recommended)**

**Open Terminal 2 and run:**
```bash
cd ~/delivery_bot_ws
source install/setup.bash
python3 robot_teleop.py
```

Then use your keyboard:
- **W** → Move forward
- **S** → Move backward  
- **A** → Turn left
- **D** → Turn right
- **Q** → Strafe left (if supported)
- **E** → Strafe right (if supported)
- **SPACE** → Stop
- **X** → Exit

---

### **Option B: Manual Command (Terminal)**

**Open Terminal 2 and run:**
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Turn right (press Ctrl+C then run another command)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'

# Stop (Ctrl+C)
```

---

### **Option C: Monitor Robot Movement**

**Open Terminal 3 to see odometry (position tracking):**
```bash
cd ~/delivery_bot_ws
source install/setup.bash
ros2 topic echo /odom
```

You'll see the robot's position update as it moves!

---

## Summary of Commands

| What | Terminal 1 | Terminal 2 | Terminal 3 |
|------|-----------|-----------|-----------|
| **Launch** | `ros2 launch robot_description_pkg gazebo_bringup.launch.py` | — | — |
| **Control** | — | `python3 robot_teleop.py` | — |
| **Monitor** | — | — | `ros2 topic echo /odom` |

---

## What's Happening?

1. **Gazebo** simulates the physics and sensors
2. **Your commands** go to `/cmd_vel` topic
3. **Gazebo's controller** reads `/cmd_vel` and moves the robot
4. **Odometry** publishes the new position to `/odom`
5. **RViz** visualizes everything

---

## Troubleshooting

### Robot won't move
- Check Terminal 2: Is teleop running?
- Check `/cmd_vel` is being published: `ros2 topic echo /cmd_vel`
- Verify Gazebo simulation is running (not paused)

### Gazebo crashes
```bash
# Clean rebuild
rm -rf ~/delivery_bot_ws/build ~/delivery_bot_ws/install
cd ~/delivery_bot_ws
colcon build --symlink-install
```

### Permission denied
```bash
chmod +x ~/delivery_bot_ws/robot_teleop.py
```

---

## Next: Advanced Control

Once comfortable with basic movement, you can:
- Add **autonomous navigation** (Nav2)
- Add **SLAM mapping** (simultaneous localization and mapping)
- Add **mission execution** (deliver packages)

Let me know what you need! 🚀
