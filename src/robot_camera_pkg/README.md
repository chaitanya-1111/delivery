# robot_camera_pkg

**Build type:** `ament_python`

Publishes **USB webcam** images as `sensor_msgs/Image` for the face pipeline or general perception.

## Executable

```bash
ros2 run robot_camera_pkg camera_node
```

**Node name:** `camera_node`

## Topics

| Direction | Topic | Type | QoS notes |
|-----------|-------|------|-----------|
| Publish | `/camera/image_raw` | `sensor_msgs/Image` | Best effort, depth 1 |

## Behavior

- Tries OpenCV `VideoCapture(0)` at **640×480**, ~**30 FPS** timer.
- If the device fails (common in headless/WSL), switches to an internal **test pattern** mode so downstream nodes still receive frames.

## Dependencies

`rclpy`, `sensor_msgs`, `cv_bridge`, OpenCV (`python3-opencv`).

## When to use vs ESP32

- **This package:** local USB camera.
- **`esp32_camera_pkg`:** Wi-Fi stream from ESP32-CAM (used in `perception.launch.py`).

Do not run both publishers on the same topic unless you intentionally remap one.
