# esp32_camera_pkg

**Build type:** `ament_python`

Reads an **MJPEG/HTTP stream** from an **ESP32-CAM** (or similar) and publishes `sensor_msgs/Image` on `/camera/image_raw`, matching the topic expected by `face_detection_pkg`.

## Executable

```bash
ros2 run esp32_camera_pkg wifi_cam_node
```

**Node name:** `esp32_cam_node`

## Configuration

Edit the stream URL in `esp32_camera_pkg/wifi_cam_node.py`:

```python
self.stream_url = "http://192.168.1.100:81/stream"
```

Use your board’s actual IP and path (ESP32-CAM web servers vary by sketch).

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Publish | `/camera/image_raw` | `sensor_msgs/Image` |

## Behavior

- OpenCV `VideoCapture` on the URL, buffer size 1 to limit latency.
- Timer ~30 Hz; reconnects if the stream drops.

## Dependencies

`rclpy`, `sensor_msgs`, `cv_bridge`, OpenCV.

## Note on `package.xml`

Lists `rclcpp` as a dependency though the node is Python; harmless for build but could be cleaned up later.
