# face_tracking_pkg

**Build type:** `ament_python`

Turns **face bounding boxes** into a **normalized gaze target** for pan/tilt control. Assumes a **640×480** image geometry for center and normalization (must match your camera pipeline or be parameterized in code).

## Executable

```bash
ros2 run face_tracking_pkg face_tracker_node
```

**Node name:** `face_tracker_node`

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/face/primary` | `robot_interfaces/FaceBox` |
| Publish | `/face/target` | `robot_interfaces/FaceTarget` |
| Publish | `/face/lost` | `std_msgs/Bool` |

## Behavior

- Computes face center from box, maps to **norm_x / norm_y** in roughly **−1…+1** relative to image center.
- If no face updates for **2 seconds** (timer at 10 Hz), can signal loss and centering logic (see full node).

## Dependencies

`rclpy`, `robot_interfaces`, `std_msgs`.

## Related

Subscribes to **`face_detection_pkg`** output; publishes to **`servo_control_pkg`**.
