# face_detection_pkg

**Build type:** `ament_python`

**Face detection** using OpenCV **FaceDetectorYN (YuNet)** with an **ONNX** model. Consumes camera images and publishes the **largest** face box plus a presence flag.

## Executable

```bash
ros2 run face_detection_pkg face_detector_node
```

**Node name:** `face_detector_node`

## Model file

Expected ONNX name: **`face_detection_yunet_2023mar.onnx`**

Search order (from code):

1. `share/face_detection_pkg/models/` (install space)
2. Fallback path relative to source checkout

Place the model in `face_detection_pkg/models/` before build/install, or ensure it is installed via `setup.py` / `data_files`.

## Topics

| Direction | Topic | Type | QoS |
|-----------|-------|------|-----|
| Subscribe | `/camera/image_raw` | `sensor_msgs/Image` | Best effort, depth 1 |
| Publish | `/face/primary` | `robot_interfaces/FaceBox` | |
| Publish | `/face/present` | `std_msgs/Bool` | `true` if any face detected |

## Parameters / thresholds (in code)

- Confidence and NMS thresholds set in the node constructor.
- Input size for YuNet: 320×320 with dynamic resize from image dimensions.

## Dependencies

`rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge`, `robot_interfaces`, OpenCV with dnn + YuNet support.

## Related

Output feeds **`face_tracking_pkg`** and **`session_manager_pkg`** (via `/face/primary` and `/face/present`).
