# servo_control_pkg

**Build type:** `ament_python`

**Pan/tilt** head control from **`FaceTarget`** messages. Current implementation is **simulation-first**: it updates internal angles and logs commands; hardware PWM/I²C can be added where marked in code.

## Executable

```bash
ros2 run servo_control_pkg servo_node
```

**Node name:** `servo_node`

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/face/target` | `robot_interfaces/FaceTarget` |

## Behavior

- Starts at **90°** pan and tilt (center).
- Proportional step from `norm_x` / `norm_y` with gain **`Kp`**; clamps **0…180°**.
- Ignores updates when `face_found` is false (optional return-to-center can be added).

## Dependencies

`rclpy`, `robot_interfaces`.

## Related

Downstream consumer of **`face_tracking_pkg`**.
