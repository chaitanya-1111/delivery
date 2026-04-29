# mission_manager_pkg

**Build type:** `ament_python`

Mission orchestration package for delivery workflows: order intake, queue handling, mission state machine, and mission logging.

## Executables

| Command | Module | Purpose |
|---------|--------|---------|
| `mission_manager_node` | `mission_manager_node.py` | Main delivery mission FSM |
| `order_queue_node` | `order_queue_node.py` | Accepts/orders/dispatches queue |
| `mission_logger_node` | `mission_logger_node.py` | Persists mission events and stats |

## Launch

```bash
ros2 launch mission_manager_pkg mission_bringup.launch.py
```

### Common mission topics

- `/order/manual` - manual order injection (JSON string payload)
- `/mission/load_confirm` - kitchen load confirm
- `/mission/pickup_confirm` - table pickup confirm
- `/mission/cancel` - cancel active mission
- `/mission/state` - mission state output
- `/order/queue_status` - queue status output

## Notes

Start this package only after navigation, localization, lidar, and hardware nodes are running and healthy.
