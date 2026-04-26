"""
mission_bringup.launch.py

Launches the complete mission management stack:
  1. mission_manager_node  — delivery FSM brain
  2. order_queue_node      — order intake, queuing, dispatch
  3. mission_logger_node   — event logging to disk

PREREQUISITES (must be running before this launch):
  - robot_lidar_pkg:       lidar_bringup.launch.py
  - robot_slam_pkg:        slam_localization.launch.py
  - robot_navigation_pkg:  navigation.launch.py
  - robot_hardware_pkg:    hardware_bringup.launch.py
  - robot_bringup_pkg:     perception.launch.py (for speech/TTS)

USAGE:
  ros2 launch mission_manager_pkg mission_bringup.launch.py

  # Send a test order (table_3):
  ros2 topic pub /order/manual std_msgs/String \
    "{data: '{\"order_id\": \"test_1\", \"table\": \"table_3\", \"items\": [\"burger\"]}'}" \
    --once

  # Confirm food loaded (press kitchen button):
  ros2 topic pub /mission/load_confirm std_msgs/Bool "{data: true}" --once

  # Confirm pickup (press table button):
  ros2 topic pub /mission/pickup_confirm std_msgs/Bool "{data: true}" --once

  # Cancel active mission:
  ros2 topic pub /mission/cancel std_msgs/Bool "{data: true}" --once

  # Monitor mission state:
  ros2 topic echo /mission/state
  ros2 topic echo /mission/stats
  ros2 topic echo /order/queue_status
"""

import os
from ament_python import LaunchDescription
from ament_python.actions import Node, DeclareLaunchArgument, LogInfo
from ament_python.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('mission_manager_pkg')
    config_file = os.path.join(pkg, 'config', 'mission_config.yaml')

    log_dir_default = os.path.expanduser('~/delivery_bot_ws/logs/missions')
    queue_file_default = os.path.expanduser('~/delivery_bot_ws/logs/order_queue.json')

    # ── Arguments ─────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    log_dir_arg = DeclareLaunchArgument(
        'log_dir', default_value=log_dir_default)

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_dir      = LaunchConfiguration('log_dir')

    # ── NODE 1: Mission Manager (the FSM brain) ────────────────
    mission_manager_node = Node(
        package='mission_manager_pkg',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # ── NODE 2: Order Queue ────────────────────────────────────
    order_queue_node = Node(
        package='mission_manager_pkg',
        executable='order_queue_node',
        name='order_queue_node',
        output='screen',
        parameters=[{
            'use_sim_time'     : use_sim_time,
            'max_queue_size'   : 10,
            'max_order_age_sec': 3600.0,
            'persist_queue'    : True,
            'queue_file'       : queue_file_default,
        }]
    )

    # ── NODE 3: Mission Logger ─────────────────────────────────
    mission_logger_node = Node(
        package='mission_manager_pkg',
        executable='mission_logger_node',
        name='mission_logger_node',
        output='screen',
        parameters=[{
            'use_sim_time'      : use_sim_time,
            'log_directory'     : log_dir,
            'retention_days'    : 30,
            'flush_interval_sec': 5.0,
        }]
    )

    startup_msg = LogInfo(msg=[
        '\n',
        '╔══════════════════════════════════════════════════╗\n',
        '║   MISSION MANAGER STACK STARTING                 ║\n',
        '╠══════════════════════════════════════════════════╣\n',
        '║  Nodes:                                          ║\n',
        '║    mission_manager_node  — delivery FSM          ║\n',
        '║    order_queue_node      — order intake/queue    ║\n',
        '║    mission_logger_node   — event logging         ║\n',
        '╠══════════════════════════════════════════════════╣\n',
        '║  Test order:                                     ║\n',
        '║    ros2 topic pub /order/manual std_msgs/String  ║\n',
        '║      "{data: \'{"order_id":"1","table":"table_3", ║\n',
        '║               "items":["burger"]}\'}" --once      ║\n',
        '╠══════════════════════════════════════════════════╣\n',
        '║  Monitor:                                        ║\n',
        '║    ros2 topic echo /mission/state                ║\n',
        '╚══════════════════════════════════════════════════╝\n',
    ])

    return LaunchDescription([
        use_sim_time_arg,
        log_dir_arg,
        startup_msg,
        mission_manager_node,
        order_queue_node,
        mission_logger_node,
    ])
