"""
slam_mapping.launch.py

PURPOSE:
  Run this ONE TIME to build your restaurant map.
  Drive the robot to every table, along every wall, through every doorway.
  When done, save the map with: ros2 service call /slam/save_map std_srvs/srv/Trigger

WHAT IT STARTS:
  1. slam_toolbox (async mapping)   → builds map from /scan_for_slam
  2. dynamic_obstacle_filter_node   → removes people from scan before SLAM
  3. map_manager_node               → auto-saves, versioning, registry
  4. map_quality_node               → monitors map quality in real-time
  5. rviz2 (optional)               → live visualization for operator

TYPICAL MAPPING SESSION (30-60 min for a restaurant):
  Terminal 1: ros2 launch robot_lidar_pkg lidar_bringup.launch.py use_rviz:=false
  Terminal 2: ros2 launch robot_hardware_pkg hardware_bringup.launch.py mock_mode:=false
  Terminal 3: ros2 launch robot_slam_pkg slam_mapping.launch.py
  Terminal 4: ros2 run teleop_twist_keyboard teleop_twist_keyboard

  Drive slowly (< 0.2 m/s) around the entire restaurant.
  Watch the map build in RViz.
  Drive past every table, into the kitchen, through all doorways.
  Make at least 2 loops to trigger loop closure corrections.

  When map looks complete:
    ros2 service call /slam/save_map std_srvs/srv/Trigger
    Check: ros2 topic echo /slam/map_quality_report --once

USAGE:
  ros2 launch robot_slam_pkg slam_mapping.launch.py
  ros2 launch robot_slam_pkg slam_mapping.launch.py use_rviz:=false
  ros2 launch robot_slam_pkg slam_mapping.launch.py maps_dir:=/data/maps
"""

import os
from ament_python import LaunchDescription
from ament_python.actions import Node, DeclareLaunchArgument, LogInfo
from ament_python.substitutions import LaunchConfiguration
from ament_python.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg       = get_package_share_directory('robot_slam_pkg')
    slam_cfg  = os.path.join(pkg, 'config', 'slam_toolbox_mapping.yaml')
    maps_default = os.path.expanduser('~/delivery_bot_ws/maps')

    # ── Arguments ─────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz for mapping visualization'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    maps_dir_arg = DeclareLaunchArgument(
        'maps_dir', default_value=maps_default,
        description='Directory to save maps'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='restaurant_map',
        description='Base name for saved maps'
    )
    enable_dynamic_filter_arg = DeclareLaunchArgument(
        'enable_dynamic_filter', default_value='true',
        description='Enable moving-person filter during mapping'
    )
    auto_save_interval_arg = DeclareLaunchArgument(
        'auto_save_interval', default_value='120.0',
        description='Auto-save interval in seconds (crash safety)'
    )

    use_rviz              = LaunchConfiguration('use_rviz')
    use_sim_time          = LaunchConfiguration('use_sim_time')
    maps_dir              = LaunchConfiguration('maps_dir')
    map_name              = LaunchConfiguration('map_name')
    enable_dyn_filter     = LaunchConfiguration('enable_dynamic_filter')
    auto_save_interval    = LaunchConfiguration('auto_save_interval')

    # ── NODE 1: Dynamic Obstacle Filter ───────────────────────────
    # Sits between /scan and slam_toolbox.
    # Removes transient scan points (moving people) so they
    # don't become phantom walls in the map.
    # slam_toolbox subscribes to /scan_for_slam (output of this node).
    dynamic_filter_node = Node(
        package='robot_slam_pkg',
        executable='dynamic_obstacle_filter_node',
        name='dynamic_obstacle_filter_node',
        output='screen',
        parameters=[{
            'window_size'           : 5,
            'consistency_threshold' : 0.3,   # meters
            'enabled'               : enable_dyn_filter,
            'use_sim_time'          : use_sim_time,
        }]
    )

    # ── NODE 2: SLAM Toolbox (mapping mode) ───────────────────────
    # async_slam_toolbox_node: processes scans asynchronously.
    # This is correct for real hardware (sync mode can cause timing issues).
    #
    # CRITICAL topic remapping:
    #   slam_toolbox expects /scan by default.
    #   We remap it to /scan_for_slam (dynamic-filtered output).
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_cfg,
            {
                'use_sim_time': use_sim_time,
                # Override the map save path from launch arg
                'map_file_name': [maps_dir, '/', map_name],
            }
        ],
        remappings=[
            # Use dynamic-filtered scan instead of raw /scan
            ('/scan', '/scan_for_slam'),
        ]
    )

    # ── NODE 3: Map Manager ────────────────────────────────────────
    # Handles versioned map saving, auto-saves, registry.
    map_manager_node = Node(
        package='robot_slam_pkg',
        executable='map_manager_node',
        name='map_manager_node',
        output='screen',
        parameters=[{
            'maps_directory'         : maps_dir,
            'map_base_name'          : map_name,
            'auto_save_interval_sec' : auto_save_interval,
            'is_mapping_mode'        : True,
            'use_sim_time'           : use_sim_time,
        }]
    )

    # ── NODE 4: Map Quality Monitor ───────────────────────────────
    map_quality_node = Node(
        package='robot_slam_pkg',
        executable='map_quality_node',
        name='map_quality_node',
        output='screen',
        parameters=[{
            'min_free_pct'          : 10.0,   # relaxed for in-progress mapping
            'max_unknown_pct'       : 80.0,   # relaxed — mapping isn't done yet
            'expected_resolution'   : 0.05,
            'use_sim_time'          : use_sim_time,
        }]
    )

    # ── NODE 5: RViz ──────────────────────────────────────────────
    # During mapping, RViz is important so the operator can see:
    #   - /map             (the map being built — grey/white/black)
    #   - /scan_for_slam   (green points — what SLAM sees)
    #   - /scan            (what lidar sees before dynamic filter)
    #   - /slam/dynamic_mask (red points — what was removed as dynamic)
    #   - /tf              (robot pose in the map)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        # Load SLAM Toolbox RViz config which includes the interactive panel
        arguments=['--display-config',
                   os.path.join(pkg, 'config', 'slam_mapping.rviz')]
            if os.path.exists(
                os.path.join(pkg, 'config', 'slam_mapping.rviz'))
            else []
    )

    startup_msg = LogInfo(msg=[
        '\n',
        '╔════════════════════════════════════════════╗\n',
        '║   RESTAURANT SLAM MAPPING SESSION          ║\n',
        '╠════════════════════════════════════════════╣\n',
        '║  1. Drive robot around entire restaurant   ║\n',
        '║     (slow: < 0.2 m/s)                      ║\n',
        '║  2. Cover ALL areas: tables, kitchen,      ║\n',
        '║     doorways, storage                      ║\n',
        '║  3. Make 2+ loops for loop closure         ║\n',
        '║  4. When done, save the map:               ║\n',
        '║     ros2 service call /slam/save_map \\     ║\n',
        '║       std_srvs/srv/Trigger                 ║\n',
        '╠════════════════════════════════════════════╣\n',
        '║  Monitor:                                   ║\n',
        '║    ros2 topic echo /slam/map_warnings       ║\n',
        '║    ros2 topic echo /slam/dynamic_point_count║\n',
        '╚════════════════════════════════════════════╝\n',
    ])

    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        maps_dir_arg,
        map_name_arg,
        enable_dynamic_filter_arg,
        auto_save_interval_arg,
        startup_msg,
        dynamic_filter_node,
        slam_toolbox_node,
        map_manager_node,
        map_quality_node,
        rviz_node,
    ])
