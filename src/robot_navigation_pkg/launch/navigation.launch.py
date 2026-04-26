"""
navigation.launch.py

PURPOSE:
  Main launch file for AUTONOMOUS NAVIGATION.
  Use this AFTER you have a saved map from slam_mapping.launch.py.

WHAT IT STARTS:
  1. map_server          → loads your saved .yaml map file
  2. amcl                → localizes robot on that map using lidar
  3. planner_server      → plans global paths
  4. controller_server   → follows paths (sends /cmd_vel)
  5. behavior_server     → recovery behaviors
  6. bt_navigator        → behavior tree orchestrator
  7. waypoint_follower   → multi-waypoint missions
  8. lifecycle_manager   → manages all above nodes
  9. nav_client_node     → accepts /navigation/goal commands
  10. nav_status_node    → publishes /navigation/status
  11. rviz2 (optional)   → visualization

USAGE:
  # Autonomous navigation with a specific map:
  ros2 launch robot_navigation_pkg navigation.launch.py map_file:=/path/to/map.yaml

  # Without RViz (headless):
  ros2 launch robot_navigation_pkg navigation.launch.py map_file:=/path/to/map.yaml use_rviz:=false

  # Send a goal manually (for testing):
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}}}}"
"""

import os
from ament_python import LaunchDescription
from ament_python.actions import (
    Node, DeclareLaunchArgument,
    IncludeLaunchDescription
)
from ament_python.launch_description_sources import PythonLaunchDescriptionSource
from ament_python.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_python.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package directories ──────────────────────────────────────
    pkg        = get_package_share_directory('robot_navigation_pkg')
    nav2_pkg   = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # ── Launch arguments ─────────────────────────────────────────
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg, 'maps', 'delivery_map.yaml'),
        description='Path to your saved map YAML file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Path to Nav2 params YAML'
    )

    map_file     = LaunchConfiguration('map_file')
    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')

    # ── Map Server ───────────────────────────────────────────────
    # Loads your .yaml map file and publishes it on /map topic
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_file,
             'use_sim_time': use_sim_time}
        ]
    )

    # ── AMCL ─────────────────────────────────────────────────────
    # Particle filter localization: "where am I on this map?"
    # Needs /scan (lidar) + /odom to work
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── Planner Server ───────────────────────────────────────────
    # Computes global path from current pose to goal
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── Controller Server ────────────────────────────────────────
    # Follows the global path by publishing /cmd_vel
    # This is what actually MOVES the robot!
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel')  # ensure it maps to your /cmd_vel topic
        ]
    )

    # ── Behavior Server ──────────────────────────────────────────
    # Recovery behaviors: spin, backup, wait
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── BT Navigator ─────────────────────────────────────────────
    # Orchestrates planner + controller + behaviors via Behavior Tree
    # This is the main action server: /navigate_to_pose
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── Waypoint Follower ────────────────────────────────────────
    # Multi-waypoint missions via /follow_waypoints action
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── Lifecycle Manager ────────────────────────────────────────
    # CRITICAL: Nav2 nodes use lifecycle pattern.
    # They start in "unconfigured" state and need to be
    # configured + activated. The lifecycle_manager does this automatically.
    # Order matters — map_server must be active before amcl starts.
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time,
             'autostart': True,
             'node_names': [
                 'map_server',
                 'amcl',
                 'planner_server',
                 'controller_server',
                 'behavior_server',
                 'bt_navigator',
                 'waypoint_follower',
             ]}
        ]
    )

    # ── Our Custom Navigation Client ─────────────────────────────
    # Bridges /navigation/goal (from mission_manager in Step 2)
    # to /navigate_to_pose Nav2 action
    nav_client_node = Node(
        package='robot_navigation_pkg',
        executable='nav_client_node',
        name='nav_client_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── Navigation Status Monitor ─────────────────────────────────
    nav_status_node = Node(
        package='robot_navigation_pkg',
        executable='nav_status_node',
        name='nav_status_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── RViz ─────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        # Args first
        map_file_arg,
        use_rviz_arg,
        use_sim_time_arg,
        params_file_arg,

        # Nav2 stack
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        behavior_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_node,

        # Our nodes
        nav_client_node,
        nav_status_node,
        rviz_node,
    ])
