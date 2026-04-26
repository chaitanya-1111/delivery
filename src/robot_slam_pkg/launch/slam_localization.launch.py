"""
slam_localization.launch.py

PURPOSE:
  This is what runs EVERY DAY in production.
  Loads the saved restaurant map and localizes the robot within it.
  Used by Step 3 (Nav2) as the localization backend instead of AMCL.

  SLAM Toolbox localization is preferred over AMCL because:
    - More robust to minor furniture rearrangements
    - Better handles long corridors (fewer particle divergence issues)
    - Supports manual pose correction via RViz interactive plugin
    - Can switch between localization and limited mapping if needed

WHAT IT STARTS:
  1. slam_toolbox (localization mode) → localizes on saved map
  2. map_quality_node                 → validates map + monitors drift
  3. map_manager_node                 → map metadata/registry

USAGE:
  # Standard daily startup:
  ros2 launch robot_slam_pkg slam_localization.launch.py \
    map_file:=~/delivery_bot_ws/maps/restaurant_map_v3_20241201

  # The map_file should point to the .posegraph file (no extension needed):
  # slam_toolbox reads: restaurant_map_v3.posegraph + restaurant_map_v3.data

NOTE ON INITIAL POSE:
  When slam_toolbox starts in localization mode, it needs to know
  roughly where the robot is to start localizing.
  Option A: map_start_at_dock: true → starts at (0,0,0) = dock position
  Option B: Use RViz "2D Pose Estimate" to click the robot's location
  Option C: set initial_pose params below (if robot always starts same spot)
"""

import os
from ament_python import LaunchDescription
from ament_python.actions import Node, DeclareLaunchArgument, LogInfo
from ament_python.substitutions import LaunchConfiguration
from ament_python.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg       = get_package_share_directory('robot_slam_pkg')
    loc_cfg   = os.path.join(pkg, 'config', 'slam_toolbox_localization.yaml')
    maps_default = os.path.expanduser('~/delivery_bot_ws/maps')

    # ── Arguments ─────────────────────────────────────────────────
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(maps_default, 'restaurant_map'),
        description=(
            'Path to saved SLAM Toolbox pose graph (no extension). '
            'e.g. ~/delivery_bot_ws/maps/restaurant_map_v3_20241201'
        )
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    maps_dir_arg = DeclareLaunchArgument(
        'maps_dir', default_value=maps_default,
        description='Maps directory for map manager'
    )

    map_file     = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    maps_dir     = LaunchConfiguration('maps_dir')

    # ── NODE 1: SLAM Toolbox (localization mode) ───────────────────
    # localization_slam_toolbox_node = fixed map, only localize.
    # Publishes: /map, map→odom TF.
    # Does NOT add new nodes to the pose graph.
    slam_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            loc_cfg,
            {
                'use_sim_time'  : use_sim_time,
                'map_file_name' : map_file,
            }
        ],
        remappings=[
            # Use clean filtered scan (no dynamic obstacle filter needed
            # in localization mode — moving people don't affect the fixed map)
            ('/scan', '/scan'),
        ]
    )

    # ── NODE 2: Map Quality Monitor ───────────────────────────────
    # Stricter thresholds in localization mode (map should be complete)
    map_quality_node = Node(
        package='robot_slam_pkg',
        executable='map_quality_node',
        name='map_quality_node',
        output='screen',
        parameters=[{
            'min_free_pct'          : 15.0,
            'max_unknown_pct'       : 40.0,   # stricter for production
            'expected_resolution'   : 0.05,
            'stale_map_days'        : 60.0,
            'use_sim_time'          : use_sim_time,
        }]
    )

    # ── NODE 3: Map Manager ────────────────────────────────────────
    map_manager_node = Node(
        package='robot_slam_pkg',
        executable='map_manager_node',
        name='map_manager_node',
        output='screen',
        parameters=[{
            'maps_directory'    : maps_dir,
            'is_mapping_mode'   : False,   # read-only in localization
            'use_sim_time'      : use_sim_time,
        }]
    )

    startup_msg = LogInfo(msg=[
        '\n',
        '╔════════════════════════════════════════════╗\n',
        '║   SLAM LOCALIZATION MODE                   ║\n',
        '╠════════════════════════════════════════════╣\n',
        '║  Map: ', map_file,                           '\n',
        '║  Mode: Fixed map, localizing only          ║\n',
        '║                                            ║\n',
        '║  If robot pose is wrong:                   ║\n',
        '║  Use RViz → "2D Pose Estimate" button      ║\n',
        '║  Click robot location on map               ║\n',
        '╠════════════════════════════════════════════╣\n',
        '║  Check localization:                       ║\n',
        '║    ros2 topic echo /slam/map_valid --once  ║\n',
        '╚════════════════════════════════════════════╝\n',
    ])

    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        maps_dir_arg,
        startup_msg,
        slam_localization_node,
        map_quality_node,
        map_manager_node,
    ])
