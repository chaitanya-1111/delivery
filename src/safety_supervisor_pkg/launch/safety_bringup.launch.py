#!/usr/bin/env python3
"""
Safety Hardware Package Launch File

Brings up all safety supervisor components:
- safety_supervisor_node: Main velocity gating
- hardware_watchdog_node: Component heartbeat monitoring
- safety_logger_node: Safety event logging
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    safety_hardware_dir = get_package_share_directory('safety_supervisor_pkg')
    config_dir = os.path.join(safety_hardware_dir, 'config')
    config_file = os.path.join(config_dir, 'safety_config.yaml')
    
    # Launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all safety nodes'
    )
    
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for ROS 2 logger'
    )
    
    declare_safety_enabled_arg = DeclareLaunchArgument(
        'safety_enabled',
        default_value='true',
        description='Enable safety supervision'
    )
    
    namespace = LaunchConfiguration('namespace')
    log_level = LaunchConfiguration('log_level')
    safety_enabled = LaunchConfiguration('safety_enabled')
    
    # Log launch info
    log_msg = LogInfo(msg="🛡️  Launching Safety Hardware Package")
    
    # Safety Supervisor Node
    # Main safety gating: gates velocity commands from nav/teleop
    safety_supervisor_node = Node(
        package='safety_supervisor_pkg',
        executable='safety_supervisor_node',
        name='safety_supervisor_node',
        namespace=namespace,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[config_file],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('cmd_vel_teleop', '/cmd_vel_teleop'),
            ('cmd_vel_safe', '/cmd_vel_safe'),
        ],
    )
    
    # Hardware Watchdog Node
    # Monitors heartbeats from motor controllers, LIDAR, power system
    hardware_watchdog_node = Node(
        package='safety_supervisor_pkg',
        executable='hardware_watchdog_node',
        name='hardware_watchdog_node',
        namespace=namespace,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'heartbeat_timeout': 1.5},
            {'latency_warning_threshold': 500},
        ]
    )
    
    # Safety Logger Node
    # Logs all safety events to JSONL files for analysis/debugging
    safety_logger_node = Node(
        package='safety_supervisor_pkg',
        executable='safety_logger_node',
        name='safety_logger_node',
        namespace=namespace,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'log_dir': '~/.ros/safety_logs'},
            {'max_log_size_mb': 100},
            {'compress_old_logs': True},
            {'keep_logs_days': 30},
        ]
    )
    
    return LaunchDescription([
        declare_namespace_arg,
        declare_log_level_arg,
        declare_safety_enabled_arg,
        log_msg,
        safety_supervisor_node,
        hardware_watchdog_node,
        safety_logger_node,
    ])
