"""
hardware_bringup.launch.py
==========================
Delivery Robot — Hardware Layer Launch File
robot_hardware_pkg

WHAT THIS LAUNCHES:
  1. hardware_interface_node  — motor control + odometry
  2. robot_state_publisher    — URDF → TF for all robot links

USAGE:

  Real hardware:
    ros2 launch robot_hardware_pkg hardware_bringup.launch.py

  Mock / software test:
    ros2 launch robot_hardware_pkg hardware_bringup.launch.py mock_mode:=true

  Custom serial port:
    ros2 launch robot_hardware_pkg hardware_bringup.launch.py serial_port:=/dev/ttyACM0

  All options:
    ros2 launch robot_hardware_pkg hardware_bringup.launch.py \
      mock_mode:=true \
      serial_port:=/dev/ttyUSB0 \
      pwm_deadband:=45 \
      use_rviz:=true
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch Arguments (overridable from command line) ──────────────
    args = [
        DeclareLaunchArgument(
            'mock_mode',
            default_value='false',
            description='Run in mock mode — no hardware required. true/false'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Arduino USB serial port'
        ),
        DeclareLaunchArgument(
            'serial_baud',
            default_value='115200',
            description='Serial baud rate'
        ),
        DeclareLaunchArgument(
            'pwm_deadband',
            default_value='40',
            description='Motor PWM deadband — tune to your motors'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz for visual debugging'
        ),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='base_footprint',
            description='Robot base frame (must match URDF)'
        ),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom',
            description='Odometry reference frame'
        ),
    ]

    # ── Config references ─────────────────────────────────────────────
    mock_mode     = LaunchConfiguration('mock_mode')
    serial_port   = LaunchConfiguration('serial_port')
    serial_baud   = LaunchConfiguration('serial_baud')
    pwm_deadband  = LaunchConfiguration('pwm_deadband')
    use_rviz      = LaunchConfiguration('use_rviz')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')

    # ── Hardware Interface Node ───────────────────────────────────────
    hardware_node = Node(
        package    = 'robot_hardware_pkg',
        executable = 'hardware_interface_node',
        name       = 'hardware_interface_node',
        output     = 'screen',
        emulate_tty= True,
        parameters = [{
            'mock_mode':     mock_mode,
            'serial_port':   serial_port,
            'serial_baud':   serial_baud,
            'pwm_deadband':  pwm_deadband,
            'base_frame_id': base_frame_id,
            'odom_frame_id': odom_frame_id,
            'publish_tf':    True,
        }],
        remappings = [
            ('/cmd_vel', '/cmd_vel'),
            ('/odom',    '/odom'),
        ],
    )

    # ── Robot State Publisher (loads URDF from robot_description_pkg) ─
    # This publishes the static TF for all URDF frames:
    #   base_footprint → base_link → lidar_link, camera_link, etc.
    # NOTE: Temporarily commented out due to URDF issues
    # urdf_file = os.path.join(
    #     get_package_share_directory('robot_description_pkg'),
    #     'urdf', 'robot.urdf.xacro'   # ← adjust filename to your URDF
    # )

    # with open(urdf_file, 'r') as f:
    #     robot_description = f.read()

    # robot_state_publisher = Node(
    #     package    = 'robot_state_publisher',
    #     executable = 'robot_state_publisher',
    #     name       = 'robot_state_publisher',
    #     output     = 'screen',
    #     parameters = [{
    #         'robot_description': robot_description,
    #         'use_sim_time':      False,
    #     }],
    # )

    # ── RViz (optional, for visual debugging) ────────────────────────
    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot_hardware_pkg'),
        'rviz', 'hardware_debug.rviz'
    ])

    rviz_node = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        condition  = IfCondition(use_rviz),
        # arguments  = ['-d', rviz_config],   # uncomment when you have an rviz config
    )

    # ── Startup log messages ──────────────────────────────────────────
    log_mock = LogInfo(
        condition = IfCondition(mock_mode),
        msg       = '>>> MOCK MODE: Running without hardware. Software test only. <<<'
    )
    log_real = LogInfo(
        condition = IfCondition(PythonExpression(["'", mock_mode, "' == 'false'"])),
        msg       = '>>> REAL HARDWARE MODE: Connecting to Arduino... <<<'
    )

    return LaunchDescription([
        *args,
        log_mock,
        log_real,
        hardware_node,
        # robot_state_publisher,  # Temporarily commented out
        rviz_node,
    ])