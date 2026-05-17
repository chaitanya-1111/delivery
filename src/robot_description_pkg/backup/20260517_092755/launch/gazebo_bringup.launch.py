"""
============================================================================
robot_description_pkg/launch/gazebo_bringup.launch.py
----------------------------------------------------------------------------
PURPOSE : Launch robot in Gazebo simulator with ROS 2 integration.
          Spawns Gazebo, loads the XACRO model, and starts robot_state_publisher.

USAGE:
  # Default (Gazebo + RViz visualization)
  ros2 launch robot_description_pkg gazebo_bringup.launch.py

  # Gazebo only (no RViz)
  ros2 launch robot_description_pkg gazebo_bringup.launch.py rviz:=false

  # Custom world
  ros2 launch robot_description_pkg gazebo_bringup.launch.py \
      world:=/path/to/world.sdf

  # Custom robot initial position
  ros2 launch robot_description_pkg gazebo_bringup.launch.py \
      x:=1.0 y:=2.0 z:=0.0 yaw:=1.57

ARGUMENTS:
  world       – Path to Gazebo world file         [path]  default: empty_world
  x, y, z     – Initial robot position           [float] default: 0.0
  yaw         – Initial robot yaw (radians)      [float] default: 0.0
  rviz        – Launch RViz2 visualization       [true|false]  default: true
  rviz_config – Path to .rviz config file        [path]
  model       – Path to robot.urdf.xacro         [path]
  gui         – Show Gazebo GUI                  [true|false]  default: true
  paused      – Start Gazebo paused              [true|false]  default: false

NODES STARTED:
  gazebo          – Gazebo simulator process
  robot_state_publisher – publishes TF from URDF + joint states
  rviz2           – visualisation (if rviz:=true)

TOPICS PUBLISHED:
  /cmd_vel        → subscribed by differential_drive_controller in Gazebo
  /odom           ← published by differential_drive_controller
  /clock          – Gazebo simulation time

AUTHOR  : Robotics Engineering Team
ROS     : ROS 2 Humble / Iron
============================================================================
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------
def _default_rviz_config() -> str:
    return os.path.join(
        get_package_share_directory("robot_description_pkg"),
        "config",
        "robot_view.rviz",
    )


def _default_xacro_path() -> str:
    return os.path.join(
        get_package_share_directory("robot_description_pkg"),
        "urdf",
        "robot.urdf.xacro",
    )


def _get_gazebo_world() -> str:
    """Return path to empty world if not specified"""
    # Try to find an empty world in Gazebo installation
    gazebo_paths = [
        "/usr/share/gazebo-11/worlds/empty.world",
        "/opt/ros/humble/share/gazebo_ros/worlds/empty.world",
        "/opt/ros/iron/share/gazebo_ros/worlds/empty.world",
    ]
    for path in gazebo_paths:
        if os.path.exists(path):
            return path
    # Fallback: Gazebo will use default empty world
    return ""


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------
def generate_launch_description():

    pkg_share = FindPackageShare("robot_description_pkg")

    # ── Declare launch arguments ──────────────────────────────────────────
    declare_world = DeclareLaunchArgument(
        name="world",
        default_value=_get_gazebo_world(),
        description="Path to Gazebo world (.world or .sdf file)",
    )

    declare_x = DeclareLaunchArgument(
        name="x",
        default_value="0.0",
        description="Initial X position of robot [m]",
    )

    declare_y = DeclareLaunchArgument(
        name="y",
        default_value="0.0",
        description="Initial Y position of robot [m]",
    )

    declare_z = DeclareLaunchArgument(
        name="z",
        default_value="0.0",
        description="Initial Z position of robot [m]",
    )

    declare_yaw = DeclareLaunchArgument(
        name="yaw",
        default_value="0.0",
        description="Initial yaw of robot [radians]",
    )

    declare_rviz = DeclareLaunchArgument(
        name="rviz",
        default_value="true",
        choices=["true", "false"],
        description="Launch RViz2 visualisation window",
    )

    declare_rviz_config = DeclareLaunchArgument(
        name="rviz_config",
        default_value=_default_rviz_config(),
        description="Absolute path to RViz2 .rviz configuration file",
    )

    declare_model = DeclareLaunchArgument(
        name="model",
        default_value=_default_xacro_path(),
        description="Absolute path to the robot XACRO/URDF file",
    )

    declare_gui = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        choices=["true", "false"],
        description="Show Gazebo GUI",
    )

    declare_paused = DeclareLaunchArgument(
        name="paused",
        default_value="false",
        choices=["true", "false"],
        description="Start Gazebo in paused state",
    )

    # ── robot_description via xacro ───────────────────────────────────────
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            LaunchConfiguration("model"),
        ]
    )

    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": True,
    }

    # ── Start Gazebo with ROS integration ─────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('paused'),
        }.items(),
    )

    # ── Spawn robot entity in Gazebo ───────────────────────────────────────
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "delivery_robot",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-Y",
            LaunchConfiguration("yaw"),
        ],
        parameters=[robot_description_param],
    )

    # ── robot_state_publisher ─────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # ── Lifecycle log messages ─────────────────────────────────────────────
    log_start = LogInfo(
        msg=[
            "\n\n",
            "=========================================================\n",
            "  🤖 DELIVERY ROBOT - GAZEBO SIMULATION LAUNCHING\n",
            "=========================================================\n",
            "  Robot spawned at: x=",
            LaunchConfiguration("x"),
            " y=",
            LaunchConfiguration("y"),
            " z=",
            LaunchConfiguration("z"),
            " yaw=",
            LaunchConfiguration("yaw"),
            "\n",
            "  Topics:\n",
            "    /cmd_vel        → send velocity commands\n",
            "    /odom           ← odometry feedback\n",
            "    /clock          ← simulation time\n",
            "  \n",
            "  Send velocity commands:\n",
            "    ros2 topic pub /cmd_vel geometry_msgs/Twist \\\n",
            "      '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'\n",
            "\n",
            "=========================================================\n\n",
        ]
    )

    log_end = LogInfo(
        msg=[
            "\n=========================================================\n",
            "  Gazebo startup complete. Robot ready for commands.\n",
            "=========================================================\n\n",
        ]
    )

    # ── Build launch description ───────────────────────────────────────────
    return LaunchDescription(
        [
            # Arguments
            declare_world,
            declare_x,
            declare_y,
            declare_z,
            declare_yaw,
            declare_rviz,
            declare_rviz_config,
            declare_model,
            declare_gui,
            declare_paused,
            # Log startup
            log_start,
            # Gazebo
            gazebo_launch,
            # Robot state publisher (before spawning)
            robot_state_publisher_node,
            # Spawn robot
            spawn_entity,
            # Visualization
            rviz_node,
            # Log completion
            log_end,
        ]
    )
