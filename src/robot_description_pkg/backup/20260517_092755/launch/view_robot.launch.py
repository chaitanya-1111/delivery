"""
============================================================================
robot_description_pkg/launch/view_robot.launch.py
----------------------------------------------------------------------------
PURPOSE : Standalone visualisation launcher.
          Parses the XACRO model, starts robot_state_publisher,
          joint_state_publisher_gui, and RViz2 with a curated config.

USAGE:
  # Default (opens RViz with full robot config)
  ros2 launch robot_description_pkg view_robot.launch.py

  # Headless URDF validation only (CI / Docker)
  ros2 launch robot_description_pkg view_robot.launch.py gui:=false rviz:=false

  # Custom RViz config
  ros2 launch robot_description_pkg view_robot.launch.py \
      rviz_config:=/path/to/my_config.rviz

ARGUMENTS:
  gui          – Launch joint_state_publisher_gui   [true|false]  default: true
  rviz         – Launch RViz2                        [true|false]  default: true
  rviz_config  – Path to .rviz config file          [path]
  model        – Path to robot.urdf.xacro            [path]
  use_sim_time – Use /clock topic (Gazebo)           [true|false]  default: false

NODES STARTED:
  /robot_state_publisher      – publishes TF from URDF + joint states
  /joint_state_publisher      – publishes zero joint states (static)
  /joint_state_publisher_gui  – GUI slider for joint positions (if gui:=true)
  /rviz2                      – visualisation (if rviz:=true)

AUTHOR  : Robotics Engineering Team
ROS     : ROS 2 Humble / Iron
============================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# ---------------------------------------------------------------------------
# Helper: resolve absolute path to the default RViz config
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


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------
def generate_launch_description():

    pkg_share = FindPackageShare("robot_description_pkg")

    # ── Declare launch arguments ──────────────────────────────────────────
    declare_gui = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        choices=["true", "false"],
        description="Launch joint_state_publisher_gui for interactive joint control",
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

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use simulation clock (/clock) instead of wall clock",
    )

    # ── robot_description via xacro ───────────────────────────────────────
    #
    #   Command() is lazy: evaluated at launch time, NOT at parse time.
    #   This means the xacro binary is called fresh each launch,
    #   picking up any edits without needing a rebuild.
    #
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            LaunchConfiguration("model"),
        ]
    )

    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    # ── robot_state_publisher ─────────────────────────────────────────────
    #
    #   Subscribes to /joint_states, publishes TF2 transforms for every
    #   joint and fixed frame in the URDF.
    #
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # ── joint_state_publisher  (non-GUI, always running) ─────────────────
    #
    #   Publishes zero-position joint states for all non-fixed joints.
    #   Needed as fallback when gui:=false so RSP gets a /joint_states topic.
    #
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    # ── joint_state_publisher_gui  (interactive sliders) ─────────────────
    #
    #   Replaces joint_state_publisher when gui:=true.
    #   Lets you drag wheel joints to verify TF visually.
    #
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # ── Lifecycle log messages ─────────────────────────────────────────────
    log_start = LogInfo(
        msg=[
            "\n\n",
            "=========================================================\n",
            "  delivery_robot  →  view_robot.launch.py\n",
            "  URDF  : ", LaunchConfiguration("model"), "\n",
            "  RViz  : ", LaunchConfiguration("rviz_config"), "\n",
            "  GUI   : ", LaunchConfiguration("gui"), "\n",
            "=========================================================\n",
        ]
    )

    log_rsp_started = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[
                LogInfo(msg="[view_robot] robot_state_publisher started → TF active")
            ],
        )
    )

    log_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[
                LogInfo(msg="[view_robot] RViz2 closed — shutting down launch"),
                launch.actions.EmitEvent(event=launch.events.Shutdown()),
            ],
        )
    )

    # ── Assemble LaunchDescription ─────────────────────────────────────────
    return LaunchDescription(
        [
            # Arguments
            declare_gui,
            declare_rviz,
            declare_rviz_config,
            declare_model,
            declare_use_sim_time,

            # Informational
            log_start,

            # Nodes
            robot_state_publisher_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,

            # Event handlers
            log_rsp_started,
            log_rviz_exit,
        ]
    )
