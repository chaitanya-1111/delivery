import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('robot_lidar_pkg')

    scan_filter_config = os.path.join(pkg, 'config', 'scan_filter.yaml')

    lidar_model_arg = DeclareLaunchArgument(
        'lidar_model',
        default_value='c1',
        description='RPLidar model: c1 or m2'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rplidar',
        description='Serial port for RPLidar. Use /dev/rplidar (udev rule) or /dev/ttyUSB0 (fallback)'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='Baud rate. C1 and M2 both use 460800.'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Lidar TF frame name'
    )

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='RPLidar scan mode: Standard, Boost, Sensitivity, Stability'
    )

    lidar_x_arg = DeclareLaunchArgument(
        'lidar_x',
        default_value='0.0',
        description='Lidar X offset from base_link center (meters, forward)'
    )
    lidar_y_arg = DeclareLaunchArgument(
        'lidar_y',
        default_value='0.0',
        description='Lidar Y offset from base_link center (meters, left)'
    )
    lidar_z_arg = DeclareLaunchArgument(
        'lidar_z',
        default_value='0.18',
        description='Lidar height above base_link (meters)'
    )
    lidar_yaw_arg = DeclareLaunchArgument(
        'lidar_yaw',
        default_value='0.0',
        description='Lidar yaw rotation (radians). Use 3.14159 if lidar is mounted rotated 180°.'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for scan visualization'
    )

    enable_watchdog_arg = DeclareLaunchArgument(
        'enable_watchdog',
        default_value='true',
        description='Enable USB disconnect watchdog'
    )

    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    scan_mode = LaunchConfiguration('scan_mode')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_yaw = LaunchConfiguration('lidar_yaw')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_watch = LaunchConfiguration('enable_watchdog')

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': serial_baud,
            'frame_id': frame_id,
            'topic_name': 'scan_raw',
            'angle_compensate': True,
            'scan_mode': scan_mode,
            'inverted': False,
        }],
        remappings=[
            ('/scan', '/scan_raw'),
        ]
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_node',
        output='screen',
        parameters=[scan_filter_config],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan'),
        ]
    )

    lidar_tf_node = Node(
        package='robot_lidar_pkg',
        executable='lidar_tf_node',
        name='lidar_tf_node',
        output='screen',
        parameters=[{
            'parent_frame': 'base_link',
            'child_frame': 'laser',
            'x': lidar_x,
            'y': lidar_y,
            'z': lidar_z,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': lidar_yaw,
            'publish_tf': True,
        }]
    )

    lidar_diag_node = Node(
        package='robot_lidar_pkg',
        executable='lidar_diagnostics_node',
        name='lidar_diagnostics_node',
        output='screen',
        parameters=[{
            'expected_frequency': 8.0,
            'timeout_sec': 3.0,
            'hardware_id': 'RPLidar C1/M2',
        }]
    )

    scan_watchdog_node = Node(
        package='robot_lidar_pkg',
        executable='scan_watchdog_node',
        name='scan_watchdog_node',
        output='screen',
        condition=IfCondition(enable_watch),
        parameters=[{
            'warn_timeout_sec': 3.0,
            'recovery_timeout_sec': 8.0,
            'critical_timeout_sec': 30.0,
            'enable_auto_recovery': True,
            'lidar_service_name': 'rplidar.service',
            'lidar_device_path': serial_port,
        }]
    )

    rviz_config = os.path.join(pkg, 'config', 'lidar_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    startup_msg = LogInfo(
        msg=[
            '\n',
            '================================================\n',
            '  RPLidar Bringup Starting\n',
            '================================================\n',
            '  Serial port : ', serial_port, '\n',
            '  Baud rate   : ', serial_baud, '\n',
            '  Frame ID    : ', frame_id, '\n',
            '  Scan mode   : ', scan_mode, '\n',
            '  Lidar mount : x=', lidar_x,
                             ' y=', lidar_y,
                             ' z=', lidar_z,
                             ' yaw=', lidar_yaw, '\n',
            '\n',
            '  Topics:\n',
            '    /scan_raw → raw 360° scan from driver\n',
            '    /scan     → filtered scan (use this for Nav2/SLAM)\n',
            '    /lidar/health         → OK / WARN / ERROR\n',
            '    /lidar/watchdog_status → USB health\n',
            '    /diagnostics          → standard ROS diagnostics\n',
            '\n',
            '  Verify scan: ros2 topic hz /scan\n',
            '  Should show: ~8.0 Hz\n',
            '================================================\n',
        ]
    )

    return LaunchDescription([
        lidar_model_arg,
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        scan_mode_arg,
        lidar_x_arg,
        lidar_y_arg,
        lidar_z_arg,
        lidar_yaw_arg,
        use_rviz_arg,
        enable_watchdog_arg,
        startup_msg,
        rplidar_node,
        laser_filter_node,
        lidar_tf_node,
        lidar_diag_node,
        scan_watchdog_node,
        rviz_node,
    ])
