from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dual_arm_mobile_hw = FindPackageShare("dual_arm_mobile_hw")

    launch_arg_imu_filter = DeclareLaunchArgument(
        'imu_filter',
        default_value=PathJoinSubstitution([pkg_dual_arm_mobile_hw, 'clearpath_config', 'imu_filter.yaml']),
        description='')

    imu_filter = LaunchConfiguration('imu_filter')

    # Include Packages
    pkg_clearpath_common = FindPackageShare('clearpath_common')
    pkg_clearpath_diagnostics = FindPackageShare('clearpath_diagnostics')
    pkg_clearpath_ros2_socketcan_interface = FindPackageShare('clearpath_ros2_socketcan_interface')
    pkg_clearpath_ros2_socketcan_interface = FindPackageShare('clearpath_ros2_socketcan_interface')

    # Declare launch files
    launch_file_platform = PathJoinSubstitution([
        # pkg_clearpath_common, 'launch', 'platform.launch.py'])
        pkg_dual_arm_mobile_hw, 'launch', 'platform.launch.py'])
    launch_file_diagnostics = PathJoinSubstitution([
        pkg_clearpath_diagnostics, 'launch', 'diagnostics.launch.py'])
    launch_file_receiver = PathJoinSubstitution([
        pkg_clearpath_ros2_socketcan_interface, 'launch', 'receiver.launch.py'])
    launch_file_sender = PathJoinSubstitution([
        pkg_clearpath_ros2_socketcan_interface, 'launch', 'sender.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    PathJoinSubstitution([pkg_dual_arm_mobile_hw, 'clearpath_config'])
                )
                ,
                (
                    'use_sim_time'
                    ,
                    'false'
                )
                ,
                (
                    'namespace'
                    ,
                    'r100_0599'
                )
                ,
                (
                    'enable_ekf'
                    ,
                    'true'
                )
                ,
            ]
    )

    launch_diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_diagnostics]),
    )

    launch_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_receiver]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'r100_0599'
                )
                ,
                (
                    'interface'
                    ,
                    'vcan0'
                )
                ,
                (
                    'from_can_bus_topic'
                    ,
                    'vcan0/rx'
                )
                ,
            ]
    )

    launch_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_sender]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'r100_0599'
                )
                ,
                (
                    'interface'
                    ,
                    'vcan0'
                )
                ,
                (
                    'to_can_bus_topic'
                    ,
                    'vcan0/tx'
                )
                ,
            ]
    )

    # Nodes
    node_wireless_watcher = Node(
        name='wireless_watcher',
        executable='wireless_watcher',
        package='wireless_watcher',
        namespace='r100_0599',
        output='screen',
        parameters=
            [
                {
                    'hz': 1.0
                    ,
                    'dev': ''
                    ,
                    'connected_topic': 'platform/wifi_connected'
                    ,
                    'connection_topic': 'platform/wifi_status'
                    ,
                }
                ,
            ]
        ,
    )

    node_battery_state_estimator = Node(
        name='battery_state_estimator',
        executable='battery_state_estimator',
        package='clearpath_hardware_interfaces',
        namespace='r100_0599',
        output='screen',
        arguments=
            [
                '-s'
                ,
                PathJoinSubstitution([FindPackageShare('dual_arm_mobile_hw'), 'clearpath_config'])
                ,
            ]
        ,
    )

    node_battery_state_control = Node(
        name='battery_state_control',
        executable='battery_state_control',
        package='clearpath_hardware_interfaces',
        namespace='r100_0599',
        output='screen',
        arguments=
            [
                '-s'
                ,
                PathJoinSubstitution([FindPackageShare('dual_arm_mobile_hw'), 'clearpath_config'])
                ,
            ]
        ,
    )

    node_imu_filter_node = Node(
        name='imu_filter_node',
        executable='imu_filter_madgwick_node',
        package='imu_filter_madgwick',
        namespace='r100_0599',
        output='screen',
        remappings=
            [
                (
                    'imu/data_raw'
                    ,
                    'sensors/imu_0/data_raw'
                )
                ,
                (
                    'imu/mag'
                    ,
                    'sensors/imu_0/magnetic_field'
                )
                ,
                (
                    'imu/data'
                    ,
                    'sensors/imu_0/data'
                )
                ,
                (
                    '/tf'
                    ,
                    'tf'
                )
                ,
            ]
        ,
        parameters=
            [
                imu_filter
                ,
            ]
        ,
    )

    node_puma_control = Node(
        name='puma_control',
        executable='multi_puma_node',
        package='puma_motor_driver',
        namespace='r100_0599',
        output='screen',
        parameters=
            [
                PathJoinSubstitution([pkg_dual_arm_mobile_hw, 'config', 'control', 'dual_arm_mobile.ros2_control.yaml'])
                ,
            ]
        ,
    )

    # Processes
    process_configure_mcu = ExecuteProcess(
        shell=True,
        cmd=
            [
                [
                    'export ROS_DOMAIN_ID=58;'
                    ,
                ]
                ,
                [
                    FindExecutable(name='ros2')
                    ,
                    ' service call platform/mcu/configure'
                    ,
                    ' clearpath_platform_msgs/srv/ConfigureMcu'
                    ,
                    ' "{domain_id: 58,'
                    ,
                    ' robot_namespace: \'r100_0599\'}"'
                    ,
                ]
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_imu_filter)
    ld.add_action(launch_platform)
    ld.add_action(launch_diagnostics)
    ld.add_action(launch_receiver)
    ld.add_action(launch_sender)
    ld.add_action(node_wireless_watcher)
    ld.add_action(node_battery_state_estimator)
    ld.add_action(node_battery_state_control)
    ld.add_action(node_imu_filter_node)
    ld.add_action(node_puma_control)
    ld.add_action(process_configure_mcu)

    return ld
