from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_clearpath_sensors = FindPackageShare('clearpath_sensors')
    pkg_dual_arm_mobile_hw = FindPackageShare("dual_arm_mobile_hw")

    # Declare launch files
    launch_file_hokuyo_ust = PathJoinSubstitution([
        pkg_clearpath_sensors, 'launch', 'hokuyo_ust.launch.py'])

    # Include launch files
    launch_hokuyo_ust = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_hokuyo_ust]),
        launch_arguments=
            [
                (
                    'parameters'
                    ,
                    PathJoinSubstitution([pkg_dual_arm_mobile_hw, 'clearpath_config', 'lidar2d_1.yaml'])
                )
                ,
                (
                    'namespace'
                    ,
                    'r100_0599/sensors/lidar2d_1'
                )
                ,
                (
                    'robot_namespace'
                    ,
                    'r100_0599'
                )
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_hokuyo_ust)
    return ld
