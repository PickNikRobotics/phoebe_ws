import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_dual_arm_mobile_hw = FindPackageShare("dual_arm_mobile_hw")
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=["sudo", "/bin/sh", "-e", PathJoinSubstitution([pkg_dual_arm_mobile_hw, "launch", "clearpath-vcan-bridge"]), 
                 "-p", "11412", "-d", "/dev/ttycan0", "-v", "vcan0", "-b", "s8"],
            output="screen"
        )
    ])
