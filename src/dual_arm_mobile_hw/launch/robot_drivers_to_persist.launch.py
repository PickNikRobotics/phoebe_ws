# TODOS:
# Add nav stuff
# Add both ur driver details
# Add clearpath driver stuff: estop details, battery info...
# Add hangar sim script for publishing linear x,y joints to odom

# Copyright 2025 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource

from moveit_studio_utils_py.launch_common import empty_gen
from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
)

def ur5e_nodes_to_launch(prefix, robot_ip_value, controller_config):
    robot_ip_name = f"{prefix}_robot_ip"
    robot_ip = LaunchConfiguration(robot_ip_name)

    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name=f"{prefix}_dashboard_client",
        output="both",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    protective_stop_manager_node = Node(
        package="moveit_studio_ur_pstop_manager",
        executable="protective_stop_manager_node",
        name=f"{prefix}_protective_stop_manager_node",
        output="both",
        parameters=[
            {
                "dashboard_client_name": f"/{prefix}_dashboard_client",
                "controllers_default_active": controller_config.get(
                    "controllers_active_at_startup", [] 
                ),
                "controllers_default_not_active": controller_config.get(
                    "controllers_inactive_at_startup", []
                ),
            }
        ],
    )

    # nodes_to_launch = [dashboard_client_node, protective_stop_manager_node]
    nodes_to_launch = [dashboard_client_node]

    # NOTE: the left gripper is plugged in via USB to the PC at /dev/ttyUSB0, so for now just launch tool comms for the right UR5e
    # TODO: once we get the right gripper on the same tool comms interface they can't both be at /tmp/ttyUR so we'll need to make some changes to the xacro macro that hardcodes that within picknik_ur_attachments_macro.xacro
    if prefix == "right":
        tool_comms_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource([FindPackageShare("picknik_ur_base_config"), "/launch/ur_tool_comms.launch.xml"]),
            launch_arguments={
                "robot_ip": robot_ip,
                "tool_tcp_port": "54321",
                "tool_device_name": "/tmp/ttyUR",
            }.items(),
        )
        nodes_to_launch.append(tool_comms_launch)

    return nodes_to_launch

def generate_launch_description():
    # Get path to config share directory.
    pkg_dual_arm_mobile_hw = FindPackageShare("dual_arm_mobile_hw")

    # Declare launch arguments for robot IPs.
    left_robot_ip_arg = DeclareLaunchArgument(
        'left_robot_ip', default_value='192.168.131.4', description='IP address of the left robot'
    )
    right_robot_ip_arg = DeclareLaunchArgument(
        'right_robot_ip', default_value='192.168.131.3', description='IP address of the right robot'
    )

    system_config_parser = SystemConfigParser()
    hardware_config = system_config_parser.get_hardware_config()
    controller_config = system_config_parser.get_ros2_control_config()

    left_robot_ip = "192.168.131.2"
    right_robot_ip = "192.168.131.3"

    left_ur_nodes_to_launch = ur5e_nodes_to_launch("left", left_robot_ip, controller_config)
    right_ur_nodes_to_launch = ur5e_nodes_to_launch("right", right_robot_ip, controller_config)

    launch_description = [left_robot_ip_arg, right_robot_ip_arg] + left_ur_nodes_to_launch + right_ur_nodes_to_launch

    # Publish odometry as joint state messages.
    odom_to_joint_state_repub = Node(
        package="dual_arm_mobile_hw",
        executable="odometry_joint_state_publisher.py",
        name="odometry_joint_state_publisher",
        output="log",
    )
    launch_description.append(odom_to_joint_state_repub)

    # Publish odometry as joint state messages.
    estop_pub = Node(
        package="dual_arm_mobile_hw",
        executable="estop_publisher.py",
        name="moveit_pro_estop_pub",
        output="log",
    )
    launch_description.append(estop_pub)

    # Setup VCAN comms.
    vcan_launch_file = PathJoinSubstitution([pkg_dual_arm_mobile_hw, "launch", "vcan-service.launch.py"])
    vcan_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(vcan_launch_file))
    launch_description.append(vcan_launch)

    # Start sensor processes.
    sensors_launch_file = PathJoinSubstitution([pkg_dual_arm_mobile_hw, "launch", "sensors-service.launch.py"])
    sensors_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(sensors_launch_file))
    launch_description.append(sensors_launch)

    # Start platform processes.
    platform_launch_file = PathJoinSubstitution([pkg_dual_arm_mobile_hw, "launch", "platform-service.launch.py"])
    platform_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(platform_launch_file))
    delayed_platform_launch = TimerAction(period=4.0, actions=[platform_launch])
    launch_description.append(delayed_platform_launch)


    return LaunchDescription(launch_description)
