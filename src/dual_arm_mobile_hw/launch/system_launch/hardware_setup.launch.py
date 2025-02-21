from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_micro_ros_agent = Node(
        name='micro_ros_agent',
        executable='micro_ros_agent',
        package='micro_ros_agent',
        namespace='r100_0599',
        output='screen',
        arguments=
            [
                'udp4'
                ,
                '--port'
                ,
                '11411'
                ,
            ]
        ,
    )

    node_lighting_node = Node(
        name='lighting_node',
        executable='lighting_node',
        package='clearpath_hardware_interfaces',
        namespace='r100_0599',
        output='screen',
        parameters=
            [
                {
                    'platform': 'r100'
                    ,
                }
                ,
            ]
        ,
    )

    return LaunchDescription([node_micro_ros_agent, node_lighting_node])
