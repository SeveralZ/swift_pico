from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swift_pico',
            executable='bit_map.py',
            name='bit_map_creator',
            output='screen'
        ),
        Node(
            package='swift_pico',
            executable='path_planning_service.py',
            name='path_planning_service',
            output='screen'
        ),
        Node(
            package='swift_pico',
            executable='pico_server_2b.py',
            name='pico_server',
            output='screen'
        ),
        Node(
            package='swift_pico',
            executable='pico_client_2b.py',
            name='pico_client',
            output='screen'
        )
    ])
