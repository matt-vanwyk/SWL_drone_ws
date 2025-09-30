from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_pkg',
            executable='ntrip_publisher',
            output='screen',
            respawn=True,
            respawn_delay=5.0,
        ),

        Node(
            package='drone_pkg',
            executable='mavsdk_node',
            name='mavsdk_node',
            output='screen',
            respawn=True,
            respawn_delay=5.0,
        ),
        
        Node(
            package='drone_pkg',
            executable='drone_state_machine',
            name='drone_state_machine',
            output='screen',
            respawn=True,
            respawn_delay=5.0,
        ),
    ])

