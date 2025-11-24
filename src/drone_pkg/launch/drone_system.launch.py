from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # 1. Start ROS 2 nodes FIRST
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
        
        # 2. Wait 3 seconds for nodes to initialize
        # 3. Then trigger discovery with ros2 node list
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'node', 'list'],
                    output='screen',
                    name='trigger_discovery'
                ),
            ]
        ),
        
        # 4. Wait total 5 seconds (3s node startup + 2s discovery)
        # 5. Then restart DDS Router to pick up discovered nodes
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['sudo', 'systemctl', 'restart', 'dds-router-drone.service'],
                    output='screen',
                    name='restart_dds_router'
                ),
            ]
        ),
    ])