from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # NTRIP Configuration parameters
        DeclareLaunchArgument(
            'ntrip_server',
            default_value='rtk2go.com',
            description='NTRIP server address'
        ),
        DeclareLaunchArgument(
            'ntrip_port',
            default_value='2101',
            description='NTRIP server port'
        ),
        DeclareLaunchArgument(
            'ntrip_mountpoint',
            default_value='Renewables',
            description='NTRIP mountpoint'
        ),
        DeclareLaunchArgument(
            'ntrip_user',
            default_value='s224482378-at-mandela.ac.za',
            description='NTRIP username'
        ),
        DeclareLaunchArgument(
            'ntrip_password',
            default_value='mukandagumbo',
            description='NTRIP password'
        ),
        
        # NTRIP Publisher Node
        Node(
            package='drone_pkg',
            executable='ntrip_publisher',
            name='ntrip_publisher',
            parameters=[{
                'ntrip_server': LaunchConfiguration('ntrip_server'),
                'ntrip_port': LaunchConfiguration('ntrip_port'),
                'ntrip_mountpoint': LaunchConfiguration('ntrip_mountpoint'),
                'ntrip_user': LaunchConfiguration('ntrip_user'),
                'ntrip_password': LaunchConfiguration('ntrip_password'),
            }],
            output='screen'
        ),
        
        # Existing drone nodes
        Node(
            package='drone_pkg',
            executable='drone_state_machine',
            name='drone_state_machine',
            output='screen'
        ),
        
        Node(
            package='drone_pkg',
            executable='mavsdk_node',
            name='mavsdk_node',
            output='screen'
        ),
    ])