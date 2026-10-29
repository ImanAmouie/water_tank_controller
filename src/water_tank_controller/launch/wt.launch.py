from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='water_tank_controller',
            namespace='controller',
            executable='controller_node',
            name='ctr',
            output='screen'
        ),
        Node(
            package='simulator',
            namespace='water_tank',
            executable='tank_sim',
            name='tank',
            output='screen'
        )
    ])