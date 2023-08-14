from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            executable='wall_finder',
            package='wall_follower',
            output='screen')
            ])