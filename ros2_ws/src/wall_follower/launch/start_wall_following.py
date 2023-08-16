from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            executable='wall_follower',
            package='wall_follower',
            output='screen',
            emulate_tty=True)])