from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    wall_finder = Node(
            executable='wall_finder',
            package='wall_follower',
            output='screen',
            emulate_tty=True)
    wall_following =  Node(
            executable='wall_follower',
            package='wall_follower',
            output='screen',
            emulate_tty=True)
    return LaunchDescription([
        wall_finder,
        wall_following
        ])