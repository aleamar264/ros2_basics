from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    LaunchDescription([
        Node(
        package='excercise31_pkg',
        executable='excercise31',
        output='screen'
        )
    ])