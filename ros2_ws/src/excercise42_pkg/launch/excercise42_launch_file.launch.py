from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='excercise42_pkg',
        executable='excercise42',
        output='screen')
    ])