from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unit_5',
            executable='exercise52',
            output='screen',
            emulate_tty=True),
    ])