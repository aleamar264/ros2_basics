""" File that launch a single Node """
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """ Create a description of the node
        that launch the file simple_pub """
    return LaunchDescription([
            Node(
            package='publisher_pkg',
            executable='simple_pub',
            output = 'screen'
            )
    ])
