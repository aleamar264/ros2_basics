from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys

def generate_launch_description():
    direction = LaunchConfiguration(
        'direction',
        default='rigth',
        )
    angular_velocity = LaunchConfiguration(
        'angular_velocity',
        default=0.0,
        )
    time = LaunchConfiguration(
        'time',
        default= 0,
        )    

    node = Node(
            package='services_quiz',
            executable='quiz_client',
            output='screen',
            arguments=[
            direction,
            angular_velocity,
            time
            ])
    return LaunchDescription([node])

if __name__ == '__main__':
    generate_launch_description()