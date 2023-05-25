""" Simple service to turn the robot
Write by: Alejandro Amar Gil
Took from: theconstructsim [ROS2 Basics in 5 Days (Python)]
May 19 2023

Notion: https://nine-athlete-7f2.notion.site/Basics-and-fundamentals-II-bedb6331ada14738b56e4df15f50ad0e
"""

from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class Service(Node):
    def __init__(self):
        super().__init__('service_moving_right')

        self.srv = self.create_service(SetBool, 
                                       'moving_rigth',
                                       self.setBool_callback)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel = Twist()
    
    def setBool_callback(self, request, response):
        if request.data == True:
            self.response_callback(
                0.3, -0.3, response, 'MOVING TO THE RIGTH'
            )
        if request.data == False:
            self.response_callback(
                0.0, 0.0, response, 'It is time to stop'
            )
        return response

    def response_callback(self, x: float, z: float, response, msg: str):
        self.cmd_vel.linear.x = x
        self.cmd_vel.linear.z = z

        self.pub.publish(self.cmd_vel)

        response.success = True
        response.message = msg

def main(args=None):
    rclpy.init()
    moving_right_service = Service()
    rclpy.spin(moving_right_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()