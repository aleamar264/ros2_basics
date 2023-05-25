""" Movement server (Quiz)
Write by: Alejandro Amar Gil
Took from: theconstructsim [ROS2 Basics in 5 Days (Python)]
May 24 2023

Notion: https://nine-athlete-7f2.notion.site/Basics-and-fundamentals-II-bedb6331ada14738b56e4df15f50ad0e
"""
from geometry_msgs.msg import Twist
from services_quiz_srv.srv import Turn
import rclpy
from rclpy.node import Node
import time

class Service(Node):

    def __init__(self):
        """ Class constructor """
        super().__init__('movement_server')
        self.srv = self.create_service(
            Turn,
            'turn',
            self.custom_service_callback)
        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

    def custom_service_callback(self, request, response):
        """ The callback function receives the self-class parameter, 
            received along with two parameters called request and response
                - receive the data by request
                - return a result as a response 
        """

        msg = Twist()

        if request.direction == 'right':
            self.send_response(msg, self.get_logger(),
                                self.publisher_,
                                -abs(request.angular_velocity),
                                'Turning to the right direction!!',
                                request.time)
            response.success = True
        elif request.direction == 'left':
            self.send_response(msg, self.get_logger(),
                                self.publisher_,
                                abs(request.angular_velocity),
                                'Turning to the left direction!!',
                                request.time)
            response.success = True
        else:
            response.success = False
        return response
    
    @staticmethod
    def send_response(msg: Twist, logger, publisher,
                      linear_z: float,
                      message_logger: str,
                      time_: int) -> None:
        """send_response

        Args:
            msg (Twist): Twist
            logger (Node.get_logger): logger from Node class
            publisher (Node.create_publisher): publisher from Node class
            linear_x (float): velocity on linear x
            linear_z (float): velocity on linear z
            message_logger (str): message send to logger
        Return:
            None
        """
        msg.angular.z = linear_z
        publisher.publish(msg)
        logger.info(message_logger)
        start = time.time()
        while  True:
            if time.time() - start > time_:
                break
        msg.angular.z = 0.0
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    service = Service()
    rclpy.spin(service)
    rclpy.shutdown()