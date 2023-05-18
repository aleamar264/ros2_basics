from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

class ServiceMove(Node):
    def __init__(self):
        super().__init__('service_moving')
        self.srv = self.create_service(Empty, 
                                       'moving', 
                                       self.empty_callback)
        
        self.publisher_ = self.create_publisher(Twist,
                                                '/cmd_vel',
                                                10)
        
    def empty_callback(self, request, response):
        """ The callback function receives the self-class parameter, 
        received along with two parameters called request and response
        - receive the data by request
        - return a result as a response """

        msg = Twist()
        msg.linear.x = 0.3
        msg.linear.z = 0.3
        self.publisher_.publish(msg)
        self.get_logger().info('RUN ROBOT RUN')
        return response

def main(args=None):
    rclpy.init(args=args)
    moving_service = ServiceMove()
    rclpy.spin(moving_service)
    rclpy.shutdown()

if __name__=="__main__":
    main()
