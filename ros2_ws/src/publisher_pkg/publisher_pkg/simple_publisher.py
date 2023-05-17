""" See the Notion page to find the example more detailed
    https://nine-athlete-7f2.notion.site/Example-Code-d8505bcf905a4344b441244e6d21ac58
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimplePublisher(Node):
    """ Creation of Node simple publisher """
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Twist,
                                                '/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """ Create a callback to send data
            every 0.5 seconds
        """
        msg = Twist()
        msg.linear.x = 0.5
        msg.linear.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing {msg}')

def main(args = None):
    """ Main function """
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
