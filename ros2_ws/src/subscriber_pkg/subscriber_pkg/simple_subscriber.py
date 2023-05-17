""" Simple subscriber
    See the Notion page to find the example more detailed
    https://nine-athlete-7f2.notion.site/Example-Code-d8505bcf905a4344b441244e6d21ac58
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import (ReliabilityPolicy,
                       QoSProfile)


class SimpleSubscriber(Node):
    """ Creation of Node simple subscriber """
    def __init__(self):
        super().__init__('simple_subsciber')
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

    def listener_callback(self, msg):
        """ 
        Create a callback to read data
        when get a msg
        """
        self.get_logger().info(f'I receive {str(msg)}')


def main(args=None):
    """ Main function """
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)

    simple_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
