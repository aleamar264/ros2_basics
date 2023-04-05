import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ListenerSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription_ = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
        self.subscription_
    
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard {msg.data}')


def main(args = None):
    rclpy.init(args=args)

    minimal_sub = ListenerSubscriber()

    rclpy.spin(minimal_sub)

    minimal_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()