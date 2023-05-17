import rclpy
# import the Node module from ROS2 Python library
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('Byakugan')
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Moe yo Byakugan! Kore ga watashi no nindō yo ')
        # english translation: "Blaze Away, Byakugan! This is My Ninja Way!"

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    node = MyNode()

    # Run until someone hit ctr + c
    rclpy.spin(node)

    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function