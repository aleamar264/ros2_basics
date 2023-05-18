import rclpy
from rclpy.node import Node

from custom_interfaces.msg import Age

class Example36(Node):
    """ Example 36
     Publish age every 0.5 seconds """
    def __init__(self):
        super().__init__('example36')
        self.publisher_ = self.create_publisher(Age, 'age', 10)
        self.age = Age()
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period,
                                       self.timer_callback)

    def timer_callback(self):
        """ Simple callback for publish age """
        self.age.year = 2031
        self.age.month = 5
        self.age.day = 21
        self.publisher_.publish(self.age)

def main(args = None):
    rclpy.init(args=args)
    example36 = Example36()
    rclpy.spin(example36)
    example36.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()