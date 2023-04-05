import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# LIBRARY TO RESET THE TURTLE SIM
from std_srvs.srv import Empty
import time


class TurtleSimMove(Node):

    LINEAR_SPEED = 0.5
    ANGULAR_SPEEED = 0.3

    def __init__(self):
        super().__init__('talker_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        time_period = 0.5
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.counter = 0
        self.t0 = time.time()

        # Service to clear Turtle Sim
        self.client = self.create_client(Empty, 'reset')
        self.request = Empty.Request()
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        print('Turtlesim reset')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = TurtleSimMove.LINEAR_SPEED
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = TurtleSimMove.ANGULAR_SPEEED

        time1 = time.time()
        time_spent = time1 - self.t0
        if time_spent > 5.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            rclpy.shutdown()

        self.publisher_.publish(msg)
        self.get_logger().info(f'Sendign a linear velocity of {msg}')


def main(args=None):
    rclpy.init(args=args)
    turtle_sim = TurtleSimMove()

    rclpy.spin(turtle_sim)
    turtle_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
