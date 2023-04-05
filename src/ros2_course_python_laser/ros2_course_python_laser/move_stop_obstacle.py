import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class TurtleBot3(Node):
    """ 
    A ROS2 node to move a Turtlebot robot and stop if an obstacle is
    within 0.5 meters 
    """

    def __init__(self):
        """ 
        Constructor

        Initialize the node, publisher and subscriber
        """
        super().__init__('turtlebot')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.current_scan = None

    def scan_callback(self, msg):
        """ 
        Callback function to handle incoming laser scan msgs

        Saves the most recent scan message to `self.current_scan` 
        """

        self.current_scan = msg

    def move(self, linear_velocity: float, angular_velocity: float):
        """
        Publishes linear and angular velocities to the robot

        Args:
            linear_velocity (float): The desired linear velocity
            angular_velocity (flaot): The desired angular velocity
        """

        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.publisher.__publisher(msg)

    def run(self):
        """
        Main loop of the node.

        Moves the robot forward if there are no obstacles within 0.5m, and stops 
        the robot if there are an pbstacle. Runs until the node shutdown
        """

        while (rclpy.ok):
            rclpy.spin_once(self)
            if self.current_scan is not None:
                ranges = self.current_scan.ranges
                min_range = min(ranges)
                if min_range < 0.2:
                    self.move(0.0, 0.0)
                else:
                    self.move(0.1, 0.0)
                self.current_scan = None

def main(args=None):
    """
    Main functions to start the node
    """
    rclpy.init(args=args)
    node = TurtleBot3()
    node.run()
    rclpy.shutdown()

if __name__=='__main__':
    main()