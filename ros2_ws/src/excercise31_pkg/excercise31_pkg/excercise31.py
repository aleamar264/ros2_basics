""" See the Notion page to find the example more detailed
    https://nine-athlete-7f2.notion.site/Example-Code-d8505bcf905a4344b441244e6d21ac58
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (ReliabilityPolicy, QoSProfile)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Excercise31(Node):
    def __init__(self):
        super().__init__('excercise31')
        self.publisher_ = self.create_publisher(Twist, 
                                                '/cmd_vel',
                                                10)
        self.subscriber = self.create_subscription(LaserScan,
                                                   'LaserScan',
                                                   self.laser_callback,
                                                   qos_profile=QoSProfile(depth = 10,
                                                                          reliability=ReliabilityPolicy.RELIABLE))
        self.timer_period = 0.5
        self.laser_forward = 0
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)


    def laser_callback(self, msg):
        """ Get the last value of the array """
        self.laser_forward  = msg.ranges[359]
    
    def motion(self):
        """ Basic logic to rotate and
            go forward """
        self.get_logger().info(f'I receive :{str(self.laser_forward)}')
        # Logic
        if self.laser_forward > 5:
            self.cmd.linear.x = 0.5
            self.cmd.linear.z = 0.5
        elif self.laser_forward < 5 and self.laser_forward >= 0.5:
            self.cmd.linear.x = 0.2
            self.cmd.linear.z = 0.0
        else:
            self.cmd.linear.x = 0.0
            self.cmd.linear.z = 0.0
        
        self.publisher_.publish(self.cmd)

def main(args=None):
    """ Main function """
    rclpy.init(args=args)
    excercise31 = Excercise31()

    rclpy.spin(excercise31)

    excercise31.destroy_node()
    rclpy.shutdown()