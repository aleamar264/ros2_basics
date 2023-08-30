from wall_interfaces.srv import FindWall

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSReliabilityPolicy, QoSProfile
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from time import sleep

class FindWallService(Node):

    def __init__(self):
        super().__init__('find_wall')
        self.srv = self.create_service(FindWall, 
            'find_wall', 
            self.find_wall_callback)
        self.min_index = None
        self.zero_index = None
        self.groupCB = MutuallyExclusiveCallbackGroup()
        self.sub_laser = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile=QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT ),
                callback_group=self.groupCB)

        self.pub_cmd = self.create_publisher(
                Twist,
                'cmd_vel',
                10)
        self.set_vel(0.0, 0.0)
        self.flag_stop = False

    def laser_callback(self, msg: LaserScan):
        """Get the data from the 90 degre
            a.k.a Wall, also get the data 
            for the front of the robot"""
        self.min_index = msg.ranges.index(min(msg.ranges))
        self.zero_index = msg.ranges[0]

    def find_wall_callback(self, request, response):
        """
        turn rigth until the 0 is the min value of laser scan
        move forward until the 0 index is almost 0.3m o least
        turn rigth until the 270 index is the min value
        Return True on the response"""
        self.zero_flag = False
        self.timer = self.create_timer(timer_period_sec=0.5, 
                callback=self.timer_callback,
                callback_group=self.groupCB)

        while not self.flag_stop:
            rclpy.spin_once(self, timeout_sec=0.2)  # Process callbacks

        self.timer.cancel()
        self.get_logger().info('Wall Found')
        response.wallfound = True
        return response

    def timer_callback(self) -> None:
        if self.min_index <= 180 and not self.zero_flag: 
            self.cmd = self.set_vel(0.3, 0.0)
        elif self.min_index > 180 and not self.zero_flag:
            self.cmd = self.set_vel(-0.3, 0.0) 
        if self.min_index > 0 and self.min_index < 35:
            self.zero_flag = True
            self.stop()
            self.cmd = self.set_vel(0.0, 0.01)
        if self.zero_index <= 0.3:
            self.stop()
            self.cmd = self.set_vel(0.3, 0.0)
        if self.min_index > 260 and self.min_index < 275 and self.zero_flag:
            self.stop()
            self.flag_stop = True
            
    def stop(self) -> None:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

    
    def set_vel(
        self,
        angular_velocity: float,
        linear_velocity: float,
    )-> None:
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FindWallService()
    executor  = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
