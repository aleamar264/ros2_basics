from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSReliabilityPolicy, QoSProfile

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        # Subscriber LaserScan
        self.sub_laser = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile=QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT ))
        self.laser_90 = None
        self.laser_0 = None
        # Publisher Vel
        self.pub_cmd = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        self.cmd = Twist()
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.cmd = self.set_vel(0.0, 0.0, self.cmd)
        self.pub_cmd.publish(self.cmd)

    def laser_callback(self, msg: LaserScan):
        """Get the data from the 90 degre
            a.k.a Wall, also get the data 
            for the front of the robot"""
        self.laser_90 = msg.ranges[269]
        self.laser_0 = msg.ranges[359]
        self.get_logger().warn(
            f'distance to the wall {self.laser_90}')

    def timer_cb(self):
        if self.laser_90 > 0.3:
            # Simulation
        #    self.cmd = self.set_vel(
        #    -0.1, 0.1, self.cmd)
            # real world
            self.cmd = self.set_vel(
               -0.05, 0.075, self.cmd)
            self.pub_cmd.publish(self.cmd)
            self.get_logger().info(
            'Approaching to the wall')
        elif self.laser_90 < 0.2:
            # Simulation
        #     self.cmd = self.set_vel(
        #    -0.1, 0., self.cmd)
            # real world
            self.cmd = self.set_vel(
               -0.05, 0.075, self.cmd)
            self.pub_cmd.publish(self.cmd)
            self.get_logger().info(
            'Moving away')
        if self.laser_90 < 0.3 and self.laser_90 > 0.2:
        # Simulation
        #     self.cmd = self.set_vel(
        #    0.0, 0.05, self.cmd)
        # real world
            self.cmd = self.set_vel(
           0.0, 0.075, self.cmd)
            self.pub_cmd.publish(self.cmd)
            self.get_logger().info(
            'Moving forward')
        if self.laser_0 < 0.65:
            self.cmd = self.set_vel(
                0.35 , 0.11, self.cmd)
            self.pub_cmd.publish(self.cmd)
            self.get_logger().info(
            'Turn left, but fasttt....')
        # pass

    @staticmethod
    def set_vel(
        angular_velocity: float,
        linear_velocity: float,
        cmd: Twist
    )-> Twist:
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        return cmd


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()