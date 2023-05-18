import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

class TopicQuiz(Node):
    def __init__(self):
        super().__init__('topics_quiz_node')
        self.publisher_ = self.create_publisher(Twist,
        '/cmd_vel', 10)

        self.subscriber = self.create_subscription(
        Odometry,
        '/odom',
        self.odom_callback, 
        qos_profile=QoSProfile(
        depth=10, 
        reliability=ReliabilityPolicy.RELIABLE)
        )            
    
    def odom_callback(self, msg):
        """  """
        self.get_logger().info(f'X {msg.pose.pose.position.x }')
        self.get_logger().info(f'Y {msg.pose.pose.position.y }')
        cmd_vel = Twist()
        if msg.pose.pose.position.x < 1:
            cmd_vel.linear.x = 0.3
            self.publisher_.publish(cmd_vel)
        if msg.pose.pose.position.x >= 1.05:
            cmd_vel.linear.x = 0.0
            self.publisher_.publish(cmd_vel)
            self.get_logger().info(f'Yaw {self.get_yaw(msg)}')
            cmd_vel.angular.z = self.turn_90_degrees(msg)
            self.publisher_.publish(cmd_vel)
        if msg.pose.pose.position.y < 0.5 and self.get_yaw(msg) >= 1.5:
            self.cmd_vel_pub(0.3, cmd_vel)
        if msg.pose.pose.position.y >= 0.5 and self.get_yaw(msg) >= 1.5:
            self.cmd_vel_pub(0.0, cmd_vel)

    # TODO Rename this here and in `odom_callback`
    def cmd_vel_pub(self, vel_x : float, cmd_vel: Twist):
        cmd_vel.linear.x = vel_x
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        self.publisher_.publish(cmd_vel)


    def turn_90_degrees(self, odom : Odometry):

        if self.get_yaw(odom) <= 1.5:
            return 0.2
        else:
            return 0.0

    def get_yaw(self, odom: Odometry):
        _, _, yaw = self.euler_from_quaternion([odom.pose.pose.orientation.x,
                                    odom.pose.pose.orientation.y,
                                    odom.pose.pose.orientation.z,
                                    odom.pose.pose.orientation.w])
        return yaw


    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    quiz = TopicQuiz()
    rclpy.spin(quiz)

    quiz.destroy_node()
    rclpy.shutdown()

