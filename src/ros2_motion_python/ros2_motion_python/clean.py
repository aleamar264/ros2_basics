import copy
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
# LIBRARY TO RESET THE TURTLE SIM
from std_srvs.srv import Empty
from turtlesim.msg import Pose

from .RobotPose import RobotPose


class RobotCleaner(Node):

    LINEAR_SPEED = 0.5
    ANGULAR_SPEEED = 0.3

    def __init__(self):
        super().__init__('robot_cleaner')
        self.velocity__publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        time_period = 0.01

        self.pose = RobotPose()
        rclpy.spin_once(self)
        # Service to clear Turtle Sim
        self.client = self.create_client(Empty, 'reset')
        self.request = Empty.Request()
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        print('Turtlesim reset')

    def pose_callback(self, msg):
        self.pose.x = msg.x
        self.pose.y = msg.y
        self.pose.theta = msg.theta
        # print(self.pose)

    def move(self, distance: float, linear_speed: float, is_forward: bool):
        print(1)
        rclpy.spin_once(self)
        twist_msg = Twist()
        
        if linear_speed > 0.5:
            print('[ERROR]: The speed must be lower than 0.5!')
            return -1
        twist_msg.linear.x = abs(linear_speed) if is_forward else -abs(linear_speed)
        start_pose  = copy.copy(self.pose)

        rclpy.spin_once(self)
        while self.get_distance(start_pose, self.pose) < distance:
            rclpy.spin_once(self)
            print(f"distance: {self.get_distance(start_pose, self.pose)}, self.pose: {self.pose}, start_pose: {start_pose}")
            self.velocity__publisher.publish(twist_msg)
            time.sleep(0.2)

        twist_msg.linear.x = 0.0
        self.velocity__publisher.publish(twist_msg)

        print("The robot has stopped ...")
        return 0
    
    def get_distance(self, start_pose, final_pose) -> float:
        return math.sqrt(((final_pose.x - start_pose.x)**2+ (final_pose.y - start_pose.y)**2))


    def rotate(self, relative_angle_degree: float, angular_speed_in_degrees: float, clockwise: bool):

        rclpy.spin_once(self)
        twist_msg = Twist()
        
        if angular_speed_in_degrees > 15:
            print('[ERROR]: The angular speed can be greater 15!')
            return -1
        
        angular_speed_radians = math.radians(angular_speed_in_degrees)
        twist_msg.angular.z = -abs(angular_speed_radians) if clockwise else abs(angular_speed_radians)
        start_pose  = copy.copy(self.pose)

        rclpy.spin_once(self)

        rotates_relative_angle_degree = 0.0

        while rotates_relative_angle_degree < relative_angle_degree:
            rclpy.spin_once(self)
            rotates_relative_angle_degree = math.degrees(abs(start_pose.theta - self.pose.theta))
            self.velocity__publisher.publish(twist_msg)
            time.sleep(0.2)

        twist_msg.angular.z = 0.0
        self.velocity__publisher.publish(twist_msg)

        print("The robot has stopped ...")
        return 0
    
    def menu(self):
        print('|---------------------------------------------------|')
        print('| 1: Move')
        print('| 2: Rotate')
        print('| Q: Quit')
        print('|---------------------------------------------------|')
        return input('Enter your choice: ')

    def run(self):
        while True:
            choice = self.menu()
            if choice == '1':
                print('Moving... ')
                self.move(3.0, 0.5, True)
            elif choice == '2':
                print('Moving... ')
                self.rotate(relative_angle_degree=45, angular_speed_in_degrees=10, clockwise=True)
            elif choice == 'Q':
                print('Quitting ...')
                break
            else:
                print("Invalid choice. Try again")


def main(args=None):
    rclpy.init(args=args)
    cleaner_robot = RobotCleaner()
    
    cleaner_robot.run()

    cleaner_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

