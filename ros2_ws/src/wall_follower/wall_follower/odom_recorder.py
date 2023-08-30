from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSReliabilityPolicy, QoSProfile
from rclpy.action import ActionServer
from wall_interfaces.action import OdomRecord
import rclpy
from geometry_msgs.msg import Point
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from math import sqrt, pow
from functools import partial


class OdometryNodeAction(Node):
    def __init__(self):
        super().__init__('odom_node_server')
        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.sub = self.create_subscription(
        Odometry,
        '/odom',
        self.odom_callback,
        qos_profile=QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT ),
        callback_group=self.cb_group)
        self._action_server = ActionServer(self,
            OdomRecord, 
            'record_odom',
            self.execute_callback,
            callback_group=self.cb_group) 
        self.last_odom = Point()
        self.first_odom = Point()
        self.total_distance = 0.0
        self.last_x, self.last_y = 0.0, 0.0
        first_measure = Point()
        first_measure.x, first_measure.y, first_measure.z = 0.0,0.0,0.0
        self.odom_records = [first_measure]
        self.first_odom_bool = False
        self.odom = Odometry()
        self.end, self.start = False, False
        
    
    def odom_callback(self, msg):
        if not self.first_odom_bool:
            self.first_odom.x = msg.pose.pose.position.x
            self.first_odom.y = msg.pose.pose.position.y
            self.first_odom.z = msg.pose.pose.orientation.z
            self.first_odom_bool = True
        self.odom = msg
        self.last_odom.x = msg.pose.pose.position.x
        self.last_odom.y = msg.pose.pose.position.y
        self.last_odom.z = msg.pose.pose.orientation.z

    def execute_callback(self, goal_handle: OdomRecord):
        
        self.get_logger().info('Recording Odom ...')

        feedback_msg = OdomRecord.Feedback()
        feedback_msg.current_total = self.total_distance

        # Feedback action

        callback_with_args = partial(
                    self.feedback_callback,
                    feedback_msg,
                    goal_handle
                )

        self.feedback_timer = self.create_timer(
                timer_period_sec=1, 
                callback=callback_with_args,
                callback_group=self.cb_group)

        while not self.end:
            rclpy.spin_once(self)
            # , timeout_sec=1)  # Process callbacks

        self.feedback_timer.cancel()
        goal_handle.succeed()
        result = OdomRecord.Result()
        result.list_of_odoms = self.odom_records
        self.get_logger().info('Recording Odometry ended')
        return result


    def feedback_callback(self, feed_msg, goal_handle: OdomRecord):
        self.odom_records.append(self.last_odom)
        self.total_distance += sqrt(
        pow(
            (self.last_x - self.last_odom.x), 2) +
        pow(
            (self.last_y - self.last_odom.y),2))
        self.last_x, self.last_y =  self.odom_records[-1].x, self.odom_records[-1].y
        feed_msg.current_total = self.total_distance
        goal_handle.publish_feedback(feed_msg)
        distance = sqrt(
        pow(
            (self.first_odom.x - self.last_x), 2) +
        pow(
            (self.first_odom.y - self.last_y),2))
        self.get_logger().info(f"The actual distance is {distance}")
        if distance <= 0.14 and self.start:
            self.end = True
        if not self.start and distance > 0.2:
            self.start = True


def main(args=None):
    rclpy.init(args=args)
    action_server = OdometryNodeAction()
    rclpy.spin(action_server)

if __name__ == "__main__":
    main()