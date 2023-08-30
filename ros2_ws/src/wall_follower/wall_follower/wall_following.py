from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSReliabilityPolicy, QoSProfile
from wall_interfaces.srv import FindWall
from rclpy.action import ActionClient
from wall_interfaces.action import OdomRecord

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
        self.timer = self.create_timer(0.2, self.timer_cb)
        self.cmd = self.set_vel(0.0, 0.0, self.cmd)
        self.pub_cmd.publish(self.cmd)

    def laser_callback(self, msg: LaserScan):
        """Get the data from the 90 degre
            a.k.a Wall, also get the data 
            for the front of the robot"""
        self.laser_90 = msg.ranges[269]
        self.laser_0 = msg.ranges[359]

    def timer_cb(self):
        if self.laser_90 > 0.3:
            # Simulation
        #    self.cmd = self.set_vel(
        #    -0.1, 0.1, self.cmd)
            # real world
            self.cmd = self.set_vel(
               -0.1, 0.075, self.cmd)
            self.pub_cmd.publish(self.cmd)
            self.get_logger().info(
            'Approaching to the wall')
        elif self.laser_90 < 0.2:
            # Simulation
        #     self.cmd = self.set_vel(
        #    -0.1, 0., self.cmd)
            # real world
            self.cmd = self.set_vel(
               0.1, 0.075, self.cmd)
            self.pub_cmd.publish(self.cmd)
            self.get_logger().info(
            'Moving away')
        if self.laser_90 <= 0.3 and self.laser_90 >= 0.2:
        # Simulation
        #     self.cmd = self.set_vel(
        #    0.0, 0.05, self.cmd)
        # real world
            self.cmd = self.set_vel(
           0.0, 0.1, self.cmd)
            self.pub_cmd.publish(self.cmd)
            self.get_logger().info(
            'Moving forward')
        if self.laser_0 < 0.6:
            self.cmd = self.set_vel(
                0.3 , 0.1, self.cmd)
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

class ClientFindWall(Node):
    def __init__(self):
        super().__init__('client_find_wall')
        self.cli = self.create_client(
            FindWall,
            'find_wall'
            )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again ...')
        self.req = FindWall.Request()
    
    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node_client')
        self._action_client = ActionClient(self, 
            OdomRecord,
            'record_odom')
    
    def init_record(self):
        goal_msg = OdomRecord.Goal()

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(
            self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accpeted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result {0}'.format(result.list_of_odoms))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Current distance {0}'.format(
            feedback.current_total))

def main(args=None):
    rclpy.init(args=args)

    # Client calll async
    client_findwall = ClientFindWall()
    response = client_findwall.send_request()
    client_findwall.get_logger().info(
        'The robot had find the wall %s' % (response.wallfound))
    client_findwall.destroy_node()

    # Record Odom
    action_node = OdomNode()
    action_node.get_logger().info('Init Odom record ...')
    action_node.init_record()

    # Control loop
    wall_follower = WallFollower()
    wall_follower.get_logger().info(
        "Following the wall")

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(action_node)
    executor.add_node(wall_follower)
    
    try:
        # Spin both nodes
        executor.spin()
        # rclpy.spin(WallFollower)
    except Exception:
        action_node.destroy_node()
    except KeyboardInterrupt:
        wall_follower.destroy_node()
        rclpy.shutdown()
    # finally:
    #     # Clean up and shut down
    #     action_node.destroy_node()
    #     wall_follower.destroy_node()
    # rclpy.shutdown()


    # wall_follower.destroy_node()
    # rclpy.shutdown()