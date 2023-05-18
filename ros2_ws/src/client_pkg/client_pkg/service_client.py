""" Simple service client 
Write by: Alejandro Amar Gil
Took from: theconstructsim [ROS2 Basics in 5 Days (Python)]
May 19 2023

Notion: https://nine-athlete-7f2.notion.site/Basics-and-fundamentals-II-bedb6331ada14738b56e4df15f50ad0e
"""

from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

class ClientAsync(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(Empty, 'moving')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')
        
        self.req = Empty.Request()
    
    def send_request(self):
        self.future = self.client.call_async(self.req)


def main(args = None):
    rclpy.init(args=args)
    client = ClientAsync()

    client.send_request()
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    f'Service call failed {e}')
            else:
                client.get_logger().info(
                    'the robot is moving')
            break
    
    client.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()