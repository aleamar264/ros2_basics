""" Movement client to test new service file created
Write by: Alejandro Amar Gil
Took from: theconstructsim [ROS2 Basics in 5 Days (Python)]
May 24 2023

Notion: https://nine-athlete-7f2.notion.site/Basics-and-fundamentals-II-bedb6331ada14738b56e4df15f50ad0e
"""

from custom_interfaces.srv import MyCustomServiceMessage
import rclpy
from rclpy.node import Node
import sys

class ClientAsync(Node):
    def __init__(self):
        """ Class constructor """
        super().__init__('movement_client')
        self.client = self.create_client(MyCustomServiceMessage,
                                         'movement')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service not runing')
        self.req = MyCustomServiceMessage.Request()
    
    def send_request(self):
        self.req.move = sys.argv[1]
        self.future = self.client.call_async(self.req)
    
def main(args=None):
    rclpy.init(args=args)
    client = ClientAsync()
    client.send_request()
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().error(
                    f'Service call failed {e}')
            else:
                client.get_logger().info(
                    f'Response state {response.success}'
                )
            break
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()