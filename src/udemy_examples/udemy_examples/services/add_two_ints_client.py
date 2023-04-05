from example_interfaces.srv import AddTwoInts
import rclpy

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('add_two_ints_client')

    cli = node.create_client(AddTwoInts, 'add_two_ints')
    while not cli.wait_for_service(timeout_sec=1.0):
        print('service not available, waiting again ...')
    
    req = AddTwoInts.Request()
    req.a = 2
    req.b = 3
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f'Result of add_two_ints: {future.result().sum}')
    else:
        node.get_logger().error(f'Excepton while calling service: {future.exception()}')

    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()