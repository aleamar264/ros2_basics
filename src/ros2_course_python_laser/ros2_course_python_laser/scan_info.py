import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class ScanNode(Node):
    def __init__(self):
        super().__init__('scan node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, scan_data):
        # find minimum range
        min_value, min_index = self.min_range_index(scan_data.ranges)
        print(f"\nthe minimum range value is {min_value}")
        print(f"\nthe minimum range index is {min_index}")

        max_value, max_index = self.max_range_index(scan_data.ranges)
        print(f"\nthe maximum range value is {max_value}")
        print(f"\nthe maximum range index is {max_index}")

        average_value = self.average_range(scan_data.ranges)
        print(f'\nthe average value is: {average_value}')

        average_value2 = self.average_between_indices(scan_data.ranges, 2, 7)
        print(f'\nthe average between 2 index is: {average_value2}')
        print(f'the field of view: {self.field_of_view(scan_data)}')

    def field_of_view(self, scan_data):
        return (scan_data.angle_max - scan_data.angle_min) * 180.0/math.pi
    
    def min_range_index(self, ranges):
        ranges = [x for x in ranges if not math.isnan(x)]
        return ( min(ranges), ranges.index(min(ranges)))

    def max_range_index(self, ranges):
        ranges = [x for x in ranges if not math.isnan(x)]
        return ( max(ranges), ranges.index(max(ranges)))
    
    def average_range(self, ranges):
        ranges = [x for x in ranges if not math.isnan(x)]
        return ( sum(ranges) / float(len(ranges)))
    
    def average_between_indices(self, ranges, i: int, j: int):
        ranges = [x for x in ranges if not math.isnan(x)]
        slice_of_array = ranges[i: j+1]
        return ( sum(slice_of_array) / float(len(slice_of_array)))

def main(args=None):
    rclpy.init(args=args)
    scan_node = ScanNode()
    rclpy.spin(scan_node)
    rclpy.shutdown()
    scan_node.destroy_node()

if __name__ == '__main__':
    main()