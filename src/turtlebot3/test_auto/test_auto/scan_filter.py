import rclpy
import numpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String


class ScanFilter(Node):

    def __init__(self):
        super().__init__('scan_filter')       

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT


        qos1 = QoSProfile(depth=10)
        qos1.reliability = ReliabilityPolicy.RELIABLE

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            qos1
        )

        self.get_logger().info('Scan filter node started')
        

    def scan_callback(self, msg):
        filtered_msg = LaserScan()

        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        ranges = list(msg.ranges).copy()
        intensities = list(msg.intensities).copy()

        for i in range (len(ranges)):
            angle = msg.angle_min + i * msg.angle_increment

            if angle > 3.265 or angle < 0.125:
                ranges[i] = float('inf')
                if i < len(intensities):
                    intensities[i] = 0.0
        filtered_msg.ranges = ranges
        filtered_msg.intensities = intensities
        self.scan_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)

    scan_filter = ScanFilter()
    
    scan_filter.get_logger().info('do spin')
    print("print do spin")
    rclpy.spin(scan_filter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
