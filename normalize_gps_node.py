#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

# Example bounding box (adjust as needed)
MIN_LAT, MAX_LAT = 38.0, 39.0
MIN_LON, MAX_LON = -10.0, -9.0

class NormalizeGPS(Node):
    def __init__(self):
        super().__init__('normalize_gps_node')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/normalized', 10)

    def listener_callback(self, msg):
        normalized_msg = NavSatFix()
        normalized_msg.header = msg.header
        normalized_msg.status = msg.status
        normalized_msg.altitude = msg.altitude

        # Normalize latitude and longitude
        normalized_msg.latitude = (msg.latitude - MIN_LAT) / (MAX_LAT - MIN_LAT)
        normalized_msg.longitude = (msg.longitude - MIN_LON) / (MAX_LON - MIN_LON)

        self.publisher_.publish(normalized_msg)
        self.get_logger().info(f'Normalized GPS: lat={normalized_msg.latitude:.3f}, lon={normalized_msg.longitude:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = NormalizeGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
