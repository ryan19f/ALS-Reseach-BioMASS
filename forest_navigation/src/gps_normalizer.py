import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSNormalizer(Node):
    def __init__(self):
        super().__init__('gps_normalizer')
        self.origin_lat = 37.0
        self.origin_lon = -122.0
        self.subscription = self.create_subscription(
            NavSatFix, '/gps/raw', self.callback, 10)
        self.publisher = self.create_publisher(NavSatFix, '/gps/normalized', 10)

    def callback(self, msg):
        normalized = NavSatFix()
        normalized.header = msg.header
        normalized.latitude = msg.latitude - self.origin_lat
        normalized.longitude = msg.longitude - self.origin_lon
        normalized.altitude = msg.altitude
        self.publisher.publish(normalized)

def main():
    rclpy.init()
    node = GPSNormalizer()
    rclpy.spin(node)
    rclpy.shutdown()
