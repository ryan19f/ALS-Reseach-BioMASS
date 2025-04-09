import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')
        self.subscription = self.create_subscription(
            LaserScan, '/scan/raw', self.callback, 10)
        self.publisher = self.create_publisher(LaserScan, '/scan/filtered', 10)

    def callback(self, msg):
        filtered = msg
        filtered.ranges = [min(r, 10.0) if r > 0.05 else 0.0 for r in msg.ranges]
        self.publisher.publish(filtered)

def main():
    rclpy.init()
    node = LidarFilter()
    rclpy.spin(node)
    rclpy.shutdown()
