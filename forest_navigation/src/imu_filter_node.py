import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUFilter(Node):
    def __init__(self):
        super().__init__('imu_filter')
        self.subscription = self.create_subscription(
            Imu, '/imu/raw', self.callback, 10)
        self.publisher = self.create_publisher(Imu, '/imu/filtered', 10)

    def callback(self, msg):
        filtered = msg
        filtered.angular_velocity.x *= 0.98
        filtered.angular_velocity.y *= 0.98
        filtered.angular_velocity.z *= 0.98
        self.publisher.publish(filtered)

def main():
    rclpy.init()
    node = IMUFilter()
    rclpy.spin(node)
    rclpy.shutdown()
