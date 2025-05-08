import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
import laspy
import os
import random
from shapely.geometry import MultiPoint

class BiomassNode(Node):
    def __init__(self):
        super().__init__('biomass_node')
        self.publisher = self.create_publisher(MarkerArray, 'tree_biomass_markers', 10)

        self.get_logger().info('🌲 Tree Biomass RViz Node Started')
        self.run_pipeline()

    def run_pipeline(self):
        las_path = os.path.expanduser('~/space_robotics_project/data/processed/downsampled_points.las')
        self.get_logger().info(f"🔍 Loading LAS file from: {las_path}")

        if not os.path.exists(las_path):
            self.get_logger().error("❌ LAS file not found!")
            return

        las = laspy.read(las_path)
        x, y, z = las.x, las.y, las.z

        if len(x) == 0:
            self.get_logger().error("❌ LAS file is empty!")
            return

        self.get_logger().info(f"✅ Loaded {len(x)} points")

        clusters = self.fake_clustering(x, y, z, cell_size=3.0)
        self.get_logger().info(f"🌲 Found {len(clusters)} tree clusters")


        marker_array = MarkerArray()
        for cluster_id, points in clusters.items():
            height = np.max(points[:, 2]) - np.min(points[:, 2])
            crown = MultiPoint(points[:, :2]).convex_hull
            area = crown.area if crown.is_valid else 0.0

            biomass = 0.05 * (height ** 2.5) * (area ** 1.0)

            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = "map"
            marker.id = cluster_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            center = np.mean(points, axis=0)
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            self.get_logger().info(f"🧭 Marker {cluster_id}: x={center[0]:.2f}, y={center[1]:.2f}, z={center[2]:.2f}, height={height:.2f}, biomass={biomass:.2f}kg")
            marker.scale.x = 1.5
            marker.scale.y = 1.5
            marker.scale.z = float(height)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.text = f'{biomass:.1f}kg'
            marker_array.markers.append(marker)
            # Text label marker for biomass
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.id = cluster_id + 100000  # make sure ID is unique
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(center[0])
            text_marker.pose.position.y = float(center[1])
            text_marker.pose.position.z = float(center[2] + height + 5.0)
            text_marker.scale.z = 5.0  # height of the text
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f'{biomass:.0f} kg'

            marker_array.markers.append(text_marker)

        self.publisher.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} tree markers')

    def fake_clustering(self, x, y, z, cell_size=5.0, min_points=50):
        from collections import defaultdict
        grid = defaultdict(list)

        for i in range(len(x)):
            key = (int(x[i] // cell_size), int(y[i] // cell_size))
            grid[key].append([x[i], y[i], z[i]])

        cluster_count = 0
        clustered = {}

        for i, pts in enumerate(grid.values()):
            if len(pts) >= min_points:
                clustered[i] = np.array(pts)
                cluster_count += 1

        self.get_logger().info(f"🔢 Grid cells checked: {len(grid)}, kept: {cluster_count}")
        return clustered



def main(args=None):
    rclpy.init(args=args)
    node = BiomassNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

