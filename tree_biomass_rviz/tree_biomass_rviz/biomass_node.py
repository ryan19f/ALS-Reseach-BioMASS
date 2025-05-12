#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import laspy
import os
import json
import subprocess
from shapely.geometry import MultiPoint
from scipy.interpolate import griddata
from collections import defaultdict
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import plotly.express as px

class BiomassVisualizerNode(Node):
    def __init__(self):
        super().__init__('biomass_visualizer_node')
        self.get_logger().info("🌲 Biomass Visualizer Node Started")
        self.create_timer(2.0, self.run_pipeline)

    def run_pipeline(self):
        data_dir = os.path.expanduser('~/space_robotics_project/data/processed/')
        full_las_path = os.path.join(data_dir, 'downsampled_points.las')
        ground_las_path = os.path.join(data_dir, 'ground_only.las')
        pipeline_json_path = os.path.join(data_dir, 'pmf_pipeline.json')

        if not os.path.exists(ground_las_path) or os.path.getsize(ground_las_path) < 1000:
            self.get_logger().info("🩹 ground_only.las missing or invalid — running PMF...")
            self.run_pmf_pipeline(full_las_path, ground_las_path, pipeline_json_path)
        else:
            self.get_logger().info("✅ Using cached PMF result: ground_only.las")

        las_all = laspy.read(full_las_path)
        x_all, y_all, z_all = las_all.x, las_all.y, las_all.z

        self.x_offset = np.min(x_all)
        self.y_offset = np.min(y_all)
        self.get_logger().info(f"📍 Normalizing coordinates with offsets: X={self.x_offset}, Y={self.y_offset}")

        las_ground = laspy.read(ground_las_path)
        xg, yg, zg = las_ground.x, las_ground.y, las_ground.z

        grid_res = 1.0
        xi = np.arange(min(x_all), max(x_all), grid_res)
        yi = np.arange(min(y_all), max(y_all), grid_res)
        xi, yi = np.meshgrid(xi, yi)

        dsm = griddata((x_all, y_all), z_all, (xi, yi), method='nearest')
        dtm = griddata((xg, yg), zg, (xi, yi), method='nearest')
        chm = dsm - dtm
        chm[chm < 0] = 0
        chm[chm > 100] = 100

        self.plot_chm_figures(dtm, dsm, chm)
        self.visualize_clusters(x_all, y_all, z_all)

    def run_pmf_pipeline(self, input_path, output_path, pipeline_path):
        pipeline = {
            "pipeline": [
                {"type": "readers.las", "filename": input_path},
                {"type": "filters.pmf", "max_window_size": 33, "slope": 1.0,
                 "initial_distance": 0.5, "max_distance": 2.5},
                {"type": "filters.range", "limits": "Classification[2:2]"},
                {"type": "writers.las", "filename": output_path}
            ]
        }
        with open(pipeline_path, 'w') as f:
            json.dump(pipeline, f, indent=2)

        self.get_logger().info("🔍 Running PDAL PMF pipeline...")
        subprocess.run(["pdal", "pipeline", pipeline_path], check=True)
        self.get_logger().info(f"✅ PMF ground points written to {output_path}")

        try:
            las_ground = laspy.read(output_path)
            self.get_logger().info(f"📦 Total ground points written: {len(las_ground.points):,}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Couldn't read PMF output: {e}")

    def visualize_clusters(self, x, y, z):
        self.get_logger().info(f"📊 Running DBSCAN on {len(x)} points...")
        clusters = self.dbscan_clustering(x, y, z)

        centers = []
        heights = []
        biomasses = []

        for cluster_id, points in clusters.items():
            if len(points) < 3:
                continue
            height = np.max(points[:, 2]) - np.min(points[:, 2])
            crown = MultiPoint(points[:, :2]).convex_hull
            area = crown.area if crown.is_valid else 0.0
            biomass = 0.05 * (height ** 2.5) * (area ** 1.0)
            center = np.mean(points, axis=0)
            centers.append(center)
            heights.append(height)
            biomasses.append(biomass)

        x_vals = [c[0] - self.x_offset for c in centers]
        y_vals = [c[1] - self.y_offset for c in centers]
        z_vals = [c[2] for c in centers]

        fig = px.scatter_3d(
            x=x_vals, y=y_vals, z=z_vals,
            color=biomasses,
            size=heights,
            color_continuous_scale='YlGn',
            labels={'color': 'Biomass (kg)', 'size': 'Height (m)'},
            hover_data={
                'X': x_vals,
                'Y': y_vals,
                'Z': z_vals,
                'Biomass': biomasses,
                'Height': heights,
            },
            title='3D Tree Biomass Visualization'
        )

        fig.update_traces(marker=dict(opacity=0.8, line=dict(width=0)))
        fig.show()

    def dbscan_clustering(self, x, y, z, eps=4.0, min_samples=5):
        height_threshold = np.percentile(z, 90)
        mask = z > height_threshold
        x, y, z = x[mask], y[mask], z[mask]

        self.get_logger().info(f"🪓 Filtered to top 10% — {len(x):,} points remaining")

        if len(x) > 200_000:
            idx = np.random.choice(len(x), size=200_000, replace=False)
            x, y, z = x[idx], y[idx], z[idx]
            self.get_logger().info(f"🎯 Downsampled to 200,000 points for DBSCAN")

        coords = np.vstack((x, y, z)).T
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(coords)
        labels = clustering.labels_
        clusters = defaultdict(list)

        for idx, label in enumerate(labels):
            if label != -1:
                clusters[label].append(coords[idx])

        self.get_logger().info(f"🔢 DBSCAN found {len(clusters)} clusters")
        return {k: np.array(v) for k, v in clusters.items()}

    def plot_chm_figures(self, dtm, dsm, chm):
        plt.figure(figsize=(16, 4))
        plt.subplot(1, 3, 1)
        plt.imshow(dtm, cmap='terrain')
        plt.title("DTM (Ground, PMF)")
        plt.colorbar()

        plt.subplot(1, 3, 2)
        plt.imshow(dsm, cmap='gist_earth')
        plt.title("DSM (Surface)")
        plt.colorbar()

        plt.subplot(1, 3, 3)
        plt.imshow(chm, cmap='YlGn')
        plt.title("CHM (Canopy Height)")
        plt.colorbar()

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = BiomassVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
