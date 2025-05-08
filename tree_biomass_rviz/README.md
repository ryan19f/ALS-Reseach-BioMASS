# Technical Documentation: Tree Biomass RViz Node

This document provides technical details on the implementation of the ROS 2 node for forest biomass estimation and visualization.

## 1. Package Structure

```
tree_biomass_rviz/
├── launch/
│   └── tree_biomass.launch.py  # Launch file for the biomass node
├── resource/
│   └── tree_biomass_rviz       # Package marker file
├── tree_biomass_rviz/
│   ├── __init__.py             # Package initialization
│   └── biomass_node.py         # Main node implementation
├── test/
│   ├── test_copyright.py       # Copyright test
│   ├── test_flake8.py          # Code style test
│   └── test_pep257.py          # Docstring test
├── package.xml                 # Package manifest
├── setup.cfg                   # Python package configuration
└── setup.py                    # Package setup script
```

## 2. Node Implementation (`biomass_node.py`)

### 2.1 Class Overview: `BiomassNode`

The `BiomassNode` class inherits from `rclpy.node.Node` and implements:
- Point cloud processing
- Tree segmentation
- Biomass calculation
- RViz visualization

### 2.2 Initialization

```python
def __init__(self):
    super().__init__('biomass_node')
    self.publisher = self.create_publisher(MarkerArray, 'tree_biomass_markers', 10)
    
    self.get_logger().info('🌲 Tree Biomass RViz Node Started')
    self.run_pipeline()
```

The node initializes a publisher for tree markers and immediately runs the processing pipeline.

### 2.3 Pipeline Execution (`run_pipeline`)

The main processing pipeline:

1. **Load point cloud data** from LAS file
   ```python
   las_path = os.path.expanduser('~/space_robotics_project/data/processed/downsampled_points.las')
   las = laspy.read(las_path)
   x, y, z = las.x, las.y, las.z
   ```

2. **Perform tree clustering**
   ```python
   clusters = self.fake_clustering(x, y, z, cell_size=3.0)
   ```

3. **Calculate biomass** for each tree cluster
   ```python
   height = np.max(points[:, 2]) - np.min(points[:, 2])
   crown = MultiPoint(points[:, :2]).convex_hull
   area = crown.area if crown.is_valid else 0.0
   biomass = 0.05 * (height ** 2.5) * (area ** 1.0)
   ```

4. **Create visualization markers** for each tree
   ```python
   marker = Marker()
   marker.header = Header()
   marker.header.frame_id = "map"
   marker.id = cluster_id
   marker.type = Marker.CYLINDER
   marker.action = Marker.ADD
   # [marker configuration...]
   ```

5. **Publish markers** for RViz visualization
   ```python
   self.publisher.publish(marker_array)
   ```

### 2.4 Tree Clustering (`fake_clustering`)

The current implementation uses a simple grid-based clustering approach:

```python
def fake_clustering(self, x, y, z, cell_size=5.0, min_points=50):
    from collections import defaultdict
    grid = defaultdict(list)
    
    # Assign points to grid cells
    for i in range(len(x)):
        key = (int(x[i] // cell_size), int(y[i] // cell_size))
        grid[key].append([x[i], y[i], z[i]])
    
    # Convert grid cells to clusters
    cluster_count = 0
    clustered = {}
    
    for i, pts in enumerate(grid.values()):
        if len(pts) >= min_points:
            clustered[i] = np.array(pts)
            cluster_count += 1
    
    return clustered
```

This approach:
- Divides the XY plane into grid cells
- Groups points that fall into the same cell
- Filters cells with too few points
- Returns each cell as a separate "tree"

**Note:** This is a simplified approach. For production use, more sophisticated algorithms like DBSCAN or watershed segmentation should be implemented.

## 3. ROS 2 Integration

### 3.1 Node Registration

The node is registered as an entry point in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'biomass_node = tree_biomass_rviz.biomass_node:main',
    ],
},
```

### 3.2 Launch File

The launch file (`tree_biomass.launch.py`) starts the biomass node:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tree_biomass_rviz',
            executable='biomass_node',
            name='biomass_node',
            output='screen'
        )
    ])
```

### 3.3 Dependencies

The package dependencies are specified in `package.xml`:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>visualization_msgs</depend>
```

## 4. Visualization Details

### 4.1 Tree Representation

Each tree is visualized as:

1. **Cylinder marker**
   - Position: Mean XYZ of the cluster points
   - Height: Maximum Z-range of the cluster
   - Color: Green (0, 1, 0) with 0.8 alpha

2. **Text label**
   - Position: Above the tree
   - Content: Biomass value in kg
   - Color: White (1, 1, 1) with 1.0 alpha

### 4.2 Marker Configuration

```python
# Cylinder for tree trunk/canopy
marker = Marker()
marker.header.frame_id = "map"
marker.id = cluster_id
marker.type = Marker.CYLINDER
marker.action = Marker.ADD
marker.scale.x = 1.5  # Diameter
marker.scale.y = 1.5  # Diameter
marker.scale.z = float(height)  # Height

# Text label for biomass value
text_marker = Marker()
text_marker.header.frame_id = "map"
text_marker.id = cluster_id + 100000  # Unique ID
text_marker.type = Marker.TEXT_VIEW_FACING
text_marker.action = Marker.ADD
text_marker.scale.z = 5.0  # Text height
text_marker.text = f'{biomass:.0f} kg'
```

## 5. Biomass Calculation Details

The biomass is calculated using the allometric equation:

```
Biomass (kg) = 0.05 * (Height^2.5) * (Crown_Area^1.0)
```

Where:
- **Height**: Maximum vertical extent of the cluster (meters)
- **Crown Area**: Area of the convex hull of the XY projection (square meters)
- **Constants**:
  - 0.05: Scaling factor
  - 2.5: Height exponent (reflects volumetric scaling)
  - 1.0: Area exponent (linear relationship with crown projection)

## 6. Future Improvements

### 6.1 Tree Segmentation

- Replace grid-based clustering with DBSCAN or watershed segmentation
- Implement canopy height model (CHM) based segmentation
- Add ground point classification

### 6.2 Biomass Estimation

- Add ML-based biomass estimation alongside allometric model
- Include species-specific allometric parameters
- Incorporate additional parameters like stem diameter

### 6.3 ROS 2 Functionality

- Add service for on-demand processing
- Create parameter server for configurable settings
- Implement online point cloud processing through `/als/pointcloud` subscription

## 7. Troubleshooting

### 7.1 Common Issues

1. **LAS file not found**
   - Ensure the path `~/space_robotics_project/data/processed/downsampled_points.las` exists
   - Check file permissions

2. **Visualization not appearing in RViz**
   - Verify the Fixed Frame is set to "map"
   - Check that the MarkerArray display is added and configured correctly

3. **Python environment issues**
   - Ensure all dependencies are installed: `pip install laspy numpy shapely`
   - Check ROS 2 environment is properly sourced

### 7.2 Logging

The node includes detailed logging to help with troubleshooting:

```
[INFO] [biomass_node]: 🌲 Tree Biomass RViz Node Started
[INFO] [biomass_node]: 🔍 Loading LAS file from: /home/user/space_robotics_project/data/processed/downsampled_points.las
[INFO] [biomass_node]: ✅ Loaded 1000000 points
[INFO] [biomass_node]: 🔢 Grid cells checked: 1000, kept: 150
[INFO] [biomass_node]: 🌲 Found 150 tree clusters
[INFO] [biomass_node]: 🧭 Marker 0: x=123.45, y=678.90, z=45.67, height=25.30, biomass=1234.56kg
...
[INFO] [biomass_node]: Published 300 tree markers
```
