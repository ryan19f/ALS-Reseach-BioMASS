# Forest Ground Biomass Detection Using ALS Point Cloud

This ROS 2 package provides tools for the detection and estimation of forest biomass using Airborne Laser Scanning (ALS) point cloud data. The system processes LiDAR data to identify individual trees, extract key parameters, and calculate biomass using both allometric and machine learning models.

## 📋 Project Overview

The project implements a complete pipeline for forest biomass estimation:

1. **Point Cloud Preprocessing**: Filters and processes ALS data
2. **Terrain Modeling**: Generates DTM, DSM, and CHM
3. **Tree Segmentation**: Identifies individual trees from point cloud data
4. **Parameter Extraction**: Calculates tree height and crown area
5. **Biomass Estimation**: Uses allometric equations and/or ML models
6. **ROS 2 Integration**: Provides real-time processing and visualization

## 🚀 Key Features

- **Individual Tree Detection**: Segments and identifies discrete trees in point cloud data
- **Dual Estimation Methods**: Implements both allometric and ML-based biomass models
- **ROS 2 Visualization**: Real-time tree visualization with biomass data in RViz
- **Complete Processing Pipeline**: From raw point cloud to biomass estimates

## 📊 Biomass Estimation Methods

### Allometric Model

The package uses the following allometric equation for biomass estimation:

```
Biomass (kg) = 0.05 * (Height^2.5) * (Crown_Area^1.0)
```

Where:
- Height (H): Tree height in meters
- Crown_Area (A): 2D projection of canopy area in square meters

### Machine Learning Model

For comparison and improved accuracy in heterogeneous forests, a Random Forest regressor model is also implemented with the following characteristics:

- **Algorithm**: RandomForestRegressor
- **Features**: Tree height, crown area, and additional metrics
- **Performance**: R² score of 0.83, RMSE of 55,557.43 kg

## 🖥️ ROS 2 Architecture

The package provides the following ROS 2 components:

- **Nodes**:
  - `biomass_node`: Main processing node for biomass estimation
  
- **Topics**:
  - `/als/pointcloud`: Input topic for point cloud data (subscribes)
  - `/tree_biomass_markers`: Output topic with biomass visualization (publishes)
  
- **Message Types**:
  - `sensor_msgs/PointCloud2`: For raw point cloud data
  - `visualization_msgs/MarkerArray`: For tree visualization with biomass data

## 🔧 Installation

### Prerequisites

- ROS 2 Humble or newer
- Python 3.10+
- Required Python packages: numpy, laspy, shapely

### Building the Package

```bash
# Clone the repository into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/your-username/tree_biomass_rviz.git

# Install dependencies
pip install laspy numpy shapely

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select tree_biomass_rviz

# Source the workspace
source install/setup.bash
```

## 🎮 Usage

### Running the Node

```bash
# Launch the biomass estimation node
ros2 launch tree_biomass_rviz tree_biomass.launch.py

# In another terminal, visualize the results in RViz2
ros2 run rviz2 rviz2
```

### Configuring RViz

1. Add a MarkerArray display
2. Set the topic to `/tree_biomass_markers`
3. Set the Fixed Frame to `map`

### Input Data

The system expects LAS-format point cloud data at:
```
~/space_robotics_project/data/processed/downsampled_points.las
```

## 🛠️ Implementation Details

### Point Cloud Processing

The `biomass_node.py` implements:
- Point cloud loading and validation
- Simple clustering using a grid-based approach
- Parameter extraction (height and crown area)
- Biomass calculation and visualization

For large-scale deployment, the clustering algorithm can be replaced with more sophisticated methods like DBSCAN or watershed segmentation.

### Biomass Visualization

Trees are visualized in RViz as:
- Cylinders with height proportional to the tree height
- Text labels showing calculated biomass in kg
- Green color indicating vegetation

## 🔍 Results and Outputs

The system produces several outputs:
- RViz visualization with tree markers and biomass labels
- Console logging of detected trees and their parameters
- (In full implementation) Geospatial biomass maps and CSV data

![RVIZ1](https://github.com/ryan19f/ALS-Reseach-BioMASS/blob/main/tree_biomass_rviz/rviz1.jpeg)
Shows BioMass of each tree
![RVIZ3](https://github.com/ryan19f/ALS-Reseach-BioMASS/blob/main/tree_biomass_rviz/rviz2.jpeg)
![RVIZ3](https://github.com/ryan19f/ALS-Reseach-BioMASS/blob/main/tree_biomass_rviz/rviz3.jpeg)
Pointcloud of ofrest 

## 👥 Team

- Rhutvik Prashant Pachghare - Biomass estimation implementation with ML model
- Shyam Kamlesh Ganatra - ROS 2 integration and visualization
- Ryan Fernandes - Biomass estimation using allometric model, filter model implementation
- Vamshikrishna Gadde - General Research work for ALS & documentation

## 🔗 References

- Chave et al. (2014): General allometric models for AGB
- Jucker et al. (2017): Height & crown-based models for biomass estimation
- ROS 2 Documentation: https://docs.ros.org/en/humble/
