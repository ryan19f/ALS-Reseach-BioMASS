# Forest Ground Biomass Detection Using Photogrammetric Point Cloud

**By: Rhutvik Prashant Pachghare, Ryan Fernandes, Vamshikrishna Gadde & Shyam Ganatra**

## Abstract  
This report explores the use of **photogrammetrically generated point clouds** for estimating forest biomass. While **Airborne Laser Scanning (ALS)** provides high-resolution 3D data, it is often inaccessible for direct testing. Instead, this study focuses on using photogrammetry-based point clouds as an alternative, generated through UAV-based imagery. Additionally, simulations using LiDAR-equipped drones will be explored to validate methodologies. The workflow, from data acquisition to validation, is outlined, integrating recent advancements in data processing and machine learning to enhance biomass estimation accuracy.

## 1. Introduction  
Forest biomass estimation is crucial for **carbon stock assessment, forest management, and climate change studies**. Traditional field-based measurements are labor-intensive and time-consuming. **Photogrammetric point clouds**, generated from UAV imagery, offer a cost-effective alternative to LiDAR, enabling detailed biomass estimation while maintaining accessibility for field testing.

## 2. Data Acquisition and Preprocessing  
Point cloud data is generated using **photogrammetry** by capturing high-resolution aerial images from a UAV and processing them into a **3D model**. For validation, a simulated environment with a **LiDAR-equipped drone** will be used. Preprocessing involves:  
- **Noise filtering, normalization, and georeferencing** to ensure data accuracy.  
- **Structure-from-Motion (SfM) techniques** to reconstruct 3D terrain and tree structures.  
- **Filtering techniques like Progressive Morphological Filtering (PMF) and Cloth Simulation Filtering (CSF)** for ground classification.

## 3. Ground Classification  
Separating ground and non-ground points is essential for accurate height estimation. **PMF and CSF** techniques are used to refine the point cloud and ensure precise tree height measurements.

## 4. Biomass Estimation Techniques  
Biomass estimation is carried out by extracting tree height, crown area, and canopy volume from the point cloud. **Allometric equations** are applied to estimate **above-ground biomass (AGB)**. Additionally, **machine learning models like Random Forest (RF) and Support Vector Regression (SVR)** improve estimation accuracy compared to traditional parametric methods. The estimation follows the general form:  
```math
Biomass = a \times (DBH^b \times H^c)
```

## 5. Validation and Accuracy Assessment

Comparison with field-based measurements to evaluate model performance.

Simulation using LiDAR-equipped drones to test methodologies in a controlled setting.

Accuracy metrics such as RMSE and R² to assess model reliability.

## 6. Conceptual Workflow of Biomass Estimation

Data Acquisition

UAV-based photogrammetry captures aerial images.

Point cloud generated using SfM techniques.

Simulated LiDAR data for validation.

Preprocessing

Noise filtering and normalization.

Ground classification using PMF and CSF.

Biomass Estimation

Extract tree height and canopy structure.

Apply allometric equations and machine learning models.

Validation

Compare photogrammetry-based biomass estimates with field data.

Test methodologies in simulation with LiDAR-equipped UAVs.

## 7. Conclusion

Photogrammetry-based point clouds offer an accessible alternative to airborne LiDAR for forest biomass estimation. Integrating simulation-based LiDAR validation further enhances methodology robustness. Future work should focus on improving data fusion techniques and AI-driven models for enhanced biomass monitoring.
