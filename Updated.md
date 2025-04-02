# (Midterm Project Report)
# Forest Ground Biomass Detection Using Photogrammetric Point Cloud

**By: Rhutvik Prashant Pachghare, Ryan Fernandes & Shyam Ganatra**

## Abstract  
This report explores the use of **photogrammetrically generated point clouds** for estimating forest biomass. While **Airborne Laser Scanning (ALS)** provides high-resolution 3D data, it is often inaccessible for direct testing. Instead, this study focuses on using photogrammetry-based point clouds as an alternative, generated through UAV-based imagery. Additionally, simulations using LiDAR-equipped drones will be explored to validate methodologies. The workflow, from data acquisition to validation, is outlined, integrating recent advancements in data processing and machine learning to enhance biomass estimation accuracy.

## 1. Introduction  
Forest biomass estimation is crucial for **carbon stock assessment, forest management, and climate change studies**. Traditional field-based measurements are labor-intensive and time-consuming. **Photogrammetric point clouds**, generated from UAV imagery, offer a cost-effective alternative to LiDAR, enabling detailed biomass estimation while maintaining accessibility for field testing.

![image](https://github.com/user-attachments/assets/1a51ff54-a2a9-421a-8289-30bf74ad2903)

## 2. Data Acquisition and Preprocessing  
Point cloud data is generated using **photogrammetry** by capturing high-resolution aerial images from a UAV and processing them into a **3D model**. For validation, a simulated environment with a **LiDAR-equipped drone** will be used. Preprocessing involves:  
- **Noise filtering, normalization, and georeferencing** to ensure data accuracy.  
- **Structure-from-Motion (SfM) techniques** to reconstruct 3D terrain and tree structures.  
- **Filtering techniques like Progressive Morphological Filtering (PMF) and Cloth Simulation Filtering (CSF)** for ground classification.

## 3. Ground Classification  
Separating ground and non-ground points is essential for accurate height estimation. **PMF and CSF** techniques are used to refine the point cloud and ensure precise tree height measurements.

![Screen Shot 2025-03-27 at 23 20 13 PM](https://github.com/user-attachments/assets/8fd0e1dc-a48d-46ef-b63e-0a219aaf0020)

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

## * Data Acquisition

  UAV-based photogrammetry captures aerial images.

  Point cloud generated using SfM techniques.

  Simulated LiDAR data for validation.

## * Preprocessing

Noise filtering and normalization.

Ground classification using PMF and CSF.

## * Biomass Estimation

Extract tree height and canopy structure.

Apply allometric equations and machine learning models.

## * Validation

Compare photogrammetry-based biomass estimates with field data.

Test methodologies in simulation with LiDAR-equipped UAVs.

## 7. Conclusion

Photogrammetry-based point clouds offer an accessible alternative to airborne LiDAR for forest biomass estimation. Integrating simulation-based LiDAR validation further enhances methodology robustness. Future work should focus on improving data fusion techniques and AI-driven models for enhanced biomass monitoring.

## 8. References

1. **Zhao et al., 2024** - *"Airborne LiDAR for Biomass Estimation: Advances and Challenges"*  [Link](https://www.sciencedirect.com/science/article/pii/S1470160X24009749)  
2. **Korhonen et al., 2023** - *"Forest Biomass Estimation with Airborne Laser Scanning: A Review of Methods and Applications"*  [Link](https://cbmjournal.biomedcentral.com/articles/10.1186/s13021-023-00222-4)  
3. **Yu et al., 2012** - *"LiDAR Point Cloud Classification Using Progressive Morphological Filtering"*  [Link](https://www.sciencedirect.com/science/article/abs/pii/S0034425712002787)  
4. **Heiskanen et al., 2012** - *"Ground Classification and Biomass Estimation Using Airborne Laser Scanning Data"*  [Link](https://www.sciencedirect.com/science/article/abs/pii/S0034425712002787)  
5. **Gonzalez-Ferreiro et al., 2015** - *"Use of ALS-Derived Metrics for Estimating Above-Ground Biomass in Forest Ecosystems"*   [Link](https://cbmjournal.biomedcentral.com/articles/10.1186/s13021-015-0037-2)  
6. **Vastaranta et al., 2012** - *"Applying Machine Learning Algorithms to Improve Biomass Estimation from ALS Data"*  [Link](https://iforest.sisef.org/abstract/?id=ifor2735-012)  
7. **Zhang et al., 2021** - *"Integrating Multispectral and ALS Data for Enhanced Forest Biomass Prediction"*  [Link](https://www.nature.com/articles/s41598-021-81267-8)
