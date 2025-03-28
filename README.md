# ASL-Reseach-BioMASS(Midterm Project Report)

## Forest Ground Biomass Detection Using ALS Point Cloud
**By: Rhutvik Prashant Pachghare, Ryan Fernandes & Shyam Ganatra**

---

## 1. Introduction
Accurate estimation of forest ground biomass is essential for ecological monitoring, carbon sequestration studies, and sustainable forest management. Airborne Laser Scanning (ALS), a remote sensing technique, provides high-resolution three-dimensional point cloud data, enabling precise biomass estimation. This project explores the application of ALS point cloud data for forest ground biomass detection, highlighting methodologies, challenges, and potential applications.

## 2. Objectives
- Utilize ALS point cloud data to estimate forest ground biomass.
- Develop an efficient methodology for ground point classification and biomass calculation.
- Assess the accuracy and effectiveness of different filtering and segmentation techniques.
- Compare the results with traditional biomass estimation methods.

## 3. Methodology

### 3.1 Data Acquisition
- ALS data collected from a forested area using a LiDAR-equipped UAV or airborne platform.
- Preprocessing to remove noise and non-relevant data points.

### 3.2 Ground Point Classification
- Ground filtering algorithms such as Progressive Morphological Filtering (PMF) and Cloth Simulation Filtering (CSF).
- Identification of ground and non-ground points using Digital Terrain Model (DTM) generation.

### 3.3 Biomass Estimation
- Extraction of tree height and canopy structure from ALS data.
- Application of allometric equations to estimate above-ground biomass.
- Integration with field-measured biomass data for validation.

## 4. Results and Discussion
- Preliminary results showing the effectiveness of ALS data in biomass detection.
- Comparison of different ground classification techniques.
- Error analysis and potential improvements for future studies.

## 5. Challenges and Limitations
- Accuracy of ground classification in dense forest regions.
- Variability in biomass estimation due to different tree species.
- Computational complexity of processing large ALS datasets.

## 6. Future Work
- Implementation of deep learning techniques for enhanced biomass estimation.
- Integration with multi-sensor data, including hyperspectral imaging.
- Application to large-scale forest biomass monitoring.

## 7. Conclusion
ALS-based biomass detection presents a promising approach for accurate and efficient forest monitoring. This study demonstrates the potential of ALS point cloud data in detecting and estimating ground biomass, contributing to improved environmental and resource management strategies.

## 8. References
1. *Forest above-ground biomass estimation based on strongly heterogeneous sample plots using airborne laser scanning*  
   This study explores the use of ALS for precise estimation of forest above-ground biomass (AGB) in heterogeneous forest conditions.

2. *Estimating biomass and soil carbon change at the level of forest stands using repeated forest surveys assisted by airborne laser scanner data*  
   This research integrates field measurements and ALS data to estimate changes in forest carbon pools, emphasizing the importance of repeated surveys for accurate assessment.

3. *Forest biomass estimation from airborne LiDAR data using machine learning approaches*  
   This study compares the effectiveness of various modeling techniques for estimating biomass in moderately dense forest conditions using airborne LiDAR data.

4. *Modelling aboveground forest biomass using airborne laser scanner data: A comparison of parametric and non-parametric approaches*  
   This research compares different modeling techniques for estimating aboveground biomass using ALS data, providing insights into the strengths and weaknesses of each method.

5. *Estimation of forest biomass components using airborne LiDAR and multispectral sensors*  
   This study estimates biomass in a Scots pine-dominated forest by combining LiDAR data with multispectral imagery and allometric equations.

6. *A brief overview and perspective of using airborne Lidar data for forest biomass estimation*  
   This paper provides an overview of using airborne Lidar data for forest biomass estimation and discusses current research problems and future directions.

7. *Estimating above-ground biomass of subtropical forest using airborne LiDAR and hyperspectral data*  
   This study compares traditional allometric modeling and LiDAR plot metrics for estimating above-ground biomass in a subtropical forest, highlighting the advantages of integrating multiple data sources.

8. *Airborne laser scanner-assisted estimation of aboveground biomass change in a boreal forest*  
   This paper discusses the use of repeated ALS acquisitions to estimate changes in aboveground tree biomass, highlighting the method's effectiveness in monitoring forest dynamics.

9. *LiDAR Applications to Estimate Forest Biomass at Individual Tree Level*  
   This research summarizes the use of various laser scanning techniques, including airborne laser scanning, for estimating individual tree volume and aboveground biomass.

10. *Improving living biomass C-stock loss estimates by combining optical satellite, airborne laser scanning, and NFI data*  
    This research combines optical satellite data, ALS, and National Forest Inventory data to enhance estimates of carbon stock losses in forests.
