# (Midterm Project Report)

## Forest Ground Biomass Detection Using ALS Point Cloud

**By:** Rhutvik Prashant Pachghare, Ryan Fernandes, Vamshikrishna Gadde & Shyam Ganatra

## Abstract
This report explores the use of Airborne Laser Scanning (ALS) for estimating forest biomass. ALS provides high-resolution three-dimensional (3D) data, enabling precise calculations of tree height, canopy structure, and biomass distribution. Recent advancements in data processing and machine learning have improved estimation accuracy. This report outlines the workflow from data acquisition to validation while incorporating recent research findings to enhance the methodology.

## 1. Introduction
Forest biomass estimation plays a crucial role in carbon stock assessment, forest management, and climate change studies. Traditional field-based measurements are time-consuming and labor-intensive. ALS technology provides a cost-effective and efficient alternative, allowing for large-scale biomass estimation with high accuracy ([Zhao et al., 2024](https://www.sciencedirect.com/science/article/pii/S1470160X24009749)).

![image](https://github.com/user-attachments/assets/1a51ff54-a2a9-421a-8289-30bf74ad2903)


## 2. Data Acquisition and Preprocessing
ALS data is collected using LiDAR-equipped UAVs or aircraft scanning forested regions ([Korhonen et al., 2023](https://cbmjournal.biomedcentral.com/articles/10.1186/s13021-023-00222-4)). The obtained point cloud data undergoes preprocessing, which includes noise filtering, normalization, and georeferencing. Advanced filtering techniques such as Progressive Morphological Filtering (PMF) and Cloth Simulation Filtering (CSF) are widely used to classify ground and non-ground points ([Yu et al., 2012](https://www.sciencedirect.com/science/article/abs/pii/S0034425712002787)).

## 3. Ground Classification
Ground classification is a critical step in biomass estimation. Algorithms like PMF and CSF help separate ground and non-ground points efficiently. These methods refine the point cloud data, ensuring accurate height calculations necessary for biomass modeling ([Heiskanen et al., 2012](https://www.sciencedirect.com/science/article/abs/pii/S0034425712002787)).

![Screen Shot 2025-03-27 at 23 20 13 PM](https://github.com/user-attachments/assets/8fd0e1dc-a48d-46ef-b63e-0a219aaf0020)


## 4. Biomass Estimation Techniques
To estimate biomass, tree height, crown area, and canopy volume are extracted from ALS point cloud data. Allometric equations are applied to calculate above-ground biomass (AGB) ([Gonzalez-Ferreiro et al., 2015](https://cbmjournal.biomedcentral.com/articles/10.1186/s13021-015-0037-2)). Machine learning models, including Random Forest (RF) and Support Vector Regression (SVR), have been shown to improve accuracy compared to traditional parametric methods ([Vastaranta et al., 2012](https://iforest.sisef.org/abstract/?id=ifor2735-012)).

Biomass estimation often uses allometric equations, which relate biomass to easily measurable tree characteristics like diameter at breast height (DBH) and height, following the general form of Biomass = a * (DBH^b * H^c), where a, b, and c are parameters specific to the species and ecosystem

## 5. Validation and Accuracy Assessment
The estimated biomass is validated using field-based measurements and statistical comparisons. Studies show that integrating ALS data with multispectral or hyperspectral imagery enhances prediction accuracy ([Zhang et al., 2021](https://www.nature.com/articles/s41598-021-81267-8)). Accuracy is evaluated using metrics such as RMSE (Root Mean Squared Error) and R^2 values, which indicate model reliability ([Heiskanen et al., 2012](https://www.sciencedirect.com/science/article/abs/pii/S0034425712002787)).

## 6. Conceptual Workflow of Biomass Estimation
1. **Data Acquisition**
   - UAV or aircraft equipped with LiDAR scans a forested area.
   - Collects high-resolution 3D point cloud data.
2. **Preprocessing**
   - Filters out noise and irrelevant points.
   - Enhances data quality for further processing.
3. **Ground Classification**
   - Uses PMF and CSF techniques.
   - Separates ground and non-ground points.
4. **Biomass Estimation**
   - Extracts tree height and canopy structure from ALS data.
   - Applies allometric equations to estimate above-ground biomass.
5. **Validation**
   - Compares ALS-based biomass estimates with field measurements.
   - Assesses accuracy and refines the model if necessary.

## 7. Conclusion
ALS has revolutionized forest biomass estimation, offering a non-destructive, scalable, and accurate approach. Future research should focus on improving data fusion techniques, incorporating AI-driven models, and integrating temporal ALS acquisitions to monitor biomass dynamics more effectively.

## 8. References
1. **Zhao et al., 2024** - *"Airborne LiDAR for Biomass Estimation: Advances and Challenges"*  [Link](https://www.sciencedirect.com/science/article/pii/S1470160X24009749)  
2. **Korhonen et al., 2023** - *"Forest Biomass Estimation with Airborne Laser Scanning: A Review of Methods and Applications"*  [Link](https://cbmjournal.biomedcentral.com/articles/10.1186/s13021-023-00222-4)  
3. **Yu et al., 2012** - *"LiDAR Point Cloud Classification Using Progressive Morphological Filtering"*  [Link](https://www.sciencedirect.com/science/article/abs/pii/S0034425712002787)  
4. **Heiskanen et al., 2012** - *"Ground Classification and Biomass Estimation Using Airborne Laser Scanning Data"*  [Link](https://www.sciencedirect.com/science/article/abs/pii/S0034425712002787)  
5. **Gonzalez-Ferreiro et al., 2015** - *"Use of ALS-Derived Metrics for Estimating Above-Ground Biomass in Forest Ecosystems"*   [Link](https://cbmjournal.biomedcentral.com/articles/10.1186/s13021-015-0037-2)  
6. **Vastaranta et al., 2012** - *"Applying Machine Learning Algorithms to Improve Biomass Estimation from ALS Data"*  [Link](https://iforest.sisef.org/abstract/?id=ifor2735-012)  
7. **Zhang et al., 2021** - *"Integrating Multispectral and ALS Data for Enhanced Forest Biomass Prediction"*  [Link](https://www.nature.com/articles/s41598-021-81267-8)

