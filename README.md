## 📌 Overview

**AgroSTMap** is a modular ROS-based framework for high-precision, spatio-temporal crop-field mapping using aerial LiDAR and RGB imagery.
It enables week-to-week 3D reconstruction, growth visualization, and crop-health analytics by combining state estimation, dense mapping, and deep-learning-based vegetation analysis.

### System Overview
- **State Estimation:** FAST-LIO2 odometry fused with GNSS EKF and fiducial (AprilTag) corrections for drift-free localization in GNSS-degraded or GPS-denied environments
- **Mapping:** Gaussian-splatting mapper for dense 3D reconstruction with sub-decimeter accuracy
- **Registration:** NDT and GICP modules for temporal alignment of weekly scans
- **Analytics:** U-Net and SegVeg models for canopy segmentation, disease detection, and per-plot health indices
- **Visualization:** DEM and orthomosaic generation in Open3D / Agisoft Metashape
- **Tested Setup:** 500 mm quadrotor equipped with Livox Mid360 LiDAR, Intel RealSense RGB-D, GNSS/IMU fusion, and onboard Odroid N2+ computer (ROS 1 Noetic)

### Future Work
- Integration of **Gaussian Splatting SLAM (GS-LIVO)** to replace FAST-LIO2 for photometric and geometric fusion during mapping
- Development of a **top-view analysis pipeline**: a neural network will process aerial imagery to generate **crop-health heatmaps**, identifying underperforming or stressed regions
- Using heatmap indices to guide **targeted reinspection** — the drone or ground robot will autonomously navigate to affected zones using prior localization (FAST-LIO + GNSS + AprilTags) to capture close-up imagery for detailed assessment
- Expansion toward real-time inference and in-field feedback for precision agriculture and sustainability-focused interventions

---

## 🗂 Sample Data

Sample point clouds from real flight missions are available for preview and analysis:

🔗 **[View Sample PCD Files](https://pern-my.sharepoint.com/:f:/g/personal/25100190_lums_edu_pk/Eqw-5kNBIQZOh_JnwwTozbwBUMuggt8dp-Hl3OUpcNrq-A?e=FWa7TT)**
