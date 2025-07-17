# 🌾 AgroSTMap

**AgroSTMap** (Agricultural Spatio-Temporal Mapping) is a ROS 1-based framework for drone-driven crop field monitoring using LiDAR and RGB imagery. It enables precise, high-resolution 3D reconstruction of agricultural environments over time, supporting digital twin generation, growth visualization, and analysis.

---

## 📌 Overview

AgroSTMap captures weekly LiDAR scans using a **Livox Mid-360** mounted on a custom-built drone. Simultaneously, a **DJI drone** collects low-altitude RGB imagery. The framework supports spatio-temporal alignment of 3D point clouds using robust registration techniques (**NDT**, **GICP**) and fuses them with RGB image data using ground markers to build a high-fidelity **digital twin** of the crop field.

---

## 🚜 Key Features

- 🔁 **Weekly 3D Mapping** via drone-mounted Livox Mid-360
- 🛰️ **Multimodal Fusion** of LiDAR and RGB data using ground markers
- 🔄 **Spatio-Temporal Registration** with NDT and GICP modules
- 🌱 **Growth Visualization** and crop height tracking over time
- 🧠 **Post-Processing Utilities** for NDVI analysis and digital twin generation
- 🧰 Compatible with ROS 1 (tested on Ubuntu 20.04 + Noetic)

---

## 🗂 View Sample Point Clouds

You can view sample `.pcd` files collected from real drone missions at the following link:

🔗 [View PCD Files](https://pern-my.sharepoint.com/:f:/g/personal/25100190_lums_edu_pk/EhLL1Nm3fFBHkgywhFM2Cl0BDCuEzwHEMqaqhqeLgHw-xA?e=DdEJBN)

---

## 🛠 Technologies Used

- ROS 1 (Noetic)
- Livox Mid-360 LiDAR + IMU
- DJI drone (RGB image capture)
- Open3D, VTK, PyQt for visualization
- YAML-based configuration system
- PCL, Eigen3, pybind11

---

## 📈 Future Plans

- Multi-drone coordination and flight scheduling  
- Geo-referenced mapping via GPS + IMU fusion  
- Real-time dashboard for in-field feedback  
- ML models for crop growth prediction  

---

Stay tuned as we grow AgroSTMap into a scalable platform for precision agriculture and autonomous crop monitoring!
