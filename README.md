# 🌾 AgroSTMap

**AgroSTMap** is a modular ROS-based framework for high-precision, spatio-temporal crop-field mapping using aerial LiDAR and RGB/multispectral imagery.

It enables **week-to-week 3D reconstruction**, **growth visualization**, and **crop-health analytics** by combining robust state estimation, LiDAR-based dense mapping, orthomosaic generation, and **real-time NDVI** computation — all flown via **autonomous, velocity-controlled, yaw-locked missions** guided by **FAST-LIO2 + GNSS**.

---

## 📌 Overview

AgroSTMap is designed for **low-cost, repeatable, and scalable agricultural mapping**.
Given a predefined field boundary and mission profile, the system:

1. Plans and executes **autonomous, yaw-locked flights** over crop fields
2. Streams LiDAR and RGB/multispectral data while logging FAST-LIO2 odometry and GNSS
3. Builds a **3D LiDAR map** in real time using FAST-LIO2
4. Generates **orthomosaics** and **DEMs** offline using Agisoft Metashape
5. Computes **real-time NDVI** and other vegetation indices from onboard data
6. Produces **per-plot health summaries** and temporal comparisons across weeks

---

## 🛠 System Overview

### 1️⃣ State Estimation & Autonomy

- **Primary Odometry:**
  - [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) LiDAR–Inertial Odometry, running onboard
- **Global Pose Fusion:**
  - GNSS fused via an EKF to obtain drift-limited global pose
- **Flight Control:**
  - **Velocity-controlled** missions (vx, vy, vz commands)
  - **Yaw-locked** along the track (e.g., facing forward or fixed heading)
  - Waypoint-based mission execution via ROS
- **Fallback & Robustness:**
  - FAST-LIO2 continues to provide odometry in **GNSS-degraded or partially denied** environments
  - Optional AprilTag-based corrections (fiducial localization) for local drift correction (if tags are installed at field boundaries)

---

### 2️⃣ Mapping: 3D Reconstruction

- **Input:** LiDAR scans + IMU at high rate
- **Backend:** FAST-LIO2, producing:
  - Real-time **deskewed point clouds**
  - Pose estimates in a world frame
- **3D Reconstruction:**
  - Aggregation of LiDAR sweeps into a global **3D point cloud map**
  - Filtering & downsampling via voxel grids and statistical outlier removal
- **Temporal Alignment:**
  - Weekly scans are registered using:
    - **NDT (Normal Distributions Transform)**
    - **GICP (Generalized ICP)**
  - This enables:
    - Week-over-week **growth visualization**
    - Change detection across planting/harvest cycles

> 🔎 Note: AgroSTMap **does not** use Gaussian Splatting.
> All 3D reconstruction is currently LiDAR-based via FAST-LIO2.

---

### 3️⃣ Orthomosaics & DEM via Agisoft Metashape

- **RGB Image Capture:**
  - Time-synchronized RGB frames during flight (or multispectral where available)
  - Each frame is associated with:
    - GNSS/FAST-LIO2 pose
    - Timestamps for cross-modality fusion
- **Offline Processing in Agisoft Metashape:**
  - Import images + external camera positions
  - Bundle adjustment & camera calibration refinement
  - Dense point cloud generation
  - Mesh & DEM creation
  - **Orthomosaic** generation
- **Outputs:**
  - Geo-referenced orthomosaics
  - Elevation maps (DEM/DSM)
  - Optional export of tiled map layers for QGIS, etc.

---

### 4️⃣ Real-time NDVI & Vegetation Analytics

- Uses a deep-learning NDVI model trained on multispectral data but deployed on **RGB-only input**
- Odroid N2+ subscribes to the drone’s RGB image topic for real-time processing
- Frames are preprocessed and fed into an **ONNX Runtime** inference engine
- Model outputs **multispectral-level NDVI maps** directly from RGB images
- NDVI heatmaps are published as ROS topics for live visualization
- Geo-tagged NDVI values are logged using FAST-LIO2 + GNSS timestamps for field-level analysis

---

### 5️⃣ Flight Profile & Mission Flow

1. **Mission Definition**
   - User specifies:
     - Field polygon / boundary
     - Desired altitude, velocity, and track spacing
   - Waypoints are generated to create a lawnmower / boustrophedon pattern.

2. **Autonomous Flight**
   - Flight controller receives:
     - **Velocity commands** (vx, vy, vz)
     - **Yaw-lock constraint** (fixed heading or aligned with track)
   - FAST-LIO2 + GNSS provide continuous position feedback.
   - Safety interlocks for:
     - Geo-fence violation
     - GNSS loss
     - Low battery

3. **Data Logging**
   - ROS bag captures:
     - LiDAR scans
     - IMU data
     - GNSS
     - Camera images
     - FAST-LIO2 odometry
   - Optional onboard compression.

4. **Post-Processing**
   - Run reconstruction scripts to:
     - Build 3D LiDAR map
     - Register week-to-week maps
   - Export camera poses for Metashape
   - Generate orthomosaics and DEMs
   - Aggregate NDVI/health metrics.

---

## 🧱 Tested Hardware Setup

- **Airframe:** 500 mm quadrotor
- **LiDAR:** Livox Mid360
- **Cameras:**
  - RGB camera (global or rolling shutter)
  - Optional NIR / multispectral camera for NDVI
- **Navigation Stack:**
  - GNSS receiver
  - IMU
  - Onboard flight controller (e.g., PX4/ArduPilot)
- **Onboard Compute:**
  - **Odroid N2+** running:
    - Ubuntu + ROS 1 Noetic
    - FAST-LIO2
    - Custom mapping & NDVI nodes

---

## 🧪 Sample Data

Sample point clouds from real flight missions are available for preview and analysis:

🔗 **[View Sample PCD Files](https://pern-my.sharepoint.com/:f:/g/personal/25100190_lums_edu_pk/Eqw-5kNBIQZOh_JnwwTozbwBUMuggt8dp-Hl3OUpcNrq-A?e=FWa7TT)**

These include:
- Sample Rosbags
- Orthomosaics from Agisoft Metashape
- Example point clouds suitable for downstream experiments or benchmarking

---

## 🚧 Roadmap / Future Work

- **Deep-Learning Crop Analytics**
  - Plug-and-play U-Net / SegVeg for canopy segmentation
  - Disease/stress region detection on orthomosaics and NDVI layers

- **Top-View Heatmap Pipeline**
  - Neural pipeline to convert orthomosaics + NDVI into **crop-health heatmaps**
  - Identification of underperforming or stressed regions at plot-level resolution

- **Closed-Loop Reinspection**
  - Use health-map hotspots to trigger:
    - **Targeted reinspection flights**
    - Ground robot missions for close-up imaging
  - Navigation guided by FAST-LIO2 + GNSS (and optional AprilTags)

- **Real-time Edge Feedback**
  - In-field feedback to farmers:
    - Simple visualizations (e.g., traffic-light style)
    - Suggestions for irrigation, fertilization, or further inspection

---
