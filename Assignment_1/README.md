# Assignment 1 – Trajectory Reconstruction & Environment Mapping

## 📌 Overview
This assignment focuses on analyzing ROS2 bag data to reconstruct the trajectory of a TurtleBot3 Burger, classify its motion patterns, estimate the control inputs, and reproduce the trajectory using different execution methods. Additionally, the surrounding environment is reconstructed using 2D LiDAR data.

---

## 🎯 Objectives
- Reconstruct robot trajectory from odometry data  
- Estimate linear and angular velocities  
- Classify motion patterns (straight, left turn, right turn)  
- Reconstruct the control input (/cmd_vel)  
- Compare different publishers (MATLAB, Python, Simulink)  
- Map the environment using LiDAR data  

---

## ⚙️ Methodology

### 🔹 Trajectory Reconstruction
- Extracted position data from ROS2 `/odom` topic  
- Computed velocities through numerical differentiation  
- Filtered noise and smoothed angular velocity  
- Classified motion based on angular velocity sign  

### 🔹 Control Estimation
- Estimated control inputs *(v, ω)* from reconstructed trajectory  
- Interpolated signals from 50 Hz to 10 Hz to match control frequency  
- Exploited differential flatness to reconstruct system inputs  

### 🔹 Replay & Comparison
- Applied estimated control sequence to the robot  
- Tested different publishers:
  - MATLAB  
  - Python  
  - Simulink  
- Evaluated performance based on trajectory tracking accuracy  

### 🔹 LiDAR Mapping
- Processed `/scan`, `/odom`, and `/tf_static` topics  
- Transformed LiDAR data into global reference frame  
- Reconstructed the environment by aggregating scans over time  

---

## 📊 Results

### Trajectory & Motion Classification
- Successfully reconstructed the robot trajectory  
- Motion classified into:
  - Straight motion  
  - Left turns  
  - Right turns  

### Control Reconstruction
- Maximum linear velocity ≈ 0.1 m/s  
- Maximum angular velocity ≈ 0.3 rad/s  
- Control sequence successfully estimated and reproduced  

### Publisher Comparison

| Publisher | RMS Error [m] | Final Error [m] |
|----------|-------------|----------------|
| MATLAB   | 0.0439      | 0.1728         |
| Python   | 0.0256      | 0.0926         |
| Simulink | 0.0077      | 0.0181         |

- Simulink achieved the best performance due to stable timing  
- MATLAB showed highest error due to frequency inconsistency  
- Python provided intermediate performance  

### LiDAR Mapping
- Environment successfully reconstructed  
- Walls clearly identified  
- Partial reconstruction of obstacles due to limited sensor coverage  

---

## 🧩 Code Structure

- `trajectory_reconstruction.m`  
  Extracts and processes odometry data to reconstruct trajectory  

- `control_estimation.m`  
  Computes linear and angular velocities and reconstructs control inputs  

- `publisher_matlab.m / publisher_python.py`  
  Sends control commands to the robot  

- `simulink_model.slx`  
  Executes control sequence with precise timing  

- `lidar_mapping.m`  
  Reconstructs environment from LiDAR data  

---

## ⚙️ Tools
- ROS2  
- MATLAB & Simulink  
- Python  

---

## 🎯 Key Insights
- Accurate timing is critical for trajectory reproduction  
- Feedforward control alone introduces tracking errors  
- Simulink provides better real-time performance than MATLAB and Python  
- LiDAR data can effectively reconstruct the environment even with partial coverage  

---

## 📊 Notes
The ROS2 bag file may not be included due to size constraints but can be provided upon request.

---

## ✅ Conclusion
The assignment demonstrates a complete pipeline from data analysis to control reconstruction and environment mapping.  
The results highlight the importance of timing accuracy in control execution and show how sensor data can be used to reconstruct both motion and environment in autonomous systems.
