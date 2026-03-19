# Assignment 3 – Feedback Control and Target Selection

## 📌 Overview
This assignment focuses on designing and implementing a feedback controller for the **TurtleBot3 Waffle Pi** robot in a simulated environment (*TurtleBot3 Empty World*). The goal is for the robot to autonomously detect and discriminate between multiple objects using onboard sensing and navigate to the correct target. 

The system integrates:
* **Camera-based color detection** (HSV)
* **LiDAR-based size estimation** (Least-squares fitting)
* **Proportional feedback control** for reliable navigation.

---

## 🎯 Problem Statement
The robot must identify and reach the correct target among multiple spherical objects:

| Object | Color | Radius [m] | Relative Position | Target? |
| :--- | :--- | :--- | :--- | :--- |
| 1 | Red | 0.1 | 1 m left | ❌ |
| 2 | Purple | 0.3 | 3 m front | ❌ |
| 3 | **Red** | **0.3** | **3 m behind** | **✅** |

**Constraints and Requirements:**
* Robot relies solely on onboard sensing (**Camera** and **LiDAR**).
* The correct target is the **large red sphere** located behind the robot's initial heading.
* The robot must ignore smaller or visually similar objects (disturbances).
* Feedback control must produce linear and angular velocity commands in real time.

---

## ⚙️ Methodology

### 1. Simulation Environment and System Setup
* **Platform:** TurtleBot3 Waffle Pi deployed via ROS 2.
* **Software:** Sensor data is processed in **MATLAB/Simulink** to generate real-time velocity commands.
* **Topics:**
  * `/odom`: Robot position and orientation.
  * `/scan`: LiDAR for distance and size estimation.
  * `/camera/image_raw`: RGB camera for color detection.
  * `/cmd_vel`: Output for movement commands.

### 2. Camera-Based Target Detection
RGB images are converted to **HSV color space** for robust detection:
* **Red Thresholds:** Hue ($0 \le H \le 0.05$), Saturation ($0.95 \le S \le 1$).
* **Processing:** Blob analysis is used to identify centroids and areas of red regions.
* **Scanning:** The robot rotates at $\omega = 0.2$ rad/s to scan the environment.

### 3. LiDAR-Based Size Estimation
Since camera perspective cannot reliably distinguish size, LiDAR data is converted from polar to Cartesian coordinates using odometry:
$$x_s = R \cdot \cos(\theta + \theta_{odom}) + x_{odom}$$
$$y_s = R \cdot \sin(\theta + \theta_{odom}) + y_{odom}$$

* **Filtering:** Only points within a 45° window are considered.
* **Fitting:** Least-squares fitting estimates the sphere's radius ($r$).
* **Fusion:** Camera detection + LiDAR radius estimation uniquely identify the 0.3m red sphere.

### 4. Feedback Control Strategy
The robot remains in rotation until the target criteria are met: $0.75 \cdot r_{max} \le r \le 1.25 \cdot r_{max}$.
Once locked, a **Proportional (P) Controller** computes:
* **Linear Velocity (v):** `v = k_rho * distance_error`
* **Angular Velocity (ω):** `ω = k_alpha * angle_error`

**Gains:** `k_rho = 0.5 s^-1`, `k_alpha = 0.3 s^-1`

---

## 📊 Results

* **Target Detection:** HSV thresholds successfully filtered red spheres from the purple distractor. Centroids were accurately tracked in Simulink.
* **Trajectory:** The robot follows a smooth, curved path. Odometry confirms stable convergence toward the target.
* **Control Performance:** * **$v$:** Reaches $0.4$ m/s post-detection with gradual deceleration upon arrival.
  * **$\omega$:** Dynamic alignment ensures the robot maintains heading toward the sphere's center.

---

## 🧩 Code Structure

[Camera Images]                 [LiDAR Scan] [Odometry]
       ↓                                    ↓
   HSV Filter → Centroid Extraction → Target Selection
                               ↓
                     Position & Size Estimation
                               ↓
                       Feedback Controller
                               ↓
                          /cmd_vel → TurtleBot3

---

## 📊 Notes
* The environment contains distractor objects, highlighting the necessity of sensor fusion.
* Simulation ensures accurate timing of sensor acquisition and control command publication.
* The system successfully demonstrates autonomous target selection and navigation using integrated perception and control.

