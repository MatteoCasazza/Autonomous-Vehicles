# Assignment 2 – Feedback Control and Waypoints Navigation

## 📌 Overview

This assignment focuses on implementing feedback controllers for the TurtleBot3 robot in a simulated environment (TurtleBot3 Empty World). The objective is to navigate the robot through a sequence of predefined waypoints while respecting physical constraints on velocity and orientation. Multiple control strategies are developed and compared for efficiency, smoothness, and precision.

---

## 🎯 Problem Statement

The robot must reach a series of waypoints defined by coordinates `(x, y)` and orientation `θ`:

| N  | x [m] | y [m] | θ [rad] |
|----|-------|-------|---------|
| 0  | 0     | 0     | 0       |
| 1  | 5     | 0     | π/2     |
| 2  | 5     | 5     | 5π/4    |
| 3  | -5    | -5    | π/2     |
| 4  | -5    | 5     | 0       |
| 5  | 0     | 0     | 0       |
| 6  | 3     | 3     | 3π/4    |
| 7  | -3    | 0     | 3π/2    |
| 8  | 0     | -3    | π/4     |
| 9  | 3     | 0     | π/2     |
| 10 | 0     | 0     | 3π/2    |

Constraints of the robot:

- Maximum linear speed: |v| ≤ 0.2 m/s  
- Maximum angular speed: |ω| ≤ 0.4 rad/s  
- Position tolerance: 0.05 m  
- Orientation tolerance: 0.05 rad  

Waypoints can be selected either from a predefined array in the source code or through an external ROS2 publisher node.

---

## ⚙️ Methodology

The workflow consists of the following main steps:

1. **ROS2 Node Initialization**  
   - MATLAB node subscribes to `/odom` for real-time position and orientation.  
   - Publisher sends `/cmd_vel` commands (linear and angular velocities) at 10 Hz.

2. **Proportional Controller**  
   - Longitudinal control: velocity `v = kv * distance_error`  
   - Angular control: `ω = kω * heading_error`  
   - Gains: `kv = 0.6 s⁻¹`, `kω = 1.4 s⁻¹`  
   - Tolerances: `tolxy = 0.02 m`, `tolθ = 0.01 rad`  
   - Linear velocity is set to zero if angular error exceeds π/2.  
   - Robot rotates to fix orientation once waypoint is reached.

3. **Waypoint Orientation Management**  
   - Controller aligns robot with final waypoint orientation using combined angular error terms:  
     - `α` = heading error to goal  
     - `β` = difference between goal orientation and heading to goal  
   - Angular velocity: `ω = kω·α + kβ·β`  
   - Gains: `kv = 0.3 s⁻¹`, `kω = 0.8 s⁻¹`, `kβ = -0.15 s⁻¹`  
   - Orientation management can be applied either along the full path or only in the last meter.

4. **Stanley Controller**  
   - Minimizes cross-track (`ect`) and heading (`eh`) errors along line segments between waypoints.  
   - Steering angle: `δ = eh + arctan(ke * ect / vref)`  
   - Angular velocity: `ω = vref / L * tan(δ)`  
   - Linear velocity is modulated based on steering angle.  
   - Gains: `ke = 0.8 s⁻¹`, `vref = 0.2 m/s`, `L = 0.14 m` (robot length).  

5. **External Waypoints Source**  
   - Python node publishes waypoints to MATLAB subscriber.  
   - Feedback signal ensures next waypoint is sent only after previous one is reached.

---

## 📊 Results

| Algorithm                                    | Time [s] | Distance [m] |
|---------------------------------------------|----------|--------------|
| Proportional Controller                      | 424.60   | 63.93        |
| Waypoint Orientation Manager (full path)    | 467.44   | 66.86        |
| Waypoint Orientation Manager (last meter)   | 454.74   | 64.65        |
| Stanley Controller                           | 409.42   | 63.82        |
| External Waypoints (Proportional Controller)| 425.34   | 63.93        |

Key observations:

- **Proportional Controller:** stable, low lateral acceleration, smooth motion, total distance 63.93 m.  
- **Orientation Management:** smoother curvilinear paths, slightly longer distance and duration, higher angular precision.  
- **Stanley Controller:** shortest distance (63.82 m) and time (409.42 s), but more aggressive angular dynamics.  
- **External Waypoints:** minor initial communication delay (~0.74 s), no impact on tracking accuracy.

---

## 🧩 Code Structure

---

## 📊 Notes
The ROS2 bag file may not be included due to size constraints but can be provided upon request.
