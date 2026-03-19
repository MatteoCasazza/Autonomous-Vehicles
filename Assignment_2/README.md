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
The project is organized into multiple MATLAB scripts and a Python ROS2 node, each serving a specific role in robot control and waypoint management.

### MATLAB Scripts

1. **`ass2_classical.m`**  
   - Implements a **classical proportional controller** for navigating through waypoints.  
   - Main functionalities:  
     - Subscribes to `/odom` for robot position and orientation  
     - Publishes velocity commands to `/cmd_vel`  
     - Computes position and heading errors to the current waypoint  
     - Proportional control for linear (`v`) and angular (`ω`) velocities  
     - Command saturation and stability logic (rotate first if heading error is too large)  
     - Automatic waypoint switching once position and orientation are within tolerance  

2. **`ass2_NodeWaypoints.m`**  
   - MATLAB controller that **receives waypoints from an external Python server**.  
   - Main functionalities:  
     - Publishes requests on `/wp_request`  
     - Receives the next waypoint on `/next_waypoint`  
     - Executes proportional control similar to `ass2_classical.m`  
     - Maintains a 10 Hz control loop  

3. **`ass2_orientation.m`**  
   - Variant of the classical controller with **orientation control applied throughout the entire path**.  
   - Uses combined angular errors `α` (heading to waypoint) and `β` (final orientation correction) for smoother and more precise trajectories.  

4. **`ass2_orientation_1M.m`**  
   - Orientation control is applied **only in the last meter before reaching the waypoint**.  
   - Results in straighter trajectories, shorter travel distance, and reduced mechanical stress on the robot.  

5. **`ass2_stanley.m`**  
   - Implements the **Stanley controller**, optimized for following segments between consecutive waypoints.  
   - Main functionalities:  
     - Computes lateral (cross-track) and heading errors relative to the line segment  
     - Calculates steering angle `δ` and angular velocity `ω = v_ref / L * tan(δ)`  
     - Modulates linear velocity based on alignment for stability  
     - High precision and fastest travel time, but more aggressive angular motion  

### Python Script

1. **`ass2_node_py.py`**  
   - ROS2 Python node serving as a **Waypoint Server**.  
   - Main functionalities:  
     - Subscribes to `/wp_request` for requests from MATLAB  
     - Publishes the requested waypoint on `/next_waypoint` as a `PoseStamped` message  
     - Converts yaw angles to quaternions for planar rotation  
     - Provides logging and index validation  
     - Keeps the node active using `rclpy.spin()`  

### System Overview
A conceptual flow of the system:
       MATLAB Controller
               |
       /wp_request (Int32)
               v
     Python Waypoint Server
               |
      /next_waypoint (PoseStamped)
               v
MATLAB Controller → /cmd_vel → TurtleBot3
               ^
               |
             /odom
               |
       MATLAB Controller

### Notes
- All MATLAB scripts run at a **10 Hz control loop** using `pause(max(0, dt - elapsed))`.  
- Velocity commands are always saturated to respect robot limits (`max_v`, `max_omega`).  
- Controllers prioritize either orientation or position depending on the variant (`ass2_orientation` vs `ass2_orientation_1M`).  
- External waypoint server allows flexible testing and decouples waypoint generation from control logic.

---

## 📊 Notes
The ROS2 bag file may not be included due to size constraints but can be provided upon request.
