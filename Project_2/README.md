# Project 2 – Autonomous Navigation of TurtleBot3 Burger

## 🎯 Problem Statement

The aim of this project is to enable **autonomous navigation** of the TurtleBot3 Burger within its **house environment**. The key goals are:

1. Map the environment using **SLAM**.  
2. Plan **optimal trajectories** from known start to goal positions using a modified **A\*** algorithm.  
3. Control the robot along the trajectories while respecting **motion constraints**:  
   - Linear velocity |v| ≤ 0.22 m/s  
   - Angular velocity |ω| ≤ 2.84 rad/s  

The project considers four different start-end pairs, designing paths that are both feasible and cost-efficient.

---

## ⚙️ Methodology

### Mapping the Environment

- **SLAM Toolbox** is used to explore and map the TurtleBot3 House.  
- The robot is controlled via **Teleop Keyboard** while simultaneously collecting data from **2D LiDAR** and **odometry sensors**.  
- The resulting map is a **212 × 297 grid**:  
  - White = free space  
  - Black = obstacles  
  - Gray = unexplored zones  

- Robot size is accounted for by **inflating obstacles** by 150 mm (3 pixels), ensuring safe navigation.

**Visualization:** The mapping process is monitored live using **Rviz2**.

---

### A* Algorithm for Path Planning

- A modified **A\*** algorithm is applied to handle large maps.  
- The adjacency matrix is replaced by a **neighbor list** of free pixels.  
- Cost calculation:  
 - **Step cost:**
  $$
  C(q, q') = \sqrt{(q_x - q'_x)^2 + (q_y - q'_y)^2}
  $$

- **Total cost to reach a node:**
  $$
  C(q') = C(q) + C(q, q')
  $$

- **Heuristic (Euclidean distance to goal):**
  $$
  h(q') = \sqrt{(x_p - x_g)^2 + (y_p - y_g)^2}
  $$

- Trajectories are constructed by backtracking from the goal to the start.  
- Start coordinates are mapped to the environment origin, and each step is scaled according to map resolution.

---

### Feedback Controller

- A **ROS 2 node** controls the robot along the planned paths.  
- A **subscriber** receives odometry data, while a **publisher** sends velocity commands in real-time.  
- A **look-ahead logic** is implemented:  
  - The robot targets a point on a **circumference of radius r = 0.05L**, instead of the closest point, to avoid stopping at each small step.  
  - Final points are considered reached when within **0.008 m** tolerance.  

- Two strategies tested:  
  1. **Constant look-ahead distance**  
  2. **Varying look-ahead distance** for smoother and faster navigation  

- This approach balances **high speed on straights** and **precision during cornering**.

---

## 📊 Results

### Mapping of the Environment

- Final map is modified to reflect **all free areas** considering robot size.  
- Blind spots, e.g., table legs or shelves, are manually adjusted.  
- Exterior areas, although partially unexplored, are set as free space.

---

### Trajectory Planning via A*

| N | Start | End | Steps | Distance [m] | Nodes Explored | Execution Time [s] |
|---|-------|-----|-------|---------------|----------------|------------------|
| 1 | (119,110) | (163,280) | 225 | 11.6 | 16740 / 51304 | 0.2860 |
| 2 | (163,281) | (82,184) | 215 | 13.0 | 14714 / 51304 | 0.2040 |
| 3 | (82,183) | (57,97) | 248 | 13.6 | 6904 / 51304 | 0.1143 |
| 4 | (57,96) | (164,38) | 158 | 8.4 | 6276 / 51304 | 0.1069 |

- **Observations:**  
  - Paths optimize distance while avoiding obstacles.  
  - Explored nodes are minimized due to selective neighbor expansion.  
  - Trajectories leverage geometrical constraints of the house, following walls closely and using diagonal movements where possible.

---

### Feedback Controller Results

- **Constant look-ahead:**  
  - Completion time: 409.25 s  
  - RMS tracking error: 0.0171 m  
  - Robot cuts corners slightly, causing minor deviations  

- **Varying look-ahead:**  
  - Completion time: 334.90 s  
  - RMS tracking error: 0.0176 m  
  - Smoother X-Y motion profiles and reduced chattering  
  - Linear velocity maintained near max on straights, reduced during turns for stability  

- **Key insight:** Adjusting look-ahead dynamically improves both speed and precision without sacrificing accuracy.

---

## ✅ Conclusions

- The TurtleBot3 successfully navigates its house environment **autonomously**.  
- **SLAM mapping**, combined with **inflated obstacles**, ensures safe trajectory planning.  
- The **modified A\*** algorithm efficiently generates optimal paths while minimizing explored nodes and computational time (<0.3 s).  
- **Look-ahead feedback control** enables smooth and accurate following of trajectories:  
  - Dynamic adjustment improves performance on corners  
  - Balances speed, stability, and safety  

- Overall, the project demonstrates that **autonomous mobile navigation** is feasible by integrating **mapping, path planning, and adaptive control**, with careful tuning to robot dynamics and environmental constraints.

---

## 🧩 Code Structure

The project is divided into two main components:

1. **Global Path Planning (A\*)**
2. **Trajectory Tracking (Feedback Control with ROS2)**

---

### 📁 Path Planning Module

- **`main_proj2_maps.m`**  
  Main script for **global path planning** using the A\* algorithm.

  It performs the following steps:
  - Loads the environment map and converts it into a binary occupancy grid
  - Builds a **graph representation** of the map using an adjacency list (8-connectivity)
  - Defines multiple start-goal pairs to generate a complete trajectory
  - Runs A\* for each pair to compute optimal paths
  - Concatenates all paths into a global trajectory
  - Saves the result in `trajectory_data.mat` for later use in control

  The script also generates:
  - Explored nodes visualization
  - Optimal trajectories
  - Performance metrics (distance, explored nodes, execution time)

---

- **`aStar_optimized.m`**  
  Implementation of the **A\* path planning algorithm**.

  Key features:
  - Uses a **priority queue** based on cost function \( f = g + h \)
  - Euclidean distance heuristic
  - Efficient node exploration with a visited set
  - Reconstruction of the optimal path using a predecessor vector

  Outputs:
  - Optimal path
  - Total distance to goal
  - Number of explored nodes

---

### 🤖 Trajectory Tracking Module (ROS2)

- **`main_proj2_2LA.m`**  
  Implements waypoint tracking using a **fixed look-ahead controller**.

  Features:
  - Loads precomputed trajectory from `trajectory_data.mat`
  - Connects to ROS2 (`/odom`, `/cmd_vel`)
  - Tracks waypoints using a **pure pursuit-like control law**
  - Uses a constant look-ahead distance
  - Splits the trajectory into multiple segments

---

- **`main_proj2_10LA.m`**  
  Improved version with **adaptive look-ahead strategy**.

  Enhancements:
  - Dynamically adjusts look-ahead distance based on path geometry
  - Detects **straight segments vs curves** using angular variation
  - Uses:
    - Larger look-ahead for straight paths → smoother and faster motion  
    - Smaller look-ahead for curves → better accuracy
  - Adapts controller gains accordingly

  This results in:
  - Improved tracking performance
  - Smoother velocity profiles
  - Better stability in sharp turns

---

## 🧪 Notes 
The ROS2 bag files are not included due to size constraints but can be provided upon request. 
