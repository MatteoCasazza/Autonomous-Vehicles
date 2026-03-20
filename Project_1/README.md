# Project 1 – RRT-Based Path Planning

## 🎯 Problem Statement

The aim of this project is to implement a **Rapidly-exploring Random Trees (RRT) planner** on the maps provided in Assignment 4 (maps 4.1 – 4.5).  

RRT is a **sampling-based, single-query planning algorithm** that explores the environment to build a tree and detect a feasible trajectory to reach a desired end point, without optimizing it.  

In addition to the basic RRT, this project also implements:

- **RRT-Connect:** Uses two trees to connect start and goal efficiently.  
- **RRT\*:** Optimized version that considers the cost to reach each node.  

The results are compared with the **A\*** algorithm in terms of:

- Computation time  
- Traveled distance  
- Number of explored nodes  

The maps are of size **30 × 30**, imposing constraints on the algorithm due to resolution.

---

## ⚙️ Methodology

### Basic RRT Algorithm

1. **Tree Initialization:**  
   - Input image is processed to detect free (white) and obstacle (black) pixels.  
   - Tree `T` is initialized with the start point as its root.

2. **Sampling Loop:**  
   - Random configuration `x_rand` is sampled in the free space (occasionally set equal to the goal for faster convergence).  
   - Nearest node `x_near` in `T` is found.  
   - Compute direction vector and define `x_new = x_near + round(δq · [î, ĵ])`, with δq = √2.  
   - If `x_new` is free, add it to `T` with its parent index.

3. **Path Construction:**  
   - Starting from the goal, traverse parent nodes backwards until the start node is reached.  

**Iteration limit:** 5000; if exceeded, a feasible path is assumed not possible.

---

### RRT-Connect Algorithm

- Uses **two trees**, `T_a` rooted at start and `T_b` at goal.  
- Each iteration alternates roles between **exploration** and **connection**.  
- `T_a` expands randomly, `T_b` attempts to connect to `x_new`.  
- Path is constructed by combining nodes from both trees at the connection point.

---

### RRT\* Algorithm

- Considers **cost to reach nodes** for optimized trajectories.  
- A **cost matrix C** is initialized at the start.  
- When adding `x_new`, its parent is chosen as the node among neighbors minimizing `C(q′) = C(q) + C(q,q′)`.  
- Rewiring step: check if `x_new` improves cost for previously sampled neighbors.  
- After tree completion, matrices are sorted by increasing cost to ensure optimality.  
- Trajectory is then traced similarly to basic RRT.

---

## 📊 Results

The three algorithms were tested on all maps. Due to stochastic sampling, multiple runs give different trajectories.

### 6.3.1 General Observations

- **Basic RRT:**  
  - Longest and most irregular paths  
  - Fast, simple exploration, stochastic nature introduces variability  

- **RRT-Connect:**  
  - Efficient in connecting start and goal  
  - Shorter paths, fewer explored nodes, generally faster computation  

- **RRT\*:**  
  - Optimized path lengths  
  - Higher computational cost  
  - Converges to near-optimal solution with more iterations  

---

### Sample Map Results

| Map | Algorithm | Time [s] | Distance [m] | Explored Nodes |
|-----|-----------|-----------|---------------|----------------|
| Test Map 1 | Basic RRT | 0.1710 | 44.870 | 135 |
| Test Map 1 | RRT-Connect | 0.0884 | 36.870 | 31 |
| Test Map 1 | RRT* | 0.2462 | 37.698 | 351 |
| First Map | Basic RRT | 0.1370 | 52.799 | 194 |
| First Map | RRT-Connect | 0.0768 | 51.971 | 106 |
| First Map | RRT* | 0.1909 | 49.042 | 158 |
| Second Map | Basic RRT | 0.1156 | 45.113 | 183 |
| Second Map | RRT-Connect | 0.0526 | 48.770 | 58 |
| Second Map | RRT* | 0.1301 | 42.527 | 167 |

> **Note:** Tables for Third and Fourth maps follow similar trends:  
> - RRT-Connect gives shorter paths for simple start-goal connections  
> - RRT* reduces path length but increases computational cost  
> - Basic RRT paths are variable and jagged due to stochastic sampling

---

### Map-Specific Observations

- **First Map:**  
  - Random sampling sometimes leaves zones unexplored, particularly around obstacles.  
  - RRT-Connect avoids these zones with straight connection lines.  
  - RRT* optimality depends on the number of sampled nodes.  

- **Second Map:**  
  - Basic RRT produces different paths in repeated runs.  
  - RRT-Connect efficiently overcomes star-shaped obstacles.  
  - RRT* gives shortest paths but explores more nodes.

- **Third Map:**  
  - Horizontal obstacles require many sampled points.  
  - RRT-Connect trajectories are longest due to repeated obstacle hits.  
  - RRT* achieves shortest paths at higher computational cost.

- **Fourth Map:**  
  - Optimal paths often require high iteration counts in RRT*.  
  - RRT-connect gives two distinct trajectories with straight segments.  

---

### Algorithm Comparison

| Algorithm | Pros | Cons |
|-----------|------|------|
| Basic RRT | Simple, fast, flexible | Long, jagged paths, stochastic variability |
| RRT-Connect | Fast, fewer explored nodes | Paths may be long if obstacles encountered |
| RRT* | Optimized trajectories | High computational cost, requires many iterations |

**Insights:**

- Stochastic nature affects results significantly.  
- Grid-based methods guarantee optimality but require prior knowledge of the full map.  
- Sample-based RRT methods are more flexible in unknown or complex environments.

---

## ✅ Conclusions

- **Feasible trajectories** were successfully computed for all maps using RRT, RRT-Connect, and RRT*.  
- **Efficiency depends** on map geometry and stochastic exploration.  
- **Basic RRT** is fast but produces irregular paths.  
- **RRT-Connect** efficiently connects start and goal with fewer explored nodes.  
- **RRT\*** provides trajectories close to optimal at a higher computational cost.  
- **Choice of planner** depends on problem requirements:  
  - Rapid feasible solution → Basic RRT / RRT-Connect  
  - Optimal path → RRT* with higher computation  
- Sample-based planners excel in **complex geometries**, but grid-based planners remain better when **optimality and map knowledge** are priorities.

---

## 🧩 Code Structure

The project is organized into multiple MATLAB scripts and functions, each implementing a different variant of the Rapidly-exploring Random Tree (RRT) algorithm and its improvements.

### 📁 Main Scripts

- **`main_RRT.m`**  
  Implements the **basic RRT algorithm**.  
  A single tree is grown starting from the initial position using random sampling with goal biasing. The algorithm expands the tree by selecting the nearest node and generating a new node in its direction. Once the goal is reached, the path is reconstructed via backtracking.

- **`main_RRT_conn.m`**  
  Implements the **RRT-Connect algorithm**.  
  Two trees are grown simultaneously: one from the start and one from the goal. The trees alternate between expansion and connection attempts, significantly improving convergence speed compared to the basic RRT.

- **`main_RRT_star.m`**  
  Implements the **RRT\*** algorithm.  
  In addition to tree expansion, this version introduces cost optimization:
  - Selection of the best parent node based on minimum cost
  - Local rewiring to improve path optimality
  - Optional global rewiring for further optimization  
  This results in asymptotically optimal paths.

---

### ⚙️ Supporting Functions

- **`parent_function.m`**  
  Selects the **best parent node** for a newly generated node by minimizing the total path cost within a local neighborhood.

- **`Rewire.m`**  
  Performs the **rewiring step** in RRT\*.  
  It checks whether nearby nodes can reduce their cost by changing parent to the newly added node, improving overall path optimality.

---

### 🔄 Core Workflow

All main scripts follow the same high-level pipeline:

1. Load and preprocess the map  
2. Define start and goal positions  
3. Initialize tree structure  
4. Iteratively expand the tree(s) using random sampling  
5. Check for collision and map boundaries  
6. (Optional) Optimize the tree (RRT\*)  
7. Detect goal reach condition  
8. Reconstruct the path via backtracking  
9. Compute performance metrics (time, distance, explored nodes)  


