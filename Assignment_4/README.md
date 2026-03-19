# Assignment 4 – Grid-Based Path Planning

## 📌 Overview

This assignment focuses on implementing and analyzing **Grid-Based Algorithms** for path planning. The work starts with a basic **Dijkstra algorithm**, progressively adding diagonal movements and finally transforming it into an **A\*** algorithm with various heuristics. Performance is tested on multiple 30×30 maps to evaluate efficiency, path length, and computational cost.

---

## 🎯 Problem Statement

The objective is to control a point or robot to move from a **start** to a **goal** position on a grid map while avoiding obstacles. The assignment includes:

- Implementing **Dijkstra algorithm** (basic 4-way connectivity).  
- Adding **diagonal movements** to improve path efficiency.  
- Transforming the algorithm into **A\*** with different heuristics (Euclidean, Manhattan, Chebyshev, Diagonal).  
- Testing all algorithms on **5 reduced 30×30 maps** with predefined start and goal positions:

| Map | Start | Goal |
|-----|-------|------|
| 0   | 1,1   | 20,30|
| 1   | 1,12  | 28,12|
| 2   | 1,8   | 27,28|
| 3   | 1,1   | 1,30 |
| 4   | 1,8   | 20,18|

The algorithms are compared in terms of **traveled distance, computation time, and number of explored nodes**.

---

## ⚙️ Methodology

### 1. Environment Setup

- Maps are initially **300×300 pixels**; resized to **30×30** for computational feasibility.  
- Binary thresholding classifies **free cells** vs **obstacles** (intensity > 0.95 → free, else obstacle).  
- **Edge matrix `G`** defines connectivity:
  - `0`: node free and self-connected  
  - `1`: horizontal/vertical connection  
  - `-1`: obstacle or no connection  
- **Cost vector**: initialized to infinity.  
- **Parenting vector**: tracks optimal parent node.  
- **Queue vector**: stores nodes to explore.  

**Algorithm loop (Dijkstra V1):**

1. Start node `q` removed from queue.  
2. Connected nodes `q'` added to queue, cost evaluated:  
   \[
   C(q') = C(q) + C(q, q')
   \]  
3. Node `q'` updated if new cost < current best.  
4. If `q' = qG` (goal), reconstruct path using parents.

---

### 2. Diagonal Movements

- Diagonal connectivity added to edge matrix:
  - `2`: diagonal connection  
- Step cost for diagonal: \(C(q,q') = \sqrt{2}\)  
- Horizontal/vertical cost remains 1.  

This improves path efficiency and allows **8-way movement**.

---

### 3. A* Algorithm

- Adds **heuristic cost-to-go** \(h(q')\) to prioritize nodes closer to the goal:
\[
C(q') = C(q) + C(q, q') + h(q')
\]
- Heuristic types:
  - **Euclidean**: \(\sqrt{(x_{\text{goal}}-x)^2 + (y_{\text{goal}}-y)^2}\)  
  - **Manhattan**: \(|x_{\text{goal}}-x| + |y_{\text{goal}}-y|\)  
  - **Chebyshev**: \(\max(|x_{\text{goal}}-x|, |y_{\text{goal}}-y|)\)  
  - **Diagonal**: combination for 8-way grids:  
\[
h(q') = |x_{\text{goal}}-x| + |y_{\text{goal}}-y| + (\sqrt{2}-2) \cdot \min(|x_{\text{goal}}-x|, |y_{\text{goal}}-y|)
\]

- Only **queue ordering** uses heuristic; node evaluation still uses cost of arrival.

---

## 📊 Results

| Algorithm | Time [s] | Distance [m] | Explored Nodes |
|-----------|----------|--------------|----------------|
| Dijkstra V1       | 0.1493 | 48.000  | 704 |
| Dijkstra V2       | 0.1532 | 36.870  | 693 |
| A* Euclidean      | 0.1450 | 36.870  | 217 |
| A* Manhattan      | 0.1110 | 36.870  | 30  |
| A* Chebyshev      | 0.1548 | 36.870  | 313 |
| A* Diagonal       | 0.1407 | 36.870  | 183 |

**Observations:**

- **Dijkstra V1**: restricted to 4-way moves, long path, many nodes explored.  
- **Dijkstra V2**: allows diagonal moves → optimal path, still evaluates many nodes.  
- **A\***: reduces explored nodes; performance depends on heuristic.  
  - **Euclidean**: reliable, finds optimal path.  
  - **Manhattan**: fastest computation, may be inadmissible for diagonal paths.  
  - **Chebyshev**: admissible, slightly slower than Euclidean.  
  - **Diagonal**: best overall, efficiently approximates cost-to-go for 8-way grids.

---

## 🧩 Code Structure

- **Map Processing**: load image, resize to 30×30, apply binary threshold.  
- **Edge Matrix G**: defines connectivity and costs.  
- **Dijkstra V1**: horizontal/vertical moves, unit step cost.  
- **Dijkstra V2**: adds diagonal moves, step cost √2.  
- **A\***: adds heuristic cost-to-go, evaluates nodes using chosen heuristic.  
- **Path Reconstruction**: uses parent vector to backtrack optimal path.  
- **Visualization**: MATLAB plots maps, explored nodes, and final trajectory.  

---

## 🧪 Notes

- Performance metrics include **computation time, path length, and explored nodes**.  
- Diagonal movement significantly improves path efficiency.  
- Heuristic choice in A\* affects speed and optimality.  
- Maps with complex obstacles highlight differences between Dijkstra and A\* approaches.  

---

## ✅ Conclusions

- **Dijkstra V1**: simple but inefficient, only 4-way moves.  
- **Dijkstra V2**: optimal 8-way paths, higher computational cost.  
- **A\***: reduces nodes explored, faster convergence.  
- **Heuristics**:  
  - Diagonal → best overall.  
  - Euclidean → reliable, fewer nodes explored.  
  - Manhattan → fast but can be suboptimal.  
  - Chebyshev → admissible, slightly slower.  

The assignment demonstrates the advantages of **heuristic-based search** in grid-based motion planning and shows how algorithmic improvements (diagonal moves, cost-to-go) reduce computation and optimize trajectories.

