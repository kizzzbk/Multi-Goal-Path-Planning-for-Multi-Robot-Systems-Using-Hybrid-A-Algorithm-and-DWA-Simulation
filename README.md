# 🧭 Multi-Goal Path Planning for Multi-Robot Systems in Dynamic Environments Using Hybrid A* and DWA

## 📖 Overview
This project presents a **hybrid path planning framework** for **multi-robot, multi-goal navigation** in **dynamic environments**.  
It integrates the **A*** algorithm for **global path planning** and the **Dynamic Window Approach (DWA)** for **local obstacle avoidance**.  
The system aims to ensure **collision-free**, **energy-efficient**, and **adaptive navigation** for multiple autonomous robots operating simultaneously.

---

## 🚀 Key Features
- 🔹 **Hybrid Planning:** Combines global (A*) and local (DWA) planners.
- 🔹 **Multi-Robot Coordination:** Assigns multiple goals among robots using an energy-aware greedy algorithm.
- 🔹 **Dynamic Obstacle Handling:** Avoids both static and moving obstacles in real time.
- 🔹 **Energy-Aware Allocation:** Each robot’s energy is considered in goal assignment.
- 🔹 **Simulation-Ready:** Designed for 2D simulated environments with static maps and dynamic agents.

---

## 🧩 System Architecture

### **1️⃣ Phase 1: Goal Assignment**
- Parses environment maps into grids.
- Assigns multiple goals to robots using an **energy-aware greedy algorithm**.
- Ensures balanced energy use and optimized path length.

### **2️⃣ Phase 2: Path Planning**
- Uses **A*** for global path generation (based on static obstacles).
- Applies **Dynamic Window Approach (DWA)** for local obstacle avoidance.
- Robots adaptively adjust velocity and heading based on predicted obstacle motion.

---

## ⚙️ Algorithms

### **A*** Global Planner
- Generates the optimal path from start to goal using grid-based search.
- Uses the **Chebyshev distance** heuristic for efficient exploration.

### **Dynamic Window Approach (DWA)**
- Samples feasible velocities within dynamic constraints.
- Evaluates candidates using an **objective function** that balances:
  - Goal progression (`ΔD`)
  - Obstacle cost (`Co`)
  - Weight factors (`wf`, `wo`)

### **Energy-Aware Goal Assignment (Algorithm 1)**
- Assigns goals based on minimum effective cost considering:
  - Path distance
  - Energy requirement
  - Charging needs

---

## 🧠 Experimental Setup
- Simulated in a **2D environment** (L × W map).
- Static obstacles represented as occupancy grids.
- Dynamic obstacles move with random velocities and bounce upon collisions.
- Robots are initialized with energy limits and pre-defined kinematic constraints.

### **Metrics Evaluated**
- Total path length  
- Average execution time  
- Collision rate  
- Navigation efficiency  

---

## 📊 Results
- **Hybrid A* + DWA** significantly outperforms standalone DWA:
  - Shorter and smoother trajectories.
  - Fewer collisions with static and dynamic obstacles.
  - Better goal tracking and inter-robot coordination.
- Robots successfully complete all assigned tasks and return to the charging station.

---

## 🧩 Future Work
- Integration with **real-world robot platforms** (e.g., TurtleBot, Pioneer).
- Extending to **3D environments** or **non-holonomic systems**.
- Applying **reinforcement learning** to optimize planner parameters.

---

## 👨‍💻 Authors
- **Nguyen Trung Thinh**, VinUniversity, Vietnam  
- **Phan Anh**, Thuyloi University, Vietnam  
- **Tran Thi Cam Giang**, Thuyloi University, Vietnam *(Corresponding author)*  
- **Nguyen Ngoc Doanh**, VinUniversity, Vietnam  

---

## 📚 Citation
If you use this work, please cite as:
```bibtex
@article{thinh2025multi,
  title={Multi-Goal Path Planning for Multi-Robot Systems in Dynamic Environments Using Hybrid A* Algorithm and Dynamic Window Approach},
  author={Nguyen Trung Thinh and Phan Anh and Tran Thi Cam Giang and Nguyen Ngoc Doanh},
  year={2025},
  institution={VinUniversity and Thuyloi University, Vietnam}
}
```

---

## 🧾 Keywords
`Robot Navigation`, `Path Planning`, `A*`, `DWA`, `Multi-Goal`, `Multi-Robot`, `Dynamic Environment`
