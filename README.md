# SmartSweep AI – Intelligent Robot Vacuum Cleaner Path Planner

> **Tagline:** Smarter Navigation. Efficient Cleaning. Powered by AI.  
> **Course / Category:** College Project-Based Learning (PBL) in Artificial Intelligence & Robotics  
> **Core Algorithm:** Pure A* (A-Star) Heuristic Search from scratch with `heapq`  
> **GUI Framework:** CustomTkinter + High-Performance Tkinter Canvas  

---

## 1. Abstract

Autonomous cleaning robots have become indispensable components of modern smart-home automation. However, conventional commercial entry-level cleaners rely heavily on semi-random bounce patterns or rigid boustrophedon (lawnmower) sweeps that consume excessive battery energy, get stuck in complex obstacles, and clean redundant zones while missing concentrated dirt. 

**SmartSweep AI** is a complete, intelligent desktop simulation system engineered for college Project-Based Learning (PBL). It models an autonomous vacuum cleaner capable of finding mathematically optimal collision-free paths across configurable 2D room layouts using the **A\* search algorithm**. The system implements:
1. Pure **A\* algorithm** from scratch with admissible heuristics (Manhattan for 4-directional moves and Octile for 8-directional diagonal moves).
2. Strict **diagonal corner-cutting prevention**, guaranteeing physical feasibility for real-world robotic chassis.
3. A **Multi-Target Smart Cleaning** strategy that sequences uncleaned dirt spots via greedy nearest-target A\* routing before docking at the designated base station.
4. A high-framerate **CustomTkinter** desktop GUI with step-by-step exploration visualization, customizable room layouts, JSON persistence, live telemetry cards, and smooth vacuum animation.

---

## 2. Problem Statement

Conventional robotic vacuum cleaners face several navigation challenges:
- **Energy and Time Inefficiency:** Blind movement models or unguided searches (such as Breadth-First Search or uninformed random walks) explore unnecessary cells and waste battery cycles.
- **Obstacle Avoidance and Corner Entrapment:** Squeezing through diagonal gaps between two obstacles often leads to physical entrapment or collisions.
- **Multi-Target Cleaning Sequencing:** In real households, debris is concentrated at irregular spatial coordinates. Navigating between scattered dirt spots while avoiding dynamic furniture obstacles requires robust multi-point path planning.
- **Lack of Educational Transparency:** Standard robotics platforms operate as closed "black boxes", preventing students and evaluators from observing the frontier, closed sets, heuristic evaluations ($f, g, h$), and performance metrics in real time.

---

## 3. Objectives

- **Develop a Scratch A\* Implementation:** Implement A\* search using Python's `heapq` priority queue without third-party pathfinding libraries.
- **Implement Admissible Heuristics:** Formulate and validate Manhattan distance for 4-connected grid topologies and Octile distance for 8-connected grid topologies.
- **Prevent Corner-Cutting:** Enforce geometric constraints such that diagonal transitions cannot cut across blocked wall corners.
- **Engineer Multi-Dirt Routing:** Build a multi-target cleaning planner that schedules visits to all reachable dirt locations and docks at the final destination.
- **Create a Visually Polished Desktop GUI:** Construct a robotics-themed dashboard with CustomTkinter and Tkinter Canvas featuring real-time animation, step-by-step search reveals, and live metrics.
- **Ensure Scientific Verification:** Implement automated test suites comparing A\* against uniform-cost Dijkstra references to verify 100% path optimality.

---

## 4. Existing System vs. Proposed System

| Feature | Existing Conventional Systems | SmartSweep AI (Proposed System) |
|---|---|---|
| **Navigation Model** | Random bounce / wall-following | Heuristic-guided A\* Search ($f = g + h$) |
| **Search Optimality** | Suboptimal; redundant pathing | Guaranteed optimal path under admissible heuristics |
| **Diagonal Movement** | Often disallowed or ignores obstacle corners | 8-Directional with strict corner-cutting prevention |
| **Multi-Target Handling** | Blind scanning of entire room | Smart greedy nearest-dirt A\* scheduling |
| **Visualization** | None / proprietary mobile apps | Interactive step-by-step frontier and closed-set visualizer |
| **Room Configuration** | Pre-mapped by hardware or fixed | Custom grid sizes, interactive cell paint, JSON save/load |
| **Telemetry & Metrics** | Basic battery % and runtime | Live path cost, distance traveled, nodes explored, time in ms, progress % |

---

## 5. Introduction to AI Pathfinding

Pathfinding in robotics is the algorithmic problem of finding a valid, collision-free trajectory from an initial coordinate $\mathbf{s}_{\text{start}}$ to a target coordinate $\mathbf{s}_{\text{goal}}$ across an environment containing spatial constraints (obstacles).

- **Uninformed Search (BFS, DFS, Dijkstra):** Explores equally in all directions without knowledge of where the goal lies. Dijkstra's algorithm guarantees the shortest path but expands a massive number of unnecessary nodes.
- **Informed (Heuristic) Search (A\*):** Employs problem-specific domain knowledge through a heuristic estimate $h(n)$ to direct the search wave towards the goal, drastically reducing the search space and execution time.

---

## 6. Mathematical Explanation of the A* Algorithm

A\* evaluates candidate nodes in the priority queue using the evaluation function:

$$f(n) = g(n) + h(n)$$

Where:
- **$g(n)$:** The **actual, exact cost** incurred to travel from the starting cell to the current node $n$.
  - Orthogonal move (Up, Down, Left, Right): $\text{cost} = 1.0$
  - Diagonal move (Up-Left, Up-Right, Down-Left, Down-Right): $\text{cost} = \sqrt{2} \approx 1.4142$
- **$h(n)$:** The **heuristic estimate** of the cheapest cost from node $n$ to the goal.
  - To guarantee that A\* is **optimal and complete**, $h(n)$ must be **admissible** (it must never overestimate the true cost to reach the goal: $h(n) \le h^*(n)$) and **consistent / monotonic** ($h(n) \le c(n, p) + h(p)$).
- **$f(n)$:** The **estimated total cost** of the cheapest solution path passing through node $n$.

### Heuristics Implemented

1. **Manhattan Distance (4-Directional Movement):**
   $$h(n) = |r_n - r_{\text{goal}}| + |c_n - c_{\text{goal}}|$$
   Strictly admissible and consistent for grids restricted to horizontal and vertical steps.

2. **Octile Distance (8-Directional Movement):**
   For grids allowing diagonal transitions of cost $\sqrt{2}$:
   $$\Delta r = |r_n - r_{\text{goal}}|, \quad \Delta c = |c_n - c_{\text{goal}}|$$
   $$h(n) = \max(\Delta r, \Delta c) + (\sqrt{2} - 1) \cdot \min(\Delta r, \Delta c)$$
   Admissible and consistent, preventing heuristic overestimation while tightly bounding diagonal search expansion.

### Diagonal Corner-Cutting Prevention

When moving diagonally from $(r, c)$ to $(r + dr, c + dc)$ with $|dr|=1$ and $|dc|=1$, a real vacuum cannot physically slip between corners if adjacent orthogonal cells are blocked. SmartSweep AI enforces:
$$\text{Move is VALID} \iff (r+dr, c) \notin \text{Obstacles} \quad \text{AND} \quad (r, c+dc) \notin \text{Obstacles}$$

---

## 7. Multi-Dirt Target-Selection Strategy

In **Smart Cleaning Mode**, the vacuum cleaner must clean all dirt locations before navigating to the final docking goal:

1. **Target Selection:** From the robot's current position, calculate the A\* path cost to all remaining reachable dirt locations.
2. **Greedy Nearest Target:** Select the uncleaned dirt spot that has the **lowest A\* path cost**.
3. **Segment Routing:** Append the A\* trajectory to that dirt spot to the master path plan.
4. **Iterative Cleaning:** Update current robot position to the selected dirt spot, mark it as visited, and repeat until all reachable dirt locations are consumed.
5. **Final Docking:** Plan the final optimal A\* route from the last dirt spot to the designated Goal (charging dock).
6. **Unreachable Detection:** Trapped dirt locations (e.g., enclosed by walls) are automatically detected and flagged to the user without halting the simulation.

> **Academic Note on Optimality:**  
> While A\* provides the mathematically **optimal route for each individual segment**, sequencing multiple locations is an instance of the NP-hard **Traveling Salesperson Problem (TSP)**. SmartSweep AI uses a greedy nearest-neighbor heuristic which provides an efficient polynomial-time solution ($O(K \cdot N \log N)$ where $K$ is dirt count), though it is not guaranteed to be globally TSP-optimal.

---

## 8. System Architecture

```
SmartSweepAI/
│
├── main.py                  # Application entry point; initializes CustomTkinter loop
├── gui.py                   # Presentation layer: Tkinter Canvas, sidebar panels, telemetry
├── astar.py                 # Core AI algorithm: pure A* search, priority queue, heuristics
├── room.py                  # Model layer: 2D grid representation, presets, JSON I/O
├── robot_simulation.py      # Controller layer: multi-target planning, state machine, metrics
├── requirements.txt         # External dependencies
├── README.md                # Documentation and Viva reference guide
├── examples/                # Pre-configured room layouts
│   ├── apartment_suite.json
│   └── complex_maze.json
└── tests/                   # Automated unit & integration test suite (30 tests)
    ├── test_astar.py        # A* algorithm unit tests + Dijkstra reference comparison
    ├── test_room.py         # Room boundaries, obstacles, dirt, JSON serialization tests
    ├── test_simulation.py   # State machine, multi-dirt planning, metrics tests
    └── test_gui_integration.py # GUI integration tests
```

---

## 9. Flowchart of A* Algorithm Execution

```mermaid
flowchart TD
    Start([Start Search]) --> Init[Initialize Open Priority Queue with Start Node<br>g(start) = 0, f(start) = h(start)]
    Init --> CheckPQ{Is Open Queue Empty?}
    CheckPQ -- Yes --> Fail([Return Failure: No Path Exists])
    CheckPQ -- No --> Pop[Pop node n with lowest f score from Priority Queue]
    Pop --> InClosed{Is n in Closed Set?}
    InClosed -- Yes --> CheckPQ
    InClosed -- No --> AddClosed[Add n to Closed Set]
    AddClosed --> IsGoal{Is n == Goal?}
    IsGoal -- Yes --> Recon[Reconstruct Optimal Path using came_from map]
    Recon --> Success([Return Path and Search Statistics])
    IsGoal -- No --> Neighbors[Generate Valid Neighbors<br>Check boundaries, obstacles & corner-cutting]
    Neighbors --> LoopNeighbors{For each neighbor m}
    LoopNeighbors --> CheckNeighborClosed{Is m in Closed Set?}
    CheckNeighborClosed -- Yes --> LoopNeighbors
    CheckNeighborClosed -- No --> CalcG[tentative_g = g(n) + step_cost]
    CalcG --> BetterG{tentative_g < g(m)?}
    BetterG -- Yes --> UpdateM[came_from[m] = n<br>g(m) = tentative_g<br>f(m) = g(m) + h(m)<br>Push m to Open Queue]
    UpdateM --> LoopNeighbors
    BetterG -- No --> LoopNeighbors
```

---

## 10. Computational Complexity of A*

- **Time Complexity:**
  - **Worst Case:** $O(b^d)$ where $b$ is the effective branching factor ($b \le 4$ for 4-direction, $b \le 8$ for 8-direction) and $d$ is the depth of the optimal path. On finite $R \times C$ grids, worst case is $O(V \log V + E)$ where $V = R \cdot C$ vertices and $E \le 8V$ edges using a binary heap.
  - **Best Case:** $O(d)$ when the heuristic is perfectly informed and directs the search straight to the goal without expanding false paths.
- **Space Complexity:**
  - $O(V) = O(R \cdot C)$ because A\* maintains all generated nodes in memory within the `open_set` priority queue and `closed_set`.

---

## 11. Technologies Used

- **Language:** Python 3.10+
- **GUI Engine:** CustomTkinter (Modern rounded UI components, theme switching)
- **Visual Grid & Animation:** Python `tkinter.Canvas` (Anti-aliased vector rendering, fast real-time frame rates)
- **Priority Queue:** Built-in Python `heapq` module
- **Math & Utilities:** Python `math`, `time`, `json`, `dataclasses`, `enum`
- **Testing Framework:** Python standard `unittest`

---

## 12. Installation and Execution Instructions

### Prerequisites
Make sure Python 3.10 or higher is installed.

### Step 1: Install Dependencies
Open PowerShell or Terminal in the project root:
```bash
pip install -r requirements.txt
```

### Step 2: Run Automated Tests
Verify all 30 algorithmic, model, and GUI integration tests:
```bash
python -m unittest discover tests
```

### Step 3: Launch SmartSweep AI
```bash
python main.py
```

---

## 13. Interactive GUI Features & User Guide

1. **Predefined Demo Room:**
   - On launch, the system automatically loads a curated 15 × 15 apartment floor plan featuring a living room sofa, dining table, divider wall with door opening, and strategic dirt spots.
2. **Interactive Editing:**
   - Select an **Interaction Mode** on the left sidebar:
     - `Add / Remove Obstacle`: Click or drag across the grid to draw walls/furniture.
     - `Add Dirt Spot`: Place custom debris spots.
     - `Set Start Position`: Move the vacuum starting dock.
     - `Set Final Goal`: Move the designated final destination.
     - `Erase Cell`: Clear any specific cell.
3. **AI Pathfinding Controls:**
   - Click **Find Optimal Path (A\*)** to compute the route instantly.
   - Toggle **Step-by-Step Exploration** to watch the A\* search unfold node by node in real time.
   - Toggle **Diagonal Moves (8-Dir)** to switch between Manhattan and Octile movement.
4. **Playback & Telemetry:**
   - Click **Start Cleaning** to animate the robot.
   - Use **Pause / Resume**, **Step Forward**, and **Reset Simulation**.
   - Adjust the **Movement Speed** slider in real-time.
   - Observe live metrics on the right: Path Cost, Distance Traveled, Nodes Explored, Time in ms, and Dirt Cleaning Progress %.

---

## 14. Testing & Verification Results

SmartSweep AI includes a comprehensive suite of **30 automated unit and integration tests** located in `tests/`:

1. `test_straight_unobstructed_route`: Verifies direct linear route accuracy.
2. `test_route_around_obstacle`: Confirms optimal detour around blocking walls.
3. `test_narrow_passage`: Validates single-cell doorway navigation.
4. `test_blocked_destination_unreachable`: Detects trapped destinations and returns graceful failure.
5. `test_identical_start_and_goal`: Checks 0-cost single-node resolution.
6. `test_invalid_start_and_goal_coordinates`: Confirms bounds-checking.
7. `test_start_or_goal_inside_obstacle`: Rejects invalid spawn points.
8. `test_diagonal_movement_costs`: Validates $\sqrt{2}$ diagonal cost calculations.
9. `test_diagonal_corner_cutting_prevention`: Proves the robot cannot cut through blocked wall corners.
10. `test_corner_cutting_one_wall_prevention`: Ensures single-wall corner safety.
11. `test_optimality_against_known_grid`: Compares against ground truth manual route.
12. `test_optimality_compared_to_dijkstra_reference`: Compares A\* costs against uniform-cost Dijkstra on randomized grids across 4-dir and 8-dir movement to prove 100% mathematical optimality.
13. `test_room_boundaries` & `test_invalid_dimensions`: Validates grid constraints.
14. `test_add_and_remove_obstacles`: Checks collision and overwrite logic.
15. `test_add_and_clean_dirt`: Validates dirt consumption and restoration.
16. `test_demo_room`: Checks predefined layout integrity.
17. `test_random_room_generation`: Ensures density and boundary constraints.
18. `test_json_save_and_load`: Verifies full room serialization roundtrip.
19. `test_invalid_json_handling`: Validates exception handling for corrupted files.
20. `test_single_destination_planning`: Validates single-target planning.
21. `test_multi_dirt_cleaning_planning`: Validates greedy multi-target sequence.
22. `test_unreachable_dirt_reported`: Confirms unreachable dirt detection.
23. `test_step_forward_movement_and_cleaning`: Tests single step advancement.
24. `test_pause_resume_reset`: Validates state machine transitions.
25. `test_gui_integration`: 6 end-to-end GUI controller tests verifying UI actions, status badges, and reset behavior.

**Execution Result:**
```
Ran 30 tests in ~11s
Status: OK (100% Pass)
```

---

## 15. Advantages and Real-World Relevance

- **Guaranteed Shortest Paths:** Eliminates battery-wasting redundant coverage paths.
- **Visual Explainability:** Makes AI decision-making visible and intuitive for students and evaluators.
- **Robust Corner Handling:** Prevents unrealistic diagonal corner clipping seen in naive grid pathfinders.
- **Graceful Fault Tolerance:** Never crashes on unreachable goals or corrupt JSON files.

---

## 16. Limitations

- **Static Environment Assumption:** The standard A\* algorithm assumes static obstacles. If obstacles move during traversal, the path must be re-planned (e.g., via $D^*$ Lite or replanning triggers).
- **Greedy Multi-Target Sequencing:** Finding the globally optimal route across $N$ dirt locations is NP-hard (TSP). The greedy nearest-dirt heuristic is fast and practical, but may produce slightly longer overall tours than an exact branch-and-bound TSP solver.
- **Grid Discretization:** Physical rooms are continuous. Discretizing into grid cells abstracts away real-world wheel slippage and non-holonomic turning radius dynamics.

---

## 17. Future Enhancements

1. **Dynamic Replanning (D\* Lite / LPA\*):** Update paths in real time when new obstacles appear dynamically.
2. **Genetic Algorithm or 2-Opt for Multi-Dirt TSP:** Optimize the global sequence of dirt spots to achieve near-optimal tours.
3. **Continuous Motion Kinematics:** Incorporate curvature smoothing (Bézier splines or Dubins paths) for physical differential-drive robots.
4. **Hardware Integration:** Export planned waypoint coordinates to an ESP32 / Arduino-based robotic chassis over MQTT or WebSockets.

---

## 18. College PBL Viva / Oral Examination Guide

### Q1: What is SmartSweep AI?
**Answer:** SmartSweep AI is an AI-based desktop simulation of an autonomous robot vacuum cleaner that plans optimal, collision-free navigation paths in a 2D grid room using the A\* search algorithm and cleans scattered dirt spots before docking at a designated destination.

### Q2: What specific problem does it solve?
**Answer:** Conventional low-cost robot cleaners rely on random or semi-random bounce movements that cause redundant energy consumption, missed dirt spots, and entrapment near obstacles. SmartSweep AI uses heuristic AI to calculate the mathematically shortest, obstacle-free path, saving battery and cleaning time.

### Q3: What represents Artificial Intelligence in this project?
**Answer:** The intelligence lies in the **A\* heuristic search algorithm** and the **admissible heuristic evaluation function** $f(n) = g(n) + h(n)$. Rather than blindly searching in all directions like BFS or Dijkstra, the algorithm evaluates domain knowledge to make smart decisions about which cell to explore next.

### Q4: Why is A* chosen over Dijkstra or BFS?
**Answer:** While BFS only works for unweighted steps and Dijkstra explores equally in all directions (wasting computation on cells moving away from the goal), A\* uses an admissible heuristic $h(n)$ to steer the search directly toward the target. This drastically reduces the number of explored nodes and execution time while maintaining a mathematical guarantee of finding the shortest path.

### Q5: What are g(n), h(n), and f(n)?
**Answer:**
- $g(n)$: The exact cost incurred from the start node to node $n$.
- $h(n)$: The heuristic estimate of the remaining cost from node $n$ to the goal.
- $f(n) = g(n) + h(n)$: The estimated total cost of the cheapest path passing through node $n$.

### Q6: What is an admissible heuristic? Why is it crucial?
**Answer:** An admissible heuristic is one that **never overestimates** the true cost to reach the goal ($h(n) \le h^*(n)$). If $h(n)$ were to overestimate, A\* might bypass the truly optimal path in favor of an apparently cheaper one, losing its optimality guarantee.

### Q7: Which heuristics are used in this project?
**Answer:** 
- **Manhattan Distance** for 4-directional movement: $h = |r_1 - r_2| + |c_1 - c_2|$.
- **Octile Distance** for 8-directional movement: $h = \max(\Delta r, \Delta c) + (\sqrt{2} - 1)\min(\Delta r, \Delta c)$.

### Q8: How does the system prevent diagonal corner-cutting?
**Answer:** When moving diagonally between $(r, c)$ and $(r+dr, c+dc)$, the algorithm checks both adjacent orthogonal cells $(r+dr, c)$ and $(r, c+dc)$. If either cell contains an obstacle, the diagonal transition is strictly prohibited because a physical robot chassis cannot squeeze through touching obstacle corners.

### Q9: How does the multi-dirt cleaning mode work?
**Answer:** It uses a greedy nearest-target strategy: at each step, A\* calculates the cost to all remaining reachable dirt locations. The robot targets the dirt with the lowest path cost, navigates and cleans it, repeats for all remaining reachable dirt, and finally navigates to the charging goal.

### Q10: How can this system be extended to a real hardware robot?
**Answer:** By equipping a physical robotic chassis with an ESP32 or Raspberry Pi running ROS (Robot Operating System), using LiDAR/ultrasonic sensors for SLAM (Simultaneous Localization and Mapping) to construct the grid occupancy map, and passing the A\* waypoint coordinates to a PID motor controller for wheel actuation.
=======
# AI-Vacuum-Cleaner-AStar
AI-based robot vacuum cleaner path planner using A* algorithm in Python

