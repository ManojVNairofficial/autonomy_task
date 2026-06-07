# TurtleBot3 Autonomy Assignment (ROS 2 Humble)

Autonomous **Exploration + SLAM**, **Agentic Semantic Reasoning**, and a **custom RRT planner**
for the TurtleBot3 Burger, fully simulated in Gazebo and visualized in RViz.

> Assignment brief: [`autonomy_takehome.pdf`](autonomy_takehome.pdf)
> Target platform: **ROS 2 Humble**, **TurtleBot3 Burger**, **Gazebo Classic 11**, **RViz2**.

---

## Demo videos

| Assignment | Demo | Notes |
|---|---|---|
| 1 — Exploration & SLAM | [`demo_/task1/task_1.mp4`](demo_/task1/task_1.mp4) | Saved map preview: [`map.png`](demo_/task1/map.png) |
| 2 — Semantic Reasoning | [`demo_/task2/task_2.mp4`](demo_/task2/task_2.mp4) | Tag rooms, then `"Where is the toilet?"` → robot navigates there |
| 3 — RRT Planner | [`demo_/task3/task_3.mp4`](demo_/task3/task_3.mp4) | Click Start/Goal in RViz → planned `nav_msgs/Path` visualized |

---

## System architecture

```
                          ┌───────────────────────────────────────────────┐
                          │                  GAZEBO CLASSIC               │
                          │   TurtleBot3 (LiDAR + camera) in office world │
                          └───────────────────────────────────────────────┘
                              │ /scan  /odom  /tf        │ /camera/image_raw
                              ▼                          ▼
   ┌────────────────────────────────────────────┐   ┌────────────────────────────────────┐
   │            ASSIGNMENT 1  (SLAM)            │   │     ASSIGNMENT 2  (Semantics)      │
   │                                            │   │                                    │
   │  slam_toolbox  ──/map──►  Nav2 stack       │   │  semantic_tagger_node              │
   │  (online async)          (planner+control) │   │   ├─ VLMInterface (mock | CLIP)    │
   │        │                       ▲           │   │   ├─ classify frame → room label   │
   │        │ /map                  │ Navigate  │   │   ├─  TF: map ← base_footprint     │
   │        ▼                       │           │   │   └─ writes ~/semantic_map.json    │
   │  frontier_explorer ───goals────┘           │   │           │                        │
   │  (detect frontiers, send Nav2 goals)       │   │           ▼                        │
   │        │                                   │   │  semantic_query_node               │
   │        ▼                                   │   │   ├─ /query_place (QueryPlace.srv) │
   │  map_saver_cli → office_map.{pgm,yaml}     │   │   ├─ resolve_query (synonym+fuzzy) │
   └────────────────────┬───────────────────────┘   │   └─ NavigateToPose → Nav2         │
                        │ saved static map          └──────────────────┬─────────────────┘
                        │                                              │ navigate
                        ▼                                              ▼
   ┌─────────────────────────────────────────────────────────────────────────────────────┐
   │                          ASSIGNMENT 3  (RRT Planner)                                │
   │                                                                                     │
   │  nav2_map_server ──/map (OccupancyGrid, TRANSIENT_LOCAL)──►  rrt_planner_node       │
   │                                                              ├─ inflate obstacles   │
   │  RViz plugin / tools:                                        ├─ RRTPlanner.plan()   │
   │    SetRRTStartTool (S) → /start_pose ───────────────────►   ├─ /rrt_path  (Path)    │
   │    SetRRTGoalTool  (G) → /goal_pose  ──(triggers plan)──►   └─ /rrt_tree  (Markers) │
   └─────────────────────────────────────────────────────────────────────────────────────┘
                                              │
                                              ▼  visualized in RViz2
```

---

## Build

```bash
# Prerequisites (Humble)
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 \
                 ros-humble-nav2-bringup ros-humble-turtlebot3-gazebo \
                 ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge \
                 ros-humble-rviz-common ros-humble-rviz-default-plugins
pip install numpy scipy opencv-python

# Build the workspace
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger     # waffle_pi is set automatically for Assignment 2
```

---

## Assignment 1 — Exploration & SLAM

The robot autonomously explores an unknown office with frontier-based exploration, builds a 2D
occupancy grid with `slam_toolbox`, and uses Nav2 for obstacle-avoiding navigation.

**Launch (Terminal 1):**
```bash
ros2 launch tb3_exploration_slam exploration_slam.launch.py
```
Brings up: Gazebo + `slam_toolbox` (online async) + Nav2 + `frontier_explorer` + RViz.

Optional world / spawn arguments:
```bash
ros2 launch tb3_exploration_slam exploration_slam.launch.py world:=turtlebot3_world x_pose:=1.0 y_pose:=0.0
ros2 launch tb3_exploration_slam exploration_slam.launch.py world:=office          # custom office world
```

**Save the map (Terminal 2, while exploration is running):**
```bash
ros2 launch tb3_exploration_slam save_map.launch.py
ros2 launch tb3_exploration_slam save_map.launch.py map_name:=office_map   # → ~/maps/office_map.{pgm,yaml}
```

A pre-saved map ships in [`src/tb3_exploration_slam/maps/office_map.{pgm,yaml}`](src/tb3_exploration_slam/maps).

**Key parameters** (`frontier_explorer`): `min_frontier_size`, `goal_timeout_sec`,
`tried_goal_radius`, `min_frontier_dist`, `max_stuck_resets`, `explore_hz`.

---

## Assignment 2 — Agentic Semantic Reasoning

The robot tags locations with semantic labels (kitchen / toilet / office / hallway) as it explores,
stores them in a JSON semantic map, and later resolves a free-form text query to a stored place and
navigates there. Queries are **not hard-coded** — a synonym + fuzzy-matching engine handles them.

**Launch (Terminal 1):**
```bash
ros2 launch tb3_semantic_mapping semantic_mapping.launch.py
```
Brings up: office Gazebo world (with room signs) + SLAM + Nav2 + `frontier_explorer`
+ `semantic_tagger` + `semantic_query` service + RViz. `TURTLEBOT3_MODEL=waffle_pi` is set
automatically (camera required).

**Query a place (Terminal 2):**
```bash
ros2 service call /query_place tb3_semantic_interfaces/srv/QueryPlace "{query: 'Where is the toilet?'}"
ros2 service call /query_place tb3_semantic_interfaces/srv/QueryPlace "{query: 'take me to the pantry'}"
```
A successful query returns `found / label / pose / confidence / message` **and** sends a
`NavigateToPose` goal so the robot drives to the matched room.

**Design (capture → store → retrieve):**
- **Capture** — `semantic_tagger_node` throttles `/camera/image_raw`, classifies each frame with a
  pluggable `VLMInterface`. The default **`MockVLM`** uses HSV colour segmentation + template matching
  on the room sign artwork; **`ClipVLM`** is a drop-in real VLM (OpenAI CLIP zero-shot) with the
  identical `VLMResult` contract — swap with the `vlm_backend` parameter.
- **Store** — the robot's pose at the moment of detection (via TF `map ← base_footprint`) is recorded
  per label, keeping the highest-confidence observation, into `~/semantic_map.json`
  (plus snapshots in `~/semantic_snapshots/` and RViz text/sphere markers on `/semantic_map`).
- **Retrieve** — `semantic_query_node` exposes `/query_place`; `resolve_query()` maps natural words
  to canonical labels (synonyms → +1.0, fuzzy/typos/plurals → +0.6) and returns the best match,
  then triggers navigation.

> **Real VLM integration:** set the tagger's `vlm_backend:='clip'` (`pip install torch ftfy clip`).
> No other code changes — `MockVLM` and `ClipVLM` both return a `VLMResult(label, confidence, description)`.

---

## Assignment 3 — Custom RRT Planner

A from-scratch RRT planner node that consumes a static `nav_msgs/OccupancyGrid` and plans between a
start and goal pose, publishing the path and the search tree for RViz.

**Launch:**
```bash
ros2 launch tb3_rrt_planner rrt_planner.launch.py
```
Brings up: `nav2_map_server` (lifecycle, auto-activated) + `rrt_planner_node` + RViz with the RRT
plugin loaded. Default map: [`src/tb3_rrt_planner/maps/office_map.yaml`](src/tb3_rrt_planner/maps)
(`turtlebot3_world.yaml` also provided).

**Set start & goal** — in RViz use the custom tools from `tb3_rrt_rviz_plugin`:
- **Set RRT Start** (shortcut **S**) → publishes `/start_pose`
- **Set RRT Goal** (shortcut **G**) → publishes `/goal_pose` and **immediately triggers planning**

Or publish poses from the CLI:
```bash
ros2 topic pub --once /start_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: -1.5, y: -1.5}, orientation: {w: 1.0}}}"
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 1.5}, orientation: {w: 1.0}}}"
```

---

## Notes & assumptions

- All tasks run in **Gazebo Classic** with `use_sim_time` enabled (Assignment 3 defaults to
  `use_sim_time:=false` since it replays a static map).
- Launch files stagger startup with `TimerAction` delays so Nav2/SLAM/RViz come up in order; allow a
  few seconds after launch before interacting.
- Assignment 2 uses **`waffle_pi`** (it has a camera); Assignments 1 & 3 use **`burger`**.
