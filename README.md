# CAMP: Campus Autonomous Mobility Project

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-orange.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-blue.svg)](https://gazebosim.org/docs/harmonic/)
[![Architecture](https://img.shields.io/badge/Stack-Autonomous_Ackermann_Shuttle-success.svg)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)]()

**CAMP** is a complete, production-grade autonomous driving and simulation stack for an electric campus shuttle buggy (`saye`), developed on **ROS 2 Jazzy** and **Gazebo Harmonic**. 

It features OpenStreetMap (OSM) vector graph ingestion, A* global route planning with dynamic 3.8m road ribbon generation, and a fully integrated **Nav2 (ROS 2 Navigation)** stack with `RegulatedPurePursuitController` for trajectory tracking and local costmap obstacle avoidance.

---

## 🆕 Latest Changes (Nav2 Integration)
- **Nav2 Stack Integration:** Replaced the legacy manual `path_follower_node.py` and `safety_monitor_node.py` with the industry-standard **ROS 2 Navigation (Nav2)** stack.
- **Regulated Pure Pursuit:** Configured Nav2's `RegulatedPurePursuitController` to handle aggressive Ackermann steering and dynamic local obstacle avoidance via the Local Costmap (powered by the 3D LiDAR).
- **Virtual Frame Translation:** Solved a legacy CAD issue (buggy drives along `-Y` instead of ROS-standard `+X`) by broadcasting a virtual `nav_base_link` TF frame (-90° rotation) from `odom_bridge_node.py`, allowing Nav2 to operate flawlessly without breaking the original URDF geometry.
- **Nav2 Adapter Bridge:** Created `nav2_adapter_node.py` to seamlessly bridge the original custom `/campus/target_path` topic into Nav2's `/follow_path` Action Server.

---

## 📸 System Architecture

```text
[ mission_controller_node ] 
   |-- Receives 'Start|Goal'
   |-- Queues multi-leg trips
   v
/campus/plan_route (String)
   |
   v
[ campus_planner_node ] 
   |-- OSM Graph A* Search
   |-- Generates visual ribbon
   |                 -------> /campus/global_path (Path)
   |                 -------> /campus/route_ribbon (Marker)
   v
/campus/target_path
   |
   v
[ nav2_adapter_node ]
   |-- Translates topic to action
   v
Nav2 /follow_path (Action)
   |
   v
[ nav2_controller_server ] <----- [ Nav2 Local Costmap ] <--- 3D LiDAR (/cloud)
   |-- (Regulated Pure Pursuit)       (Obstacle Avoidance)
   v
/cmd_vel (Twist)
   |
   v
[ gz-sim-ackermann-plugin ]

(TF Tree: map -> base_link -> nav_base_link provided by odom_bridge_node)
```

---

## 🚀 Quick Start

### 1. Build the Workspace
Open a terminal in the root directory:
```bash
cd ~/CAMP
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### 2. Launch Complete Simulation (One Command — 3 Windows)
Launches Gazebo Harmonic (with NVIDIA GPU offload), spawns the `saye` electric buggy on the road at **SAB C**, starts all sensor bridges, odometry transforms, A* planner, mission controller, Nav2 obstacle avoidance stack, and automatically arranges 3 synchronized operator windows:

```bash
source /opt/ros/jazzy/setup.bash
source /home/adarsh4our/CAMP\ A/install/setup.bash  # or: source install/setup.bash
export DISPLAY=:1
ros2 launch campus_nav full_simulation.launch.py
```

---

## 🖥️ 3-Window Multi-Viewport Operator Dashboard

Running the single launch command automatically opens and arranges three specialized windows across a standard 1080p display:

1. **Window 1: Gazebo Harmonic Sim (Bottom-Right, 960x520 at X=960, Y=520)**
   - High-fidelity 3D simulation of campus roads, physics engine, obstacle spawners, and vehicle dynamics.
   - GPU-accelerated rendering with direct NVIDIA hardware offload.

2. **Window 2: RViz Campus Map & RealSense Camera Feed (Left Half, 960x1040 at X=0, Y=0)**
   - **Main 3D Viewport:** Third-person camera tracking (`base_link`) following the buggy, showing the clean vector road network, all 32 campus stops (`SAB S`, `LOC`, etc.), active 3.8m route ribbon carpet, path centerline, and live 3D `saye` buggy model. *(All LiDAR point feeds and 2D BEV panels are completely removed from Window 2).*
   - **Side Panel — RealSense Camera Feed:** Live RGB forward-facing stream on `/real_sense/image_raw`.

3. **Window 3: RViz Livox Mid-360 LiDAR Cloud Inspection (Top-Right, 960x520 at X=960, Y=0)**
   - **Live 3D Point Cloud (`/cloud`):** Dedicated high-speed inspection of the roof-mounted Livox Mid-360.
   - **Instantaneous 10 Hz Sweeps:** Zero decay time (`Decay Time: 0.0`) for real-time sensor verification without ghosting artifacts.
   - **Optimized Ray Density:** 360 azimuth samples $\times$ 64 vertical rings (~23,000 rays per frame) rendered as 0.12m flat squares for solid, dense surface mapping with zero DDS network lag.
   - **Coloring & Target:** AxisColor rainbow elevation mapping in dark tactical theme, dynamically centered on vehicle `base_link`.

### 📡 Livox Mid-360 Sensor Specifications
- **Horizontal FOV:** 360° omnidirectional azimuth coverage (360 samples, 1° resolution).
- **Vertical FOV:** 75° asymmetric downward aperture ($[-60^\circ, +15^\circ]$ or $[-1.047, 0.262]\,\text{rad}$) with 64 vertical rings.
- **Blind-Spot Elimination:** Lower vertical angle of $-60^\circ$ sweeps the road starting just $0.88\,\text{m}$ from the vehicle center (immediately in front of the front bumper), completely eliminating forward ground blind spots.
- **Effective Range:** $0.1\,\text{m}$ to $70.0\,\text{m}$.

---

## 🕹️ Driving & Mission Dispatching

### 1. Dispatch 2-Leg Autonomous Shuttle Missions (`StartStop|GoalStop`)
Operates as an autonomous campus shuttle: drives to pick up passengers at `StartStop`, waits **5 seconds** for boarding, and drives to `GoalStop`:
```bash
# SAB C -> Chemistry Block:
ros2 topic pub --once /campus/mission std_msgs/msg/String "{data: 'SAB C|Chemistry Block'}"

# LOC -> SAB C:
ros2 topic pub --once /campus/mission std_msgs/msg/String "{data: 'LOC|SAB C'}"

# Ravi Back -> Chemistry Block:
ros2 topic pub --once /campus/mission std_msgs/msg/String "{data: 'Ravi Back|Chemistry Block'}"
```

### 2. Direct 1-Leg Navigation
Drive directly from the buggy's current location to a destination:
```bash
ros2 topic pub --once /campus/goal_stop std_msgs/msg/String "{data: 'Chemistry Block'}"
```
*(Or click anywhere along a road in RViz using the **2D Goal Pose** tool on the top toolbar).*

### 3. Orient Buggy Parallel to Road & Turn Around (180°)
Instant heading correction and turnaround tools:
```bash
# Snap buggy perfectly parallel to the nearest road lane:
ros2 run campus_nav orient_buggy

# Rotate buggy 180 degrees (U-turn / reverse direction on road):
ros2 run campus_nav orient_buggy --flip

# Rotate by arbitrary angle (e.g. +90° left, -90° right):
ros2 run campus_nav orient_buggy --rotate 90
```

### 4. Teleport Buggy to Any Campus Stop
```bash
# Teleport to SAB C (automatically aligned to road):
ros2 run campus_nav teleport_buggy --stop "SAB C"

# Teleport to LOC:
ros2 run campus_nav teleport_buggy --stop "LOC"

# Teleport to Ravi Back (and face reverse direction):
ros2 run campus_nav teleport_buggy --stop "Ravi Back" --flip
```

### 5. Spawn Dynamic Test Obstacles
```bash
# Spawn a standing pedestrian 6m ahead:
ros2 run campus_nav test_obstacle_spawner --type human --distance 6.0

# Spawn an elevated speed breaker 8m ahead:
ros2 run campus_nav test_obstacle_spawner --type speed_breaker --distance 8.0

# Spawn a low dog / animal 5m ahead:
ros2 run campus_nav test_obstacle_spawner --type dog --distance 5.0

# Despawn all test obstacles:
ros2 run campus_nav test_obstacle_spawner --remove
```

### 6. Manual WASD Video-Game Teleop
```bash
ros2 run campus_nav wasd_teleop
```

---

## 📦 Packages in this Repository

| Package | Description |
|---|---|
| [`campus_nav`](src/campus_nav/) | Core autonomy stack: OSM graph loader, A* planner, dynamic road ribbon generator, mission controller, Nav2 adapter bridge, and teleport/orient CLI tools. |
| [`saye_description`](src/saye_description/) | Robot description: Physics-accurate URDF/Xacro, 3D meshes, 3D LiDAR (1024x128), RealSense RGB-D camera, IMU, and Gazebo world files. |
| [`saye_bringup`](src/saye_bringup/) | System bringup: `ros_gz_bridge` parameter mapping, Nav2 stack configurations, SLAM Toolbox, and AMCL configs. |
| [`saye_msgs`](src/saye_msgs/) | Custom ROS 2 message and service interfaces (`Map.msg`, `ShareMap.srv`). |
| [`CART_description`](src/CART_description/) | Baseline legacy golf cart model. |

---

## 📐 Coordinate Systems & Calibration

* **Buggy CAD Geometry:**
  * Front axle and steering knuckles: $y = -0.40\,\text{m}$ (Buggy front is along the **$-Y$** axis of `base_link`).
  * Rear drive axle: $y = +1.72\,\text{m}$ (Buggy rear is along the **$+Y$** axis of `base_link`).
  * Forward spawn heading: $\text{yaw} = \pi$ ($3.14159$) points $-Y$ North along $+Y$ of Gazebo.
* **Map $\leftrightarrow$ Gazebo Transform:**
  * Metric offset: $X = 802.0\,\text{m}, Y = 679.7\,\text{m}$.
  * Managed by `odom_bridge_node` via `TransformBroadcaster` with zero cycle dependencies.

---

## 📖 Additional Documentation

* **[NAV2_INTEGRATION.md](NAV2_INTEGRATION.md):** Detailed "Before & After" change catalogue documenting the shift from legacy manual scripts to the Nav2 navigation stack.
* **[COMMANDS.md](COMMANDS.md):** Complete cheat sheet with all build, launch, teleport, telemetry, and debugging commands.
* **[BUG_AUDIT.md](BUG_AUDIT.md):** Full technical audit log of resolved physics, TF, and parameter bridge bugs.
* **[campus_nav Documentation](src/campus_nav/README.md):** Detailed guide to navigation algorithms and topics.
* **[saye_description Documentation](src/saye_description/README.md):** Vehicle kinematics, sensors, and Gazebo plugins.
* **[saye_bringup Documentation](src/saye_bringup/README.md):** Bridge mappings and Nav2 configurations.

---

## 📄 License
This project is licensed under the [MIT License](src/CART_description/LICENSE).
