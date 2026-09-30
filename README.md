# CAMP: Campus Autonomous Mobility Project

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-orange.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-blue.svg)](https://gazebosim.org/docs/harmonic/)
[![Architecture](https://img.shields.io/badge/Stack-Autonomous_Ackermann_Shuttle-success.svg)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)]()

**CAMP** is a complete, production-grade autonomous driving and simulation stack for an electric campus shuttle buggy (`saye`), developed on **ROS 2 Jazzy** and **Gazebo Harmonic**. 

It features OpenStreetMap (OSM) vector graph ingestion, A* global route planning with dynamic 3.8m road ribbon generation, curvature-adaptive Pure Pursuit trajectory tracking, a proactive 3D LiDAR/RGB-D camera safety pipeline with real-time speed breaker detection, and a dual-feed RViz visualization dashboard.

---

## 📸 System Architecture

```
                       +---------------------------------------+
                       |    OpenStreetMap Vector Campus Map    |
                       |   (campus_with_junctions_and_stops)   |
                       +-------------------+-------------------+
                                           |
+----------------------+                   v
| /campus/mission      |----------> [ campus_planner_node ] --------> /campus/global_path
| /campus/goal_stop    |                   |                 -------> /campus/route_ribbon
+----------------------+                   |                 -------> /campus/road_graph
                                           v
+----------------------+         [ path_follower_node ] ------------> /cmd_vel
| /campus/safety_status|-------> (Pure Pursuit Control)
+----------------------+                   ^
           ^                               |
           |                               |
[ safety_monitor_node ]          [ odom_bridge_node ] <-------------- /model/saye/odometry_world
   |-- 3D LiDAR (/cloud)            (TF & World Bridge)
   |-- Depth Camera
   v
/campus/lidar_bev_image (2D Radar Feed)
/campus/safety_corridor (Visual Bounds)
/campus/safety_threats  (Threat Clusters)
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

### 2. Launch Complete Simulation (One Command)
Launches Gazebo Harmonic (with NVIDIA GPU offload), spawns the `saye` electric buggy on the road at **SAB C**, starts all sensor bridges, odometry transforms, A* planner, Pure Pursuit follower, safety monitor, and RViz2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export DISPLAY=:1
ros2 launch campus_nav full_simulation.launch.py
```

---

## 🖥️ RViz2 Visualizer Dashboard

The RViz configuration (`campus_nav.rviz`) provides a clean, information-rich operator interface:

1. **Main 3D Viewport:**
   * Clean campus road network map and all 32 named stop stations (`SAB S`, `LOC`, `Ravi Back`, etc.).
   * Active 3.8m wide solid road carpet ribbon (`/campus/route_ribbon`) showing exact drivable lane boundaries.
   * Trajectory centerline and live 3D visual `saye` buggy model.
   * *(3D point cloud points are kept off the main 3D map to eliminate visual clutter).*

2. **Side Panel 1 — RealSense Camera Feed:**
   * Live forward-facing RGB camera stream (`/real_sense/image_raw`).

3. **Side Panel 2 — LiDAR 2D BEV Tactical Radar Feed:**
   * Real-time top-down radar stream (`/campus/lidar_bev_image`) generated directly from the 3D LiDAR cloud.
   * $5\,\text{m}, 10\,\text{m}, 15\,\text{m}, 20\,\text{m}$ concentric distance rings.
   * Color-coded classification: Flat road (cyan), speed bumps (bright yellow), obstacles/pedestrians (bright red).
   * Dynamic safety corridor bounding box and vehicle footprint with heading vector.
   * Real-time HUD banner displaying current safety state (`CLEAR` / `SLOWDOWN_BUMP` / `EMERGENCY_STOP`), speed, and obstacle proximity.

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
| [`campus_nav`](src/campus_nav/) | Core autonomy stack: OSM graph loader, A* planner, road ribbon generator, Pure Pursuit follower, 3D LiDAR/camera safety monitor, mission controller, and teleport/orient CLI tools. |
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

* **[COMMANDS.md](COMMANDS.md):** Complete cheat sheet with all build, launch, teleport, telemetry, and debugging commands.
* **[BUG_AUDIT.md](BUG_AUDIT.md):** Full technical audit log of resolved physics, TF, and parameter bridge bugs.
* **[campus_nav Documentation](src/campus_nav/README.md):** Detailed guide to navigation algorithms and topics.
* **[saye_description Documentation](src/saye_description/README.md):** Vehicle kinematics, sensors, and Gazebo plugins.
* **[saye_bringup Documentation](src/saye_bringup/README.md):** Bridge mappings and Nav2 configurations.

---

## 📄 License
This project is licensed under the [MIT License](src/CART_description/LICENSE).
