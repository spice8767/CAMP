# Campus Autonomous Buggy — Command Cheat Sheet

This file contains all commands to build, launch, teleport, drive, and monitor the autonomous campus buggy (`saye`).

---

## 1. Build Workspace
Run after any code, URDF, or map changes:
```bash
cd '/home/adarsh4our/CAMP'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

To build only specific packages:
```bash
colcon build --symlink-install --packages-select campus_nav saye_description saye_bringup saye_msgs
```

---

## 2. Launch Entire Simulation (One Command — 3 Windows)
Launches Gazebo Harmonic (NVIDIA GPU offload), `saye` buggy (spawned on road at SAB C), `ros_gz_bridge`, A* planner, RViz2 (Map & Camera), RViz2 (Livox Mid-360 LiDAR Cloud), odometry bridge, Pure Pursuit follower, safety monitor, and mission controller:

```bash
source /opt/ros/jazzy/setup.bash
source /home/adarsh4our/CAMP\ A/install/setup.bash  # or: source '/home/adarsh4our/CAMP/install/setup.bash'
export DISPLAY=:1
ros2 launch campus_nav full_simulation.launch.py
```

- **Spawn Location:** SAB C road node at Gazebo `(x=15.02, y=43.00, z=0.80, yaw=3.14159)` facing North along the road towards campus.
- **Buggy Speed:** Max 8.0 km/h (2.22 m/s), automatically slows down at sharp curves with dynamic lookahead tracking.
- **Color & Sensors:** High-visibility safety orange chassis with canopy, roof-mounted Livox Mid-360 LiDAR, RealSense depth camera, and IMU.

### 🖥️ 3-Window Multi-Viewport Display Layout (1920x1080 Screen):
When launched, 3 synchronized windows automatically open in a tiled layout:

1. **Window 1: Gazebo Harmonic Sim (Bottom-Right, 960x520 at X=960, Y=520)**
   - High-fidelity 3D simulation of the campus, physics engine, obstacle spawners, and vehicle dynamics.
   - GPU-accelerated rendering with direct NVIDIA hardware offload.

2. **Window 2: RViz Campus Map & RealSense Camera Feed (Left Half, 960x1040 at X=0, Y=0)**
   - **Main 3D Viewport:** Third-person camera tracking (`base_link`) showing the clean vector road network, all 32 campus stops (`SAB S`, `LOC`, etc.), active 3.8m route ribbon carpet, path centerline, and live 3D `saye` buggy model. *(All LiDAR point feeds and 2D BEV panels are completely removed from Window 2 to keep the map pristine).*
   - **Side Panel — RealSense Camera Feed:** Live RGB forward-facing stream on `/real_sense/image_raw`.

3. **Window 3: RViz Livox Mid-360 LiDAR Inspection (Top-Right, 960x520 at X=960, Y=0)**
   - **Live 3D Point Cloud (`/cloud`):** Dedicated high-speed inspection of the roof-mounted Livox Mid-360.
   - **Instantaneous 10 Hz Sweeps:** Zero decay time (`Decay Time: 0.0`) for real-time sensor verification without ghosting artifacts.
   - **Ultra-Dense Ray Density:** 2048 azimuth samples $\times$ 256 vertical rings (524k points per frame) rendered as 0.12m flat squares for dense, solid surface detection.
   - **Tracking:** Automatically follows the vehicle's `base_link` frame.

### 📡 Livox Mid-360 Sensor Specifications:
- **Horizontal FOV:** 360° omnidirectional azimuth coverage (2048 samples).
- **Vertical FOV:** 60° asymmetric aperture ($[-45^\circ, +15^\circ]$ or $[-0.785, 0.262]\,\text{rad}$) with 256 vertical rings.
- **Blind-Spot Elimination:** Lower vertical angle of $-45^\circ$ sweeps the road starting just $1.6\,\text{m}$ from the vehicle center (immediately in front of the front bumper), completely eliminating forward ground blind spots for speed breakers and pedestrians.
- **Effective Range:** $0.1\,\text{m}$ to $70.0\,\text{m}$.

### Re-open / Standalone RViz Windows Individually:
```bash
# Re-open Window 2 (Campus Nav Map & Camera):
ros2 run rviz2 rviz2 -d "/home/adarsh4our/CAMP/src/campus_nav/rviz/campus_nav.rviz" --ros-args -p use_sim_time:=true

# Re-open Window 3 (Livox Mid-360 LiDAR Cloud):
ros2 run rviz2 rviz2 -d "/home/adarsh4our/CAMP/src/campus_nav/rviz/livox_mid360.rviz" --ros-args -p use_sim_time:=true
```

---

## 3. Spawn, Teleport & Orient Buggy Instantly

### Quick Orientation & 180° Turnaround (At Current Position):
```bash
source /opt/ros/jazzy/setup.bash
source '/home/adarsh4our/CAMP/install/setup.bash'

# 1. Snap buggy exactly parallel to the nearest road lane:
ros2 run campus_nav orient_buggy

# 2. Rotate buggy 180 degrees (U-turn / reverse direction on road):
ros2 run campus_nav orient_buggy --flip
# (or: ros2 run campus_nav orient_buggy --rotate 180)

# 3. Rotate by arbitrary angle (e.g. +90° turn left, -90° turn right):
ros2 run campus_nav orient_buggy --rotate 90
ros2 run campus_nav orient_buggy --rotate -90
```

### Option A: Teleport to Any Named Campus Stop (Automatically Aligned to Road)
```bash
# Teleport to SAB C:
ros2 run campus_nav teleport_buggy --stop "SAB C"

# Teleport to LOC:
ros2 run campus_nav teleport_buggy --stop "LOC"

# Teleport to Ravi Back (and flip 180° to reverse direction):
ros2 run campus_nav teleport_buggy --stop "Ravi Back" --flip

# Teleport to Chemistry Block:
ros2 run campus_nav teleport_buggy --stop "Chemistry Block"

# Teleport to Main Gate:
ros2 run campus_nav teleport_buggy --stop "Main Gate"
```

### Option B: Teleport to Raw Coordinates with Road Alignment
```bash
# Teleport to coordinates and automatically align parallel to road:
ros2 run campus_nav teleport_buggy --x 15.02 --y 43.00 --align-road

# Raw Gazebo service call:
gz service -s /world/campus_world/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'name: "saye", position: {x: 15.02, y: 43.00, z: 0.80}, orientation: {x: 0, y: 0, z: 0.7071, w: 0.7071}'
```

---

## 4. Dispatch Autonomous 2-Leg Missions (`StartStop|GoalStop`)

When you publish a mission, the system operates as an **Autonomous Campus Shuttle**:
1. **Leg 1 (Pick-up):** Buggy autonomously navigates from its current location to `StartStop`.
2. **Boarding:** Buggy stops at `StartStop` and pauses for **5 seconds** for passengers.
3. **Leg 2 (Drop-off):** Buggy drives from `StartStop` to `GoalStop`.
*(If the buggy is already at `StartStop`, it immediately executes Leg 2).*

```bash
source /opt/ros/jazzy/setup.bash
source '/home/adarsh4our/CAMP/install/setup.bash'

# SAB C to Chemistry Block:
ros2 topic pub --once /campus/mission std_msgs/msg/String "{data: 'SAB C|Chemistry Block'}"

# LOC to SAB C:
ros2 topic pub --once /campus/mission std_msgs/msg/String "{data: 'LOC|SAB C'}"

# Ravi Back to Chemistry Block:
ros2 topic pub --once /campus/mission std_msgs/msg/String "{data: 'Ravi Back|Chemistry Block'}"

# SAB C to Auditorium:
ros2 topic pub --once /campus/mission std_msgs/msg/String "{data: 'SAB C|Auditorium'}"

# Main Gate to Library:
ros2 topic pub --once /campus/mission std_msgs/msg/String "{data: 'Main Gate|Nalanda Library'}"
```

---

## 5. Direct Navigation from Current Location (1-Leg Route)
To drive directly from wherever the buggy currently is to a single destination (no pick-up phase):
```bash
ros2 topic pub --once /campus/goal_stop std_msgs/msg/String "{data: 'Chemistry Block'}"
```
*(Or click anywhere along a road using the **2D Goal Pose** tool on the top toolbar in RViz).*

---

## 6. Live Telemetry & Monitoring

### Watch Driving Speed, Steering & Progress:
```bash
ros2 topic echo /campus/pursuit_status
```

### Watch Turn-by-Turn Directions & Route Status:
```bash
ros2 topic echo /campus/nav_status
```

### Watch Live Buggy Coordinates in Planner Frame:
```bash
ros2 topic echo /campus/vehicle_pose
```

### Watch Real-Time Safety & Obstacle Detection:
```bash
ros2 topic echo /campus/safety_status
```

### Check Available Sensor Topics:
```bash
# LiDAR 2D laser scan
ros2 topic echo --once /scan

# 3D LiDAR point cloud (1024x128)
ros2 topic echo --once /cloud

# LiDAR 2D Top-Down Tactical BEV Radar Stream
ros2 topic hz /campus/lidar_bev_image

# Safety Corridor & Threat Visual Markers
ros2 topic echo --once /campus/safety_corridor
ros2 topic echo --once /campus/safety_threats

# IMU telemetry
ros2 topic echo --once /imu

# RealSense RGB camera stream
ros2 topic hz /real_sense/image_raw

# RealSense Depth point cloud
ros2 topic echo --once /real_sense/depth_point

# RealSense Depth image
ros2 topic hz /real_sense/depth_image

# Ground-truth odometry
ros2 topic echo --once /model/saye/odometry_world
```

### Spawn Dynamic Obstacles & Speed Breakers:
```bash
# Spawn a standing pedestrian 6m in front of the buggy:
ros2 run campus_nav test_obstacle_spawner --type human --distance 6.0

# Spawn a road speed breaker 8m ahead:
ros2 run campus_nav test_obstacle_spawner --type speed_breaker --distance 8.0

# Spawn a low animal / dog 5m ahead:
ros2 run campus_nav test_obstacle_spawner --type dog --distance 5.0

# Despawn all test obstacles:
ros2 run campus_nav test_obstacle_spawner --remove
```

---

## 7. Manual WASD Teleop
Drive manually with keyboard GUI window:
```bash
source /opt/ros/jazzy/setup.bash
source '/home/adarsh4our/CAMP/install/setup.bash'
export DISPLAY=:1
ros2 run campus_nav wasd_teleop
```

---

## 8. List All 32 Campus Stops
```bash
python3 -c "
import sys; sys.path.insert(0, '/home/adarsh4our/CAMP/src/campus_nav')
from campus_nav.osm_loader import CampusMap
c = CampusMap('/home/adarsh4our/CAMP/campus_with_junctions_and_stops.osm')
print('\n'.join(f' - {name}' for name in sorted(c.stops.keys())))
"
```
