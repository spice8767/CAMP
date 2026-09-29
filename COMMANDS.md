# Campus Autonomous Buggy — Command Cheat Sheet

This file contains all commands to build, launch, teleport, drive, and monitor the autonomous campus buggy (`saye`).

---

## 1. Build Workspace
Run after any code, URDF, or map changes:
```bash
cd '/home/adarsh4our/CAMP A'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

To build only specific packages:
```bash
colcon build --symlink-install --packages-select campus_nav saye_description saye_bringup saye_msgs
```

---

## 2. Launch Entire Simulation (One Command)
Launches Gazebo Harmonic (NVIDIA GPU offload), `saye` buggy (spawned on road at SAB C), `ros_gz_bridge`, A* planner, RViz2, odometry bridge, Pure Pursuit follower, safety monitor (with 2D LiDAR radar feed), and mission controller:

```bash
source /opt/ros/jazzy/setup.bash
source '/home/adarsh4our/CAMP A/install/setup.bash'
export DISPLAY=:1
ros2 launch campus_nav full_simulation.launch.py
```

- **Spawn Location:** SAB C road node at Gazebo `(x=15.02, y=43.00, z=0.80, yaw=3.14159)` facing North along the road towards campus.
- **Buggy Speed:** Max 8.0 km/h (2.22 m/s), automatically slows down at sharp curves with dynamic lookahead tracking.
- **Color & Sensors:** High-visibility safety orange chassis with canopy, roof-mounted 3D LiDAR, front camera, RealSense depth camera, and IMU.

### RViz2 Window Layout:
- **Main 3D Viewport:** Clean campus map view showing the road network, active 3.8m route carpet, campus stops (`SAB S`), path centerline, and the 3D `saye` buggy. *(3D LiDAR point cloud is disabled from the main map to prevent visual clutter).*
- **Side Panel 1 — RealSense Camera Feed:** Live forward-facing camera stream on `/real_sense/image_raw`.
- **Side Panel 2 — LiDAR 2D BEV Feed:** Live top-down tactical radar on `/campus/lidar_bev_image`:
  - Concentric distance rings: $5\,\text{m}, 10\,\text{m}, 15\,\text{m}, 20\,\text{m}$.
  - Color-coded returns: Flat road (cyan), speed bumps (bright yellow), obstacles/pedestrians (bright red).
  - Dynamic safety corridor bounding box and vehicle footprint.
  - HUD header with real-time speed, safety arbitration state, and obstacle proximity.

### Re-open / Standalone RViz (Standard Campus Nav):
```bash
ros2 run rviz2 rviz2 -d "/home/adarsh4our/CAMP A/src/campus_nav/rviz/campus_nav.rviz" --ros-args -p use_sim_time:=true
```

### Solo 3D LiDAR Point Cloud Map View (Raw 3D Points Only):
To switch RViz to a dedicated dark view showing only the raw 3D LiDAR point cloud with rainbow elevation coloring:
```bash
ros2 run rviz2 rviz2 -d "/home/adarsh4our/CAMP A/src/campus_nav/rviz/solo_lidar.rviz" --ros-args -p use_sim_time:=true
```

---

## 3. Spawn, Teleport & Orient Buggy Instantly

### Quick Orientation & 180° Turnaround (At Current Position):
```bash
source /opt/ros/jazzy/setup.bash
source '/home/adarsh4our/CAMP A/install/setup.bash'

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
source '/home/adarsh4our/CAMP A/install/setup.bash'

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
source '/home/adarsh4our/CAMP A/install/setup.bash'
export DISPLAY=:1
ros2 run campus_nav wasd_teleop
```

---

## 8. List All 32 Campus Stops
```bash
python3 -c "
import sys; sys.path.insert(0, '/home/adarsh4our/CAMP A/src/campus_nav')
from campus_nav.osm_loader import CampusMap
c = CampusMap('/home/adarsh4our/CAMP A/campus_with_junctions_and_stops.osm')
print('\n'.join(f' - {name}' for name in sorted(c.stops.keys())))
"
```
