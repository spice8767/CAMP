# campus_nav — Autonomous Campus Road Navigation & Safety Stack

`campus_nav` is the core autonomy package for the `saye` electric buggy. It provides OpenStreetMap (OSM) graph ingestion, A* global path planning, 3.8m road ribbon generation, curvature-adaptive Pure Pursuit path following, a proactive 3D LiDAR/camera safety pipeline with a live 2D tactical radar feed, and an autonomous shuttle mission controller.

---

## Architecture Overview

```
                      +-----------------------------------+
                      |   OpenStreetMap Campus Graph      |
                      | (campus_with_junctions_and_stops) |
                      +-----------------+-----------------+
                                        |
+-------------------+                   v
| /campus/mission   |-------> [ campus_planner_node ] -------> /campus/global_path
| /campus/goal_stop |                   |              -------> /campus/route_ribbon
+-------------------+                   |              -------> /campus/road_graph
                                        v
+-------------------+         [ path_follower_node ] --------> /cmd_vel
| /campus/safety_status |----> (Pure Pursuit Control)
+-------------------+                   ^
          ^                             |
          |                             |
[ safety_monitor_node ]       [ odom_bridge_node ] <-------- /model/saye/odometry_world
   |-- 3D LiDAR (/cloud)         (TF & Metric Bridge)
   |-- Depth Camera
   v
/campus/lidar_bev_image (2D Radar Feed)
/campus/safety_corridor (Visual Bounds)
/campus/safety_threats  (Threat Clusters)
```

---

## Nodes & Modules

### 1. `planner_node`
* **File:** `campus_nav/planner_node.py`
* **Function:** Ingests `campus_with_junctions_and_stops.osm`, builds a network graph of roads, named campus stops, and intersection nodes, and computes optimal routes using A*.
* **Outputs:**
  * `/campus/global_path` (`nav_msgs/msg/Path`): Centerline waypoint trajectory.
  * `/campus/route_ribbon` (`visualization_msgs/msg/Marker`): Solid 3.8m wide road mesh overlay ("road carpet") marking the active navigation corridor.
  * `/campus/road_graph` & `/campus/stop_markers`: Interactive RViz markers for all 32 campus stops and road segments.

### 2. `path_follower_node`
* **File:** `campus_nav/path_follower_node.py`
* **Function:** Pure Pursuit controller tailored for Ackermann steering kinematics.
* **Features:**
  * Curvature-adaptive speed profiling: Automatically decelerates from cruising speed ($2.22\,\text{m/s} = 8\,\text{km/h}$) down to $0.8\,\text{m/s}$ when approaching tight turns.
  * Lookahead distance dynamically scaled with speed: $L = \text{clamp}(1.5 \cdot v, 2.5\,\text{m}, 6.0\,\text{m})$.
  * Safety arbitration: Reacts instantly to `/campus/safety_status` by halting on `EMERGENCY_STOP`, slowing to $0.8\,\text{m/s}$ on `SLOWDOWN_BUMP`, or injecting lateral steering bias on `EDGE_WARNING`.

### 3. `safety_monitor_node`
* **File:** `campus_nav/safety_monitor_node.py`
* **Function:** Proactive 3D LiDAR and RGB-D depth camera obstacle detection, elevation profiling, and road drop-off supervisor.
* **Features:**
  * **Dynamic Lookahead Corridor:** Longitudinal corridor extending from $2.0\,\text{m}$ to $8.5\,\text{m}$ ahead based on current vehicle velocity.
  * **Geometric Elevation Profiling:**
    * Ground plane estimation ($<4\,\text{cm}$ relative height): Identified as normal drivable asphalt.
    * Speed breaker detection ($4\,\text{cm} - 16\,\text{cm}$): Triggers `SLOWDOWN_BUMP` ($0.8\,\text{m/s}$).
    * Obstacle detection ($>16\,\text{cm}$): Triggers `EMERGENCY_STOP` ($0.0\,\text{m/s}$) for humans, dogs, vehicles, and barriers.
  * **Flank Void & Cliff Guard:** Monitors $\pm 0.8\,\text{m}$ to $\pm 2.2\,\text{m}$ along wheel flanks. Detects elevation drops $>35\,\text{cm}$ off the elevated road mesh and commands steering bias away from the cliff.
  * **Tactical 2D BEV Radar Feed:** Publishes `/campus/lidar_bev_image` (a $400 \times 400$ top-down radar image with $5\text{m}/10\text{m}/15\text{m}/20\text{m}$ distance rings, vehicle footprint, color-coded returns, and real-time HUD status banner).

### 4. `mission_controller_node`
* **File:** `campus_nav/mission_controller_node.py`
* **Function:** Autonomous Campus Shuttle orchestrator supporting multi-leg missions (`"StartStop|GoalStop"`).
* **Protocol:**
  1. Navigates autonomously from current location to `StartStop` (Pick-up leg).
  2. Halts at `StartStop` and pauses for **5 seconds** for passenger boarding.
  3. Navigates from `StartStop` to `GoalStop` (Drop-off leg).
  4. Supports single-destination goals via `/campus/goal_stop` or RViz **2D Goal Pose** tool.

### 5. `odom_bridge_node`
* **File:** `campus_nav/odom_bridge_node.py`
* **Function:** Metric and TF frame bridge connecting Gazebo's world frame to the planner's map frame.
* **Calibration:** Translates between Gazebo world coords and planner map coords via calibrated offset ($X = 802.0\,\text{m}, Y = 679.7\,\text{m}$).
* **TF Tree:** Broadcasts `map -> world -> base_link -> top_1 -> lidar_link` ensuring clock and TF continuity with zero loop cycles.
* **Void Catch Supervisor:** Automatically detects if the buggy drops below $Z = -0.5\,\text{m}$ (accidental cliff fall) and safely teleports it back to the nearest road node.

### 6. `orient_buggy` & `teleport_buggy`
* **File:** `campus_nav/teleport_buggy.py`
* **Function:** Instant repositioning, road lane alignment, and turnaround CLI tools.
* **Commands:**
  * `ros2 run campus_nav orient_buggy`: Automatically finds the nearest road segment and snaps the buggy's heading perfectly parallel to the road.
  * `ros2 run campus_nav orient_buggy --flip`: Performs an instant $180^\circ$ U-turn facing the opposite road direction.
  * `ros2 run campus_nav teleport_buggy --stop "<STOP_NAME>"`: Instantly teleports the buggy to any of the 32 campus stops, snapped parallel to the road.

### 7. `test_obstacle_spawner`
* **File:** `campus_nav/test_obstacle_spawner.py`
* **Function:** Spawns physical obstacles in Gazebo for reactive testing:
  * `--type human`: Spawns a standing pedestrian in front of the vehicle.
  * `--type speed_breaker`: Spawns an $8\,\text{cm}$ elevated bump perpendicular across the road.
  * `--type dog`: Spawns a low $35\,\text{cm}$ animal obstacle.
  * `--remove`: Removes all test entities.

---

## Topics Reference

| Topic | Type | Description |
|---|---|---|
| `/campus/mission` | `std_msgs/String` | 2-leg mission command: `"PickUpStop\|DropOffStop"` |
| `/campus/goal_stop` | `std_msgs/String` | 1-leg direct goal destination stop name |
| `/campus/global_path` | `nav_msgs/Path` | Global path waypoints generated by A* planner |
| `/campus/route_ribbon` | `visualization_msgs/Marker` | 3.8m wide continuous road carpet mesh |
| `/campus/vehicle_pose` | `geometry_msgs/PoseStamped` | Filtered vehicle pose in `map` frame |
| `/campus/vehicle_marker` | `visualization_msgs/MarkerArray` | 3D visual buggy model in RViz |
| `/campus/safety_status` | `std_msgs/String` | JSON safety arbitration state, speed limit, and threat info |
| `/campus/lidar_bev_image` | `sensor_msgs/Image` | Live 2D Bird's-Eye-View tactical radar stream |
| `/campus/safety_corridor` | `visualization_msgs/Marker` | Dynamic 3D lookahead corridor bounding box |
| `/campus/safety_threats` | `visualization_msgs/MarkerArray` | Clustered obstacle return points |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity and steering commands to Gazebo |
| `/cloud` | `sensor_msgs/PointCloud2` | 3D LiDAR point cloud (1024 azimuth x 128 rings) |
| `/real_sense/image_raw` | `sensor_msgs/Image` | Forward RealSense RGB camera feed |

---

## Configuration Files

* `config/campus_junctions_and_stops.yaml`: Defines coordinates, degree, and connections for all campus road junctions and the 32 named stops.
* `data/campus.osm`: Master OpenStreetMap XML vector map containing roads, walkways, and building footprints.
* `rviz/campus_nav.rviz`: Default RViz configuration with clean 3D campus map, RealSense camera feed panel, and 2D LiDAR tactical radar panel.
* `rviz/solo_lidar.rviz`: Dedicated point-cloud-only RViz configuration for raw 3D LiDAR inspection.
