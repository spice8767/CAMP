# 🔄 Nav2 Integration: Change Catalogue

This document outlines the architectural shift from the legacy manual control system to the industry-standard **ROS 2 Navigation (Nav2)** stack.

## 1. Path Following Control
**Before:**
- Handled by a custom python script (`path_follower_node.py`).
- Used basic mathematical pure pursuit. If it drifted too far or the turn was too sharp, it lacked the ability to dynamically slow down or scale velocity based on curvature.
- Blindly followed points without any awareness of obstacles in the environment.

**Now:**
- Handled by Nav2's `nav2_controller_server` running the **Regulated Pure Pursuit (RPP) Controller**.
- RPP dynamically scales velocity based on path curvature, actively brakes for obstacles, and natively handles complex Ackermann steering constraints.

## 2. Obstacle Avoidance & Safety
**Before:**
- Handled by a custom `safety_monitor_node.py`.
- Hardcoded to perform a binary "Emergency Stop" if the 3D LiDAR detected an obstacle in a fixed rectangular corridor ahead of the buggy. It could not drive around obstacles.

**Now:**
- Handled by the **Nav2 Local Costmap**.
- The 3D LiDAR point cloud (`/cloud`) is fed directly into a heavily optimized 2D occupancy grid costmap. The controller uses this costmap to gracefully brake and navigate around static obstacles on the road.

## 3. Mission Dispatching Bridge
**Before:**
- `campus_planner_node` generated a path and published it as a continuous topic (`/campus/target_path`). 
- `path_follower_node` listened to this topic directly.

**Now:**
- `campus_planner_node` still publishes the custom topic (preserving the original architecture).
- Introduced **`nav2_adapter_node.py`**. This lightweight bridge listens to `/campus/target_path` and securely translates it into standard Nav2 `/follow_path` Action requests, bridging the gap between the custom planner and the Nav2 ecosystem.

## 4. Coordinate Frame Alignment (The "Sideways" Buggy Fix)
**Before:**
- The CAD model/URDF of the buggy was built rotated by -90 degrees (the physical front bumper was on the Y-axis instead of the ROS-standard X-axis).
- The manual path follower script contained hardcoded math hacks (`yaw - pi/2`) to force the buggy to drive sideways to compensate.

**Now:**
- **No changes made to the URDF!** (Original physics architecture is preserved).
- We added a "Virtual Frame Translator" in `odom_bridge_node.py`. It broadcasts a new invisible coordinate frame called `nav_base_link` that is permanently rotated by +90 degrees.
- Nav2 is configured to use `nav_base_link` as its `robot_base_frame`. Nav2 commands the buggy in standard ROS coordinates, and the virtual frame seamlessly translates those commands to the buggy's physical hardware.

## 5. Launch File Modernization
**Before:**
- `full_simulation.launch.py` executed `path_follower_node` and `safety_monitor_node`.

**Now:**
- Those legacy nodes are removed.
- The launch file now executes `nav2_lifecycle_manager`, `nav2_controller`, and `nav2_adapter_node`, booting up the entire navigation stack in sync with the physics engine.
