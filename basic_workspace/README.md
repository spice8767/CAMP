# CAMP: Campus Autonomous Mobility Project

Welcome to the CAMP Gazebo simulation environment! This repository contains a fully configured ROS 2 (Jazzy) and Gazebo Harmonic simulation for a realistic, physics-accurate buggy driving in a campus environment.

## 🚀 Getting Started

Just follow these steps to build and run the simulation:

### 1. Build the Workspace
Open a terminal and navigate to the root of this cloned repository (`CAMP`), then build it using `colcon`:
```bash
cd ~/CAMP  # Or wherever you cloned this repository
source /opt/ros/jazzy/setup.bash
colcon build
```

### 2. Launch the Simulation
Once built, you need to source your newly compiled workspace and launch the Gazebo simulation:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch CART_description gazebo.launch.py
```
*This will open Gazebo, load the 3D campus map, and spawn the autonomous buggy.*

### 3. Drive the Buggy!
We have built a custom, video-game style WASD teleop controller. 
Open a **second terminal**, and run:
```bash
cd ~/CAMP
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run CART_description wasd_teleop
```
A small grey window will appear. **Click on it to focus it**, and use **W/A/S/D** to drive the buggy around the campus!

---

## 🛠️ System Architecture & Setup Details

### The Buggy (URDF/Xacro)
- **Real-World Scale:** The CAD model was heavily processed to ensure accurate physical scaling (a 2.12m wheelbase, matching a standard golf cart).
- **Inertia & Physics:** The mass and inertia matrices were mathematically re-calculated to ensure stability in the Gazebo Dartsim physics engine.
- **Rear-Wheel Drive (RWD):** The buggy is strictly rear-wheel drive. The front wheels are completely passive and are steered via Ackermann knuckles.

### The Physics Plugin
- We use the Gazebo Harmonic `gz-sim-ackermann-steering-system` plugin.

### The Campus Map
- The environment is a custom `map1.obj` loaded into a custom `campus.sdf` world file.
- The map serves as the ground plane—drive off the edge of the road, and you will fall into the void!
- Spawn coordinates are hardcoded in `gazebo.launch.py` to ensure the buggy drops safely onto the pavement on startup.

### The Teleop Controller (`wasd_teleop.py`)
- Standard ROS `teleop_twist_keyboard` does not handle simultaneous keypresses (e.g., holding Gas and Steering at the same time).
- Our custom `tkinter`-based node actively listens for hardware KeyDown and KeyUp events, allowing true simultaneous WASD video-game controls.
- It dynamically calculates the exact mathematical yaw-rate multiplier required to hold the physical steering wheels at a perfect angle, regardless of the vehicle's current speed or direction.
