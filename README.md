# Saye Ackermann Vehicle Gazebo ROS 2 Simulation

A ROS 2 Humble and Gazebo simulation stack for an Ackermann-steering autonomous cart (`saye`)[cite: 4]. Features integrated 3D/2D LiDAR[cite: 4], depth camera[cite: 4], IMU[cite: 4], odometry bridge[cite: 2], and 2D mapping via `slam_toolbox`[cite: 1, 3].

---

## 📋 Table of Contents
- [Prerequisites](#-prerequisites)
- [Quick Start with Docker](#-quick-start-with-docker)
- [Workspace Setup & Build](#-workspace-setup--build)
- [Running the Simulation](#-running-the-simulation)
- [Controls & Teleoperation](#-controls--teleoperation)
- [Running SLAM (Mapping)](#-running-slam-mapping)
- [Topic & Frame Reference](#-topic--frame-reference)
- [Troubleshooting](#-troubleshooting)

---

## 🛠 Prerequisites

* **Host System:** Ubuntu 22.04 LTS
* **Dependencies:** Docker, `xhost` (for X11 GUI forwarding)
* **Middleware:** `rmw_cyclonedds_cpp` (Enforced inside container to handle high-bandwidth sensor streams)

---

## 🐳 Quick Start with Docker

### 1. Allow X11 GUI Access on Host
Run this command on your host machine before launching the container:
```bash
xhost +local:root
```

### 2. Launch the Docker Container
Replace `/path/to/your/ackermann-vehicle-gzsim-ros2` with the absolute path to your local workspace code directory on the host machine:

```bash
sudo docker run -it \
  --name ackermann_sim \
  --hostname ackermann_sim \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=$XAUTHORITY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTHORITY:$XAUTHORITY:ro" \
  --volume="/path/to/your/ackermann-vehicle-gzsim-ros2:/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2" \
  --privileged \
  alitekes1/ackermann_sim:latest
```

---

## 🔨 Workspace Setup & Build

Inside the running container terminal (`root@ackermann_sim:/#`):

### 1. Configure Shell Environment
Set CycloneDDS as the default middleware in `~/.bashrc`:
```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc
source /root/.bashrc
```

### 2. Clean and Build Workspace
```bash
cd /root/colcon_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Running the Simulation

### Terminal 1: Launch Gazebo & Vehicle Spawner
```bash
source /root/colcon_ws/install/setup.bash
ros2 launch saye_bringup saye_spawn.launch.py
```

### Terminal 2: Attach to Container for Secondary Commands
Open a new terminal tab on your host system:
```bash
sudo docker exec -it ackermann_sim bash
```

---

## 🎮 Controls & Teleoperation

To control the Ackermann vehicle using the keyboard:

1. **Install teleop package (if not present):**
   ```bash
   apt-get update && apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard
   ```

2. **Run Teleop Node:**
   ```bash
   source /root/colcon_ws/install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

---

## 🗺 Running SLAM (Mapping)

To run 2D mapping with `slam_toolbox`[cite: 1, 3]:

1. **Launch SLAM with Simulation Time Enabled[cite: 3]:**
   ```bash
   source /root/colcon_ws/install/setup.bash
   ros2 launch saye_bringup slam.launch.py use_sim_time:=true
   ```

2. **Verify Transform Tree:**
   Ensure the `odom -> base_link` transform is active[cite: 1, 4]:
   ```bash
   ros2 run tf2_ros tf2_echo odom base_link
   ```

3. **Save Map:**
   When mapping is complete, use the RViz `SlamToolboxPlugin` panel or run:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/my_map
   ```

---

## 📡 Topic & Frame Reference

### Primary Coordinate Frames
* **`map`**: Global fixed frame[cite: 1]
* **`odom`**: Odometry parent frame[cite: 1, 2]
* **`base_link`**: Robot chassis root link[cite: 1, 4]
* **`lidar_link`**: Primary LiDAR sensor frame[cite: 4]

### Key ROS Topics
| ROS Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command for vehicle control[cite: 2, 4] |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry telemetry[cite: 2, 4] |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR scan stream[cite: 2, 4] |
| `/cloud` | `sensor_msgs/msg/PointCloud2` | 3D PointCloud stream[cite: 2] |
| `/imu` | `sensor_msgs/msg/Imu` | IMU telemetry[cite: 2, 4] |
| `/real_sense/image_raw` | `sensor_msgs/msg/Image` | Camera RGB stream[cite: 2] |

---

## ❓ Troubleshooting

* **`serdata.cpp` or buffer error messages:**  
  Ensure `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set and run `ros2 daemon stop`.
* **GUI / RViz fails to open:**  
  Verify `xhost +local:root` was executed on the host system before launching Docker.
* **`Failed to compute odom pose` in SLAM[cite: 1]:**  
  Verify `use_sim_time:=true` is passed to the launch file[cite: 3] and check that `base_frame` is set to `base_link` in `saye_bringup/config/slam.yaml`[cite: 1].
