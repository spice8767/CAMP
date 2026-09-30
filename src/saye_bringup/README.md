# saye_bringup — Simulation Bridge, Launch & Navigation Configurations

`saye_bringup` provides launch files, Gazebo-to-ROS bridges, and navigation stack configurations (Nav2, SLAM Toolbox, AMCL) for the `saye` autonomous campus buggy.

---

## ROS ↔ Gazebo Parameter Bridge (`config/ros_gz_bridge.yaml`)

The package configures `ros_gz_bridge` (`parameter_bridge`) to seamlessly map topics between Gazebo Harmonic and ROS 2 Jazzy:

| ROS Topic | Gazebo Topic | ROS Type | Direction |
|---|---|---|---|
| `/cmd_vel` | `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS $\rightarrow$ Gazebo |
| `/clock` | `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo $\rightarrow$ ROS |
| `/joint_states` | `joint_state` | `sensor_msgs/msg/JointState` | Gazebo $\rightarrow$ ROS |
| `/model/saye/odometry_world` | `/model/saye/odometry_world` | `nav_msgs/msg/Odometry` | Gazebo $\rightarrow$ ROS |
| `/tf` | `/model/saye/tf` | `tf2_msgs/msg/TFMessage` | Gazebo $\rightarrow$ ROS |
| `/imu` | `/imu` | `sensor_msgs/msg/Imu` | Gazebo $\rightarrow$ ROS |
| `/cloud` | `/scan/points` | `sensor_msgs/msg/PointCloud2` | Gazebo $\rightarrow$ ROS |
| `/scan` | `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo $\rightarrow$ ROS |
| `/real_sense/image_raw` | `/rs_front/image` | `sensor_msgs/msg/Image` | Gazebo $\rightarrow$ ROS |
| `/real_sense/camera_info` | `/rs_front/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo $\rightarrow$ ROS |
| `/real_sense/depth_image` | `/rs_front/depth_image` | `sensor_msgs/msg/Image` | Gazebo $\rightarrow$ ROS |
| `/real_sense/depth_point` | `/rs_front/points` | `sensor_msgs/msg/PointCloud2` | Gazebo $\rightarrow$ ROS |

*Note: Critical sensor topics are configured with `lazy: false` so that Gazebo sensors are permanently activated and publish continuously.*

---

## Launch Files

* `launch/saye_spawn.launch.py`: Spawns the `saye` buggy entity into an active Gazebo world.
* `launch/navigation_bringup.launch.py`: Launches standard Nav2 lifecycle nodes, planners, controllers, and costmaps.
* `launch/slam.launch.py`: Launches `slam_toolbox` for 2D SLAM mapping using planar laser scan slices.
* `launch/amcl.launch.py`: Launches Adaptive Monte Carlo Localization (AMCL) against a pre-built 2D occupancy grid.

---

## Configuration Files

* `config/nav2_params.yaml`: Nav2 stack parameters (costmaps, recovery behaviors, behavior trees).
* `config/slam.yaml`: SLAM Toolbox parameters for online asynchronous mapping.
* `config/amcl.yaml`: AMCL particle filter localization parameters.
* `maps/map.pgm` & `maps/map.yaml`: Pre-built 2D occupancy grid map of the campus road loop.
