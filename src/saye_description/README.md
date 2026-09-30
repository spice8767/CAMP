# saye_description — Buggy URDF, Meshes & Simulation Model

`saye_description` defines the physical and visual model of the `saye` electric campus buggy for ROS 2 and Gazebo Harmonic (`gz-sim`). It contains the URDF/Xacro descriptions, 3D meshes, sensor configurations (3D LiDAR, RealSense depth camera, IMU), and Gazebo simulation world environments.

---

## Vehicle Kinematics & Specifications

The vehicle is modeled after a standard campus electric passenger buggy with rear-wheel drive and front Ackermann steering geometry:

* **Wheelbase:** $2.124\,\text{m}$ (distance between front knuckle steer axis and rear drive axle).
* **Kingpin Width:** $0.755\,\text{m}$ (lateral spacing between front steering kingpins).
* **Wheel Radius:** $0.277\,\text{m}$.
* **Steering Limit:** $\pm 0.8\,\text{rad}$ ($\approx 45.8^\circ$).
* **Velocity Range:** $-15.0\,\text{m/s}$ to $+15.0\,\text{m/s}$ (speed governed to $2.22\,\text{m/s} = 8.0\,\text{km/h}$ by `campus_nav`).

---

## ⚠️ Coordinate System & Forward Vector

In the CAD model of `saye`:
* **Buggy Front:** Along the **$-Y$** axis of `base_link` (front axle at $y = -0.40\,\text{m}$).
* **Buggy Rear:** Along the **$+Y$** axis of `base_link` (rear axle at $y = +1.72\,\text{m}$).
* **Buggy Left:** Along the **$-X$** axis of `base_link`.
* **Buggy Right:** Along the **$+X$** axis of `base_link`.
* **Buggy Up:** Along the **$+Z$** axis of `base_link`.

### Sensor Frame Alignment
Because ROS robotics conventions expect $+X$ to be the forward sensor axis:
* **LiDAR Joint (`lidar_link`):** Mounted at $(x=0, y=0, z=1.65\,\text{m})$ with rotation `rpy="0 0 -1.5707963"`. Its $+X$ axis points directly forward along the buggy's driving heading ($-Y$ of `base_link`).
* **Depth Camera Joint (`depth_camera_link`):** Mounted at $(x=0, y=-0.55, z=1.0\,\text{m})$ with rotation `rpy="0 0 -1.5707963"`. Its $+X$ optical axis points forward along the driving direction.

### World Spawn Heading
In Gazebo's world coordinate frame ($+Y$ is North):
* Spawning with **`yaw = 3.14159` ($\pi$)** rotates $-Y$ of `base_link` North along $+Y_{gz}$, aligning the vehicle facing forward down the campus road towards SAB S.
* Spawning with `yaw = 0.0` points the vehicle South.

---

## Sensor Specifications

### 1. 3D LiDAR (`gpu_lidar`)
* **Link:** `lidar_link`
* **Gazebo Topic:** `/scan/points` (bridged to ROS `/cloud`)
* **Horizontal Resolution:** 1024 samples ($360^\circ$ azimuth coverage)
* **Vertical Channels:** 128 rings ($\pm 30^\circ$ vertical elevation aperture: $-0.52356\,\text{rad}$ to $+0.52356\,\text{rad}$)
* **Range:** $0.3\,\text{m}$ to $50.0\,\text{m}$ ($0.01\,\text{m}$ resolution)
* **Update Rate:** 10 Hz

### 2. RealSense RGB-D Camera (`rs_front`)
* **Link:** `depth_camera_link`
* **Gazebo Topic:** `/rs_front/image` & `/rs_front/depth_image` (bridged to `/real_sense/image_raw` and `/real_sense/depth_image`)
* **Resolution:** $640 \times 360$ px
* **Horizontal FOV:** $86^\circ$ ($1.501\,\text{rad}$)
* **Update Rate:** 10 Hz

### 3. Inertial Measurement Unit (`imu_sensor`)
* **Link:** `base_link`
* **Gazebo Topic:** `/imu` (bridged to ROS `/imu`)
* **Update Rate:** 250 Hz with Gaussian noise modeling for angular velocity and linear acceleration.

---

## Gazebo Harmonic Plugins

The model uses native Gazebo Harmonic systems:
1. `gz::sim::systems::AckermannSteering`:
   * Drives rear wheels `Revolute_20` and `Revolute_21`.
   * Steers front knuckles `Revolute_22` and `Revolute_23`.
   * Subscribes to `/cmd_vel` (`geometry_msgs/Twist`).
2. `gz::sim::systems::OdometryPublisher`:
   * Ground truth world odometry published to `/model/saye/odometry_world`.
   * Fixed frame `world`, child frame `base_link`.
3. `gz::sim::systems::Sensors`:
   * Handles GPU LiDAR rendering, camera projection, and IMU data generation.

---

## World Environment (`worlds/saye_world.sdf`)

* Loads `worlds/map1.obj` (the 3D elevated road mesh of the campus).
* Physics engine: DART (`gz-physics-dartsim-plugin`).
* Lighting: Sunlight directional light with realistic shadows and soft ambient fill.
