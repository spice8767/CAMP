# CART_description — Baseline Golf Cart Description (Legacy)

`CART_description` contains the initial baseline model of the campus electric cart.

> **Note:** For the primary active autonomous simulation with 3D LiDAR, RealSense depth camera, and the `campus_nav` autonomy stack, use [`saye_description`](../saye_description/) and [`campus_nav`](../campus_nav/).

---

## Contents
* `urdf/CART.xacro`: Baseline URDF model with Ackermann steering joints.
* `launch/gazebo.launch.py`: Baseline single-model Gazebo spawn launch.
* `meshes/`: 3D STL collision and visual meshes for chassis, wheels, knuckles, and canopy.
* `worlds/campus.sdf`: Baseline SDF world.
