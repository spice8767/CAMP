"""
Full Simulation Launch — Phase 4

One command to start the complete autonomous campus navigation stack:
  - Gazebo Harmonic with campus world
  - saye buggy robot spawned on campus
  - ros_gz_bridge (cmd_vel, tf, clock)
  - campus_nav planner (A* + trajectory + markers + RViz)
  - odom_bridge_node (Gazebo pose → planner coords)
  - path_follower_node (Pure Pursuit controller → /cmd_vel)
  - mission_controller_node (orchestrates teleport + plan + follow)

Usage:
  ros2 launch campus_nav full_simulation.launch.py

Then in a new terminal, publish a mission:
  ros2 topic pub /campus/mission std_msgs/msg/String \\
    "data: 'Ravi Back|Chemistry Block'" --once
"""

import os
import subprocess
import tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import join


def generate_launch_description():
    # ── Pre-launch cleanup: terminate any lingering Gazebo processes ───────────
    # Prevents multiple Gazebo GUI windows from opening.
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    subprocess.run(['killall', '-9', 'gz-sim-server', 'gz-sim-gui'], capture_output=True)

    # ── Package paths ──────────────────────────────────────────────────────────
    pkg_campus_nav    = get_package_share_directory('campus_nav')
    pkg_saye          = get_package_share_directory('saye_description')
    pkg_saye_bringup  = get_package_share_directory('saye_bringup')
    pkg_ros_gz_sim    = get_package_share_directory('ros_gz_sim')

    # ── File paths ─────────────────────────────────────────────────────────────
    saye_xacro        = os.path.join(pkg_saye, 'models', 'saye', 'model.xacro')
    bridge_config     = os.path.join(pkg_saye_bringup, 'config', 'ros_gz_bridge.yaml')
    campus_rviz       = os.path.join(pkg_campus_nav, 'rviz', 'campus_nav.rviz')
    livox_rviz        = os.path.join(pkg_campus_nav, 'rviz', 'livox_mid360.rviz')

    # ── Set GZ_SIM_RESOURCE_PATH at Python level so ALL subprocesses inherit it.
    #    SetEnvironmentVariable() launch action is unreliable for IncludeLaunchDescription.
    gz_resource_path = os.path.dirname(pkg_saye)  # parent of saye_description share dir
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    # ── Patch saye_world.sdf at launch time: replace model:// URIs with absolute
    #    file:// URIs so Gazebo never attempts a Fuel network download.
    #    This also avoids the space-in-path issue with 'CAMP A'.
    map_obj_abs = os.path.join(pkg_saye, 'worlds', 'map1.obj')
    sdf_template_path = os.path.join(pkg_saye, 'worlds', 'saye_world.sdf')
    with open(sdf_template_path, 'r') as f:
        sdf_content = f.read()
    sdf_content = sdf_content.replace(
        'model://saye_description/worlds/map1.obj',
        'file://' + map_obj_abs
    )
    # Write patched SDF to a /tmp path (no spaces, survives launch session)
    tmp_sdf = tempfile.NamedTemporaryFile(
        mode='w', suffix='_saye_world.sdf', delete=False)
    tmp_sdf.write(sdf_content)
    tmp_sdf.flush()
    tmp_sdf.close()
    campus_sdf = tmp_sdf.name

    # Updated OSM — prefer workspace root copy if present
    default_osm = next((p for p in ['/home/yash/CAMP/campus_with_junctions_and_stops.osm',
                                    '/home/yash/CAMP A/campus_with_junctions_and_stops.osm']
                        if os.path.exists(p)), os.path.join(pkg_campus_nav, 'data', 'campus.osm'))

    stops_yaml = os.path.join(pkg_campus_nav, 'config', 'campus_junctions_and_stops.yaml')

    # ── Robot description ──────────────────────────────────────────────────────
    robot_desc = xacro.process_file(saye_xacro).toxml()

    # ── Launch arguments ───────────────────────────────────────────────────────
    declare_start = DeclareLaunchArgument(
        'start_stop', default_value='',
        description='Optional: auto-start mission from this stop'
    )
    declare_goal = DeclareLaunchArgument(
        'goal_stop', default_value='',
        description='Optional: auto-start mission to this stop'
    )
    declare_speed = DeclareLaunchArgument(
        'nominal_speed', default_value='2.22',
        description='Nominal cruising speed in m/s (default 2.22 m/s = 8 km/h)'
    )

    # ── NVIDIA GPU + Gazebo resource path environment ──────────────────────────
    env_vars = [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only'),
        SetEnvironmentVariable('__GL_SYNC_TO_VBLANK', '0'),
        SetEnvironmentVariable('vblank_mode', '0'),
        # Belt-and-suspenders: also set via launch action
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
    ]

    # ── Gazebo simulation ──────────────────────────────────────────────────────
    # campus_sdf is a /tmp/<hash>_saye_world.sdf with no spaces — safe to pass
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 4 ' + campus_sdf}.items()
    )

    # ── Robot state publisher (URDF → /tf static) ──────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # ── Spawn saye in Gazebo after 5s world-load delay ─────────────────────────
    spawn_saye = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_saye',
            arguments=[
                '-string', robot_desc,
                '-name',   'saye',
                '-world',  'campus_world',
                '-allow_renaming', 'false',
                '-x', '15.02',
                '-y', '43.00',
                '-z', '0.80',
                '-Y', '3.14159'
            ],
            output='screen'
        )]
    )

    # ── ROS ↔ Gazebo bridge ────────────────────────────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # ── Campus Nav Planner (A* + RViz markers) ─────────────────────────────────
    planner_node = Node(
        package='campus_nav',
        executable='planner_node',
        name='campus_planner_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'osm_file': default_osm,
            'stops_file': stops_yaml,
            'default_start_stop': 'SAB C',
            'default_goal_stop': 'Chemistry Block',
            'nominal_speed': LaunchConfiguration('nominal_speed'),
            'frame_id': 'map',
            'use_sim_time': True
        }]
    )

    # ── Window 2: RViz Campus Map & RealSense Camera Feed (delayed) ───────────
    rviz_map_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_campus_map',
        arguments=['-d', campus_rviz],
        parameters=[{'use_sim_time': True}],
        output='screen',
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            '__GL_SYNC_TO_VBLANK': '0',
            'vblank_mode': '0'
        }
    )

    # ── Window 3: RViz Livox Mid-360 LiDAR Cloud Inspection (delayed) ─────────
    rviz_lidar_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_livox_lidar',
        arguments=['-d', livox_rviz],
        parameters=[{'use_sim_time': True}],
        output='screen',
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            '__GL_SYNC_TO_VBLANK': '0',
            'vblank_mode': '0'
        }
    )

    # ── Odometry Bridge (Gazebo TF → campus_nav pose) ─────────────────────────
    odom_bridge_node = Node(
        package='campus_nav',
        executable='odom_bridge_node',
        name='odom_bridge_node',
        output='screen',
        parameters=[{'model_name': 'saye', 'use_sim_time': True}]
    )

    # ── [COMMENTED OUT] Pure Pursuit Path Follower ─────────────────────────────
    # path_follower_node = Node(
    #     package='campus_nav',
    #     executable='path_follower_node',
    #     ...
    # )

    # ── [COMMENTED OUT] Safety Monitor Node ────────────────────────────────────
    # safety_monitor_node = Node(
    #     package='campus_nav',
    #     executable='safety_monitor_node',
    #     ...
    # )

    # ── Nav2 Local Controller & Costmap (Replaces Follower & Safety) ───────────
    nav2_params_file = join(pkg_campus_nav, 'config', 'nav2_params.yaml')

    nav2_adapter_node = Node(
        package='campus_nav',
        executable='nav2_adapter_node',
        name='nav2_adapter_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    nav2_controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('/cmd_vel', '/cmd_vel')] # Ensure we output to Gazebo's /cmd_vel
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['controller_server']}
        ]
    )

    # ── Mission Controller ─────────────────────────────────────────────────────
    mission_controller_node = Node(
        package='campus_nav',
        executable='mission_controller_node',
        name='mission_controller_node',
        output='screen',
        parameters=[{'osm_file': default_osm, 'use_sim_time': True}]
    )

    # ── Audit Logger (Safety-critical stack flight recorder) ───────────────────
    audit_logger_node = Node(
        package='campus_nav',
        executable='audit_logger',
        name='audit_logger',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ── Assemble launch description ────────────────────────────────────────────
    return LaunchDescription([
        # Environment variables first
        *env_vars,

        # Launch arguments
        declare_start,
        declare_goal,
        declare_speed,

        # Simulation stack
        gazebo,
        robot_state_publisher,
        spawn_saye,
        ros_gz_bridge,

        # Navigation stack (slight delay to ensure Gazebo is up)
        TimerAction(period=2.0, actions=[planner_node]),
        TimerAction(period=2.0, actions=[odom_bridge_node]),
        # TimerAction(period=2.0, actions=[safety_monitor_node]),
        # TimerAction(period=2.0, actions=[path_follower_node]),
        
        # Launch Nav2 Controller and Lifecycle Manager
        TimerAction(period=2.0, actions=[nav2_adapter_node]),
        TimerAction(period=2.0, actions=[nav2_controller_node]),
        TimerAction(period=2.5, actions=[nav2_lifecycle_manager]),

        TimerAction(period=2.0, actions=[mission_controller_node]),
        TimerAction(period=2.0, actions=[audit_logger_node]),
        TimerAction(period=3.0, actions=[rviz_map_node]),
        TimerAction(period=3.5, actions=[rviz_lidar_node]),
        TimerAction(period=4.0, actions=[Node(
            package='campus_nav',
            executable='arrange_windows',
            name='arrange_windows',
            output='screen'
        )]),
    ])
