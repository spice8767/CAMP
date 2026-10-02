import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('campus_nav')

    default_rviz = os.path.join(pkg_share, 'rviz', 'campus_nav.rviz')
    candidates_osm = [
        os.path.join(pkg_share, 'data', 'campus_with_junctions_and_stops.osm'),
        os.path.join(pkg_share, 'data', 'campus.osm'),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'campus_with_junctions_and_stops.osm'),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'campus_with_junctions_and_stops.osm')
    ]
    default_osm = next((p for p in candidates_osm if os.path.exists(p)), os.path.join(pkg_share, 'data', 'campus.osm'))
    default_yaml = os.path.join(pkg_share, 'config', 'campus_junctions_and_stops.yaml')

    declare_start = DeclareLaunchArgument(
        'start_stop',
        default_value='Ravi Back',
        description='Default starting stop for initial route'
    )

    declare_goal = DeclareLaunchArgument(
        'goal_stop',
        default_value='Chemistry Block',
        description='Default destination stop for initial route'
    )

    declare_speed = DeclareLaunchArgument(
        'nominal_speed',
        default_value='2.22',
        description='Nominal cruising speed in m/s (default 2.22 m/s = 8 km/h)'
    )

    planner_node = Node(
        package='campus_nav',
        executable='planner_node',
        name='campus_planner_node',
        output='screen',
        parameters=[{
            'osm_file': default_osm,
            'stops_file': default_yaml,
            'default_start_stop': LaunchConfiguration('start_stop'),
            'default_goal_stop': LaunchConfiguration('goal_stop'),
            'nominal_speed': LaunchConfiguration('nominal_speed'),
            'frame_id': 'map'
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        output='screen',
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            '__VK_LAYER_NV_optimus': 'NVIDIA_only',
            '__GL_SYNC_TO_VBLANK': '0',
            'vblank_mode': '0'
        }
    )

    return LaunchDescription([
        # Force rendering and compute on NVIDIA RTX 4060 uncapped to 60+ FPS
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only'),
        SetEnvironmentVariable('__GL_SYNC_TO_VBLANK', '0'),
        SetEnvironmentVariable('vblank_mode', '0'),
        declare_start,
        declare_goal,
        declare_speed,
        planner_node,
        rviz_node
    ])
