import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('campus_nav')
    config_file = os.path.join(pkg_dir, 'config', 'fastlio_shadow.yaml')

    add_time = Node(
        package='campus_nav',
        executable='add_time',
        name='add_time',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[config_file, {'use_sim_time': True}]
    )

    odom_to_slam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_camera_init',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'camera_init',
                   '--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([add_time, fast_lio_node, odom_to_slam])
