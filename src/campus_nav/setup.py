import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'campus_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'data'), glob('data/*.osm')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adarsh4our',
    maintainer_email='as0654224@gmail.com',
    description='Autonomous Campus Road Navigation and A* Path Planning for Golf Cart',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node             = campus_nav.planner_node:main',
            'odom_bridge_node         = campus_nav.odom_bridge_node:main',
            'path_follower_node       = campus_nav.path_follower_node:main',
            'safety_monitor_node      = campus_nav.safety_monitor_node:main',
            'mission_controller_node  = campus_nav.mission_controller_node:main',
            'audit_logger             = campus_nav.audit_logger_node:main',
            'teleport_buggy           = campus_nav.teleport_buggy:main',
            'orient_buggy             = campus_nav.teleport_buggy:orient_main',
            'wasd_teleop              = campus_nav.wasd_teleop:main',
            'test_obstacle_spawner    = campus_nav.test_obstacle_spawner:main',
        ],
    },
)
