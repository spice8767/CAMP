"""
Coordinate bridge between campus_nav planner frame and Gazebo world frame.

Both use the same metric scale (1 unit = 1 meter), but have a constant XY offset.
The offset was calibrated by matching the Gazebo spawn point (14.926, 40.592) to
the nearest road node in the campus_nav graph → node 14103608599 at (604.97, 900.70).

Transform:
    nav_x = gz_x + OFFSET_X
    nav_y = gz_y + OFFSET_Y
"""

import math

# Constant offset between coordinate frames (meters)
# Calibrated via ICP alignment between map1.obj road mesh and OSM road graph:
# Gazebo spawn (14.926, 40.592) maps to SAB C node 11752291183 at (816.9, 720.3)
OFFSET_X = 802.0
OFFSET_Y = 679.7


def gz_to_nav(gz_x, gz_y):
    """Convert Gazebo world coordinates to campus_nav planner coordinates."""
    return gz_x + OFFSET_X, gz_y + OFFSET_Y


def nav_to_gz(nav_x, nav_y):
    """Convert campus_nav planner coordinates to Gazebo world coordinates."""
    return nav_x - OFFSET_X, nav_y - OFFSET_Y


def convert_path_nav_to_gz(path_msg):
    """
    Convert a nav_msgs/Path from planner coords to Gazebo coords (in-place).
    Returns the same path_msg for convenience.
    """
    for pose in path_msg.poses:
        gx, gy = nav_to_gz(pose.pose.position.x, pose.pose.position.y)
        pose.pose.position.x = gx
        pose.pose.position.y = gy
    return path_msg


def convert_path_gz_to_nav(path_msg):
    """
    Convert a nav_msgs/Path from Gazebo coords to planner coords (in-place).
    Returns the same path_msg for convenience.
    """
    for pose in path_msg.poses:
        nx, ny = gz_to_nav(pose.pose.position.x, pose.pose.position.y)
        pose.pose.position.x = nx
        pose.pose.position.y = ny
    return path_msg


def yaw_from_quaternion(q):
    """Extract yaw angle from a quaternion (geometry_msgs Quaternion or similar)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """Return (x, y, z, w) quaternion tuple from a yaw angle."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
