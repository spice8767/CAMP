"""
Test Obstacle Spawner for Gazebo Harmonic

Spawns test obstacles in front of the saye buggy or at specific coordinates:
  - human: Standing pedestrian model (height 1.7m, radius 0.25m)
  - dog: Small animal model (height 0.45m, length 0.6m)
  - speed_breaker: Realistic road bump (height 0.08m, width 3.6m, length 0.5m)

Usage:
  ros2 run campus_nav test_obstacle_spawner --type human --distance 6.0
  ros2 run campus_nav test_obstacle_spawner --type speed_breaker --distance 8.0
  ros2 run campus_nav test_obstacle_spawner --type dog --distance 5.0
  ros2 run campus_nav test_obstacle_spawner --remove
"""

import argparse
import subprocess
import sys


SDF_HUMAN = """<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='test_obstacle_human'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>0.25</radius>
            <length>1.7</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>0.25</radius>
            <length>1.7</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.1 0.1 1.0</ambient>
          <diffuse>0.9 0.1 0.1 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

SDF_DOG = """<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='test_obstacle_dog'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.6 0.35 0.45</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.6 0.35 0.45</size>
          </box>
        </geometry>
        <material>
          <ambient>0.6 0.35 0.1 1.0</ambient>
          <diffuse>0.7 0.4 0.15 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

SDF_BUMP = """<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='test_obstacle_bump'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box>
            <size>3.6 0.5 0.08</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>3.6 0.5 0.08</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.85 0.0 1.0</ambient>
          <diffuse>1.0 0.85 0.0 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def get_buggy_pose():
    """Query Gazebo for saye buggy pose."""
    # Default to SAB C spawn coordinates facing North (yaw = 1.5708)
    return (15.02, 43.00, 0.30, 1.5708)


def spawn_obstacle(model_name: str, sdf_str: str, x: float, y: float, z: float, yaw: float = 0.0):
    """Call Gazebo Harmonic EntityFactory service."""
    # Flatten SDF to single line with escaped quotes
    sdf_escaped = sdf_str.replace('"', '\\"').replace('\n', ' ')
    req_str = (
        f'sdf: "{sdf_escaped}", '
        f'pose: {{ position: {{ x: {x:.2f}, y: {y:.2f}, z: {z:.2f} }} }}'
    )
    cmd = [
        'gz', 'service', '-s', '/world/campus_world/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', req_str
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if "data: true" in res.stdout or res.returncode == 0:
        print(f"✅ Successfully spawned '{model_name}' at Gazebo ({x:.2f}, {y:.2f}, {z:.2f})")
    else:
        print(f"❌ Failed to spawn '{model_name}': {res.stderr or res.stdout}")


def remove_obstacle(model_name: str):
    """Remove entity from Gazebo."""
    req_str = f'name: "{model_name}", type: 2'
    cmd = [
        'gz', 'service', '-s', '/world/campus_world/remove',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req_str
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if "data: true" in res.stdout or res.returncode == 0:
        print(f"🧹 Removed '{model_name}'")
    else:
        print(f"⚠️ Could not remove '{model_name}' (may not exist)")


def main():
    parser = argparse.ArgumentParser(description="Test Obstacle Spawner")
    parser.add_argument('--type', choices=['human', 'dog', 'speed_breaker'],
                        default='human', help="Obstacle type to spawn")
    parser.add_argument('--distance', type=float, default=6.0,
                        help="Distance ahead of buggy in meters (default 6.0m)")
    parser.add_argument('--x', type=float, default=None, help="Explicit Gazebo X")
    parser.add_argument('--y', type=float, default=None, help="Explicit Gazebo Y")
    parser.add_argument('--z', type=float, default=None, help="Explicit Gazebo Z")
    parser.add_argument('--remove', action='store_true',
                        help="Remove all test obstacles from simulation")
    args = parser.parse_args()

    models = ['test_obstacle_human', 'test_obstacle_dog', 'test_obstacle_bump']

    if args.remove:
        for m in models:
            remove_obstacle(m)
        return

    # Determine spawn location
    bx, by, bz, byaw = get_buggy_pose()
    if args.x is not None and args.y is not None:
        target_x, target_y = args.x, args.y
        target_z = args.z if args.z is not None else 0.30
    else:
        # Default ahead of buggy along heading (at SAB C heading is +Y)
        target_x = bx
        target_y = by + args.distance
        target_z = 0.30

    if args.type == 'human':
        spawn_obstacle('test_obstacle_human', SDF_HUMAN, target_x, target_y, target_z + 0.85)
    elif args.type == 'dog':
        spawn_obstacle('test_obstacle_dog', SDF_DOG, target_x, target_y, target_z + 0.22)
    elif args.type == 'speed_breaker':
        spawn_obstacle('test_obstacle_bump', SDF_BUMP, target_x, target_y, target_z + 0.04)


if __name__ == '__main__':
    main()
