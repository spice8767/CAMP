#!/usr/bin/env python3
"""
CLI helper to teleport and orient the buggy in Gazebo simulation:
  - Orient parallel to the nearest road lane
  - Rotate 180 degrees (turn around) or arbitrary angles
  - Teleport to named campus stops or XY coordinates with automatic road alignment

Usage:
  # Orient buggy parallel to the current road:
  ros2 run campus_nav orient_buggy

  # Rotate buggy 180 degrees (turn around):
  ros2 run campus_nav orient_buggy --flip
  ros2 run campus_nav orient_buggy --rotate 180

  # Teleport to named stop (automatically aligned to road):
  ros2 run campus_nav teleport_buggy --stop "LOC"
  ros2 run campus_nav teleport_buggy --stop "SAB C" --flip
"""

import os
import sys
import argparse
import subprocess
import math
import re
from campus_nav.osm_loader import CampusMap
from campus_nav.coord_bridge import nav_to_gz, gz_to_nav


ROAD_SURFACE_Z = 0.60  # Road surface is at Z=0.30m + wheel radius 0.28m
DEFAULT_OSM = next((p for p in ['/home/adarsh4our/CAMP/campus_with_junctions_and_stops.osm',
                                '/home/adarsh4our/CAMP A/campus_with_junctions_and_stops.osm']
                    if os.path.exists(p)), '/home/adarsh4our/CAMP/campus_with_junctions_and_stops.osm')


def get_current_gz_pose(model='saye'):
    """Query current model position and yaw directly from Gazebo Harmonic."""
    res = subprocess.run(['gz', 'model', '-m', model, '-p'], capture_output=True, text=True)
    m = re.search(r'\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]\s*\n\s*\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]', res.stdout)
    if m:
        x, y, z = float(m.group(1)), float(m.group(2)), float(m.group(3))
        r, p, yaw = float(m.group(4)), float(m.group(5)), float(m.group(6))
        return x, y, z, yaw
    return None


def find_parallel_road_yaw(gz_x, gz_y, current_yaw=3.14159, osm_path=DEFAULT_OSM):
    """
    Find the tangent heading of the nearest road segment to (gz_x, gz_y),
    and return the Gazebo yaw that aligns the buggy parallel to that road.
    Selects the direction closest to current_yaw so the vehicle doesn't reverse unexpectedly.
    """
    c = CampusMap(osm_path)
    nav_x, nav_y = gz_to_nav(gz_x, gz_y)

    candidate_segments = []
    for way in c.road_ways:
        for i in range(len(way) - 1):
            u, v = way[i], way[i + 1]
            x1, y1 = c.xy_nodes[u]
            x2, y2 = c.xy_nodes[v]
            dx = x2 - x1
            dy = y2 - y1
            L2 = dx * dx + dy * dy
            if L2 == 0:
                continue
            t = max(0.0, min(1.0, ((nav_x - x1) * dx + (nav_y - y1) * dy) / L2))
            px = x1 + t * dx
            py = y1 + t * dy
            d = math.hypot(nav_x - px, nav_y - py)
            if d < 20.0:
                candidate_segments.append((d, (x1, y1), (x2, y2)))

    if not candidate_segments:
        # Fallback to closest of all segments if far from road
        for way in c.road_ways:
            for i in range(len(way) - 1):
                u, v = way[i], way[i + 1]
                x1, y1 = c.xy_nodes[u]
                x2, y2 = c.xy_nodes[v]
                dx = x2 - x1
                dy = y2 - y1
                L2 = dx * dx + dy * dy
                if L2 == 0:
                    continue
                t = max(0.0, min(1.0, ((nav_x - x1) * dx + (nav_y - y1) * dy) / L2))
                px = x1 + t * dx
                py = y1 + t * dy
                candidate_segments.append((math.hypot(nav_x - px, nav_y - py), (x1, y1), (x2, y2)))

    candidate_segments.sort(key=lambda s: s[0])

    def angle_diff(a, b):
        return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)

    best_diff = float('inf')
    best_yaw = current_yaw
    best_road_deg = 0.0

    # Consider the nearest road segments
    for d, (x1, y1), (x2, y2) in candidate_segments[:4]:
        road_angle = math.atan2(y2 - y1, x2 - x1)
        for direction in (road_angle, road_angle + math.pi):
            # Because front is along -Y of base_link:
            # yaw_gz = direction + pi/2
            cand_yaw = (direction + math.pi / 2.0 + math.pi) % (2.0 * math.pi) - math.pi
            diff = angle_diff(cand_yaw, current_yaw)
            if diff < best_diff:
                best_diff = diff
                best_yaw = cand_yaw
                best_road_deg = math.degrees(direction)

    return best_yaw, best_road_deg, candidate_segments[0][0]


def teleport_gz(gz_x, gz_y, gz_z=ROAD_SURFACE_Z, yaw=3.14159, world='campus_world', model='saye'):
    """Send set_pose request to Gazebo simulation service and zero out velocities."""
    # 1. Stop any momentum immediately
    subprocess.run(
        ['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist', '{}'],
        capture_output=True
    )

    # 2. Compute orientation quaternion
    sin_h = math.sin(yaw / 2.0)
    cos_h = math.cos(yaw / 2.0)
    req = (
        f'name: "{model}", '
        f'position: {{x: {gz_x:.3f}, y: {gz_y:.3f}, z: {gz_z:.3f}}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: {sin_h:.6f}, w: {cos_h:.6f}}}'
    )
    cmd = [
        'gz', 'service',
        '-s', f'/world/{world}/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', req
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode == 0 and 'data: true' in res.stdout:
        print(f"✅ Successfully positioned {model} in Gazebo ({gz_x:.2f}, {gz_y:.2f}, Z={gz_z:.2f}, Yaw={math.degrees(yaw):.1f}°)")
        return True
    else:
        print(f"⚠️ Teleport result: {res.stdout.strip() or res.stderr.strip()}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Teleport and orient buggy in Gazebo simulation.")
    parser.add_argument('--stop', type=str, help="Name of campus stop (e.g. 'LOC', 'Ravi Back', 'SAB C')")
    parser.add_argument('--x', type=float, help="Gazebo X coordinate")
    parser.add_argument('--y', type=float, help="Gazebo Y coordinate")
    parser.add_argument('--z', type=float, default=None, help=f"Gazebo Z height (default {ROAD_SURFACE_Z}m)")
    parser.add_argument('--yaw', type=float, default=None, help="Heading in radians")
    parser.add_argument('--align-road', action='store_true', help="Orient buggy parallel to the nearest road segment")
    parser.add_argument('--rotate-180', '--flip', dest='flip_180', action='store_true', help="Rotate buggy 180 degrees (turn around)")
    parser.add_argument('--rotate', type=float, default=0.0, help="Relative rotation angle in degrees (e.g. 180, 90, -90)")
    args, _ = parser.parse_known_args()

    # Query current position if available
    current_pose = get_current_gz_pose()
    curr_x = current_pose[0] if current_pose else 15.02
    curr_y = current_pose[1] if current_pose else 43.00
    curr_z = current_pose[2] if current_pose else ROAD_SURFACE_Z
    curr_yaw = current_pose[3] if current_pose else 3.14159

    target_x = curr_x
    target_y = curr_y
    target_z = args.z if args.z is not None else (curr_z if current_pose else ROAD_SURFACE_Z)
    target_yaw = args.yaw if args.yaw is not None else curr_yaw

    if args.stop:
        c = CampusMap(DEFAULT_OSM)
        nid = c.get_stop_road_node(args.stop)
        if not nid:
            print(f"❌ Unknown stop: '{args.stop}'. Available: {sorted(c.stops.keys())}")
            sys.exit(1)
        nav_x, nav_y = c.xy_nodes[nid]
        target_x, target_y = nav_to_gz(nav_x, nav_y)
        target_z = args.z if args.z is not None else ROAD_SURFACE_Z
        print(f"Stop '{args.stop}' at Nav ({nav_x:.1f}, {nav_y:.1f}) → Gazebo ({target_x:.1f}, {target_y:.1f})")
        # Automatically snap parallel to road at stop unless explicit yaw specified
        if args.yaw is None:
            target_yaw, road_deg, dist = find_parallel_road_yaw(target_x, target_y, curr_yaw)
            print(f"  Snapping parallel to road ({road_deg:.1f}°) -> Yaw: {math.degrees(target_yaw):.1f}°")

    elif args.x is not None and args.y is not None:
        target_x = args.x
        target_y = args.y
        if args.z is not None:
            target_z = args.z

    # If --align-road requested
    if args.align_road:
        target_yaw, road_deg, dist = find_parallel_road_yaw(target_x, target_y, target_yaw)
        print(f"🚗 Road alignment: Nearest segment is {dist:.2f}m away (heading {road_deg:.1f}°)")
        print(f"   Orienting buggy parallel to road -> Yaw: {math.degrees(target_yaw):.1f}° ({target_yaw:.4f} rad)")

    # If rotate 180 degrees requested
    if args.flip_180:
        target_yaw = (target_yaw + math.pi + math.pi) % (2.0 * math.pi) - math.pi
        print(f"🔄 Rotated 180° -> New Yaw: {math.degrees(target_yaw):.1f}° ({target_yaw:.4f} rad)")

    # If relative rotation requested
    if args.rotate != 0.0:
        rot_rad = math.radians(args.rotate)
        target_yaw = (target_yaw + rot_rad + math.pi) % (2.0 * math.pi) - math.pi
        print(f"🔄 Rotated {args.rotate}° -> New Yaw: {math.degrees(target_yaw):.1f}° ({target_yaw:.4f} rad)")

    teleport_gz(target_x, target_y, target_z, target_yaw)


def orient_main():
    """Dedicated quick command: ros2 run campus_nav orient_buggy [--flip] [--rotate DEG]."""
    parser = argparse.ArgumentParser(description="Quickly orient buggy parallel to road or rotate 180 degrees.")
    parser.add_argument('--flip', '--rotate-180', dest='flip_180', action='store_true', help="Rotate buggy 180 degrees (turn around on road)")
    parser.add_argument('--rotate', type=float, default=0.0, help="Relative rotation angle in degrees (e.g. 180, 90, -90)")
    args, _ = parser.parse_known_args()

    current_pose = get_current_gz_pose()
    if not current_pose:
        print("❌ Could not connect to Gazebo or query 'saye' pose. Is the simulation running?")
        sys.exit(1)

    curr_x, curr_y, curr_z, curr_yaw = current_pose

    # Default action: snap parallel to the nearest road!
    new_yaw, road_deg, dist = find_parallel_road_yaw(curr_x, curr_y, curr_yaw)
    print(f"🚗 Road alignment: Nearest road segment {dist:.2f}m away (road heading {road_deg:.1f}°)")

    if args.flip_180 or args.rotate == 180.0:
        new_yaw = (new_yaw + math.pi + math.pi) % (2.0 * math.pi) - math.pi
        print(f"🔄 Applying 180° U-turn parallel to road -> Yaw: {math.degrees(new_yaw):.1f}° ({new_yaw:.4f} rad)")
    elif args.rotate != 0.0:
        rot_rad = math.radians(args.rotate)
        new_yaw = (new_yaw + rot_rad + math.pi) % (2.0 * math.pi) - math.pi
        print(f"🔄 Applying {args.rotate}° rotation -> Yaw: {math.degrees(new_yaw):.1f}° ({new_yaw:.4f} rad)")
    else:
        print(f"✅ Snapped parallel to road lane -> Yaw: {math.degrees(new_yaw):.1f}° ({new_yaw:.4f} rad)")

    teleport_gz(curr_x, curr_y, curr_z, new_yaw)


if __name__ == '__main__':
    main()
