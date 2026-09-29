"""
Trajectory smoothing and speed profiling for Ackermann golf cart.
Uses localized Bézier corner fillets to keep straight road segments strictly on-track,
while smoothly rounding intersection corners for vehicle steering feasibility.
"""

import math
import numpy as np


def generate_smooth_trajectory(waypoints, step_size=0.5, corner_radius=3.5, min_turning_radius=2.5, v_max=2.22, a_lat_max=1.2):
    """
    Smooths road waypoints while strictly staying on straight road segments.
    Applies localized Bézier corner rounding at intersection bends.
    
    Args:
        waypoints: list of (x, y) tuples from A*
        step_size: distance between consecutive trajectory points in meters
        corner_radius: maximum corner rounding radius in meters
        v_max: maximum straight-line cruising speed in m/s
        a_lat_max: maximum lateral acceleration for passenger comfort in m/s^2
        
    Returns:
        trajectory: list of dicts {'x': float, 'y': float, 'yaw': float, 'v': float, 'curvature': float}
    """
    if len(waypoints) < 2:
        return []

    # 1. Clean consecutive duplicate or ultra-close points (< 0.1m)
    pts = [waypoints[0]]
    for p in waypoints[1:]:
        if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 0.1:
            pts.append(p)

    if len(pts) < 2:
        return []

    # 2. Build piecewise segments with localized corner fillets
    refined_points = []
    corner_curvatures = {}  # index -> curvature

    for i in range(len(pts)):
        if i == 0 or i == len(pts) - 1:
            refined_points.append(pts[i])
            continue

        p_prev = pts[i - 1]
        p_curr = pts[i]
        p_next = pts[i + 1]

        d_in = math.hypot(p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
        d_out = math.hypot(p_next[0] - p_curr[0], p_next[1] - p_curr[1])

        # Corner radius bounded by half of adjacent segment lengths
        r = min(corner_radius, d_in * 0.45, d_out * 0.45)

        v_in = ((p_curr[0] - p_prev[0]) / d_in, (p_curr[1] - p_prev[1]) / d_in)
        v_out = ((p_next[0] - p_curr[0]) / d_out, (p_next[1] - p_curr[1]) / d_out)

        dot = max(-1.0, min(1.0, v_in[0] * v_out[0] + v_in[1] * v_out[1]))

        # If heading change is negligible or segments too short, keep original point
        if dot > 0.996 or r < 0.6:
            refined_points.append(p_curr)
        else:
            # Round corner with quadratic Bézier
            p0 = (p_curr[0] - v_in[0] * r, p_curr[1] - v_in[1] * r)
            p1 = p_curr
            p2 = (p_curr[0] + v_out[0] * r, p_curr[1] + v_out[1] * r)

            turn_angle = math.acos(dot)
            curvature = turn_angle / max(0.5, 2.0 * r)

            num_arc_steps = max(4, int(2.5 * r / step_size))
            for s in range(num_arc_steps + 1):
                t = s / num_arc_steps
                bx = (1.0 - t)**2 * p0[0] + 2.0 * (1.0 - t) * t * p1[0] + t**2 * p2[0]
                by = (1.0 - t)**2 * p0[1] + 2.0 * (1.0 - t) * t * p1[1] + t**2 * p2[1]
                idx = len(refined_points)
                refined_points.append((bx, by))
                corner_curvatures[idx] = curvature

    # 3. Resample at regular step_size intervals
    trajectory = []
    for i in range(len(refined_points) - 1):
        p1 = refined_points[i]
        p2 = refined_points[i + 1]
        seg_d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if seg_d < 1e-4:
            continue

        steps = max(1, int(seg_d / step_size))
        yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        curv = corner_curvatures.get(i, 0.0)

        # Speed profile: slow down at curves
        if curv > 0.05:
            speed = min(v_max, math.sqrt(a_lat_max / max(curv, 1e-3)))
        else:
            speed = v_max

        for s in range(steps):
            t = s / steps
            trajectory.append({
                'x': float(p1[0] + t * (p2[0] - p1[0])),
                'y': float(p1[1] + t * (p2[1] - p1[1])),
                'yaw': float(yaw),
                'v': float(speed),
                'curvature': float(curv)
            })

    if refined_points:
        trajectory.append({
            'x': float(refined_points[-1][0]),
            'y': float(refined_points[-1][1]),
            'yaw': float(trajectory[-1]['yaw']) if trajectory else 0.0,
            'v': 0.0,
            'curvature': 0.0
        })

    return trajectory
