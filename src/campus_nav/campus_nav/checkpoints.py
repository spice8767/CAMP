"""
Junction checkpoint analysis and turn-by-turn directions.
"""

import math


def analyze_route_checkpoints(node_path, xy_nodes, junctions_dict, nominal_speed=2.22):
    """
    Identifies all junctions traversed in node_path and computes turn-by-turn directions.
    
    Args:
        node_path: list of node IDs forming route
        xy_nodes: dict of node_id -> (x, y)
        junctions_dict: dict of node_id -> junction info
        nominal_speed: vehicle cruising speed for ETA in m/s
        
    Returns:
        checkpoints: list of dicts with junction details, turn directions, and distance
        directions_summary: list of formatted string instructions
    """
    if len(node_path) < 2:
        return [], ["Arrived at destination."]

    checkpoints = []
    directions = []
    cumulative_dist = 0.0

    for i in range(len(node_path)):
        nid = node_path[i]
        
        # Calculate distance from start
        if i > 0:
            prev_p = xy_nodes[node_path[i - 1]]
            curr_p = xy_nodes[nid]
            cumulative_dist += math.hypot(curr_p[0] - prev_p[0], curr_p[1] - prev_p[1])

        # If it's a junction (or start/end)
        is_junc = nid in junctions_dict
        is_start = (i == 0)
        is_end = (i == len(node_path) - 1)

        if is_junc or is_start or is_end:
            turn = "START"
            angle_deg = 0.0

            if is_end:
                turn = "ARRIVED"
            elif i > 0 and i < len(node_path) - 1:
                p_prev = xy_nodes[node_path[i - 1]]
                p_curr = xy_nodes[nid]
                p_next = xy_nodes[node_path[i + 1]]

                heading_in = math.atan2(p_curr[1] - p_prev[1], p_curr[0] - p_prev[0])
                heading_out = math.atan2(p_next[1] - p_curr[1], p_next[0] - p_curr[0])

                d_theta = heading_out - heading_in
                # Normalize to [-pi, pi]
                d_theta = math.atan2(math.sin(d_theta), math.cos(d_theta))
                angle_deg = math.degrees(d_theta)

                if angle_deg > 25.0:
                    turn = "Turn LEFT"
                elif angle_deg < -25.0:
                    turn = "Turn RIGHT"
                else:
                    turn = "Continue STRAIGHT"

            jtype = junctions_dict.get(nid, {}).get('type', 'Road Point')
            ckpt = {
                'node_id': nid,
                'index_in_path': i,
                'x': xy_nodes[nid][0],
                'y': xy_nodes[nid][1],
                'distance_from_start': round(cumulative_dist, 1),
                'eta_seconds': round(cumulative_dist / max(0.5, nominal_speed), 0),
                'junction_type': jtype,
                'turn': turn,
                'turn_angle_deg': round(angle_deg, 1)
            }
            checkpoints.append(ckpt)

            if is_start:
                directions.append(f"1. Depart from starting location at node {nid}.")
            elif is_end:
                directions.append(f"{len(directions)+1}. Arrive at destination ({cumulative_dist:.0f}m total).")
            else:
                directions.append(f"{len(directions)+1}. At {jtype} (Node {nid}): {turn} (after {cumulative_dist:.0f}m).")

    return checkpoints, directions


class CheckpointTracker:
    """
    Tracks a vehicle's live progress through pre-computed route checkpoints.

    Usage:
        # After planning a route:
        tracker = CheckpointTracker(checkpoints, arrival_radius=8.0)

        # Each time a vehicle position update arrives (e.g. from GPS/odometry):
        status_string = tracker.update(vehicle_x, vehicle_y)
        # → "Checkpoint 3/10 | Turn RIGHT in 45m | ETA: 18s"
        # → "ARRIVED at destination ✅"

    Args:
        checkpoints: list of checkpoint dicts from analyze_route_checkpoints()
        arrival_radius: distance in meters to consider a checkpoint as "passed"
                        (8m is intentionally large to handle OSM node offset from road centre)
        nominal_speed: vehicle speed in m/s used to calculate ETA (default 2.5 m/s)
    """

    def __init__(self, checkpoints, arrival_radius=8.0, nominal_speed=2.22):
        self.checkpoints = checkpoints
        self.arrival_radius = arrival_radius
        self.nominal_speed = nominal_speed
        self.current_idx = 0   # index of the checkpoint we are currently heading TOWARD
        self.arrived = False

    def reset(self, checkpoints):
        """Reset tracker with a new route's checkpoints."""
        self.checkpoints = checkpoints
        self.current_idx = 0
        self.arrived = False

    def update(self, vx, vy):
        """
        Process a new vehicle position (vx, vy) in local Cartesian metres.
        Advances the checkpoint index when the vehicle is within arrival_radius.
        Returns a human-readable navigation status string.
        """
        if self.arrived or not self.checkpoints:
            return "ARRIVED at destination ✅"

        # Advance past any checkpoints the vehicle has already passed
        while self.current_idx < len(self.checkpoints):
            target = self.checkpoints[self.current_idx]
            dist_to_target = math.hypot(vx - target['x'], vy - target['y'])

            if dist_to_target <= self.arrival_radius:
                # Passed this checkpoint
                if target['turn'] == 'ARRIVED':
                    self.arrived = True
                    return "ARRIVED at destination ✅"
                self.current_idx += 1
            else:
                break

        # Guard: all checkpoints consumed
        if self.current_idx >= len(self.checkpoints):
            self.arrived = True
            return "ARRIVED at destination ✅"

        target = self.checkpoints[self.current_idx]
        dist_to_target = math.hypot(vx - target['x'], vy - target['y'])
        eta_s = round(dist_to_target / max(0.5, self.nominal_speed))
        total = len(self.checkpoints)
        turn = target['turn']
        jtype = target.get('junction_type', 'Junction')

        return (
            f"Checkpoint {self.current_idx + 1}/{total} | "
            f"{turn} at {jtype} in {dist_to_target:.0f}m | "
            f"ETA: {eta_s}s"
        )
