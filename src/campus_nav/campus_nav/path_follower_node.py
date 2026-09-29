"""
Pure Pursuit Path Follower Node — Phase 2

Subscribes to /campus/target_path (nav_msgs/Path in GAZEBO coordinates)
and /campus/vehicle_pose (PoseStamped in PLANNER coordinates).

Drives the buggy along the path by publishing /cmd_vel (geometry_msgs/Twist).

Algorithm: Pure Pursuit
  1. Find closest trajectory point to current vehicle position
  2. Find a "lookahead point" L_d meters ahead along the path
  3. Compute the circular arc curvature to reach that point
  4. Convert curvature → steer angle → yaw rate for the Ackermann plugin
  5. Apply pre-computed speed profile (slow at curves, cruise on straights)

Control rate: 20 Hz
"""

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from campus_nav.coord_bridge import gz_to_nav, yaw_from_quaternion


class PathFollowerNode(Node):

    # Ackermann geometry (from saye.xacro plugin parameters)
    WHEELBASE = 2.124   # metres
    MAX_STEER = 0.8     # rad (~45.8°)

    def __init__(self):
        super().__init__('path_follower_node')

        from rcl_interfaces.msg import ParameterDescriptor
        dyn_desc = ParameterDescriptor(dynamic_typing=True)

        # Parameters (Configured for 8 km/h cruising)
        self.declare_parameter('lookahead_distance', 4.0, dyn_desc)   # metres (scaled for 8 km/h = 2.22 m/s)
        self.declare_parameter('cruise_speed', 2.22, dyn_desc)        # m/s (8.0 km/h cruising)
        self.declare_parameter('max_speed', 2.22, dyn_desc)           # m/s max ceiling (8.0 km/h)
        self.declare_parameter('min_speed', 0.8, dyn_desc)            # m/s at tight curves (~2.9 km/h)
        self.declare_parameter('goal_tolerance', 2.2, dyn_desc)       # metres to declare arrival
        self.declare_parameter('control_rate_hz', 20.0, dyn_desc)

        self.Ld      = float(self.get_parameter('lookahead_distance').value)
        self.v_cruise= float(self.get_parameter('cruise_speed').value)
        self.v_max   = float(self.get_parameter('max_speed').value)
        self.v_min   = float(self.get_parameter('min_speed').value)
        self.goal_tol= float(self.get_parameter('goal_tolerance').value)
        hz           = float(self.get_parameter('control_rate_hz').value)

        # State
        self.path_gz: list = []       # [(x, y, speed), ...] in Gazebo coords
        self.closest_idx: int = 0
        self.vehicle_gz_x: float = None   # current vehicle position in GZ coords
        self.vehicle_gz_y: float = None
        self.vehicle_yaw: float = 0.0
        self.following: bool = False
        self.arrived: bool = False

        # Publishers
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/campus/pursuit_status', 10)

        # Safety Monitor state
        self.safety_state = "CLEAR"
        self.safety_speed_limit = self.v_cruise
        self.safety_steering_bias = 0.0
        self.safety_obstacle_dist = 999.0
        self.safety_obstacle_type = "NONE"

        # Subscribers
        # /campus/target_path is in GAZEBO coords (converted by mission controller)
        self.path_sub = self.create_subscription(
            Path, '/campus/target_path', self.path_callback, 10)
        # /campus/vehicle_pose is in PLANNER coords — we convert back to GZ here
        self.pose_sub = self.create_subscription(
            PoseStamped, '/campus/vehicle_pose', self.pose_callback, 10)
        # /campus/safety_status receives real-time LiDAR/depth obstacle & void alerts
        self.safety_sub = self.create_subscription(
            String, '/campus/safety_status', self.safety_callback, 10)

        # Control timer
        self.timer = self.create_timer(1.0 / hz, self.control_loop)

        self.get_logger().info(
            f"PathFollowerNode ready | Ld={self.Ld}m | v_cruise={self.v_cruise}m/s "
            f"| goal_tol={self.goal_tol}m | {hz}Hz")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def safety_callback(self, msg: String):
        """Receive real-time safety classification from safety_monitor_node."""
        try:
            import json
            data = json.loads(msg.data)
            self.safety_state = data.get('state', 'CLEAR')
            self.safety_speed_limit = float(data.get('speed_limit', self.v_cruise))
            self.safety_steering_bias = float(data.get('steering_bias', 0.0))
            self.safety_obstacle_dist = float(data.get('obstacle_dist', 999.0))
            self.safety_obstacle_type = data.get('obstacle_type', 'NONE')
        except Exception:
            pass

    def path_callback(self, msg: Path):
        """Receive a new path (in GAZEBO coords) and start following it."""
        if len(msg.poses) < 2:
            self.get_logger().warn("Received path with fewer than 2 poses — ignoring.")
            return

        # Extract (gz_x, gz_y, speed) for each waypoint
        # Speed is encoded in pose.position.z (set by mission controller)
        self.path_gz = [
            (p.pose.position.x, p.pose.position.y, max(self.v_min, p.pose.position.z))
            for p in msg.poses
        ]
        self.closest_idx = 0
        if self.vehicle_gz_x is not None:
            best_d = float('inf')
            for i, (px, py, _) in enumerate(self.path_gz):
                d = math.hypot(px - self.vehicle_gz_x, py - self.vehicle_gz_y)
                if d < best_d:
                    best_d = d
                    self.closest_idx = i
                if d < 1.5:
                    break

        self.following = True
        self.arrived = False
        self.get_logger().info(
            f"New path received: {len(self.path_gz)} waypoints. Starting at index {self.closest_idx}")

    def pose_callback(self, msg: PoseStamped):
        """Receive vehicle pose in PLANNER coords and convert to Gazebo coords."""
        from campus_nav.coord_bridge import nav_to_gz
        nav_x = msg.pose.position.x
        nav_y = msg.pose.position.y
        self.vehicle_gz_x, self.vehicle_gz_y = nav_to_gz(nav_x, nav_y)
        self.vehicle_yaw = yaw_from_quaternion(msg.pose.orientation)

    # ── Control Loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        """Main Pure Pursuit control loop at 20 Hz."""
        if not self.following or self.arrived:
            return
        if self.vehicle_gz_x is None:
            return
        if not self.path_gz:
            return

        vx = self.vehicle_gz_x
        vy = self.vehicle_gz_y
        yaw = self.vehicle_yaw

        # ── 1. Advance closest index ─────────────────────────────────────────
        # Only search forward from current closest (avoids U-turn backtracking)
        search_start = max(0, self.closest_idx - 5)
        best_dist = float('inf')
        for i in range(search_start, len(self.path_gz)):
            px, py, _ = self.path_gz[i]
            d = math.hypot(px - vx, py - vy)
            if d < best_dist:
                best_dist = d
                self.closest_idx = i

        # Safety check: if vehicle has strayed off path (> 10m cross-track error)
        if best_dist > 10.0:
            self._stop()
            self.following = False
            self._publish_status(f"OFF_PATH: Buggy strayed {best_dist:.1f}m off path. Stopped for safety.")
            self.get_logger().warn(f"Buggy strayed {best_dist:.1f}m away from path! Emergency stop engaged.")
            return

        # ── 2. Check goal arrival ─────────────────────────────────────────────
        goal_x, goal_y, _ = self.path_gz[-1]
        dist_to_goal = math.hypot(goal_x - vx, goal_y - vy)

        if dist_to_goal < self.goal_tol:
            self._stop()
            self.arrived = True
            self.following = False
            self._publish_status("ARRIVED")
            self.get_logger().info(f"✅ ARRIVED at goal (within {dist_to_goal:.1f}m).")
            return

        # ── 3. Find lookahead point ───────────────────────────────────────────
        lookahead_pt = None
        target_speed = self.v_cruise
        for i in range(self.closest_idx, len(self.path_gz)):
            px, py, spd = self.path_gz[i]
            # Dynamic lookahead: tighter (2.8m) on sharp turns, full Ld (4.0m) at cruising speed
            ld = max(2.8, min(self.Ld, 1.8 * max(self.v_min, spd)))
            if math.hypot(px - vx, py - vy) >= ld:
                lookahead_pt = (px, py)
                target_speed = spd
                break

        if lookahead_pt is None:
            # Past all waypoints — use the goal
            lookahead_pt = (goal_x, goal_y)
            target_speed = self.v_min

        # ── 4. Pure Pursuit geometry ──────────────────────────────────────────
        lx, ly = lookahead_pt
        # Vector from vehicle to lookahead point in vehicle frame
        dx = lx - vx
        dy = ly - vy

        # Angle to lookahead point in world frame, relative to vehicle heading
        alpha = math.atan2(dy, dx) - yaw
        # Normalize to [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Actual distance to lookahead (may differ from Ld at path end)
        L_actual = max(0.1, math.hypot(dx, dy))

        # Handle target behind vehicle (|alpha| > 90°):
        # When target is behind, pure pursuit degenerates (sin(180°) = 0 causes car to shoot straight ahead).
        # We clamp to full steering lock towards the target and crawl at min_speed to turn around.
        if abs(alpha) > (math.pi / 2.0):
            steer_angle = math.copysign(self.MAX_STEER, alpha)
            target_speed = self.v_min
        else:
            # Pure Pursuit curvature: κ = 2·sin(α) / L
            curvature = 2.0 * math.sin(alpha) / L_actual
            steer_angle = math.atan(curvature * self.WHEELBASE)
            steer_angle = max(-self.MAX_STEER, min(self.MAX_STEER, steer_angle))

        # ── 4b. Void & Road Edge Guard Steering Correction ────────────────────
        if self.safety_state == "EDGE_WARNING":
            steer_angle += self.safety_steering_bias
            steer_angle = max(-self.MAX_STEER, min(self.MAX_STEER, steer_angle))

        # ── 5. Speed control ──────────────────────────────────────────────────
        # Blend target speed from trajectory with reduction based on steer angle
        steer_factor = 1.0 - 0.5 * abs(steer_angle) / self.MAX_STEER
        v = max(self.v_min, min(self.v_max, target_speed * steer_factor))

        # ── 5b. Safety Supervisor Speed Clamping & Emergency Stop ─────────────
        if self.safety_state == "EMERGENCY_STOP":
            self._stop()
            self._publish_status(
                f"🛑 EMERGENCY STOP: {self.safety_obstacle_type} detected at {self.safety_obstacle_dist:.1f}m!")
            return
        elif self.safety_state in ("SLOWDOWN_BUMP", "SLOWDOWN_APPROACH", "EDGE_WARNING"):
            v = min(v, self.safety_speed_limit)

        # ── 6. Convert steer → angular.z for Ackermann plugin ─────────────────
        # From saye teleop: angular.z = tan(steer_angle) / wheelbase * v
        angular_z = (math.tan(steer_angle) / self.WHEELBASE) * v

        # ── 7. Publish /cmd_vel ───────────────────────────────────────────────
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

        # ── 8. Publish status ─────────────────────────────────────────────────
        progress_pct = 100.0 * self.closest_idx / max(1, len(self.path_gz) - 1)
        safety_tag = f" [{self.safety_state}]" if self.safety_state != "CLEAR" else ""
        self._publish_status(
            f"Following: {progress_pct:.0f}% | "
            f"Speed: {v:.2f}m/s ({(v*3.6):.1f}km/h) | "
            f"Steer: {math.degrees(steer_angle):.1f}° | "
            f"Goal: {dist_to_goal:.0f}m{safety_tag}"
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        """Send zero velocity command to stop the buggy."""
        twist = Twist()  # all zeros
        self.cmd_pub.publish(twist)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = f"[follower] {text}"
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
