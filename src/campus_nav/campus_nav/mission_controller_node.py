"""
Mission Controller Node — Phase 3

The orchestrator that ties all layers together.

When you publish "StartStop|GoalStop" to /campus/mission:
  1. Teleports the buggy to the start stop's road node in Gazebo
  2. Triggers A* replanning via /campus/goal_stop
  3. Receives the planned path and converts it to Gazebo coordinates
  4. Publishes the Gazebo-frame path to /campus/target_path for the path follower
  5. Monitors pursuit_status — reports live mission progress
  6. On ARRIVED, publishes mission complete status

Teleportation is done via the `gz service` CLI tool which calls
the Gazebo /world/<world>/set_pose service.
"""

import math
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from campus_nav.coord_bridge import nav_to_gz, gz_to_nav
from campus_nav.osm_loader import CampusMap
import os


WORLD_NAME = 'campus_world'
MODEL_NAME = 'saye'


class MissionControllerNode(Node):

    def __init__(self):
        super().__init__('mission_controller_node')
        self.cb_group = ReentrantCallbackGroup()

        # Load map to look up stop road node coordinates
        default_osm = '/home/adarsh4our/CAMP A/campus_with_junctions_and_stops.osm'
        if not os.path.exists(default_osm):
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('campus_nav')
            default_osm = os.path.join(pkg, 'data', 'campus.osm')
        self.declare_parameter('osm_file', default_osm)
        osm_file = self.get_parameter('osm_file').get_parameter_value().string_value

        self.campus = CampusMap(osm_file)
        self.get_logger().info(
            f"MissionController loaded map: {len(self.campus.stops)} stops, "
            f"{len(self.campus.main_nodes)} road nodes")

        # State
        self.current_start_stop: str = None
        self.current_goal_stop: str = None
        self.waiting_for_path: bool = False
        self.mission_active: bool = False
        self.mission_start_time: float = None
        self.mission_state: str = 'IDLE'  # 'IDLE', 'LEG_1_DISPATCH', 'BOARDING', 'LEG_2_DISPATCH'
        self.current_pose_nav = None
        self._pose_last_updated: float = 0.0   # wall-clock time of last pose update
        self._pending_mission: tuple = None      # (start_name, goal_name) queued until fresh pose
        self._boarding_timer = None

        # Publishers
        self.plan_route_pub   = self.create_publisher(String, '/campus/plan_route', 10)
        self.goal_pub         = self.create_publisher(String, '/campus/goal_stop', 10)
        self.start_pub        = self.create_publisher(String, '/campus/start_stop', 10)
        self.clear_pub        = self.create_publisher(String, '/campus/clear_route', 10)
        self.target_path_pub  = self.create_publisher(Path, '/campus/target_path', 10)
        self.status_pub       = self.create_publisher(String, '/campus/nav_status', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/campus/vehicle_pose', self.vehicle_pose_callback, 10,
            callback_group=self.cb_group)

        # /campus/mission → "StartStop|GoalStop"
        self.mission_sub = self.create_subscription(
            String, '/campus/mission', self.mission_callback, 10,
            callback_group=self.cb_group)

        # Receive planned path from planner node and convert to GZ coords
        self.path_sub = self.create_subscription(
            Path, '/campus/global_path', self.planned_path_callback, 10,
            callback_group=self.cb_group)

        # Also subscribe to speed profile to embed speeds into path waypoints
        from std_msgs.msg import Float32MultiArray
        self.speeds: list = []
        self.speed_sub = self.create_subscription(
            Float32MultiArray, '/campus/speed_profile', self.speed_callback, 10,
            callback_group=self.cb_group)

        # Monitor path follower status
        self.pursuit_sub = self.create_subscription(
            String, '/campus/pursuit_status', self.pursuit_callback, 10,
            callback_group=self.cb_group)

        self.get_logger().info(
            "MissionController ready. 2-Leg Autonomous Missions active ('Start|Goal').")

    def vehicle_pose_callback(self, msg: PoseStamped):
        """Track buggy's live position in planner coords."""
        self.current_pose_nav = (msg.pose.position.x, msg.pose.position.y)
        self._pose_last_updated = time.time()

        # If a mission was queued waiting for fresh pose, execute it now
        if self._pending_mission:
            start_name, goal_name = self._pending_mission
            self._pending_mission = None
            self.get_logger().info(f"📍 Pose fresh — executing queued mission: '{start_name}' → '{goal_name}'")
            self._dispatch_mission(start_name, goal_name)

    # ── Mission Entry Point ────────────────────────────────────────────────────

    def mission_callback(self, msg: String):
        """Parse mission string and execute autonomous 2-leg route (Pick-up → Boarding → Drop-off)."""
        raw = msg.data.strip()
        if '|' not in raw:
            self.get_logger().error(
                f"Invalid mission format: '{raw}'. Expected 'StartStop|GoalStop'")
            return

        parts = raw.split('|', 1)
        start_name = parts[0].strip()
        goal_name  = parts[1].strip()

        self.get_logger().info(f"🚀 Mission Request: '{start_name}' → '{goal_name}'")

        # Validate stops
        start_nid = self.campus.get_stop_road_node(start_name)
        goal_nid  = self.campus.get_stop_road_node(goal_name)

        if not start_nid:
            self.get_logger().error(
                f"Unknown start stop: '{start_name}'. Available: {sorted(self.campus.stops.keys())}")
            return
        if not goal_nid:
            self.get_logger().error(
                f"Unknown goal stop: '{goal_name}'. Available: {sorted(self.campus.stops.keys())}")
            return

        # ── Pose freshness check ───────────────────────────────────────────────
        # Reject stale poses (older than 1.5s) — e.g. after teleporting, old SAB C
        # pose may still be in current_pose_nav while the buggy is now at LOC.
        pose_age = time.time() - self._pose_last_updated
        pose_is_fresh = (self.current_pose_nav is not None) and (pose_age < 1.5)

        if not pose_is_fresh:
            self.get_logger().warn(
                f"⏳ Pose not fresh (age={pose_age:.1f}s). Queuing mission until fresh pose arrives...")
            self._pending_mission = (start_name, goal_name)
            return

        self._dispatch_mission(start_name, goal_name)

    def _dispatch_mission(self, start_name: str, goal_name: str):
        """Dispatch a validated mission using the current live pose."""
        start_nid = self.campus.get_stop_road_node(start_name)

        self.current_start_stop = start_name
        self.current_goal_stop  = goal_name
        self.mission_active = True
        self.mission_start_time = time.time()

        start_nav_x = self.campus.xy_nodes[start_nid][0]
        start_nav_y = self.campus.xy_nodes[start_nid][1]

        # Calculate distance from buggy's current location to requested start stop
        dist_to_start = math.hypot(
            self.current_pose_nav[0] - start_nav_x,
            self.current_pose_nav[1] - start_nav_y
        )

        self.get_logger().info(
            f"📐 Buggy at nav ({self.current_pose_nav[0]:.1f}, {self.current_pose_nav[1]:.1f}), "
            f"'{start_name}' at nav ({start_nav_x:.1f}, {start_nav_y:.1f}), "
            f"dist={dist_to_start:.1f}m")

        if dist_to_start > 25.0:
            # Buggy is elsewhere (e.g. at LOC): Leg 1 (drive from live buggy position to pick-up stop)
            self.mission_state = 'LEG_1_DISPATCH'
            self._publish_status(
                f"Leg 1: Driving from current location to Pick-up point '{start_name}' ({dist_to_start:.0f}m away)...")
            self.waiting_for_path = True
            self.plan_route_pub.publish(String(data=f"CURRENT|{start_name}"))
        else:
            # Buggy is already at start stop: Leg 2 directly
            self.mission_state = 'LEG_2_DISPATCH'
            self._publish_status(
                f"At start '{start_name}'. Planning route to '{goal_name}'...")
            self.waiting_for_path = True
            self.plan_route_pub.publish(String(data=f"CURRENT|{goal_name}"))

    # ── Path Reception ─────────────────────────────────────────────────────────

    def speed_callback(self, msg):
        """Cache the latest speed profile from the planner."""
        self.speeds = list(msg.data)

    def planned_path_callback(self, msg: Path):
        """
        Receive planned path (in PLANNER/campus_nav coords) from planner_node.
        Convert to GAZEBO coords and publish to /campus/target_path for the follower.
        """
        if not self.waiting_for_path:
            return
            
        if not msg.poses:
            self.get_logger().info("Empty path received. Assuming we are already at the destination.")
            self.waiting_for_path = False
            msg_arr = String()
            msg_arr.data = 'ARRIVED'
            self.pursuit_callback(msg_arr)
            return

        self.waiting_for_path = False
        n = len(msg.poses)
        self.get_logger().info(
            f"Path received: {n} poses in planner coords. Converting to Gazebo coords...")

        # Build a new Path in Gazebo coordinates
        gz_path = Path()
        gz_path.header.stamp = self.get_clock().now().to_msg()
        gz_path.header.frame_id = 'world'

        for i, pose_stamped in enumerate(msg.poses):
            nav_x = pose_stamped.pose.position.x
            nav_y = pose_stamped.pose.position.y
            gz_x, gz_y = nav_to_gz(nav_x, nav_y)

            # Embed pre-computed speed into z channel (read by path follower)
            spd = self.speeds[i] if i < len(self.speeds) else 2.22

            ps = PoseStamped()
            ps.header = gz_path.header
            ps.pose.position.x = gz_x
            ps.pose.position.y = gz_y
            ps.pose.position.z = float(spd)   # speed embedded in z
            ps.pose.orientation = pose_stamped.pose.orientation
            gz_path.poses.append(ps)

        self.target_path_pub.publish(gz_path)
        dest = self.current_start_stop if self.mission_state == 'LEG_1_DISPATCH' else self.current_goal_stop
        self._publish_status(f"Route active ({n} waypoints). Driving to '{dest}'...")
        self.get_logger().info(f"✅ Published Gazebo path ({n} poses) for target '{dest}'")

    # ── Pursuit Monitoring ─────────────────────────────────────────────────────

    def pursuit_callback(self, msg: String):
        """Monitor path follower progress and manage 2-leg state transitions."""
        if not self.mission_active:
            return

        status = msg.data

        if 'ARRIVED' in status:
            if self.mission_state == 'LEG_1_DISPATCH':
                # Leg 1 Complete: Arrived at pick-up stop
                self.mission_state = 'BOARDING'
                self._publish_status(
                    f"Arrived at Pick-up '{self.current_start_stop}'! Pausing 5s for boarding...")
                self.get_logger().info(f"🛑 Arrived at pick-up '{self.current_start_stop}'. Pausing 5s...")
                # Use one-shot timer instead of time.sleep(5.0)
                self._boarding_timer = self.create_timer(5.0, self._boarding_complete)

            elif self.mission_state == 'LEG_2_DISPATCH':
                # Leg 2 Complete: Arrived at final destination
                elapsed = time.time() - self.mission_start_time if self.mission_start_time else 0
                m, s = divmod(int(elapsed), 60)
                self._publish_status(
                    f"Mission Complete ✅ | '{self.current_start_stop}' → "
                    f"'{self.current_goal_stop}' | Time: {m}m {s}s")
                self.get_logger().info(f"🏁 Mission complete in {m}m {s}s")
                self.mission_active = False
                self.mission_state = 'IDLE'
                self.clear_pub.publish(String(data='clear'))
        elif 'OFF_PATH' in status:
            # Automatic Dynamic Re-planning on Deviation / Recovery
            now = time.time()
            if not hasattr(self, '_last_replan_time') or (now - self._last_replan_time > 3.0):
                self._last_replan_time = now
                target = self.current_start_stop if self.mission_state == 'LEG_1_DISPATCH' else self.current_goal_stop
                self.get_logger().warn(f"Buggy off-path! Auto-recalculating fresh route to '{target}'...")
                self._publish_status(f"🔄 Re-calculating route to '{target}' from current pose...")
                self.waiting_for_path = True
                self.plan_route_pub.publish(String(data=f"CURRENT|{target}"))
        else:
            if self.mission_state == 'LEG_1_DISPATCH':
                self._publish_status(f"[Pick-up Leg → {self.current_start_stop}] {status}")
            elif self.mission_state == 'LEG_2_DISPATCH':
                self._publish_status(f"[{self.current_start_stop} → {self.current_goal_stop}] {status}")

    # ── Gazebo Teleport ────────────────────────────────────────────────────────

    def _boarding_complete(self):
        """Called by timer 5s after arriving at pick-up stop."""
        # Cancel the one-shot timer
        self._boarding_timer.cancel()
        self._boarding_timer = None
        # Launch Leg 2: Drive to drop-off stop
        self.mission_state = 'LEG_2_DISPATCH'
        self._publish_status(
            f"Leg 2: Boarding complete! Driving to Drop-off '{self.current_goal_stop}'...")
        self.waiting_for_path = True
        self.plan_route_pub.publish(String(data=f"CURRENT|{self.current_goal_stop}"))

    def _teleport_model(self, model_name: str, gz_x: float, gz_y: float,
                        gz_z: float = 1.2, yaw: float = 0.0) -> bool:
        """
        Teleport a Gazebo model to (gz_x, gz_y, gz_z) with given yaw.
        Uses `gz service` CLI to call /world/<world>/set_pose.
        Returns True if the command succeeded.
        """
        sin_h = math.sin(yaw / 2.0)
        cos_h = math.cos(yaw / 2.0)

        req = (
            f'name: "{model_name}", '
            f'position: {{x: {gz_x:.4f}, y: {gz_y:.4f}, z: {gz_z:.4f}}}, '
            f'orientation: {{x: 0.0, y: 0.0, z: {sin_h:.6f}, w: {cos_h:.6f}}}'
        )

        cmd = [
            'gz', 'service',
            '-s', f'/world/{WORLD_NAME}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', req
        ]

        self.get_logger().info(f"Teleporting {model_name} to GZ ({gz_x:.1f}, {gz_y:.1f})")
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if result.returncode == 0:
                self.get_logger().info(f"  Teleport successful.")
                return True
            else:
                self.get_logger().warn(
                    f"  Teleport returned non-zero: {result.stderr.strip()}")
                return False
        except subprocess.TimeoutExpired:
            self.get_logger().warn("  Teleport timed out (5s)")
            return False
        except FileNotFoundError:
            self.get_logger().warn(
                "  'gz' CLI not found. Is Gazebo Harmonic installed and sourced?")
            return False

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"[status] {text}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
