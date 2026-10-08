"""
Odometry Bridge & Vehicle Visualization Node

Responsibilities:
  1. Subscribes to /model/saye/odometry (ground truth Gazebo odometry from ros_gz_bridge).
  2. Converts Gazebo coordinates (gz_x, gz_y) -> planner coordinates (nav_x, nav_y).
  3. Publishes /campus/vehicle_pose (PoseStamped) in map frame for navigation tracking.
  4. Publishes /campus/vehicle_marker (MarkerArray) -> 3D Orange Buggy with white canopy,
     yellow heading arrow, and label, visible in RViz!
  5. Void Recovery Supervisor: If the buggy ever drops below Z = -0.5m, it automatically
     catches the buggy, zeroes velocity, and safely respawns it onto the nearest road.
  6. Ground truth is published ONLY as /gt/odom (frame 'world') to compare against SLAM.
     This node does NOT broadcast the robot's TF: SLAM owns odom -> base_footprint and
     robot_state_publisher owns base_footprint -> base_link -> chassis_link -> sensors.
  7. Publishes static map -> world (the planner/Gazebo coordinate offset). The mission
     controller publishes /campus/target_path in frame 'world', so Nav2 needs this link.
  8. Seeds the static map -> odom transform once from the first ground-truth pose
     (simulation only; on the real cart use a known start pose or GPS instead).
  9. Optional bring-up mode (param gt_odom_tf=True): ALSO publishes odom -> base_footprint
     from ground truth, so the whole stack runs before SLAM exists. Set it False as soon
     as SLAM publishes odom -> base_footprint (two parents would break TF).

Frame convention: base_link is x-forward (REP-103), so the Gazebo yaw of base_footprint
is already the driving yaw. No nav_base_link frame is used any more.
"""

import math
import subprocess
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Empty
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from campus_nav.coord_bridge import gz_to_nav, nav_to_gz, yaw_from_quaternion, quaternion_from_yaw
from campus_nav.osm_loader import CampusMap
import os


WORLD_NAME = 'campus_world'
MODEL_NAME = 'saye'
ROAD_SURFACE_Z = 0.60  # Road mesh is at 0.30m + wheel radius 0.28m


class OdomBridgeNode(Node):
    def __init__(self):
        super().__init__('odom_bridge_node')

        self.declare_parameter('model_name', MODEL_NAME)
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value

        # Load map for void recovery road snapping
        candidates_osm = []
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('campus_nav')
            candidates_osm.extend([
                os.path.join(pkg_share, 'data', 'campus_with_junctions_and_stops.osm'),
                os.path.join(pkg_share, 'data', 'campus.osm')
            ])
        except Exception:
            pass
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.abspath(os.path.join(curr_dir, '..', '..', '..'))
        candidates_osm.extend([
            os.path.join(curr_dir, '..', 'data', 'campus_with_junctions_and_stops.osm'),
            os.path.join(curr_dir, '..', 'data', 'campus.osm'),
            os.path.join(repo_root, 'campus_with_junctions_and_stops.osm'),
            os.path.join(repo_root, 'campus.osm')
        ])
        default_osm = next((p for p in candidates_osm if os.path.exists(p)), '')
        self.campus = CampusMap(default_osm)

        # Publishers
        self.vehicle_pose_pub = self.create_publisher(
            PoseStamped, '/campus/vehicle_pose', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/gt/odom', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/campus/vehicle_marker', 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.abort_pub = self.create_publisher(
            Path, '/campus/target_path', 10)
        # Fired after a void-recovery teleport so SLAM / localization can be reset
        self.respawn_pub = self.create_publisher(
            Empty, '/campus/respawn_event', 10)

        # Static TF map -> odom, seeded once from the first ground-truth pose.
        # (odom -> base_footprint comes from SLAM, the rest from robot_state_publisher.)
        self.declare_parameter('seed_map_to_odom', True)
        self.seed_map_to_odom = self.get_parameter(
            'seed_map_to_odom').get_parameter_value().bool_value
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.map_to_odom_sent = False
        self.seed_pose = None   # (nav_x, nav_y, z, yaw) of the first ground-truth pose

        # Bring-up mode: ground-truth odom -> base_footprint (turn OFF when SLAM runs)
        self.declare_parameter('gt_odom_tf', False)
        self.gt_odom_tf = self.get_parameter('gt_odom_tf').get_parameter_value().bool_value
        self.tf_broadcaster = TransformBroadcaster(self)

        # Static map -> world: planner coords = Gazebo coords + (802.0, 679.7).
        # Needed because /campus/target_path is published in frame 'world'.
        t_world = TransformStamped()
        t_world.header.stamp = self.get_clock().now().to_msg()
        t_world.header.frame_id = 'map'
        t_world.child_frame_id = 'world'
        t_world.transform.translation.x = 802.0
        t_world.transform.translation.y = 679.7
        t_world.transform.translation.z = 0.0
        t_world.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t_world)

        # Subscribers
        # Ground truth world odometry from Gazebo bridge (/model/saye/odometry_world)
        self.odom_sub = self.create_subscription(
            Odometry, '/model/saye/odometry_world', self.odometry_callback, 10)

        self.last_gz_x = None
        self.last_gz_y = None
        self.last_gz_z = 0.60
        self.last_yaw = 0.0
        self.last_recovery_time = 0.0

        self.get_logger().info(
            f"OdomBridgeNode ready. Tracking '{self.model_name}' via ground-truth /model/saye/odometry.")

    def odometry_callback(self, msg: Odometry):
        """Process ground-truth world odometry from Gazebo."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self._process_pose(pos.x, pos.y, pos.z, ori)

    def _process_pose(self, gz_x, gz_y, gz_z, orientation_q):
        yaw = yaw_from_quaternion(orientation_q)
        self.last_gz_x = gz_x
        self.last_gz_y = gz_y
        self.last_gz_z = gz_z
        self.last_yaw = yaw

        # ── 1. Void Recovery Supervisor ─────────────────────────────────────────
        if gz_z < -0.5:
            now_sec = time.time()
            if now_sec - self.last_recovery_time > 2.0:
                self.last_recovery_time = now_sec
                self._recover_from_void(gz_x, gz_y)
                return

        # ── 2. Convert to planner coords ────────────────────────────────────────
        nav_x, nav_y = gz_to_nav(gz_x, gz_y)
        now = self.get_clock().now().to_msg()

        # base_link is x-forward, so the model yaw IS the driving heading
        driving_yaw = yaw
        qx, qy, qz, qw = quaternion_from_yaw(driving_yaw)

        # ── 3. Publish /campus/vehicle_pose ─────────────────────────────────────
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = nav_x
        pose_msg.pose.position.y = nav_y
        pose_msg.pose.position.z = gz_z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.vehicle_pose_pub.publish(pose_msg)

        # ── 4. Publish ground truth as /gt/odom (NOT /odom) ─────────────────────────────────────
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = gz_x
        odom_msg.pose.pose.position.y = gz_y
        odom_msg.pose.pose.position.z = gz_z
        odom_msg.pose.pose.orientation = orientation_q
        self.odom_pub.publish(odom_msg)

        # ── 4b. Seed static map -> odom once (SLAM's odom frame starts at identity) ──
        if self.seed_pose is None:
            self.seed_pose = (nav_x, nav_y, gz_z, driving_yaw)
        if self.seed_map_to_odom and not self.map_to_odom_sent:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = nav_x
            t.transform.translation.y = nav_y
            t.transform.translation.z = gz_z
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.static_broadcaster.sendTransform(t)
            self.map_to_odom_sent = True
            self.get_logger().info(
                f"Seeded static map->odom at ({nav_x:.2f}, {nav_y:.2f}), yaw {driving_yaw:.2f} rad")

        # ── 4c. Bring-up only: ground-truth odom -> base_footprint (relative to seed pose) ──
        if self.gt_odom_tf and self.seed_pose is not None:
            sx, sy, sz, syaw = self.seed_pose
            dx, dy = nav_x - sx, nav_y - sy
            c, s_ = math.cos(syaw), math.sin(syaw)
            ox = c * dx + s_ * dy
            oy = -s_ * dx + c * dy
            oqx, oqy, oqz, oqw = quaternion_from_yaw(driving_yaw - syaw)
            t_odom = TransformStamped()
            t_odom.header.stamp = now
            t_odom.header.frame_id = 'odom'
            t_odom.child_frame_id = 'base_footprint'
            t_odom.transform.translation.x = ox
            t_odom.transform.translation.y = oy
            t_odom.transform.translation.z = gz_z - sz
            t_odom.transform.rotation.x = oqx
            t_odom.transform.rotation.y = oqy
            t_odom.transform.rotation.z = oqz
            t_odom.transform.rotation.w = oqw
            self.tf_broadcaster.sendTransform(t_odom)

        # ── 5. Publish 3D High-Contrast Vehicle Marker for RViz ────────────────
        self._publish_vehicle_markers(now)

    def _publish_vehicle_markers(self, stamp):
        """Constructs and publishes a 3D high-visibility buggy marker in RViz (attached to chassis_link)."""
        ma = MarkerArray()

        # 1. Main Chassis (Fluorescent Orange)
        body = Marker()
        body.header.frame_id = 'chassis_link'
        body.header.stamp = stamp
        body.ns = 'buggy'
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = 0.0
        body.pose.position.y = 0.60
        body.pose.position.z = 0.40
        body.pose.orientation.w = 1.0
        body.scale.x = 1.3  # Width along X
        body.scale.y = 2.4  # Length along Y
        body.scale.z = 0.5  # Height
        body.color.r = 1.0
        body.color.g = 0.35
        body.color.b = 0.0
        body.color.a = 0.95
        ma.markers.append(body)

        # 2. Canopy Roof (Clean White)
        roof = Marker()
        roof.header = body.header
        roof.ns = 'buggy'
        roof.id = 1
        roof.type = Marker.CUBE
        roof.action = Marker.ADD
        roof.pose.position.x = 0.0
        roof.pose.position.y = 0.60
        roof.pose.position.z = 1.30
        roof.pose.orientation.w = 1.0
        roof.scale.x = 1.2
        roof.scale.y = 1.7
        roof.scale.z = 0.08
        roof.color.r = 0.95
        roof.color.g = 0.95
        roof.color.b = 0.95
        roof.color.a = 0.95
        ma.markers.append(roof)

        # 3. Heading Arrow (Bright Yellow, points straight forward along -Y through the front bumper)
        arrow = Marker()
        arrow.header = body.header
        arrow.ns = 'buggy'
        arrow.id = 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [
            Point(x=0.0, y=0.0, z=0.70),
            Point(x=0.0, y=-2.0, z=0.70)
        ]
        arrow.scale.x = 0.25 # Shaft diameter
        arrow.scale.y = 0.50 # Head diameter
        arrow.scale.z = 0.40 # Head length
        arrow.color.r = 1.0
        arrow.color.g = 1.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0
        ma.markers.append(arrow)

        # 4. Text Label
        label = Marker()
        label.header = body.header
        label.ns = 'buggy'
        label.id = 3
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = 0.0
        label.pose.position.y = 0.0
        label.pose.position.z = 1.85
        label.scale.z = 1.0
        label.text = "saye Buggy"
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        ma.markers.append(label)

        self.marker_pub.publish(ma)

    def _recover_from_void(self, current_gz_x, current_gz_y):
        """Emergency catch: buggy fell below the road. Respawn onto nearest road."""
        self.get_logger().warn(
            f"🚨 Buggy fell below road (Z={self.last_gz_z:.1f}m)! Catching and respawning onto nearest road...")

        # 1. Immediately zero velocity and clear path follower
        self.cmd_pub.publish(Twist())
        self.abort_pub.publish(Path())

        # 2. Find nearest road node to last known location
        nav_x, nav_y = gz_to_nav(current_gz_x, current_gz_y)
        nearest_nid, d = self.campus.snap_to_road(nav_x, nav_y)

        if nearest_nid:
            target_nav_x, target_nav_y = self.campus.xy_nodes[nearest_nid]
            respawn_gz_x, respawn_gz_y = nav_to_gz(target_nav_x, target_nav_y)
        else:
            respawn_gz_x, respawn_gz_y = 15.02, 43.00
            target_nav_x, target_nav_y = gz_to_nav(respawn_gz_x, respawn_gz_y)

        # Orient buggy towards connected road neighbor (inwards towards campus)
        respawn_yaw_gz = 0.0  # nav yaw 0 == gz yaw 0 (base_link is x-forward)
        if nearest_nid and nearest_nid in self.campus.road_graph and self.campus.road_graph[nearest_nid]:
            nbr = next(iter(self.campus.road_graph[nearest_nid]))
            nbr_x, nbr_y = self.campus.xy_nodes[nbr]
            nav_angle = math.atan2(nbr_y - target_nav_y, nbr_x - target_nav_x)
            respawn_yaw_gz = nav_angle

        qz = math.sin(respawn_yaw_gz / 2.0)
        qw = math.cos(respawn_yaw_gz / 2.0)

        # 3. Teleport buggy to road height (Z = 0.60m)
        self.get_logger().info(f"Respawning saye at Gazebo ({respawn_gz_x:.1f}, {respawn_gz_y:.1f}, Z={ROAD_SURFACE_Z})")
        req = (
            f'name: "{self.model_name}", '
            f'position: {{x: {respawn_gz_x:.3f}, y: {respawn_gz_y:.3f}, z: {ROAD_SURFACE_Z:.3f}}}, '
            f'orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}'
        )
        cmd = [
            'gz', 'service',
            '-s', f'/world/{WORLD_NAME}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', req
        ]
        subprocess.run(cmd, capture_output=True, text=True)

        # Tell SLAM / localization the pose just jumped (they must reset and re-seed)
        self.respawn_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()