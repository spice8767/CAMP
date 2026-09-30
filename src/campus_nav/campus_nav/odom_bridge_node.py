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
"""

import math
import subprocess
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

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
        default_osm = next((p for p in ['/home/adarsh4our/CAMP/campus_with_junctions_and_stops.osm',
                                        '/home/adarsh4our/CAMP A/campus_with_junctions_and_stops.osm']
                            if os.path.exists(p)), '/home/adarsh4our/CAMP/campus_with_junctions_and_stops.osm')
        self.campus = CampusMap(default_osm)

        # Publishers
        self.vehicle_pose_pub = self.create_publisher(
            PoseStamped, '/campus/vehicle_pose', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/campus/vehicle_marker', 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.abort_pub = self.create_publisher(
            Path, '/campus/target_path', 10)

        # TF Broadcaster: connects map -> base_footprint -> base_link -> lidar_link
        self.tf_broadcaster = TransformBroadcaster(self)

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

        # Buggy's driving heading in map coordinates: yaw - pi/2 (due to URDF local frame)
        driving_yaw = yaw - (math.pi / 2.0)
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

        # ── 4. Publish /odom (Gazebo frame) ─────────────────────────────────────
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = f'{self.model_name}/base_link'
        odom_msg.pose.pose.position.x = gz_x
        odom_msg.pose.pose.position.y = gz_y
        odom_msg.pose.pose.position.z = gz_z
        odom_msg.pose.pose.orientation = orientation_q
        self.odom_pub.publish(odom_msg)

        # ── 4b. Broadcast TF: map -> world (connects Gazebo world to planner map frame) ───
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'world'
        t.transform.translation.x = 802.0
        t.transform.translation.y = 679.7
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # ── 5. Publish 3D High-Contrast Vehicle Marker for RViz ────────────────
        self._publish_vehicle_markers(nav_x, nav_y, gz_z, driving_yaw, now)

    def _publish_vehicle_markers(self, x, y, z, yaw, stamp):
        """Constructs and publishes a 3D high-visibility buggy marker in RViz."""
        ma = MarkerArray()
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        # 1. Main Chassis (Fluorescent Orange)
        body = Marker()
        body.header.frame_id = 'map'
        body.header.stamp = stamp
        body.ns = 'buggy'
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.position.z = 0.40
        body.pose.orientation.x = qx
        body.pose.orientation.y = qy
        body.pose.orientation.z = qz
        body.pose.orientation.w = qw
        body.scale.x = 2.4  # Length
        body.scale.y = 1.3  # Width
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
        roof.pose.position.x = x
        roof.pose.position.y = y
        roof.pose.position.z = 1.30
        roof.pose.orientation = body.pose.orientation
        roof.scale.x = 1.7
        roof.scale.y = 1.2
        roof.scale.z = 0.08
        roof.color.r = 0.95
        roof.color.g = 0.95
        roof.color.b = 0.95
        roof.color.a = 0.95
        ma.markers.append(roof)

        # 3. Heading Arrow (Bright Yellow)
        arrow = Marker()
        arrow.header = body.header
        arrow.ns = 'buggy'
        arrow.id = 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = 0.70
        arrow.pose.orientation = body.pose.orientation
        arrow.scale.x = 2.0  # Arrow length
        arrow.scale.y = 0.35 # Arrow shaft width
        arrow.scale.z = 0.35 # Arrow head height
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
        label.pose.position.x = x
        label.pose.position.y = y
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
        respawn_yaw_gz = 1.5708
        if nearest_nid and nearest_nid in self.campus.road_graph and self.campus.road_graph[nearest_nid]:
            nbr = next(iter(self.campus.road_graph[nearest_nid]))
            nbr_x, nbr_y = self.campus.xy_nodes[nbr]
            nav_angle = math.atan2(nbr_y - target_nav_y, nbr_x - target_nav_x)
            respawn_yaw_gz = nav_angle + (math.pi / 2.0)

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
