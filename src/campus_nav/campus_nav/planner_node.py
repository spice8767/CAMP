"""
ROS 2 Planner Node for Campus Road Navigation and A* Path Planning.
"""

import math
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from campus_nav.osm_loader import CampusMap
from campus_nav.astar import astar_search
from campus_nav.trajectory import generate_smooth_trajectory
from campus_nav.checkpoints import analyze_route_checkpoints


class CampusPlannerNode(Node):
    def __init__(self):
        super().__init__('campus_planner_node')

        # Declare parameters
        pkg_share = get_package_share_directory('campus_nav') if 'campus_nav' in os.environ.get('AMENT_PREFIX_PATH', '') else ''
        default_osm = '/home/adarsh4our/CAMP A/campus_with_junctions_and_stops.osm' if os.path.exists('/home/adarsh4our/CAMP A/campus_with_junctions_and_stops.osm') else (os.path.join(pkg_share, 'data', 'campus.osm') if pkg_share else '/home/adarsh4our/CAMP A/campus_with_junctions_and_stops.osm')
        default_yaml = os.path.join(pkg_share, 'config', 'campus_junctions_and_stops.yaml') if pkg_share else '/home/adarsh4our/CAMP A/src/campus_nav/config/campus_junctions_and_stops.yaml'

        from rcl_interfaces.msg import ParameterDescriptor
        dyn_desc = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter('osm_file', default_osm)
        self.declare_parameter('stops_file', default_yaml)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('nominal_speed', 2.22, dyn_desc)
        self.declare_parameter('min_turning_radius', 2.5, dyn_desc)
        self.declare_parameter('default_start_stop', 'SAB C')
        self.declare_parameter('default_goal_stop', 'Chemistry Block')

        osm_file = str(self.get_parameter('osm_file').value)
        stops_file = str(self.get_parameter('stops_file').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.nominal_speed = float(self.get_parameter('nominal_speed').value)
        self.min_turning_radius = float(self.get_parameter('min_turning_radius').value)
        self.current_start_stop = str(self.get_parameter('default_start_stop').value)
        self.current_goal_stop = str(self.get_parameter('default_goal_stop').value)

        self.get_logger().info(f"Loading Campus OSM: {osm_file}")
        self.campus = CampusMap(osm_file, stops_file)
        self.get_logger().info(f"Loaded {len(self.campus.main_nodes)} road nodes, {len(self.campus.junctions)} junctions, {len(self.campus.stops)} stops.")

        # Broadcast static identity TF for map frame to ensure RViz has valid TF
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/campus/global_path', 10)
        self.nav2_plan_pub = self.create_publisher(Path, '/plan', 10)
        self.speed_pub = self.create_publisher(Float32MultiArray, '/campus/speed_profile', 10)
        self.status_pub = self.create_publisher(String, '/campus/nav_status', 10)
        self.road_marker_pub = self.create_publisher(Marker, '/campus/road_graph', 10)
        self.junction_marker_pub = self.create_publisher(MarkerArray, '/campus/junction_markers', 10)
        self.stop_marker_pub = self.create_publisher(MarkerArray, '/campus/stop_markers', 10)
        self.building_marker_pub = self.create_publisher(MarkerArray, '/campus/building_markers', 10)
        self.route_ribbon_pub = self.create_publisher(Marker, '/campus/route_ribbon', 10)
        self.centerline_pub = self.create_publisher(Marker, '/campus/route_centerline', 10)
        self.current_ribbon = None
        self.current_centerline = None
        self.current_path = None
        self.checkpoint_tracker = None  # set when a route is planned
        self._pending_route = None

        # Subscribers
        self.goal_stop_sub = self.create_subscription(String, '/campus/goal_stop', self.goal_stop_callback, 10)
        self.start_stop_sub = self.create_subscription(String, '/campus/start_stop', self.start_stop_callback, 10)
        self.rviz_goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_callback, 10)
        self.plan_route_sub = self.create_subscription(String, '/campus/plan_route', self.plan_route_callback, 10)
        self.clear_route_sub = self.create_subscription(String, '/campus/clear_route', self.clear_route_callback, 10)

        # Vehicle position subscribers for live checkpoint tracking
        # Accepts either a PoseStamped (from GPS/localization) or Odometry (from wheel encoders)
        from nav_msgs.msg import Odometry
        self.vehicle_pose_sub = self.create_subscription(
            PoseStamped, '/campus/vehicle_pose', self.vehicle_pose_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Build static road graph marker once (roads never change — only timestamp updates each tick)
        self.cached_road_marker = self._build_road_graph_marker()

        # Periodic timer for markers and active path (2 Hz continuous refresh for instant RViz loading)
        self.marker_timer = self.create_timer(0.5, self.publish_markers)

        # Do NOT plan an initial route on startup: map loads clean showing only roads, stops, and live buggy.
        # Route visuals appear only when a mission or destination is requested.
        self.current_start_stop = None
        self.current_goal_stop = None

    # ── Fix 3: Build road graph marker once at startup ──────────────────────
    def _build_road_graph_marker(self):
        """Build the static road graph LINE_LIST marker once. Roads never change."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = "road_network"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 3.6          # 3.6m wide asphalt road surface
        m.color.r = 0.28
        m.color.g = 0.30
        m.color.b = 0.35
        m.color.a = 0.95
        for refs in self.campus.road_ways:
            for i in range(len(refs) - 1):
                u, v = refs[i], refs[i + 1]
                if u in self.campus.xy_nodes and v in self.campus.xy_nodes:
                    m.points.append(Point(x=self.campus.xy_nodes[u][0], y=self.campus.xy_nodes[u][1], z=0.0))
                    m.points.append(Point(x=self.campus.xy_nodes[v][0], y=self.campus.xy_nodes[v][1], z=0.0))
        return m

    # ── Fix 4: Live vehicle position callbacks ───────────────────────────────
    def vehicle_pose_callback(self, msg):
        """Called when GPS/localization publishes vehicle position as PoseStamped."""
        from campus_nav.coord_bridge import yaw_from_quaternion
        yaw = yaw_from_quaternion(msg.pose.orientation)
        self.latest_pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        
        if getattr(self, '_pending_route', None):
            start_name, goal_name = self._pending_route
            self._pending_route = None
            self.get_logger().info(f"📍 Executing queued route: '{start_name}' -> '{goal_name}'")
            self.plan_named_route(start_name, goal_name)

        if self.checkpoint_tracker is not None:
            status = self.checkpoint_tracker.update(
                msg.pose.position.x, msg.pose.position.y)
            status_msg = String()
            status_msg.data = status
            self.status_pub.publish(status_msg)

    def _reorient_180_gz(self, nav_x, nav_y, current_yaw):
        """Rotate buggy 180 degrees in Gazebo simulation when facing a dead end."""
        import subprocess
        from campus_nav.coord_bridge import nav_to_gz
        gz_x, gz_y = nav_to_gz(nav_x, nav_y)
        new_driving_yaw = current_yaw + math.pi
        new_gz_yaw = new_driving_yaw + (math.pi / 2.0)
        qz = math.sin(new_gz_yaw / 2.0)
        qw = math.cos(new_gz_yaw / 2.0)
        req = (
            f'name: "saye", '
            f'position: {{x: {gz_x:.3f}, y: {gz_y:.3f}, z: 0.60}}, '
            f'orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}'
        )
        cmd = [
            'gz', 'service',
            '-s', '/world/campus_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', req
        ]
        subprocess.run(cmd, capture_output=True, text=True)

    def odom_callback(self, msg):
        """Called when wheel odometry publishes vehicle position as Odometry."""
        if self.checkpoint_tracker is not None:
            status = self.checkpoint_tracker.update(
                msg.pose.pose.position.x, msg.pose.pose.position.y)
            status_msg = String()
            status_msg.data = status
            self.status_pub.publish(status_msg)

    def start_stop_callback(self, msg):
        stop_name = msg.data.strip()
        if stop_name.upper() in ['CURRENT', 'CURRENT_LOCATION', 'LIVE']:
            if hasattr(self, 'latest_pose') and self.latest_pose:
                x, y, yaw = self.latest_pose
                fwd_nid, rear_nid, is_dead_end, d = self.campus.snap_to_road_directed(x, y, yaw)
                if is_dead_end:
                    self.get_logger().warn(
                        f"🚨 Buggy facing dead-end node {fwd_nid}! Auto-reorienting 180° down open road...")
                    self._reorient_180_gz(x, y, yaw)
                    self.current_start_node_override = rear_nid
                else:
                    self.current_start_node_override = fwd_nid
                self.current_start_stop = 'Current Location'
                self.get_logger().info(
                    f"Start set to Current Location (Forward Node {self.current_start_node_override}, offset {d:.1f}m)")
                return
        if self.campus.get_stop_road_node(stop_name):
            self.current_start_node_override = None
            self.current_start_stop = stop_name
            self.get_logger().info(f"Start stop set to: {self.current_start_stop}")
            status_msg = String()
            status_msg.data = f"Start location set to: {self.current_start_stop}"
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().warn(f"Unknown start stop: '{stop_name}'. Available: {list(self.campus.stops.keys())}")

    def clear_route_callback(self, msg=None):
        """Clear active route visuals from RViz and reset planner state."""
        self.current_ribbon = None
        self.current_centerline = None
        self.current_path = None
        self.current_start_stop = None
        self.current_goal_stop = None
        self.checkpoint_tracker = None
        now = self.get_clock().now().to_msg()
        for ns, pub in [('route_ribbon', self.route_ribbon_pub), ('route_centerline', self.centerline_pub)]:
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.ns = ns
            m.id = 0
            m.action = Marker.DELETEALL
            pub.publish(m)
        empty_path = Path()
        empty_path.header.stamp = now
        empty_path.header.frame_id = self.frame_id
        self.path_pub.publish(empty_path)
        self.nav2_plan_pub.publish(empty_path)
        self.get_logger().info("🧹 Cleared active route and markers from RViz.")

    def goal_stop_callback(self, msg):
        goal_name = msg.data.strip()
        start = self.current_start_stop if self.current_start_stop else 'CURRENT'
        self.plan_named_route(start, goal_name)

    def plan_route_callback(self, msg):
        raw = msg.data.strip()
        if '|' in raw:
            parts = raw.split('|', 1)
            start_name = parts[0].strip()
            goal_name = parts[1].strip()
            self.get_logger().info(f"📍 Route Plan Request: '{start_name}' → '{goal_name}'")
            self.plan_named_route(start_name, goal_name)

    def rviz_goal_callback(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f"Received RViz 2D Goal Pose: ({gx:.1f}m, {gy:.1f}m)")
        goal_nid, gdist = self.campus.snap_to_road(gx, gy)
        
        start_nid = self.campus.get_stop_road_node(self.current_start_stop)
        if not start_nid:
            start_nid = list(self.campus.main_nodes)[0]

        self.execute_route_plan(start_nid, goal_nid, f"Custom Click ({gx:.1f}, {gy:.1f})")

    def plan_named_route(self, start_name, goal_name, one_shot=False):
        if one_shot:
            if hasattr(self, '_initial_planned'):
                return
            self._initial_planned = True

        norm_start = (start_name.strip().upper().replace('_', ' ')) if start_name else ''
        if getattr(self, 'current_start_node_override', None):
            start_nid = self.current_start_node_override
            self.current_start_node_override = None
            label_start = 'Current Location'
        elif not norm_start or norm_start in ['CURRENT', 'CURRENT LOCATION', 'LIVE']:
            if hasattr(self, 'latest_pose') and self.latest_pose:
                x, y, yaw = self.latest_pose
                # Use nearest road node — buggy is stationary when mission starts,
                # so we want the closest node (which IS the LOC stop node itself),
                # not the "forward" directional node which may be further away.
                start_nid, snap_dist = self.campus.snap_to_road(x, y)
                label_start = 'Current Location'
                self.get_logger().info(
                    f"📍 Snapped current pos ({x:.1f},{y:.1f}) to node {start_nid} (dist={snap_dist:.1f}m)")
            else:
                self._pending_route = (start_name or 'CURRENT', goal_name)
                self.get_logger().warn("⏳ No vehicle pose yet! Queuing route request — will plan once odometry arrives...")
                return
        else:
            start_nid = self.campus.get_stop_road_node(start_name)
            label_start = start_name

        goal_nid = self.campus.get_stop_road_node(goal_name)

        if not start_nid:
            self.get_logger().error(f"Cannot find start stop: '{start_name}'")
            return
        if not goal_nid:
            self.get_logger().error(f"Cannot find goal stop: '{goal_name}'")
            return

        self.current_start_stop = label_start
        self.current_goal_stop = goal_name
        self.execute_route_plan(start_nid, goal_nid, f"{label_start} -> {goal_name}")

    def execute_route_plan(self, start_nid, goal_nid, label):
        self.get_logger().info(f"Computing A* route for [{label}]...")
        node_path, total_dist = astar_search(self.campus.road_graph, self.campus.xy_nodes, start_nid, goal_nid)

        if not node_path:
            self.get_logger().error(f"No navigable route found between nodes {start_nid} and {goal_nid}!")
            status_msg = String()
            status_msg.data = f"Failed to find route: {label}"
            self.status_pub.publish(status_msg)
            return

        self.get_logger().info(f"✅ Route Found: {total_dist:.1f}m over {len(node_path)} road nodes.")

        # Analyze junction checkpoints & turn directions
        checkpoints, directions = analyze_route_checkpoints(node_path, self.campus.xy_nodes, self.campus.junctions, self.nominal_speed)
        for d in directions:
            self.get_logger().info(f"  {d}")

        # Fix 4: Activate live checkpoint tracker with new route
        from campus_nav.checkpoints import CheckpointTracker
        self.checkpoint_tracker = CheckpointTracker(checkpoints, arrival_radius=8.0)
        self.get_logger().info(f"  Live tracker armed: {len(checkpoints)} checkpoints, arrival_radius=8m")

        # Generate smooth trajectory
        raw_waypoints = [self.campus.xy_nodes[nid] for nid in node_path]
        trajectory = generate_smooth_trajectory(
            raw_waypoints,
            step_size=0.5,
            min_turning_radius=self.min_turning_radius,
            v_max=self.nominal_speed
        )

        # Publish nav_msgs/Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id

        speed_array = Float32MultiArray()

        for pt in trajectory:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = pt['x']
            ps.pose.position.y = pt['y']
            ps.pose.position.z = 0.0

            # Yaw to quaternion
            half_yaw = pt['yaw'] * 0.5
            ps.pose.orientation.z = math.sin(half_yaw)
            ps.pose.orientation.w = math.cos(half_yaw)

            path_msg.poses.append(ps)
            speed_array.data.append(float(pt['v']))

        self.path_pub.publish(path_msg)
        self.nav2_plan_pub.publish(path_msg)
        self.speed_pub.publish(speed_array)
        self.current_path = path_msg

        # 1. Active Route Road Carpet: 3.8m wide flat planar ribbon (luminous electric green)
        # Using TRIANGLE_LIST with per-vertex colors forces Ogre to disable lighting,
        # ensuring 100% emissive luminosity independent of camera light angles or shadowing.
        ribbon = Marker()
        ribbon.header.stamp = path_msg.header.stamp
        ribbon.header.frame_id = self.frame_id
        ribbon.ns = "route_ribbon"
        ribbon.id = 0
        ribbon.type = Marker.TRIANGLE_LIST
        ribbon.action = Marker.ADD
        ribbon.pose.orientation.w = 1.0
        ribbon.scale.x = 1.0
        ribbon.scale.y = 1.0
        ribbon.scale.z = 1.0
        green_color = ColorRGBA(r=0.05, g=1.00, b=0.20, a=1.0)
        ribbon.color = green_color

        half_w = 1.90  # 3.8m total width, solidly covering the 3.6m asphalt road
        z_road = 0.15  # sits flat just above the z=0.0 dark asphalt road

        left_pts = []
        right_pts = []
        for pt in trajectory:
            yaw = pt['yaw']
            nx = -math.sin(yaw) * half_w
            ny = math.cos(yaw) * half_w
            left_pts.append(Point(x=pt['x'] + nx, y=pt['y'] + ny, z=z_road))
            right_pts.append(Point(x=pt['x'] - nx, y=pt['y'] - ny, z=z_road))

        ribbon.points = []
        ribbon.colors = []
        for i in range(len(left_pts) - 1):
            l1, r1 = left_pts[i], right_pts[i]
            l2, r2 = left_pts[i + 1], right_pts[i + 1]
            ribbon.points.extend([r1, r2, l2, r1, l2, l1])
            ribbon.colors.extend([green_color] * 6)

        self.route_ribbon_pub.publish(ribbon)
        self.current_ribbon = ribbon

        # 2. Crisp White Centerline: 0.36m wide flat planar ribbon (pure white)
        # Sits at z=0.25m, strictly 10cm above the green road carpet to prevent any z-fighting
        centerline = Marker()
        centerline.header.stamp = path_msg.header.stamp
        centerline.header.frame_id = self.frame_id
        centerline.ns = "route_centerline"
        centerline.id = 0
        centerline.type = Marker.TRIANGLE_LIST
        centerline.action = Marker.ADD
        centerline.pose.orientation.w = 1.0
        centerline.scale.x = 1.0
        centerline.scale.y = 1.0
        centerline.scale.z = 1.0
        white_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        centerline.color = white_color

        half_cw = 0.18  # 36cm wide painted stripe
        z_center = 0.25

        c_left = []
        c_right = []
        for pt in trajectory:
            yaw = pt['yaw']
            nx = -math.sin(yaw) * half_cw
            ny = math.cos(yaw) * half_cw
            c_left.append(Point(x=pt['x'] + nx, y=pt['y'] + ny, z=z_center))
            c_right.append(Point(x=pt['x'] - nx, y=pt['y'] - ny, z=z_center))

        centerline.points = []
        centerline.colors = []
        for i in range(len(c_left) - 1):
            l1, r1 = c_left[i], c_right[i]
            l2, r2 = c_left[i + 1], c_right[i + 1]
            centerline.points.extend([r1, r2, l2, r1, l2, l1])
            centerline.colors.extend([white_color] * 6)

        self.centerline_pub.publish(centerline)
        self.current_centerline = centerline

        # Clear any legacy road_marker overlays
        self.current_route_marker = None
        self.current_centerline_marker = None

        # Publish status
        status_msg = String()
        status_msg.data = f"Active Route: {label} | Distance: {total_dist:.0f}m | Checkpoints: {len(checkpoints)} | {directions[1] if len(directions) > 1 else 'En route'}"
        self.status_pub.publish(status_msg)

    def publish_markers(self):
        now = self.get_clock().now().to_msg()

        # 1. Road graph lines — Fix 3: cached at startup, only update timestamp here
        self.cached_road_marker.header.stamp = now
        self.road_marker_pub.publish(self.cached_road_marker)

        # 2. Junction markers (bright orange spheres)
        junc_array = MarkerArray()
        for i, (jid, jinfo) in enumerate(self.campus.junctions.items()):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.ns = "junctions"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = jinfo['x']
            m.pose.position.y = jinfo['y']
            m.pose.position.z = 0.3
            diam = 5.0 if jinfo['degree'] == 4 else 3.5
            m.scale.x = diam
            m.scale.y = diam
            m.scale.z = diam
            m.color.r = 1.0
            m.color.g = 0.55
            m.color.b = 0.0
            m.color.a = 0.9
            junc_array.markers.append(m)

        self.junction_marker_pub.publish(junc_array)

        # 3. Stop markers (bright cyan/teal cylinders + high-visibility text labels)
        stop_array = MarkerArray()
        idx = 0
        for name, sinfo in self.campus.stops.items():
            is_start = (name == self.current_start_stop)
            is_goal = hasattr(self, 'current_goal_stop') and (name == self.current_goal_stop)

            # Cylinder marker
            sm = Marker()
            sm.header.stamp = now
            sm.header.frame_id = self.frame_id
            sm.ns = "campus_stops"
            sm.id = idx
            sm.type = Marker.CYLINDER
            sm.action = Marker.ADD
            sm.pose.position.x = sinfo['x']
            sm.pose.position.y = sinfo['y']
            sm.pose.position.z = 1.5 if (is_start or is_goal) else 1.2
            diam = 8.5 if (is_start or is_goal) else 6.0
            sm.scale.x = diam
            sm.scale.y = diam
            sm.scale.z = 3.0 if (is_start or is_goal) else 2.0

            if is_start:
                sm.color.r = 0.05
                sm.color.g = 1.0
                sm.color.b = 0.2
                sm.color.a = 1.0
            elif is_goal:
                sm.color.r = 1.0
                sm.color.g = 0.65
                sm.color.b = 0.0
                sm.color.a = 1.0
            else:
                sm.color.r = 0.0
                sm.color.g = 0.85
                sm.color.b = 0.95
                sm.color.a = 0.80

            stop_array.markers.append(sm)
            idx += 1

            # Text label
            tm = Marker()
            tm.header.stamp = now
            tm.header.frame_id = self.frame_id
            tm.ns = "stop_labels"
            tm.id = idx
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x = sinfo['x']
            tm.pose.position.y = sinfo['y']
            tm.pose.position.z = 5.5 if (is_start or is_goal) else 4.2
            tm.scale.z = 4.5 if (is_start or is_goal) else 3.2  # text height

            if is_start:
                tm.color.r = 0.2
                tm.color.g = 1.0
                tm.color.b = 0.3
                tm.color.a = 1.0
                tm.text = f"▶ START: {name}"
            elif is_goal:
                tm.color.r = 1.0
                tm.color.g = 0.8
                tm.color.b = 0.1
                tm.color.a = 1.0
                tm.text = f"🏁 DESTINATION: {name}"
            else:
                tm.color.r = 1.0
                tm.color.g = 1.0
                tm.color.b = 1.0
                tm.color.a = 0.85
                tm.text = name

            stop_array.markers.append(tm)
            idx += 1

        # Draw connector line from road node to destination stop if offset
        if hasattr(self, 'current_goal_stop') and self.current_goal_stop in self.campus.stops:
            sinfo = self.campus.stops[self.current_goal_stop]
            rx = sinfo.get('road_node_x', sinfo['x'])
            ry = sinfo.get('road_node_y', sinfo['y'])
            if math.hypot(rx - sinfo['x'], ry - sinfo['y']) > 1.0:
                conn = Marker()
                conn.header.stamp = now
                conn.header.frame_id = self.frame_id
                conn.ns = "campus_stops"
                conn.id = idx
                conn.type = Marker.LINE_STRIP
                conn.action = Marker.ADD
                conn.scale.x = 0.6
                conn.color.r = 1.0
                conn.color.g = 0.75
                conn.color.b = 0.0
                conn.color.a = 0.95
                conn.points.append(Point(x=rx, y=ry, z=0.3))
                conn.points.append(Point(x=sinfo['x'], y=sinfo['y'], z=1.5))
                stop_array.markers.append(conn)
                idx += 1

        self.stop_marker_pub.publish(stop_array)

        # 4. Building footprints & 3D outlines
        bldg_array = MarkerArray()
        for b_idx, (bname, bheight, bpts) in enumerate(self.campus.building_polygons):
            bm = Marker()
            bm.header.stamp = now
            bm.header.frame_id = self.frame_id
            bm.ns = "buildings"
            bm.id = b_idx
            bm.type = Marker.LINE_STRIP
            bm.action = Marker.ADD
            bm.scale.x = 1.2  # wall outline width
            bm.color.r = 0.35
            bm.color.g = 0.55
            bm.color.b = 0.8
            bm.color.a = 0.75
            for pt in bpts:
                bm.points.append(Point(x=pt[0], y=pt[1], z=0.1))
            if bpts:
                bm.points.append(Point(x=bpts[0][0], y=bpts[0][1], z=0.1))
            bldg_array.markers.append(bm)

        self.building_marker_pub.publish(bldg_array)

        # 5. Re-publish active route visuals continuously, or actively clear them if no route is active
        if self.current_ribbon is not None:
            self.current_ribbon.header.stamp = now
            self.route_ribbon_pub.publish(self.current_ribbon)
        else:
            del_marker = Marker()
            del_marker.header.stamp = now
            del_marker.header.frame_id = self.frame_id
            del_marker.ns = "route_ribbon"
            del_marker.id = 0
            del_marker.action = Marker.DELETEALL
            self.route_ribbon_pub.publish(del_marker)

        if self.current_centerline is not None:
            self.current_centerline.header.stamp = now
            self.centerline_pub.publish(self.current_centerline)
        else:
            del_marker = Marker()
            del_marker.header.stamp = now
            del_marker.header.frame_id = self.frame_id
            del_marker.ns = "route_centerline"
            del_marker.id = 0
            del_marker.action = Marker.DELETEALL
            self.centerline_pub.publish(del_marker)

        if self.current_path is not None:
            self.current_path.header.stamp = now
            self.path_pub.publish(self.current_path)
            self.nav2_plan_pub.publish(self.current_path)


def main(args=None):
    rclpy.init(args=args)
    node = CampusPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
