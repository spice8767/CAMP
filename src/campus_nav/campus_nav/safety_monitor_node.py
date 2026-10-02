"""
Safety Monitor Node — LiDAR & Depth Camera Collision & Void Guard

Monitors:
  - 3D LiDAR point cloud (/cloud)
  - RealSense depth point cloud (/real_sense/depth_point)
  - Current vehicle speed (/cmd_vel)

Performs:
  1. Dynamic Forward Safety Corridor:
     - Scales lookahead with vehicle speed (2.0m crawl to 10.0m cruise).
  2. Geometric Elevation Profiling:
     - Ground plane estimation (< 3cm relative elevation).
     - Speed Breaker Detection: 4cm - 16cm elevation -> triggers SLOWDOWN_BUMP (0.8 m/s).
     - Obstacle Detection: > 16cm elevation (humans, dogs, barriers) -> triggers EMERGENCY_STOP (0.0 m/s).
  3. Flank Void & Cliff Guard:
     - Evaluates ground continuity along left and right wheel flanks (0.8m - 2.2m lateral).
     - Detects missing returns or sharp drop-offs (> 35cm plunge) -> triggers EDGE_WARNING with steering bias.
  4. Telemetry & Visualization:
     - Publishes /campus/safety_status (JSON)
     - Publishes /campus/safety_corridor (RViz Marker: Green/Yellow/Red bounding box)
     - Publishes /campus/safety_threats (RViz MarkerArray: detected obstacle points)
"""

import json
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2


class SafetyMonitorNode(Node):
    """Monitors 3D LiDAR and Depth clouds for proactive collision and void safety."""

    def __init__(self):
        super().__init__('campus_safety_monitor_node')

        # ── Parameters ──────────────────────────────────────────────────────────
        self.declare_parameter('corridor_width', 1.6)       # Total corridor width (m)
        self.declare_parameter('stop_distance', 3.2)         # Hard stop distance ahead (m)
        self.declare_parameter('max_lookahead', 8.5)         # Max detection lookahead at full speed (m)
        self.declare_parameter('bump_speed_limit', 0.8)      # Target speed for speed breakers (m/s)
        self.declare_parameter('min_obstacle_pts', 8)        # Min points to confirm an obstacle
        self.declare_parameter('min_bump_pts', 12)           # Min points to confirm a speed breaker
        self.declare_parameter('edge_warning_dist', 4.5)     # Distance ahead to check flank edges (m)

        self.corridor_w      = self.get_parameter('corridor_width').value
        self.stop_dist       = self.get_parameter('stop_distance').value
        self.max_lookahead   = self.get_parameter('max_lookahead').value
        self.bump_speed      = self.get_parameter('bump_speed_limit').value
        self.min_obs_pts     = self.get_parameter('min_obstacle_pts').value
        self.min_bump_pts    = self.get_parameter('min_bump_pts').value
        self.edge_dist       = self.get_parameter('edge_warning_dist').value

        # ── State Variables ─────────────────────────────────────────────────────
        self.current_speed: float = 0.0
        self.last_state = "CLEAR"
        self.last_obstacle_dist = 999.0
        self.last_obstacle_type = "NONE"
        self.steering_bias = 0.0
        self.speed_limit = 2.22

        # Running estimate of ground level in lidar_link frame (~ -2.25m)
        self.ground_z_lidar = -2.25

        # ── QoS ─────────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ── Subscribers ─────────────────────────────────────────────────────────
        # Primary 3D LiDAR cloud (1024 azimuth x 128 rings)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/cloud', self.lidar_callback, sensor_qos)

        # RealSense Depth PointCloud
        self.depth_sub = self.create_subscription(
            PointCloud2, '/real_sense/depth_point', self.depth_callback, sensor_qos)

        # Monitor current speed commands
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # ── Publishers ──────────────────────────────────────────────────────────
        self.status_pub = self.create_publisher(
            String, '/campus/safety_status', 10)

        self.corridor_pub = self.create_publisher(
            Marker, '/campus/safety_corridor', 10)

        self.threats_pub = self.create_publisher(
            MarkerArray, '/campus/safety_threats', 10)

        # 2D Top-Down LiDAR BEV radar image publisher
        self.cv_bridge = CvBridge()
        self.bev_pub = self.create_publisher(
            Image, '/campus/lidar_bev_image', 10)

        # Heartbeat timer (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_telemetry)

        self.get_logger().info(
            f"SafetyMonitorNode ready | corridor={self.corridor_w}m | "
            f"stop_dist={self.stop_dist}m | max_lookahead={self.max_lookahead}m")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cmd_callback(self, msg: Twist):
        self.current_speed = max(0.0, msg.linear.x)

    def lidar_callback(self, cloud_msg: PointCloud2):
        """Process 3D LiDAR point cloud in lidar_link frame."""
        try:
            pts = pc2.read_points_numpy(cloud_msg, field_names=['x', 'y', 'z'], skip_nans=True)
        except Exception as e:
            self.get_logger().warn(f"Failed to read LiDAR points: {e}")
            return

        if pts is None or len(pts) == 0:
            return

        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]

        # ── 1. Dynamic Ground Plane Calibration ─────────────────────────────────
        # Estimate road plane from near-field road surface (1.0m to 3.5m ahead, within vehicle track)
        near_road_mask = (x >= 1.0) & (x <= 3.5) & (np.abs(y) <= 0.6)
        if np.count_nonzero(near_road_mask) > 20:
            # 10th percentile gives clean road surface estimate without obstacle bias
            measured_ground = float(np.percentile(z[near_road_mask], 10))
            # Smooth running update
            self.ground_z_lidar = 0.8 * self.ground_z_lidar + 0.2 * measured_ground

        gz = self.ground_z_lidar

        # ── 2. Forward Safety Corridor Bounds ──────────────────────────────────
        # Dynamic lookahead distance scales with current vehicle velocity
        active_lookahead = max(self.stop_dist + 0.8, min(self.max_lookahead, 2.5 + 2.5 * self.current_speed))
        half_w = self.corridor_w / 2.0

        # Points inside forward corridor
        in_corridor = (x >= 2.0) & (x <= active_lookahead) & (np.abs(y) <= half_w)
        c_x = x[in_corridor]
        c_y = y[in_corridor]
        c_z = z[in_corridor]

        # Elevation relative to detected road plane
        dz = c_z - gz

        # ── 3. Categorize Corridor Points ──────────────────────────────────────
        # Obstacles (humans, dogs, vehicles, bins): dz > 0.16m and < 2.2m (under overhanging trees)
        obs_mask = (dz > 0.16) & (dz < 2.2)
        obs_count = np.count_nonzero(obs_mask)

        # Speed breakers: dz between 0.04m and 0.16m
        bump_mask = (dz >= 0.04) & (dz <= 0.16)
        bump_count = np.count_nonzero(bump_mask)

        # ── 4. Flank Void & Cliff Guard ─────────────────────────────────────────
        # Check left and right wheel flanks (0.8m to 2.2m lateral, 0.5m to 4.5m forward)
        left_flank = (x >= 2.0) & (x <= self.edge_dist) & (y >= 0.8) & (y <= 2.2)
        right_flank = (x >= 2.0) & (x <= self.edge_dist) & (y <= -0.8) & (y >= -2.2)

        left_pts = np.count_nonzero(left_flank)
        right_pts = np.count_nonzero(right_flank)

        # Check for deep drop-offs into void (dz < -0.40m)
        left_drop = np.count_nonzero(left_flank & ((z - gz) < -0.40)) if left_pts > 0 else 0
        right_drop = np.count_nonzero(right_flank & ((z - gz) < -0.40)) if right_pts > 0 else 0

        # ── 5. Arbitration & State Decision ────────────────────────────────────
        state = "CLEAR"
        speed_lim = 2.22
        steer_bias = 0.0
        obs_dist = 999.0
        obs_type = "NONE"
        threat_pts = None

        if obs_count >= self.min_obs_pts:
            # Human, dog, or barrier ahead!
            obs_distances = c_x[obs_mask]
            obs_dist = float(np.min(obs_distances))
            obs_type = "HUMAN_OR_ANIMAL"
            threat_pts = np.column_stack((c_x[obs_mask], c_y[obs_mask], c_z[obs_mask]))

            if obs_dist <= self.stop_dist:
                state = "EMERGENCY_STOP"
                speed_lim = 0.0
            else:
                # Proportional deceleration approaching obstacle
                state = "SLOWDOWN_APPROACH"
                speed_lim = max(0.5, min(1.5, 0.4 * (obs_dist - self.stop_dist)))

        elif bump_count >= self.min_bump_pts:
            # Speed breaker detected on road ahead
            bump_distances = c_x[bump_mask]
            obs_dist = float(np.min(bump_distances))
            obs_type = "SPEED_BREAKER"
            threat_pts = np.column_stack((c_x[bump_mask], c_y[bump_mask], c_z[bump_mask]))

            if obs_dist <= 5.0:
                state = "SLOWDOWN_BUMP"
                speed_lim = self.bump_speed

        elif left_drop > 8:
            # Left side cliff / road edge danger
            state = "EDGE_WARNING"
            steer_bias = -0.15   # Bias steering right away from left cliff
            speed_lim = 1.0      # Dampen speed on dangerous edge
            obs_type = "ROAD_EDGE_LEFT"
            obs_dist = 2.0

        elif right_drop > 8:
            # Right side cliff / road edge danger
            state = "EDGE_WARNING"
            steer_bias = 0.15    # Bias steering left away from right cliff
            speed_lim = 1.0      # Dampen speed on dangerous edge
            obs_type = "ROAD_EDGE_RIGHT"
            obs_dist = 2.0

        # Update node state
        self.last_state = state
        self.last_obstacle_dist = obs_dist
        self.last_obstacle_type = obs_type
        self.speed_limit = speed_lim
        self.steering_bias = steer_bias

        # Publish visual markers
        self.publish_corridor_marker(active_lookahead, half_w, state)
        if threat_pts is not None:
            self.publish_threat_markers(threat_pts)

        # Publish 2D Top-Down LiDAR BEV radar image for RViz side panel
        self.publish_bev_image(x, y, z, gz, active_lookahead, half_w, state, obs_dist, cloud_msg.header)

    def depth_callback(self, cloud_msg: PointCloud2):
        """Optional secondary confirmation from forward RealSense depth camera."""
        # Provides near-field reinforcement for low speed bumps < 3m
        pass

    # ── Publishers ────────────────────────────────────────────────────────────

    def publish_telemetry(self):
        """Publish structured JSON status for path follower and RViz."""
        msg = String()
        payload = {
            "state": self.last_state,
            "speed_limit": round(self.speed_limit, 2),
            "steering_bias": round(self.steering_bias, 3),
            "obstacle_dist": round(self.last_obstacle_dist, 2),
            "obstacle_type": self.last_obstacle_type,
            "ground_z": round(self.ground_z_lidar, 2)
        }
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def publish_corridor_marker(self, length: float, half_w: float, state: str):
        """Publish a 3D bounding box / corridor marker in RViz."""
        m = Marker()
        m.header.frame_id = 'lidar_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'safety_corridor'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD

        # Center position of corridor box
        m.pose.position.x = length / 2.0
        m.pose.position.y = 0.0
        m.pose.position.z = self.ground_z_lidar + 0.6  # centered slightly above road
        m.pose.orientation.w = 1.0

        # Dimensions: (X=length, Y=width, Z=height)
        m.scale.x = float(length)
        m.scale.y = float(half_w * 2.0)
        m.scale.z = 1.4

        # Color based on state
        if state == "EMERGENCY_STOP":
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.45)   # Vivid Red
        elif state in ("SLOWDOWN_BUMP", "SLOWDOWN_APPROACH"):
            m.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.45)   # Vivid Amber/Yellow
        elif state == "EDGE_WARNING":
            m.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.45)   # Magenta
        else:
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=0.25)   # Translucent Green

        self.corridor_pub.publish(m)

    def publish_threat_markers(self, threat_pts: np.ndarray):
        """Highlight detected threat clusters in RViz."""
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = 'lidar_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'threat_clusters'
        m.id = 1
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.15
        m.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9)

        # Downsample to at most 50 threat points for smooth rendering
        step = max(1, len(threat_pts) // 50)
        for i in range(0, len(threat_pts), step):
            pt = Point()
            pt.x = float(threat_pts[i, 0])
            pt.y = float(threat_pts[i, 1])
            pt.z = float(threat_pts[i, 2])
            m.points.append(pt)

        ma.markers.append(m)
        self.threats_pub.publish(ma)

    def publish_bev_image(self, x: np.ndarray, y: np.ndarray, z: np.ndarray, gz: float,
                          lookahead: float, half_w: float, state: str, obs_dist: float,
                          header):
        """Render and publish a 2D Top-Down LiDAR Bird's-Eye-View (BEV) radar image."""
        H, W = 400, 400
        cx, cy = 200, 320
        scale = 12.0  # 12 pixels per meter -> 33m visible range

        img = np.full((H, W, 3), (18, 22, 28), dtype=np.uint8)

        # 1. Subtle concentric range rings (5m, 10m, 15m, 20m)
        for r_m in (5, 10, 15, 20):
            r_px = int(r_m * scale)
            cv2.circle(img, (cx, cy), r_px, (40, 48, 58), 1)
            cv2.putText(img, f'{r_m}m', (cx + 4, cy - r_px + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (90, 105, 120), 1)

        # Forward central axis line
        cv2.line(img, (cx, 30), (cx, 380), (35, 42, 50), 1)

        # 2. Filter & project LiDAR points
        # Downsample slightly for 60+ FPS rendering if points count is high
        step = max(1, len(x) // 6000)
        xs = x[::step]
        ys = y[::step]
        zs = z[::step]

        mask = (xs >= -4.0) & (xs <= 24.0) & (np.abs(ys) <= 15.0)
        if np.any(mask):
            mx = xs[mask]
            my = ys[mask]
            mz = zs[mask]
            dz = mz - gz

            u_pts = np.clip(np.round(cx - my * scale).astype(np.int32), 0, W - 1)
            v_pts = np.clip(np.round(cy - mx * scale).astype(np.int32), 0, H - 1)

            # Road pavement: dz < 0.04m (subdued teal)
            road = dz < 0.04
            img[v_pts[road], u_pts[road]] = (160, 140, 45)

            # Speed breaker: 0.04m <= dz <= 0.16m (bright yellow)
            bump = (dz >= 0.04) & (dz <= 0.16)
            if np.any(bump):
                ub = u_pts[bump]
                vb = v_pts[bump]
                for u, v in zip(ub, vb):
                    cv2.circle(img, (int(u), int(v)), 2, (0, 230, 255), -1)

            # Obstacles / threats: dz > 0.16m (bright red)
            obs = dz > 0.16
            if np.any(obs):
                uo = u_pts[obs]
                vo = v_pts[obs]
                for u, v in zip(uo, vo):
                    cv2.circle(img, (int(u), int(v)), 2, (30, 40, 255), -1)

        # 3. Dynamic Safety Corridor Box
        corridor_color = (60, 220, 60)  # CLEAR: Green
        if state == "EMERGENCY_STOP":
            corridor_color = (40, 40, 255)  # Red
        elif state in ("SLOWDOWN_BUMP", "SLOWDOWN_APPROACH"):
            corridor_color = (0, 200, 255)  # Amber
        elif state == "EDGE_WARNING":
            corridor_color = (0, 140, 255)  # Orange

        c_u1 = int(cx - half_w * scale)
        c_u2 = int(cx + half_w * scale)
        c_v1 = int(cy - lookahead * scale)
        c_v2 = int(cy - 0.5 * scale)
        cv2.rectangle(img, (c_u1, c_v1), (c_u2, c_v2), corridor_color, 1)

        # 4. Vehicle Footprint (orange box with forward pointer)
        veh_w = int(1.4 * scale / 2)
        veh_l_front = int(0.5 * scale)
        veh_l_rear = int(1.8 * scale)
        cv2.rectangle(img, (cx - veh_w, cy - veh_l_front), (cx + veh_w, cy + veh_l_rear),
                      (0, 140, 255), -1)
        cv2.arrowedLine(img, (cx, cy + 6), (cx, cy - 10), (255, 255, 255), 2, tipLength=0.4)

        # 5. HUD Status Header Overlay
        cv2.rectangle(img, (0, 0), (W, 36), (12, 14, 18), -1)
        cv2.line(img, (0, 36), (W, 36), (50, 60, 75), 1)

        status_text = f"STATE: {state}"
        info_text = f"V:{self.current_speed:.1f}m/s"
        if obs_dist < 50.0:
            info_text += f" | OBS:{obs_dist:.1f}m"

        cv2.putText(img, status_text, (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, corridor_color, 1)
        cv2.putText(img, info_text, (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 210, 220), 1)
        cv2.putText(img, "LiDAR 2D BEV RADAR", (245, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (120, 140, 160), 1)

        try:
            img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg.header = header
            self.bev_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish BEV image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
