#!/usr/bin/env python3
"""
Audit Logger Node — Safety-Critical Blackbox Event & Decision Recorder

CAMP Autonomous Campus Shuttle (saye)
Package: campus_nav

Subscribes to:
  - /campus/safety_status (std_msgs/msg/String)
  - /campus/mission (std_msgs/msg/String)
  - /campus/goal_stop (std_msgs/msg/String)
  - /campus/global_path (nav_msgs/msg/Path)
  - /cloud (sensor_msgs/msg/PointCloud2)
  - /real_sense/image_raw (sensor_msgs/msg/Image)

Logs events to persistent audit file: ~/CAMP/log/audit_<YYYY-MM-DD_HH-MM-SS>.log
Format: [YYYY-MM-DD HH:MM:SS.mmm] [LEVEL] [CATEGORY] MESSAGE
"""

import atexit
from datetime import datetime
import json
import os
import signal
import sys
import threading
from typing import Optional, Tuple

from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String


class AuditLoggerNode(Node):
    """Safety-critical flight recorder node logging stack state and decisions."""

    def __init__(self):
        super().__init__('audit_logger_node')

        # ── Parameters ──────────────────────────────────────────────────────────
        default_log_dir = os.path.expanduser('~/CAMP/log')
        self.declare_parameter('log_dir', default_log_dir)
        self.declare_parameter('sensor_timeout_sec', 3.0)
        self.declare_parameter('watchdog_rate_hz', 10.0)

        self.log_dir = self.get_parameter('log_dir').value
        self.sensor_timeout_sec = float(self.get_parameter('sensor_timeout_sec').value)
        watchdog_rate = float(self.get_parameter('watchdog_rate_hz').value)

        # ── Ensure Log Directory & Open Session File ─────────────────────────────
        os.makedirs(self.log_dir, exist_ok=True)
        self.session_start = datetime.now()
        timestamp_str = self.session_start.strftime('%Y-%m-%d_%H-%M-%S')
        self.filename = f"audit_{timestamp_str}.log"
        self.filepath = os.path.join(self.log_dir, self.filename)

        self._file = open(self.filepath, 'a', encoding='utf-8')
        self._lock = threading.Lock()
        self._is_shutdown = False
        self.event_count = 0

        # Register cleanup hooks
        atexit.register(self.shutdown)

        # ── State Tracking ──────────────────────────────────────────────────────
        self.last_safety_state: Optional[str] = None
        self.last_path_sig: Optional[Tuple[int, float, float, float, float]] = None

        # Watchdog timing state (seconds from ROS clock)
        self._lidar_last_time: float = 0.0
        self._lidar_stale: bool = False
        self._camera_last_time: float = 0.0
        self._camera_stale: bool = False
        self._watchdog_initialized: bool = False

        # ── QoS Profiles ────────────────────────────────────────────────────────
        # Best effort sensor QoS matching LiDAR and camera publishers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Subscriptions ───────────────────────────────────────────────────────
        # 1. Safety Monitor
        self.safety_sub = self.create_subscription(
            String, '/campus/safety_status', self.safety_status_callback, 10
        )

        # 2. Mission Lifecycle & Operator Commands
        self.mission_sub = self.create_subscription(
            String, '/campus/mission', self.mission_callback, 10
        )
        self.goal_stop_sub = self.create_subscription(
            String, '/campus/goal_stop', self.goal_stop_callback, 10
        )

        # 3. Path Planning Decisions
        self.path_sub = self.create_subscription(
            Path, '/campus/global_path', self.global_path_callback, 10
        )

        # 4. Sensor Health Watchdog Topics
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/cloud', self.lidar_callback, sensor_qos
        )
        self.camera_sub = self.create_subscription(
            Image, '/real_sense/image_raw', self.camera_callback, sensor_qos
        )

        # 5. Periodic Watchdog Timer
        timer_period = 1.0 / max(1.0, watchdog_rate)
        self.watchdog_timer = self.create_timer(timer_period, self.watchdog_check)

        self.get_logger().info(f"AuditLoggerNode initialized. Writing to {self.filepath}")

    # ── Thread-Safe File Writing ──────────────────────────────────────────────

    def log_event(self, level: str, category: str, message: str) -> None:
        """Format and append an audit event line to disk in a thread-safe manner."""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        line = f"[{timestamp}] [{level}] [{category}] {message}\n"

        with self._lock:
            if self._is_shutdown:
                return
            self.event_count += 1
            self._file.write(line)
            self._file.flush()

        # Mirror to ROS logging console
        log_msg = f"[{category}] {message}"
        if level == 'CRITICAL':
            self.get_logger().error(log_msg)
        elif level == 'ERROR':
            self.get_logger().error(log_msg)
        elif level == 'WARN':
            self.get_logger().warn(log_msg)
        else:
            self.get_logger().info(log_msg)

    # ── Callbacks: A. Safety Monitor Events ────────────────────────────────────

    def safety_status_callback(self, msg: String) -> None:
        """Process safety status JSON updates and log state transitions."""
        raw_str = msg.data.strip()
        state = 'CLEAR'
        obs_dist = 999.0
        obs_type = 'NONE'

        try:
            payload = json.loads(raw_str)
            state = payload.get('state', 'CLEAR')
            obs_dist = float(payload.get('obstacle_dist', 999.0))
            obs_type = payload.get('obstacle_type', 'NONE')
        except Exception:
            state = raw_str

        # Only log on state transition
        if state == self.last_safety_state:
            return

        self.last_safety_state = state

        # Determine severity level
        if state == 'EMERGENCY_STOP':
            level = 'CRITICAL'
        elif state in ('SLOWDOWN_BUMP', 'SLOWDOWN_APPROACH', 'EDGE_WARNING'):
            level = 'WARN'
        elif state == 'CLEAR':
            level = 'INFO'
        else:
            level = 'WARN'

        # Format message with obstacle details when relevant
        if state == 'CLEAR' or obs_dist >= 900.0:
            msg_text = f"State: {state}"
        else:
            msg_text = f"State: {state} — obstacle at {obs_dist:.1f}m"

        self.log_event(level, 'SAFETY', msg_text)

    # ── Callbacks: B & D. Mission Lifecycle & Operator Commands ────────────────

    @staticmethod
    def _parse_route_name(raw: str) -> str:
        """Parse raw stop command string into readable 'Start → Goal' format."""
        cleaned = raw.strip()
        if '|' in cleaned:
            parts = cleaned.split('|', 1)
            return f"{parts[0].strip()} → {parts[1].strip()}"
        elif '->' in cleaned:
            parts = cleaned.split('->', 1)
            return f"{parts[0].strip()} → {parts[1].strip()}"
        elif '→' in cleaned:
            parts = cleaned.split('→', 1)
            return f"{parts[0].strip()} → {parts[1].strip()}"
        return cleaned

    def mission_callback(self, msg: String) -> None:
        """Log 2-leg mission commands dispatched by operators."""
        route_str = self._parse_route_name(msg.data)
        # Log as both operator command and mission lifecycle event
        self.log_event('INFO', 'OPERATOR', f"Operator command: Mission dispatched: {route_str}")
        self.log_event('INFO', 'MISSION', f"Mission dispatched: {route_str}")

    def goal_stop_callback(self, msg: String) -> None:
        """Log single-leg destination goal dispatched by operators."""
        goal_name = msg.data.strip()
        self.log_event('INFO', 'OPERATOR', f"Operator command: Single-leg goal dispatched: {goal_name}")
        self.log_event('INFO', 'MISSION', f"Single-leg goal dispatched: {goal_name}")

    # ── Callbacks: C. Path Planning Decisions ──────────────────────────────────

    def global_path_callback(self, msg: Path) -> None:
        """Log newly computed global paths with waypoint count and endpoints."""
        if not msg.poses:
            # Route cleared
            self.last_path_sig = None
            return

        n = len(msg.poses)
        first_pt = msg.poses[0].pose.position
        last_pt = msg.poses[-1].pose.position

        # Fingerprint path to avoid logging 2 Hz periodic republishes from planner_node
        sig = (n, round(first_pt.x, 2), round(first_pt.y, 2), round(last_pt.x, 2), round(last_pt.y, 2))
        if sig == self.last_path_sig:
            return

        self.last_path_sig = sig
        msg_text = (
            f"Global path computed: {n} waypoints, "
            f"({first_pt.x:.1f}, {first_pt.y:.1f}) → ({last_pt.x:.1f}, {last_pt.y:.1f})"
        )
        self.log_event('INFO', 'PLANNING', msg_text)

    # ── Callbacks: E. Sensor Health Watchdog ───────────────────────────────────

    def _get_ros_time_sec(self) -> float:
        """Get current ROS time in seconds."""
        now = self.get_clock().now()
        return now.nanoseconds / 1e9

    def lidar_callback(self, msg: PointCloud2) -> None:
        """Track LiDAR point cloud stream liveness and log recovery."""
        now = self._get_ros_time_sec()
        resumed = False
        with self._lock:
            if self._lidar_stale:
                self._lidar_stale = False
                resumed = True
            self._lidar_last_time = now

        if resumed:
            self.log_event('INFO', 'SENSOR', "LiDAR topic /cloud resumed")

    def camera_callback(self, msg: Image) -> None:
        """Track RealSense camera stream liveness and log recovery."""
        now = self._get_ros_time_sec()
        resumed = False
        with self._lock:
            if self._camera_stale:
                self._camera_stale = False
                resumed = True
            self._camera_last_time = now

        if resumed:
            self.log_event('INFO', 'SENSOR', "Camera topic /real_sense/image_raw resumed")

    def watchdog_check(self) -> None:
        """Periodic check for sensor silence exceeding timeout threshold."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns == 0:
            # When use_sim_time=True, clock starts at 0 until first /clock is received
            return

        now = now_ns / 1e9
        lidar_timed_out = False
        camera_timed_out = False

        with self._lock:
            if self._is_shutdown:
                return

            # Initialize baselines once clock is active
            if not self._watchdog_initialized:
                self._watchdog_initialized = True
                if self._lidar_last_time == 0.0:
                    self._lidar_last_time = now
                if self._camera_last_time == 0.0:
                    self._camera_last_time = now
                return

            if not self._lidar_stale:
                if (now - self._lidar_last_time) >= self.sensor_timeout_sec:
                    self._lidar_stale = True
                    lidar_timed_out = True

            if not self._camera_stale:
                if (now - self._camera_last_time) >= self.sensor_timeout_sec:
                    self._camera_stale = True
                    camera_timed_out = True

        # Log alerts outside lock
        if lidar_timed_out:
            self.log_event('WARN', 'SENSOR', f"LiDAR topic /cloud has not published for {self.sensor_timeout_sec:.1f}s")
        if camera_timed_out:
            self.log_event('WARN', 'SENSOR', f"Camera topic /real_sense/image_raw has not published for {self.sensor_timeout_sec:.1f}s")

    # ── Shutdown & Cleanup ────────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Idempotent shutdown: writes final session summary line and closes file."""
        with self._lock:
            if self._is_shutdown:
                return
            self._is_shutdown = True

            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            final_line = f"[{timestamp}] [INFO] [SESSION] Audit logger shut down. Total events: {self.event_count}\n"
            try:
                self._file.write(final_line)
                self._file.flush()
                self._file.close()
            except Exception as e:
                self.get_logger().warn(f"Error closing audit log: {e}")

        if rclpy.ok():
            try:
                self.get_logger().info(f"Audit logger shut down. Total events: {self.event_count}")
            except Exception:
                pass

    def destroy_node(self) -> None:
        """Ensure shutdown is called upon node destruction."""
        self.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AuditLoggerNode()

    def sig_handler(signum, frame):
        node.shutdown()
        sys.exit(0)

    try:
        signal.signal(signal.SIGTERM, sig_handler)
    except Exception:
        pass

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
