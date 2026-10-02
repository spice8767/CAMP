#!/usr/bin/env python3
"""
CAMP Autonomous Buggy — Automated Test Suite
============================================
Standalone test runner for the CAMP ROS 2 / Gazebo Harmonic autonomous campus
shuttle stack. Launches the full simulation, waits for boot, executes 8 test
cases sequentially, screenshots at key moments, and generates a rich HTML report.

Usage:
    cd ~/CAMP
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 test_suite/run_tests.py

Requirements: CAMP workspace must be built (colcon build --symlink-install).
"""

# ── Standard library ─────────────────────────────────────────────────────────
import base64
import datetime
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

SUITE_DIR    = Path(__file__).parent.resolve()
SS_DIR       = SUITE_DIR / "screenshots"
REPORTS_DIR  = SUITE_DIR / "reports"
CAMP_ROOT    = SUITE_DIR.parent
DISPLAY_ENV  = ":1"

# Map frame origin offset (Gazebo → map coords):  nav = gz + OFFSET
MAP_OFFSET_X = 802.0
MAP_OFFSET_Y = 679.7

# Spawn position in Gazebo world (SAB C)
SPAWN_GZ_X   = 15.02
SPAWN_GZ_Y   = 43.00
SPAWN_GZ_Z   = 0.80

# Computed nav-frame spawn position
SPAWN_NAV_X  = SPAWN_GZ_X + MAP_OFFSET_X   # ≈ 817.0
SPAWN_NAV_Y  = SPAWN_GZ_Y + MAP_OFFSET_Y   # ≈ 722.7

# Campus bounds in nav-frame (from YAML; generous margin around known stops)
NAV_X_MIN, NAV_X_MAX = 0.0,   1500.0
NAV_Y_MIN, NAV_Y_MAX = 0.0,   1300.0

# Minimum Z above ground to prove the buggy is not underground.
# Gazebo physics settles the buggy at ~0.28m shortly after spawn (wheel contact
# point on the 0.30m-high road mesh). Accept anything > 0.10m as "on the road".
MIN_SPAWN_Z  = 0.10

# Required ROS 2 nodes
REQUIRED_NODES = [
    "/campus_planner_node",
    "/path_follower_node",
    "/safety_monitor_node",
    "/odom_bridge_node",
    "/mission_controller_node",
    "/audit_logger",
    "/rviz2",
]

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)


# ═══════════════════════════════════════════════════════════════════════════════
#  RESULT DATACLASS
# ═══════════════════════════════════════════════════════════════════════════════

class TestResult:
    """Holds outcome + metadata for a single test."""
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"

    def __init__(self, name: str, description: str):
        self.name        = name
        self.description = description
        self.status      = self.SKIP
        self.message     = "Not executed"
        self.screenshots: List[str] = []
        self.start_time  = time.time()
        self.duration    = 0.0
        self.details: List[str] = []

    def mark(self, status: str, message: str):
        self.status   = status
        self.message  = message
        self.duration = time.time() - self.start_time

    def add_detail(self, msg: str):
        self.details.append(msg)
        print(f"    {msg}")


# ═══════════════════════════════════════════════════════════════════════════════
#  SCREENSHOT HELPER
# ═══════════════════════════════════════════════════════════════════════════════

def take_screenshot(filename: str) -> Optional[str]:
    """
    Capture the virtual display to <SS_DIR>/<filename>.
    Tries scrot first, then ImageMagick import as fallback.
    Returns the absolute path on success, None on failure.
    """
    SS_DIR.mkdir(parents=True, exist_ok=True)
    dest = str(SS_DIR / filename)
    env  = {**os.environ, "DISPLAY": DISPLAY_ENV}

    # Try scrot
    if shutil.which("scrot"):
        r = subprocess.run(["scrot", "-d", "1", dest], env=env,
                           capture_output=True, text=True)
        if r.returncode == 0 and Path(dest).exists():
            print(f"  📸 Screenshot saved: {filename}")
            return dest
        print(f"  ⚠️  scrot failed ({r.stderr.strip()}) — trying import")

    # Fallback: ImageMagick import
    if shutil.which("import"):
        r = subprocess.run(["import", "-window", "root", dest], env=env,
                           capture_output=True, text=True)
        if r.returncode == 0 and Path(dest).exists():
            print(f"  📸 Screenshot saved via import: {filename}")
            return dest
        print(f"  ⚠️  import failed: {r.stderr.strip()}")

    print(f"  ⚠️  No screenshot tool available — skipping {filename}")
    return None


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS 2 HELPER UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════

def spin_for(node: rclpy.node.Node, seconds: float, step: float = 0.05):
    """Non-blocking spin for <seconds>, checking every <step> seconds."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=step)


def wait_for_message(node: rclpy.node.Node,
                     msg_type,
                     topic: str,
                     timeout: float,
                     qos=10) -> Optional[Any]:
    """
    Subscribe to <topic>, spin until one message arrives or timeout.
    Returns the message or None on timeout.
    """
    received = [None]

    def cb(msg):
        received[0] = msg

    sub = node.create_subscription(msg_type, topic, cb, qos)
    deadline = time.time() + timeout
    while time.time() < deadline and received[0] is None:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_subscription(sub)
    return received[0]


def collect_messages(node: rclpy.node.Node,
                     msg_type,
                     topic: str,
                     count: int,
                     interval: float,
                     qos=10) -> List[Any]:
    """
    Collect <count> messages from <topic>, one per <interval> seconds.
    Each sample triggers after the previous interval, or takes whatever arrives.
    """
    msgs = []
    for _ in range(count):
        m = wait_for_message(node, msg_type, topic, interval * 2, qos)
        if m is not None:
            msgs.append(m)
        spin_for(node, interval)
    return msgs


def get_ros_node_list() -> List[str]:
    """Run `ros2 node list` and return the list of node names."""
    r = subprocess.run(["ros2", "node", "list"],
                       capture_output=True, text=True, timeout=10)
    return [line.strip() for line in r.stdout.splitlines() if line.strip()]


# ═══════════════════════════════════════════════════════════════════════════════
#  SIMULATION LAUNCHER
# ═══════════════════════════════════════════════════════════════════════════════

def build_launch_env() -> Dict[str, str]:
    """
    Source both ROS 2 and CAMP workspace setups, collect the resulting
    environment variables, and merge with current env.
    """
    print("  🔧 Collecting ROS 2 + CAMP workspace environment …")
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {CAMP_ROOT}/install/setup.bash && "
        "env"
    )
    result = subprocess.run(
        ["bash", "-c", cmd],
        capture_output=True, text=True, timeout=30
    )
    env: Dict[str, str] = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            env[k] = v

    # Merge: start from current env, overlay sourced vars
    merged = {**os.environ, **env}
    merged["DISPLAY"] = DISPLAY_ENV
    return merged


def launch_simulation(env: Dict[str, str]) -> subprocess.Popen:
    """Start the full simulation as a background subprocess."""
    print("  🚀 Launching full_simulation.launch.py …")
    proc = subprocess.Popen(
        ["ros2", "launch", "campus_nav", "full_simulation.launch.py"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid   # create process group for clean teardown
    )
    print(f"  🔢 Simulation PID: {proc.pid}")
    return proc


def shutdown_simulation(proc: subprocess.Popen):
    """Gracefully stop the simulation; SIGKILL if needed."""
    if proc.poll() is not None:
        return
    print("\n  🛑 Sending SIGINT to simulation process group …")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass

    try:
        proc.wait(timeout=15)
        print("  ✅ Simulation exited cleanly.")
    except subprocess.TimeoutExpired:
        print("  ⚠️  Timeout — sending SIGKILL …")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()
        print("  🔴 Simulation killed.")


# ═══════════════════════════════════════════════════════════════════════════════
#  INDIVIDUAL TESTS
# ═══════════════════════════════════════════════════════════════════════════════

def test_1_launch_check(node: rclpy.node.Node) -> TestResult:
    """TEST 1 — Sim + RViz Launch Check (wait up to 60s for 7 nodes)."""
    result = TestResult(
        "T1 — Sim + RViz Launch Check",
        "Wait ≤60s for all 7 required ROS nodes to appear in ros2 node list."
    )
    print(f"\n{'='*60}")
    print(f"▶  {result.name}")
    print(f"{'='*60}")

    deadline = time.time() + 60.0
    found_nodes: List[str] = []

    while time.time() < deadline:
        node_list = get_ros_node_list()
        found_nodes = [n for n in REQUIRED_NODES if n in node_list]
        if len(found_nodes) == len(REQUIRED_NODES):
            break
        missing = [n for n in REQUIRED_NODES if n not in node_list]
        result.add_detail(f"Waiting … missing: {missing}")
        spin_for(node, 3.0)

    # Screenshot after polling finishes
    ss = take_screenshot("test1_launch.png")
    if ss:
        result.screenshots.append(ss)

    missing = [n for n in REQUIRED_NODES if n not in found_nodes]
    if not missing:
        result.add_detail(f"All {len(REQUIRED_NODES)} nodes confirmed running.")
        result.mark(TestResult.PASS, f"All {len(REQUIRED_NODES)} required nodes are up.")
    else:
        result.add_detail(f"Missing nodes after 60s: {missing}")
        result.mark(TestResult.FAIL,
                    f"Timeout after 60s — missing: {', '.join(missing)}")

    return result


def test_2_buggy_spawn(node: rclpy.node.Node) -> TestResult:
    """TEST 2 — Buggy Spawn Check (pose from /model/saye/odometry_world)."""
    result = TestResult(
        "T2 — Buggy Spawn Check",
        "Subscribe to /model/saye/odometry_world; verify z > 0.3 m and position within campus bounds."
    )
    print(f"\n{'='*60}")
    print(f"▶  {result.name}")
    print(f"{'='*60}")

    result.add_detail("Waiting up to 20s for Odometry on /model/saye/odometry_world …")
    msg = wait_for_message(node, Odometry,
                           "/model/saye/odometry_world",
                           timeout=20.0)

    ss = take_screenshot("test2_spawn.png")
    if ss:
        result.screenshots.append(ss)

    if msg is None:
        result.mark(TestResult.FAIL, "Timeout after 20s — no pose received on /model/saye/odometry_world")
        return result

    gz_x = msg.pose.pose.position.x
    gz_y = msg.pose.pose.position.y
    gz_z = msg.pose.pose.position.z

    # Convert to nav-frame for bounds check
    nav_x = gz_x + MAP_OFFSET_X
    nav_y = gz_y + MAP_OFFSET_Y

    result.add_detail(f"Gazebo pose: x={gz_x:.2f}  y={gz_y:.2f}  z={gz_z:.2f}")
    result.add_detail(f"Nav-frame:   x={nav_x:.2f}  y={nav_y:.2f}")

    if gz_z <= 0.0:
        result.mark(TestResult.FAIL, f"Buggy is underground: z={gz_z:.3f} m")
        return result
    if gz_z < MIN_SPAWN_Z:
        result.mark(TestResult.FAIL,
                    f"Buggy z={gz_z:.3f} m < threshold {MIN_SPAWN_Z} m (too low)")
        return result
    if not (NAV_X_MIN <= nav_x <= NAV_X_MAX and NAV_Y_MIN <= nav_y <= NAV_Y_MAX):
        result.mark(TestResult.FAIL,
                    f"Nav-frame position ({nav_x:.1f}, {nav_y:.1f}) out of campus bounds.")
        return result

    result.mark(TestResult.PASS,
                f"Buggy spawned at gz=({gz_x:.1f},{gz_y:.1f},{gz_z:.2f}) "
                f"→ nav=({nav_x:.1f},{nav_y:.1f})  z OK")
    return result


def test_3_sensor_feeds(node: rclpy.node.Node) -> TestResult:
    """TEST 3 — Sensor Feed Check (LiDAR PointCloud2 + RealSense camera)."""
    result = TestResult(
        "T3 — Sensor Feed Check",
        "Wait ≤15s for at least 1 message from /cloud (LiDAR) and /real_sense/image_raw (camera)."
    )
    print(f"\n{'='*60}")
    print(f"▶  {result.name}")
    print(f"{'='*60}")

    result.add_detail("Waiting for /cloud (LiDAR PointCloud2) …")
    lidar_msg = wait_for_message(node, PointCloud2, "/cloud",
                                 timeout=15.0, qos=SENSOR_QOS)
    result.add_detail(f"  /cloud: {'✅ received' if lidar_msg else '❌ timeout'}")

    result.add_detail("Waiting for /real_sense/image_raw (camera Image) …")
    camera_msg = wait_for_message(node, Image, "/real_sense/image_raw",
                                  timeout=15.0, qos=SENSOR_QOS)
    result.add_detail(f"  /real_sense/image_raw: {'✅ received' if camera_msg else '❌ timeout'}")

    ss = take_screenshot("test3_sensors.png")
    if ss:
        result.screenshots.append(ss)

    if lidar_msg and camera_msg:
        pts = lidar_msg.width * lidar_msg.height
        result.mark(TestResult.PASS,
                    f"Both sensors active — LiDAR ({pts} pts), camera "
                    f"({camera_msg.width}×{camera_msg.height})")
    elif not lidar_msg and not camera_msg:
        result.mark(TestResult.FAIL, "Timeout after 15s — neither /cloud nor /real_sense/image_raw received")
    elif not lidar_msg:
        result.mark(TestResult.FAIL, "Timeout after 15s — /cloud silent (LiDAR not publishing)")
    else:
        result.mark(TestResult.FAIL, "Timeout after 15s — /real_sense/image_raw silent (camera not publishing)")

    return result


def test_4_pose_drift(node: rclpy.node.Node) -> TestResult:
    """TEST 4 — Pose Sync / Drift Check (10 samples over 5s while stationary)."""
    result = TestResult(
        "T4 — Pose Sync / Drift Check",
        "Sample buggy pose 10× over 5s (no mission dispatched). Max deviation must be < 0.05 m."
    )
    print(f"\n{'='*60}")
    print(f"▶  {result.name}")
    print(f"{'='*60}")

    result.add_detail("Collecting 10 pose samples from /campus/vehicle_pose …")
    xs: List[float] = []
    ys: List[float] = []

    for i in range(10):
        msg = wait_for_message(node, PoseStamped, "/campus/vehicle_pose",
                               timeout=3.0)
        if msg is not None:
            xs.append(msg.pose.position.x)
            ys.append(msg.pose.position.y)
            result.add_detail(f"  Sample {i+1:02d}: x={msg.pose.position.x:.4f}  y={msg.pose.position.y:.4f}")
        else:
            result.add_detail(f"  Sample {i+1:02d}: ⚠️  No message received")
        spin_for(node, 0.5)

    ss = take_screenshot("test4_drift.png")
    if ss:
        result.screenshots.append(ss)

    if len(xs) < 3:
        result.mark(TestResult.FAIL,
                    f"Only {len(xs)}/10 pose samples received — /campus/vehicle_pose not publishing.")
        return result

    x_dev = max(xs) - min(xs)
    y_dev = max(ys) - min(ys)
    result.add_detail(f"Max X deviation: {x_dev*100:.1f} cm | Max Y deviation: {y_dev*100:.1f} cm")

    if x_dev < 0.05 and y_dev < 0.05:
        result.mark(TestResult.PASS,
                    f"Pose stable — ΔX={x_dev*100:.1f}cm  ΔY={y_dev*100:.1f}cm (both < 5cm)")
    else:
        result.mark(TestResult.FAIL,
                    f"Pose unstable — ΔX={x_dev*100:.1f}cm  ΔY={y_dev*100:.1f}cm (threshold 5cm)")

    return result


def test_5_safety_baseline(node: rclpy.node.Node) -> TestResult:
    """TEST 5 — Safety Monitor Baseline (expect state == 'CLEAR' with no obstacles)."""
    result = TestResult(
        "T5 — Safety Monitor Baseline",
        "Subscribe to /campus/safety_status; parse JSON; expect state == 'CLEAR'."
    )
    print(f"\n{'='*60}")
    print(f"▶  {result.name}")
    print(f"{'='*60}")

    result.add_detail("Waiting up to 10s for /campus/safety_status …")
    msg = wait_for_message(node, String, "/campus/safety_status", timeout=10.0)

    ss = take_screenshot("test5_safety.png")
    if ss:
        result.screenshots.append(ss)

    if msg is None:
        result.mark(TestResult.FAIL,
                    "Timeout after 10s — no message on /campus/safety_status")
        return result

    try:
        payload = json.loads(msg.data)
    except json.JSONDecodeError as e:
        result.mark(TestResult.FAIL, f"JSON parse error: {e} | raw='{msg.data[:120]}'")
        return result

    state       = payload.get("state", "UNKNOWN")
    speed_limit = payload.get("speed_limit", "?")
    obs_dist    = payload.get("obstacle_dist", "?")
    obs_type    = payload.get("obstacle_type", "?")

    result.add_detail(
        f"state={state}  speed_limit={speed_limit}  "
        f"obs_dist={obs_dist}  obs_type={obs_type}"
    )

    if state == "CLEAR":
        result.mark(TestResult.PASS,
                    f"Safety state is CLEAR — corridor open, speed_limit={speed_limit} m/s")
    else:
        result.mark(TestResult.FAIL,
                    f"Expected CLEAR but got '{state}' (obs_dist={obs_dist}, type={obs_type})")

    return result


def test_6_mission_execution(node: rclpy.node.Node) -> TestResult:
    """TEST 6 — Single Mission Execution ('SAB C|Chemistry Block')."""
    result = TestResult(
        "T6 — Single Mission Execution",
        "Dispatch 'SAB C|Chemistry Block'; expect global path within 10s and buggy moves > 5m in 90s."
    )
    print(f"\n{'='*60}")
    print(f"▶  {result.name}")
    print(f"{'='*60}")

    # Record start pose
    result.add_detail("Recording start pose from /campus/vehicle_pose …")
    start_msg = wait_for_message(node, PoseStamped, "/campus/vehicle_pose", timeout=5.0)
    if start_msg is None:
        result.add_detail("  ⚠️  No start pose — assuming spawn position")
        start_x, start_y = SPAWN_NAV_X, SPAWN_NAV_Y
    else:
        start_x = start_msg.pose.position.x
        start_y = start_msg.pose.position.y
    result.add_detail(f"Start nav-frame: ({start_x:.2f}, {start_y:.2f})")

    # Publish mission
    mission_pub = node.create_publisher(String, "/campus/mission", 10)
    spin_for(node, 0.5)   # give publisher time to advertise

    mission_msg = String()
    mission_msg.data = "SAB C|Chemistry Block"
    mission_pub.publish(mission_msg)
    result.add_detail("Published mission: 'SAB C|Chemistry Block'")

    # Screenshot at dispatch
    ss_start = take_screenshot("test6_mission_start.png")
    if ss_start:
        result.screenshots.append(ss_start)

    # Wait for global path (up to 10s)
    result.add_detail("Waiting ≤10s for /campus/global_path …")
    path_msg = wait_for_message(node, NavPath, "/campus/global_path", timeout=10.0)
    path_ok = path_msg is not None
    if path_ok:
        result.add_detail(f"  ✅ Global path received ({len(path_msg.poses)} waypoints)")
    else:
        result.add_detail("  ❌ /campus/global_path not received within 10s")

    # Wait for movement (up to 90s, poll every 5s)
    result.add_detail("Monitoring buggy movement for up to 90s …")
    moved = False
    deadline = time.time() + 90.0
    mid_screenshot_taken = False

    while time.time() < deadline:
        spin_for(node, 5.0)

        if not mid_screenshot_taken and (deadline - time.time()) < 45.0:
            ss_mid = take_screenshot("test6_mission_mid.png")
            if ss_mid:
                result.screenshots.append(ss_mid)
            mid_screenshot_taken = True

        pose_msg = wait_for_message(node, PoseStamped, "/campus/vehicle_pose", timeout=2.0)
        if pose_msg:
            cx = pose_msg.pose.position.x
            cy = pose_msg.pose.position.y
            dist = math.sqrt((cx - start_x)**2 + (cy - start_y)**2)
            result.add_detail(f"  Distance from start: {dist:.2f} m")
            if dist > 5.0:
                moved = True
                result.add_detail(f"  ✅ Buggy moved {dist:.2f} m — mission active!")
                break

    # Ensure mid-mission screenshot taken
    if not mid_screenshot_taken:
        ss_mid = take_screenshot("test6_mission_mid.png")
        if ss_mid:
            result.screenshots.append(ss_mid)

    node.destroy_publisher(mission_pub)

    if path_ok and moved:
        result.mark(TestResult.PASS,
                    "Global path published AND buggy moved > 5m from spawn — mission executing.")
    elif not path_ok:
        result.mark(TestResult.FAIL,
                    "No global path published within 10s of mission dispatch.")
    else:
        result.mark(TestResult.FAIL,
                    "Global path received but buggy did not move > 5m in 90s.")

    return result


def spawn_obstacle_gz(obstacle_type: str = "human", distance: float = 5.0) -> bool:
    """
    Spawn an obstacle in Gazebo Harmonic by calling the gz EntityFactory service
    directly (mirrors test_obstacle_spawner.py logic), placing it 5m ahead of SAB C.
    Returns True if the spawn command reported success.
    """
    # SAB C heading is +Y → spawn ahead
    target_x = SPAWN_GZ_X
    target_y = SPAWN_GZ_Y + distance

    if obstacle_type == "human":
        sdf = (
            "<?xml version='1.0'?>"
            "<sdf version='1.8'>"
            "<model name='test_obstacle_human'>"
            "<static>true</static>"
            "<link name='link'>"
            "<collision name='collision'><geometry><cylinder>"
            "<radius>0.25</radius><length>1.7</length>"
            "</cylinder></geometry></collision>"
            "<visual name='visual'><geometry><cylinder>"
            "<radius>0.25</radius><length>1.7</length>"
            "</cylinder></geometry>"
            "<material><ambient>0.8 0.1 0.1 1.0</ambient>"
            "<diffuse>0.9 0.1 0.1 1.0</diffuse></material>"
            "</visual></link></model></sdf>"
        )
        model_name = "test_obstacle_human"
        target_z   = 0.30 + 0.85
    else:
        return False

    sdf_escaped = sdf.replace('"', '\\"')
    req_str = (
        f'sdf: "{sdf_escaped}", '
        f'pose: {{ position: {{ x: {target_x:.2f}, y: {target_y:.2f}, z: {target_z:.2f} }} }}'
    )
    cmd = [
        "gz", "service", "-s", "/world/campus_world/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", req_str
    ]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
    success = ("data: true" in r.stdout) or (r.returncode == 0)
    if success:
        print(f"    ✅ Spawned '{model_name}' at Gazebo ({target_x:.1f}, {target_y:.1f}, {target_z:.2f})")
    else:
        print(f"    ❌ Spawn failed: {r.stderr or r.stdout}")
    return success


def remove_all_obstacles():
    """Remove all test obstacles from Gazebo."""
    for model_name in ["test_obstacle_human", "test_obstacle_dog", "test_obstacle_bump"]:
        req_str = f'name: "{model_name}", type: 2'
        cmd = [
            "gz", "service", "-s", "/world/campus_world/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", req_str
        ]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
        if "data: true" in r.stdout or r.returncode == 0:
            print(f"    🧹 Removed '{model_name}'")
        else:
            print(f"    ⚠️  Could not remove '{model_name}' (may not exist)")


def test_7_estop(node: rclpy.node.Node) -> TestResult:
    """TEST 7 — Emergency Stop Test (spawn human 5m ahead, expect EMERGENCY_STOP)."""
    result = TestResult(
        "T7 — Emergency Stop Test",
        "Spawn human 5m ahead; wait ≤10s for safety_status to reach EMERGENCY_STOP."
    )
    print(f"\n{'='*60}")
    print(f"▶  {result.name}")
    print(f"{'='*60}")

    result.add_detail("Spawning human obstacle 5m ahead of SAB C spawn point …")
    spawn_ok = spawn_obstacle_gz("human", distance=5.0)
    if not spawn_ok:
        result.add_detail("  ⚠️  Spawn command may have failed, continuing to monitor …")

    result.add_detail("Waiting 5s for obstacle to register in LiDAR …")
    spin_for(node, 5.0)

    result.add_detail("Polling /campus/safety_status for EMERGENCY_STOP (10s) …")
    deadline = time.time() + 10.0
    estop_reached = False
    last_state = "UNKNOWN"

    while time.time() < deadline:
        msg = wait_for_message(node, String, "/campus/safety_status", timeout=1.5)
        if msg:
            try:
                payload = json.loads(msg.data)
                last_state = payload.get("state", "UNKNOWN")
                result.add_detail(f"  state={last_state}  obs_dist={payload.get('obstacle_dist','?')}")
                if last_state == "EMERGENCY_STOP":
                    estop_reached = True
                    break
            except json.JSONDecodeError:
                pass

    ss = take_screenshot("test7_estop.png")
    if ss:
        result.screenshots.append(ss)

    # Cleanup
    result.add_detail("Removing test obstacles …")
    remove_all_obstacles()

    if estop_reached:
        result.mark(TestResult.PASS,
                    "Safety monitor reached EMERGENCY_STOP within 10s of obstacle spawn.")
    else:
        result.mark(TestResult.FAIL,
                    f"EMERGENCY_STOP not reached in 10s — last state: '{last_state}'. "
                    "Buggy may be moving away from spawn or obstacle not visible to LiDAR.")

    return result


def test_8_multi_mission(node: rclpy.node.Node) -> TestResult:
    """TEST 8 — Multi-Mission Map Marking (3 sequential missions, check route ribbon).

    Root causes fixed vs original implementation:
    1. /campus/route_ribbon publishes visualization_msgs/Marker (not MarkerArray).
    2. The planner re-stamps current_ribbon with `now` every 0.5s timer tick, so
       stamp comparison always succeeds immediately — it detects the *old* ribbon
       from T6, not a genuinely new route.
    3. Correct detection: use a persistent subscription callback that counts
       distinct ADD messages and tracks point-list length.  A new route is
       confirmed when:
         a) We receive a DELETEALL (action=3) followed by an ADD (action=0),  OR
         b) The ADD message's point count differs from the pre-dispatch snapshot
            (new route = different geometry = different number of ribbon points).
    4. Added a stabilisation wait after T7 so the safety monitor is back to
       CLEAR before dispatching T8 missions.
    """
    result = TestResult(
        "T8 — Multi-Mission Map Marking",
        "Dispatch 3 missions sequentially; confirm /campus/route_ribbon has new geometry after each."
    )
    print(f"\n{'='*60}")
    print(f"▶  {result.name}")
    print(f"{'='*60}")

    # ── Give the system a moment to settle after T7 (obstacles removed) ───────
    result.add_detail("Waiting 5s post-T7 for safety state to return to CLEAR …")
    spin_for(node, 5.0)

    missions = [
        "SAB C|LOC",
        "LOC|Chemistry Block",
        "Chemistry Block|SAB C",
    ]
    screenshots = [
        "test8_mission1.png",
        "test8_mission2.png",
        "test8_mission3.png",
    ]

    mission_pub = node.create_publisher(String, "/campus/mission", 10)
    spin_for(node, 0.5)   # let publisher advertise

    ribbon_updates = 0

    for idx, (mission, ss_name) in enumerate(zip(missions, screenshots)):
        result.add_detail(f"\n  Mission {idx+1}/3: '{mission}'")

        # ── Step A: Snapshot the current ribbon geometry BEFORE dispatch ──────
        # /campus/route_ribbon → Marker (single, not MarkerArray)
        # action=0 → ADD (has geometry);  action=3 → DELETEALL (no geometry)
        pre_npts = None
        pre_msg = wait_for_message(node, Marker, "/campus/route_ribbon", timeout=4.0)
        if pre_msg is not None and pre_msg.action == 0:   # ADD with geometry
            pre_npts = len(pre_msg.points)
            result.add_detail(
                f"    Pre-dispatch ribbon: {pre_npts} points "
                f"(action=ADD, ns='{pre_msg.ns}')"
            )
        else:
            action_str = "DELETEALL" if (pre_msg and pre_msg.action == 3) else "none"
            result.add_detail(f"    Pre-dispatch ribbon: no geometry ({action_str})")

        # ── Step B: Dispatch mission ──────────────────────────────────────────
        m = String()
        m.data = mission
        mission_pub.publish(m)
        result.add_detail(f"    ✉️  Published: '{mission}'")

        # ── Step C: Wait for a genuinely *new* ribbon (different point count) ─
        # Strategy:
        #   • Keep a persistent callback that counts every ADD message received
        #     after dispatch time and captures the latest point count.
        #   • We also watch for DELETEALL followed by ADD (clear-then-replan).
        #   • A new route is confirmed when we receive an ADD whose point count
        #     differs from pre_npts  ─OR─  if pre_npts was None (no route before)
        #     and we now receive any ADD with points.
        #   • Timeout: 150s (missions involve 2 legs including teleport to start).

        new_route_found    = [False]
        new_route_npts     = [0]
        saw_deleteall      = [False]
        add_count_after    = [0]

        def ribbon_cb(msg: Marker):
            if new_route_found[0]:
                return
            if msg.action == 3:                      # DELETEALL — route was cleared
                saw_deleteall[0] = True
                return
            if msg.action != 0:                      # ignore MODIFY etc.
                return
            # It's an ADD with geometry
            npts = len(msg.points)
            add_count_after[0] += 1

            genuinely_new = False
            if pre_npts is None and npts > 0:
                # There was no route before; any ADD counts as new
                genuinely_new = True
            elif saw_deleteall[0] and npts > 0:
                # Route was cleared then a new one arrived — definitely new
                genuinely_new = True
            elif pre_npts is not None and npts != pre_npts:
                # Different number of ribbon points → new route geometry
                genuinely_new = True

            if genuinely_new:
                new_route_found[0] = True
                new_route_npts[0]  = npts

        ribbon_sub = node.create_subscription(
            Marker, "/campus/route_ribbon", ribbon_cb, 10
        )

        deadline = time.time() + 150.0
        last_log  = time.time()
        log_interval = 10.0   # print progress every 10s

        while time.time() < deadline and not new_route_found[0]:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - last_log > log_interval:
                elapsed = time.time() - (deadline - 150.0)
                result.add_detail(
                    f"    ⏳ {elapsed:.0f}s elapsed — "
                    f"ADD msgs received: {add_count_after[0]}, "
                    f"saw_deleteall: {saw_deleteall[0]}"
                )
                last_log = time.time()

        node.destroy_subscription(ribbon_sub)

        if new_route_found[0]:
            ribbon_updates += 1
            result.add_detail(
                f"    ✅ New route ribbon confirmed: {new_route_npts[0]} pts "
                f"(prev={pre_npts}, deleteall_seen={saw_deleteall[0]})"
            )
            # Allow the buggy time to execute the mission before next dispatch
            result.add_detail("    Allowing up to 60s for mission execution …")
            exec_deadline = time.time() + 60.0
            while time.time() < exec_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            elapsed = 150.0
            result.add_detail(
                f"    ❌ No new route ribbon within {elapsed:.0f}s "
                f"(ADD msgs received: {add_count_after[0]}, "
                f"deleteall={saw_deleteall[0]})"
            )

        ss = take_screenshot(ss_name)
        if ss:
            result.screenshots.append(ss)

    node.destroy_publisher(mission_pub)

    if ribbon_updates == 3:
        result.mark(TestResult.PASS,
                    "Route ribbon geometry changed 3/3 times — map marking working correctly.")
    else:
        result.mark(TestResult.FAIL,
                    f"New route ribbon detected for only {ribbon_updates}/3 missions.")

    return result


# ═══════════════════════════════════════════════════════════════════════════════
#  HTML REPORT GENERATOR
# ═══════════════════════════════════════════════════════════════════════════════

def _encode_image(path: str) -> str:
    """Base64-encode a PNG for inline embedding."""
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode("ascii")
    except Exception:
        return ""


def generate_report(results: List[TestResult],
                    session_start: float,
                    session_end: float) -> Path:
    """Generate a self-contained dark-theme HTML report with embedded screenshots."""
    timestamp  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = REPORTS_DIR / f"report_{timestamp}.html"
    REPORTS_DIR.mkdir(parents=True, exist_ok=True)

    total      = len(results)
    n_pass     = sum(1 for r in results if r.status == TestResult.PASS)
    n_fail     = sum(1 for r in results if r.status == TestResult.FAIL)
    n_skip     = sum(1 for r in results if r.status == TestResult.SKIP)
    duration   = session_end - session_start
    run_time   = datetime.datetime.fromtimestamp(session_start).strftime("%Y-%m-%d %H:%M:%S")

    def badge(status: str) -> str:
        colours = {
            TestResult.PASS: ("#22c55e", "🟢"),
            TestResult.FAIL: ("#ef4444", "🔴"),
            TestResult.SKIP: ("#f59e0b", "🟡"),
        }
        colour, icon = colours.get(status, ("#94a3b8", "⚪"))
        return (
            f'<span class="badge" style="background:{colour};">'
            f'{icon} {status}</span>'
        )

    def screenshot_html(ss_paths: List[str]) -> str:
        parts = []
        for path in ss_paths:
            b64 = _encode_image(path)
            name = Path(path).name
            if b64:
                parts.append(
                    f'<figure>'
                    f'<img src="data:image/png;base64,{b64}" alt="{name}" '
                    f'title="{name}">'
                    f'<figcaption>{name}</figcaption>'
                    f'</figure>'
                )
            else:
                parts.append(f'<p class="no-img">⚠️ Screenshot not available: {name}</p>')
        return "\n".join(parts) if parts else '<p class="no-img">No screenshots captured.</p>'

    test_cards = []
    for r in results:
        details_html = ""
        if r.details:
            items = "\n".join(f"<li>{d}</li>" for d in r.details)
            details_html = f"<ul class='details'>{items}</ul>"

        card_colour = {
            TestResult.PASS: "#22c55e",
            TestResult.FAIL: "#ef4444",
            TestResult.SKIP: "#f59e0b",
        }.get(r.status, "#94a3b8")

        test_cards.append(f"""
        <div class="card" style="border-left:4px solid {card_colour};">
          <div class="card-header">
            {badge(r.status)}
            <h2>{r.name}</h2>
            <span class="duration">{r.duration:.1f}s</span>
          </div>
          <p class="desc">{r.description}</p>
          <p class="result-msg"><strong>Result:</strong> {r.message}</p>
          {details_html}
          <div class="screenshots">
            {screenshot_html(r.screenshots)}
          </div>
        </div>""")

    cards_html = "\n".join(test_cards)

    summary_colour = "#22c55e" if n_fail == 0 and n_skip == 0 else (
        "#ef4444" if n_fail > 0 else "#f59e0b"
    )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>CAMP Autonomous Buggy — System Test Report</title>
<style>
  :root {{
    --bg:       #0f172a;
    --surface:  #1e293b;
    --border:   #334155;
    --text:     #e2e8f0;
    --muted:    #94a3b8;
    --pass:     #22c55e;
    --fail:     #ef4444;
    --warn:     #f59e0b;
    --accent:   #3b82f6;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{
    background: var(--bg);
    color: var(--text);
    font-family: 'Segoe UI', system-ui, sans-serif;
    line-height: 1.6;
    padding: 2rem;
  }}
  header {{
    text-align: center;
    margin-bottom: 2.5rem;
    padding-bottom: 1.5rem;
    border-bottom: 2px solid var(--border);
  }}
  header h1 {{
    font-size: 2rem;
    font-weight: 700;
    color: var(--text);
    margin-bottom: 0.4rem;
  }}
  header .subtitle {{ color: var(--muted); font-size: 0.95rem; }}
  .summary {{
    display: flex;
    gap: 1rem;
    justify-content: center;
    flex-wrap: wrap;
    margin-bottom: 2.5rem;
  }}
  .stat {{
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 12px;
    padding: 1rem 1.8rem;
    text-align: center;
    min-width: 120px;
  }}
  .stat .val {{
    font-size: 2.2rem;
    font-weight: 700;
  }}
  .stat .lbl {{ color: var(--muted); font-size: 0.8rem; letter-spacing: 0.05em; }}
  .overall {{
    border-color: {summary_colour};
    box-shadow: 0 0 16px {summary_colour}33;
  }}
  .overall .val {{ color: {summary_colour}; }}
  .pass-stat .val {{ color: var(--pass); }}
  .fail-stat .val {{ color: var(--fail); }}
  .skip-stat .val {{ color: var(--warn); }}
  .card {{
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 12px;
    padding: 1.5rem;
    margin-bottom: 1.5rem;
    transition: box-shadow 0.2s;
  }}
  .card:hover {{ box-shadow: 0 4px 24px #0004; }}
  .card-header {{
    display: flex;
    align-items: center;
    gap: 1rem;
    margin-bottom: 0.75rem;
    flex-wrap: wrap;
  }}
  .card-header h2 {{ font-size: 1.05rem; font-weight: 600; flex: 1; }}
  .duration {{ color: var(--muted); font-size: 0.8rem; white-space: nowrap; }}
  .badge {{
    display: inline-block;
    padding: 0.25rem 0.75rem;
    border-radius: 999px;
    font-size: 0.78rem;
    font-weight: 700;
    color: #0f172a;
    white-space: nowrap;
  }}
  .desc {{ color: var(--muted); font-size: 0.88rem; margin-bottom: 0.6rem; }}
  .result-msg {{ font-size: 0.92rem; margin-bottom: 0.6rem; }}
  ul.details {{
    list-style: none;
    margin: 0.5rem 0 0.5rem 0.5rem;
    font-size: 0.82rem;
    color: var(--muted);
  }}
  ul.details li {{
    padding: 0.15rem 0;
    border-left: 2px solid var(--border);
    padding-left: 0.6rem;
    margin-bottom: 0.1rem;
    font-family: 'Cascadia Code', 'Fira Code', monospace;
  }}
  .screenshots {{
    display: flex;
    flex-wrap: wrap;
    gap: 1rem;
    margin-top: 1rem;
  }}
  .screenshots figure {{
    flex: 1 1 300px;
    max-width: 600px;
  }}
  .screenshots img {{
    width: 100%;
    border-radius: 8px;
    border: 1px solid var(--border);
    display: block;
  }}
  .screenshots figcaption {{
    font-size: 0.75rem;
    color: var(--muted);
    margin-top: 0.3rem;
    text-align: center;
  }}
  .no-img {{ color: var(--muted); font-size: 0.82rem; }}
  footer {{
    text-align: center;
    color: var(--muted);
    font-size: 0.82rem;
    margin-top: 3rem;
    padding-top: 1.5rem;
    border-top: 1px solid var(--border);
  }}
</style>
</head>
<body>
<header>
  <h1>🚌 CAMP Autonomous Buggy — System Test Report</h1>
  <p class="subtitle">
    Run at {run_time} &nbsp;|&nbsp;
    Total duration: {duration:.0f}s ({duration/60:.1f} min) &nbsp;|&nbsp;
    ROS 2 Jazzy / Gazebo Harmonic
  </p>
</header>

<div class="summary">
  <div class="stat overall">
    <div class="val">{n_pass}/{total}</div>
    <div class="lbl">PASSED</div>
  </div>
  <div class="stat pass-stat">
    <div class="val">{n_pass}</div>
    <div class="lbl">PASS</div>
  </div>
  <div class="stat fail-stat">
    <div class="val">{n_fail}</div>
    <div class="lbl">FAIL</div>
  </div>
  <div class="stat skip-stat">
    <div class="val">{n_skip}</div>
    <div class="lbl">SKIP</div>
  </div>
  <div class="stat">
    <div class="val" style="color:var(--accent);">{duration:.0f}s</div>
    <div class="lbl">DURATION</div>
  </div>
</div>

{cards_html}

<footer>
  Generated by <strong>CAMP Test Suite</strong> &nbsp;·&nbsp;
  {run_time} &nbsp;·&nbsp; {n_pass} PASS / {n_fail} FAIL / {n_skip} SKIP
</footer>
</body>
</html>"""

    report_path.write_text(html, encoding="utf-8")
    print(f"\n  📄 Report saved: {report_path}")
    return report_path


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    print("\n" + "═"*64)
    print("  CAMP Autonomous Buggy — Automated Test Suite")
    print("  ROS 2 Jazzy | Gazebo Harmonic | campus_nav stack")
    print("═"*64 + "\n")

    session_start = time.time()
    results: List[TestResult] = []
    sim_proc: Optional[subprocess.Popen] = None

    # ── Step 0: Build launch environment ─────────────────────────────────────
    print("[0/9] Building ROS 2 + CAMP launch environment …")
    try:
        launch_env = build_launch_env()
    except Exception as e:
        print(f"  ❌ FATAL: Could not build env — {e}")
        print("  Make sure workspace is sourced:  source install/setup.bash")
        sys.exit(1)

    # ── Step 1: Launch simulation ─────────────────────────────────────────────
    print("\n[1/9] Launching full simulation …")
    try:
        sim_proc = launch_simulation(launch_env)
    except Exception as e:
        print(f"  ❌ FATAL: Could not launch simulation — {e}")
        sys.exit(1)

    # ── Step 2: Init rclpy ────────────────────────────────────────────────────
    print("\n[2/9] Initialising rclpy node 'camp_test_runner' …")
    try:
        rclpy.init()
        node = rclpy.node.Node("camp_test_runner")
        print("  ✅ rclpy node ready.")
    except Exception as e:
        print(f"  ❌ FATAL: rclpy init failed — {e}")
        shutdown_simulation(sim_proc)
        sys.exit(1)

    # ── Step 3: Give simulation 15s head start before polling ─────────────────
    print("\n[3/9] Waiting 15s for simulation processes to initialise …")
    spin_for(node, 15.0)

    # ── Step 4–11: Run test cases ──────────────────────────────────────────────
    test_funcs = [
        test_1_launch_check,
        test_2_buggy_spawn,
        test_3_sensor_feeds,
        test_4_pose_drift,
        test_5_safety_baseline,
        test_6_mission_execution,
        test_7_estop,
        test_8_multi_mission,
    ]

    for i, fn in enumerate(test_funcs, start=4):
        print(f"\n[{i}/11] Running {fn.__name__} …")
        try:
            res = fn(node)
        except KeyboardInterrupt:
            print("\n  ⚠️  KeyboardInterrupt — aborting remaining tests.")
            break
        except Exception as exc:
            res = TestResult(fn.__name__, "Unexpected exception in test function.")
            res.mark(TestResult.FAIL, f"Exception: {exc}")
            res.add_detail(traceback.format_exc())
            print(f"  ❌ Exception in {fn.__name__}: {exc}")

        icon = {"PASS": "✅", "FAIL": "❌", "SKIP": "🟡"}.get(res.status, "?")
        print(f"  {icon} {res.name}: {res.status} — {res.message}")
        results.append(res)

    # ── Teardown ──────────────────────────────────────────────────────────────
    print("\n[11/11] Tearing down …")
    try:
        node.destroy_node()
        rclpy.shutdown()
        print("  ✅ rclpy shutdown.")
    except Exception as e:
        print(f"  ⚠️  rclpy shutdown warning: {e}")

    if sim_proc:
        shutdown_simulation(sim_proc)

    # ── Report ────────────────────────────────────────────────────────────────
    session_end = time.time()
    print("\n" + "═"*64)
    print("  Generating HTML report …")
    report_path = generate_report(results, session_start, session_end)

    n_pass = sum(1 for r in results if r.status == TestResult.PASS)
    n_fail = sum(1 for r in results if r.status == TestResult.FAIL)
    n_skip = sum(1 for r in results if r.status == TestResult.SKIP)
    duration = session_end - session_start

    print("\n" + "═"*64)
    print(f"  ✅ PASS: {n_pass}   ❌ FAIL: {n_fail}   🟡 SKIP: {n_skip}")
    print(f"  ⏱️  Session duration: {duration:.0f}s ({duration/60:.1f} min)")
    print(f"  📄 Report: {report_path}")
    print("═"*64 + "\n")

    # Open report in browser if possible
    for browser in ["xdg-open", "firefox", "chromium-browser", "google-chrome"]:
        if shutil.which(browser):
            env = {**os.environ, "DISPLAY": DISPLAY_ENV}
            subprocess.Popen([browser, str(report_path)], env=env,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"  🌐 Opened report in {browser}.")
            break

    sys.exit(0 if n_fail == 0 else 1)


if __name__ == "__main__":
    main()
