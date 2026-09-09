#!/usr/bin/env python3
"""
run_nav2_experiments_fresh.py

Alternative to run_nav2_experiments.py that avoids the Gazebo teleport
problem entirely. For EACH run (one start/goal pair, one repeat):

    1. Launches Gazebo from scratch with the robot spawned directly at the
       run's start pose (no /gazebo/set_entity_state teleport involved).
    2. Launches Nav2 with the given planner's params file.
    3. Optionally launches RViz (off by default -- slows startup, only
       useful if you want to watch).
    4. Publishes /initialpose once Nav2 is up, matching the true spawn
       pose, so AMCL starts converged.
    5. Waits STABILIZE_SECONDS (measured from Nav2 launch) for everything
       to settle.
    6. Sends exactly one NavigateToPose goal to the run's goal pose, times
       it, computes path length from /plan, records success/failure.
    7. Tears down Gazebo/Nav2/RViz completely (SIGINT, then SIGKILL if a
       process won't die) before moving to the next run.

Run this as a single script from one terminal -- it manages all the
"launch three terminals" work itself.

*** BEFORE RUNNING: edit GAZEBO_LAUNCH_CMD and NAV2_LAUNCH_CMD below to
match your actual package/launch-file names and argument names. Check
what your Gazebo spawn launch file actually accepts with:

    ros2 launch <your_gazebo_package> <your_launch_file>.launch.py --show-args

Common argument names for spawn pose are x_pose/y_pose/yaw or
x:=/y:=/Y:= -- adjust the .format() call to match. ***
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path


# ---------------------------------------------------------------------------
# EDIT THESE to match your actual setup
# ---------------------------------------------------------------------------

WORKSPACE_DIR = os.path.expanduser("~/projects/ros2/augdd_ws")
WORKSPACE_SETUP = f"source {WORKSPACE_DIR}/install/setup.bash"

# Must accept pose args for where to spawn the robot. Rename x_pose/y_pose/yaw
# to whatever your launch file's DeclareLaunchArgument names actually are.
GAZEBO_LAUNCH_CMD = (
    "ros2 launch augdd_gazebo gazebo.launch.py "
    "x_pose:={x} y_pose:={y} yaw:={yaw}"
)

NAV2_LAUNCH_CMD = (
    "ros2 launch augdd_navigation navigation.launch.py "
    "params_file:={params_file}"
)

RVIZ_CMD = (
    "rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"
)

# Absolute paths -- must not be relative, since the script's subprocess
# working directory may not be the workspace root.
PARAMS_FILES = {
    "dwb": os.path.join(WORKSPACE_DIR, "src/augdd_navigation/config/nav2_params_dwb.yaml"),
    "rpp": os.path.join(WORKSPACE_DIR, "src/augdd_navigation/config/nav2_params_rpp.yaml"),
}

# Edit with real map coordinates (see earlier RViz "Publish Point" instructions)
POSE_PAIRS = [
    {"start": (4.08799, 8.31414, 0.0), "goal": (-4.81866, -0.0987258, 0.0)},
    {"start": (4.9174, -0.217252, 0.0), "goal": (1.0467, 7.95868, 0.0)},
    {"start": (5.29262, -0.039517, 0.0), "goal": (-5.07539, -0.01973, 0.0)},
    {"start": (2.48834, 5.54934, 0.0), "goal": (-4.79891, -0.138222, 0.0)},
    {"start": (-4.8779, -0.0987256, 0.0), "goal": (4.08799, 8.1759, 0.0)},
]

REPEATS_PER_PAIR = 3
GAZEBO_SETTLE_S = 5          # head start before launching Nav2
NAV2_INITIALPOSE_DELAY_S = 3 # let Nav2 nodes come up before publishing /initialpose
STABILIZE_SECONDS = 10       # total wait counted from Nav2 launch, per your request
NAV_RESULT_TIMEOUT_S = 90.0
LAUNCH_RVIZ = False          # flip to True to watch (adds startup time)
SHUTDOWN_GRACE_S = 5
BETWEEN_RUN_PAUSE_S = 2      # let ports/processes fully release before next run

# Process name patterns to force-kill before/after every run, in case a
# previous Gazebo/Nav2 launch left orphaned children that survived SIGINT
# (component containers, gzserver/gzclient, individual Nav2 nodes, etc.)
STALE_PROCESS_PATTERNS = [
    "gzserver",
    "gzclient",
    "gazebo_ros",
    "spawn_entity.py",
    "component_container_isolated",
    "component_container",
    "robot_state_publisher",
    "amcl",
    "controller_server",
    "planner_server",
    "recoveries_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "lifecycle_manager",
    "map_server",
    "rviz2",
]


def kill_stale_processes():
    """Force-kill any leftover Gazebo/Nav2 processes from a previous run
    that didn't die cleanly. Safe to call even if nothing is running."""
    for pattern in STALE_PROCESS_PATTERNS:
        subprocess.run(
            ["pkill", "-9", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    # give the OS a moment to actually release ports/shared memory
    time.sleep(1.5)


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose_stamped(node, x, y, yaw, frame="map"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    _, _, qz, qw = yaw_to_quat(yaw)
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


def path_length_m(path_msg: Path) -> float:
    pts = path_msg.poses
    total = 0.0
    for i in range(1, len(pts)):
        dx = pts[i].pose.position.x - pts[i - 1].pose.position.x
        dy = pts[i].pose.position.y - pts[i - 1].pose.position.y
        total += math.hypot(dx, dy)
    return total


LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_logs")
os.makedirs(LOG_DIR, exist_ok=True)


class ProcessHandle:
    """Launches a shell command in its own process group so the whole
    subtree (ros2 launch spawns many children) can be killed cleanly.
    Output goes to a log file (not DEVNULL) so failures are diagnosable --
    check run_logs/ if a run silently fails."""

    def __init__(self, cmd, name, run_id):
        self.name = name
        full_cmd = f"{WORKSPACE_SETUP} && cd {WORKSPACE_DIR} && {cmd}"
        log_path = os.path.join(LOG_DIR, f"run{run_id}_{name}.log")
        print(f"  -> launching {name}: {cmd}")
        print(f"     (log: {log_path})")
        self._log_file = open(log_path, "w")
        self.proc = subprocess.Popen(
            full_cmd,
            shell=True,
            executable="/bin/bash",
            preexec_fn=os.setsid,
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
        )

    def terminate(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            self.proc.wait(timeout=SHUTDOWN_GRACE_S)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        finally:
            self._log_file.close()


class SingleRunClient(Node):
    """Fresh rclpy node per run: publishes /initialpose once, then sends
    exactly one NavigateToPose goal and reports the outcome."""

    def __init__(self):
        super().__init__("nav2_single_run_client")
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._latest_path = None
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Path, "/plan", self._plan_cb, qos)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

    def _plan_cb(self, msg):
        self._latest_path = msg

    def publish_initialpose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        _, _, qz, qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        # publish a few times -- /initialpose has no guaranteed transient-local
        # delivery, and AMCL's subscriber may not be fully up on the first shot
        for _ in range(3):
            self._initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.3)

    def send_goal_and_wait(self, gx, gy, gyaw):
        if not self._nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("navigate_to_pose action server never came up.")
            return 0.0, 0.0, False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(self, gx, gy, gyaw)

        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        t_start = time.monotonic()
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Goal rejected or send timed out.")
            return 0.0, 0.0, False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=NAV_RESULT_TIMEOUT_S)
        elapsed = time.monotonic() - t_start

        if result_future.result() is None:
            self.get_logger().warn("Result timed out -- treating as failure.")
            goal_handle.cancel_goal_async()
            return 0.0, elapsed, False

        status = result_future.result().status
        success = status == GoalStatus.STATUS_SUCCEEDED

        for _ in range(5):
            if self._latest_path is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.2)

        length = path_length_m(self._latest_path) if self._latest_path else 0.0
        return length, elapsed, success


def run_one_trial(planner, run_id, start, goal):
    sx, sy, syaw = start
    gx, gy, gyaw = goal

    print(f"\n=== [{planner}] run {run_id}: start=({sx},{sy}) goal=({gx},{gy}) ===")

    print("  clearing any stale Gazebo/Nav2 processes before starting...")
    kill_stale_processes()

    handles = []
    try:
        gazebo_cmd = GAZEBO_LAUNCH_CMD.format(x=sx, y=sy, yaw=syaw)
        handles.append(ProcessHandle(gazebo_cmd, "gazebo", run_id))
        print(f"  waiting {GAZEBO_SETTLE_S}s for Gazebo to come up...")
        time.sleep(GAZEBO_SETTLE_S)

        nav2_cmd = NAV2_LAUNCH_CMD.format(params_file=PARAMS_FILES[planner])
        handles.append(ProcessHandle(nav2_cmd, "nav2", run_id))

        if LAUNCH_RVIZ:
            handles.append(ProcessHandle(RVIZ_CMD, "rviz", run_id))

        print(f"  waiting {NAV2_INITIALPOSE_DELAY_S}s before publishing /initialpose...")
        time.sleep(NAV2_INITIALPOSE_DELAY_S)

        rclpy.init()
        node = SingleRunClient()
        node.publish_initialpose(sx, sy, syaw)

        remaining = STABILIZE_SECONDS - NAV2_INITIALPOSE_DELAY_S
        if remaining > 0:
            print(f"  waiting remaining {remaining}s to reach {STABILIZE_SECONDS}s total stabilization...")
            time.sleep(remaining)

        length, elapsed, success = node.send_goal_and_wait(gx, gy, gyaw)
        node.destroy_node()
        rclpy.shutdown()

        print(f"  result: success={success} time={elapsed:.2f}s path_length={length:.2f}m")
        return length, elapsed, success

    finally:
        print("  tearing down Gazebo/Nav2/RViz for this run...")
        for h in reversed(handles):
            h.terminate()
        print("  clearing any leftover processes after teardown...")
        kill_stale_processes()
        time.sleep(BETWEEN_RUN_PAUSE_S)


def write_csv(rows, path, append):
    file_exists = os.path.isfile(path)
    mode = "a" if append and file_exists else "w"
    with open(path, mode, newline="") as f:
        writer = csv.writer(f)
        if mode == "w":
            writer.writerow([
                "planner", "run_id", "start_x", "start_y", "goal_x", "goal_y",
                "time_to_goal_s", "path_length_m", "success",
            ])
        for r in rows:
            writer.writerow(r)
    print(f"\nWrote {len(rows)} rows to {path}")


def main():
    parser = argparse.ArgumentParser(
        description="Run Nav2 planner comparison experiments with a full "
                     "Gazebo/Nav2 restart per run (avoids teleport issues)."
    )
    parser.add_argument("--planner", required=True, choices=["dwb", "rpp"])
    parser.add_argument("--output", default=None)
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--repeats", type=int, default=REPEATS_PER_PAIR)
    args = parser.parse_args()

    output_path = args.output or f"results_{args.planner}.csv"

    rows = []
    run_id = 0
    for pair in POSE_PAIRS:
        for _ in range(args.repeats):
            run_id += 1
            length, elapsed, success = run_one_trial(
                args.planner, run_id, pair["start"], pair["goal"]
            )
            sx, sy, _ = pair["start"]
            gx, gy, _ = pair["goal"]
            rows.append([
                args.planner, run_id, sx, sy, gx, gy,
                round(elapsed, 3), round(length, 3), success,
            ])

    write_csv(rows, output_path, args.append)


if __name__ == "__main__":
    main()
