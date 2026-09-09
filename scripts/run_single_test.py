#!/usr/bin/env python3
"""
run_single_test.py

Manual single-trial tester. Edit START / GOAL / PLANNER below, run the
script, and it will:

    1. Kill any stale Gazebo/Nav2 processes left over from a previous test.
    2. Launch Gazebo fresh with the robot spawned at START.
    3. Launch Nav2 with the chosen planner's params file.
    4. Wait for AMCL to become active, publish /initialpose at START, and
       verify AMCL actually converged to it (not just a blind sleep).
    5. Send one NavigateToPose goal to GOAL, time it, compute path length
       from /plan, and record success/failure.
    6. Tear everything down (Gazebo/Nav2/RViz) and kill stale processes again.
    7. Write one row to a timestamped CSV so repeated runs never overwrite
       each other, e.g. result_dwb_20260909_143012.csv

Usage:
    python3 run_single_test.py
    python3 run_single_test.py --planner rpp
    python3 run_single_test.py --rviz          # also open RViz to watch

*** Edit START, GOAL, PLANNER below before each manual test. ***
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from lifecycle_msgs.srv import GetState


# ---------------------------------------------------------------------------
# EDIT THESE before each manual test
# ---------------------------------------------------------------------------

START = (4.08799, 8.31414, 0.0)   # x, y, yaw (radians)
GOAL = (-4.81866, -0.0987258, 0.0)
PLANNER = "dwb"                    # "dwb" or "rpp" (overridden by --planner)

WORKSPACE_DIR = os.path.expanduser("~/projects/ros2/augdd_ws")
WORKSPACE_SETUP = f"source {WORKSPACE_DIR}/install/setup.bash"

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

PARAMS_FILES = {
    "dwb": os.path.join(WORKSPACE_DIR, "src/augdd_navigation/config/nav2_params_dwb.yaml"),
    "rpp": os.path.join(WORKSPACE_DIR, "src/augdd_navigation/config/nav2_params_rpp.yaml"),
}

GAZEBO_SETTLE_S = 5
NAV_RESULT_TIMEOUT_S = 90.0
SHUTDOWN_GRACE_S = 5

STALE_PROCESS_PATTERNS = [
    "gzserver", "gzclient", "gazebo_ros", "spawn_entity.py",
    "component_container_isolated", "component_container",
    "robot_state_publisher", "amcl", "controller_server",
    "planner_server", "recoveries_server", "behavior_server",
    "bt_navigator", "waypoint_follower", "lifecycle_manager",
    "map_server", "rviz2",
]


def kill_stale_processes():
    for pattern in STALE_PROCESS_PATTERNS:
        subprocess.run(["pkill", "-9", "-f", pattern],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
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
    def __init__(self, cmd, name, tag):
        self.name = name
        full_cmd = f"{WORKSPACE_SETUP} && cd {WORKSPACE_DIR} && {cmd}"
        log_path = os.path.join(LOG_DIR, f"{tag}_{name}.log")
        print(f"  -> launching {name} (log: {log_path})")
        self._log_file = open(log_path, "w")
        self.proc = subprocess.Popen(
            full_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid,
            stdout=self._log_file, stderr=subprocess.STDOUT,
        )

    def terminate(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except ProcessLookupError:
            self._log_file.close()
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
    def __init__(self):
        super().__init__("single_test_client")
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._latest_path = None
        self._latest_amcl_pose = None
        qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE, history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Path, "/plan", self._plan_cb, qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, qos)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._amcl_get_state_client = self.create_client(GetState, "/amcl/get_state")

    def _plan_cb(self, msg):
        self._latest_path = msg

    def _amcl_pose_cb(self, msg):
        self._latest_amcl_pose = msg

    def wait_for_amcl_active(self, timeout_s=30.0):
        deadline = time.monotonic() + timeout_s
        if not self._amcl_get_state_client.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().warn("/amcl/get_state service never appeared.")
            return False
        while time.monotonic() < deadline:
            future = self._amcl_get_state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            result = future.result()
            if result is not None and result.current_state.label == "active":
                self.get_logger().info("AMCL is active.")
                return True
            time.sleep(0.5)
        self.get_logger().warn("Timed out waiting for AMCL to become active.")
        return False

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
        for _ in range(3):
            self._initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.3)

    def wait_for_pose_convergence(self, x, y, tolerance_m=0.5, timeout_s=10.0):
        deadline = time.monotonic() + timeout_s
        self._latest_amcl_pose = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.3)
            if self._latest_amcl_pose is not None:
                px = self._latest_amcl_pose.pose.pose.position.x
                py = self._latest_amcl_pose.pose.pose.position.y
                dist = math.hypot(px - x, py - y)
                if dist <= tolerance_m:
                    self.get_logger().info(f"AMCL converged: ({px:.2f},{py:.2f}), off by {dist:.2f}m")
                    return True
        if self._latest_amcl_pose is not None:
            px = self._latest_amcl_pose.pose.pose.position.x
            py = self._latest_amcl_pose.pose.pose.position.y
            self.get_logger().warn(f"AMCL did NOT converge: reports ({px:.2f},{py:.2f}), expected ({x:.2f},{y:.2f})")
        else:
            self.get_logger().warn("Never received /amcl_pose -- check nav2 log.")
        return False

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


def main():
    parser = argparse.ArgumentParser(description="Run a single manual Nav2 test.")
    parser.add_argument("--planner", default=PLANNER, choices=["dwb", "rpp"])
    parser.add_argument("--rviz", action="store_true", help="Also launch RViz to watch.")
    args = parser.parse_args()

    sx, sy, syaw = START
    gx, gy, gyaw = GOAL

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = f"result_{args.planner}_{timestamp}.csv"

    print(f"=== single test [{args.planner}]: start=({sx},{sy}) goal=({gx},{gy}) ===")
    print("Clearing any stale processes before starting...")
    kill_stale_processes()

    handles = []
    length, elapsed, success = 0.0, 0.0, False
    try:
        gazebo_cmd = GAZEBO_LAUNCH_CMD.format(x=sx, y=sy, yaw=syaw)
        handles.append(ProcessHandle(gazebo_cmd, "gazebo", timestamp))
        print(f"Waiting {GAZEBO_SETTLE_S}s for Gazebo to come up...")
        time.sleep(GAZEBO_SETTLE_S)

        nav2_cmd = NAV2_LAUNCH_CMD.format(params_file=PARAMS_FILES[args.planner])
        handles.append(ProcessHandle(nav2_cmd, "nav2", timestamp))

        if args.rviz:
            handles.append(ProcessHandle(RVIZ_CMD, "rviz", timestamp))

        rclpy.init()
        node = SingleRunClient()

        print("Waiting for AMCL to become active...")
        node.wait_for_amcl_active(timeout_s=30.0)

        print(f"Publishing /initialpose at ({sx},{sy},{syaw})...")
        node.publish_initialpose(sx, sy, syaw)

        print("Verifying AMCL converged to the published pose...")
        node.wait_for_pose_convergence(sx, sy, tolerance_m=0.5, timeout_s=10.0)

        print("Sending goal...")
        length, elapsed, success = node.send_goal_and_wait(gx, gy, gyaw)
        node.destroy_node()
        rclpy.shutdown()

        print(f"\nRESULT: success={success} time={elapsed:.2f}s path_length={length:.2f}m")

    finally:
        print("Tearing down Gazebo/Nav2/RViz...")
        for h in reversed(handles):
            h.terminate()
        kill_stale_processes()

    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["planner", "start_x", "start_y", "goal_x", "goal_y",
                          "time_to_goal_s", "path_length_m", "success"])
        writer.writerow([args.planner, sx, sy, gx, gy,
                          round(elapsed, 3), round(length, 3), success])
    print(f"Wrote result to {output_path}")


if __name__ == "__main__":
    main()
