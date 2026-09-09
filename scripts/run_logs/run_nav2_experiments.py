#!/usr/bin/env python3
"""
run_nav2_experiments.py

Automates repeated NavigateToPose runs against a running Nav2 stack for a
single planner configuration, logging time-to-goal, path length, and
success/failure to a CSV row per run.

Usage:
    # Launch Nav2 with the DWB params file first, then:
    python3 run_nav2_experiments.py --planner dwb --output results_dwb.csv

    # Relaunch Nav2 with the RPP params file, then:
    python3 run_nav2_experiments.py --planner rpp --output results_rpp.csv

    # Then concatenate results_dwb.csv + results_rpp.csv (or just point
    # both runs at the same --output file with --append) before running
    # aggregate_results.py.

Requirements:
    - Nav2 bringup + AMCL already running for the current map
      (~/projects/ros2/augdd_ws/maps/augdd_map.yaml)
    - Gazebo Classic running with the robot spawned as entity AUGDD_NAME
      (edit ROBOT_ENTITY_NAME below to match your spawn call)
    - gazebo_msgs, nav2_msgs, nav_msgs available (standard with Nav2 Humble)

Edit POSE_PAIRS below to match real corridor waypoints in your map.
"""

import argparse
import csv
import math
import os
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path

try:
    from gazebo_msgs.srv import SetEntityState
    from gazebo_msgs.msg import EntityState
    HAVE_GAZEBO_MSGS = True
except ImportError:
    HAVE_GAZEBO_MSGS = False


# --------------------------------------------------------------------------
# Config — edit these to match your map/world
# --------------------------------------------------------------------------

ROBOT_ENTITY_NAME = "augdd"  # must match the name used when spawning in Gazebo

# Each pair: (start_x, start_y, start_yaw_rad), (goal_x, goal_y, goal_yaw_rad)
# First pair reuses the known spawn point (-4.5, 0.0) as start.
POSE_PAIRS = [
    {"start": (4.08799, 8.31414, 0), "goal": (-4.81866, -0.0987258, 0)},
    {"start": (4.9174, -0.217252, 0), "goal": (1.0467, 7.95868, 0)},
    {"start": (5.29262, -0.039517, 0), "goal": (-5.07539, -0.01973, 0)},
    {"start": (2.48834, 5.54934, 0), "goal": (-4.79891, -0.138222, 0)},
    {"start": (-4.8779, -0.0987256, 0), "goal": (4.08799, 8.1759, 0)},
]

REPEATS_PER_PAIR = 3
NAV_RESULT_TIMEOUT_S = 90.0        # give up on a run after this long
AMCL_SETTLE_TIME_S = 1.5           # wait after publishing /initialpose
GAZEBO_TELEPORT_SETTLE_S = 0.5


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose_stamped(node, x, y, yaw, frame="map"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
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


@dataclass
class RunResult:
    planner: str
    run_id: int
    start_x: float
    start_y: float
    goal_x: float
    goal_y: float
    time_to_goal_s: float
    path_length_m: float
    success: bool


class ExperimentRunner(Node):
    def __init__(self, planner_name: str):
        super().__init__("nav2_experiment_runner")
        self.planner_name = planner_name

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._latest_path = None
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Path, "/plan", self._plan_cb, latched_qos)

        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        self._set_entity_state_client = None
        if HAVE_GAZEBO_MSGS:
            self._set_entity_state_client = self.create_client(
                SetEntityState, "/gazebo/set_entity_state"
            )

        self.get_logger().info("Waiting for NavigateToPose action server...")
        self._nav_client.wait_for_server()
        self.get_logger().info("Action server available.")

    def _plan_cb(self, msg: Path):
        self._latest_path = msg

    # ------------------------------------------------------------------
    def teleport_robot(self, x, y, yaw):
        """Move the robot in Gazebo to the given pose. No-op if gazebo_msgs
        is unavailable (e.g. running against a real robot / different sim)."""
        if self._set_entity_state_client is None:
            self.get_logger().warn(
                "gazebo_msgs not available — skipping teleport. "
                "Assuming robot is already at the intended start pose."
            )
            return

        if not self._set_entity_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "/gazebo/set_entity_state service not available — skipping teleport."
            )
            return

        req = SetEntityState.Request()
        state = EntityState()
        state.name = ROBOT_ENTITY_NAME
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = 0.05
        _, _, qz, qw = yaw_to_quat(yaw)
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        req.state = state

        future = self._set_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        time.sleep(GAZEBO_TELEPORT_SETTLE_S)

    def reinit_amcl(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        _, _, qz, qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Modest covariance so AMCL trusts this estimate but can still correct.
        msg.pose.covariance[0] = 0.25   # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = 0.06853891945200942  # yaw
        self._initialpose_pub.publish(msg)
        time.sleep(AMCL_SETTLE_TIME_S)

    # ------------------------------------------------------------------
    def run_single(self, run_id, start, goal) -> RunResult:
        sx, sy, syaw = start
        gx, gy, gyaw = goal

        self.get_logger().info(
            f"[{self.planner_name}] run {run_id}: start=({sx},{sy}) goal=({gx},{gy})"
        )

        self.teleport_robot(sx, sy, syaw)
        self.reinit_amcl(sx, sy, syaw)
        self._latest_path = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(self, gx, gy, gyaw)

        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        t_start = time.monotonic()

        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        goal_handle = send_goal_future.result()

        success = False
        elapsed = 0.0

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected or send timed out.")
        else:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=NAV_RESULT_TIMEOUT_S
            )
            elapsed = time.monotonic() - t_start

            if result_future.result() is None:
                self.get_logger().warn("Result timed out — treating as failure.")
                goal_handle.cancel_goal_async()
            else:
                status = result_future.result().status
                success = status == GoalStatus.STATUS_SUCCEEDED

        # give /plan a moment to catch the final published path if it hasn't already
        for _ in range(5):
            if self._latest_path is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.2)

        length = path_length_m(self._latest_path) if self._latest_path else 0.0

        return RunResult(
            planner=self.planner_name,
            run_id=run_id,
            start_x=sx, start_y=sy,
            goal_x=gx, goal_y=gy,
            time_to_goal_s=round(elapsed, 3),
            path_length_m=round(length, 3),
            success=success,
        )


def write_csv(rows, output_path, append):
    file_exists = os.path.isfile(output_path)
    mode = "a" if append and file_exists else "w"
    with open(output_path, mode, newline="") as f:
        writer = csv.writer(f)
        if mode == "w":
            writer.writerow([
                "planner", "run_id", "start_x", "start_y", "goal_x", "goal_y",
                "time_to_goal_s", "path_length_m", "success",
            ])
        for r in rows:
            writer.writerow([
                r.planner, r.run_id, r.start_x, r.start_y, r.goal_x, r.goal_y,
                r.time_to_goal_s, r.path_length_m, r.success,
            ])
    print(f"Wrote {len(rows)} rows to {output_path} (mode={mode})")


def main():
    parser = argparse.ArgumentParser(description="Run Nav2 planner comparison experiments.")
    parser.add_argument("--planner", required=True, choices=["dwb", "rpp"],
                         help="Label for the planner currently loaded in Nav2 (informational only — "
                              "you must have already launched Nav2 with the matching params file).")
    parser.add_argument("--output", default=None,
                         help="CSV path to write. Defaults to results_<planner>.csv")
    parser.add_argument("--append", action="store_true",
                         help="Append to --output instead of overwriting.")
    parser.add_argument("--repeats", type=int, default=REPEATS_PER_PAIR,
                         help=f"Repeats per start/goal pair (default {REPEATS_PER_PAIR}).")
    args = parser.parse_args()

    output_path = args.output or f"results_{args.planner}.csv"

    rclpy.init()
    node = ExperimentRunner(args.planner)

    all_results = []
    run_id = 0
    try:
        for pair in POSE_PAIRS:
            for _ in range(args.repeats):
                run_id += 1
                result = node.run_single(run_id, pair["start"], pair["goal"])
                all_results.append(result)
                print(result)
    finally:
        write_csv(all_results, output_path, args.append)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
