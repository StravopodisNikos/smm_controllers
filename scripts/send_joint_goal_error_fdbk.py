#!/usr/bin/env python3

import argparse
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64


def load_yaml(path):
    path = os.path.expanduser(path)

    if not os.path.exists(path):
        raise RuntimeError(f"YAML file not found: {path}")

    with open(path, "r") as f:
        data = yaml.safe_load(f)

    return data if data is not None else {}


def load_active_joint_names(active_joint_names_yaml):
    data = load_yaml(active_joint_names_yaml)

    if "active_joint_names" not in data:
        raise RuntimeError(
            f"Missing key 'active_joint_names' in {active_joint_names_yaml}"
        )

    joint_names = data["active_joint_names"]

    if not isinstance(joint_names, list) or len(joint_names) == 0:
        raise RuntimeError("'active_joint_names' must be a non-empty list.")

    return joint_names


def load_manual_joint_goal(goal_yaml, goal_name, dof):
    data = load_yaml(goal_yaml)

    if "goals" not in data:
        raise RuntimeError(f"Missing top-level key 'goals' in {goal_yaml}")

    if goal_name not in data["goals"]:
        raise RuntimeError(f"Goal '{goal_name}' not found in {goal_yaml}")

    goal = data["goals"][goal_name]

    if "positions" not in goal:
        raise RuntimeError(f"Goal '{goal_name}' is missing required field 'positions'.")

    positions = goal["positions"]
    velocities = goal.get("velocities", [0.0] * dof)
    accelerations = goal.get("accelerations", [0.0] * dof)
    time_from_start = float(goal.get("time_from_start", 2.0))

    for field_name, values in [
        ("positions", positions),
        ("velocities", velocities),
        ("accelerations", accelerations),
    ]:
        if not isinstance(values, list):
            raise RuntimeError(f"'{field_name}' must be a list.")

        if len(values) != dof:
            raise RuntimeError(
                f"'{field_name}' length mismatch. "
                f"Expected {dof} values from active_joint_names.yaml, got {len(values)}."
            )

    return (
        [float(v) for v in positions],
        [float(v) for v in velocities],
        [float(v) for v in accelerations],
        time_from_start,
    )


class JointGoalSender(Node):
    def __init__(
        self,
        topic,
        controller_name,
        joint_names,
        positions,
        velocities,
        accelerations,
        time_from_start,
        publish_rate_hz,
        publish_duration_sec,
        stop_on_error,
        error_tolerance,
        stable_time_sec,
        timeout_sec,
    ):
        super().__init__("send_joint_goal_error_fdbk")

        self.controller_name = controller_name
        self.joint_names = joint_names
        self.dof = len(joint_names)

        self.stop_on_error = stop_on_error
        self.error_tolerance = error_tolerance
        self.stable_time_sec = stable_time_sec
        self.timeout_sec = timeout_sec

        self.latest_errors = [None] * self.dof
        self.stable_since = None
        self.start_time = time.time()

        self.publisher = self.create_publisher(
            JointTrajectory,
            topic,
            10,
        )

        self.msg = JointTrajectory()
        self.msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.accelerations = accelerations

        sec = int(time_from_start)
        nanosec = int((time_from_start - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        self.msg.points.append(point)

        self.publish_count = 0
        self.max_publish_count = max(1, int(publish_rate_hz * publish_duration_sec))

        if self.stop_on_error:
            self.error_subs = []
            for i in range(self.dof):
                topic_name = f"/{self.controller_name}/q_error_{i}"
                sub = self.create_subscription(
                    Float64,
                    topic_name,
                    lambda msg, idx=i: self.error_callback(msg, idx),
                    10,
                )
                self.error_subs.append(sub)

            self.get_logger().info(
                f"Stop-on-error enabled: tolerance={self.error_tolerance}, "
                f"stable_time={self.stable_time_sec}s, timeout={self.timeout_sec}s"
            )

        timer_period = 1.0 / publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Publishing joint goal to '{topic}' at {publish_rate_hz:.2f} Hz "
            f"for up to {publish_duration_sec:.2f}s "
            f"({self.max_publish_count} messages)."
        )

    def error_callback(self, msg, idx):
        self.latest_errors[idx] = float(msg.data)

    def all_errors_available(self):
        return all(e is not None for e in self.latest_errors)

    def max_abs_error(self):
        if not self.all_errors_available():
            return None

        return max(abs(e) for e in self.latest_errors)

    def stop_condition_reached(self):
        if not self.stop_on_error:
            return False

        if time.time() - self.start_time > self.timeout_sec:
            self.get_logger().warn("Timeout reached before error tolerance was satisfied.")
            return True

        max_error = self.max_abs_error()

        if max_error is None:
            return False

        if max_error < self.error_tolerance:
            if self.stable_since is None:
                self.stable_since = time.time()

            if time.time() - self.stable_since >= self.stable_time_sec:
                self.get_logger().info(
                    f"Goal reached. max_abs_error={max_error:.6f} < {self.error_tolerance}"
                )
                return True
        else:
            self.stable_since = None

        return False

    def timer_callback(self):
        if self.stop_condition_reached():
            rclpy.shutdown()
            return

        if self.publish_count >= self.max_publish_count and not self.stop_on_error:
            self.get_logger().info("Finished publishing joint goal.")
            rclpy.shutdown()
            return

        if self.publish_count < self.max_publish_count:
            self.publisher.publish(self.msg)
            self.publish_count += 1

            self.get_logger().info(
                f"Published joint goal with {self.dof} joints "
                f"({self.publish_count}/{self.max_publish_count})."
            )


def main():
    parser = argparse.ArgumentParser(
        description="Publish a manual joint-space goal using runtime active_joint_names.yaml."
    )

    parser.add_argument(
        "--active-joint-names-yaml",
        required=True,
        help="Runtime YAML generated by smm_synthesis containing active_joint_names.",
    )

    parser.add_argument(
        "--goal-yaml",
        required=True,
        help="Manual joint goal YAML. Contains numerical command values only.",
    )

    parser.add_argument(
        "--goal",
        required=True,
        help="Goal name inside the manual goal YAML.",
    )

    parser.add_argument(
        "--topic",
        default="/smm_joint_controller/reference",
        help="Controller reference topic.",
    )

    parser.add_argument(
        "--controller-name",
        default="smm_joint_controller",
        help="Controller name used for q_error topics.",
    )

    parser.add_argument(
        "--publish-rate-hz",
        type=float,
        default=2.0,
        help="Repeated publish rate in Hz.",
    )

    parser.add_argument(
        "--publish-duration-sec",
        type=float,
        default=3.0,
        help="Maximum command publish duration.",
    )

    parser.add_argument(
        "--stop-on-error",
        action="store_true",
        help="Stop after q_error_i topics are below threshold.",
    )

    parser.add_argument(
        "--error-tolerance",
        type=float,
        default=0.01,
        help="Max absolute joint error threshold in rad.",
    )

    parser.add_argument(
        "--stable-time-sec",
        type=float,
        default=0.5,
        help="Required time below error threshold before stopping.",
    )

    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=10.0,
        help="Timeout when stop-on-error is enabled.",
    )

    args = parser.parse_args()

    joint_names = load_active_joint_names(args.active_joint_names_yaml)
    dof = len(joint_names)

    positions, velocities, accelerations, time_from_start = load_manual_joint_goal(
        goal_yaml=args.goal_yaml,
        goal_name=args.goal,
        dof=dof,
    )

    rclpy.init()

    node = JointGoalSender(
        topic=args.topic,
        controller_name=args.controller_name,
        joint_names=joint_names,
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
        time_from_start=time_from_start,
        publish_rate_hz=args.publish_rate_hz,
        publish_duration_sec=args.publish_duration_sec,
        stop_on_error=args.stop_on_error,
        error_tolerance=args.error_tolerance,
        stable_time_sec=args.stable_time_sec,
        timeout_sec=args.timeout_sec,
    )

    rclpy.spin(node)


if __name__ == "__main__":
    main()