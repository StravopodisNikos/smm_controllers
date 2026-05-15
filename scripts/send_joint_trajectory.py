#!/usr/bin/env python3

import argparse
import os
import time
import yaml
import subprocess
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


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


def duration_from_float(seconds_float):
    seconds_float = float(seconds_float)
    sec = int(seconds_float)
    nanosec = int((seconds_float - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def validate_vector(field_name, values, dof, point_index):
    if not isinstance(values, list):
        raise RuntimeError(
            f"Point {point_index}: '{field_name}' must be a list."
        )

    if len(values) != dof:
        raise RuntimeError(
            f"Point {point_index}: '{field_name}' length mismatch. "
            f"Expected {dof}, got {len(values)}."
        )

    return [float(v) for v in values]


def load_manual_joint_trajectory(trajectory_yaml, trajectory_name, dof):
    data = load_yaml(trajectory_yaml)

    if "trajectories" not in data:
        raise RuntimeError(
            f"Missing top-level key 'trajectories' in {trajectory_yaml}"
        )

    if trajectory_name not in data["trajectories"]:
        raise RuntimeError(
            f"Trajectory '{trajectory_name}' not found in {trajectory_yaml}"
        )

    trajectory = data["trajectories"][trajectory_name]

    if "points" not in trajectory:
        raise RuntimeError(
            f"Trajectory '{trajectory_name}' is missing required field 'points'."
        )

    raw_points = trajectory["points"]

    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise RuntimeError(
            f"Trajectory '{trajectory_name}' must contain at least 2 points."
        )

    points = []
    previous_time = None

    for idx, raw_point in enumerate(raw_points):
        if "positions" not in raw_point:
            raise RuntimeError(f"Point {idx}: missing required field 'positions'.")

        if "time_from_start" not in raw_point:
            raise RuntimeError(f"Point {idx}: missing required field 'time_from_start'.")

        positions = validate_vector(
            "positions",
            raw_point["positions"],
            dof,
            idx,
        )

        velocities = validate_vector(
            "velocities",
            raw_point.get("velocities", [0.0] * dof),
            dof,
            idx,
        )

        accelerations = validate_vector(
            "accelerations",
            raw_point.get("accelerations", [0.0] * dof),
            dof,
            idx,
        )

        time_from_start = float(raw_point["time_from_start"])

        if previous_time is not None and time_from_start <= previous_time:
            raise RuntimeError(
                f"Point {idx}: time_from_start must be strictly increasing. "
                f"Previous={previous_time}, current={time_from_start}"
            )

        previous_time = time_from_start

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.accelerations = accelerations
        point.time_from_start = duration_from_float(time_from_start)

        points.append(point)

    return points


class JointTrajectorySender(Node):
    def __init__(
        self,
        topic,
        joint_names,
        points,
        publish_count,
        publish_period_sec,
    ):
        super().__init__("send_joint_trajectory")

        self.publisher = self.create_publisher(
            JointTrajectory,
            topic,
            10,
        )

        self.msg = JointTrajectory()
        self.msg.joint_names = joint_names
        self.msg.points = points

        self.publish_count = 0
        self.max_publish_count = max(1, int(publish_count))

        self.timer = self.create_timer(
            publish_period_sec,
            self.publish_once,
        )

        self.get_logger().info(
            f"Prepared trajectory with {len(joint_names)} joints and "
            f"{len(points)} points. Publishing to '{topic}'."
        )

    def publish_once(self):
        if self.publish_count >= self.max_publish_count:
            self.get_logger().info("Finished publishing joint trajectory.")
            rclpy.shutdown()
            return

        self.publisher.publish(self.msg)
        self.publish_count += 1

        self.get_logger().info(
            f"Published joint trajectory "
            f"({self.publish_count}/{self.max_publish_count})."
        )


def main():
    parser = argparse.ArgumentParser(
        description="Publish a manual N-DOF joint trajectory using runtime active_joint_names.yaml."
    )

    parser.add_argument(
        "--active-joint-names-yaml",
        required=True,
        help="Runtime YAML generated by smm_synthesis containing active_joint_names.",
    )

    parser.add_argument(
        "--trajectory-yaml",
        required=True,
        help="Manual joint trajectory YAML. Contains numerical command values only.",
    )

    parser.add_argument(
        "--trajectory",
        required=True,
        help="Trajectory name inside the manual trajectory YAML.",
    )

    parser.add_argument(
        "--topic",
        default="/smm_joint_controller/reference",
        help="Controller reference topic.",
    )

    parser.add_argument(
        "--publish-count",
        type=int,
        default=5,
        help="How many times to publish the full JointTrajectory message.",
    )

    parser.add_argument(
        "--publish-period-sec",
        type=float,
        default=0.5,
        help="Period between repeated trajectory publishes.",
    )
    parser.add_argument(
        "--interpolation-mode",
        default="",
        choices=["", "sync_cubic", "cubic_hermite"],
        help=(
            "Optional controller interpolation mode. "
            "If provided, the script sets /<controller_name>.trajectory_interpolation_mode before publishing."
        ),
    )

    parser.add_argument(
        "--controller-name",
        default="smm_joint_controller",
        help="Controller node name used when setting interpolation parameter.",
    )

    args = parser.parse_args()

    joint_names = load_active_joint_names(args.active_joint_names_yaml)
    dof = len(joint_names)

    points = load_manual_joint_trajectory(
        trajectory_yaml=args.trajectory_yaml,
        trajectory_name=args.trajectory,
        dof=dof,
    )

    if args.interpolation_mode:
        controller_node = f"/{args.controller_name}"

        print(
            f"[send_joint_trajectory] Setting {controller_node} "
            f"trajectory_interpolation_mode={args.interpolation_mode}"
        )

        subprocess.run(
            [
                "ros2",
                "param",
                "set",
                controller_node,
                "trajectory_interpolation_mode",
                args.interpolation_mode,
            ],
            check=True,
        )

    rclpy.init()

    node = JointTrajectorySender(
        topic=args.topic,
        joint_names=joint_names,
        points=points,
        publish_count=args.publish_count,
        publish_period_sec=args.publish_period_sec,
    )

    rclpy.spin(node)


if __name__ == "__main__":
    main()