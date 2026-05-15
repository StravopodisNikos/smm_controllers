#!/usr/bin/env python3

import argparse
import os
import subprocess
import time
import yaml

import rclpy
from rclpy.node import Node
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


class JointErrorPlotLauncher(Node):
    def __init__(self, controller_name, dof):
        super().__init__("plot_joint_pos_error_launcher")

        self.controller_name = controller_name
        self.dof = dof

        self.received = [False] * dof
        self.latest_values = [0.0] * dof
        self.subs = []

        for i in range(dof):
            topic = f"/{controller_name}/q_error_{i}"

            sub = self.create_subscription(
                Float64,
                topic,
                lambda msg, idx=i: self.error_callback(msg, idx),
                10,
            )

            self.subs.append(sub)

        self.get_logger().info("Waiting for first q_error messages:")
        for i in range(dof):
            self.get_logger().info(f"  /{controller_name}/q_error_{i}")

    def error_callback(self, msg, idx):
        self.received[idx] = True
        self.latest_values[idx] = float(msg.data)

    def all_received(self):
        return all(self.received)


def main():
    parser = argparse.ArgumentParser(
        description="Wait for q_error data, then launch rqt_plot."
    )

    parser.add_argument(
        "--active-joint-names-yaml",
        required=True,
        help="Runtime YAML containing active_joint_names.",
    )

    parser.add_argument(
        "--controller-name",
        default="smm_joint_controller",
        help="Controller name used for q_error topics.",
    )

    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=30.0,
        help="Maximum wait time for first q_error messages.",
    )

    args = parser.parse_args()

    joint_names = load_active_joint_names(args.active_joint_names_yaml)
    dof = len(joint_names)

    rclpy.init()
    node = JointErrorPlotLauncher(args.controller_name, dof)

    start_time = time.time()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        if node.all_received():
            node.get_logger().info("Received q_error data from all joints.")
            break

        if time.time() - start_time > args.timeout_sec:
            node.get_logger().error("Timeout while waiting for q_error data.")
            rclpy.shutdown()
            return

    rqt_plot_args = [
        f"/{args.controller_name}/q_error_{i}"
        for i in range(dof)
    ]

    node.get_logger().info("Starting rqt_plot with:")
    for arg in rqt_plot_args:
        node.get_logger().info(f"  {arg}")

    rclpy.shutdown()

    cmd = ["ros2", "run", "rqt_plot", "rqt_plot", *rqt_plot_args]

    print("\n[plot_joint_pos_error] Launching rqt_plot command:")
    print(" ".join(cmd))
    print("")

    process = subprocess.Popen(
        cmd,
        stdout=None,
        stderr=None,
        stdin=None,
        start_new_session=True,
    )

    print(f"[plot_joint_pos_error] rqt_plot started with PID={process.pid}")

    # Give the GUI process time to detach/open before this helper exits.
    time.sleep(2.0)


if __name__ == "__main__":
    main()