#!/usr/bin/env python3
"""Race node: publishes waypoint targets from RaceEnvironment to the drone.

The node instantiates `RaceEnvironment` (from `race_environment.py`) and
publishes successive waypoints as `geometry_msgs/PoseStamped` messages on
`/drone/target_pose` so the simulation controller or bridge can consume them.

Parameters (ROS2 declared parameters):
  - seed (int | None): seed passed to `RaceEnvironment` (default: 42)
  - rate_hz (float): publishing frequency in Hz (default: 10.0)
  - use_control_points (bool): publish control points instead of dense waypoints
  - step (int): stride when sampling dense waypoints (default: 30)
  - loop (bool): whether to loop the waypoint sequence (default: True)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from .race_environment import RaceEnvironment


class RaceNode(Node):
    def __init__(self):
        super().__init__("race_node")

        # Parameters
        self.declare_parameter("seed", 42)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("use_control_points", False)
        self.declare_parameter("step", 30)
        self.declare_parameter("loop", True)

        seed = self.get_parameter("seed").get_parameter_value().integer_value
        rate_hz = float(
            self.get_parameter("rate_hz").get_parameter_value().double_value
        )
        self.use_control_points = bool(
            self.get_parameter("use_control_points").get_parameter_value().bool_value
        )
        self.step = int(self.get_parameter("step").get_parameter_value().integer_value)
        self.loop = bool(self.get_parameter("loop").get_parameter_value().bool_value)

        # Instantiate environment
        self.get_logger().info(f"Initializing RaceEnvironment(seed={seed})")
        env = RaceEnvironment(seed=seed)

        if self.use_control_points:
            pts = np.array(env.control_points, dtype=float)
        else:
            # Dense waypoints may be many; sample with stride `step`.
            pts = np.array(env.waypoints, dtype=float)[:: max(1, self.step), :]

        if pts.shape[0] == 0:
            raise RuntimeError("No waypoints available from RaceEnvironment")

        self.waypoints = pts
        self.idx = 0
        self.total = len(self.waypoints)

        # Publisher to the common target topic used in this package
        self.pub = self.create_publisher(PoseStamped, "/drone/target_pose", 10)

        # Timer
        self.rate_hz = rate_hz
        self.timer = self.create_timer(1.0 / max(1e-3, self.rate_hz), self.timer_cb)

        self.get_logger().info(
            f"Publishing {self.total} waypoints to /drone/target_pose at {self.rate_hz} Hz"
        )

    def timer_cb(self):
        p = self.waypoints[self.idx]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.position.z = float(p[2])
        # Neutral orientation
        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)

        self.idx += 1
        if self.idx >= self.total:
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info("Reached final waypoint; stopping timer")
                self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = RaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
