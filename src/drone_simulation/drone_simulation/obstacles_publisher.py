#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import numpy as np


class ObstaclesPublisher(Node):
    def __init__(self):
        super().__init__("obstacles_publisher")

        self.marker_pub = self.create_publisher(Marker, "/scene/obstacle_marker", 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

        # Base control points (copied from Env.py to match obstacle locations)
        self._BASE_CONTROL_POINTS = np.array(
            [
                [0.0, 0.0, 2.0],
                [20.0, 5.0, 2.5],
                [35.0, 14.0, 3.5],
                [50.0, 8.0, 5.0],
                [65.0, -2.0, 4.5],
                [80.0, -12.0, 3.0],
                [90.0, -6.0, 2.5],
                [100.0, 0.0, 2.0],
            ],
            dtype=float,
        )

        # Obstacles defined relative to a control point index
        self._BASE_OBSTACLES = [
            {"cp_idx": 2, "offset": np.array([-2.0, 2.0, 0.5]), "radius": 1.5},
            {"cp_idx": 4, "offset": np.array([2.0, -2.0, 1.0]), "radius": 1.0},
            {"cp_idx": 5, "offset": np.array([-1.5, 1.5, 0.0]), "radius": 1.2},
        ]

        self.get_logger().info("Publishing obstacle markers on /scene/obstacle_marker")

    def publish_markers(self):
        now = self.get_clock().now().to_msg()

        for i, obs in enumerate(self._BASE_OBSTACLES):
            center = self._BASE_CONTROL_POINTS[obs["cp_idx"]] + obs["offset"]
            radius = float(obs["radius"])

            m = Marker()
            m.header.stamp = now
            m.header.frame_id = "world"
            m.ns = "obstacles"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(center[0])
            m.pose.position.y = float(center[1])
            m.pose.position.z = float(center[2])
            m.scale.x = radius * 2.0
            m.scale.y = radius * 2.0
            m.scale.z = radius * 2.0
            m.color.r = 0.8
            m.color.g = 0.1
            m.color.b = 0.1
            m.color.a = 0.9
            m.lifetime.sec = 0
            m.frame_locked = True

            self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclesPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
