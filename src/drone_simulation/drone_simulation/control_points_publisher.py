#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np


class ControlPointsPublisher(Node):
    def __init__(self):
        super().__init__("control_points_publisher")

        self.pub = self.create_publisher(Marker, "/scene/control_points_marker", 10)
        self.timer = self.create_timer(1.0, self.publish_marker)

        # Base control points (same as Env.py)
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

        self.get_logger().info(
            "Publishing control points on /scene/control_points_marker"
        )

    def publish_marker(self):
        now = self.get_clock().now().to_msg()

        m = Marker()
        m.header.stamp = now
        m.header.frame_id = "world"
        m.ns = "scene"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        # diameter of each sphere
        m.scale.x = 0.35
        m.scale.y = 0.35
        m.scale.z = 0.35
        m.color.r = 0.1
        m.color.g = 0.8
        m.color.b = 0.1
        m.color.a = 0.9

        for p in self._BASE_CONTROL_POINTS:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            m.points.append(pt)

        m.lifetime.sec = 0
        m.frame_locked = True

        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ControlPointsPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
