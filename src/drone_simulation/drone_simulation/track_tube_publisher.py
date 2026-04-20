#!/usr/bin/env python3
"""Publish the race track as a tube visualization in RViz.

Uses RaceEnvironment to generate a spline track and publishes it as:
- A LINE_STRIP marker for the track centerline
- Cylinder markers along the spline to represent the tube volume
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

from .race_environment import RaceEnvironment


class TrackTubePublisher(Node):
    def __init__(self):
        super().__init__("track_tube_publisher")

        # Declare parameters
        self.declare_parameter("seed", 42)
        self.declare_parameter("tube_radius", 5.0)
        self.declare_parameter("cylinder_samples", 20)  # cylinders along the path

        seed = int(self.get_parameter("seed").get_parameter_value().integer_value)
        self.tube_radius = float(
            self.get_parameter("tube_radius").get_parameter_value().double_value
        )
        cylinder_samples = int(
            self.get_parameter("cylinder_samples").get_parameter_value().integer_value
        )

        # Initialize RaceEnvironment
        self.get_logger().info(
            f"Initializing RaceEnvironment(seed={seed}, tube_radius={self.tube_radius})"
        )
        env = RaceEnvironment(seed=seed, tube_radius=self.tube_radius)

        self.waypoints = env.waypoints
        self.tube_radius = env.tube_radius
        self.cylinder_samples = cylinder_samples

        # Publishers
        self.line_pub = self.create_publisher(Marker, "/scene/track_centerline", 10)
        self.cylinders_pub = self.create_publisher(MarkerArray, "/scene/track_tube", 10)

        # Timer to publish at regular interval
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(
            f"Publishing track tube with {len(self.waypoints)} waypoints, "
            f"radius={self.tube_radius:.2f}m, {cylinder_samples} cylinders"
        )

    def publish_markers(self):
        now = self.get_clock().now().to_msg()

        # ===== Publish centerline as LINE_STRIP =====
        line_marker = Marker()
        line_marker.header.stamp = now
        line_marker.header.frame_id = "world"
        line_marker.ns = "track"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1  # line width
        line_marker.color.r = 0.0
        line_marker.color.g = 0.8
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8

        for pt in self.waypoints:
            p = Point()
            p.x = float(pt[0])
            p.y = float(pt[1])
            p.z = float(pt[2])
            line_marker.points.append(p)

        line_marker.lifetime.sec = 0
        line_marker.frame_locked = True
        self.line_pub.publish(line_marker)

        # ===== Publish hollow cylinder markers along the path =====
        marker_array = MarkerArray()
        n_cylinders = min(self.cylinder_samples, len(self.waypoints))

        for i in range(n_cylinders):
            idx = int(i * len(self.waypoints) / n_cylinders)
            pt = self.waypoints[idx]

            # Compute tangent direction for cylinder orientation
            if idx < len(self.waypoints) - 1:
                next_pt = self.waypoints[idx + 1]
                tangent = next_pt - pt
            else:
                prev_pt = self.waypoints[idx - 1]
                tangent = pt - prev_pt

            tangent_norm = np.linalg.norm(tangent)
            if tangent_norm > 1e-8:
                tangent = tangent / tangent_norm

            # Create hollow cylinder as TRIANGLE_LIST
            cylinder = Marker()
            cylinder.header.stamp = now
            cylinder.header.frame_id = "world"
            cylinder.ns = "track_tube"
            cylinder.id = i
            cylinder.type = Marker.TRIANGLE_LIST
            cylinder.action = Marker.ADD

            # Build orthonormal frame
            z_axis = np.array([0.0, 0.0, 1.0])
            if np.abs(np.dot(tangent, z_axis)) > 0.9999:
                right = np.array([1.0, 0.0, 0.0])
            else:
                right = np.cross(z_axis, tangent)
                right = right / (np.linalg.norm(right) + 1e-8)

            up = np.cross(tangent, right)
            up = up / (np.linalg.norm(up) + 1e-8)

            # Generate hollow cylinder surface (sides only, no caps)
            height = 2.0
            n_segments = 16  # segments around the cylinder

            # Top and bottom circle points
            top_pts = []
            bot_pts = []
            half_h = height / 2.0

            for j in range(n_segments):
                angle = 2.0 * np.pi * j / n_segments
                offset = (
                    self.tube_radius * np.cos(angle) * right
                    + self.tube_radius * np.sin(angle) * up
                )
                top_pts.append(pt + tangent * half_h + offset)
                bot_pts.append(pt - tangent * half_h + offset)

            # Create triangle strips along the side
            for j in range(n_segments):
                j_next = (j + 1) % n_segments

                # Triangle 1 (bottom-left, bottom-right, top-left)
                cylinder.points.append(
                    Point(
                        x=float(bot_pts[j][0]),
                        y=float(bot_pts[j][1]),
                        z=float(bot_pts[j][2]),
                    )
                )
                cylinder.points.append(
                    Point(
                        x=float(bot_pts[j_next][0]),
                        y=float(bot_pts[j_next][1]),
                        z=float(bot_pts[j_next][2]),
                    )
                )
                cylinder.points.append(
                    Point(
                        x=float(top_pts[j][0]),
                        y=float(top_pts[j][1]),
                        z=float(top_pts[j][2]),
                    )
                )

                # Triangle 2 (bottom-right, top-right, top-left)
                cylinder.points.append(
                    Point(
                        x=float(bot_pts[j_next][0]),
                        y=float(bot_pts[j_next][1]),
                        z=float(bot_pts[j_next][2]),
                    )
                )
                cylinder.points.append(
                    Point(
                        x=float(top_pts[j_next][0]),
                        y=float(top_pts[j_next][1]),
                        z=float(top_pts[j_next][2]),
                    )
                )
                cylinder.points.append(
                    Point(
                        x=float(top_pts[j][0]),
                        y=float(top_pts[j][1]),
                        z=float(top_pts[j][2]),
                    )
                )

            cylinder.scale.x = 1.0
            cylinder.scale.y = 1.0
            cylinder.scale.z = 1.0

            cylinder.color.r = 0.2
            cylinder.color.g = 0.6
            cylinder.color.b = 0.9
            cylinder.color.a = 0.4

            cylinder.lifetime.sec = 0
            cylinder.frame_locked = True

            marker_array.markers.append(cylinder)

        self.cylinders_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrackTubePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
