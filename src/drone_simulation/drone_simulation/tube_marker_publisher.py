#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class TubeMarkerPublisher(Node):
    def __init__(self):
        super().__init__("tube_marker_publisher")

        self.marker_pub = self.create_publisher(Marker, "/scene/tube_marker", 10)
        self.timer = self.create_timer(1.0, self.publish_marker)

        # Tube geometry (meters)
        self.outer_diameter = 1.2
        self.height = 3.0
        self.wall_thickness = 0.12

        # Tube position in world frame
        self.center_x = 2.5
        self.center_y = 0.0
        self.center_z = 1.5

        self.get_logger().info("Publishing tube marker on /scene/tube_marker")

    def publish_marker(self):
        now = self.get_clock().now().to_msg()

        inner_diameter = self.outer_diameter - 2.0 * self.wall_thickness
        if inner_diameter <= 0.0:
            # Fallback to solid cylinder if wall thickness is invalid.
            inner_diameter = 0.01

        # Outer shell
        outer = Marker()
        outer.header.stamp = now
        outer.header.frame_id = "world"
        outer.ns = "scene"
        outer.id = 0
        outer.type = Marker.CYLINDER
        outer.action = Marker.ADD
        outer.pose.position.x = self.center_x
        outer.pose.position.y = self.center_y
        outer.pose.position.z = self.center_z
        outer.pose.orientation.y = 0.70710678
        outer.pose.orientation.w = 0.70710678
        outer.scale.x = self.outer_diameter
        outer.scale.y = self.outer_diameter
        outer.scale.z = self.height
        outer.color.r = 0.9
        outer.color.g = 0.5
        outer.color.b = 0.1
        outer.color.a = 0.7
        outer.lifetime.sec = 0
        outer.frame_locked = True

        # Inner void mask (matches default RViz dark background to look hollow)
        inner = Marker()
        inner.header.stamp = now
        inner.header.frame_id = "world"
        inner.ns = "scene"
        inner.id = 1
        inner.type = Marker.CYLINDER
        inner.action = Marker.ADD
        inner.pose.position.x = self.center_x
        inner.pose.position.y = self.center_y
        inner.pose.position.z = self.center_z
        inner.pose.orientation.y = 0.70710678
        inner.pose.orientation.w = 0.70710678
        inner.scale.x = inner_diameter
        inner.scale.y = inner_diameter
        inner.scale.z = self.height + 0.01
        inner.color.r = 0.188
        inner.color.g = 0.188
        inner.color.b = 0.188
        inner.color.a = 1.0
        inner.lifetime.sec = 0
        inner.frame_locked = True

        self.marker_pub.publish(outer)
        self.marker_pub.publish(inner)


def main(args=None):
    rclpy.init(args=args)
    node = TubeMarkerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
