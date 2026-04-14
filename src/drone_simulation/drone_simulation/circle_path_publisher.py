#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class CirclePathPublisher(Node):
    def __init__(self):
        super().__init__("circle_path_publisher")

        self.publisher = self.create_publisher(PoseStamped, "/drone/target_pose", 10)

        # Circular trajectory settings.
        self.radius = 1.5
        self.altitude = 1.0
        self.angular_speed = 0.35  # rad/s
        self.center_x = 0.0
        self.center_y = 0.0

        self.theta = 0.0
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.publish_target)

        self.get_logger().info(
            f"Publishing circular targets on /drone/target_pose (r={self.radius}, z={self.altitude}, w={self.angular_speed})"
        )

    def publish_target(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.position.x = self.center_x + self.radius * math.cos(self.theta)
        msg.pose.position.y = self.center_y + self.radius * math.sin(self.theta)
        msg.pose.position.z = self.altitude

        # Keep a neutral orientation for the simple controller.
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)

        self.theta += self.angular_speed * self.dt
        if self.theta > 2.0 * math.pi:
            self.theta -= 2.0 * math.pi


def main(args=None):
    rclpy.init(args=args)
    node = CirclePathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
