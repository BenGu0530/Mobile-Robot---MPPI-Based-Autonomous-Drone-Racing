#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class DroneController(Node):
    def __init__(self):
        super().__init__("position_controller")

        # Current drone position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Current velocity
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Subscribe to target pose commands from teammates' controller
        self.target_pose_sub = self.create_subscription(
            PoseStamped, "/drone/target_pose", self.target_pose_callback, 10
        )

        # Subscribe to velocity commands (alternative interface)
        self.target_vel_sub = self.create_subscription(
            Twist, "/drone/cmd_vel", self.target_vel_callback, 10
        )

        # Publisher for current drone state
        self.drone_state_pub = self.create_publisher(PoseStamped, "/drone/pose", 10)
        self.drone_path_pub = self.create_publisher(Path, "/drone/path", 10)

        # Keep a bounded path history so RViz can render a trail line.
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"
        self.max_path_points = 1000

        # TF broadcaster (publish drone position in TF tree)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parameter: threshold distance to detect a discontinuity (meters).
        # If a jump larger than this is observed between consecutive poses,
        # the stored Path is cleared to avoid RViz drawing a long connector
        # line between laps or teleports.
        self.declare_parameter("jump_threshold", 5.0)
        self.jump_threshold = float(
            self.get_parameter("jump_threshold").get_parameter_value().double_value
        )

        # Timer for publishing state
        self.timer = self.create_timer(0.02, self.publish_state)

        self.get_logger().info("Position controller initialized")

    def target_pose_callback(self, msg: PoseStamped):
        """Receive target pose from teammates' controller"""
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        self.o_x = msg.pose.orientation.x
        self.o_y = msg.pose.orientation.y
        self.o_z = msg.pose.orientation.z
        self.o_w = msg.pose.orientation.w

        self.get_logger().info(
            f"Target pose received: [{self.x:.2f}, {self.y:.2f}, {self.z:.2f}]"
        )

    def target_vel_callback(self, msg: Twist):
        """Receive velocity commands"""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z

        # Simple velocity integration (basic Euler integration)
        dt = 0.1
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

    def publish_state(self):
        """Publish current drone state and TF transform"""
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = self.z
        pose_msg.pose.orientation.w = 1.0

        self.drone_state_pub.publish(pose_msg)

        # Update and publish movement history as nav_msgs/Path.
        self.path_msg.header.stamp = pose_msg.header.stamp
        # Detect large discontinuities and reset path to avoid drawing
        # a connector line between the last point of the previous run
        # and the first point of the next run.
        if len(self.path_msg.poses) > 0:
            last = self.path_msg.poses[-1].pose.position
            dx = pose_msg.pose.position.x - last.x
            dy = pose_msg.pose.position.y - last.y
            dz = pose_msg.pose.position.z - last.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > self.jump_threshold:
                # Clear historical poses to start a new, unconnected trail.
                self.path_msg.poses = []

        self.path_msg.poses.append(pose_msg)
        if len(self.path_msg.poses) > self.max_path_points:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_points :]
        self.drone_path_pub.publish(self.path_msg)

        # Broadcast TF transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
