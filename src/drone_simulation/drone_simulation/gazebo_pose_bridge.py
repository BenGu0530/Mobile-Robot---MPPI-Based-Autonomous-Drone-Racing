#!/usr/bin/env python3

import subprocess

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GazeboPoseBridge(Node):
    def __init__(self):
        super().__init__("gazebo_pose_bridge")

        self.world_name = "quadcopter_world"
        self.model_name = "quadcopter"

        self.sub = self.create_subscription(
            PoseStamped, "/drone/target_pose", self.target_pose_callback, 10
        )

        self.get_logger().info(
            "Gazebo pose bridge enabled: /drone/target_pose -> /world/quadcopter_world/set_pose"
        )

    def target_pose_callback(self, msg: PoseStamped):
        req = (
            f'name: "{self.model_name}" '
            f"position {{ x: {msg.pose.position.x} y: {msg.pose.position.y} z: {msg.pose.position.z} }} "
            f"orientation {{ x: {msg.pose.orientation.x} y: {msg.pose.orientation.y} z: {msg.pose.orientation.z} w: {msg.pose.orientation.w} }}"
        )

        cmd = [
            "ign",
            "service",
            "-s",
            f"/world/{self.world_name}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "200",
            "--req",
            req,
        ]

        try:
            # Lightweight fire-and-forget call to update model pose in Gazebo.
            subprocess.run(
                cmd,
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=0.5,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warning("Timed out sending pose to Gazebo")


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
