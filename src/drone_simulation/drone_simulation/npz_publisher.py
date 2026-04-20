import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import numpy as np


class NpzPublisher(Node):
    """Node that publishes waypoints loaded from a .npz file on the /drone/target_pose topic.
    This node publishes at a rate of 20 Hz."""

    def __init__(self):
        super().__init__("npz_publisher")
        # Declare file name parameter
        self.declare_parameter("waypoints_file", "waypoints.npz")

        self.publisher = self.create_publisher(PoseStamped, "/drone/target_pose", 10)

        self.theta = 0.0
        self.dt = 0.02  # provided by team
        self.timer = self.create_timer(self.dt, self.publish_target)

        waypoints_file = self.get_parameter("waypoints_file").value
        if waypoints_file is None or waypoints_file == "None":
            self.get_logger().error(
                "No waypoints file provided. Set the 'waypoints_file' parameter to a valid .npz file."
            )
            return
        self.waypoint_files = np.load(waypoints_file)
        self.waypoints_pos = self.waypoint_files["pos"]
        self.waypoints_quat = self.waypoint_files["quat"]
        self.num_waypoints = self.waypoints_pos.shape[0]
        self.step = 0

        self.get_logger().info(f"Publishing NPZ waypoints on /drone/target_pose")

    def publish_target(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        pos = self.waypoints_pos[self.step, :]
        quat = self.waypoints_quat[self.step, :]
        self.step += 1
        # Reset number of steps
        if self.step > self.num_waypoints:
            self.step = 0

        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NpzPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
