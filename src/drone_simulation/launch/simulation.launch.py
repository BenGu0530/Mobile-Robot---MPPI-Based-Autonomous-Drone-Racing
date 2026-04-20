from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_circle_path = LaunchConfiguration("use_circle_path", default="false")
    use_obstacles = LaunchConfiguration("use_obstacles", default="true")
    use_control_points = LaunchConfiguration("use_control_points", default="true")
    use_track_tube = LaunchConfiguration("use_track_tube", default="true")
    use_race_node = LaunchConfiguration("use_race_node", default="false")
    use_gazebo_bridge = LaunchConfiguration("use_gazebo_bridge", default="true")
    waypoints_file_name = LaunchConfiguration("waypoints_file", default="None")

    drone_sim_dir = get_package_share_directory("drone_simulation")
    drone_models_dir = get_package_share_directory("drone_models")
    rviz_config = os.path.join(drone_sim_dir, "config", "rviz_config.rviz")
    urdf_file = os.path.join(drone_models_dir, "urdf", "quadcopter.urdf")

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints_file",
                default_value="None",
                description="Path to .npz file containing waypoints to publish. "
                "If None, will use default.",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="Start Rviz2"
            ),
            DeclareLaunchArgument(
                "use_circle_path",
                default_value="false",
                description="Publish circular target path on /drone/target_pose",
            ),
            DeclareLaunchArgument(
                "use_obstacles",
                default_value="true",
                description="Publish obstacle markers in RViz",
            ),
            DeclareLaunchArgument(
                "use_control_points",
                default_value="true",
                description="Publish control points in RViz",
            ),
            DeclareLaunchArgument(
                "use_track_tube",
                default_value="true",
                description="Publish the race track as a spline tube in RViz",
            ),
            DeclareLaunchArgument(
                "use_race_node",
                default_value="false",
                description="Start the race_node to publish targets to /drone/target_pose",
            ),
            DeclareLaunchArgument(
                "use_gazebo_bridge",
                default_value="false",
                description="Forward /drone/target_pose commands to Gazebo model pose",
            ),
            # Publish the URDF so RViz RobotModel display works
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            # Position controller node
            Node(
                package="drone_simulation",
                executable="position_controller",
                name="position_controller",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="drone_simulation",
                executable="circle_path_publisher",
                name="circle_path_publisher",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(use_circle_path),
            ),
            Node(
                package="drone_simulation",
                executable="obstacles_publisher",
                name="obstacles_publisher",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(use_obstacles),
            ),
            Node(
                package="drone_simulation",
                executable="control_points_publisher",
                name="control_points_publisher",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(use_control_points),
            ),
            Node(
                package="drone_simulation",
                executable="track_tube_publisher",
                name="track_tube_publisher",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(use_track_tube),
            ),
            Node(
                package="drone_simulation",
                executable="gazebo_pose_bridge",
                name="gazebo_pose_bridge",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(use_gazebo_bridge),
            ),
            Node(
                package="drone_simulation",
                executable="race_node",
                name="race_node",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(use_race_node),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
            Node(
                package="drone_simulation",
                executable="npz_publisher",
                name="npz_publisher",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(
                    PythonExpression(["'", waypoints_file_name, "' != 'None'"])
                ),
                parameters=[{"waypoints_file": waypoints_file_name}],
            ),
        ]
    )
