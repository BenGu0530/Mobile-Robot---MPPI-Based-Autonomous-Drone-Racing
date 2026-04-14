from setuptools import find_packages, setup

package_name = "drone_simulation"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/simulation.launch.py"]),
        ("share/" + package_name + "/config", ["config/rviz_config.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Quadcopter simulation controller",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "circle_path_publisher = drone_simulation.circle_path_publisher:main",
            "gazebo_pose_bridge = drone_simulation.gazebo_pose_bridge:main",
            "position_controller = drone_simulation.position_controller:main",
            "tube_marker_publisher = drone_simulation.tube_marker_publisher:main",
            "obstacles_publisher = drone_simulation.obstacles_publisher:main",
            "control_points_publisher = drone_simulation.control_points_publisher:main",
        ],
    },
)
