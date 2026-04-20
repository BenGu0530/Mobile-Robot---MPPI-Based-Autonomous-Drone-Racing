# Quadcopter Drone Simulation (Docker)

Lightweight ROS 2 drone simulation with Docker containerization.

## Quick Start

### Build and Run

```bash
# Match container devuser UID/GID to your host user to avoid permission issues
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)

# Build the Docker image
docker compose build

# Start the container (publish service ports for noVNC/VNC)
docker compose run --rm --service-ports drone_sim bash

# noVNC in browser:
# http://localhost:8080/vnc.html

# Inside container:
colcon build
source /ros2_ws/install/setup.bash

# Launch simulation
ros2 launch drone_simulation simulation.launch.py

# Launch simulation with built-in circular target publisher
ros2 launch drone_simulation simulation.launch.py use_circle_path:=true

# Headless/container-safe launch (no RViz)
ros2 launch drone_simulation simulation.launch.py use_rviz:=false

# To run with loading in a trajectory from NPZ file: 
ros2 launch drone_simulation simulation.launch.py npz_publisher:=[filename].npz
# Example using the raceline file
ros2 launch drone_simulation simulation.launch.py waypoints_file:=src/trajectory-raceline.npz

# Manual target publish mode (default: use_circle_path:=false)
# In another terminal (docker exec):
docker exec -it drone_sim bash
ros2 topic pub /drone/target_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "world"}, pose: {position: {x: 1.0, y: 1.0, z: 1.0}, orientation: {w: 1.0}}}'
```

Container processes run as `devuser` (non-root), so workspace artifacts in mounted folders
are created with your host UID/GID instead of root.

## Additional Notes
docker-compose --profile gazebo up
