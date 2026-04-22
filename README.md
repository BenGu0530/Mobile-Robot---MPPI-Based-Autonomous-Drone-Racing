# MPPI Controller and Simulation

This project consists of a MPPI controller, race environment defintion, and ROS visualization to display the drone racing through the environment.


## MPPI Data Generation

To run the MPPI controller, ensure you have `numpy` and `tqdm` installed in your Python environment. 

```bash
cd src
python mppi_controller.py
```
Results are saved to `trajectory.npz`.

To tweak parameters, edit the `mppi_controller.py` file directly.

The existing files are with the following parameters:
- trajectory-5ms.npz has a target speed of 5 meters/second.
- trajectory-10ms.npz has a target speed of 10 meters/second.
- trajectory-15ms.npz has a target speed of 15 meters/second.
- trajectory-cup-ts22ms.npz has a target speed of 22 meters/second.
- trajectory-ts245ms.npz has a target speed of 24 meters/second.
- trajectory-raceline.npz follows the raceline as closely as possible.

## Simulation Quick Start

This simulation relies on Docker and has been tested successfully on a Macbook Pro with an M3 processor.

### Build and Run

To run the simulation, use the following steps: 

```bash
# Match container devuser UID/GID to your host user to avoid permission issues
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)

# Build the Docker image
docker compose build

# Start the container (publish service ports for noVNC/VNC)
docker compose run --rm --service-ports drone_sim bash

# Access noVNC in browser:
# http://localhost:8080/vnc.html

# Inside container:
colcon build
source /ros2_ws/install/setup.bash

# Launch simulation
ros2 launch drone_simulation simulation.launch.py

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


## Citations

For MPPI: 

[1] G. Williams, A. Aldrich, and E. Theodorou, “Model Predictive Path Integral Control using Covariance Variable Importance Sampling,” Oct. 28, 2015, arXiv: arXiv:1509.01149. doi: 10.48550/arXiv.1509.01149.

