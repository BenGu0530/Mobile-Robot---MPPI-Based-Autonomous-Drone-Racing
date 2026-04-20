import os
import numpy as np
import matplotlib.pyplot as plt
import argparse

BASE_CONTROL_POINTS = np.array(
    [
        [0.0, 0.0, 2.0],
        [20.0, 5.0, 2.5],
        [35.0, 14.0, 3.5],  # bend 1
        [50.0, 8.0, 5.0],
        [65.0, -2.0, 4.5],  # bend 2
        [80.0, -12.0, 3.0],  # bend 3
        [90.0, -6.0, 2.5],
        [100.0, 0.0, 2.0],
    ],
    dtype=float,
)

BASE_OBSTACLES = [
    {"cp_idx": 2, "offset": np.array([-2.0, 2.0, 0.5]), "radius": 1.5},
    {"cp_idx": 4, "offset": np.array([2.0, -2.0, 1.0]), "radius": 1.0},
    {"cp_idx": 5, "offset": np.array([-1.5, 1.5, 0.0]), "radius": 1.2},
]


def main():
    parser = argparse.ArgumentParser(description="Visualize drone path from .npz file")
    parser.add_argument("npz_file", help="Path to the .npz file containing the path")
    parser.add_argument("--save", help="Save the plot to a file instead of showing")
    args = parser.parse_args()

    # Load the .npz file
    data = np.load(args.npz_file)
    path = data["pos"]  # Assuming shape (N, 3) for positions

    # Plot the path as a line in XY plane
    plt.figure(figsize=(10, 8))
    plt.plot(path[:, 0], path[:, 1], "b-", linewidth=2, label="Drone Path")

    # Plot control points as green dots
    plt.scatter(
        BASE_CONTROL_POINTS[:, 0],
        BASE_CONTROL_POINTS[:, 1],
        c="green",
        s=50,
        label="Control Points",
    )

    # Plot obstacles as red circles
    for obs in BASE_OBSTACLES:
        center = BASE_CONTROL_POINTS[obs["cp_idx"]] + obs["offset"]
        circle = plt.Circle(
            (center[0], center[1]),
            obs["radius"],
            color="red",
            alpha=0.5,
            label="Obstacle" if obs == BASE_OBSTACLES[0] else "",
        )
        plt.gca().add_patch(circle)

    # Set labels and title
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Drone Path Visualization (Top-Down View)")
    plt.suptitle(os.path.basename(args.npz_file))
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    if args.save:
        plt.savefig(args.save)
        print(f"Plot saved to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
