#!/usr/bin/env python3
"""Generate matplotlib diagrams for all .npz files in the current directory.

Saves images to an 'images' folder with the same name as the .npz file but with .png extension.
Also creates a combined overlay plot of all paths.
"""

import os
import subprocess
import sys
import numpy as np
import matplotlib.pyplot as plt

# Import the constants from matplotlib_viz.py
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

TUBE_RADIUS = 5.0


def catmull_rom_spline(P0, P1, P2, P3, t):
    """Evaluate Catmull-Rom spline at parameter t in [0, 1]."""
    t2 = t * t
    t3 = t2 * t

    q = 0.5 * np.array(
        [
            -t3 + 2 * t2 - t,
            3 * t3 - 5 * t2 + 2,
            -3 * t3 + 4 * t2 + t,
            t3 - t2,
        ]
    )
    return q[0] * P0 + q[1] * P1 + q[2] * P2 + q[3] * P3


def compute_centerline(control_points, samples=200):
    """Generate centerline from control points using Catmull-Rom spline."""
    centerline = []
    n_cp = len(control_points)

    for i in range(n_cp - 1):
        P0 = control_points[(i - 1) % n_cp]
        P1 = control_points[i]
        P2 = control_points[(i + 1) % n_cp]
        P3 = control_points[(i + 2) % n_cp]

        samples_per_segment = samples // (n_cp - 1)
        for j in range(samples_per_segment):
            t = j / samples_per_segment
            point = catmull_rom_spline(P0, P1, P2, P3, t)
            centerline.append(point)

    return np.array(centerline)


def compute_track_boundaries(centerline, tube_radius):
    """Compute inner and outer track boundaries as parallel curves."""
    inner_boundary = []
    outer_boundary = []

    for i in range(len(centerline)):
        pt = centerline[i]

        # Compute tangent using finite differences
        if i == 0:
            tangent = centerline[1] - centerline[0]
        elif i == len(centerline) - 1:
            tangent = centerline[i] - centerline[i - 1]
        else:
            tangent = centerline[i + 1] - centerline[i - 1]

        tangent_norm = np.linalg.norm(tangent[:2])
        if tangent_norm > 1e-8:
            tangent_2d = tangent[:2] / tangent_norm
        else:
            tangent_2d = np.array([1.0, 0.0])

        # Perpendicular in 2D (rotate 90 degrees)
        perp = np.array([-tangent_2d[1], tangent_2d[0]])

        # Compute offset boundaries
        inner_pt = pt.copy()
        outer_pt = pt.copy()
        inner_pt[:2] = pt[:2] + perp * tube_radius
        outer_pt[:2] = pt[:2] - perp * tube_radius

        inner_boundary.append(inner_pt)
        outer_boundary.append(outer_pt)

    return np.array(inner_boundary), np.array(outer_boundary)


def main():
    # Get the directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Create images folder if it doesn't exist
    images_dir = os.path.join(script_dir, "images")
    os.makedirs(images_dir, exist_ok=True)

    # Find all .npz files in the script directory
    npz_files = [f for f in os.listdir(script_dir) if f.endswith(".npz")]

    if not npz_files:
        print("No .npz files found in the directory.")
        return

    # Path to matplotlib_viz.py
    viz_script = os.path.join(script_dir, "matplotlib_viz.py")
    stats_script = os.path.join(script_dir, "matplotlib_stats.py")

    # Generate individual plots
    for npz_file in npz_files:
        npz_path = os.path.join(script_dir, npz_file)
        # Create image filename: same name but .png
        image_name = os.path.splitext(npz_file)[0] + ".png"
        image_path = os.path.join(images_dir, image_name)

        # Call matplotlib_viz.py with --save
        cmd = [sys.executable, viz_script, "--save", image_path, npz_path]
        print(f"Generating image for {npz_file}...")
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error generating image for {npz_file}: {result.stderr}")
        else:
            print(f"Saved {image_path}")

        cmd = [sys.executable, stats_script, "--save", image_path, npz_path]
        print(f"Generating stats image for {npz_file}...")
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error generating image for {npz_file}: {result.stderr}")
        else:
            print(f"Saved {image_path}")

    # Generate combined overlay plot
    print("Generating combined overlay plot...")
    plt.figure(figsize=(14, 10))
    colors = plt.cm.tab10(np.linspace(0, 1, len(npz_files)))

    # Generate and plot track boundaries
    centerline = compute_centerline(BASE_CONTROL_POINTS)
    inner_boundary, outer_boundary = compute_track_boundaries(centerline, TUBE_RADIUS)

    plt.plot(
        inner_boundary[:, 0],
        inner_boundary[:, 1],
        "orange",
        linewidth=2,
        linestyle="-",
        alpha=0.7,
        label="Track Boundary (Inner)",
    )
    plt.plot(
        outer_boundary[:, 0],
        outer_boundary[:, 1],
        "orange",
        linewidth=2,
        linestyle="-",
        alpha=0.7,
        label="Track Boundary (Outer)",
    )

    for i, npz_file in enumerate(npz_files):
        npz_path = os.path.join(script_dir, npz_file)
        data = np.load(npz_path)
        path = data["pos"]
        label = os.path.splitext(npz_file)[0]
        plt.plot(path[:, 0], path[:, 1], color=colors[i], linewidth=2, label=label)

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

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Overlay of All Drone Paths (Top-Down View)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    combined_image_path = os.path.join(images_dir, "all_paths.png")
    plt.savefig(combined_image_path)
    print(f"Saved combined plot to {combined_image_path}")


if __name__ == "__main__":
    main()
