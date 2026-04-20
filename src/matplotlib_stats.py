#!/usr/bin/env python3
"""Matplotlib script to display drone trajectory statistics over time.

Displays two figures:
  Figure 1 (Position & Motors):
    - x position over time
    - y position over time
    - z position over time
    - RPM over time (per motor)

  Figure 2 (Dynamics):
    - velocity magnitude over time
    - acceleration over time
    - jerk over time

Data is loaded from an NPZ file with keys: "t" (time), "pos" (positions Nx3),
"vel" (velocity Nx3), and "rpm" (motor RPM Nx4).
"""

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os


def main():
    parser = argparse.ArgumentParser(
        description="Visualize drone trajectory statistics from .npz file"
    )
    parser.add_argument(
        "npz_file", help="Path to the .npz file containing trajectory data"
    )
    parser.add_argument(
        "--save",
        help="Save the plots to files instead of showing (base name for files)",
    )
    args = parser.parse_args()

    # Load the .npz file
    data = np.load(args.npz_file)
    t = data["t"]  # Time array
    pos = data["pos"]  # Position array (N, 3) with x, y, z
    vel_vec = data["vel"]  # Velocity vector (N, 3) with vx, vy, vz
    rpm = data["rpm"]  # RPM data (N, 4) for 4 motors

    # Calculate velocity magnitude
    vel = np.linalg.norm(vel_vec, axis=1)

    # Extract x, y, z
    x = pos[:, 0]
    y = pos[:, 1]
    z = pos[:, 2]

    # Calculate derivatives (always)
    # Acceleration: derivative of velocity magnitude with respect to time
    acceleration = np.abs(np.gradient(vel, t))
    # Jerk: derivative of acceleration with respect to time
    jerk = np.abs(np.gradient(acceleration, t))

    filename = os.path.basename(args.npz_file)

    # ===== FIGURE 1: Position & Motor RPM =====
    fig1, axes1 = plt.subplots(4, 1, figsize=(12, 12))

    # Plot x over time
    axes1[0].plot(t, x, "b-", linewidth=1.5)
    axes1[0].set_xlabel("Time (s)")
    axes1[0].set_ylabel("X Position (m)")
    axes1[0].set_title("X Position over Time")
    axes1[0].grid(True)

    # Plot y over time
    axes1[1].plot(t, y, "g-", linewidth=1.5)
    axes1[1].set_xlabel("Time (s)")
    axes1[1].set_ylabel("Y Position (m)")
    axes1[1].set_title("Y Position over Time")
    axes1[1].grid(True)

    # Plot z over time
    axes1[2].plot(t, z, "r-", linewidth=1.5)
    axes1[2].set_xlabel("Time (s)")
    axes1[2].set_ylabel("Z Position (m)")
    axes1[2].set_title("Z Position over Time")
    axes1[2].grid(True)

    # Plot RPM over time (one line per motor)
    colors_rpm = ["red", "blue", "green", "orange"]
    motor_labels = ["Motor 1", "Motor 2", "Motor 3", "Motor 4"]
    for i in range(rpm.shape[1]):
        axes1[3].plot(
            t, rpm[:, i], color=colors_rpm[i], linewidth=1.5, label=motor_labels[i]
        )
    axes1[3].set_xlabel("Time (s)")
    axes1[3].set_ylabel("RPM")
    axes1[3].set_title("Motor RPM over Time")
    axes1[3].legend()
    axes1[3].grid(True)

    fig1.suptitle(f"Position & Motors: {filename}")
    fig1.tight_layout()

    # ===== FIGURE 2: Velocity & Derivatives =====
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 9))

    # Plot velocity over time
    axes2[0].plot(t, vel, "m-", linewidth=1.5)
    axes2[0].set_xlabel("Time (s)")
    axes2[0].set_ylabel("Velocity (m/s)")
    axes2[0].set_title("Velocity Magnitude over Time")
    axes2[0].grid(True)

    # Plot acceleration over time
    axes2[1].plot(t, acceleration, "orange", linewidth=1.5)
    axes2[1].set_xlabel("Time (s)")
    axes2[1].set_ylabel("Acceleration (m/s²)")
    axes2[1].set_title("Acceleration (Derived) over Time")
    axes2[1].grid(True)

    # Plot jerk over time
    axes2[2].plot(t, jerk, "brown", linewidth=1.5)
    axes2[2].set_xlabel("Time (s)")
    axes2[2].set_ylabel("Jerk (m/s³)")
    axes2[2].set_title("Jerk (Derived)over Time")
    axes2[2].grid(True)

    fig2.suptitle(f"Velocity & Derivatives: {filename}")
    fig2.tight_layout()

    if args.save:
        # Generate filenames for both figures
        base_name = os.path.splitext(args.save)[0]
        fig1_path = f"{base_name}_position.png"
        fig2_path = f"{base_name}_dynamics.png"

        fig1.savefig(fig1_path)
        fig2.savefig(fig2_path)
        print(f"Figures saved to {fig1_path} and {fig2_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
