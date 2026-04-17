"""
env.py - Race track environment for MPPI drone racing

Defines a 3D Catmull-Rom spline race track with sphere obstacles.
Supports optional randomization via seed parameter.

Public API:
    env = RaceEnvironment(seed=42)   # reproducible
    env = RaceEnvironment(seed=None) # randomized

    # Single point query
    progress, offset, is_collision = env.query(np.array([x, y, z]))

    # Batch query (N x 3)
    progress, offset, is_collision = env.query(np.array([[x1,y1,z1], [x2,y2,z2], ...]))

    # For visualization
    env.waypoints      # (N, 3) dense points along the spline
    env.control_points # (K, 3) Catmull-Rom control points
    env.obstacles      # list of {"center": np.array, "radius": float}
    env.tube_radius    # float
"""

import numpy as np


# ---------------------------------------------------------------------------
# Catmull-Rom spline helpers
# ---------------------------------------------------------------------------


def _catmull_rom_segment(p0, p1, p2, p3, t):
    """
    Evaluate a single Catmull-Rom segment at parameter t in [0, 1].
    Uses the standard centripetal formulation (alpha=0.5).
    """
    t2 = t * t
    t3 = t2 * t
    return 0.5 * (
        (2.0 * p1)
        + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
    )


def _catmull_rom_segment_deriv(p0, p1, p2, p3, t):
    """First derivative of a Catmull-Rom segment w.r.t. t."""
    t2 = t * t
    return 0.5 * (
        (-p0 + p2)
        + (4.0 * p0 - 10.0 * p1 + 8.0 * p2 - 2.0 * p3) * t
        + (-3.0 * p0 + 9.0 * p1 - 9.0 * p2 + 3.0 * p3) * t2
    )


def _build_spline(control_points, samples_per_segment=200):
    """
    Build a dense polyline approximation of the Catmull-Rom spline.

    For K control points we have K-3 interior segments
    (using phantom endpoints so the curve passes through all K points).

    Returns:
        pts   : (N, 3) dense waypoints
        tangents : (N, 3) unit tangents at each waypoint
        arc_lengths : (N,) cumulative arc length at each waypoint
    """
    cp = np.array(control_points, dtype=float)
    # Phantom endpoints to make the curve pass through first and last control point
    phantom_start = 2.0 * cp[0] - cp[1]
    phantom_end = 2.0 * cp[-1] - cp[-2]
    cp_ext = np.vstack([phantom_start, cp, phantom_end])

    n_segments = len(cp_ext) - 3  # = len(cp) - 1
    pts = []
    tangents = []

    for i in range(n_segments):
        p0, p1, p2, p3 = cp_ext[i], cp_ext[i + 1], cp_ext[i + 2], cp_ext[i + 3]
        # Don't repeat the endpoint of each segment (it's the start of the next)
        ts = np.linspace(0, 1, samples_per_segment, endpoint=(i == n_segments - 1))
        for t in ts:
            pts.append(_catmull_rom_segment(p0, p1, p2, p3, t))
            d = _catmull_rom_segment_deriv(p0, p1, p2, p3, t)
            norm = np.linalg.norm(d)
            tangents.append(d / norm if norm > 1e-8 else np.array([1.0, 0.0, 0.0]))

    pts = np.array(pts)
    tangents = np.array(tangents)

    # Cumulative arc length
    diffs = np.diff(pts, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    arc_lengths = np.concatenate([[0.0], np.cumsum(seg_lengths)])

    return pts, tangents, arc_lengths


# ---------------------------------------------------------------------------
# RaceEnvironment
# ---------------------------------------------------------------------------


class RaceEnvironment:
    """
    A 3D drone racing environment with a Catmull-Rom spline track and
    sphere obstacles.

    Args:
        seed (int or None): Random seed for track/obstacle perturbation.
                            Use a fixed seed for reproducible development,
                            None for randomized tracks.
        tube_radius (float): Radius of the racing tube. is_collision is True
                             if the drone's offset from the path exceeds this.
        drone_radius (float): Abstracted drone sphere radius for collision
                              detection with obstacles.
        samples_per_segment (int): Spline resolution for arc length queries.
    """

    # Base control points defining the nominal track shape (~100m long, S-curve)
    _BASE_CONTROL_POINTS = np.array(
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

    # Base obstacle definitions: (offset from nearest control point, radius)
    # stored as (control_point_index, local_offset_xyz, base_radius)
    _BASE_OBSTACLES = [
        {"cp_idx": 2, "offset": np.array([-2.0, 2.0, 0.5]), "radius": 1.5},
        {"cp_idx": 4, "offset": np.array([2.0, -2.0, 1.0]), "radius": 1.0},
        {"cp_idx": 5, "offset": np.array([-1.5, 1.5, 0.0]), "radius": 1.2},
    ]

    def __init__(
        self,
        seed=42,
        tube_radius=5.0,
        drone_radius=0.2,
        samples_per_segment=300,
    ):
        self.tube_radius = tube_radius
        self.drone_radius = drone_radius
        self.seed = seed

        rng = np.random.default_rng(seed)

        # --- Perturb control points ---
        cp = self._BASE_CONTROL_POINTS.copy()
        if seed is not None or True:
            # Always apply perturbation; magnitude controlled by rng
            perturbation_scale = 3.0  # meters
            # Keep start and end fixed, perturb interior points
            noise = rng.uniform(-1, 1, cp.shape) * perturbation_scale
            noise[0] = 0.0  # fix start
            noise[-1] = 0.0  # fix end
            # Keep Z perturbations smaller
            noise[:, 2] *= 0.4
            cp = cp + noise

        self.control_points = cp

        # --- Build spline ---
        self.waypoints, self._tangents, self._arc_lengths = _build_spline(
            cp, samples_per_segment=samples_per_segment
        )
        self.total_length = self._arc_lengths[-1]

        # --- Build obstacles ---
        self.obstacles = []
        for obs_def in self._BASE_OBSTACLES:
            base_center = cp[obs_def["cp_idx"]] + obs_def["offset"]
            # Small random jitter on position and radius
            pos_jitter = rng.uniform(-1.0, 1.0, 3)
            radius_jitter = rng.uniform(-0.2, 0.2)
            self.obstacles.append(
                {
                    "center": base_center + pos_jitter,
                    "radius": max(0.5, obs_def["radius"] + radius_jitter),
                }
            )

    # ------------------------------------------------------------------
    # Public query API
    # ------------------------------------------------------------------

    def query(self, points):
        """
        Query the environment for one or more 3D points.

        Args:
            points: array-like, shape (3,) for single point or (N, 3) for batch

        Returns:
            progress     : float or (N,) array — arc length from start to
                           closest point on the spline
            offset       : float or (N,) array — perpendicular distance from
                           point to the spline
            is_collision : bool or (N,) bool array — True if drone sphere
                           intersects any obstacle OR offset > tube_radius
        """
        points = np.asarray(points, dtype=float)
        single = points.ndim == 1
        if single:
            points = points[None, :]  # (1, 3)

        progress, offset = self._query_spline(points)
        is_collision = self._query_collision(points, offset)

        if single:
            return float(progress[0]), float(offset[0]), bool(is_collision[0])
        return progress, offset, is_collision

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _query_spline(self, points):
        """
        For each point in (N, 3), find the closest waypoint on the spline
        and return (progress, offset).

        Uses vectorized nearest-neighbor search over the dense waypoint array.
        """
        # points: (N, 3), waypoints: (M, 3)
        # Compute squared distances: (N, M)
        diff = self.waypoints[None, :, :] - points[:, None, :]  # (N, M, 3)
        sq_dist = np.sum(diff**2, axis=2)  # (N, M)

        nearest_idx = np.argmin(sq_dist, axis=1)  # (N,)
        progress = self._arc_lengths[nearest_idx]  # (N,)
        offset = np.sqrt(sq_dist[np.arange(len(points)), nearest_idx])  # (N,)

        return progress, offset

    def _query_collision(self, points, offset):
        """
        is_collision = True if:
          (a) offset > tube_radius  (out of racing tube), or
          (b) distance to any obstacle center < obstacle_radius + drone_radius
        """
        # Tube collision
        tube_collision = offset > self.tube_radius

        # Obstacle collision
        obs_collision = np.zeros(len(points), dtype=bool)
        for obs in self.obstacles:
            dist_to_obs = np.linalg.norm(points - obs["center"], axis=1)
            obs_collision |= dist_to_obs < (obs["radius"] + self.drone_radius)

        return tube_collision | obs_collision

    # ------------------------------------------------------------------
    # Convenience accessors for visualization
    # ------------------------------------------------------------------

    def get_track_info(self):
        """
        Returns a dict with everything needed for visualization.

        Keys:
            waypoints      : (N, 3) dense spline points
            control_points : (K, 3) Catmull-Rom control points
            obstacles      : list of {"center": np.array(3,), "radius": float}
            tube_radius    : float
            total_length   : float
        """
        return {
            "waypoints": self.waypoints,
            "control_points": self.control_points,
            "obstacles": self.obstacles,
            "tube_radius": self.tube_radius,
            "total_length": self.total_length,
        }


# ---------------------------------------------------------------------------
# Quick sanity check
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    env = RaceEnvironment(seed=42)
    print(f"Track length: {env.total_length:.2f} m")
    print(f"Waypoints:    {len(env.waypoints)}")
    print(f"Obstacles:    {len(env.obstacles)}")

    # Single point on the track start
    p = env.control_points[0]
    prog, off, coll = env.query(p)
    print(f"\nQuery at start {p}:")
    print(f"  progress={prog:.2f}m  offset={off:.3f}m  collision={coll}")

    # Batch query
    pts = np.array(
        [
            [50.0, 5.0, 4.0],
            [200.0, 0.0, 0.0],  # way outside tube
            env.obstacles[0]["center"],  # inside obstacle
        ]
    )
    progress, offset, is_collision = env.query(pts)
    print("\nBatch query:")
    for i, (pg, of, co) in enumerate(zip(progress, offset, is_collision)):
        print(f"  pt{i}: progress={pg:.2f}m  offset={of:.3f}m  collision={co}")
