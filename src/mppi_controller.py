import numpy as np
from quadrotor_simulator_py.quadrotor_model import QuadrotorModel
from Env import RaceEnvironment
from tqdm import tqdm


def compute_hover_rpm(model):
    cT2 = float(model.model_params.motor_model[0])
    cT1 = float(model.model_params.motor_model[1])
    cT0 = float(model.model_params.motor_model[2])
    T_hover = model.model_params.mass * model.model_params.gravity_norm / 4.0
    return (-cT1 + np.sqrt(cT1**2 - 4*cT2*(cT0 - T_hover))) / (2*cT2)


class MPPI:
    def __init__(self, model, env):
        self.model = model
        self.env = env
        self.K = 100
        self.T = 100
        self.dt = 0.02
        self.sigma = 100.0
        self.n_iter = 3
        self.k_progress = 1.0
        self.k_offset = 0.0
        self.max_speed = 20.0

        self.rpm_min = model.model_params.rpm_params.min
        self.rpm_max = model.model_params.rpm_params.max
        self.rpm_hover = compute_hover_rpm(model)
        self.U = np.ones((self.T, 4)) * self.rpm_hover

        stride = max(1, len(env.waypoints) // 200)
        self._wp  = env.waypoints[::stride]
        self._arc = env._arc_lengths[::stride]

    def _get_state(self):
        x = np.zeros(17)
        x[0:3]   = self.model.get_pose().translation().flatten()
        x[3:7]   = self.model.get_pose().quaternion().flatten()
        x[7:10]  = self.model.vw.flatten()
        x[10:13] = self.model.wb.flatten()
        x[13:17] = self.model.rs.flatten()
        return x

    def _ode_batch(self, X, U):
        p = self.model.model_params
        motor = p.motor_model.flatten()

        rpms = X[:, 13:17]
        o = np.stack([rpms**2, rpms, np.ones_like(rpms)], axis=2)
        thrust_per_rotor = o @ motor
        result = (p.mixer @ thrust_per_rotor.T).T
        F = result[:, 0]
        M = result[:, 1:]

        qw, qx, qy, qz = X[:, 3], X[:, 4], X[:, 5], X[:, 6]
        zb = np.stack([
            2*(qx*qz + qw*qy),
            2*(qy*qz - qw*qx),
            1 - 2*(qx**2 + qy**2),
        ], axis=1)

        lin_acc = np.array([0., 0., -p.gravity_norm]) + (F / p.mass)[:, None] * zb

        wb = X[:, 10:13]
        Icm_wb = (p.inertia @ wb.T).T
        ang_acc = (p.inertia_inv @ (-np.cross(wb, Icm_wb) + M).T).T

        dq = 0.5 * np.stack([
            -qx*wb[:, 0] - qy*wb[:, 1] - qz*wb[:, 2],
             qw*wb[:, 0] + qy*wb[:, 2] - qz*wb[:, 1],
             qw*wb[:, 1] - qx*wb[:, 2] + qz*wb[:, 0],
             qw*wb[:, 2] + qx*wb[:, 1] - qy*wb[:, 0],
        ], axis=1)

        w_cur = X[:, 13:17]
        k_m = np.where(U >= w_cur, p.kmotor_u, p.kmotor_d)
        drpm = (w_cur - U) * -k_m

        xdot = np.empty_like(X)
        xdot[:, 0:3]   = X[:, 7:10]
        xdot[:, 3:7]   = dq
        xdot[:, 7:10]  = lin_acc
        xdot[:, 10:13] = ang_acc
        xdot[:, 13:17] = drpm
        return xdot

    def _step_batch(self, X, U):
        dt = self.dt
        k1 = self._ode_batch(X, U)
        k2 = self._ode_batch(X + dt/2 * k1, U)
        k3 = self._ode_batch(X + dt/2 * k2, U)
        k4 = self._ode_batch(X + dt * k3, U)
        X_new = X + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        q = X_new[:, 3:7]
        X_new[:, 3:7] = q / np.linalg.norm(q, axis=1, keepdims=True)
        return X_new

    def _update(self, x0):
        increments = np.random.randn(self.K, self.T, 4) * self.sigma
        eps = np.cumsum(increments, axis=1)
        costs = np.zeros(self.K)
        X = np.tile(x0, (self.K, 1))

        x0_diff = self._wp - x0[0:3]
        x0_progress = self._arc[np.argmin(np.sum(x0_diff**2, axis=1))]

        for t in range(self.T):
            U = np.clip(self.U[t] + eps[:, t, :], self.rpm_min, self.rpm_max)
            X = self._step_batch(X, U)
            pos = X[:, 0:3]
            progress, offset, is_collision = self.env.query(pos)

            dt_elapsed = (t + 1) * self.dt
            progress_rate = np.clip((progress - x0_progress) / (self.max_speed * dt_elapsed), -1.0, 1.0)
            step_cost = -self.k_progress * progress_rate
            step_cost += self.k_offset * (offset / self.env.tube_radius)**2
            collision_penalty = 2.0 * (self.k_progress+self.k_offset) * self.T
            step_cost[is_collision] += collision_penalty
            step_cost[X[:, 2] < 1.5] += collision_penalty
            costs += step_cost

        lam = max(costs.std(), 1e-6)

        k_top = 5   

   
        best_idx = np.argsort(costs)[:k_top]

        costs_top = costs[best_idx]
        eps_top   = eps[best_idx]

        beta = costs_top.min()
        w = np.exp(-(costs_top - beta) / lam)
        w /= w.sum()

        for t in range(self.T):
            self.U[t] = np.clip(
                self.U[t] + w @ eps_top[:, t, :],
                self.rpm_min,
                self.rpm_max
            )

        return costs.min(), self.U

    def compute(self, debug=False):
        x0 = self._get_state()
        for _ in range(self.n_iter):
            best, _ = self._update(x0)
        if debug:
            print(f"    [MPPI] best_cost={best:.2f}  U[0]={self.U[0].astype(int)}")
        u = self.U[0].copy()
        self.U = np.roll(self.U, -1, axis=0)
        self.U[-1] = self.U[-2]
        return u


if __name__ == "__main__":
    import os, sys
    import matplotlib.pyplot as plt

    qs_path = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(qs_path)

    model = QuadrotorModel()
    model.initialize(qs_path + "/config/rocky0704_model_params.yaml")
    model.reset_simulation()

    start = np.array([0.0, 0.0, 2.0])
    model.set_pose(np.array([*start, 1, 0, 0, 0]))

    rpm_hover = compute_hover_rpm(model)
    model.rs = np.ones(4) * rpm_hover
    model.model_params.uRPM = np.ones((4, 1)) * rpm_hover
    print(f"Hover RPM:   {rpm_hover:.1f}")
    print(f"RPM range:   [{model.model_params.rpm_params.min:.1f}, {model.model_params.rpm_params.max:.1f}]")

    env = RaceEnvironment()
    mppi = MPPI(model, env)
    print(f"Track length: {env.total_length:.1f} m")

    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 5))

    stride = 10
    wp  = env.waypoints[::stride]
    tan = env._tangents[::stride]
    perp = np.stack([-tan[:, 1], tan[:, 0]], axis=1)
    perp /= np.linalg.norm(perp, axis=1, keepdims=True).clip(1e-8)
    r = env.tube_radius
    left  = wp[:, :2] + r * perp
    right = wp[:, :2] - r * perp
    ax.plot(left[:, 0],  left[:, 1],  'b-', lw=1, alpha=0.25)
    ax.plot(right[:, 0], right[:, 1], 'b-', lw=1, alpha=0.25)
    ax.plot(wp[:, 0], wp[:, 1], 'b-', lw=1, alpha=0.5)

    for obs in env.obstacles:
        ax.plot(obs["center"][0], obs["center"][1], 'rx', ms=10, mew=2)
    ax.plot(*env.waypoints[0, :2],  'go', ms=8)
    ax.plot(*env.waypoints[-1, :2], 'y*', ms=10)
    ax.set_aspect('equal'); ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    drone_dot, = ax.plot([], [], 'ko', ms=7)
    trail_line, = ax.plot([], [], 'k-', lw=1, alpha=0.4)
    title = ax.set_title('')
    plt.tight_layout()
    fig.show()

    log = {"pos": [], "vel": [], "quat": [], "rpm": [], "u": [], "progress": [], "offset": [], "t": []}

    trail = []
    t, dt = 0.0, 0.02
    for step in tqdm(range(5000)):
        pos = model.get_state().pos.flatten()
        progress, offset, collision = env.query(pos)
        if progress >= env.total_length * 0.99:
            print("Track finished!")
            break
        if collision:
            print("died")
            break
        u = mppi.compute(debug=(step < 3))
        model.apply_command(u)
        t += dt
        model.update(t)

        s = model.get_state()
        log["pos"].append(s.pos.flatten())
        log["vel"].append(s.vel.flatten())
        log["quat"].append(model.get_pose().quaternion().flatten())
        log["rpm"].append(model.rs.flatten())
        log["u"].append(u.copy())
        log["progress"].append(progress)
        log["offset"].append(offset)
        log["t"].append(t)

        if step % 20 == 0:
            speed = np.linalg.norm(s.vel.flatten())
            print(f"  step={step:4d}  pos={pos}  progress={progress:.2f}m  speed={speed:.2f}m/s  u={u.astype(int)}")

        if step % 5 == 0:
            trail.append(pos.copy())
            ta = np.array(trail)
            drone_dot.set_data([pos[0]], [pos[1]])
            trail_line.set_data(ta[:, 0], ta[:, 1])
            pct = 100 * progress / env.total_length
            title.set_text(f"step={step}  {pct:.1f}%  {'COLLISION' if collision else f'off={offset:.1f}m'}")
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

    out = qs_path + "/trajectory.npz"
    np.savez(out, **{k: np.array(v) for k, v in log.items()})
    print(f"Trajectory saved to {out}")

    plt.ioff()
    plt.show()
