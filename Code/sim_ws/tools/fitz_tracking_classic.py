"""Lateral-only Fitzpatrick Gym trackers with a shared speed-command loop."""

import math

import numpy as np
from scipy import sparse
from scipy.linalg import solve_discrete_are


def _wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def _reference_steering(refs, wheelbase, steer_limit):
    chord = np.linalg.norm(np.diff(refs[:, :2], axis=0), axis=1)
    yaw_change = _wrap(np.diff(refs[:, 2]))
    curvature = np.divide(yaw_change, chord, out=np.zeros_like(chord), where=chord > 1e-4)
    return np.clip(np.arctan(wheelbase * curvature), -steer_limit, steer_limit)


class ClassicController:
    """Return (speed command in m/s, front steering angle in radians).

    refs[j] = [x, y, yaw, v] at j*dt. The first point is the route reference
    at the current progress. Create/reset the controller independently per run.
    """

    def __init__(self, method, params=None):
        p = params or {}
        self.method = {"lmpc": "linear_mpc"}.get(method.lower(), method.lower())
        if self.method not in ("pp", "lqr", "linear_mpc"):
            raise ValueError(f"Unsupported lateral tracker: {method}")
        self.dt = float(p.get("dt", 0.05))
        self.wheelbase = float(p.get("wheelbase", 0.3302))
        self.max_speed = float(p.get("max_speed", 2.0))
        self.max_accel = float(p.get("max_accel", 3.0))
        self.max_decel = float(p.get("max_decel", 3.0))
        self.max_accel_step = float(p.get("max_accel_step", 0.15))
        self.motor_tau = p.get("motor_tau")
        self.qp_backend = p.get("qp_backend", "osqp")
        if self.qp_backend not in ("scipy", "osqp"):
            raise ValueError("Unknown steering QP backend")
        if self.motor_tau is not None and not (math.isfinite(self.motor_tau) and self.motor_tau > 0.):
            raise ValueError("motor_tau must be a positive response time")
        self.max_steer = float(p.get("max_steer", 0.4189))
        self.max_steer_step = self.dt * float(p.get("max_steer_rate", 0.8))
        if min(self.dt, self.wheelbase, self.max_speed, self.max_accel,
               self.max_decel, self.max_steer, self.max_steer_step, self.max_accel_step) <= 0:
            raise ValueError("Fitzpatrick tracker settings must be positive")
        self.q_y, self.q_psi, self.r_delta = 12.0, 3.0, 2.0
        self.solver = None
        self.horizon = None
        if self.method == "lqr":
            self._prepare_lqr()
        self.reset()

    def reset(self, initial_speed=0.0, initial_steer=0.0):
        self.speed_command = float(initial_speed)
        self.accel_command = 0.0
        self.last_steer = float(initial_steer)
        self.last_status = "ready"
        if self.solver is not None:
            self.solver.warm_start(x=np.zeros(self.horizon), y=np.zeros(2 * self.horizon))

    def _prepare_lqr(self):
        # DARE gains for the linearized two-state bicycle along a straight
        # reference. Curvature feedforward supplies the nominal curved motion.
        self.speed_grid = np.linspace(0.15, max(self.max_speed, 0.3), 21)
        self.gains = []
        for speed in self.speed_grid:
            ds = self.dt * speed
            A = np.array([[1.0, ds], [0.0, 1.0]])
            B = np.array([[0.5 * ds * ds], [ds]]) / self.wheelbase
            P = solve_discrete_are(A, B, np.diag([self.q_y, self.q_psi]),
                                   np.array([[self.r_delta]]))
            self.gains.append(np.linalg.solve(self.r_delta + B.T @ P @ B, B.T @ P @ A)[0])
        self.gains = np.asarray(self.gains)

    def _pure_pursuit(self, state, refs):
        # Select a point by route distance, including an interpolated short
        # final segment near the stop. Use its actual car-frame chord below.
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(refs[:, :2], axis=0), axis=1))]
        lookahead = np.clip(0.42 + 0.12 * state["v"], 0.42, 0.85)
        x = np.interp(lookahead, arc, refs[:, 0]) - state["x"]
        y = np.interp(lookahead, arc, refs[:, 1]) - state["y"]
        chord_squared = x*x + y*y
        if chord_squared < 1e-5:
            return 0.0
        lateral = -math.sin(state["yaw"]) * x + math.cos(state["yaw"]) * y
        return math.atan2(2.0 * self.wheelbase * lateral, chord_squared)

    def _lqr(self, state, error, feedforward):
        gain = [np.interp(state["v"], self.speed_grid, self.gains[:, j]) for j in range(2)]
        return feedforward[0] - np.dot(gain, error)

    def _configure_qp(self, n):
        # The constraints and Hessian sparsity remain constant throughout the
        # run. Numerical Hessian/gradient updates reflect the current preview.
        import osqp
        self.horizon = n
        difference = np.eye(n) - np.eye(n, k=-1)
        self.diff_penalty = 3.0 * difference.T @ difference
        upper = sparse.triu(np.ones((n, n)), format="csc")
        self.upper_rows = upper.indices.copy()
        self.upper_cols = np.repeat(np.arange(n), np.diff(upper.indptr))
        upper.data[:] = (self.upper_rows == self.upper_cols).astype(float)
        constraint = sparse.vstack([sparse.eye(n), sparse.csc_matrix(difference)], format="csc")
        self.upper_bounds = np.r_[np.full(n, self.max_steer), np.full(n, self.max_steer_step)]
        self.lower_bounds = -self.upper_bounds.copy()
        self.solver = osqp.OSQP()
        self.solver.setup(P=upper, q=np.zeros(n), A=constraint,
                          l=self.lower_bounds, u=self.upper_bounds, verbose=False,
                          eps_abs=1e-5, eps_rel=1e-5, max_iter=4000)

    def _linear_mpc(self, state, refs, error, feedforward):
        # Condensed LTV kinematic bicycle. Linearize tan(delta) around the
        # nominal curvature at each stage; integrate heading into lateral error.
        n = len(feedforward)
        if self.qp_backend == "osqp" and self.horizon != n:
            self._configure_qp(n)
        if self.qp_backend == "scipy":
            difference = np.eye(n) - np.eye(n, k=-1)
            self.diff_penalty = 3.0 * difference.T @ difference
        speed = np.clip(refs[:-1, 3], 0.0, self.max_speed)
        speed[0] = max(0.0, state["v"])
        ds = self.dt * speed
        gain = ds / (self.wheelbase * np.cos(feedforward)**2)
        distance = np.cumsum(ds)
        psi_map = np.tril(np.broadcast_to(gain, (n, n)))
        y_map = np.tril((distance[:, None] - distance + 0.5 * ds) * gain)
        psi_bias = error[1] - psi_map @ feedforward
        y_bias = error[0] + distance * error[1] - y_map @ feedforward
        hessian = (self.q_y * y_map.T @ y_map + self.q_psi * psi_map.T @ psi_map
                   + self.diff_penalty + self.r_delta * np.eye(n))
        gradient = self.q_y * y_bias @ y_map + self.q_psi * psi_bias @ psi_map
        gradient -= self.r_delta * feedforward
        gradient[0] -= 3.0 * self.last_steer
        if self.qp_backend == "scipy":
            # The same convex QP in steering increments. Foxy ships SciPy and
            # commonly has no OSQP. Bound the cumulative angle and each change.
            from scipy.optimize import minimize
            total = np.tril(np.ones((n, n)))
            def angles(step):
                return self.last_steer + total @ step
            objective = lambda u: .5 * u @ hessian @ u + gradient @ u
            constraint = {"type": "ineq",
                          "fun": lambda d: np.r_[self.max_steer - angles(d),
                                                  self.max_steer + angles(d)],
                          "jac": lambda d: np.vstack((-total, total))}
            solved = minimize(lambda d: objective(angles(d)), np.zeros(n),
                              jac=lambda d: total.T @ (hessian @ angles(d) + gradient),
                              bounds=[(-self.max_steer_step, self.max_steer_step)]*n,
                              constraints=[constraint], method="SLSQP",
                              options={"ftol": 1e-9, "maxiter": 100})
            if not solved.success or not np.isfinite(solved.x).all() or min(constraint["fun"](solved.x)) < -1.e-6:
                raise ValueError("linear MPC QP rejected: " + str(solved.message))
            self.last_status = "solved"
            return float(angles(solved.x)[0])
        self.upper_bounds[n] = self.last_steer + self.max_steer_step
        self.lower_bounds[n] = self.last_steer - self.max_steer_step
        self.solver.update(Px=hessian[self.upper_rows, self.upper_cols], q=gradient,
                           l=self.lower_bounds, u=self.upper_bounds)
        solution = self.solver.solve()
        self.last_status = solution.info.status.lower()
        if self.last_status != "solved":
            raise RuntimeError(f"Fitzpatrick linear MPC: {self.last_status}")
        return float(solution.x[0])

    def step(self, state, refs):
        refs = np.asarray(refs, dtype=float)
        if refs.ndim != 2 or refs.shape[0] < 2 or refs.shape[1] != 4:
            raise ValueError("refs must have shape (N+1, 4), with N >= 1")
        if self.method == "pp":
            steer = self._pure_pursuit(state, refs)
        else:
            dx, dy = state["x"] - refs[0, 0], state["y"] - refs[0, 1]
            yaw = refs[0, 2]
            error = (-math.sin(yaw) * dx + math.cos(yaw) * dy,
                     _wrap(state["yaw"] - yaw))
            feedforward = _reference_steering(refs, self.wheelbase, self.max_steer)
            if self.method == "lqr":
                steer = self._lqr(state, error, feedforward)
            else:
                steer = self._linear_mpc(state, refs, error, feedforward)
        steer = np.clip(steer, self.last_steer - self.max_steer_step,
                        self.last_steer + self.max_steer_step)
        self.last_steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        # Same speed servo for all three lateral methods. Gym consumes a speed
        # setpoint, so rate-limit the published command as for the joint methods.
        reference_accel = (refs[1, 3] - refs[0, 3]) / self.dt
        speed_tau = self.motor_tau or (20.0 / 95.1 if reference_accel >= 0 else 5.0 / 95.1)
        desired = refs[1, 3] + speed_tau * reference_accel
        accel = (reference_accel + 3.0 * (desired - self.speed_command)
                 + 2.0 * (refs[0, 3] - state["v"]))
        if refs[0, 3] <= 1e-5 and refs[1, 3] <= 1e-5:
            accel = -self.max_decel
        accel = np.clip(accel, self.accel_command - self.max_accel_step,
                        self.accel_command + self.max_accel_step)
        accel = np.clip(accel, -self.max_decel, self.max_accel)
        previous = self.speed_command
        self.speed_command = float(np.clip(previous + self.dt * accel, 0., self.max_speed))
        self.accel_command = (self.speed_command - previous) / self.dt
        return self.speed_command, self.last_steer
