"""Single-car kinematic contouring MPC for the Fitzpatrick Gym experiment."""

import casadi as ca
import numpy as np


class ContouringController:
    """Optimize actuator commands and an independent virtual progress speed."""

    def __init__(self, route, params):
        self.route = route
        self.curvature = np.gradient(route.yaw, route.s)
        self.dt = dt = params["dt"]
        self.n = n = 24
        self.command = self.steer = self.accel = 0.
        self.previous = None
        opt = self.opt = ca.Opti()
        # Global position, heading, body speed, commanded speed, steering, arc progress.
        self.x = x = opt.variable(7, n + 1)
        self.u = u = opt.variable(3, n)  # command acceleration, steer rate, progress rate
        self.initial = opt.parameter(7)
        self.last_accel = opt.parameter()
        opt.subject_to(x[:, 0] == self.initial)
        a, steer_rate, progress_rate = (u[j, :] for j in range(3))
        v, cmd, steer, progress = (x[j, :-1] for j in range(3, 7))
        next_cmd = cmd + dt * a
        next_steer = steer + dt * steer_rate
        # Gym speed setpoints have different acceleration and braking responses.
        tau = (12.5 + 7.5 * ca.tanh((next_cmd - v) / .10)) / 95.1
        next_v = v + dt / (dt + tau) * (next_cmd - v)
        mid_v = (v + next_v) / 2
        yaw_rate = mid_v * ca.tan(next_steer) / params["wheelbase"]
        mid_heading = x[2, :-1] + 0.5 * dt * yaw_rate
        next_state = ca.vertcat(x[0, :-1] + dt * mid_v * ca.cos(mid_heading),
                                x[1, :-1] + dt * mid_v * ca.sin(mid_heading),
                                x[2, :-1] + dt * yaw_rate, next_v, next_cmd, next_steer,
                                progress + dt * progress_rate)
        opt.subject_to(x[:, 1:] == next_state)
        cap = params["max_speed"]
        opt.subject_to(opt.bounded(0., x[4, 1:], cap))
        opt.subject_to(opt.bounded(-params["max_steer"], x[5, 1:], params["max_steer"]))
        opt.subject_to(opt.bounded(-params["max_decel"], a, params["max_accel"]))
        opt.subject_to(opt.bounded(-params["max_steer_rate"], steer_rate, params["max_steer_rate"]))
        accel_step = a - ca.horzcat(self.last_accel, a[:, :-1])
        opt.subject_to(opt.bounded(-params["max_accel_step"], accel_step, params["max_accel_step"]))
        opt.subject_to(opt.bounded(0., progress_rate, cap))
        opt.subject_to(x[6, -1] <= route.length)

        # First-order route geometry around the shifted horizon from the previous solve.
        self.geometry = opt.parameter(5, n)
        anchor_s, anchor_x, anchor_y, anchor_heading, curvature = (
            self.geometry[j, :] for j in range(5))
        theta = x[6, 1:]
        shift = theta - anchor_s
        rx = anchor_x + ca.cos(anchor_heading) * shift
        ry = anchor_y + ca.sin(anchor_heading) * shift
        heading = anchor_heading + curvature * shift
        dx, dy = x[0, 1:] - rx, x[1, 1:] - ry
        lag = ca.cos(heading) * dx + ca.sin(heading) * dy
        contour = -ca.sin(heading) * dx + ca.cos(heading) * dy
        # Penalize predicted endpoint overshoot, including residual braking distance.
        remaining = route.length - theta
        braking = ca.fmax(0., x[3, 1:]**2 / (2 * 1.0) - remaining)
        cost = (65 * ca.sumsqr(contour) + 90 * ca.sumsqr(lag)
                + 2 * ca.sumsqr(x[2, 1:] - heading) + 100 * ca.sumsqr(braking)
                + .015 * ca.sumsqr(a) + .12 * ca.sumsqr(steer_rate)
                - 12 * (theta[-1] - x[6, 0]))
        opt.minimize(cost)
        opt.solver("ipopt", {"print_time": False, "expand": True}, {"print_level": 0, "sb": "yes",
                   "max_iter": 80, "mu_strategy": "adaptive", "tol": 1e-5, "bound_relax_factor": 0.})

    def step(self, state, progress):
        heading = self.route.at_s(progress)[2]
        yaw = heading + np.arctan2(np.sin(state["yaw"] - heading), np.cos(state["yaw"] - heading))
        initial = [state["x"], state["y"], yaw, state["v"], self.command, self.steer, progress]
        self.opt.set_value(self.initial, initial)
        self.opt.set_value(self.last_accel, self.accel)
        if self.previous is None:
            self.opt.set_initial(self.x, np.tile(initial, (self.n + 1, 1)).T)
        else:
            states, inputs = self.previous
            self.opt.set_initial(self.x, np.column_stack((states[:, 1:], states[:, -1])))
            self.opt.set_initial(self.u, np.column_stack((inputs[:, 1:], inputs[:, -1])))
        if self.previous is None:
            anchors = np.full(self.n, progress)
        else:
            anchors = np.r_[states[6, 2:], states[6, -1] + self.dt * inputs[2, -1]]
            anchors = np.clip(anchors + progress - states[6, 1], progress, self.route.length)
        ref = self.route.at_s(anchors)
        self.opt.set_value(self.geometry, np.vstack((anchors, ref[:, :3].T,
                           np.interp(anchors, self.route.s, self.curvature))))
        solution = self.opt.solve()
        states, inputs = solution.value(self.x), solution.value(self.u)
        self.previous = states, inputs
        self.command, self.steer = states[4:6, 1]
        self.accel = inputs[0, 0]
        return self.command, self.steer
