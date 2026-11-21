"""Fixed Fitzpatrick route and single-agent, headless official F1TENTH Gym plant."""

import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


SIM = Path(__file__).resolve().parents[1]
ROUTE_CSV = SIM.parents[1] / "real_vehicle/routes/Fitzpatrick_fixed_start_arc_straight_v2_speed_profile.csv"
MAP_BASE = SIM / "src/f1tenth_gym_ros/maps/Fitzpatrick_sim"
CONTROL_DT = .05
PHYSICS_DT = .01


class Route:
    """One standing-start, standing-finish lap, with unwrapped route progress."""

    def __init__(self, path=ROUTE_CSV):
        route = np.genfromtxt(path, delimiter=",", names=True)
        self.xy = np.column_stack((route["x"], route["y"]))
        self.yaw = np.unwrap(route["yaw"])
        self.speed = route["speed_mps"]
        self.s = route["distance_m"]
        self.length = float(self.s[-1])
        self.edges = np.diff(self.xy, axis=0)
        self.edge_squared = np.sum(self.edges**2, axis=1)
        ds = np.diff(self.s)
        summed_speed = self.speed[:-1] + self.speed[1:]
        if np.any(ds <= 0) or np.any(summed_speed <= 0) or np.any(self.edge_squared <= 0):
            raise ValueError("Route must advance along nonzero segments with finite travel times")
        # Piecewise constant acceleration exactly reproduces every speed in the CSV,
        # including both zero endpoints, without a synthetic startup speed floor.
        self.segment_times = 2 * ds / summed_speed
        self.times = np.r_[0., np.cumsum(self.segment_times)]
        self.accel = np.diff(self.speed) / self.segment_times

    def at_s(self, s):
        s = np.clip(s, 0, self.length)
        x = np.interp(s, self.s, self.xy[:, 0])
        y = np.interp(s, self.s, self.xy[:, 1])
        yaw = np.interp(s, self.s, self.yaw)
        v = np.sqrt(np.maximum(0., np.interp(s, self.s, self.speed**2)))
        return np.stack((x, y, yaw, v), axis=-1)

    def project(self, state, previous_s):
        # The initial and terminal positions coincide. A global nearest point
        # could jump across the entire lap or wrap before stopping.
        start = max(0, self.s.searchsorted(previous_s - 1.) - 1)
        end = min(len(self.edges), self.s.searchsorted(previous_s + 2.) + 1)
        edges = self.edges[start:end]
        relative = np.array([state["x"], state["y"]]) - self.xy[start:end]
        frac = np.clip(np.sum(relative * edges, axis=1) / self.edge_squared[start:end], 0., 1.)
        residual = relative - frac[:, None] * edges
        i = np.argmin(np.sum(residual**2, axis=1))
        j = start + i
        s = self.s[j] + frac[i] * (self.s[j+1] - self.s[j])
        cross = (edges[i, 0]*residual[i, 1] - edges[i, 1]*residual[i, 0]) / math.sqrt(self.edge_squared[j])
        return float(s), float(cross)

    def window(self, s, stages=25, dt=CONTROL_DT):
        s = float(np.clip(s, 0, self.length))
        index = min(len(self.accel)-1, self.s.searchsorted(s, side="right")-1)
        v0 = self.speed[index]
        ds = s - self.s[index]
        v = math.sqrt(max(0., v0*v0 + 2*self.accel[index]*ds))
        elapsed = 2*ds/(v0+v) if ds > 0 else 0.
        t = np.minimum(self.times[index] + elapsed + np.arange(stages)*dt, self.times[-1])
        segments = np.clip(self.times.searchsorted(t, side="right") - 1, 0, len(self.accel)-1)
        u = t - self.times[segments]
        positions = self.s[segments] + self.speed[segments]*u + .5*self.accel[segments]*u*u
        positions[t >= self.times[-1]] = self.length
        return self.at_s(positions)


class Plant:
    def __init__(self, route, initial_offset=0., initial_heading=0., seed=1):
        # pyglet 1.4 imports GL in the official Gym constructor. Disable its
        # implicit display window so a run also works on servers without X11.
        import pyglet
        pyglet.options["shadow_window"] = False
        import gym

        self.env = gym.make("f110_gym:f110-v0", map=str(MAP_BASE), map_ext=".png",
                            num_agents=1, seed=int(seed), timestep=PHYSICS_DT, lidar_dist=.275)
        self.params = self.env.unwrapped.params
        meta = yaml.safe_load(MAP_BASE.with_suffix(".yaml").read_text())
        if meta["origin"][2] != 0.:
            raise ValueError("Fitzpatrick occupancy footprint check requires axis-aligned map")
        self.origin = np.array(meta["origin"][:2])
        self.resolution = float(meta["resolution"])
        with Image.open(MAP_BASE.with_suffix(".png")) as image:
            self.occupied = np.flipud(np.asarray(image.convert("L")) < 250)
        x, y, yaw, _ = route.at_s(0.)
        pose = [x-initial_offset*math.sin(yaw), y+initial_offset*math.cos(yaw), yaw+initial_heading]
        obs, _, _, _ = self.env.reset(np.array([pose]))
        self.state = self._observation(obs)
        # Gym reset itself executes an observation-only zero-command step.
        # Trial time starts with the first measured controller command.
        self._physics_steps = 0

    @property
    def sim_time(self):
        return self._physics_steps * PHYSICS_DT

    def _boundary(self, x, y, yaw):
        # Exact separating-axis overlap of the oriented body with each occupied
        # pixel in its axis-aligned bounding box; unknown cells are non-drivable.
        half_l = self.params["length"] / 2
        half_w = self.params["width"] / 2
        c, s = math.cos(yaw), math.sin(yaw)
        extent = np.array([abs(c)*half_l + abs(s)*half_w, abs(s)*half_l + abs(c)*half_w])
        center = np.array([x, y]) - self.origin
        lo = np.floor((center - extent - 1e-12) / self.resolution).astype(int)
        hi = np.floor((center + extent + 1e-12) / self.resolution).astype(int)
        rows, cols = self.occupied.shape
        if lo[0] < 0 or lo[1] < 0 or hi[0] >= cols or hi[1] >= rows:
            return True
        region = self.occupied[lo[1]:hi[1]+1, lo[0]:hi[0]+1]
        if not region.any():
            return False
        row, col = np.nonzero(region)
        dx = (col + lo[0] + .5)*self.resolution - center[0]
        dy = (row + lo[1] + .5)*self.resolution - center[1]
        pixel_radius = .5*self.resolution*(abs(c)+abs(s))
        return bool(np.any((abs(dx*c+dy*s) <= half_l+pixel_radius) &
                           (abs(-dx*s+dy*c) <= half_w+pixel_radius)))

    def _observation(self, obs):
        x, y, yaw = (float(obs[key][0]) for key in ("poses_x", "poses_y", "poses_theta"))
        return {"x": x, "y": y, "yaw": yaw, "v": float(obs["linear_vels_x"][0]),
                "vy": float(obs["linear_vels_y"][0]), "yaw_rate": float(obs["ang_vels_z"][0]),
                "collision": bool(obs["collisions"][0]), "boundary": self._boundary(x, y, yaw)}

    def step_physics(self, speed, steer):
        action = np.array([[steer, speed]], dtype=float)
        if not np.isfinite(action).all():
            raise ValueError("Non-finite Gym command")
        obs, _, _, _ = self.env.step(action)
        self._physics_steps += 1
        self.state = self._observation(obs)
        return self.state

    def step(self, speed, steer):
        for _ in range(round(CONTROL_DT / PHYSICS_DT)):
            self.step_physics(speed, steer)
            if self.state["collision"] or self.state["boundary"]:
                break
        return self.state

    def close(self):
        self.env.close()
