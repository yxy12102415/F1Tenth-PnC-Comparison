"""Fitzpatrick static benchmark: Gym scene, independent scoring geometry and LiDAR map.

Only Scene and the output of LocalPlant.sensed_obstacles belong at the planner
boundary. Scenario definitions, rendered obstacles and truth remain plant-side.
"""
import json
import math
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
from numba import njit

from fitz_tracking_plant import MAP_BASE, SIM, Plant, Route

REPO = SIM.parents[1]
PLACEMENT_FILE = REPO / "real_vehicle/config/Fitzpatrick_two_obstacles_A_B.json"
REPLAY_FILE = REPO / "results/fitzpatrick_v8_20260927/two_obstacle_summary.json"
STAGGER_CASES = ("stagger_left_first", "stagger_right_first")
SINGLE_CASES = ("single_left", "single_right")
SCENARIOS = ("clear", "A", "B", "AB", "AB_recorded", *SINGLE_CASES, *STAGGER_CASES)
LENGTH = .58
WIDTH = .31
RASTER_SCALE = 2


def physical_placements(scenario):
    """Ground truth for rendering/scoring. Designed fixtures are labeled separately."""
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario {scenario!r}; expected {SCENARIOS}")
    if scenario == "clear":
        return []
    if scenario in (*SINGLE_CASES, *STAGGER_CASES):
        # The 11.05 m final straight is wide on both sides here. Mirror only the
        # arrangement: a measured map need not have mirror-symmetric walls.
        route = Route()
        first_sign = 1. if scenario in ("single_left", "stagger_left_first") else -1.
        placed = []
        for name, station, offset in (("S1", 33.5, .18*first_sign),
                                      ("S2", 36.0, -.18*first_sign)):
            x, y, heading, _ = route.at_s(station)
            center = [float(x-offset*math.sin(heading)), float(y+offset*math.cos(heading))]
            placed.append(dict(name=name, center_xy=center, route_distance_m=station,
                               lateral_offset_m=offset, desired_diameter_m=.40,
                               source="designed_simulation_fixture"))
        return placed[:1] if scenario in SINGLE_CASES else placed
    placements = json.loads(PLACEMENT_FILE.read_text())["placements"]
    if scenario == "AB_recorded":
        cases = json.loads(REPLAY_FILE.read_text())
        case = next(c for c in cases if c["case"] == "recorded_response")
        route = Route()
        for placement, center, diameter in zip(placements, case["box_centers"], case["diameters_m"]):
            placement["center_xy"] = center
            placement["desired_diameter_m"] = diameter
            placement["route_distance_m"], _ = route.project(
                {"x": center[0], "y": center[1]}, placement["route_distance_m"])
        return placements
    return [p for p in placements if p["name"] in scenario]


@lru_cache(maxsize=1)
def _background():
    meta = yaml.safe_load(MAP_BASE.with_suffix(".yaml").read_text())
    if meta["origin"][2] != 0:
        raise ValueError("Fitzpatrick map is not axis aligned")
    with Image.open(MAP_BASE.with_suffix(".png")) as image:
        grid = np.flipud(np.asarray(image.convert("L"))).copy()
    origin = np.array(meta["origin"][:2], dtype=float)
    resolution = float(meta["resolution"])
    # Pad to make the map exterior non-drivable as for the exact footprint check.
    edt = distance_transform_edt(np.pad(grid >= 250, 1))[1:-1, 1:-1] * resolution
    return meta, origin, resolution, grid, edt


def prepare_world(scenario, folder):
    """Paint cylinders on a 2x nearest-neighbour copy of the native map for Gym rays."""
    meta, origin, resolution, base_grid, _ = _background()
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    target = folder / f"Fitzpatrick_{scenario}"
    placements = physical_placements(scenario)
    # Keep walls, unknown cells and the world extent fixed across *all* cases.
    grid = np.repeat(np.repeat(base_grid, RASTER_SCALE, axis=0), RASTER_SCALE, axis=1)
    res = resolution / RASTER_SCALE
    for obstacle in placements:
        center = np.array(obstacle["center_xy"]) - origin
        radius = obstacle["desired_diameter_m"] / 2
        lo = np.floor((center-radius)/res).astype(int)
        hi = np.ceil((center+radius)/res).astype(int)
        if np.any(lo < 0) or hi[0] > grid.shape[1] or hi[1] > grid.shape[0]:
            raise ValueError(f"obstacle outside map: {obstacle}")
        y, x = np.ogrid[lo[1]:hi[1], lo[0]:hi[0]]
        inside = (((x+.5)*res-center[0])**2 + ((y+.5)*res-center[1])**2 <= radius**2)
        region = grid[lo[1]:hi[1], lo[0]:hi[0]]
        if not inside.any() or (region[inside] < 250).any():
            raise ValueError(f"invalid obstacle footprint: {obstacle}")
        region[inside] = 0
    Image.fromarray(np.flipud(grid)).save(target.with_suffix(".png"))
    target.with_suffix(".yaml").write_text(yaml.safe_dump(
        dict(meta, image=target.with_suffix(".png").name, resolution=res), sort_keys=False))
    return target


def circle_clearance(poses, circles):
    """Signed true rectangle-to-disc clearance, minimized across discs at each pose."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    circles = np.asarray(circles, dtype=float).reshape(-1, 3)
    nearest = np.full(len(poses), np.inf)
    cos_yaw, sin_yaw = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    for x, y, r in circles:
        dx, dy = x-poses[:, 0], y-poses[:, 1]
        lon = np.abs(dx*cos_yaw + dy*sin_yaw) - LENGTH/2
        lat = np.abs(dy*cos_yaw - dx*sin_yaw) - WIDTH/2
        distance = np.hypot(np.maximum(lon, 0), np.maximum(lat, 0))
        distance += np.minimum(np.maximum(lon, lat), 0)
        nearest = np.minimum(nearest, distance-r)
    return nearest


@njit(cache=True)
def _plan_clearance(poses, obstacles, edt, ox, oy, res, cover_radius):
    """Fused loop so the hundreds of short edge checks in tree search stay cheap."""
    out = np.empty(len(poses))
    grid_error = math.sqrt(2.)*res
    for i in range(len(poses)):
        x, y, heading = poses[i]
        c, s = math.cos(heading), math.sin(heading)
        gap = math.inf
        for index in (-1, 0, 1):
            offset = index * LENGTH/3
            col = math.floor((x + offset*c - ox)/res)
            row = math.floor((y + offset*s - oy)/res)
            if not (0 <= row < edt.shape[0] and 0 <= col < edt.shape[1]):
                gap = -math.inf
                break
            gap = min(gap, edt[row, col] - grid_error - cover_radius)
        for j in range(len(obstacles)):
            dx = obstacles[j, 0]-x
            dy = obstacles[j, 1]-y
            lon = abs(dx*c+dy*s)-LENGTH/2
            lat = abs(dy*c-dx*s)-WIDTH/2
            body = math.hypot(max(lon, 0.), max(lat, 0.)) + min(max(lon, lat), 0.)
            gap = min(gap, body-obstacles[j, 2])
        out[i] = gap
    return out


class Scene:
    """Planner's known static background, without injected test obstacles."""
    def __init__(self):
        _, self.origin, self.resolution, _, self.edt = _background()
        # Three circles covering three entire rectangular sub-bodies, including corners.
        self.cover_radius = math.hypot(LENGTH/6, WIDTH/2)
        # Numba is a dependency of Gym's physics and laser engine. Compile the
        # float64 signature during scene setup, outside measured planning calls.
        self.clearance(np.zeros((1, 3)), np.empty((0, 3)))

    def point_clearance(self, x, y):
        col = np.floor((x - self.origin[0])/self.resolution).astype(int)
        row = np.floor((y - self.origin[1])/self.resolution).astype(int)
        valid = (col >= 0) & (row >= 0) & (col < self.edt.shape[1]) & (row < self.edt.shape[0])
        col = np.clip(col, 0, self.edt.shape[1]-1)
        row = np.clip(row, 0, self.edt.shape[0]-1)
        # Cell-center quantization at the query and occupied-cell corner extent.
        return np.where(valid, self.edt[row, col] - math.sqrt(2)*self.resolution, -np.inf)

    def clearance(self, poses, obstacles):
        poses = np.ascontiguousarray(poses, dtype=float).reshape(-1, 3)
        obstacles = np.ascontiguousarray(obstacles, dtype=float).reshape(-1, 3)
        return _plan_clearance(poses, obstacles, self.edt, *self.origin,
                               self.resolution, self.cover_radius)


class LocalPlant(Plant):
    """Single-car Gym; sensor mapper consumes true noisy Gym scans and ideal Gym pose."""
    def __init__(self, route, map_base, seed=11, initial_offset=0., initial_heading=0.):
        self._injected_centers = np.empty((0, 2))
        self._injected_cell_size = 0.
        self.physics_poses = []
        self.scan = None
        self._tracks = []
        self._mapped_step = -1
        self._background_scene = Scene()
        super().__init__(route, initial_offset, initial_heading, seed)
        if not np.allclose((self.params["length"], self.params["width"]), (LENGTH, WIDTH)):
            raise ValueError("Gym and benchmark vehicle footprints disagree")
        map_base = Path(map_base)
        meta = yaml.safe_load(map_base.with_suffix(".yaml").read_text())
        self._injected_cell_size = float(meta["resolution"])
        with Image.open(map_base.with_suffix(".png")) as image:
            grid = np.flipud(np.asarray(image.convert("L")))
        background = np.repeat(np.repeat(self.occupied, RASTER_SCALE, axis=0), RASTER_SCALE, axis=1)
        rows, cols = np.nonzero((grid < 128) & ~background)
        self._injected_centers = (np.column_stack([cols, rows]) + .5)*self._injected_cell_size + self.origin
        self.env.unwrapped.update_map(str(map_base.with_suffix(".yaml")), ".png")
        car = self.env.unwrapped.sim.agents[0]
        self._lidar_offset = car.lidar_dist
        self._beam_angles = np.linspace(-car.fov/2, car.fov/2, car.num_beams)
        x, y, yaw, _ = route.at_s(0.)
        pose = [x-initial_offset*math.sin(yaw), y+initial_offset*math.cos(yaw), yaw+initial_heading]
        obs, _, _, _ = self.env.reset(np.array([pose]))
        self.physics_poses = []
        self.state = self._observation(obs)

    def _obstacle_contact(self, x, y, yaw):
        # SAT: check axes of each occupied square and of oriented body rectangle.
        c, s = math.cos(yaw), math.sin(yaw)
        half_pixel = self._injected_cell_size/2
        dx, dy = (self._injected_centers - (x, y)).T
        near = ((abs(dx) <= abs(c)*LENGTH/2 + abs(s)*WIDTH/2 + half_pixel) &
                (abs(dy) <= abs(s)*LENGTH/2 + abs(c)*WIDTH/2 + half_pixel))
        if not near.any():
            return False
        dx, dy = dx[near], dy[near]
        pad = half_pixel*(abs(c)+abs(s))
        return bool(np.any((abs(c*dx+s*dy) <= LENGTH/2+pad) &
                           (abs(c*dy-s*dx) <= WIDTH/2+pad)))

    def _observation(self, obs):
        state = super()._observation(obs)
        pose = (state["x"], state["y"], state["yaw"])
        state["raster_obstacle_contact"] = self._obstacle_contact(*pose)
        state["gym_collision"] = state["collision"]
        state["collision"] |= state["raster_obstacle_contact"]
        self.scan = np.asarray(obs["scans"][0], dtype=float)
        self.physics_poses.append(pose)
        return state

    def step(self, speed, steer):
        self.physics_poses = []
        return super().step(speed, steer)

    def sensed_obstacles(self):
        """Segment non-background LiDAR returns; persist static detections for this lap."""
        if self._mapped_step == self._physics_steps:
            return np.asarray(self._tracks).reshape(-1, 3).copy()
        self._mapped_step = self._physics_steps
        state = self.state
        heading = state["yaw"]
        sensor = np.array([state["x"]+self._lidar_offset*math.cos(heading),
                           state["y"]+self._lidar_offset*math.sin(heading)])
        valid = np.isfinite(self.scan) & (self.scan > .04) & (self.scan < 5.)
        indices = np.flatnonzero(valid)
        angles = self._beam_angles[indices]+heading
        hits = sensor + self.scan[indices, None]*np.column_stack((np.cos(angles), np.sin(angles)))
        keep = self._background_scene.point_clearance(*hits.T) > .12
        hits, indices = hits[keep], indices[keep]
        if len(hits):
            breaks = ((np.diff(indices) > 1) | (np.linalg.norm(np.diff(hits, axis=0), axis=1) > .12))
            for cluster in np.split(hits, np.flatnonzero(breaks)+1):
                if len(cluster) < 3:
                    continue
                chord = np.linalg.norm(cluster[-1] - cluster[0])
                if chord > .45:
                    continue
                # Estimate the unseen back of a compact object using its apparent
                # beam width; the radius allowance includes map pixels and noise.
                radius = float(np.clip(chord/2+.02, .045, .25))
                surface = np.median(cluster, axis=0)
                ray = surface - sensor
                center = surface + (radius*.65/max(np.linalg.norm(ray), 1e-8))*ray
                matched = next((i for i, old in enumerate(self._tracks)
                                if np.linalg.norm(old[:2]-center) < .18), None)
                if matched is None:
                    self._tracks.append(np.r_[center, radius])
                else:
                    old = self._tracks[matched]
                    old[:2] = .8*old[:2] + .2*center
                    old[2] = max(old[2], radius)
        return np.asarray(self._tracks).reshape(-1, 3).copy()


def audit_stagger_geometry(scenario):
    """Ground-truth geometry witnesses for the designed layout, outside the planner."""
    if scenario not in STAGGER_CASES:
        raise ValueError("select a designed staggered scenario")
    route, scene = Route(), Scene()
    placements = physical_placements(scenario)
    discs = [p["center_xy"] + [p["desired_diameter_m"]/2] for p in placements]
    stations = np.linspace(31., 39., 8001)
    refs = route.at_s(stations)
    if np.ptp(refs[:, 2]) > 1e-6:
        raise ValueError("witness interval is no longer a straight")
    normal = np.column_stack((-np.sin(refs[:, 2]), np.cos(refs[:, 2])))

    def blend(start, end):
        q = np.clip((stations-start)/(end-start), 0., 1.)
        return q**3*(10+q*(-15+6*q))

    first_sign = np.sign(placements[0]["lateral_offset_m"])
    alternate = -first_sign*.38*(blend(31, 33.5)-2*blend(33.5, 36)+blend(36, 39))
    outside = .73*(blend(31, 33.3)-blend(36.5, 39))
    witness = {}
    for name, offset in (("direct", np.zeros_like(stations)), ("alternating", alternate),
                         ("same_side_left", outside), ("same_side_right", -outside)):
        xy = refs[:, :2]+offset[:, None]*normal
        dp = np.gradient(xy, stations, axis=0, edge_order=2)
        ddp = np.gradient(dp, stations, axis=0, edge_order=2)
        heading = np.arctan2(dp[:, 1], dp[:, 0])
        curvature = ((dp[:, 0]*ddp[:, 1]-dp[:, 1]*ddp[:, 0]) /
                     np.linalg.norm(dp, axis=1)**3)
        poses = np.column_stack((xy, heading))
        witness[name] = dict(minimum_disc_gap_m=float(np.min(circle_clearance(poses, discs))),
                             conservative_wall_gap_m=float(np.min(scene.clearance(poses, []))),
                             maximum_abs_curvature_1pm=float(np.max(abs(curvature))))
    if witness["direct"]["minimum_disc_gap_m"] >= 0:
        raise ValueError("centerline no longer intersects these fixtures")
    for name in ("alternating", "same_side_left", "same_side_right"):
        result = witness[name]
        if (result["minimum_disc_gap_m"] < .065 or result["conservative_wall_gap_m"] < .065
                or result["maximum_abs_curvature_1pm"] > math.tan(.4189)/.3302):
            raise ValueError(f"infeasible {scenario} witness: {name}")
    return dict(scenario=scenario, placements=placements, sample_spacing_m=.001,
                start_s_m=31., end_s_m=39., geometry_only=True, paths=witness)


if __name__ == "__main__":
    print(json.dumps([audit_stagger_geometry(case) for case in STAGGER_CASES], indent=2))
