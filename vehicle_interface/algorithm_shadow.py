"""Fitzpatrick physical-parameter algorithms for passive capture replay."""
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
V8 = REPO / "real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8"
ASSETS = V8 / "runtime_assets.json"
WORKER = REPO / "build/vehicle_shadow/algorithm_native"
sys.path.insert(0, str(REPO / "Code/sim_ws/tools"))

from fitz_local_paths import DT, MARGIN, PathPlanner
from fitz_local_teb_dwa import dwa_command
from fitz_tracking_classic import ClassicController
from fitz_tracking_plant import Route

TRACKERS = ("pp", "lqr", "linear_mpc", "nmpc", "mppi")
PLANNERS = ("hybrid_astar", "lattice", "rrt_star", "dwa", "teb")
METHODS = TRACKERS + PLANNERS
WHEELBASE, STEER_LIMIT = .25, .269
CURVATURE = math.tan(STEER_LIMIT) / WHEELBASE
LIMITS = dict(dt=DT, wheelbase=WHEELBASE, max_speed=2., max_accel=1., max_decel=.8,
              max_accel_step=.15, max_steer=STEER_LIMIT, max_steer_rate=.8,
              motor_tau=.15, qp_backend="scipy")


class Scene:
    """The v8 physical map clearance threshold and its three-disc obstacle footprint."""
    def __init__(self, assets):
        self.grid = np.asarray(assets["clearance"]).reshape(assets["height"], assets["width"])
        self.origin = np.array(assets["origin"][:2])
        self.res = assets["resolution"]

    def clearance(self, poses, obstacles):
        poses = np.asarray(poses).reshape(-1, 3)
        xy = poses[:, :2]
        cells = np.floor((xy-self.origin)/self.res).astype(int)
        col, row = cells[:, 0], self.grid.shape[0]-1-cells[:, 1]
        inside = (row >= 0) & (col >= 0) & (row < self.grid.shape[0]) & (col < self.grid.shape[1])
        gaps = np.full(len(xy), -.25)
        gaps[inside] = self.grid[row[inside], col[inside]]-.25
        tangent = np.column_stack((np.cos(poses[:, 2]), np.sin(poses[:, 2])))
        for x, y, radius in obstacles:
            residual = xy - (x, y)
            for shift in (-.08, 0., .08):
                gaps = np.minimum(gaps, np.linalg.norm(residual + shift*tangent, axis=1) - .15 - radius)
        return gaps


class Native:
    """Exchange JSONL with a local process that has no ROS symbols."""
    def __init__(self, seed):
        if not WORKER.is_file() or not ASSETS.is_file():
            raise ValueError("build the target-native solver with vehicle_interface/algorithm_build.sh first")
        self.process = subprocess.Popen([str(WORKER), str(ASSETS), str(seed)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True, bufsize=1)
        try:
            if self.call({"method": "ready"}).get("wheelbase_m") != WHEELBASE:
                raise ValueError("unexpected solver model")
        except BaseException:
            self.close()
            raise

    def call(self, request):
        self.process.stdin.write(json.dumps(request, allow_nan=False, separators=(",", ":")) + "\n")
        self.process.stdin.flush()
        response = self.process.stdout.readline()
        if not response:
            raise ValueError("physical solver process exited")
        result = json.loads(response)
        if not result["ok"]:
            raise ValueError(result["reason"])
        return result

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process.stdout.close()


def dense_poses(poses):
    """Check translation and rotation between solver samples."""
    pts = np.asarray(poses)
    if len(pts) < 2:
        return pts
    delta = np.diff(pts, axis=0)
    delta[:, 2] = (delta[:, 2]+np.pi) % (2*np.pi)-np.pi
    steps = np.maximum(1, np.ceil((np.linalg.norm(delta[:, :2], axis=1)+.25*abs(delta[:, 2]))/.02)).astype(int)
    idx = np.repeat(np.arange(len(steps)), steps)
    start = np.repeat(np.cumsum(steps)-steps, steps)
    fractions = (np.arange(len(idx))-start)/steps[idx]
    return np.vstack((pts[:-1][idx] + fractions[:, None]*delta[idx], pts[-1]))


class Algorithm:
    def __init__(self, name, route, scene, seed):
        self.name, self.scene, self.route = name, scene, route
        self.failure = ""
        self.previous = (0., 0.)
        self.controller = ClassicController(name, LIMITS) if name in TRACKERS[:3] else None
        self.planner = (PathPlanner(name, route, scene, seed, max_curvature=CURVATURE, acceleration_limit=.8)
                        if name in PLANNERS and name != "dwa" else None)

    def step(self, state, station, obstacles, native, stamp, result):
        ref = self.route.window(station)
        idx = self.route.s.searchsorted(station)
        # Use geometric segments at the terminal zero-velocity time samples.
        path = self.route.at_s(self.route.s[max(0, idx-2):self.route.s.searchsorted(station+4.)+1])[:, :3]
        if self.planner:
            event_count = len(self.planner.events)
            try:
                ref, planned = self.planner.reference(state, station, obstacles, stamp)
                result["planned"] = bool(planned)
            finally:
                result["planner_invoked"] = len(self.planner.events) > event_count
            if self.planner.cached:
                m = self.planner.cached
                path = np.column_stack((m.xy, m.yaw))[max(0, m.index-2):]
                result["local_path_points"] = len(m.planned)
        if self.name == "dwa":
            rollout = []
            speed, angle = dwa_command(self.route, state, station, self.scene, obstacles, *self.previous,
                                       on_rollout=rollout.append, wheelbase=WHEELBASE, steer_limit=STEER_LIMIT,
                                       acceleration=1., deceleration=.8, require_safe_rollout=True)
            result["planner_invoked"] = True
            predicted = np.vstack(([state[k] for k in ("x", "y", "yaw")], rollout[0])) if rollout else ref[:, :3]
        elif self.controller:
            if (np.linalg.norm(ref[0, :2]-[state["x"], state["y"]]) > .25 or
                    abs(math.remainder(state["yaw"]-ref[0, 2], 2*math.pi)) > .70):
                raise ValueError("fixed-route tracking error outside v8 envelope")
            # Fixed-route trackers have no obstacle avoidance planner.
            # Gate their reference corridor; this is not a motion prediction.
            speed, angle = self.controller.step(state, ref)
            predicted = ref[:, :3]
        else:
            obs = [state[k] for k in ("x", "y", "yaw", "v", "vy", "yaw_rate")]
            response = native.call(dict(method=self.name, state=obs, path=path.tolist(), ref=ref.tolist(),
                                        remaining=float(self.route.length-station)))
            speed, angle = response["speed"], response["steering"]
            predicted = response["prediction"]
        if not np.isfinite([speed, angle]).all() or not (0. <= speed <= 2.00001 and abs(angle) <= STEER_LIMIT+.00001):
            raise ValueError("command outside v8 bounds")
        if not len(predicted) or not np.isfinite(predicted).all():
            raise ValueError("missing or nonfinite preview")
        gap = self.scene.clearance(dense_poses(predicted), obstacles).min()
        result["min_preview_gap_m"] = float(gap)
        if gap < MARGIN:
            raise ValueError("preview footprint clearance")
        self.previous = float(speed), float(angle)
        return self.previous


def validate_snapshot(row, previous):
    if not isinstance(row, dict) or row.get("schema") != "pnc.vehicle.v1":
        raise ValueError("expected pnc.vehicle.v1")
    keys = ("stamp", "scan_stamp", "odom_stamp", "x", "y", "yaw", "v", "vy", "yaw_rate", "laser_offset_x",
            "angle_min", "angle_increment", "range_min", "range_max")
    for k in keys:
        if type(row.get(k)) not in (int, float) or not math.isfinite(row[k]):
            raise ValueError("missing or nonfinite " + k)
    if min(row[k] for k in ("stamp", "scan_stamp", "odom_stamp")) <= 0:
        raise ValueError("invalid source stamp")
    stamps = [row[k] for k in ("stamp", "scan_stamp", "odom_stamp")]
    if max(stamps)-min(stamps) > .525:
        raise ValueError("incoherent stamped sources")
    if previous:
        gap = row["scan_stamp"]-previous["scan_stamp"]
        if gap <= 0 or gap > .1+1.e-6:
            raise ValueError("scan clock reset or replay gap over 100 ms")
        if row["stamp"] < previous["stamp"] or row["odom_stamp"] < previous["odom_stamp"]:
            raise ValueError("pose/odom source clock regressed")
    if not (0 <= row["v"] <= 2.4 and abs(row["vy"]) <= 2.5 and abs(row["yaw_rate"]) <= 8.):
        raise ValueError("odom outside physical limits")
    pose = row.get("scan_pose")
    if not isinstance(pose, list) or len(pose) != 3 or any(type(v) not in (float, int) or not math.isfinite(v) for v in pose):
        raise ValueError("missing scan-time map pose")
    if abs(row["laser_offset_x"]-.27) > 1.e-6 or row["angle_increment"] <= 0 or not 0 <= row["range_min"] < row["range_max"]:
        raise ValueError("invalid laser geometry")
    ranges = row.get("ranges")
    if not isinstance(ranges, list) or not 1 <= len(ranges) <= 10000:
        raise ValueError("invalid ranges")
    if any(type(v) not in (float, int) or not math.isfinite(v) for v in ranges if v is not None):
        raise ValueError("nonfinite beam must be null")
    if not any(v is not None and row["range_min"] <= v <= row["range_max"] for v in ranges):
        raise ValueError("scan has no usable returns")
