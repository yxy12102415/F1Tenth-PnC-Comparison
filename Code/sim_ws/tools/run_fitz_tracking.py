#!/usr/bin/env python3
"""Fixed-route, single-car controller comparison in the official F1TENTH Gym."""
import argparse
import contextlib
import csv
import gzip
import hashlib
import importlib.metadata
import json
import math
import inspect
import platform
import random
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import f110_gym

from fitz_tracking_classic import ClassicController
from fitz_tracking_plant import Plant, Route

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parents[1]
NATIVE = SIM / "build/fitz_tracking/fitz_tracking_joint"
ASSETS = SIM / "takeover_assets/Fitzpatrick_sim.json"
ROUTE_FILE = REPO / "real_vehicle/routes/Fitzpatrick_fixed_start_arc_straight_v2_speed_profile.csv"
METHODS = ("pp", "lqr", "linear_mpc", "nmpc", "mppi")
NAMES = dict(zip(METHODS, ("Pure Pursuit", "LQR", "Linear MPC", "NMPC", "MPPI")))
CASES = {"center": (0., 0.), "left": (.10, 4.), "right": (-.10, -4.)}
DT = .05
BUDGET_MS = 45.
PARAMS = dict(dt=DT, wheelbase=.3302, max_speed=2., max_accel=3., max_decel=3.,
              max_steer=.4189, max_steer_rate=.8, max_accel_step=.15)
END = dict(remaining_m=.15, distance_m=.20, yaw_rad=.18, speed_mps=.10, dwell_s=.50)
COLUMNS = ("t_s", "x", "y", "yaw", "v", "vy", "yaw_rate", "s_m", "cte_m",
           "heading_error_rad", "ref_v_mps", "ref_x", "ref_y", "cmd_v_mps", "cmd_delta_rad",
           "controller_ms", "call_ms", "collision", "boundary")


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")


def git_revision(directory):
    result = subprocess.run(["git", "-C", str(directory), "rev-parse", "--verify", "HEAD"],
                            capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else None


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


@contextlib.contextmanager
def controller(method, seed, folder):
    if method in ("pp", "lqr", "linear_mpc"):
        ctrl = ClassicController(method, PARAMS.copy())

        def command(state, ref, remaining):
            start = time.perf_counter()
            speed, steer = ctrl.step(state, ref)
            return speed, steer, (time.perf_counter() - start) * 1000.
        yield command
    else:
        with (folder / "solver.log").open("w") as log:
            process = subprocess.Popen([str(NATIVE), method, str(ASSETS), "2.0", str(seed)],
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=log, text=True, bufsize=1)

            def command(state, ref, remaining, *, local_geometry=None, sensed_discs=None):
                obs = [float(state[k]) for k in ("x", "y", "yaw", "v", "vy", "yaw_rate")]
                data = {"state": obs, "ref": ref.tolist(), "remaining_m": float(remaining)}
                if local_geometry is not None:
                    data["local_geometry"] = local_geometry.tolist()
                if sensed_discs is not None:
                    data["sensed_discs"] = sensed_discs.tolist()
                process.stdin.write(json.dumps(data, allow_nan=False) + "\n")
                process.stdin.flush()
                line = process.stdout.readline()
                if not line:
                    raise RuntimeError(f"native controller exited ({process.poll()})")
                ret = json.loads(line)
                if not ret["ok"]:
                    raise RuntimeError(ret.get("error", "native rejected command"))
                return ret["speed"], ret["steer"], ret["compute_ms"]
            try:
                yield command
            finally:
                process.stdin.close()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                process.stdout.close()
        if not (folder / "solver.log").stat().st_size:
            (folder / "solver.log").unlink()


def trial(route, method, case, seed, folder, timeout):
    folder.mkdir(parents=True)
    offset, angle = CASES[case]
    plant = Plant(route, initial_offset=offset, initial_heading=math.radians(angle), seed=seed)
    state = plant.state
    s, cte = route.project(state, 0.)
    goal = route.at_s(route.length)
    samples, ticks_at_goal = [], 0
    reason, detail = "time_limit", ""
    failure_ms = None
    try:
        with controller(method, seed, folder) as step:
            for _ in range(round(timeout / DT)):
                if state["collision"] or state["boundary"]:
                    reason = "collision" if state["collision"] else "boundary"
                    break
                reference = route.window(s)
                start = time.perf_counter()
                try:
                    speed, steer, solver_ms = step(state, reference, route.length - s)
                    if not all(math.isfinite(x) for x in (speed, steer, solver_ms)):
                        raise ValueError("nonfinite control")
                    if not (-1e-5 <= speed <= PARAMS["max_speed"] + 1e-5 and
                            abs(steer) <= PARAMS["max_steer"] + 1e-5):
                        raise ValueError(f"out-of-range command {speed:.5f}, {steer:.5f}")
                except (RuntimeError, ValueError, OSError) as exc:
                    failure_ms = (time.perf_counter() - start) * 1000.
                    reason, detail = "controller_failure", str(exc)
                    break
                call_ms = (time.perf_counter() - start) * 1000.
                state = plant.step(speed, steer)
                s, cte = route.project(state, s)
                ref = route.at_s(s)
                heading = wrap(state["yaw"] - ref[2])
                samples.append((plant.sim_time, state["x"], state["y"], state["yaw"], state["v"],
                                state["vy"], state["yaw_rate"], s, cte, heading, ref[3],
                                ref[0], ref[1], speed, steer, solver_ms, call_ms, int(state["collision"]), int(state["boundary"])))
                if state["collision"] or state["boundary"]:
                    reason = "collision" if state["collision"] else "boundary"
                    break
                at_goal = (s >= route.length - END["remaining_m"] and
                           math.hypot(state["x"]-goal[0], state["y"]-goal[1]) <= END["distance_m"] and
                           abs(wrap(state["yaw"]-goal[2])) <= END["yaw_rad"] and
                           abs(state["v"]) <= END["speed_mps"])
                ticks_at_goal = ticks_at_goal + 1 if at_goal else 0
                if ticks_at_goal >= round(END["dwell_s"] / DT):
                    reason = "finished"
                    break
    finally:
        plant.close()
    with gzip.open(folder / "trace.csv.gz", "wt", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        writer.writerows(samples)
    result = dict(method=method, case=case, seed=seed, status=reason, detail=detail,
                  simulated_s=plant.sim_time, progress_m=float(s), collision=bool(state["collision"]), boundary=bool(state["boundary"]),
                  final_speed_mps=float(state["v"]), failed_call_ms=failure_ms,
                  final_distance_m=math.hypot(state["x"]-goal[0], state["y"]-goal[1]))
    if samples:
        data = dict(zip(COLUMNS, np.array(samples).T))
        def rms(x):
            return float(np.sqrt(np.mean(np.square(x))))
        ms = data["controller_ms"]
        result.update(cte_rmse_cm=100*rms(data["cte_m"]),
                      cte_p95_cm=float(np.percentile(abs(data["cte_m"]),95))*100,
                      heading_rmse_deg=math.degrees(rms(data["heading_error_rad"])),
                      speed_rmse_mps=rms(data["v"]-data["ref_v_mps"]),
                      steer_rate_rms_rad_s=rms(np.diff(data["cmd_delta_rad"], prepend=0.)/DT),
                      peak_actual_speed_mps=float(max(data["v"])),
                      max_abs_delta_step_rad=float(max(abs(np.diff(data["cmd_delta_rad"], prepend=0.)))),
                      compute_p95_ms=float(np.percentile(ms,95)),
                      call_p95_ms=float(np.percentile(data["call_ms"],95)),
                      over_budget_steps=int(np.count_nonzero(ms>BUDGET_MS)),
                      samples=len(samples))
    save(folder / "summary.json", result)
    return result


def aggregate(rows):
    groups = []
    for method in METHODS:
        trials = [r for r in rows if r["method"] == method]
        if not trials:
            continue
        good = [r for r in trials if r["status"] == "finished"]
        metrics = ("simulated_s", "cte_rmse_cm", "speed_rmse_mps", "steer_rate_rms_rad_s",
                   "compute_p95_ms", "call_p95_ms", "heading_rmse_deg")
        item = {"method":method, "finished":len(good), "total":len(trials),
                "collisions":sum(t["collision"] for t in trials),
                "boundary_violations":sum(t["boundary"] for t in trials),
                "failures":sum(t["status"]=="controller_failure" for t in trials),
                "over_budget_accepted_steps":sum(t.get("over_budget_steps",0) for t in trials)}
        for key in metrics:
            item["mean_trial_"+key] = float(np.mean([t[key] for t in good])) if good else None
        groups.append(item)
    return groups


def plot(out, route, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    dest = out / "figures"
    dest.mkdir()
    line = route.at_s(np.linspace(0, route.length, 1200))
    for method in dict.fromkeys(r["method"] for r in rows):
        fig, ax = plt.subplots(figsize=(8,5))
        ax.plot(line[:,0], line[:,1], c=".50", lw=3, label="Reference")
        for case in CASES:
            r = next((a for a in rows if a["method"]==method and a["case"]==case),None)
            if not r or not r.get("samples"):
                continue
            source = out / f"{method}_{case}_{r['seed']}" / "trace.csv.gz"
            trace = np.atleast_1d(np.genfromtxt(source, delimiter=",", names=True))
            ax.plot(trace["x"],trace["y"],lw=1.1,label=f"{case} ({r['status']})")
        ax.set(xlabel="x (m)", ylabel="y (m)", aspect="equal", title=f"Fitzpatrick fixed route | {NAMES[method]}")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(dest / f"{method}.png", dpi=170)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=list(METHODS))
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--seeds", nargs="+", type=int, default=[11, 29])
    parser.add_argument("--output", type=Path, default=SIM / "results/fitz_tracking" /
                        datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    parser.add_argument("--time-limit", type=float, default=55.)
    args = parser.parse_args()
    if min(args.seeds) < 0 or not math.isfinite(args.time_limit) or args.time_limit < DT:
        parser.error("specify nonnegative seeds and a positive simulation duration")
    for values in (args.seeds, args.methods, args.cases):
        if len(values) != len(set(values)):
            parser.error("duplicate method, case or seed")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    route = Route()
    order_rng = random.Random(20261120)
    order = []
    for seed in args.seeds:
        for case in args.cases:
            block = list(args.methods)
            order_rng.shuffle(block)
            order.extend((name,case,seed) for name in block)
    def filehash(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()
    model_dir = REPO/"real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8"
    sources = [ROUTE_FILE, SIM/"run_fitz_tracking.sh", *sorted((model_dir/"src/cpp_frenet").glob("*.hpp")),
               SIM/"src/takeover_sim/include/takeover_sim/dynamic_overtaking.hpp", SIM/"src/f1tenth_gym_ros/maps/Fitzpatrick_sim.png",
               SIM/"src/f1tenth_gym_ros/maps/Fitzpatrick_sim.yaml", *sorted((SIM/"tools").glob("*fitz_tracking*")),
               SIM/"src/takeover_sim/include/takeover_sim/mppi_control.hpp",
               REPO/"real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/generate_joint_solver.py"]
    if NATIVE.exists() and any(m in ("nmpc","mppi") for m in args.methods):
        sources.extend([ASSETS,NATIVE])
    gym_module = Path(inspect.getfile(f110_gym)).resolve().parent
    save(out/"manifest.json", dict(route_length_m=route.length, control_dt_s=DT, physics_dt_s=.01,
         method_order=order, cases_offset_m_heading_deg=CASES, params=PARAMS, end=END,
         gym_git=git_revision(gym_module),
         gym_package=f110_gym.__name__, observer="Gym pose and longitudinal speed / yaw rate",
         reference_policy="project current position onto unwrapped arc; read shared 25-stage reference; stopped finish",
         noise="no injected pose noise; nominal classic and NMPC runs are deterministic by design",
         sim_timeout_s=args.time_limit, measured_budget_ms=BUDGET_MS, platform=platform.platform(),
         python=sys.version, commit=git_revision(REPO),
         versions={k:importlib.metadata.version(k) for k in ("gym","f110-gym","numpy","scipy","osqp")},
         sha256={str(p.relative_to(REPO)):filehash(p) for p in sources if p.is_file()}))
    print(f"Output: {out}", flush=True)
    rows = []
    for method, case, seed in order:
        row = trial(route, method, case, seed, out/f"{method}_{case}_{seed}", args.time_limit)
        rows.append(row)
        print(f"{method:10s} {case:6s} {seed:3d}: {row['status']}  "
              f"{row['progress_m']:.2f} m  {row['simulated_s']:.2f} s  {row['detail']}",flush=True)
        save(out/"trials.json", rows)
    summary = aggregate(rows)
    with (out/"comparison.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=summary[0])
        w.writeheader()
        w.writerows(summary)
    plot(out,route,rows)
    return int(any(r["status"]!="finished" for r in rows))


if __name__ == "__main__":
    sys.exit(main())
