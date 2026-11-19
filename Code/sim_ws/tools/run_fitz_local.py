#!/usr/bin/env python3
"""Single-car Fitzpatrick static-obstacle benchmark in the official Gym."""
import argparse
import contextlib
import csv
import gzip
import hashlib
import json
import importlib.metadata
import math
import platform
import random
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from fitz_local_paths import (DT, MARGIN, REJOIN_LENGTH, GROUP_DISTANCE, PathPlanner)
from fitz_local_scene import LocalPlant, Scene, SCENARIOS, physical_placements, prepare_world
from fitz_local_teb_dwa import dwa_command
from fitz_tracking_plant import MAP_BASE, Route, ROUTE_CSV
from run_fitz_tracking import ASSETS, NATIVE, REPO, SIM, END, controller, git_revision, save, wrap

METHODS = ("hybrid_astar", "lattice", "rrt_star", "dwa", "teb")
BUDGET_MS = DT * 1000.
TRACE = ("t", "x", "y", "yaw", "v", "s", "cte", "heading_error", "speed_cmd", "steer_cmd",
         "perception_ms", "planner_ms", "controller_ms", "pipeline_ms", "n_observed",
         "planned", "active_path", "collision", "boundary","gym_collision","raster_obstacle_contact")


class Evaluation:
    """World placements used only after observation, independently of the planner."""
    def __init__(self, placements, params):
        self.half = np.array([params["length"], params["width"]])/2
        self.previous = (0., 0.)
        self.events = [dict(name=p["name"], center=p["center_xy"], radius=p["desired_diameter_m"]/2,
                            station=p["route_distance_m"], minimum=None, rejoin_s=None,
                            first_detection_s=None, passing_offset_m=None, dwell=0,
                            passed_station=False, rejoin_required=True)
                       for p in sorted(placements,key=lambda p:p["route_distance_m"])]
        for first, second in zip(self.events, self.events[1:]):
            first["rejoin_required"] = second["station"]-first["station"] > GROUP_DISTANCE

    def observe(self, s, obstacles):
        for e in self.events:
            if e["first_detection_s"] is None and len(obstacles):
                if np.min(np.linalg.norm(obstacles[:, :2]-e["center"], axis=1)) < .25:
                    e["first_detection_s"] = float(s)

    def update(self, poses, s, cte, heading):
        for e in self.events:
            dx, dy = (np.asarray(e["center"])-poses[:, :2]).T
            cos, sin = np.cos(poses[:, 2]), np.sin(poses[:, 2])
            q = np.abs(np.column_stack((dx*cos+dy*sin, dy*cos-dx*sin)))-self.half
            dist = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(np.max(q, axis=1), 0) - e["radius"]
            gap = float(np.min(dist))
            e["minimum"] = gap if e["minimum"] is None else min(gap, e["minimum"])
            old_s, old_cte = self.previous
            if e["passing_offset_m"] is None and old_s <= e["station"] <= s and s > old_s:
                f = (e["station"]-old_s)/(s-old_s)
                e["passing_offset_m"] = float(old_cte+f*(cte-old_cte))
            if s >= e["station"]+.55:
                e["passed_station"] = True
            if e["rejoin_required"] and e["station"]+.55 <= s <= e["station"]+4.0:
                e["dwell"] = e["dwell"]+1 if abs(cte)<=.12 and abs(heading)<=.16 else 0
                if e["rejoin_s"] is None and e["dwell"] >= 5:
                    e["rejoin_s"] = float(s)
        self.previous = (s, cte)


def trial(route, scene, map_base, method, case, seed, folder, time_limit, view=None):
    folder.mkdir()
    plant = LocalPlant(route, map_base, seed=seed)
    state = plant.state
    s, cte = route.project(state, 0.)
    judge = Evaluation(physical_placements(case), plant.params)
    engine = PathPlanner(method, route, scene, seed)
    reference_goal = route.at_s(route.length)
    samples, measurements = [], []
    terminal_timing = None
    dwell, prev_v, prev_delta = 0, 0., 0.
    dwa_rollout = []
    status, detail = "timeout", ""
    try:
        context = contextlib.nullcontext() if method == "dwa" else controller("nmpc", seed, folder)
        with context as track:
            window_open = view is None or view.update(state, plant.sim_time, s, None)
            for tick in range(int(time_limit / DT)):
                if not window_open:
                    status = "user_closed"
                    break
                planned = False
                p_ms = c_ms = sen_ms = 0.
                phase = "perception"
                previous_events = len(engine.events)
                begin = time.perf_counter()
                try:
                    obstacles = np.asarray(plant.sensed_obstacles(), dtype=float).reshape(-1, 3)
                    sensed = time.perf_counter()
                    sen_ms = 1000*(sensed-begin)
                    phase = "planner"
                    if method == "dwa":
                        speed, steer = dwa_command(route, state, s, scene, obstacles, prev_v, prev_delta,
                                                   on_rollout=dwa_rollout.append if view is not None else None)
                    elif method == "unmodified_reference":
                        ref = route.window(s)
                    else:
                        ref, planned = engine.reference(state, s, obstacles, plant.sim_time)
                    planned_at = time.perf_counter()
                    p_ms = (planned_at-sensed)*1000
                    phase = "controller"
                    if method != "dwa":
                        geometry = None
                        if engine.cached is not None:
                            m = engine.cached
                            geometry = np.column_stack((m.xy, m.yaw))[max(0,m.index-4):]
                        discs = None if method == "unmodified_reference" else obstacles
                        speed, steer, _ = track(state, ref, route.length-s,
                                                local_geometry=geometry, sensed_discs=discs)
                        c_ms = (time.perf_counter()-planned_at)*1000
                    if not np.isfinite([speed,steer]).all() or not (-.0001 <= speed <= 2.0001 and abs(steer) <= .419):
                        raise ValueError("invalid control command")
                    total = (time.perf_counter()-begin)*1000
                except (ValueError, RuntimeError, OSError) as exc:
                    total = (time.perf_counter()-begin)*1000
                    if phase == "planner":
                        p_ms = total-sen_ms
                    if phase == "controller":
                        c_ms = total-sen_ms-p_ms
                    measurements.append((sen_ms,p_ms,c_ms,total))
                    terminal_timing = dict(tick=tick, t=plant.sim_time, phase=phase,
                                           perception_ms=sen_ms, planner_ms=p_ms,
                                           controller_ms=c_ms, pipeline_ms=total)
                    status, detail = phase+"_rejection", str(exc)
                    break
                finally:
                    # An accepted path can be rejected by the controller before any
                    # physical step. Save timing with the request on both exit paths.
                    if len(engine.events) > previous_events:
                        engine.events[-1]["planner_ms"] = p_ms
                measurements.append((sen_ms,p_ms,c_ms,total))
                judge.observe(s, obstacles)
                state = plant.step(speed, steer)
                s, cte = route.project(state, s)
                heading = wrap(state["yaw"]-route.at_s(s)[2])
                judge.update(np.asarray(plant.physics_poses), s, cte, heading)
                samples.append((plant.sim_time,state["x"],state["y"],state["yaw"],state["v"],s,cte,
                                heading,speed,steer,sen_ms,p_ms,c_ms,total,len(obstacles),int(planned),
                                int(engine.cached is not None),int(state["collision"]),int(state["boundary"]),
                                int(state["gym_collision"]),int(state["raster_obstacle_contact"])))
                prev_v, prev_delta = speed, steer
                if view is not None:
                    path = dwa_rollout.pop() if dwa_rollout else getattr(engine.cached, "planned", None)
                    window_open = view.update(state, plant.sim_time, s, path)
                if state["collision"] or any(e["minimum"] <= 0 for e in judge.events):
                    status = "collision"
                    break
                if state["boundary"]:
                    status = "boundary"
                    break
                reached = (route.length-s <= END["remaining_m"] and
                           np.linalg.norm(reference_goal[:2]-[state["x"],state["y"]]) <= END["distance_m"] and
                           abs(heading) <= END["yaw_rad"] and abs(state["v"]) <= END["speed_mps"])
                dwell = dwell+1 if reached else 0
                if dwell*DT+1.e-9 >= END["dwell_s"]:
                    completed = all(e["passed_station"] and (not e["rejoin_required"] or e["rejoin_s"] is not None)
                                    for e in judge.events)
                    status = "passed" if completed else "missing_rejoin"
                    break
    finally:
        plant.close()
    with gzip.open(folder/"trace.csv.gz", "wt", newline="") as f:
        w = csv.writer(f)
        w.writerow(TRACE)
        w.writerows(samples)
    save(folder/"plans.json", engine.events)
    times = np.asarray(measurements)
    plan_calls = [event["planner_ms"] for event in engine.events]
    if method == "dwa":
        plan_calls = [row[11] for row in samples]
        if terminal_timing is not None and terminal_timing["phase"] != "perception":
            plan_calls.append(terminal_timing["planner_ms"])
    minimum = [e["minimum"] for e in judge.events if e["minimum"] is not None]
    result = dict(method=method, scenario=case, seed=seed, status=status, detail=detail,
                  progress_m=float(s), simulated_s=plant.sim_time, encounters=judge.events,
                  min_body_gap_m=min(minimum, default=None), command_ticks=len(samples),
                  reached_stop=dwell*DT+1.e-9 >= END["dwell_s"], active_plan_calls=len(plan_calls),
                  terminal_tick_timing=terminal_timing,
                  planning_call_p95_ms=float(np.percentile(plan_calls,95)) if plan_calls else None,
                  pipeline_p95_ms=float(np.percentile(times[:,3],95)) if len(times) else None,
                  pipeline_max_ms=float(np.max(times[:,3])) if len(times) else None,
                  deadline_misses=int(np.sum(times[:,3]>BUDGET_MS)) if len(times) else 0,
                  collision=bool(status=="collision"), boundary=bool(state["boundary"]),
                  steer_rate_rms_rad_s=float(np.sqrt(np.mean(np.diff([0.]+[r[9] for r in samples])**2))/DT) if samples else None)
    save(folder/"summary.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="Show one live trial; default: lattice / stagger_left_first / seed 11.")
    parser.add_argument("--methods", nargs="+", choices=(*METHODS, "unmodified_reference"))
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS)
    parser.add_argument("--seeds", nargs="+", type=int)
    parser.add_argument("--output", type=Path, default=SIM/"results"/"fitz_local"/datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    parser.add_argument("--time-limit", type=float, default=55.)
    args = parser.parse_args()
    args.methods = args.methods or (["lattice"] if args.gui else list(METHODS))
    args.scenarios = args.scenarios or (["stagger_left_first"] if args.gui else [c for c in SCENARIOS if c != "AB_recorded"])
    args.seeds = args.seeds or ([11] if args.gui else [11,29])
    if args.gui and any(len(group) != 1 for group in (args.methods, args.scenarios, args.seeds)):
        parser.error("--gui shows one trial at a time: select one method, scenario and seed")
    if not math.isfinite(args.time_limit) or args.time_limit < DT or min(args.seeds)<0:
        parser.error("invalid seed or time limit")
    if any(len(set(x))!=len(x) for x in (args.methods,args.scenarios,args.seeds)):
        parser.error("duplicate schedule entry")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    print(f"Output: {out}", flush=True)
    route, scene = Route(), Scene()
    world_dir = out/"maps"
    world_dir.mkdir()
    maps = {case:prepare_world(case, world_dir) for case in args.scenarios}
    order_rng = random.Random(733)
    schedule = []
    for seed in args.seeds:
        for case in args.scenarios:
            block = list(args.methods)
            order_rng.shuffle(block)
            schedule.extend((method,case,seed) for method in block)
    sources = [ROUTE_CSV, MAP_BASE.with_suffix(".yaml"), MAP_BASE.with_suffix(".png"),
               SIM/"tools/build_fitz_tracking_joint.sh", SIM/"tools/generate_takeover_assets.py", SIM/"run_fitz_local.sh", *sorted((SIM/"tools").glob("fitz_local*.py")),
               Path(__file__).resolve(), SIM/"tools/fitz_tracking_plant.py", SIM/"tools/run_fitz_tracking.py",
               SIM/"tools/fitz_tracking_joint.cpp", ASSETS, NATIVE, REPO/"real_vehicle/config/Fitzpatrick_two_obstacles_A_B.json",
               *(p for p in world_dir.rglob("*") if p.is_file())]
    hashes = {str(p.relative_to(REPO) if p.is_relative_to(REPO) else p.name):
              hashlib.sha256(p.read_bytes()).hexdigest() for p in sources if p.is_file()}
    save(out/"manifest.json", dict(schedule=schedule, command_dt=DT, physics_dt=.01, timeout_s=args.time_limit,
         live_gui=args.gui,
         platform=platform.platform(), python=platform.python_version(),
         packages={p:importlib.metadata.version(p) for p in ("gym", "numpy", "scipy", "numba", "Pillow")},
         source_sha256=hashes, git=git_revision(REPO), placements={c:physical_placements(c) for c in maps},
         sensing="official single-agent Gym scan vs known background, exact simulator ego pose",
         rejoin="after the last obstacle in each group (gap <=3 m), +0.55 to +4 m, within 0.12 m and 0.16 rad for five ticks",
         budget_ms=BUDGET_MS, timing="serial synchronous simulation; deadline misses counted, latency not injected"))
    rows, view = [], None
    try:
        if args.gui:
            try:
                import tkinter
                from fitz_local_view import LocalView
            except ImportError as exc:
                parser.exit(2, f"Local GUI requires Python Tk and Pillow ImageTk: {exc}\n")
            method, case, seed = schedule[0]
            try:
                view = LocalView(route, physical_placements(case), method, case, seed)
            except tkinter.TclError as exc:
                parser.exit(2, f"Cannot open the desktop window: {exc}. Run --gui in a graphical session.\n")
        for method,case,seed in schedule:
            row = trial(route,scene,maps[case],method,case,seed,out/f"{method}_{case}_{seed}",args.time_limit, view)
            rows.append(row)
            save(out/"trials.json",rows)
            print(f"{method:21} {case:12} {seed:3}  {row['status']:20}  s={row['progress_m']:5.2f}  "
                  f"t={row['simulated_s']:5.2f}  gap={row['min_body_gap_m']}  {row['detail']}",flush=True)
        from fitz_local_report import write_report
        write_report(out, route, rows)
        if view is not None and row["status"] != "user_closed":
            view.finish(row)
    finally:
        if view is not None:
            view.close()
    return int(any(r["status"]!="passed" for r in rows))


if __name__ == "__main__":
    raise SystemExit(main())
