#!/usr/bin/env python3
"""Paired single-car timing sensitivity trials on the Fitzpatrick route."""

import argparse
import csv
import hashlib
import importlib.metadata
import itertools
import platform
import random
from collections import Counter
from datetime import datetime
from pathlib import Path
from statistics import mean

from fitz_timing_profiles import PROFILES, make_schedule
from fitz_timing_trial import trial
from fitz_tracking_plant import MAP_BASE, PHYSICS_DT, Route
from run_fitz_tracking import (ASSETS, CASES, DT, END, METHODS, NATIVE, REPO, ROUTE_FILE,
                               SIM, TRACKERS, VEHICLE_PARAMS, git_revision, save)


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)


def fingerprint():
    tools = SIM / "tools"
    headers = SIM / "src/takeover_sim/include/takeover_sim"
    model = next((REPO / "real_vehicle/candidate_cpp_gpu_20260909").glob("*_v8"))
    build = SIM / "build/takeover_sim/gym_model/generated_joint"
    sources = {Path(__file__).resolve(), tools / "report_fitz_timing.py",
               ROUTE_FILE, ASSETS, NATIVE, MAP_BASE.with_suffix(".png"),
               MAP_BASE.with_suffix(".yaml"), NATIVE.with_name("libfitz_tracking_gym_solver.so"),
               SIM / "run_fitz_timing.sh", tools / "build_fitz_tracking_joint.sh",
               tools / "generate_gym_mpc.sh", model / "generate_joint_solver.py"}
    for directory, pattern in [(tools, "fitz_timing*.py"), (tools, "*fitz_tracking*.py"),
                               (tools, "fitz_tracking_joint.cpp"), (headers, "*.hpp"),
                               (model / "src/cpp_frenet", "*.hpp"), (build, "**/*.c")]:
        sources.update(directory.glob(pattern))
    return {str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(sources)}


def summarize(rows):
    groups = []
    for method, mode in dict.fromkeys((r["method"], r["mode"]) for r in rows):
        subset = [r for r in rows if (r["method"], r["mode"]) == (method, mode)]
        good = [r for r in subset if r["status"] == "finished"]
        counts = Counter(r["status"] for r in subset)
        groups.append(dict(method=method, mode=mode, trials=len(subset), finished=len(good),
                           collisions=sum(r["collision"] for r in subset),
                           boundaries=sum(r["boundary"] for r in subset),
                           outcomes=" ".join(f"{k}={v}" for k, v in sorted(counts.items())),
                           mean_progress_m_all=mean(r["progress_m"] for r in subset),
                           mean_finished_p95_cte_cm=mean(r["cte_p95_cm"] for r in good) if good else None,
                           mean_finished_cte_rmse_cm=mean(r["cte_rmse_cm"] for r in good) if good else None,
                           mean_finish_s=mean(r["simulated_s"] for r in good) if good else None))
    return sorted(groups, key=lambda r: (METHODS.index(r["method"]), PROFILES.index(r["mode"])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=TRACKERS)
    parser.add_argument("--modes", nargs="+", choices=PROFILES, default=PROFILES)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--seeds", nargs="+", type=int, default=[11, 29])
    parser.add_argument("--output", type=Path, default=SIM / "results/fitz_timing" /
                        datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    args = parser.parse_args()
    if "mpcc" in args.methods and len(args.methods) > 1:
        parser.error("Run --methods mpcc separately for the free-progress experiment")
    packages = ["f110-gym", "gym", "numpy", "scipy", "osqp"]
    if "mpcc" in args.methods:
        packages.append("casadi")
    for values in (args.methods, args.modes, args.cases, args.seeds):
        if len(set(values)) != len(values):
            parser.error("duplicate selection")
    if any(not 0 <= seed < 2**32 for seed in args.seeds):
        parser.error("seed must be an unsigned 32-bit integer")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    route = Route()
    timeout = 55.
    ticks = round(timeout / DT)
    schedules = {}
    order = []
    rng = random.Random(20261122)
    (out / "schedules").mkdir()
    for seed in args.seeds:
        for mode in args.modes:
            seq = make_schedule(mode, seed, ticks)
            schedules[seed, mode] = seq
            write_csv(out / "schedules" / f"{mode}_{seed}.csv",
                      [dict(tick=i, pose_age_steps=p.pose_age_steps, odom_age_steps=p.odom_age_steps,
                            command_delay_steps=p.command_delay_steps) for i, p in enumerate(seq)])
        for case in args.cases:
            block = list(itertools.product(args.methods, args.modes))
            rng.shuffle(block)
            order.extend((method, mode, case, seed) for method, mode in block)
    hashes = fingerprint()
    save(out / "manifest.json", dict(source="synthetic paired timing profiles; no physical timing replay",
         reference_policy="free progress on route geometry" if "mpcc" in args.methods else "shared timed reference",
         route_length_m=route.length, physics_dt_s=PHYSICS_DT, control_dt_s=DT, time_limit_s=timeout,
         cases=CASES, end=END, command_parameters=VEHICLE_PARAMS, planned_order=order,
         git=git_revision(REPO), source_sha256=hashes, platform=platform.platform(),
         python=platform.python_version(), packages={k: importlib.metadata.version(k)
         for k in packages}))
    rows = []
    for i, (method, mode, case, seed) in enumerate(order, 1):
        folder = out / f"{method}_{mode}_{case}_{seed}"
        row = trial(route, method, case, seed, folder, timeout, schedules[seed, mode])
        row.update(mode=mode, trial_dir=folder.name)
        save(folder / "summary.json", row)
        rows.append(row)
        save(out / "trials.json", rows)
        print(f"{i}/{len(order)} {folder.name}: {row['status']}, {row['progress_m']:.2f} m", flush=True)
    write_csv(out / "comparison.csv", summarize(rows))
    from report_fitz_timing import report
    report(out)
    current = fingerprint()
    changed = sorted(key for key in hashes.keys() | current.keys() if hashes.get(key) != current.get(key))
    save(out / "integrity.json", {"fingerprinted_files": len(hashes), "changed_during_run": changed})
    if changed:
        raise RuntimeError(f"Campaign source or executable changed; retain run for diagnosis: {changed}")
    print(out, flush=True)
    return int(any(r["status"] != "finished" for r in rows))


if __name__ == "__main__":
    raise SystemExit(main())
