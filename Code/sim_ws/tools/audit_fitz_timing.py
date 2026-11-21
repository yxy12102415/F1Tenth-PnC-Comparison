#!/usr/bin/env python3
"""Read-only consistency audit of the saved Fitzpatrick timing campaign."""
import argparse
from collections import Counter, defaultdict
import csv
import gzip
from itertools import groupby, product
import json
from pathlib import Path

import numpy as np

from fitz_tracking_plant import Route
from fitz_timing_profiles import WINDOW_TICKS, make_schedule


def require(condition, message):
    if not condition:
        raise ValueError(message)


def close(actual, expected, label):
    require(np.allclose(actual, expected, atol=1.e-7, rtol=1.e-8),
            f"{label}: numeric mismatch")


def read_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def check_schedules(folder, modes, seeds, releases):
    schedules = {}
    for seed, mode in product(seeds, modes):
        rows = read_csv(folder / "schedules" / f"{mode}_{seed}.csv")
        require([int(r["tick"]) for r in rows] == list(range(releases)), "schedule ticks")
        samples = [tuple(int(row[k]) for k in
                         ("pose_age_steps", "odom_age_steps", "command_delay_steps")) for row in rows]
        require(samples == list(make_schedule(mode, seed, releases)), f"{mode}/{seed}: seed mismatch")
        schedules[mode, seed] = samples
    for seed in seeds:
        if {"ordered", "shuffled"} <= set(modes):
            for start in range(0, releases, WINDOW_TICKS):
                interval = slice(start, start + WINDOW_TICKS)
                require(Counter(schedules["ordered", seed][interval]) ==
                        Counter(schedules["shuffled", seed][interval]), "joint histogram differs")
        if {"mean", "ordered"} <= set(modes) and releases % 4 == 0:
            close(np.mean(schedules["ordered", seed], axis=0), schedules["mean", seed][0],
                  "prescribed component means")
    return schedules


def audit_trial(folder, row, route, plan, schedule):
    dt, phys = plan["control_dt_s"], plan["physics_dt_s"]
    period = round(dt / phys)
    stop = round(row["simulated_s"] / phys)
    params, end = plan["command_parameters"], plan["end"]
    with gzip.open(folder / "commands.csv.gz", "rt") as f:
        commands = list(csv.DictReader(f))
    with gzip.open(folder / "trace.csv.gz", "rt") as f:
        trace = list(csv.DictReader(f))
    require(len(commands) == row["requests"] and len(trace) == row["samples"], "row counts")
    close(row["simulated_s"], stop * phys, "sim clock")
    expected_releases = (stop + period - 1) // period
    if row["status"] in ("controller_failure", "stale_stop") and commands and int(commands[-1]["release_step"]) == stop:
        expected_releases += 1
    require(len(commands) == expected_releases, "missing control releases")

    # Reconstruct delivery from release, age and delay only. The receipt with
    # the highest source sequence supersedes earlier arrivals at the same tick.
    arrivals = defaultdict(list)
    nominal = []
    runtimes, calls = [], []
    expected_status = []
    sources = []
    previous_pose = previous_odom = 0
    for i, cmd in enumerate(commands):
        release = i * period
        require((int(cmd["id"]), int(cmd["release_step"])) == (i, release), "release sequence")
        ages = tuple(int(cmd[k]) for k in ("requested_pose_age_steps", "requested_odom_age_steps", "delay_steps"))
        require(ages == schedule[i], "schedule used")
        previous_pose = max(previous_pose, release - ages[0])
        previous_odom = max(previous_odom, release - ages[1])
        require((int(cmd["pose_source_step"]), int(cmd["odom_source_step"])) ==
                (previous_pose, previous_odom), "input source causality")
        sources.append(min(previous_pose, previous_odom))
        require(int(cmd["due_step"]) == release + ages[2], "due clock")
        if cmd["call_ms"]:
            calls.append(float(cmd["call_ms"]))
        if cmd["nominal_speed_mps"]:
            arrivals[release + ages[2]].append(i)
            nominal.append((float(cmd["nominal_speed_mps"]), float(cmd["nominal_steer_rad"])))
            runtimes.append(float(cmd["controller_ms"]))
            expected_status.append("pending")
        else:
            require(release == stop and i == len(commands)-1, "rejected output mid-run")
            expected = "controller_failure" if row["status"] == "controller_failure" else "rejected_stale_observation"
            require(row["status"] in ("controller_failure", "stale_stop"), "unexplained missing command")
            expected_status.append(expected)
    if nominal:
        speed, steer = np.array(nominal).T
        rate = np.diff(speed, prepend=0.) / dt
        require(np.min(speed) >= -1e-5 and np.max(speed) <= params["max_speed"] + 1e-5, "speed cap")
        require(np.max(abs(steer)) <= params["max_steer"] + 1e-5, "steering cap")
        require(np.min(rate) >= -params["max_decel"] - 1e-4 and
                np.max(rate) <= params["max_accel"] + 1e-4, "nominal acceleration cap")
        require(np.max(abs(np.diff(steer, prepend=0.))) <= dt * params["max_steer_rate"] + 1e-5,
                "nominal steering rate cap")
    require(all(np.isfinite(x) and x >= 0 for x in calls + runtimes), "invalid wall time")

    active = -1
    delivered = []
    applied = np.full(stop, -1)
    stale_steps = round(.5 / phys)
    for tick in range(stop):
        batch = arrivals.get(tick, ())
        for i in batch:
            if i < max(active, max(batch)):
                expected_status[i] = "superseded"
            elif tick - sources[i] > stale_steps:
                expected_status[i] = "expired"
            else:
                expected_status[i] = "applied"
                require(int(commands[i]["applied_step"]) == tick, "arrival not applied on correct tick")
                delivered.append((tick - int(commands[i]["pose_source_step"])) * phys * 1000)
                active = i
        require(tick - (sources[active] if active >= 0 else 0) <= stale_steps, "stale held output")
        applied[tick] = active
    require(expected_status == [r["disposition"] for r in commands], "delivery dispositions")
    for cmd in commands:
        require(bool(cmd["applied_step"]) == (cmd["disposition"] == "applied"), "applied step")
    for label in ("applied", "superseded", "expired", "pending"):
        require(row[label + "_commands"] == expected_status.count(label), label + " count")
    late = [int(c["delay_steps"]) > period for c in commands]
    require(sum(late) == row["over_period_requests"], "over-period count")
    require(row["max_consecutive_over_period"] == max((len(list(g)) for key, g in groupby(late) if key), default=0),
            "consecutive over-period count")
    if delivered:
        close([row["mean_applied_pose_age_ms"], row["applied_pose_age_p95_ms"]],
              [np.mean(delivered), np.percentile(delivered, 95)], "delivered ages")
    if runtimes:
        close(row["compute_p95_ms"], np.percentile(runtimes, 95), "runtime P95")
        require(row["over_budget_steps"] == sum(t > 45. for t in runtimes), "wall budget")
    if calls:
        close(row["call_p95_ms"], np.percentile(calls, 95), "call P95")

    expected_ticks = list(range(period, stop + 1, period))
    if (row["collision"] or row["boundary"]) and stop % period:
        expected_ticks.append(stop)
    close([float(t["t_s"]) for t in trace], np.array(expected_ticks)*phys, "scoring ticks")
    require(len(trace) == len(expected_ticks), "trace coverage")
    if not trace:
        require(row["status"] in ("controller_failure", "stale_stop"), "empty completed trial")
        return
    data = {k: np.array([float(t[k]) for t in trace]) for k in trace[0]}
    require(all(np.all(np.isfinite(a)) for a in data.values()), "nonfinite scoring state")
    ids = applied[np.asarray(expected_ticks) - 1]
    close(data["applied_command_id"], ids, "held command sequence")
    acts = np.array([nominal[i] if i >= 0 else (0., 0.) for i in ids])
    close(data["cmd_v_mps"], acts[:, 0], "applied speed")
    close(data["cmd_delta_rad"], acts[:, 1], "applied steering")

    # Recompute the geometric errors from raw xy/yaw against the repository route.
    prev, station, cte = 0., [], []
    for x, y in zip(data["x"], data["y"]):
        prev, error = route.project({"x":x, "y":y}, prev)
        station.append(prev)
        cte.append(error)
    close(station, data["s_m"], "projected progress")
    # The scorer retains sub-control-period states only on contact. Check every
    # visible input source that overlaps this independent state log.
    saved_station = dict(zip(expected_ticks, station))
    x0, y0, yaw0, _ = route.at_s(0.)
    offset = plan["cases"][row["case"]][0]
    saved_station[0] = route.project({"x": x0 - offset * np.sin(yaw0),
                                      "y": y0 + offset * np.cos(yaw0)}, 0.)[0]
    for cmd in commands:
        source = int(cmd["pose_source_step"])
        if source in saved_station:
            close(float(cmd["observed_s_m"]), saved_station[source], "causal reference station")
    close(cte, data["cte_m"], "cross-track error")
    reference = route.at_s(np.array(station))
    close(reference[:, [0,1,3]], np.column_stack([data[k] for k in ("ref_x", "ref_y", "ref_v_mps")]),
          "reference projection")
    yaw_error = data["yaw"] - reference[:,2]
    close(np.arctan2(np.sin(yaw_error), np.cos(yaw_error)), data["heading_error_rad"], "heading")
    require(not np.any(data["collision"][:-1]) and not np.any(data["boundary"][:-1]), "continued after contact")
    require(bool(data["collision"][-1]) == row["collision"] and
            bool(data["boundary"][-1]) == row["boundary"], "terminal flags")
    goal = route.at_s(route.length)
    dest = np.hypot(data["x"] - goal[0], data["y"] - goal[1])
    goal_yaw = data["yaw"] - goal[2]
    yaw_diff = np.abs(np.arctan2(np.sin(goal_yaw), np.cos(goal_yaw)))
    inside = ((data["s_m"] >= route.length-end["remaining_m"]) & (dest <= end["distance_m"])
              & (yaw_diff <= end["yaw_rad"]) & (abs(data["v"]) <= end["speed_mps"]))
    dwell = round(end["dwell_s"] / dt)
    completed = np.flatnonzero(np.convolve(inside.astype(int), np.ones(dwell), mode="valid") == dwell) if len(inside)>=dwell else []
    status = row["status"]
    if row["collision"] or row["boundary"]:
        require(status == ("collision" if row["collision"] else "boundary"), "contact result")
    elif len(completed):
        require(status == "finished" and completed[0]+dwell == len(trace), "stopped finish")
    elif status == "time_limit":
        close(row["simulated_s"], plan["time_limit_s"], "timeout")
    else:
        require(status in ("controller_failure", "stale_stop") and row["detail"], "unexplained outcome")
    rms = lambda a: float(np.sqrt(np.mean(np.square(a))))
    for k, value in dict(progress_m=prev, cte_rmse_cm=100*rms(cte),
                         cte_p95_cm=100*np.percentile(np.abs(cte),95),
                         speed_rmse_mps=rms(data["v"] - reference[:,3]),
                         heading_rmse_deg=np.degrees(rms(data["heading_error_rad"])),
                         steer_rate_rms_rad_s=rms(np.diff(data["cmd_delta_rad"],prepend=0.)/dt),
                         peak_actual_speed_mps=np.max(data["v"])).items():
        close(row[k], value, k)
    if stop == expected_ticks[-1]:
        close([row["final_speed_mps"], row["final_distance_m"]], [data["v"][-1], dest[-1]], "finish state")


def audit(folder):
    plan = json.loads((folder / "manifest.json").read_text())
    rows = json.loads((folder / "trials.json").read_text())
    fields = ("method", "mode", "case", "seed")
    order = [tuple(key) for key in plan["planned_order"]]
    require(order == [tuple(row[k] for k in fields) for row in rows], "incomplete or reordered campaign")
    factors = [set(column) for column in zip(*order)]
    require(len(factors) == 4 and len(order) == len(set(order)) and
            set(order) == set(product(*factors)), "incomplete factorial campaign")
    n = round(plan["time_limit_s"] / plan["control_dt_s"])
    schedules = check_schedules(folder, factors[1], factors[3], n)
    route = Route()
    close(plan["route_length_m"], route.length, "route length")
    lookup = {}
    for key, row in zip(order, rows):
        trial_folder = folder / row["trial_dir"]
        require(trial_folder.name == "_".join(map(str, key)), "trial folder name")
        require(json.loads((trial_folder / "summary.json").read_text()) == row, "summary vs trials.json")
        try:
            audit_trial(trial_folder, row, route, plan, schedules[row["mode"],row["seed"]])
        except ValueError as exc:
            raise ValueError(f"{trial_folder.name}: {exc}") from exc
        lookup[key] = row
    groups = read_csv(folder / "comparison.csv")
    grouped = defaultdict(list)
    for (method, mode, _, _), row in lookup.items():
        grouped[method, mode].append(row)
    require(len(groups) == len(grouped) and
            {(g["method"], g["mode"]) for g in groups} == set(grouped), "missing comparison groups")
    for group in groups:
        block = grouped[group["method"], group["mode"]]
        good = [r for r in block if r["status"] == "finished"]
        counts = Counter(r["status"] for r in block)
        require(group["outcomes"] == " ".join(f"{k}={v}" for k, v in sorted(counts.items())),
                "aggregate outcome distribution")
        for k, value in dict(trials=len(block), finished=len(good),
                             collisions=sum(r["collision"] for r in block),
                             boundaries=sum(r["boundary"] for r in block),
                             mean_progress_m_all=np.mean([r["progress_m"] for r in block])).items():
            close(float(group[k]), value, "comparison: " + k)
        for k, field in (("mean_finished_p95_cte_cm", "cte_p95_cm"), ("mean_finish_s", "simulated_s"),
                         ("mean_finished_cte_rmse_cm", "cte_rmse_cm")):
            if good:
                close(float(group[k]), np.mean([r[field] for r in good]), k)
            else:
                require(not group[k], "unfinished group carries finish mean")
    expected_pairs = {(m,base,c,s) for m,mode,c,s in order if mode=="ordered"
                      for base in factors[1] - {"ordered"}}
    pairs = read_csv(folder/"paired.csv") if expected_pairs else []
    seen = []
    for pair in pairs:
        m, base, c, s = pair["method"], pair["baseline"], pair["case"], int(pair["seed"])
        seen.append((m,base,c,s))
        left, right = lookup[m,"ordered",c,s], lookup[m,base,c,s]
        require((left["status"],right["status"]) == (pair["ordered_outcome"], pair["baseline_outcome"]),
                "paired statuses")
        value = pair["ordered_minus_baseline_p95_cte_cm"]
        if left["status"] == right["status"] == "finished":
            close(float(value), left["cte_p95_cm"] - right["cte_p95_cm"], "paired error difference")
        else:
            require(not value, "failed pairs carry survivor score")
    require(len(seen) == len(expected_pairs) and set(seen)==expected_pairs, "unpaired trials")
    print(f"Verified {len(rows)} trials, {sum(r['requests'] for r in rows)} commands, "
          f"{len(pairs)} paired contrasts; outcomes {dict(Counter(r['status'] for r in rows))}.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    try:
        audit(args.folder)
    except (ValueError, KeyError, OSError, IndexError) as error:
        parser.exit(1, f"Audit failed: {error}\n")
