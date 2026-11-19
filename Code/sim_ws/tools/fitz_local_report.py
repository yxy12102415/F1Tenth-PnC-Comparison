"""Aggregate Fitzpatrick trials and plot the traces and obstacles of each two-object layout."""
from collections import Counter, defaultdict
import csv
import gzip
import json
from pathlib import Path

import numpy as np


LABELS = {"hybrid_astar": "Hybrid A*", "lattice": "Frenet lattice",
          "rrt_star": "Kinematic local RRT*", "dwa": "Ackermann DWA",
          "teb": "Car-like TEB", "unmodified_reference": "Unmodified reference (negative control)"}
CASES = {"clear": "Clear track", "A": "A only", "B": "B only",
         "AB": "A+B (historical configuration)", "AB_recorded": "A+B (recorded-response estimates)",
         "single_left": "Matched first obstacle, left", "single_right": "Matched first obstacle, right",
         "stagger_left_first": "Designed stagger, left obstacle first",
         "stagger_right_first": "Designed stagger, right obstacle first"}
STAGGERED = ("stagger_left_first", "stagger_right_first")
PATH_METHODS = ("hybrid_astar", "lattice", "rrt_star", "teb")


def _median(rows, key):
    values = [r[key] for r in rows if r[key] is not None]
    return float(np.median(values)) if values else None


def _audit_plan_timing(out, raw):
    """Derive a reporting view without changing the original trial measurements."""
    captured = raw["active_plan_calls"]
    attempts = captured
    if raw["method"] in PATH_METHODS:
        folder = out / f"{raw['method']}_{raw['scenario']}_{raw['seed']}"
        attempts = len(json.loads((folder / "plans.json").read_text()))
    return dict(raw, plan_attempts=attempts, missing_plan_timings=max(0, attempts - captured),
                excess_plan_timings=max(0, captured - attempts),
                complete_planning_p95_ms=raw["planning_call_p95_ms"] if attempts == captured else None)


def _group(method, case, trials):
    counts = Counter(r["status"] for r in trials)
    finished = [r for r in trials if r["status"] == "passed"]
    gaps = [r["min_body_gap_m"] for r in trials if r["min_body_gap_m"] is not None]
    success_gaps = [r["min_body_gap_m"] for r in finished if r["min_body_gap_m"] is not None]
    # A rejected observation or update contributes a timing sample before the
    # car advances, so it is missing from the post-step command tick count.
    timed_ticks = sum(r["command_ticks"] + int(r["status"].endswith("_rejection")) for r in trials)
    return dict(method=method, scenario=case, trials=len(trials), passed=len(finished),
                status_counts=json.dumps(dict(sorted(counts.items()))),
                collisions=sum(r["collision"] for r in trials), boundary_violations=sum(r["boundary"] for r in trials),
                median_success_time_s=_median(finished, "simulated_s"),
                min_body_gap_all_m=min(gaps, default=None),
                min_body_gap_success_m=min(success_gaps, default=None),
                median_max_progress_m=_median(trials, "progress_m"),
                generated_plan_or_dwa_calls=sum(r["plan_attempts"] for r in trials),
                missing_plan_timings=sum(r["missing_plan_timings"] for r in trials),
                excess_plan_timings=sum(r["excess_plan_timings"] for r in trials),
                trials_with_plan_p95=sum(r["complete_planning_p95_ms"] is not None for r in trials),
                median_trial_planning_call_p95_ms=_median(trials, "complete_planning_p95_ms"),
                median_trial_all_tick_pipeline_p95_ms=_median(trials, "pipeline_p95_ms"),
                max_pipeline_ms=max((r["pipeline_max_ms"] for r in trials if r["pipeline_max_ms"] is not None), default=None),
                pipeline_deadline_misses=sum(r["deadline_misses"] for r in trials), timed_ticks=timed_ticks,
                median_trial_steer_rate_rms_rad_s=_median(trials, "steer_rate_rms_rad_s"))


def _figure(out, route, method, case, trials, placements):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    figures = out / "figures"
    figures.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.8, 6.7))
    ax.plot(*route.xy.T, color="#222222", ls="--", lw=1.4, label="Global reference", zorder=3)
    seen_paths = False
    colors = ["#0072b2", "#d55e00", "#009e73", "#cc79a7"]
    for i, trial in enumerate(sorted(trials, key=lambda r: r["seed"])):
        folder = out / f"{method}_{case}_{trial['seed']}"
        if i == 0:
            for event in json.loads((folder / "plans.json").read_text()):
                path = np.asarray(event.get("path", []))
                if len(path):
                    ax.plot(path[:, 0], path[:, 1], ":", lw=2.1, color="#6a4c93", zorder=4,
                            label="Accepted local plans (first seed)" if not seen_paths else None)
                    seen_paths = True
        with gzip.open(folder / "trace.csv.gz", "rt", newline="") as f:
            xy = np.array([(float(r["x"]), float(r["y"])) for r in csv.DictReader(f)])
        if not len(xy):
            continue
        ax.plot(*xy.T, color=colors[i % len(colors)], lw=1.75, alpha=.8,
                label=f"Seed {trial['seed']} ({trial['status'].replace('_', ' ')})", zorder=2)
        if trial["status"] != "passed":
            ax.scatter(*xy[-1], s=40, marker="x", color=colors[i % len(colors)], zorder=6)
    for place in placements:
        x, y = place["center_xy"]
        ax.add_patch(Circle((x, y), place["desired_diameter_m"]/2,
                            facecolor="black", edgecolor="black", zorder=6))
        ax.annotate(place["name"], (x, y), xytext=(9, 7), textcoords="offset points",
                    fontsize=11, weight="bold", arrowprops=dict(arrowstyle="-", lw=.8))
    ax.set(xlabel="Map x (m)", ylabel="Map y (m)", aspect="equal",
           title=f"Fitzpatrick | {LABELS[method]}\n{CASES[case]}")
    ax.grid(alpha=.2)
    ax.legend(loc="upper center", bbox_to_anchor=(.5, -.10), ncol=2, fontsize=8)
    fig.tight_layout()
    filename = f"figures/{method}_{case}.png"
    fig.savefig(out / filename, dpi=160)
    outputs = {"course": filename}
    if case in STAGGERED:
        centers = np.array([p["center_xy"] for p in placements])
        stations = np.array([p["route_distance_m"] for p in placements])
        nearby = route.xy[(route.s >= stations.min()-1.1) & (route.s <= stations.max()+1.1)]
        bounds = np.vstack((centers, nearby))
        lo, hi = bounds.min(axis=0)-.85, bounds.max(axis=0)+.85
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]),
               title=f"Fitzpatrick obstacle area | {LABELS[method]}\n{CASES[case]}")
        fig.tight_layout()
        zoom = f"figures/{method}_{case}_detail.png"
        fig.savefig(out / zoom, dpi=160)
        outputs["obstacle detail"] = zoom
    plt.close(fig)
    return outputs


def _number(value, digits=2):
    return "—" if value is None else f"{value:.{digits}f}"


def write_report(out, route, rows):
    out = Path(out)
    manifest = json.loads((out / "manifest.json").read_text())
    rows = [_audit_plan_timing(out, row) for row in rows]
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["method"], row["scenario"]].append(row)
    order = [(m, c) for m in LABELS for c in CASES if (m, c) in grouped]
    unknown = set(grouped) - set(order)
    if unknown:
        raise ValueError(f"Unrecognised method/scenario groups in the results: {sorted(unknown)}")
    comparisons = [_group(m, c, grouped[m, c]) for m, c in order]
    if not comparisons:
        raise ValueError("No completed trials to report")
    with (out / "comparison.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=comparisons[0].keys())
        writer.writeheader()
        writer.writerows(comparisons)
    trials = [r for r in rows if r["method"] != "unmodified_reference"]
    passed = sum(r["status"] == "passed" for r in trials)
    negative = [r for r in rows if r["method"] == "unmodified_reference"]
    lines = ["# Fitzpatrick static local planning", "",
             f"Of {len(trials)} planner trials, **{passed} passed**. "
             f"The schedule included {len(negative)} additional unmodified-reference negative control(s).", "",
             "A pass requires the scenario’s route-rejoin checks and a stop at the finish. "
             "Gym supplies the scans and ego state. "
             "The planner receives obstacle estimates derived from the scans. "
             "Ground-truth placements are used to construct the scene and score the body clearance.", ""]
    included_cases = {case for _, case in grouped}
    if included_cases.intersection(("A", "B", "AB", "AB_recorded")):
        lines.extend(["The A and B cases replay the nominal physical layout. Where selected, AB_recorded "
                      "uses obstacle-position and diameter estimates from the archived vehicle response. "
                      "The outcomes below are new simulations.", ""])
    if included_cases.intersection(STAGGERED):
        lines.extend(["The two stagger names denote mirrored, simulation-designed obstacle layouts on the "
                      "Fitzpatrick route. Left and right are relative to the vehicle's reference direction. "
                      "The following coordinates describe the simulated obstacle centres in the map frame.", "",
                      "| Designed scenario | Obstacle | Route progress (m) | Map x (m) | Map y (m) | Diameter (m) |",
                      "|---|---|---:|---:|---:|---:|"])
        for case in STAGGERED:
            if case in included_cases:
                for obj in manifest["placements"][case]:
                    x, y = obj["center_xy"]
                    lines.append(f"| {CASES[case]} | {obj['name']} | {obj['route_distance_m']:.2f} | "
                                 f"{x:.3f} | {y:.3f} | {obj['desired_diameter_m']:.3f} |")
        lines.append("")
    lines.extend(["| Method | Scenario | Passed / run | Collisions | Boundary | Successful time (s) | Min gap (m) | Plan P95 (ms) | Pipeline P95 (ms) | Late / timed ticks |",
                  "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"])
    for c in comparisons:
        nums = [_number(c["median_success_time_s"]), _number(c["min_body_gap_all_m"], 3),
                _number(c["median_trial_planning_call_p95_ms"]), _number(c["median_trial_all_tick_pipeline_p95_ms"])]
        lines.append(f"| {LABELS[c['method']]} | {CASES[c['scenario']]} | {c['passed']}/{c['trials']} | "
                     f"{c['collisions']} | {c['boundary_violations']} | " + " | ".join(nums)
                     + f" | {c['pipeline_deadline_misses']}/{c['timed_ticks']} |")
    lines.extend(["", "Successful time is the median completed-lap simulation time. Minimum gap is the smallest "
                  "continuous-body-to-physical-disc clearance recorded at the physics steps across all outcomes. "
                  "A dash indicates no observation (for example a clear track has no obstacle gap). "
                  "The CSV also gives the minimum gap among successful runs, steering-rate RMS and the status counts.", "",
                  "Both P95 columns are medians of within-trial 95th percentiles. The plan column measures "
                  "new local-path computations, including rejected planner requests, or the DWA decision on each tick. "
                  "The pipeline column covers all measured ticks, including perception, planning and the downstream "
                  "controller when used. It excludes Gym stepping. The synchronous simulation counts deadlines over "
                  f"{manifest['budget_ms']:.0f} ms without injecting latency. Timings reflect this host and workload.", "",
                  "Hybrid A*, lattice, RRT* and TEB feed the same joint NMPC. TEB retains its segment times. "
                  "DWA executes its selected speed and steering directly. Lap-time differences include these "
                  "execution and speed-policy differences. The unmodified reference runs NMPC without obstacle avoidance.", "",
                  "## Planning timing completeness", ""])
    missing = sum(r["missing_plan_timings"] for r in rows)
    excess = sum(r["excess_plan_timings"] for r in rows)
    affected = [r for r in rows if r["missing_plan_timings"] or r["excess_plan_timings"]]
    lines.extend([f"Path-planning requests are counted from each trial's planning-event log. "
                  f"The audit found {missing} missing timing sample(s) and {excess} excess sample(s) "
                  f"across {len(affected)} trial(s). The CSV reports the counts per group and the number "
                  "of trials contributing to its planning P95.", ""])
    if affected:
        lines.extend(["Older trials could accept a new path and then terminate on a controller rejection "
                      "in the same tick, before a command trace sample was written. For each count mismatch "
                      "the report retains the true request count and excludes the trial's planning P95. "
                      "The original measurements and pipeline timing are preserved.", "",
                      "| Method | Scenario | Seed | Requests | Recorded timings |",
                      "|---|---|---:|---:|---:|"])
        for r in affected:
            lines.append(f"| {LABELS[r['method']]} | {CASES[r['scenario']]} | {r['seed']} | "
                         f"{r['plan_attempts']} | {r['active_plan_calls']} |")
        lines.append("")
    lines.extend(["## Outcomes requiring inspection", ""])
    failures = [r for r in rows if r["status"] != "passed"]
    if failures:
        lines.extend(["| Method | Scenario | Seed | Outcome | Progress (m) | Detail |", "|---|---|---:|---|---:|---|"])
        for r in failures:
            detail = r["detail"].replace("|", "/").replace("\n", " ") or "—"
            lines.append(f"| {LABELS[r['method']]} | {CASES[r['scenario']]} | {r['seed']} | {r['status']} | "
                         f"{r['progress_m']:.2f} | {detail} |")
    else:
        lines.append("All scheduled trials passed.")
    plotted = [(m, c) for m, c in order if c in ("AB", "AB_recorded", *STAGGERED)]
    if plotted:
        lines.extend(["", "## Two-obstacle trajectories", "",
                      "Each course plot uses map coordinates and shows the actual vehicle-centre traces. "
                      "Obstacles are drawn to scale. Crosses mark the end of unsuccessful traces.", ""])
        for method, case in plotted:
            files = _figure(out, route, method, case, grouped[method, case], manifest["placements"][case])
            links = " · ".join(f"[{label}]({file})" for label, file in files.items())
            lines.append(f"- {LABELS[method]}, {CASES[case]} — {links}")
    lines.extend(["", "Each trial directory contains the full compressed trace, summary and planning-event records. "
                  "The manifest records the run order, geometry, software hashes and evaluation thresholds."])
    (out / "REPORT.md").write_text("\n".join(lines) + "\n")
