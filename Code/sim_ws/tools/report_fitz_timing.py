#!/usr/bin/env python3
"""Generate descriptive results and matched contrasts for a completed timing campaign."""

import argparse
import csv
import json
import gzip
from collections import Counter
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

NAMES = {"pp": "PP", "lqr": "LQR", "linear_mpc": "Linear MPC", "nmpc": "NMPC", "mppi": "MPPI", "mpcc": "MPCC"}
MODE_LABEL = {"sync": "Synchronous", "mean": "Mean", "shuffled": "Shuffled", "ordered": "Ordered"}


def report(folder):
    folder = Path(folder)
    rows = json.loads((folder / "trials.json").read_text())
    plan = json.loads((folder / "manifest.json").read_text())["planned_order"]
    if [list((r["method"], r["mode"], r["case"], r["seed"])) for r in rows] != plan:
        raise ValueError("Trial records do not match the complete scheduled campaign")
    with (folder / "comparison.csv").open() as f:
        groups = list(csv.DictReader(f))
    by_key = {(r["method"], r["mode"], r["case"], r["seed"]): r for r in rows}
    pairs = []
    for r in rows:
        if r["mode"] != "ordered":
            continue
        for base in ("sync", "mean", "shuffled"):
            other = by_key.get((r["method"], base, r["case"], r["seed"]))
            if other is None:
                continue
            complete = other["status"] == r["status"] == "finished"
            pairs.append(dict(method=r["method"], case=r["case"], seed=r["seed"], baseline=base,
                              ordered_outcome=r["status"], baseline_outcome=other["status"],
                              ordered_minus_baseline_p95_cte_cm=(r["cte_p95_cm"] - other["cte_p95_cm"])
                              if complete else None))
    if pairs:
        with (folder / "paired.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=pairs[0])
            writer.writeheader()
            writer.writerows(pairs)

    methods = [m for m in NAMES if any(r["method"] == m for r in rows)]
    modes = [m for m in MODE_LABEL if any(r["mode"] == m for r in rows)]
    lookup = {(g["method"], g["mode"]): g for g in groups}
    fig, ax = plt.subplots(figsize=(8.0, 4.7), layout="constrained")
    for j, (mode, marker) in enumerate(zip(modes, ("o", "s", "^", "D"))):
        x = np.arange(len(methods)) + (j - (len(modes)-1)/2) * .16
        for xx, m in zip(x, methods):
            g = lookup[m, mode]
            yy = 100 * int(g["finished"]) / int(g["trials"])
            ax.scatter(xx, yy, marker=marker, c=f"C{j}", s=52,
                       label=MODE_LABEL[mode] if m == methods[0] else None)
    ax.set(xticks=np.arange(len(methods)), xticklabels=[NAMES[m] for m in methods],
           ylim=(-6, 108), ylabel="Scheduled trials reaching stopped finish (%)",
           title="Fitzpatrick  |  Completion under synthetic timing")
    ax.grid(axis="y", alpha=.25)
    ax.legend(loc="lower center", bbox_to_anchor=(.5, -.22), ncol=len(modes))
    fig.savefig(folder / "completion.png", dpi=170)
    plt.close(fig)

    # Score an identical interior section separately from the standing-finish test.
    section = {}
    segment_rows = []
    for r in rows:
        path = folder / r["trial_dir"] / "trace.csv.gz"
        with gzip.open(path, "rt") as stream:
            reader = csv.DictReader(stream)
            errors = [abs(float(row["cte_m"])) * 100. for row in reader
                      if 5. <= float(row["s_m"]) < 35.]
        value = float(np.percentile(errors, 95)) if errors and r["progress_m"] >= 35. else None
        if value is not None:
            section.setdefault((r["method"], r["mode"]), []).append(value)
        segment_rows.append(dict(method=r["method"], mode=r["mode"], case=r["case"],
                                 seed=r["seed"], status=r["status"], p95_abs_cte_cm_5_to_35m=value))
    with (folder / "interior_section.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=segment_rows[0])
        writer.writeheader()
        writer.writerows(segment_rows)
    fig, ax = plt.subplots(figsize=(8.0, 4.7), layout="constrained")
    for j, (mode, marker) in enumerate(zip(modes, ("o", "s", "^", "D"))):
        for i, method in enumerate(methods):
            values = section.get((method, mode), [])
            xx = i + (j - (len(modes) - 1)/2) * .17
            ax.scatter(np.full(len(values), xx), values, marker=marker, facecolors="none",
                       edgecolors=f"C{j}", alpha=.75, s=30, label=MODE_LABEL[mode] if i == 0 else None)
            if values:
                ax.plot([xx-.06, xx+.06], [mean(values)] * 2, color=f"C{j}", linewidth=2.6)
    ax.set(xticks=np.arange(len(methods)), xticklabels=[NAMES[m] for m in methods],
           ylabel="Per-trial P95 absolute lateral error (cm)", title="Fitzpatrick  |  Route progress 5–35 m")
    ax.grid(axis="y", alpha=.25)
    ax.legend(ncol=len(modes), loc="lower center", bbox_to_anchor=(.5, -.22))
    fig.savefig(folder / "interior_error.png", dpi=170)
    plt.close(fig)

    header = "| Method | " + " | ".join(MODE_LABEL[m] for m in modes) + " |"
    table = [header, "|---" * (len(modes) + 1) + "|"]
    for m in methods:
        cells = []
        for mode in modes:
            g = lookup[m, mode]
            val = g["mean_finished_p95_cte_cm"]
            cells.append(f"{g['finished']}/{g['trials']} · {float(val):.2f} cm" if val
                         else f"{g['finished']}/{g['trials']} · n/a")
        table.append("| " + " | ".join([NAMES[m], *cells]) + " |")

    outcomes = Counter(r["status"] for r in rows)
    details = Counter((r["method"], r["status"], r.get("detail", "")) for r in rows if r["status"] != "finished")
    lines = ["# Fitzpatrick timing sensitivity", "",
             "Single car on the 44.73 m reference route. Each cell gives completed / scheduled trials and "
             "mean per-lap P95 absolute lateral error among completed laps. "
             "See comparison.csv for all-run progress and outcomes. A missing error means no stopped finish.", "",
             *table, "", "## Paired outcome", "",
             "Ordered minus shuffled P95 lateral error in centimetres, averaged over exactly matched starts and seeds "
             "for which both methods of scheduling finished. Positive values mean greater error with ordered delays.", ""]
    if methods == ["mpcc"]:
        lines.insert(2, "MPCC selects its own progress speed along the route. This separate experiment "
                        "uses the vehicle command envelope across all four timing profiles. Host solve time "
                        "is measured independently of the synthetic command-arrival delay.\n")
    for m in methods:
        selected = [p for p in pairs if p["method"] == m and p["baseline"] == "shuffled"]
        diffs = [p["ordered_minus_baseline_p95_cte_cm"] for p in selected
                 if p["ordered_minus_baseline_p95_cte_cm"] is not None]
        text = f"{mean(diffs):+.3f} cm" if diffs else "n/a"
        lines.append(f"- {NAMES[m]}: {text} ({len(diffs)}/{len(selected)} matched pairs).")
    lines.extend(["", "## Common interior section (5–35 m)", "",
                  "At 20 Hz, compute per-run P95 absolute lateral error for this section, including runs "
                  "that stopped outside the finish tolerance later. Each cell is the mean in cm and number of "
                  "trials that traversed the section. Raw markers and horizontal mean lines appear in the figure.", "",
                  header, "|---" * (len(modes) + 1) + "|"])
    for m in methods:
        cells = []
        for mode in modes:
            vals = section.get((m, mode), [])
            cells.append(f"{mean(vals):.2f} ({len(vals)})" if vals else "n/a")
        lines.append("| " + " | ".join([NAMES[m], *cells]) + " |")
    lines.extend(["", "## All trials", "", f"Outcomes: {dict(outcomes)}. Gym collision flags: "
                  f"{sum(r['collision'] for r in rows)}. Body / map boundary flags: {sum(r['boundary'] for r in rows)}.", ""])
    if details:
        lines += [f"- {NAMES[m]}: {status} ({n} runs). {detail}" for (m, status, detail), n in sorted(details.items())]
    timed_out = [r for r in rows if r["status"] == "time_limit"]
    if timed_out:
        tolerance = json.loads((folder / "manifest.json").read_text())["end"]
        distances = [r["final_distance_m"] for r in timed_out]
        stopped_outside = [r for r in timed_out if r["final_speed_mps"] <= tolerance["speed_mps"]
                           and r["final_distance_m"] > tolerance["distance_m"]]
        lines.extend(["", f"At termination {len(stopped_outside)}/{len(timed_out)} time-limit runs were at "
                      f"or below the finish speed tolerance but outside the goal distance tolerance. "
                      f"Terminal goal distances for all time-limit runs spanned "
                      f"{100*min(distances):.1f}–{100*max(distances):.1f} cm."])
    lines.extend(["", "## Effective command delivery", "",
                  "Scheduled bursts and shuffled tuples can produce different orderings at the receiver. "
                  "Counts below cover the actually executed part of each trial, including the stopped tail.", "",
                  "| Profile | Released | Applied | Superseded | Pending at end | Longest consecutive releases with delay >50 ms |",
                  "|---|---:|---:|---:|---:|---:|"])
    for mode in modes:
        block = [r for r in rows if r["mode"] == mode]
        counts = [sum(r[k] for r in block) for k in
                  ("requests", "applied_commands", "superseded_commands", "pending_commands")]
        longest = max(r["max_consecutive_over_period"] for r in block)
        lines.append("| " + " | ".join(map(str, [MODE_LABEL[mode], *counts, longest])) + " |")
    lines.extend(["", "Synthetic schedules are shared across controllers. Different completion times give different "
                  "exposure lengths. Timing is imposed in simulated time; laptop wall time is diagnostic only. "
                  "Physical prediction error requires separate held-out vehicle runs and hardware timing records. "
                  "These counts do not represent independent physical sessions or a general controller ranking.", "",
                  "[Interior tracking figure](interior_error.png) · [Interior tracking data](interior_section.csv) · [Completion figure](completion.png) · [All trial summaries](trials.json) · [Schedule and checksums](manifest.json)"])
    (folder / "REPORT.md").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    report(parser.parse_args().folder)
