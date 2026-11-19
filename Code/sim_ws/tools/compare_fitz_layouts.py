"""Paired comparison of adding the second obstacle; reads an existing campaign."""
import argparse
import csv
import json
from pathlib import Path
from statistics import median

PAIRS = (("single_left", "stagger_left_first"), ("single_right", "stagger_right_first"))


def paired_rows(folder):
    trials = json.loads((folder / "trials.json").read_text())
    records = {(r["method"], r["scenario"], r["seed"]): r for r in trials}
    if len(records) != len(trials):
        raise ValueError("duplicate method, scenario and seed")
    manifest = json.loads((folder / "manifest.json").read_text())
    for method in sorted({r["method"] for r in trials}):
        for single, double in PAIRS:
            singles = {seed for m, c, seed in records if m == method and c == single}
            doubles = {seed for m, c, seed in records if m == method and c == double}
            if not (singles or doubles):
                continue
            if singles != doubles:
                raise ValueError(f"incomplete block for {method}, {single}, {double}")
            single_fixture = manifest["placements"][single]
            fixtures = manifest["placements"][double]
            if len(single_fixture) != 1 or len(fixtures) != 2 or single_fixture[0] != fixtures[0]:
                raise ValueError(f"unmatched first obstacle for {single}, {double}")
            additions, gaps = [], []
            single_pass = double_pass = changed_side = 0
            for seed in sorted(singles):
                a, b = records[method, single, seed], records[method, double, seed]
                single_pass += a["status"] == "passed"
                double_pass += b["status"] == "passed"
                if b["status"] == "passed":
                    # Determine the side of the obstacle, measured at its own station.
                    # Positive displacement is left of travel.
                    side = [e["passing_offset_m"] - p["lateral_offset_m"]
                            for e, p in zip(b["encounters"], fixtures)]
                    changed_side += side[0] * side[1] < 0
                    gaps.append(b["min_body_gap_m"])
                if a["status"] == b["status"] == "passed":
                    additions.append(b["simulated_s"] - a["simulated_s"])
            yield dict(method=method, single=single, double=double, seeds=len(singles),
                       single_pass=single_pass, double_pass=double_pass,
                       both_pass=len(additions), median_added_lap_s=round(median(additions), 3) if additions else None,
                       min_double_gap_on_success_m=round(min(gaps), 4) if gaps else None,
                       double_pass_alternating_sides=changed_side, double_pass_common_side=double_pass-changed_side)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    args = parser.parse_args()
    rows = list(paired_rows(args.campaign))
    if not rows:
        parser.error("supply a campaign containing matched single- and double-obstacle cases")
    dest = args.campaign / "paired_layouts.csv"
    with dest.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(dest)


if __name__ == "__main__":
    main()
