"""Regenerate the README tracking figure from the 30 archived Gym trials."""
import json
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FixedLocator, FormatStrFormatter

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "results/fitzpatrick_tracking_20261120/trials.json"
OUTPUT = ROOT / "docs/figures/tracking_accuracy_runtime.svg"
# Labels sit next to the method means; axis positions come from the trial data.
METHODS = {
    "pp": ("Pure Pursuit", "#195b74", (19, -10)),
    "lqr": ("LQR", "#865016", (10, -2)),
    "linear_mpc": ("Linear MPC", "#57448c", (10, -1)),
    "nmpc": ("NMPC", "#146b4c", (10, 3)),
    "mppi": ("MPPI", "#943257", (-12, -12)),
}


def main():
    trials = json.loads(SOURCE.read_text())
    with plt.rc_context({"svg.fonttype": "none", "svg.hashsalt": "fitzpatrick-tracking",
                         "font.size": 11, "axes.spines.top": False, "axes.spines.right": False}):
        fig, ax = plt.subplots(figsize=(8.3, 3.9), layout="constrained")
        for method, (label, color, offset) in METHODS.items():
            rows = [r for r in trials if r["method"] == method and r["status"] == "finished"]
            x = [r["compute_p95_ms"] for r in rows]
            y = [r["cte_rmse_cm"] for r in rows]
            ax.scatter(x, y, edgecolors=color, facecolors="none", s=36, linewidth=.9)
            ax.scatter(mean(x), mean(y), color=color, marker="D", s=58, zorder=3)
            ax.annotate(label, (mean(x), mean(y)), xytext=offset, textcoords="offset points",
                        ha="right" if offset[0] < 0 else "left", va="center", weight="bold")
        ax.set_xscale("log")
        ax.set(xlabel="Controller compute time, per-trial P95 (ms, log scale)",
               ylabel="Cross-track RMSE (cm)", xlim=(.09, 4.6))
        ax.xaxis.set_major_locator(FixedLocator([.1, .2, .5, 1, 2, 4]))
        ax.xaxis.set_major_formatter(FormatStrFormatter("%g"))
        ax.grid(which="major", alpha=.23)
        ax.set_axisbelow(True)
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(OUTPUT, metadata={"Date": None, "Title": "Tracking accuracy and controller runtime",
                                      "Description": "Open circles show trials; diamonds show method means."})
        plt.close(fig)


if __name__ == "__main__":
    main()
