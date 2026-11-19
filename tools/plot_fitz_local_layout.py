#!/usr/bin/env python3
"""Render the surveyed Fitzpatrick reference and the two designed simulation layouts."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Rectangle
import numpy as np
from PIL import Image
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "Code/sim_ws/tools"))
from fitz_local_scene import STAGGER_CASES, audit_stagger_geometry, physical_placements
from fitz_tracking_plant import MAP_BASE, Route


def draw(output):
    output.mkdir(parents=True, exist_ok=True)
    route = Route()
    placements = physical_placements("AB")
    audits = [audit_stagger_geometry(case) for case in STAGGER_CASES]
    meta = yaml.safe_load(MAP_BASE.with_suffix(".yaml").read_text())
    with Image.open(MAP_BASE.with_suffix(".png")) as image:
        grid = np.array(image.convert("L"))
    # Use exact native pixels; tone values distinguish wall, unknown and free.
    shade = np.where(grid == 0, .20, np.where(grid < 250, .87, 1.))
    x0, y0 = meta["origin"][:2]
    extent = (x0, x0+grid.shape[1]*meta["resolution"], y0, y0+grid.shape[0]*meta["resolution"])
    route_style = dict(color=".15", linestyle=(0, (4, 2)), linewidth=1.3)
    fig, axes = plt.subplots(1, 3, figsize=(11.8, 6.6),
                             gridspec_kw={"width_ratios": (1.46, 1, 1)})
    fig.subplots_adjust(left=.065, right=.98, bottom=.19, top=.82, wspace=.28)
    fig.suptitle("Fitzpatrick static-obstacle layouts", fontsize=19, weight="bold", x=.5, y=.97)
    fig.text(.5, .90, "Fixed 44.73 m reference lap    ·    Single-car simulation",
             ha="center", fontsize=11)
    for ax in axes:
        ax.imshow(shade, extent=extent, origin="upper", interpolation="nearest",
                  cmap="gray", vmin=0, vmax=1)
        ax.plot(route.xy[:, 0], route.xy[:, 1], **route_style)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("Map x (m)")
        ax.tick_params(direction="out", length=3)
    axes[0].set_ylabel("Map y (m)")
    axes[0].set_title("Archived A + B placements", fontsize=13, pad=11)
    axes[0].set_xlim(-3.5, 10.6)
    axes[0].set_ylim(-2.2, 17.6)
    axes[0].set_xticks([-2, 2, 6, 10])
    axes[0].set_yticks([0, 4, 8, 12, 16])
    start = route.xy[0]
    axes[0].plot(*start, marker="s", color="white", markeredgecolor=".1", markersize=5)
    axes[0].annotate("Start / finish", start, (3.5, -.7), ha="left", fontsize=9,
                     arrowprops={"arrowstyle": "-", "lw": .8})
    for obstacle, label in zip(placements, ((5., 3.75), (.65, 5.85))):
        pos = obstacle["center_xy"]
        axes[0].add_patch(Circle(pos, obstacle["desired_diameter_m"]/2,
                                 edgecolor="black", facecolor="black", zorder=4))
        axes[0].annotate(obstacle["name"], pos, label, ha="center", fontsize=11, weight="bold",
                         arrowprops={"arrowstyle": "->", "shrinkB": 2, "lw": .8})
    first, second = audits[0]["placements"]
    zoom_x = (route.at_s(first["route_distance_m"])[0]-1.4,
              route.at_s(first["route_distance_m"])[0]+1.4)
    zoom_y = (second["center_xy"][1]-.68, first["center_xy"][1]+.70)
    axes[0].add_patch(Rectangle((zoom_x[0], zoom_y[0]), np.ptp(zoom_x), np.ptp(zoom_y),
                                fill=False, edgecolor=".15", linewidth=.9, linestyle=":"))
    axes[0].annotate("Detail", (zoom_x[1], zoom_y[1]), (.35, 11.45), ha="left",
                     fontsize=9, arrowprops={"arrowstyle": "-", "lw": .7})
    for ax, audit, title in zip(axes[1:], audits, ("Vehicle-left first", "Vehicle-right first")):
        ax.set_title(title, fontsize=13, pad=11)
        ax.set_xlim(zoom_x)
        ax.set_ylim(zoom_y)
        ax.set_xticks([-2, -1, 0])
        ax.set_yticks([7, 8, 9, 10])
        ax.annotate("", (route.at_s(34.9)[0], 8.35), (route.at_s(34.9)[0], 9.00),
                    arrowprops={"arrowstyle": "-|>", "lw": 1.5, "color": ".15"})
        for p in audit["placements"]:
            x, y = p["center_xy"]
            radius = p["desired_diameter_m"]/2
            ax.add_patch(Circle((x, y), radius, edgecolor="black",
                                facecolor="white", hatch="////", linewidth=1.35, zorder=4))
            # Keep labels on the outer side of each cylinder.
            sign = np.sign(x-route.at_s(p["route_distance_m"])[0])
            ax.text(x + sign*(radius+.09), y, p["name"], va="center",
                    ha="left" if sign>0 else "right", fontsize=11, weight="bold")
    legend = [Line2D([], [], label="Reference route", **route_style),
              Patch(facecolor=".20", label="Wall"),
              Patch(facecolor=".87", edgecolor=".7", label="Unknown"),
              Patch(facecolor="white", hatch="////", edgecolor="black", label="Designed cylinder")]
    fig.legend(handles=legend, loc="lower center", bbox_to_anchor=(.5, .083),
               ncol=4, frameon=False, fontsize=10)
    fig.text(.5, .043, "Historical A/B diameter 0.10 m.    S1/S2 diameter 0.40 m, "
             "2.50 m apart along the route, offset 0.18 m to either side.", ha="center", fontsize=10)
    fig.savefig(output/"layout.png", dpi=200, facecolor="white")
    plt.close(fig)
    (output/"geometry.json").write_text(json.dumps(audits, indent=2, allow_nan=False)+"\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    draw(parser.parse_args().output)
