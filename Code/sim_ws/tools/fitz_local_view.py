"""Interactive observer for a single Fitzpatrick trial, with in-memory replay."""
from bisect import bisect_right
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon
from matplotlib.patheffects import withStroke

from fitz_local_report import CASES, LABELS
from fitz_local_scene import LENGTH, WIDTH, _background


REFERENCE = "#566776"
DRIVEN = "#087e77"
PLANNED = "#8c47ad"
OBSTACLE = "#c64822"
BODY = np.array([[LENGTH, WIDTH], [LENGTH, -WIDTH], [-LENGTH, -WIDTH],
                 [-LENGTH, WIDTH]]) / 2


class LocalView:
    """Call update after each step, then finish after saving the trial result."""

    def __init__(self, route, placements, method, case, seed):
        self.root = tk.Tk()
        self.root.title(f"Fitzpatrick · {LABELS.get(method, method)} · {case} · seed {seed}")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<Escape>", lambda event: self.close())
        self.open = True
        self.route = route
        self.placements = sorted(placements, key=lambda p: p["route_distance_m"])
        self.times = []
        self.poses = []
        self.stations = []
        self.paths = []
        self.index = None
        self.origin_time = None
        self.replay_timer = None
        self.result = None
        self.detail_key = None
        self.backdrop = None

        screen_w, screen_h = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.root.geometry(f"{min(1040, int(.91*screen_w))}x{min(730, int(.84*screen_h))}")
        header = ttk.Frame(self.root, padding=(16, 10, 16, 0))
        header.pack(fill="x")
        ttk.Label(header, text=f"{LABELS.get(method, method)}  ·  {CASES.get(case, case)}",
                  font=("TkDefaultFont", 14, "bold")).pack(anchor="w")
        self.status = ttk.Label(header, text=f"Starting simulation   ·   Seed {seed}")
        self.status.pack(anchor="w", pady=(3, 2))

        self.figure = Figure(figsize=(10, 5), facecolor="white")
        self.axes = self.figure.subplots(1, 2)
        self.figure.subplots_adjust(left=.075, right=.97, bottom=.17, top=.91, wspace=.22)
        _, origin, resolution, grid, _ = _background()
        xmax, ymax = origin + resolution * np.array(grid.shape[::-1])
        extent = (origin[0], xmax, origin[1], ymax)
        cmap = ListedColormap(["#37434a", "#dce1e3", "#ffffff"])
        norm = BoundaryNorm([-.5, 127.5, 249.5, 255.5], cmap.N)
        self.moving = []
        for ax in self.axes:
            ax.imshow(grid, extent=extent, origin="lower", interpolation="nearest",
                      cmap=cmap, norm=norm)
            ax.set_aspect("equal")
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.tick_params(labelsize=8)
            ax.plot(*route.xy.T, color=REFERENCE, lw=1.3, linestyle="--")
            for obstacle in self.placements:
                x, y = obstacle["center_xy"]
                ax.add_patch(Circle((x, y), obstacle["desired_diameter_m"]/2,
                                    facecolor=OBSTACLE, edgecolor="#853115", zorder=4))
                ax.annotate(obstacle["name"], (x, y), xytext=(7, 5), textcoords="offset points",
                            color="#8a3014", fontsize=9, weight="bold", zorder=5,
                            path_effects=[withStroke(linewidth=2.5, foreground="white")])
            trail, = ax.plot([], [], color=DRIVEN, lw=2.0, animated=True)
            plan, = ax.plot([], [], color=PLANNED, lw=1.9, linestyle="-.", animated=True)
            car = Polygon(BODY, closed=True, facecolor=DRIVEN, edgecolor="#0a3c3b", animated=True)
            ax.add_patch(car)
            nose, = ax.plot([], [], color="white", lw=1.5, animated=True)
            self.moving.append((trail, plan, car, nose))
        self.axes[0].set(title="Full lap · north up",
                         xlim=(route.xy[:, 0].min()-1.7, route.xy[:, 0].max()+1.7),
                         ylim=(route.xy[:, 1].min()-1.4, route.xy[:, 1].max()+1.4))
        self._detail(0., route.xy[0])
        handles = [Line2D([], [], color=REFERENCE, ls="--", label="Reference"),
                   Line2D([], [], color=DRIVEN, lw=2, label="Driven path"),
                   Line2D([], [], color=PLANNED, ls="-.", label="Selected plan")]
        if self.placements:
            handles.append(Line2D([], [], ls="", marker="o", color=OBSTACLE, label="Obstacle"))
        self.figure.legend(handles=handles, loc="lower center", ncol=len(handles),
                           frameon=False, fontsize=9)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.mpl_connect("draw_event", self._cache_background)

        footer = ttk.Frame(self.root, padding=(16, 2, 16, 10))
        footer.pack(side="bottom", fill="x")
        self.replay_lap = ttk.Button(footer, text="Replay lap", command=lambda: self._replay(0))
        self.replay_lap.pack(side="left")
        self.replay_lap.state(["disabled"])
        self.replay_approach = ttk.Button(footer, text="Replay approach", command=self._approach)
        if self.placements:
            self.replay_approach.pack(side="left", padx=6)
        self.replay_approach.state(["disabled"])
        ttk.Button(footer, text="Close", command=self.close).pack(side="right")
        widget = self.canvas.get_tk_widget()
        widget.pack(fill="both", expand=True)
        self.root.update()
        if self.open:
            # Tk can apply display scaling after the first Configure event.
            self.figure.set_size_inches(widget.winfo_width() / self.figure.dpi,
                                        widget.winfo_height() / self.figure.dpi, forward=False)
            self.canvas.draw()

    def _detail(self, station, xy):
        """Keep nearby staggered obstacles together. Switch distant obstacles by station."""
        if self.placements:
            focus = min(self.placements, key=lambda p: abs(station - p["route_distance_m"]))
            group = [p for p in self.placements
                     if abs(focus["route_distance_m"] - p["route_distance_m"]) <= 3.]
            key = tuple(p["name"] for p in group)
            if key == self.detail_key:
                return False
            centers = np.array([p["center_xy"] for p in group])
            center = (centers.max(axis=0) + centers.min(axis=0)) / 2
            half = np.maximum([1.9, 1.9], .5 * np.ptp(centers, axis=0) + 1.65)
            title = "Obstacle detail · " + " + ".join(key)
        else:
            center = np.asarray(xy)
            if self.detail_key is not None and np.max(np.abs(center - self.detail_key)) < 1.:
                return False
            key, half, title = center.copy(), (2.3, 2.3), "Vehicle detail"
        self.detail_key = key
        x, y = center
        self.axes[1].set(xlim=(x-half[0], x+half[0]), ylim=(y-half[1], y+half[1]), title=title)
        return True

    def _draw_artists(self):
        if self.index is not None:
            for ax, group in zip(self.axes, self.moving):
                for artist in group:
                    ax.draw_artist(artist)

    def _cache_background(self, event):
        self.backdrop = self.canvas.copy_from_bbox(self.figure.bbox)
        self._draw_artists()

    def _draw(self, index, mode):
        self.index = index
        x, y, yaw, speed = self.poses[index]
        station, path = self.stations[index], self.paths[index]
        trail_xy = np.asarray(self.poses[:index+1])[:, :2]
        c, s = np.cos(yaw), np.sin(yaw)
        body_xy = BODY @ np.array([[c, s], [-s, c]]) + [x, y]
        for trail, plan, car, nose in self.moving:
            trail.set_data(*trail_xy.T)
            plan.set_data(*path.T)
            car.set_xy(body_xy)
            nose.set_data([x, x+.46*LENGTH*c], [y, y+.46*LENGTH*s])
        self.status.configure(text=(f"{mode}    ·    {self.times[index]:.1f} s    ·    "
                                    f"{station:.1f} / {self.route.length:.1f} m    ·    "
                                    f"{speed:.2f} m/s"))
        if self._detail(station, (x, y)) or self.backdrop is None:
            self.canvas.draw()
        else:
            self.canvas.restore_region(self.backdrop)
            self._draw_artists()
            self.canvas.blit()

    def update(self, state, sim_time, station, path):
        """Observe the real state and selected path once per tick. False on user close."""
        if not self.open:
            return False
        self.root.update()
        if not self.open:
            return False
        now = time.monotonic()
        if self.origin_time is None:
            self.origin_time = now - sim_time
        # A slow planning tick should remain visible instead of accelerating later
        # frames to catch up with a missed wall-clock deadline.
        self.origin_time = max(self.origin_time, now - sim_time)
        self.times.append(float(sim_time))
        self.poses.append(tuple(state[k] for k in ("x", "y", "yaw", "v")))
        self.stations.append(float(station))
        self.paths.append(np.empty((0, 2)) if path is None else np.asarray(path)[:, :2].copy())
        while self.open:
            delay = self.origin_time + sim_time - time.monotonic()
            if delay <= 0:
                self._draw(len(self.times)-1, "Live")
                self.root.update()
                break
            time.sleep(min(.015, delay))
            self.root.update()
        return self.open

    def _approach(self):
        target = self.placements[0]["route_distance_m"] - 5.
        start = next((i for i, s in enumerate(self.stations) if s >= target), 0)
        self._replay(start)

    def _replay(self, start):
        if self.replay_timer is not None:
            self.root.after_cancel(self.replay_timer)
        first_t = self.times[start]
        started = time.monotonic()

        def advance():
            elapsed = time.monotonic() - started
            i = max(start, bisect_right(self.times, first_t + elapsed) - 1)
            if i == len(self.times)-1:
                self.replay_timer = None
                self._draw(i, self.result)
            else:
                self._draw(i, "Replay")
                delay = self.times[i+1] - first_t - (time.monotonic() - started)
                self.replay_timer = self.root.after(max(1, round(1000*delay)), advance)

        advance()

    def finish(self, result):
        """Keep the terminal frame and replay controls available until the viewer closes."""
        if not self.open:
            return
        self.result = "Finished · " + result["status"].replace("_", " ")
        if self.times:
            self._draw(len(self.times)-1, self.result)
            self.replay_lap.state(["!disabled"])
            if self.placements:
                self.replay_approach.state(["!disabled"])
        else:
            self.status.configure(text=self.result)
        self.root.mainloop()

    def close(self):
        if self.open:
            self.open = False
            if self.replay_timer is not None:
                self.root.after_cancel(self.replay_timer)
            self.root.destroy()
