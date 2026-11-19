#!/usr/bin/env python3
"""Physical-map synthetic scan fixtures and regression checks. No ROS or vehicle transport."""
import argparse
import io
import json
import math
import unittest
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from algorithm_shadow import ASSETS, CURVATURE, LIMITS, METHODS, REPO, PathPlanner, Route, Scene, validate_snapshot
from fitz_tracking_classic import ClassicController
from shadow import replay


def synthetic(case, frames=8):
    """1081 Hokuyo-style rays intersect the actual PGM walls and an optional cylinder."""
    mapfile = REPO / "real_vehicle/maps/Fitzpatrick.yaml"
    meta = yaml.safe_load(mapfile.read_text())
    with Image.open(mapfile.parent / meta["image"]) as image:
        free = np.asarray(image) > int(round(255*(1-meta["free_thresh"])))
    route = Route()
    near = route.at_s(3.)
    # Only the fixture renderer sees the cylinder center.
    center = near[:2]+.12*np.array([-math.sin(near[2]), math.cos(near[2])])
    angles = np.linspace(-3*np.pi/4., 3*np.pi/4., 1081)
    distances = np.arange(.05, 12.01, .025)

    def raycast(x, y, heading):
        laser = np.array([x, y]) + .27*np.array([math.cos(heading), math.sin(heading)])
        rays = np.column_stack((np.cos(heading+angles), np.sin(heading+angles)))
        cells = np.floor((laser + distances[None, :, None]*rays[:, None, :] -
                          meta["origin"][:2])/meta["resolution"]).astype(int)
        col, row = cells[:, :, 0], free.shape[0]-1-cells[:, :, 1]
        inside = (col >= 0) & (col < free.shape[1]) & (row >= 0) & (row < free.shape[0])
        hit = ~inside | ~free[row.clip(0, free.shape[0]-1), col.clip(0, free.shape[1]-1)]
        ranges = np.where(hit.any(axis=1), distances[hit.argmax(axis=1)], 12.)
        if case != "clear":
            offset = center-laser
            projected = rays @ offset
            discriminant = .12**2 - (offset @ offset - projected**2)
            intercept = np.where((discriminant >= 0) & (projected > 0),
                                 projected-np.sqrt(discriminant.clip(0)), np.inf)
            ranges = np.minimum(ranges, intercept)
        return ranges.round(5).tolist()

    for index in range(frames):
        t = index*.05
        # A prescribed rising-speed segment along the route for scan/odom replay.
        # This trajectory is deterministic and independent of every tested command.
        station = .55*t**3/6 if case == "moving" else 0.
        speed = .55*t**2/2 if case == "moving" else 0.
        x, y, heading, _ = route.at_s(station)
        yaw_rate = speed*(route.at_s(station+.01)[2]-heading)/.01
        if index == 0 or case == "moving":
            ranges = raycast(x, y, heading)
        stamp = 1_790_000_000. + t
        yield dict(schema="pnc.vehicle.v1", stamp=stamp, scan_stamp=stamp, odom_stamp=stamp,
                   x=float(x), y=float(y), yaw=float(heading), v=float(speed), vy=0., yaw_rate=float(yaw_rate),
                   scan_pose=[float(x), float(y), float(heading)], laser_offset_x=.27,
                   angle_min=float(angles[0]), angle_increment=float(angles[1]-angles[0]),
                   range_min=.05, range_max=12., ranges=ranges)


class ShadowTests(unittest.TestCase):
    def test_invalid_input(self):
        frame = next(synthetic("clear"))
        for change in (dict(ranges=[None]), dict(laser_offset_x=.275),
                       dict(scan_stamp=frame["scan_stamp"]+.7), dict(range_min=-1)):
            with self.assertRaises(ValueError):
                validate_snapshot(dict(frame, **change), None)
        with self.assertRaises(ValueError):
            validate_snapshot(frame, frame)

    def test_cached_path_rechecked_against_new_obstacle(self):
        route = Route()
        scene = Scene(json.loads(ASSETS.read_text()))
        frame = next(synthetic("clear"))
        planner = PathPlanner("lattice", route, scene, seed=11, max_curvature=CURVATURE)
        obstacle = [*route.at_s(3.)[:2], .15]
        planner.reference(frame, 0., np.asarray([obstacle]), frame["stamp"])
        original = planner.cached
        self.assertIsNotNone(original)
        # A new return overlaps the ego at the first cached sample. Revalidation
        # must reject the cached path even if its rejoin target has not changed.
        blocked = np.asarray([obstacle, [frame["x"], frame["y"], .15]])
        with self.assertRaises(ValueError):
            planner.reference(frame, 0., blocked, frame["stamp"]+.05)
        self.assertIsNone(planner.cached)

    def test_same_qp_on_optional_osqp(self):
        try:
            import osqp
        except ImportError:
            self.skipTest("OSQP optional; physical replay uses SciPy")
        route = Route()
        state = dict(x=1., y=-.41, yaw=.06, v=.65)
        ref = route.window(.4)
        a = ClassicController("linear_mpc", LIMITS)
        b = ClassicController("linear_mpc", dict(LIMITS, qp_backend="osqp"))
        for _ in range(3):
            np.testing.assert_allclose(a.step(state, ref), b.step(state, ref), atol=2.e-5, rtol=0)

    def test_all_replay_methods(self):
        # Use actual deadlines. Target load may reject a candidate; this test
        # distinguishes solver timing/feasibility from absent/miswired methods.
        for case in ("clear", "obstacle"):
            src = io.StringIO("\n".join(json.dumps(f) for f in synthetic(case, 4)))
            out = io.StringIO()
            status = replay(src, out, METHODS)
            self.assertIn(status, (0, 1), out.getvalue()[:500])
            rows = [json.loads(s) for s in out.getvalue().splitlines()]
            self.assertEqual({r["method"] for r in rows}, set(METHODS))
            self.assertTrue(all(not r["actuation_authorized"] for r in rows))
            self.assertTrue(all((r["speed"] is not None) == r["valid"] for r in rows))
            self.assertTrue(all((r["steering"] is not None) == r["valid"] for r in rows))
            for r in rows:
                if r["valid"]:
                    self.assertTrue(0 <= r["speed"] <= 2.)
                    self.assertLessEqual(abs(r["steering"]), .26901)
            if case == "obstacle":
                self.assertTrue(any(r["confirmed_discs"] > 0 for r in rows))
            else:
                self.assertTrue(any(r["valid"] for r in rows))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", choices=("clear", "obstacle", "moving"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.fixture:
        if not args.output:
            parser.error("--fixture requires --output")
        with args.output.open("x") as stream:
            for row in synthetic(args.fixture, 32 if args.fixture == "moving" else 8):
                print(json.dumps(row, allow_nan=False, separators=(",", ":")), file=stream)
    else:
        unittest.main(argv=[__file__])
