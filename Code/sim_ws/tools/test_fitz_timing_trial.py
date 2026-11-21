"""Causality checks using a deterministic tick-labelled plant (no Gym/ROS execution)."""
import csv
import gzip
import tempfile
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

from fitz_timing_profiles import TimingSample
import fitz_timing_trial as experiment


class StraightRoute:
    length = 1000.

    def project(self, state, previous):
        return state["x"], state["y"]

    def at_s(self, progress):
        return progress, 0., 0., 1.

    def window(self, progress):
        return progress


class TickPlant:
    def __init__(self):
        self.commands = []
        self.sim_time = 0.
        self.state = self.observation()

    def observation(self):
        index = len(self.commands)
        return dict(x=float(index), y=0., yaw=0., v=float(index), vy=0., yaw_rate=0.,
                    collision=False, boundary=False)

    def step_physics(self, speed, angle):
        self.commands.append(speed)
        self.sim_time = .01 * len(self.commands)
        self.state = self.observation()
        return self.state

    def close(self):
        pass


class EventTest(unittest.TestCase):
    def simulate(self, schedule):
        self.plant = TickPlant()
        self.seen = []

        @contextmanager
        def controller(*args, **kwargs):
            def solve(state, ref, remaining):
                self.seen.append((state["x"], state["v"], ref, remaining))
                return .05 * len(self.seen), 0., 0.
            yield solve

        with tempfile.TemporaryDirectory() as tmp, patch.object(experiment, "Plant", return_value=self.plant), \
                patch.object(experiment.tracking, "controller", controller):
            folder = Path(tmp) / "test"
            summary = experiment.trial(StraightRoute(), "pp", "center", 11, folder, .05*len(schedule), schedule)
            with gzip.open(folder / "commands.csv.gz", "rt") as f:
                rows = list(csv.DictReader(f))
        return summary, rows

    def test_sensor_age_and_reference_are_causal(self):
        self.simulate([TimingSample(0, 0, 0), TimingSample(3, 1, 0), TimingSample(9, 0, 0)])
        # Second release sees x at tick 2 and speed at tick 4. The third, arriving
        # with an older pose source, keeps the most recent x and advances odometry.
        self.assertEqual(self.seen, [(0., 0., 0., 1000.), (2., 4., 2., 998.), (2., 10., 2., 998.)])

    def test_zero_delay_matches_five_physics_steps_per_command(self):
        self.simulate([TimingSample(0, 0, 0)] * 2)
        self.assertEqual(self.plant.commands, [.05]*5 + [.10]*5)

    def test_later_command_supersedes_old_and_simultaneous_arrival(self):
        for delay, next_delay in [(9, 1), (5, 0)]:
            with self.subTest(delay=delay):
                summary, rows = self.simulate([TimingSample(0, 0, delay), TimingSample(0, 0, next_delay)])
                self.assertEqual([r["disposition"] for r in rows], ["superseded", "applied"])
                applied_tick = 5 + next_delay
                self.assertEqual(self.plant.commands, [0.]*applied_tick + [.1]*(10-applied_tick))
                self.assertEqual(summary["superseded_commands"], 1)

    def test_stop_with_no_fresh_command(self):
        summary, _ = self.simulate([TimingSample(0, 0, 80)] * 16)
        self.assertEqual(summary["status"], "stale_stop")
        self.assertEqual(summary["applied_commands"], 0)
        self.assertTrue(all(v == 0. for v in self.plant.commands))


if __name__ == "__main__":
    unittest.main()
