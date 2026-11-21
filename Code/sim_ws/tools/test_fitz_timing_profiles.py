"""Tests for matched timing schedules used by the Fitzpatrick campaign."""

from collections import Counter
import unittest

from fitz_timing_profiles import LATE, MEAN, WINDOW_TICKS, make_schedule


class TimingProfilesTest(unittest.TestCase):
    def test_matched_schedules(self):
        for seed in (11, 29, 47):
            ordered = make_schedule("ordered", seed, 1100)
            shuffled = make_schedule("shuffled", seed, 1100)
            self.assertEqual(ordered, make_schedule("ordered", seed, 1100))
            self.assertNotEqual(ordered, shuffled)
            for start in range(0, len(ordered), WINDOW_TICKS):
                block = ordered[start:start + WINDOW_TICKS]
                self.assertEqual(Counter(block), Counter(shuffled[start:start + WINDOW_TICKS]))
                late = [k for k, row in enumerate(block) if row == LATE]
                self.assertEqual(late, list(range(late[0], late[0] + len(block) // 4)))
            for j in range(3):
                self.assertEqual(sum(row[j] for row in ordered), 1100 * MEAN[j])
            for schedule in (ordered, shuffled):
                for age_channel in (0, 1):
                    stamps = [tick * 5 - row[age_channel] for tick, row in enumerate(schedule)]
                    self.assertEqual(stamps, sorted(stamps))
            self.assertEqual(set(make_schedule("mean", seed, 1100)), {MEAN})

    def test_sync_and_inputs(self):
        self.assertEqual(make_schedule("sync", 11, 2), ((0, 0, 0), (0, 0, 0)))
        self.assertNotEqual(make_schedule("ordered", 11, 1100), make_schedule("ordered", 29, 1100))
        self.assertEqual(len(make_schedule("ordered", 11, 43)), 43)
        with self.assertRaises(ValueError):
            make_schedule("unknown", 11, 20)


if __name__ == "__main__":
    unittest.main()
