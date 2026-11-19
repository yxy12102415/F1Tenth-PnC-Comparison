"""Bounded passive wait behavior with a fake identity and fully matched sensor clock."""
import io
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from unittest.mock import Mock

from commissioned_zero_capture import CaptureWindow, capture


class Clock:
    def __init__(self):
        self.t = 0.

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class WaitZeroTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.dest = Path(self.temp.name)/"capture.jsonl"
        self.clock = Clock()
        self.verifier = Mock()

    def run_capture(self, observer):
        with redirect_stderr(io.StringIO()):
            return capture(self.dest, 1.5, 3., self.verifier, observer, self.clock.now, self.clock.sleep)

    def test_waits_past_wrong_session_and_startup_then_records_only_zero(self):
        self.verifier.check.side_effect = [dict(status=s) for s in
                                          ("DIFFERENT_RELEASE", "MISSING_SUPERVISOR", "VERIFIED", "VERIFIED")]
        attempts = []
        def observe(args, output, window):
            self.assertEqual(args.mode, "zero")
            self.assertFalse(self.dest.exists())
            attempts.append(args)
            if len(attempts) == 1:
                return {"status": "NOT_READY", "captured_rows": 0}, 1
            for i in range(31):
                self.assertTrue(window.include(1000.+i*.05))
                output.write(str(i)+"\n")
            return {"status": "OBSERVED_MATCH", "ready": True, "captured_rows": 31}, 0
        report, code = self.run_capture(observe)
        self.assertEqual(code, 0)
        self.assertEqual(report["captured_scan_span_s"], 1.5)
        self.assertEqual(len(self.dest.read_text().splitlines()), 31)
        self.assertEqual([call.args for call in self.verifier.check.call_args_list], [("zero",)]*4)

    def test_wrong_release_times_out_without_observer_or_output(self):
        self.verifier.check.return_value = {"status": "DIFFERENT_RELEASE"}
        observer = Mock(side_effect=AssertionError("unexpected graph access"))
        report, code = self.run_capture(observer)
        self.assertEqual(code, 1)
        self.assertEqual(report["last_identity_status"], "DIFFERENT_RELEASE")
        self.assertEqual(self.clock.now(), 3.)
        self.assertFalse(self.dest.exists())

    def test_rejects_lost_guard_and_preserves_existing_capture(self):
        self.verifier.check.return_value = {"status": "VERIFIED"}
        def observe(args, output, window):
            window.include(123.)
            output.write("partial\n")
            return {"status": "NOT_READY", "issues": ["guard lost"]}, 1
        report, code = self.run_capture(observe)
        self.assertEqual((code, report["issues"]), (1, ["guard lost"]))
        self.assertEqual(list(self.dest.parent.iterdir()), [])
        self.dest.write_text("keep")
        with self.assertRaises(FileExistsError):
            self.run_capture(observe)
        self.assertEqual(self.dest.read_text(), "keep")

    def test_window_excludes_later_scans_and_rejects_missing_scan(self):
        window = CaptureWindow(1., 0.)
        for i in range(13):
            self.assertTrue(window.include(2000.+i*.08))
        self.assertFalse(window.include(2001.04))
        self.assertTrue(window.complete)
        self.assertAlmostEqual(window.last, 2000.96)
        with self.assertRaises(ValueError):
            window.include(2001.50)


if __name__ == "__main__":
    unittest.main()
