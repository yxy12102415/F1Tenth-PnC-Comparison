#!/usr/bin/env python3
"""Check stage gates and exercise the actual producer, CLI and DDS message types."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import unittest

from nominal_stage import Candidate, DOMAIN, TOPICS


def diagnostic(stamp=100., line=1):
    return dict(schema="pnc.vehicle.shadow.v1", method="pp", source_mode="live_stream", valid=True,
                actuation_authorized=False, reason="shadow", stamp=stamp, odom_stamp=stamp,
                pose_stamp=stamp, input_line=line, compute_ms=2., min_preview_gap_m=.2, speed=.15, steering=-.1)


class InputTests(unittest.TestCase):
    def test_liveness_and_repeated_tick(self):
        c = Candidate("pp")
        self.assertFalse(c.next_output(100., 0.)[2])
        c.accept(diagnostic(), 100., 10.)
        self.assertEqual(c.next_output(100., 10.), (.15, -.1, True, "shadow"))
        self.assertIsNone(c.next_output(100.05, 10.05))
        self.assertEqual(c.next_output(100.11, 10.11)[:3], (0., 0., False))
        c.accept(diagnostic(100.12, 2), 100.12, 10.12)
        self.assertFalse(c.next_output(100.12, 10.12)[2])

    def test_rejected_rows(self):
        for changes in (dict(schema="other"), dict(method="dwa"), dict(source_mode="capture_replay"),
                        dict(valid=False), dict(actuation_authorized=True), dict(reason="failed"),
                        dict(speed=float("nan")), dict(steering=.26901), dict(speed=-.1), dict(speed=2.1),
                        dict(compute_ms=50.1), dict(compute_ms=True), dict(min_preview_gap_m=.064),
                        dict(stamp=100.05), dict(odom_stamp=99.48), dict(pose_stamp=0.)):
            with self.subTest(changes=changes):
                c = Candidate("pp")
                c.accept(dict(diagnostic(), **changes), 100., 1.)
                self.assertEqual(c.next_output(100., 1.)[:3], (0., 0., False))
        for changes in (dict(input_line=1), dict(stamp=100.), dict(pose_stamp=99.99)):
            with self.subTest(changes=changes):
                c = Candidate("pp")
                c.accept(diagnostic(), 100., 1.)
                c.accept(dict(diagnostic(100.05, 2), **changes), 100.05, 1.05)
                self.assertFalse(c.next_output(100.05, 1.05)[2])


def dds_test(stale=False):
    if os.environ.get("ROS_DOMAIN_ID") != DOMAIN or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("DDS test requires the localhost staging domain")
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from test_shadow import synthetic

    fixture = next(iter(synthetic("clear", frames=1)))
    cmd = [sys.executable, str(Path(__file__).with_name("nominal_stage.py")), "--method", "pp"]
    rclpy.init(args=[])
    node = Node("pnc_stage_test_reader", start_parameter_services=False, use_global_arguments=False)
    commands, flags, status = [], [], []
    node.create_subscription(AckermannDriveStamped, TOPICS[0], commands.append, 10)
    node.create_subscription(Bool, TOPICS[1], lambda m: flags.append(m.data), 10)
    node.create_subscription(String, TOPICS[2], lambda m: status.append(json.loads(m.data)), 10)
    child = subprocess.Popen(cmd, stdin=subprocess.PIPE, text=True)

    def wait_for(check, seconds=5.):
        deadline = time.monotonic()+seconds
        while not check():
            if child.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("staging DDS condition failed; latest status " + str(status[-1:]))
            rclpy.spin_once(node, timeout_sec=.01)
    try:
        wait_for(lambda: any(s["reason"] == "waiting_for_input" for s in status), seconds=30.)
        if stale:
            stamp = time.time()-2.
            child.stdin.write(json.dumps(dict(fixture, stamp=stamp, scan_stamp=stamp, odom_stamp=stamp))+"\n")
            child.stdin.flush()
            wait_for(lambda: status and "stale" in status[-1]["reason"] and commands and flags)
            assert not any(flags) and not any(s["valid"] for s in status)
            assert all(m.drive.speed == m.drive.steering_angle == 0. for m in commands)
            print(json.dumps(dict(domain=int(DOMAIN), method="pp", intentional_source_age_s=2.,
                                  valid_rows=0, final_reason=status[-1]["reason"], final_command=[0., 0.])))
            return
        duplicate = subprocess.run(cmd, input="", capture_output=True, text=True, timeout=4)
        assert duplicate.returncode == 2 and "Address already in use" in duplicate.stderr
        # The fixture supplies new system-clock timestamps exclusively inside this test.
        # The native model and Python worker signal their readiness before the first row.
        base = time.monotonic()
        for index in range(8):
            stamp = time.time()
            packet = dict(fixture, stamp=stamp, odom_stamp=stamp, scan_stamp=stamp)
            child.stdin.write(json.dumps(packet) + "\n")
            child.stdin.flush()
            end = base+(index+1)*.05
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=max(0., min(.005, end-time.monotonic())))
        wait_for(lambda: sum(s["valid"] for s in status) == 8 and True in flags and any(d.drive.speed > 0 for d in commands))
        positive = [m for m in commands if m.drive.speed > 0]
        assert all(m.header.frame_id == "base_link" and m.header.stamp.sec > 0 for m in positive)
        good = [s for s in status if s["valid"]]
        assert good[-1]["actuation_authorized"] is False and len({s["input_line"] for s in good}) == len(good)
        child.stdin.close()
        wait_for(lambda: bool(status) and status[-1]["reason"] in
                 ("diagnostic producer closed", "diagnostic producer exited", "diagnostic producer timeout")
                 and not flags[-1] and commands[-1].drive.speed == commands[-1].drive.steering_angle == 0.)
        assert all(node.count_publishers(topic) == 1 for topic in TOPICS)
        print(json.dumps(dict(method="pp", domain=int(DOMAIN), valid_rows=len(good),
                              final_reason=status[-1]["reason"], final_command=[0., 0.],
                              max_compute_ms=max(s["compute_ms"] for s in good), backend_ready_handshake=True,
                              duplicate_publisher_rejected=True, fixture="synthetic current-stamp DDS")))
    finally:
        if not child.stdin.closed:
            child.stdin.close()
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dds", action="store_true")
    parser.add_argument("--stale", action="store_true", help="inject a two-second-old source after native readiness")
    args = parser.parse_args()
    if args.stale and not args.dds:
        parser.error("--stale requires --dds")
    if args.dds:
        dds_test(args.stale)
    else:
        unittest.main(argv=[__file__])
