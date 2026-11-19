"""Offline regression tests for the physical v8 observer; standard library only."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parent))
from session_identity import ASSETS, BINDIR, CONTROL_EXE, PLANNER_EXE, SUPERVISOR, SessionIdentity
from contract import (ACTUATOR, CONTROL, DRIVE, ODOM_TOPIC, POSE_TOPIC, SCAN_TOPIC, VALID_TOPIC,
                      VESC, Samples, aligned_pose, channels, graph_issues)


def header(stamp, frame):
    nanos = round(stamp * 1e9)
    return NS(stamp=NS(sec=nanos // 10**9, nanosec=nanos % 10**9), frame_id=frame)


def odom(stamp, frame):
    q = NS(x=0., y=0., z=0., w=1.)
    pose = NS(position=NS(x=1., y=2.), orientation=q)
    return NS(header=header(stamp, frame), child_frame_id="base_link", pose=NS(pose=pose),
              twist=NS(twist=NS(linear=NS(x=-.3, y=.2), angular=NS(z=.5))))


def scan(stamp):
    return NS(header=header(stamp, "laser"), angle_min=-1., angle_increment=.02,
              range_min=.02, range_max=10., ranges=[.8, math.inf, .75])


def endpoint(node, spec):
    return dict(node=node, type=spec.type, reliability="RELIABLE", durability="VOLATILE")


def complete_graph(mode):
    graph = {}
    for topic, spec in channels(mode).items():
        writers = [endpoint("/sensor" if spec.owner == "*" else spec.owner, spec)] if spec.owner else []
        graph[topic] = {"publishers": writers, "subscriptions": [endpoint(n, spec) for n in spec.consumers]}
    return graph


def sensor_set(s, stamp, receipt=8.):
    for topic, msg in ((POSE_TOPIC, odom(stamp, "map")), (SCAN_TOPIC, scan(stamp)),
                       (ODOM_TOPIC, odom(stamp, "odom")), (VALID_TOPIC, NS(data=True))):
        s.receive(topic, msg, stamp + .01, receipt)


class TopologyTests(unittest.TestCase):
    def test_expected_roles_and_extra_observers(self):
        for mode in ("run", "zero"):
            topics = channels(mode)
            graph = complete_graph(mode)
            topic = "/baseline/final"
            graph[topic]["subscriptions"].append(endpoint("/third_party_monitor", topics[topic]))
            self.assertEqual(graph_issues(topics, graph, mode), [])

    def test_competing_writer_and_writer_replacement(self):
        topics = channels("run")
        graph = complete_graph("run")
        topic = "/baseline/drive_nominal"
        graph[topic]["publishers"].append(endpoint(CONTROL, topics[topic]))
        graph["/stand_pf/motor_current"]["publishers"][0]["node"] = "/another_actuator"
        issues = graph_issues(topics, graph, "run")
        self.assertTrue(any("/baseline/drive_nominal" in i for i in issues))
        self.assertTrue(any("/another_actuator" in i for i in issues))

    def test_unguarded_input_missing_consumer_wrong_type(self):
        topics = channels("run")
        graph = complete_graph("run")
        graph["/stand_pf/disabled_brake"]["publishers"].append(endpoint("/unexpected", topics["/stand_pf/disabled_brake"]))
        graph["/stand_pf/actuator_input"]["subscriptions"] = []
        graph[POSE_TOPIC]["publishers"][0]["type"] = DRIVE
        issues = graph_issues(topics, graph, "run")
        self.assertEqual(len(issues), 3)
        self.assertTrue(any(ACTUATOR in i for i in issues))

    def test_zero_run_servo_and_operator(self):
        topics = channels("zero")
        graph = complete_graph("zero")
        graph["/stand_pf/servo"]["subscriptions"].append(endpoint(VESC, topics["/stand_pf/servo"]))
        graph["/baseline/enable"]["publishers"] = [endpoint("/route_operator_gate", topics["/baseline/enable"])]
        self.assertEqual(len(graph_issues(topics, graph, "zero")), 2)

    def test_incompatible_and_unknown_qos(self):
        topics = channels("run")
        graph = complete_graph("run")
        graph[POSE_TOPIC]["publishers"][0]["reliability"] = "BEST_EFFORT"
        graph["/stand_pf/servo"]["publishers"][0]["durability"] = "UNKNOWN"
        self.assertTrue(any("QoS" in i for i in graph_issues(topics, graph, "run")))
        graph[POSE_TOPIC]["publishers"][0]["reliability"] = "RELIABLE"
        graph["/stand_pf/servo"]["publishers"][0]["durability"] = "VOLATILE"
        for r in graph[SCAN_TOPIC]["subscriptions"]:
            r["reliability"] = "BEST_EFFORT"
        graph[SCAN_TOPIC]["publishers"][0]["reliability"] = "BEST_EFFORT"
        self.assertEqual(graph_issues(topics, graph, "run"), [])


class SampleTests(unittest.TestCase):
    def test_two_clocks_and_replay_schema(self):
        s = Samples("zero")
        sensor_set(s, 1000.)
        self.assertIsNone(s.snapshot(1000.506, 8.1))
        self.assertIsNone(s.snapshot(1000.1, 8.501))
        row = s.snapshot(1000.01, 8.01)
        self.assertEqual(row["ranges"], [.8, None, .75])
        self.assertEqual(row["scan_pose"], (1., 2., 0.))
        self.assertEqual(row["v"], .3)
        self.assertEqual(row["odom_stamp"], row["scan_stamp"])
        json.dumps(row, allow_nan=False)
        self.assertIsNone(s.snapshot(1000.02, 8.02))

    def test_invalid_scan_and_low_quality(self):
        s = Samples("run")
        sensor_set(s, 10.)
        s.receive(VALID_TOPIC, NS(data=False), 10.01, 8.01)
        self.assertIsNone(s.snapshot(10.01, 8.01))
        bad = scan(10.1)
        bad.header.frame_id = "/laser"
        s.receive(SCAN_TOPIC, bad, 10.101, 8.1)
        self.assertIn("frame", s.issue(SCAN_TOPIC, 10.101, 8.1))

    def test_future_duplicate_recovery_and_planarity(self):
        s = Samples("run")
        sensor_set(s, 10.)
        s.receive(POSE_TOPIC, odom(10.2, "map"), 10.1, 8.1)
        self.assertIn("clock", s.issue(POSE_TOPIC, 10.1, 8.1))
        s.receive(POSE_TOPIC, odom(10., "map"), 10.1, 8.11)
        self.assertIn("nonincreasing", s.issue(POSE_TOPIC, 10.1, 8.11))
        bad = odom(10.1, "map")
        bad.pose.pose.orientation.x = .1
        s.receive(POSE_TOPIC, bad, 10.11, 8.11)
        self.assertIn("quaternion", s.issue(POSE_TOPIC, 10.11, 8.11))
        s.receive(POSE_TOPIC, odom(10.1, "map"), 10.11, 8.12)
        self.assertIsNone(s.issue(POSE_TOPIC, 10.11, 8.13))

    def test_alignment_and_wrapped_heading(self):
        trace = [(1., (0., 0., 3.13)), (1.1, (1., 2., -3.13))]
        for stamp in (.92, 1.18):
            self.assertIsNone(aligned_pose(trace, stamp))
        self.assertEqual(aligned_pose(trace, .93), trace[0][1])
        self.assertEqual(aligned_pose(trace, 1.17), trace[-1][1])
        p = aligned_pose(trace, 1.05)
        self.assertAlmostEqual(p[0], .5)
        self.assertAlmostEqual(p[2], math.pi)
        self.assertIsNone(aligned_pose([(1., trace[0][1]), (1.2, trace[1][1])], 1.1))

    def test_settled_zero_and_latched_command_observations(self):
        s = Samples("zero")
        health = dict(erpm=0, voltage=15, fet_temp=20, fault=0, duty_cycle=0, motor_current=0, input_current=0)
        payloads = {
            "/baseline/controller_status": {"reason": "waiting_for_operator", "valid": True},
            "/baseline/final_status": dict(arm_state="DISARMED", armed=False, latched=False, real_vehicle_profile=True,
                                            straight_line_fallback_allowed=False),
            "/stand_pf/health_guard_status": {"healthy": True, "qualified": True},
            "/stand_pf/actuator_status": {"motor_mode": "coast", "stop_reason": "", "rpm": 0., "current": 0., "servo": .4304},
            "/stand_pf/window_status": {"latched": False, "health_ready": True, "reason": "no_fresh_permit",
                                       "started": None, "speed": 0., "steering": 0.},
        }
        for stamp in (100., 100.05):
            sensor_set(s, stamp)
            s.snapshot(stamp + .01, 8.)
            for topic in ("/baseline/drive_nominal", "/baseline/final", "/stand_pf/actuator_input"):
                msg = NS(header=header(stamp, "" if topic.endswith("actuator_input") else "base_link"),
                         drive=NS(speed=0., steering_angle=0.))
                s.receive(topic, msg, stamp + .01, 8.)
            for topic in ("/baseline/planner_valid", "/stand_pf/health_permit"):
                s.receive(topic, NS(data=True), stamp + .01, 8.)
            for topic, value in payloads.items():
                s.receive(topic, NS(data=json.dumps(value)), stamp + .01, 8.)
            s.receive("/stand_pf/health", NS(data=json.dumps(dict(health, stamp=stamp))), stamp + .01, 8.)
            s.receive("/stand_pf/motor_current", NS(data=0.), stamp + .01, 8.)
            s.receive("/stand_pf/servo", NS(data=.4304), stamp + .01, 8.)
        self.assertEqual(s.report(100.06, 8.01)[0], [])
        s.receive("/stand_pf/motor_speed", NS(data=0.), 100.06, 8.01)
        s.receive("/stand_pf/health_permit", NS(data=False), 100.06, 8.01)
        issues, _ = s.report(100.06, 8.02)
        self.assertTrue(any("speed-mode" in i for i in issues))
        self.assertTrue(any("health_permit: false" in i for i in issues))


class ArchivedStatusTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / "results/jetson_interface_20261120/archived_status_check.json"
        cls.sessions = json.loads(path.read_text())["sessions"]

    def test_actual_vehicle_status_fields_and_shutdown(self):
        streams = {"pp": "/baseline/controller_status", "actuator": "/stand_pf/actuator_status",
                   "final": "/baseline/final_status", "window": "/stand_pf/window_status",
                   "health_guard": "/stand_pf/health_guard_status"}
        for mode, data in self.sessions.items():
            s = Samples(mode)
            for name, topic in streams.items():
                s.receive(topic, NS(data=json.dumps(data["status_fields"][name])), 0., 1.)
                self.assertIsNone(s.issue(topic, 0., 1.), (mode, name))
            h = data["example_health"]
            s.receive("/stand_pf/health", NS(data=json.dumps(h)), h["bridge_ros"], h["receipt_mono"])
            self.assertIsNone(s.issue("/stand_pf/health", h["bridge_ros"], h["receipt_mono"]))
            for name, expected in data["teardown_result"].items():
                topic = streams[name]
                s.receive(topic, NS(data=json.dumps(data["teardown"][name])), 0., 1.)
                self.assertEqual(s.issue(topic, 0., 1.), expected)

    def test_inconsistent_zero_outputs_are_rejected(self):
        s = Samples("zero")
        fields = self.sessions["zero"]["status_fields"]
        for name, topic, edits in (
            ("actuator", "/stand_pf/actuator_status", {"rpm": 20.}),
            ("window", "/stand_pf/window_status", {"reason": "deadman_not_ready"}),
            ("window", "/stand_pf/window_status", {"started": 2840.}),
            ("final", "/baseline/final_status", {"armed": True}),
        ):
            s.receive(topic, NS(data=json.dumps(dict(fields[name], **edits))), 0., 1.)
            self.assertTrue(s.issue(topic, 0., 1.))
        s.receive("/stand_pf/servo", NS(data=.7), 0., 1.)
        self.assertTrue(any("off-center" in issue for issue in s.limits))


class IdentityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.proc = Path(self.temp.name) / "proc"
        self.ws = Path(self.temp.name) / "commissioned"
        self.proc.mkdir()
        self.ws.mkdir()
        self.identity = SessionIdentity(self.ws, self.proc)

    def process(self, pid, parent, exe, args, start=123):
        directory = self.proc / str(pid)
        directory.mkdir(exist_ok=True)
        (directory / "cmdline").write_bytes(b"\0".join(str(x).encode() for x in args) + b"\0")
        fields = ["S", str(parent)] + ["0"] * 17 + [str(start)]
        (directory / "stat").write_text(str(pid) + " (with ) space) " + " ".join(fields))
        (directory / "environ").write_bytes(b"ROS_DOMAIN_ID=85\0ROS_LOCALHOST_ONLY=1\0")
        (directory / "exe").symlink_to(exe)
        (directory / "cwd").symlink_to(self.ws)
        return directory

    def test_idle_other_track_and_ambiguous_sessions(self):
        self.assertEqual(self.identity.check("zero")["status"], "MISSING_SUPERVISOR")
        other = self.ws / "ground_381_2_0_joint_ble_20261117_v3/stand_follow.py"
        self.process(9, 1, sys.executable, ["python3", other, "--zero-only"])
        self.assertEqual(self.identity.check("zero")["status"], "DIFFERENT_RELEASE")
        self.process(10, 1, sys.executable, ["python3", self.ws / SUPERVISOR, "--zero-only"])
        self.assertEqual(self.identity.check("zero")["status"], "MULTIPLE_SUPERVISORS")

    def test_canonical_path_mode_pid_digests_and_child_assets(self):
        import hashlib
        (self.ws / BINDIR).mkdir(parents=True)
        expected = {}
        for path in (SUPERVISOR, ASSETS, CONTROL_EXE, PLANNER_EXE):
            data = b"elf fixture" if path in (CONTROL_EXE, PLANNER_EXE) else str(path).encode()
            (self.ws / path).write_bytes(data)
            expected[path] = hashlib.sha256(data).hexdigest()
        alias = self.ws / "alias.py"
        alias.symlink_to(self.ws / SUPERVISOR)
        main = self.process(10, 1, sys.executable, ["python3", "-u", alias, "--zero-only"])
        self.assertEqual(self.identity.check("run")["status"], "DIFFERENT_MODE")
        self.assertEqual(self.identity.check("zero", (10, 99))["status"], "SUPERVISOR_CHANGED")
        for index, exe in enumerate((PLANNER_EXE, CONTROL_EXE), 11):
            self.process(index, 10, self.ws / exe, [self.ws / exe, "-p", "assets_file:=" + str(self.ws / ASSETS)])
        with patch("session_identity.PINNED", expected), patch("session_identity.BINARY_DIGEST", expected[PLANNER_EXE]):
            self.assertEqual(self.identity.check("zero", (10, 123))["status"], "VERIFIED")
            (self.ws / ASSETS).write_text("changed")
            self.assertEqual(self.identity.check("zero")["status"], "IDENTITY_UNVERIFIED")
            child = self.proc / "11/cmdline"
            child.write_bytes(child.read_bytes().replace(b"runtime_assets.json", b"wrong_track.json"))
            self.assertTrue(any("assets binding" in x for x in self.identity.check("zero")["issues"]))
            (main / "environ").write_bytes(b"ROS_DOMAIN_ID=83\0ROS_LOCALHOST_ONLY=1\0")
            self.assertTrue(any("environment" in x for x in self.identity.check("zero")["issues"]))


class FoxyQosTest(unittest.TestCase):
    def test_vendor_names_report_the_same_negotiated_policies(self):
        from preflight import qos_name
        for family, names in (("RELIABILITY", ("RELIABLE", "BEST_EFFORT")),
                              ("DURABILITY", ("VOLATILE", "TRANSIENT_LOCAL")),
                              ("HISTORY", ("KEEP_LAST",))):
            for name in names:
                self.assertEqual(qos_name(NS(name=name), family), name)
                self.assertEqual(qos_name(NS(name="RMW_QOS_POLICY_" + family + "_" + name), family), name)
        self.assertEqual(qos_name(NS(name="SYSTEM_DEFAULT"), "RELIABILITY"), "SYSTEM_DEFAULT")


if __name__ == "__main__":
    unittest.main()
