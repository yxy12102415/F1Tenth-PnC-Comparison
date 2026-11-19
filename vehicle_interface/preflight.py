#!/usr/bin/env python3
"""Read the v8 physical ROS graph and observe the current interface. No drive publisher."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

from session_identity import SessionIdentity, process_stat

from contract import (BOOL, DRIVE, JOY, ODOM, SCALAR, SCAN, SENSORS, TEXT, Samples,
                      channels, graph_issues)


def qos_name(policy, family):
    # Foxy exposes the RMW C enum name. Humble also exposes short policy names.
    prefix = "RMW_QOS_POLICY_" + family + "_"
    name = policy.name
    return name[len(prefix):] if name.startswith(prefix) else name


def endpoint(info):
    qos = info.qos_profile
    return {"node": info.node_namespace.rstrip("/") + "/" + info.node_name, "type": info.topic_type,
            "reliability": qos_name(qos.reliability, "RELIABILITY"),
            "durability": qos_name(qos.durability, "DURABILITY"),
            "history": qos_name(qos.history, "HISTORY"), "depth": qos.depth}


def topology(node, topics):
    graph = {}
    for topic in topics:
        graph[topic] = {
            "publishers": [endpoint(p) for p in node.get_publishers_info_by_topic(topic)],
            "subscriptions": [endpoint(s) for s in node.get_subscriptions_info_by_topic(topic)
                              if s.node_name != node.get_name() or s.node_namespace != node.get_namespace()],
        }
    return graph


def subscribe(node, samples, on_snapshot):
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Joy
    from std_msgs.msg import Bool, String, Float64
    from rclpy.qos import qos_profile_sensor_data

    types = {ODOM: Odometry, SCAN: LaserScan, JOY: Joy, BOOL: Bool, TEXT: String,
             DRIVE: AckermannDriveStamped, SCALAR: Float64}

    def callback(topic):
        def receive(msg):
            ros_now = node.get_clock().now().nanoseconds * 1e-9
            mono_now = time.monotonic()
            samples.receive(topic, msg, ros_now, mono_now)
            if topic in SENSORS:
                row = samples.snapshot(ros_now, mono_now)
                if row is not None and on_snapshot is not None:
                    on_snapshot(row)
        return receive

    for topic, spec in samples.topics.items():
        if not spec.sampled and topic not in ("/stand_pf/motor_speed", "/stand_pf/motor_current"):
            continue
        qos = qos_profile_sensor_data if spec.type in (SCAN, JOY) else 1
        node.create_subscription(types[spec.type], topic, callback(topic), qos)


def observe(args, output, window=None):
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=[])
    node = None
    try:
        # use_global_arguments=False prevents ambient ROS remap arguments on this observer.
        node = Node("pnc_interface_observer_" + str(os.getpid()), start_parameter_services=False,
                    use_global_arguments=False)
        mode = args.mode
        topics = channels("run" if mode == "inventory" else mode)
        samples = Samples(mode)
        verifier = SessionIdentity()
        identity = verifier.check(mode)
        started = time.monotonic()
        graph = {}
        graph_errors = []
        checked_at = None
        captured = 0
        loss = []
        last_match = False
        bound = None
        if mode != "inventory" and identity["status"] == "VERIFIED":
            process = identity["candidates"][0]
            bound = (process["pid"], process["start_ticks"])
            def save(row):
                nonlocal captured, last_match
                # The last graph/identity check must be recent. Recheck process lifetime
                # on every row to avoid attributing a restarted supervisor's data.
                now = time.monotonic()
                if checked_at is None or now - checked_at > .5 or graph_errors or loss:
                    return
                errors, _ = samples.report(node.get_clock().now().nanoseconds * 1e-9, now)
                if errors:
                    if last_match:
                        loss.extend(errors)
                    return
                try:
                    _, ticks = process_stat(verifier.proc / str(bound[0]))
                except OSError:
                    ticks = None
                if ticks != bound[1]:
                    loss.append("supervisor changed during capture")
                    return
                if window is not None:
                    try:
                        if not window.include(row["scan_stamp"]):
                            return
                    except ValueError as exc:
                        loss.append(str(exc))
                        return
                if output is not None:
                    output.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
                if args.stream_snapshots:
                    sys.stdout.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
                    sys.stdout.flush()
                captured += 1
                last_match = True
            subscribe(node, samples, save)
        end = started + args.seconds
        if window is not None:
            end = min(end, window.deadline) if bound else started
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=max(0., min(.05, end - time.monotonic())))
            # Inventory can see a competing graph without making any subscriptions.
            if bound and (checked_at is None or time.monotonic() - checked_at >= .25):
                identity = verifier.check(mode, bound)
                if identity["status"] != "VERIFIED":
                    loss.append(identity["status"])
                    break
                graph = topology(node, topics)
                graph_errors = graph_issues(topics, graph, mode)
                if last_match and graph_errors:
                    loss.extend(graph_errors)
                    break
                checked_at = time.monotonic()
            if loss or (window is not None and window.complete):
                break
        sample_ros = node.get_clock().now().nanoseconds * 1e-9
        sample_mono = time.monotonic()
        identity = verifier.check(mode, bound)
        graph = topology(node, topics)
        observed_nodes = sorted(ns.rstrip("/") + "/" + name for name, ns in node.get_node_names_and_namespaces()
                                if name != node.get_name() or ns != node.get_namespace())
        report = {"domain": 85, "localhost_only": True, "mode": mode, "sample_end_ros_s": sample_ros,
                  "sample_duration_s": sample_mono - started, "nodes": observed_nodes, "graph": graph,
                  "identity": identity, "captured_rows": captured,
                  "ros_distro": os.environ.get("ROS_DISTRO"), "motion_authorized": False}
        if mode == "inventory":
            optional = ("/baseline/map", "/baseline/route", "/initialpose", "/baseline/visual_pose",
                        "/baseline/visual_scan", "/baseline/scan_relay_status", "/baseline/localization_status",
                        "/stand_pf/operator_status", "/baseline/lattice_selected", "/baseline/confirmed_obstacles")
            report["optional_graph"] = topology(node, optional)
            report["map_services"] = {name: types for name, types in node.get_service_names_and_types()
                                      if name == "/map_server/map"}
            report.update(status="OBSERVED", ready=None)
            return report, 0
        sample_errors, summary = samples.report(sample_ros, sample_mono)
        issues = list(loss)
        if window is not None and not window.complete:
            issues.append("continuous matched zero capture interval incomplete")
        if not bound or identity["status"] != "VERIFIED":
            issues.append("commissioned supervisor unverified before or at end of observation: " + identity["status"])
        issues.extend(graph_issues(topics, graph, mode) + sample_errors)
        if captured == 0:
            issues.append("no synchronized rows accepted with verified session and graph")
        ready = not issues
        report.update(ready=ready, status="OBSERVED_MATCH" if ready else "NOT_READY",
                      issues=sorted(set(issues)), samples=summary)
        return report, 0 if ready else 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("inventory", "zero", "run"), default="inventory")
    parser.add_argument("--seconds", type=float, default=3., help="DDS discovery and sampling window")
    parser.add_argument("--stream-snapshots", action="store_true",
                        help="emit matched JSONL on stdout and send the summary report to stderr")
    parser.add_argument("--snapshots", type=Path, help="write synchronized JSONL to a new path after a match")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    if (args.snapshots is not None or args.stream_snapshots) and args.mode == "inventory":
        parser.error("capture requires --mode zero or run")
    tmp = None
    try:
        if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "85":
            raise RuntimeError("v8 physical audit requires ROS_DOMAIN_ID=85 and ROS_LOCALHOST_ONLY=1")
        if args.snapshots is not None:
            if args.snapshots.exists():
                raise FileExistsError(str(args.snapshots))
            tmp = tempfile.NamedTemporaryFile(mode="w", dir=args.snapshots.parent, delete=False)
        report, code = observe(args, tmp)
        if tmp is not None and code == 0:
            tmp.close()
            os.link(tmp.name, args.snapshots)  # atomic create; concurrent observers cannot overwrite
            report["snapshots"] = str(args.snapshots)
        print(json.dumps(report, indent=2, allow_nan=False), file=sys.stderr if args.stream_snapshots else sys.stdout)
        return code
    except (ImportError, RuntimeError, OSError, NotImplementedError, AttributeError) as exc:
        print(json.dumps({"status": "UNVERIFIED", "ready": None, "error": str(exc)}),
              file=sys.stderr if args.stream_snapshots else sys.stdout)
        return 2
    finally:
        if tmp is not None:
            tmp.close()
            Path(tmp.name).unlink()


if __name__ == "__main__":
    sys.exit(main())
