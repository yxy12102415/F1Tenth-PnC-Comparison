#!/usr/bin/env python3
"""Publish one physical-profile algorithm's nominal messages on isolated localhost DDS."""
import argparse
import contextlib
import json
import math
import os
import select
from pathlib import Path
import socket
import subprocess
import sys
import time

DOMAIN = "143"
PREFIX = "/pnc_comparison/staging/baseline"
TOPICS = tuple(PREFIX + "/" + name for name in ("drive_nominal", "planner_valid", "controller_status"))
METHODS = ("pp", "lqr", "linear_mpc", "nmpc", "mppi", "hybrid_astar", "lattice", "rrt_star", "dwa", "teb")
PERIOD = .05
MAX_SILENCE = .10
STAMP_FIELDS = ("stamp", "pose_stamp", "odom_stamp")


def number(row, key):
    value = row.get(key)
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("nonfinite or missing " + key)
    return value


def check_stamps(row, now):
    for k in STAMP_FIELDS:
        stamp = number(row, k)
        if stamp <= 0 or not -.020 <= now - stamp <= .505:
            raise ValueError("stale or future " + k)


class Candidate:
    def __init__(self, method):
        self.method = method
        self.row = None
        self.received = None
        self.pending = False
        self.failure = ""

    def fail(self, reason):
        if not self.failure:
            self.failure = reason

    def accept(self, row, ros_now, monotonic_now):
        if self.failure:
            return
        try:
            if not isinstance(row, dict) or row.get("schema") != "pnc.vehicle.shadow.v1":
                raise ValueError("unknown diagnostic schema")
            if row.get("method") != self.method or row.get("source_mode") != "live_stream":
                raise ValueError("unexpected diagnostic method or source mode")
            if row.get("valid") is not True or row.get("reason") != "shadow" or row.get("actuation_authorized") is not False:
                detail = row.get("reason")
                raise ValueError("invalid shadow result: " + (detail[:160] if isinstance(detail, str) else "missing reason"))
            check_stamps(row, ros_now)
            if type(row.get("input_line")) is not int or row["input_line"] < 1:
                raise ValueError("invalid input sequence")
            if self.row is not None:
                if monotonic_now - self.received > MAX_SILENCE:
                    raise ValueError("diagnostic producer timeout")
                if (row["input_line"] <= self.row["input_line"] or row["stamp"] <= self.row["stamp"]
                        or any(row[k] < self.row[k] for k in ("pose_stamp", "odom_stamp"))):
                    raise ValueError("duplicate or out-of-order source")
            if not 0 <= number(row, "compute_ms") <= 50:
                raise ValueError("algorithm missed its compute budget")
            if number(row, "min_preview_gap_m") < .065:
                raise ValueError("preview clearance")
            if not 0 <= number(row, "speed") <= 2 or abs(number(row, "steering")) > .269:
                raise ValueError("outside v8 nominal command limits")
            self.row, self.received, self.pending = row, monotonic_now, True
        except (ValueError, OverflowError) as exc:
            self.fail(str(exc))

    def next_output(self, ros_now, monotonic_now):
        if self.row is not None and not self.failure:
            if monotonic_now - self.received > MAX_SILENCE:
                self.fail("diagnostic producer timeout")
            try:
                check_stamps(self.row, ros_now)
            except ValueError as exc:
                self.fail(str(exc))
        if self.failure or self.row is None:
            return 0., 0., False, self.failure or "waiting_for_input"
        if not self.pending:
            return None
        self.pending = False
        return self.row["speed"], self.row["steering"], True, "shadow"


class WorkerPipe:
    """The adapter creates the child's stdout pipe; no ROS writer supplies commands."""
    def __init__(self, stream):
        self.fd = stream.fileno()
        os.set_blocking(self.fd, False)
        self.buffer = b""

    def read(self, candidate, now, monotonic_now):
        closed = False
        try:
            while True:
                try:
                    data = os.read(self.fd, 65536)
                except BlockingIOError:
                    break
                if not data:
                    closed = True
                    break
                self.buffer += data
                if len(self.buffer) > 65536:
                    raise ValueError("diagnostic input backlog")
            complete = self.buffer.split(b"\n")
            self.buffer = complete.pop()
            for line in complete:
                candidate.accept(json.loads(line), now, monotonic_now)
            if closed:
                candidate.fail("diagnostic producer closed")
        except (OSError, ValueError, RecursionError) as exc:
            candidate.fail("invalid diagnostic stream: " + str(exc))


def emit(node, publishers, candidate, speed, steering, valid, reason):
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import Bool, String
    drive = AckermannDriveStamped()
    drive.header.stamp = node.get_clock().now().to_msg()
    drive.header.frame_id = "base_link"
    drive.drive.speed, drive.drive.steering_angle = float(speed), float(steering)
    status = dict(controller=candidate.method, implementation="isolated_shadow_stage",
                  valid=valid, reason=reason, speed_command=speed, steering_command=steering,
                  actuation_authorized=False)
    if valid:
        status.update({k: candidate.row[k] for k in (*STAMP_FIELDS, "input_line", "compute_ms")})
    publishers[0].publish(drive)
    publishers[1].publish(Bool(data=valid))
    publishers[2].publish(String(data=json.dumps(status, allow_nan=False)))


def serve(method):
    if os.environ.get("ROS_DOMAIN_ID") != DOMAIN or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise ValueError("requires ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID=" + DOMAIN)
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    with contextlib.ExitStack() as cleanup:
        owner = cleanup.enter_context(socket.socket(socket.AF_UNIX, socket.SOCK_STREAM))
        owner.bind("\0pnc.nominal_stage." + DOMAIN)
        rclpy.init(args=[])
        cleanup.callback(lambda: rclpy.shutdown() if rclpy.ok() else None)
        node = Node("pnc_nominal_stage", namespace="/pnc_comparison/staging",
                    start_parameter_services=False, use_global_arguments=False)
        cleanup.callback(node.destroy_node)
        discovery = time.monotonic() + .7
        while time.monotonic() < discovery:
            rclpy.spin_once(node, timeout_sec=.025)
        if any(node.count_publishers(topic) for topic in TOPICS):
            raise ValueError("staging nominal topics already have a publisher")
        pubs = [node.create_publisher(msg_type, topic, 1)
                for msg_type, topic in zip((AckermannDriveStamped, Bool, String), TOPICS)]
        candidate = Candidate(method)
        read_fd, write_fd = os.pipe()
        cleanup.callback(os.close, read_fd)
        try:
            worker = subprocess.Popen([sys.executable, str(Path(__file__).with_name("shadow.py")),
                                       "--input", "-", "--output", "-", "--methods", method,
                                       "--ready-fd", str(write_fd)], stdin=sys.stdin, stdout=subprocess.PIPE,
                                      pass_fds=(write_fd,))
        finally:
            os.close(write_fd)
        ready = False
        reader = WorkerPipe(worker.stdout)
        last_publish, last_reason = float("-inf"), None
        try:
            while rclpy.ok():
                if any(node.count_publishers(topic) != 1 for topic in TOPICS):
                    emit(node, pubs, candidate, 0., 0., False, "staging publisher conflict")
                    raise ValueError("multiple staging publishers or publisher discovery loss")
                if not ready and not candidate.failure and select.select([read_fd], [], [], 0)[0]:
                    if os.read(read_fd, 1) == b"R":
                        ready = True
                    else:
                        candidate.fail("diagnostic initialization failed")
                if not candidate.failure:
                    reader.read(candidate, node.get_clock().now().nanoseconds * 1e-9, time.monotonic())
                if worker.poll() is not None:
                    candidate.fail("diagnostic producer exited")
                decision = candidate.next_output(node.get_clock().now().nanoseconds * 1e-9, time.monotonic())
                if not ready and not candidate.failure:
                    decision = (0., 0., False, "initializing_algorithm")
                if decision is not None:
                    valid, reason = decision[2:]
                    now = time.monotonic()
                    if valid or reason != last_reason or now-last_publish >= PERIOD:
                        emit(node, pubs, candidate, *decision)
                        last_publish, last_reason = now, reason
                rclpy.spin_once(node, timeout_sec=.005)
        finally:
            if rclpy.ok():
                emit(node, pubs, candidate, 0., 0., False, "staging stopped")
            if worker.poll() is None:
                worker.terminate()
                try:
                    worker.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    worker.kill()
                    worker.wait()
            worker.stdout.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--method", choices=METHODS, required=True)
    args = parser.parse_args()
    try:
        serve(args.method)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError) as exc:
        parser.exit(2, str(exc) + "\n")


if __name__ == "__main__":
    main()
