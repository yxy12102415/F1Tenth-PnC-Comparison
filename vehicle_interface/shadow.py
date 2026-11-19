#!/usr/bin/env python3
"""Read a captured physical-interface JSONL stream and write offline diagnostic candidates."""
import argparse
import contextlib
import gzip
import json
import math
import os
import select
import stat
import sys
import time
from pathlib import Path

import numpy as np
from algorithm_shadow import ASSETS, DT, METHODS, Algorithm, Native, Route, Scene, validate_snapshot


def live_lines(ready_fd=None):
    """Bound pipe silence and partial input on an actual system-clock ROS stream."""
    pending = bytearray()
    stream = sys.stdin.fileno()
    skip_fragment = False
    if ready_fd is not None:
        # This generator first runs after Route, Scene, algorithms, and the native
        # model handshake are ready. Discard queued startup bytes, never re-date a row.
        drained = 0
        while select.select([stream], [], [], 0)[0]:
            data = os.read(stream, 65536)
            if not data:
                raise ValueError("live snapshot producer closed during initialization")
            drained += len(data)
            if drained > 2_000_000:
                raise ValueError("excess queued input during initialization")
            skip_fragment = not data.endswith(b"\n")
        os.write(ready_fd, b"R")
        os.close(ready_fd)
    deadline = time.monotonic() + 30.0  # graph discovery before the first accepted sample
    while True:
        wait = max(0., deadline-time.monotonic())
        if not select.select([stream], [], [], wait)[0]:
            raise ValueError("live snapshot stream timeout")
        data = os.read(stream, 65536)
        if not data:
            if pending:
                raise ValueError("incomplete live JSONL row")
            return
        if skip_fragment:
            _, marker, data = data.partition(b"\n")
            if not marker:
                continue
            skip_fragment = False
        pending.extend(data)
        while b"\n" in pending:
            line, _, rest = pending.partition(b"\n")
            pending = bytearray(rest)
            deadline = time.monotonic()+.50
            yield line
        if len(pending) > 1_000_000:
            raise ValueError("live snapshot row too large")


def live_fresh(row):
    now = time.time()
    for key in ("scan_stamp", "stamp", "odom_stamp"):
        age = now-row[key]
        if not -.02 <= age <= .505:
            raise ValueError("live {} stale or in the future".format(key))


def replay(source, output, methods, seed=11, start_s=0., live=False):
    route = Route()
    if not 0. <= start_s <= route.length:
        raise ValueError("initial route station outside lap")
    scene = Scene(json.loads(ASSETS.read_text()))
    algorithms = [Algorithm(m, route, scene, seed) for m in methods]
    station = start_s
    previous = None
    last_control = None
    failed = False
    count = 0
    with contextlib.closing(Native(seed)) as native:
        for line_number, line in enumerate(source, 1):
            try:
                row = json.loads(line)
                validate_snapshot(row, previous)
                if live:
                    live_fresh(row)
                if previous is None and row["v"] > .10:
                    raise ValueError("replay needs a stationary first snapshot for actuator initialization")
                previous = row
                stamp = row["scan_stamp"]
                begin = time.perf_counter()
                observed = native.call(dict(row, method="perception"))
                if not observed["raw_scan_point_count"]:
                    raise ValueError("v8 scan has no usable returns within 7 m")
                if last_control is not None:
                    period = stamp-last_control
                    if period < .040-1.e-6:
                        continue
                    if period > .065+1.e-6:
                        raise ValueError("input cadence cannot supply a 20 Hz control tick")
                last_control = stamp
                discs = np.asarray(observed["discs"]).reshape(-1, 3)
                station, _ = route.project(row, station)
                point = route.at_s(station)
                if (math.hypot(row["x"]-point[0], row["y"]-point[1]) > .85 or
                        abs(math.remainder(row["yaw"]-point[2], 2*math.pi)) > .70):
                    raise ValueError("pose outside supplied lap station/heading window")
                overhead = (time.perf_counter()-begin)*1000
            except (KeyError, ValueError, TypeError, OSError) as exc:
                for method in methods:
                    write(output, dict(method=method, stamp=None, input_line=line_number, valid=False,
                                       reason=str(exc), speed=None, steering=None, actuation_authorized=False), live)
                return 2
            for algorithm in algorithms:
                result = dict(method=algorithm.name, stamp=stamp, pose_stamp=row["stamp"], odom_stamp=row["odom_stamp"],
                              input_line=line_number, station_m=station, confirmed_discs=len(discs),
                              planner_invoked=False, actuation_authorized=False, speed=None, steering=None,
                              valid=False, reason=algorithm.failure, compute_ms=None)
                if not algorithm.failure:
                    begin = time.perf_counter()
                    try:
                        speed, steering = algorithm.step(row, station, discs, native, stamp, result)
                        result["compute_ms"] = overhead + (time.perf_counter()-begin)*1000
                        if result["compute_ms"] > DT*1000:
                            raise ValueError("pipeline exceeded 50 ms")
                        if live:
                            live_fresh(row)
                        result.update(speed=speed, steering=steering, valid=True, reason="shadow")
                    except (ValueError, RuntimeError, OSError, ArithmeticError) as exc:
                        result.update(reason=str(exc), compute_ms=overhead+(time.perf_counter()-begin)*1000)
                        # The candidate may have mutated solver/observer state. Require a
                        # new stationary replay rather than advancing an unaccepted state.
                        algorithm.failure = "latched after " + result["reason"]
                        failed = True
                write(output, result, live)
                count += 1
    if not count:
        raise ValueError("capture has no control samples")
    return int(failed)


def write(stream, item, live):
    item.update(schema="pnc.vehicle.shadow.v1", source_mode="live_stream" if live else "capture_replay")
    print(json.dumps(item, allow_nan=False, separators=(",", ":")), file=stream, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="JSONL or .jsonl.gz capture file, or - for live system-clock JSONL stdin")
    parser.add_argument("--output", required=True, help="new diagnostics JSONL file, or - for stdout")
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=METHODS)
    parser.add_argument("--ready-fd", type=int, help=argparse.SUPPRESS)
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--start-s", type=float, default=0., help="known unwrapped starting station (metres)")
    args = parser.parse_args()
    if len(set(args.methods)) != len(args.methods) or not 0 <= args.seed < 2**32:
        parser.error("methods must be unique and seed an unsigned 32-bit integer")
    try:
        if not ASSETS.is_file():
            parser.error("run vehicle_interface/algorithm_build.sh first")
        live = args.input == "-"
        if args.ready_fd is not None and (not live or args.ready_fd < 3 or
                                        not stat.S_ISFIFO(os.fstat(args.ready_fd).st_mode)):
            parser.error("--ready-fd requires live input and an inherited readiness pipe")
        with contextlib.ExitStack() as stack:
            if live:
                src = live_lines(args.ready_fd)
            elif args.input.endswith(".gz"):
                src = stack.enter_context(gzip.open(args.input, "rt"))
            else:
                src = stack.enter_context(Path(args.input).open())
            dst = sys.stdout if args.output == "-" else stack.enter_context(Path(args.output).open("x"))
            try:
                return replay(src, dst, args.methods, args.seed, args.start_s, live=live)
            except (OSError, ValueError) as exc:
                for method in args.methods:
                    write(dst, dict(method=method, stamp=None, valid=False, reason=str(exc),
                                    speed=None, steering=None, actuation_authorized=False), live)
                raise
    except (OSError, RuntimeError, ValueError) as exc:
        parser.exit(2, str(exc)+"\n")


if __name__ == "__main__":
    sys.exit(main())
