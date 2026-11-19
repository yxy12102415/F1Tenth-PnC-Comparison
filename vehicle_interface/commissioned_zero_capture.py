#!/usr/bin/env python3
"""Wait for an operator's commissioned v8 ZERO session and record matched sensor data."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

from preflight import observe
from session_identity import SessionIdentity


class CaptureWindow:
    """Store a source-clock window beginning with the first fully qualified scan."""
    def __init__(self, seconds, deadline):
        self.seconds = seconds
        self.deadline = deadline
        self.first = None
        self.last = None
        self.complete = False

    def include(self, stamp):
        if not math.isfinite(stamp) or (self.last is not None and not 0 < stamp-self.last <= .100001):
            raise ValueError("missing or nonincreasing scan during matched zero window")
        if self.first is None:
            self.first = stamp
        if stamp-self.first >= self.seconds-1e-6:
            self.complete = True
        if stamp-self.first > self.seconds+1e-6:
            return False
        self.last = stamp
        return True


def capture(destination, seconds, timeout, verifier, observer=observe,
            clock=time.monotonic, pause=time.sleep):
    if os.path.lexists(destination):
        raise FileExistsError("capture already exists: " + str(destination))
    if not destination.parent.is_dir():
        raise FileNotFoundError("capture directory does not exist: " + str(destination.parent))
    if not 1. <= seconds <= 2. or not math.isfinite(timeout) or timeout <= seconds:
        raise ValueError("choose --seconds between 1 and 2 and a finite --timeout longer than the capture")
    deadline = clock()+timeout
    previous_status = None
    last_report = None
    while clock() < deadline:
        identity = verifier.check("zero")
        status = identity["status"]
        if status != previous_status:
            print("v8 zero session: " + status, file=sys.stderr, flush=True)
            previous_status = status
        if status == "VERIFIED" and clock()+seconds <= deadline:
            window = CaptureWindow(seconds, deadline)
            args = argparse.Namespace(mode="zero", stream_snapshots=False, seconds=deadline-clock())
            # Temporary and final files share a filesystem. The path becomes visible
            # only after all graph, identity, sample, zero and final checks pass.
            with tempfile.NamedTemporaryFile(mode="w", dir=destination.parent) as tmp:
                last_report, code = observer(args, tmp, window=window)
                if (code == 0 and last_report.get("status") == "OBSERVED_MATCH"
                        and last_report.get("ready") is True and window.complete
                        and last_report.get("captured_rows", 0) > 0):
                    tmp.flush()
                    os.link(tmp.name, destination)
                    last_report.update(snapshots=str(destination), requested_capture_s=seconds,
                                       first_scan_stamp=window.first, last_scan_stamp=window.last,
                                       captured_scan_span_s=window.last-window.first)
                    return last_report, 0
                # A failed check after the first matched row invalidates that window.
                # Keep its evidence instead of clearing a supervisor/health fault.
                if window.first is not None or code != 1:
                    return last_report, code if code else 1
        pause(max(0., min(.25, deadline-clock())))
    report = {"status": "NOT_READY", "ready": False, "motion_authorized": False,
              "reason": "timeout waiting for a complete verified v8 zero interval",
              "requested_capture_s": seconds, "timeout_s": timeout, "last_identity_status": previous_status}
    if last_report:
        report["issues"] = last_report.get("issues", [])
    return report, 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path, help="new capture JSONL path")
    parser.add_argument("--seconds", type=float, default=1.5, help="matched recording window, 1–2 seconds")
    parser.add_argument("--timeout", type=float, default=120., help="total startup and recording time limit")
    args = parser.parse_args()
    try:
        if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "85":
            raise ValueError("use the commissioned localhost-only domain 85 environment")
        report, code = capture(args.destination, args.seconds, args.timeout, SessionIdentity())
        print(json.dumps(report, indent=2, allow_nan=False))
        return code
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(json.dumps({"status": "UNVERIFIED", "ready": None, "error": str(exc), "motion_authorized": False}))
        return 2


if __name__ == "__main__":
    sys.exit(main())
