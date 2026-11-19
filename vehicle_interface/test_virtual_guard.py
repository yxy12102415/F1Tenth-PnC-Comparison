#!/usr/bin/env python3
"""Test the installed Foxy final guard on private DDS, then its downstream numerical interfaces."""
import json
import math
import os
from pathlib import Path
import sys
import time


def check(condition, label):
    if not condition:
        raise RuntimeError(label)


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "142" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("test requires localhost-only virtual domain 142")
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from f1tenth_safety_migration import final_command_guard
    from f1tenth_safety_migration.safety_core import HardwareActuatorCore, HardwareActuatorConfig

    # Compare installed implementation to the exact version imported by the commissioned supervisor.
    sources = Path(__file__).resolve().parent / "commissioned_guard_sources"
    for name in ("final_command_guard", "safety_core"):
        local = sources / (name + ".py")
        installed = Path(final_command_guard.__file__).resolve().parent / local.name
        check(local.read_bytes() == installed.read_bytes(), "guard overlay mismatch: " + name)

    prefix = "/pnc_guard_fixture_" + str(os.getpid())
    fields = {"nominal": AckermannDriveStamped, "odom": Odometry, "scan": LaserScan,
              "planner_valid": Bool, "deadman": Bool, "arm": Bool}
    names = list(fields) + ["final", "status", "target_belief_age", "reset"]
    init_args = ["--ros-args", "--log-level", "ERROR"]
    for key in names:
        init_args.extend(["-p", key + "_topic:=" + prefix + "/" + key])
    params = {"nominal_timeout": ".50", "odom_timeout": ".50", "scan_timeout": ".50",
              "deadman_timeout": ".50", "planner_timeout": ".50", "max_speed": "2.0",
              "max_steering_angle": ".27", "require_target_belief": "false"}
    for k, v in params.items():
        init_args.extend(["-p", k + ":=" + v])
    rclpy.init(args=init_args)
    guard = final_command_guard.FinalCommandGuard()
    peer = Node("pnc_test_io_" + str(os.getpid()), use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(guard)
    executor.add_node(peer)
    pubs = {k: peer.create_publisher(t, prefix + "/" + k, 10) for k, t in fields.items()}
    final = []
    status = []
    peer.create_subscription(AckermannDriveStamped, prefix + "/final", final.append, 10)
    peer.create_subscription(String, prefix + "/status", lambda m: status.append(json.loads(m.data)), 10)

    def exercise(arm, nominal, duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            now = peer.get_clock().now().to_msg()
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = now, "laser"
            scan.range_min, scan.range_max, scan.angle_increment = .1, 10., .01
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now, "odom", "base_link"
            odom.pose.pose.orientation.w = 1.
            pubs["scan"].publish(scan)
            pubs["odom"].publish(odom)
            pubs["planner_valid"].publish(Bool(data=True))
            pubs["deadman"].publish(Bool(data=True))
            if nominal:
                msg = AckermannDriveStamped()
                msg.header.stamp, msg.header.frame_id = now, "base_link"
                msg.drive.speed, msg.drive.steering_angle = .9, .1
                pubs["nominal"].publish(msg)
            pubs["arm"].publish(Bool(data=arm))
            next_tick = time.monotonic() + .02
            while time.monotonic() < next_tick:
                executor.spin_once(timeout_sec=.002)

    try:
        exercise(False, True, .8)
        check(status and status[-1]["arm_state"] == "DISARMED" and final[-1].drive.speed == 0., "disarmed zero")
        exercise(True, True, .3)
        check(status[-1]["arm_state"] == "ARMED" and final[-1].header.frame_id == "base_link", "armed handshake")
        accepted = final[-1].drive
        check(math.isclose(accepted.speed, .9, abs_tol=1e-5) and
              math.isclose(accepted.steering_angle, .1, abs_tol=1e-5), "nominal wire units")
        exercise(True, False, .70)
        check(status[-1]["stop_reason"] == "nominal_timeout" and status[-1]["latched"] and
              final[-1].drive.speed == 0. and final[-1].drive.steering_angle == 0., "missing nominal did not stop")
        exercise(True, True, .15)
        check(status[-1]["latched"] and final[-1].drive.speed == 0., "latch reset unexpectedly")
    finally:
        executor.shutdown()
        peer.destroy_node()
        guard.destroy_node()
        rclpy.shutdown()

    release = "ground_fitzpatrick_joint_20260927_v8"
    sys.path.insert(0, str(sources.parents[1] / "real_vehicle/candidate_cpp_gpu_20260909" / release))
    from stand_window import StandWindow
    window = StandWindow()
    actuator = HardwareActuatorCore(HardwareActuatorConfig(max_abs_speed=2., max_abs_rpm=15000.,
                                                          final_command_timeout=.50))
    window.joy(False, 0.)
    for i in range(60):
        t = i * .02
        window.health(True, t)
        window.permit(True, t)
        window.joy(True, t)
        window.command(accepted.speed, accepted.steering_angle, t)
        speed, steering, reason = window.evaluate(t)
        check(not reason, "window: " + reason)
        actuator.update(speed, steering, t)
    out = actuator.evaluate(t)
    check(out.motor_mode == "speed" and math.isclose(out.rpm, 6750., abs_tol=.01)
          and math.isclose(out.servo, .4304-.12135, abs_tol=.00001), "physical unit conversion")
    expired = actuator.evaluate(t+.51)
    check(expired.motor_mode == "coast" and expired.current == 0. and expired.stop_reason == "final_command_timeout",
          "downstream freshness")
    window.joy(False, t+.02)
    stopped = window.evaluate(t+.02)
    check(stopped == (0., 0., "deadman_release"), "L1 release")
    print(json.dumps({"domain": 142, "localhost_only": True, "passed": True, "actuation": False,
                      "ROS_final_guard": ["disarmed_zero", "armed_nominal", "missing_nominal_latched", "stays_latched"],
                      "window_and_actuator_core": ["startup_slew", "L1_release", "stale_final_zero_current"],
                      "rpm_for_0_9_mps": out.rpm, "servo_for_0_1_rad": out.servo}, indent=2))


if __name__ == "__main__":
    main()
