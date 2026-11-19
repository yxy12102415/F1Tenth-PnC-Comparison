#!/usr/bin/env python3
"""DDS loopback of four synthetic inputs on localhost domain 141, in a test namespace."""
import argparse
import json
import math
import os
import time

from contract import ODOM_TOPIC, POSE_TOPIC, SCAN_TOPIC, VALID_TOPIC, SENSORS, Samples, compatible
from preflight import endpoint, subscribe


def transport(rows):
    if os.environ.get("ROS_DOMAIN_ID") != "141" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("transport fixture requires localhost-only ROS domain 141")
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool

    prefix = "/pnc_transport_{}/".format(os.getpid())
    aliases = {t: prefix + t.rsplit("/", 1)[1] for t in SENSORS}
    remaps = ["--ros-args"]
    for topic, alias in aliases.items():
        remaps.extend(["-r", topic + ":=" + alias])
    rclpy.init(args=[])
    writer = Node("pnc_synthetic_writer_" + str(os.getpid()), start_parameter_services=False)
    observer = Node("pnc_synthetic_reader_" + str(os.getpid()), cli_args=remaps, start_parameter_services=False)
    snapshots = []
    try:
        topics = {POSE_TOPIC: Odometry, ODOM_TOPIC: Odometry, SCAN_TOPIC: LaserScan, VALID_TOPIC: Bool}
        publishers = {t: writer.create_publisher(cls, aliases[t], qos_profile_sensor_data if t == SCAN_TOPIC else 1)
                      for t, cls in topics.items()}
        s = Samples("zero")
        s.topics = {t: s.topics[t] for t in SENSORS}
        subscribe(observer, s, snapshots.append)
        end = time.monotonic() + 5.
        while not all(p.get_subscription_count() == 1 for p in publishers.values()):
            if time.monotonic() >= end:
                raise RuntimeError("DDS discovery timed out")
            rclpy.spin_once(observer, timeout_sec=.01)
        # Inspect actual negotiated graph descriptors from DDS, excluding rosout/parameter endpoints.
        for topic, alias in aliases.items():
            writers = observer.get_publishers_info_by_topic(alias)
            readers = observer.get_subscriptions_info_by_topic(alias)
            if (len(writers) != 1 or len(readers) != 1 or writers[0].topic_type != s.topics[topic].type
                    or readers[0].topic_type != s.topics[topic].type
                    or not compatible(endpoint(writers[0]), endpoint(readers[0]))):
                raise RuntimeError("wire type/graph QoS did not match for " + alias)
        base_ns = writer.get_clock().now().nanoseconds
        for index, row in enumerate(rows):
            # Rebase the synthetic 20 Hz clock for the physical source-age checks.
            # Each fixture pose is the scan-time base pose; the fixture has synchronous input stamps.
            tick_ns = base_ns + index * 50_000_000
            while writer.get_clock().now().nanoseconds < tick_ns:
                rclpy.spin_once(observer, timeout_sec=.001)
            pose, odom, scan = Odometry(), Odometry(), LaserScan()
            for msg, frame in ((pose, "map"), (odom, "odom"), (scan, "laser")):
                msg.header.frame_id = frame
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(tick_ns, 10**9)
            pose.child_frame_id = odom.child_frame_id = "base_link"
            pose.pose.pose.position.x = row["x"]
            pose.pose.pose.position.y = row["y"]
            pose.pose.pose.orientation.z = math.sin(row["yaw"] / 2)
            pose.pose.pose.orientation.w = math.cos(row["yaw"] / 2)
            odom.pose.pose.orientation.w = 1.
            odom.twist.twist.linear.x = row["v"]
            odom.twist.twist.linear.y = row["vy"]
            odom.twist.twist.angular.z = row["yaw_rate"]
            scan.angle_min, scan.angle_increment = row["angle_min"], row["angle_increment"]
            scan.angle_max = scan.angle_min + (len(row["ranges"]) - 1) * scan.angle_increment
            scan.range_min, scan.range_max = row["range_min"], row["range_max"]
            scan.ranges = [v if v is not None else math.inf for v in row["ranges"]]
            for t, msg in ((POSE_TOPIC, pose), (ODOM_TOPIC, odom), (VALID_TOPIC, Bool(data=True)), (SCAN_TOPIC, scan)):
                publishers[t].publish(msg)
            end = time.monotonic() + .45
            while len(snapshots) <= index:
                if time.monotonic() >= end:
                    raise RuntimeError("no synchronized snapshot for fixture frame {}".format(index))
                rclpy.spin_once(observer, timeout_sec=.005)
            saved = snapshots[-1]
            if (abs(saved["scan_stamp"] - tick_ns * 1e-9) > 1e-6 or len(saved["ranges"]) != len(row["ranges"])
                    or any((a is None) != (b is None) or (a is not None and abs(a-b) > 1.e-5)
                           for a, b in zip(row["ranges"], saved["ranges"]))):
                raise RuntimeError("scan wire round-trip differs at frame {}".format(index))
        return snapshots
    finally:
        observer.destroy_node()
        writer.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", help="synchronous 20 Hz synthetic map-ray fixture in pnc.vehicle.v1 format")
    parser.add_argument("--output", help="new file to receive re-stamped serialized input")
    args = parser.parse_args()
    if args.input:
        with open(args.input) as stream:
            rows = [json.loads(line) for line in stream]
    else:
        row = dict(x=1., y=-2., yaw=.4, v=.2, vy=.01, yaw_rate=-.1, ranges=[.75, None, 1.33],
                   angle_min=-.02, angle_increment=.02, range_min=.05, range_max=12.)
        rows = [row] * 8
    if not rows:
        parser.error("empty fixture")
    snapshots = transport(rows)
    if args.output:
        with open(args.output, "x") as output:
            for row in snapshots:
                output.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
    print(json.dumps({"evidence": "synthetic localhost DDS", "domain": 141, "frames": len(snapshots),
                      "rays_per_scan": len(snapshots[0]["ranges"]), "ros_distro": os.environ.get("ROS_DISTRO")}))


if __name__ == "__main__":
    main()
