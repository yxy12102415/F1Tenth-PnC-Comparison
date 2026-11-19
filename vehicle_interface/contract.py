"""Fitzpatrick v8 physical topology and sample contracts. No ROS dependency."""
import json
import math
from collections import deque
from dataclasses import dataclass

ODOM = "nav_msgs/msg/Odometry"
SCAN = "sensor_msgs/msg/LaserScan"
JOY = "sensor_msgs/msg/Joy"
BOOL = "std_msgs/msg/Bool"
TEXT = "std_msgs/msg/String"
SCALAR = "std_msgs/msg/Float64"
DRIVE = "ackermann_msgs/msg/AckermannDriveStamped"
PLANNER = "/frenet_cpp_planner"
CONTROL = "/frenet_cpp_controller"
FINAL = "/final_command_guard"
ACTUATOR = "/hardware_actuator_guard"
OPERATOR = "/route_operator_gate"
WINDOW = "/stand_window"
VESC = "/vesc_driver"
PF = "/particle_filter"
HEALTH = "/ground_health_interlock"
SUPERVISOR = "/stand_pf_supervisor"
ODOM_NODE = "/stand_vesc_odometry"

POSE_TOPIC = "/baseline/localization_pose"
ODOM_TOPIC = "/baseline/odom"
SCAN_TOPIC = "/baseline/scan"
VALID_TOPIC = "/baseline/localization_valid"
SENSORS = (POSE_TOPIC, ODOM_TOPIC, SCAN_TOPIC, VALID_TOPIC)


@dataclass(frozen=True)
class Channel:
    type: str
    owner: str  # '' explicitly forbids writers. '*' accepts one sensor driver.
    consumers: tuple = ()
    sampled: bool = False  # a recent sample is expected when the graph is settled


def channels(mode):
    run = mode == "run"
    gate = OPERATOR if run else ""
    nominal_readers = (FINAL, OPERATOR) if run else (FINAL,)
    nodes = (PLANNER, CONTROL)
    topics = {
        "/baseline/raw_scan": Channel(SCAN, "*", ("/stand_scan_relay",)),
        SCAN_TOPIC: Channel(SCAN, "/stand_scan_relay", nodes + (PF, FINAL), True),
        POSE_TOPIC: Channel(ODOM, PF, nodes, True),
        ODOM_TOPIC: Channel(ODOM, ODOM_NODE, nodes + (PF, FINAL), True),
        VALID_TOPIC: Channel(BOOL, PF, nodes, True),
        "/baseline/local_plan_cpp": Channel(TEXT, PLANNER, (CONTROL,)),
        "/baseline/drive_nominal": Channel(DRIVE, CONTROL, nominal_readers, True),
        "/baseline/planner_valid": Channel(BOOL, CONTROL, nominal_readers, True),
        "/baseline/controller_status": Channel(TEXT, CONTROL, (SUPERVISOR,), True),
        "/baseline/arm": Channel(BOOL, gate, (FINAL,)),
        "/baseline/deadman": Channel(BOOL, gate, (FINAL,), run),
        "/baseline/enable": Channel(BOOL, gate, nodes, run),
        "/stage1_offline/reset_latch": Channel(BOOL, "", (FINAL,)),
        "/baseline/final_status": Channel(TEXT, FINAL, (SUPERVISOR,), True),
        "/baseline/final": Channel(DRIVE, FINAL, (WINDOW,), True),
        "/stand_pf/permit": Channel(BOOL, SUPERVISOR, (WINDOW,)),
        "/stand_pf/health": Channel(TEXT, "/stand_telemetry_bridge", (HEALTH,), True),
        "/stand_pf/health_permit": Channel(BOOL, HEALTH, (WINDOW,), True),
        "/stand_pf/health_guard_status": Channel(TEXT, HEALTH, (SUPERVISOR,), True),
        "/stand_pf/actuator_status": Channel(TEXT, ACTUATOR, (SUPERVISOR,), True),
        "/stand_pf/window_status": Channel(TEXT, WINDOW, (SUPERVISOR,), True),
        "/stand_pf/actuator_input": Channel(DRIVE, WINDOW, (ACTUATOR,), True),
        "/stand_pf/motor_current": Channel(SCALAR, ACTUATOR, (VESC,)),
        "/stand_pf/motor_speed": Channel(SCALAR, ACTUATOR, (VESC,)),
        "/stand_pf/servo": Channel(SCALAR, ACTUATOR, (VESC,) if run else (), True),
        "/stand_pf/telemetry": Channel("vesc_msgs/msg/VescStateStamped", VESC,
                                      (ODOM_NODE, "/stand_telemetry_bridge")),
        "/stand_pf/servo_feedback": Channel(SCALAR, VESC, (ODOM_NODE,) if run else ()),
    }
    if run:
        topics["/baseline/joy"] = Channel(JOY, "*", (OPERATOR, WINDOW), True)
    for port in ("disabled_brake", "disabled_position", "disabled_duty_cycle", "servo_disabled"):
        driver_reads = port != "servo_disabled" or not run
        topics["/stand_pf/" + port] = Channel(SCALAR, "", (VESC,) if driver_reads else ())
    return topics


def graph_issues(topics, graph, mode):
    """Extra readers are legitimate monitors. Writers must match in number and identity."""
    issues = []
    for topic, spec in topics.items():
        info = graph.get(topic, {"publishers": [], "subscriptions": []})
        writers, readers = info["publishers"], info["subscriptions"]
        owners = [p["node"] for p in writers]
        if len(owners) != int(bool(spec.owner)) or (
            spec.owner and spec.owner != "*" and owners != [spec.owner]
        ):
            issues.append("{} expected writer {}, observed {}".format(topic, spec.owner or "none", owners))
        for endpoint in writers:
            if endpoint["type"] != spec.type:
                issues.append("{} writer type {} expected {}".format(topic, endpoint["type"], spec.type))
        for node in spec.consumers:
            matched = [s for s in readers if s["node"] == node and s["type"] == spec.type]
            if not matched:
                issues.append("{} missing {} reader {}".format(topic, spec.type, node))
                continue
            for writer in writers:
                if not any(compatible(writer, reader) for reader in matched):
                    issues.append("{} writer/{} reader QoS incompatible or unknown".format(topic, node))
        if topic == "/stand_pf/servo" and mode == "zero" and any(s["node"] == VESC for s in readers):
            issues.append("zero mode VESC connected to /stand_pf/servo")
    return issues


def compatible(writer, reader):
    offered, requested = writer["reliability"], reader["reliability"]
    if offered not in ("RELIABLE", "BEST_EFFORT") or requested not in ("RELIABLE", "BEST_EFFORT"):
        return False
    if offered == "BEST_EFFORT" and requested == "RELIABLE":
        return False
    offered, requested = writer["durability"], reader["durability"]
    return (offered in ("VOLATILE", "TRANSIENT_LOCAL") and requested in ("VOLATILE", "TRANSIENT_LOCAL")
            and (requested == "VOLATILE" or offered == "TRANSIENT_LOCAL"))


def planar_yaw(q):
    if (not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w))
            or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > .02
            or abs(q.x) > .01 or abs(q.y) > .01):
        raise ValueError("invalid planar quaternion")
    return 2 * math.atan2(q.z, q.w)


def aligned_pose(history, stamp):
    """Match PoseHistory::at from v8 perception.hpp, including its edge holds."""
    if not history:
        return None
    for index, (right_stamp, right) in enumerate(history):
        if right_stamp < stamp:
            continue
        if index == 0:
            return right if right_stamp - stamp <= .075 else None
        left_stamp, left = history[index - 1]
        span = right_stamp - left_stamp
        if not 0 < span <= .15:
            return None
        fraction = (stamp - left_stamp) / span
        delta = right[2] - left[2]
        return (left[0] + fraction * (right[0] - left[0]), left[1] + fraction * (right[1] - left[1]),
                left[2] + fraction * math.atan2(math.sin(delta), math.cos(delta)))
    last_stamp, last = history[-1]
    return last if stamp - last_stamp <= .075 else None


def decode(topic, msg, kind):
    """Extract only the subscribed message fields consumed by v8 or this audit."""
    if kind == ODOM:
        yaw = planar_yaw(msg.pose.pose.orientation)
        is_pose = topic == POSE_TOPIC
        if msg.header.frame_id != ("map" if is_pose else "odom") or msg.child_frame_id != "base_link":
            raise ValueError("wrong odometry parent/child frame")
        if is_pose:
            pos = msg.pose.pose.position
            values = (pos.x, pos.y, yaw)
        else:
            twist = msg.twist.twist
            values = (abs(twist.linear.x), twist.linear.y, twist.angular.z)
            if abs(values[1]) > 2.5 or abs(values[2]) > 8:
                raise ValueError("odom lateral speed or yaw rate outside v8 envelope")
        if not all(math.isfinite(v) for v in values):
            raise ValueError("nonfinite state")
        return values
    if kind == SCAN:
        meta = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        if (msg.header.frame_id != "laser" or not all(math.isfinite(v) for v in meta)
                or msg.angle_increment <= 0 or not 0 <= msg.range_min < msg.range_max
                or not 1 <= len(msg.ranges) <= 10000):
            raise ValueError("invalid laser frame or scan metadata")
        return msg
    if kind == DRIVE:
        speed, steer = msg.drive.speed, msg.drive.steering_angle
        if topic != "/stand_pf/actuator_input" and msg.header.frame_id != "base_link":
            raise ValueError("wrong drive frame")
        # Source values are float32. Compare against the float32 wire bounds.
        angle_limit = .269 if topic == "/baseline/drive_nominal" else .27
        if not 0 <= speed <= 2 or not abs(steer) <= angle_limit + 2e-8:
            raise ValueError("command exceeds v8 envelope")
        return speed, steer
    if kind == TEXT:
        data = json.loads(msg.data)
        if not isinstance(data, dict):
            raise ValueError("expected JSON object")
        return data
    if kind == JOY:
        if len(msg.buttons) <= 4:
            raise ValueError("missing L1 button")
        return bool(msg.buttons[4])
    if kind == SCALAR and not math.isfinite(msg.data):
        raise ValueError("nonfinite scalar")
    return msg.data


class Samples:
    def __init__(self, mode):
        self.mode = mode
        self.topics = channels(mode)
        self.history = deque(maxlen=80)
        self.latest = {}
        self.last_source = {}
        self.count = {}
        self.invalid = {}
        self.limits = set()
        self.last_scan = None
        self.last_snapshot_receipt = None
        self.snapshots = 0

    def receive(self, topic, msg, ros_now, mono_now):
        spec = self.topics[topic]
        self.count[topic] = self.count.get(topic, 0) + 1
        try:
            value = decode(topic, msg, spec.type)
            source = None
            if spec.type in (SCAN, ODOM, DRIVE):
                source = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if topic == "/stand_pf/health":
                source = value["stamp"]
                fields = [value[k] for k in ("stamp", "erpm", "voltage", "fet_temp", "fault",
                                             "duty_cycle", "motor_current", "input_current")]
                if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                           for v in fields):
                    raise ValueError("incomplete/nonfinite health")
                if (value.get("synthetic", False) is not False or value["fault"] != 0
                        or value["voltage"] < 10.5 or not 0 < value["fet_temp"] < 60
                        or abs(value["erpm"]) > 15750 or abs(value["duty_cycle"]) > 1):
                    raise ValueError("health outside physical v8 interlock")
            if source is not None:
                max_age = .505 if spec.type in (SCAN, ODOM) else .5
                if not -.020 <= ros_now - source <= max_age:
                    raise ValueError("source age outside clock window")
                if topic in SENSORS or topic == "/stand_pf/health":
                    if source <= self.last_source.get(topic, -math.inf):
                        raise ValueError("nonincreasing source stamp")
                    self.last_source[topic] = source
            if topic == POSE_TOPIC:
                self.history.append((source, value))
            if self.mode == "zero":
                if spec.type == DRIVE and any(value):
                    self.limits.add(topic + " observed nonzero zero-mode command")
                if topic == "/stand_pf/motor_speed":
                    self.limits.add(topic + " observed speed-mode output in zero mode")
                if topic == "/stand_pf/health" and abs(value["erpm"]) >= 100:
                    self.limits.add(topic + " observed nonstationary eRPM in zero mode")
            if topic == "/stand_pf/motor_current" and value != 0:
                self.limits.add(topic + " observed nonzero or invalid current demand")
            if topic == "/stand_pf/motor_speed" and not 0 <= value <= 15000:
                self.limits.add(topic + " observed out-of-range eRPM demand")
            if topic == "/stand_pf/servo" and not .10 - 1e-6 <= value <= .76 + 1e-6:
                self.limits.add(topic + " observed invalid normalized servo command")
            self.latest[topic] = (source, mono_now, value)
            self.invalid.pop(topic, None)
        except (ValueError, ArithmeticError, TypeError, KeyError) as exc:
            self.latest.pop(topic, None)
            self.invalid[topic] = str(exc)
            if spec.type in (DRIVE, SCALAR):
                self.limits.add(topic + " rejected command: " + str(exc))

    def issue(self, topic, ros_now, mono_now):
        if topic not in self.latest:
            return self.invalid.get(topic, "no sample")
        source, receipt, value = self.latest[topic]
        if not 0 <= mono_now - receipt <= .50:
            return "monotonic receipt age outside [0, .50] s"
        if source is not None:
            max_age = .505 if self.topics[topic].type in (ODOM, SCAN) else .50
            if not -.02 <= ros_now - source <= max_age:
                return "ROS source age outside window"
        if topic in (VALID_TOPIC, "/baseline/planner_valid", "/stand_pf/health_permit") and value is not True:
            return "false"
        if topic == "/baseline/controller_status" and (value.get("valid") is not True or (
                self.mode == "zero" and value.get("reason") != "waiting_for_operator")):
            return "controller reports " + str(value.get("reason"))
        if topic == "/baseline/final_status":
            if (value.get("arm_state") not in (("DISARMED",) if self.mode == "zero" else ("DISARMED", "ARMED"))
                    or value.get("latched") is not False or value.get("real_vehicle_profile") is not True
                    or value.get("straight_line_fallback_allowed") is not False):
                return "unexpected final guard state/profile " + str(value.get("stop_reason"))
        if topic == "/stand_pf/health_guard_status" and (value.get("healthy") is not True
                                                         or value.get("qualified") is not True):
            return "health guard reports " + str(value.get("reason"))
        if topic == "/stand_pf/actuator_status":
            modes = ("coast",) if self.mode == "zero" else ("speed", "coast")
            if value.get("motor_mode") not in modes or value.get("stop_reason") != "":
                return "actuator reports " + str(value.get("stop_reason"))
        if topic == "/stand_pf/window_status" and (value.get("latched") is not False
                                                    or value.get("health_ready") is not True):
            return "window reports " + str(value.get("reason"))
        return None

    def snapshot(self, ros_now, mono_now):
        if any(self.issue(t, ros_now, mono_now) for t in SENSORS):
            return None
        scan_stamp, _, scan = self.latest[SCAN_TOPIC]
        if self.last_scan is not None and scan_stamp <= self.last_scan:
            return None
        aligned = aligned_pose(self.history, scan_stamp)
        if aligned is None:
            return None
        pose_stamp, _, (x, y, yaw) = self.latest[POSE_TOPIC]
        odom_stamp, _, (v, vy, yaw_rate) = self.latest[ODOM_TOPIC]
        self.last_scan = scan_stamp
        self.last_snapshot_receipt = mono_now
        self.snapshots += 1
        return dict(schema="pnc.vehicle.v1", stamp=pose_stamp, scan_stamp=scan_stamp, odom_stamp=odom_stamp,
                    x=x, y=y, yaw=yaw, v=v, vy=vy, yaw_rate=yaw_rate, scan_pose=aligned,
                    laser_offset_x=.27, angle_min=scan.angle_min, angle_increment=scan.angle_increment,
                    range_min=scan.range_min, range_max=scan.range_max,
                    ranges=[r if math.isfinite(r) else None for r in scan.ranges])

    def report(self, ros_now, mono_now):
        issues = sorted(self.limits)
        for topic, spec in self.topics.items():
            if spec.sampled or (self.mode == "zero" and topic == "/stand_pf/motor_current"):
                error = self.issue(topic, ros_now, mono_now)
                if error:
                    issues.append(topic + ": " + error)
        if (self.snapshots < 2 or self.last_snapshot_receipt is None
                or not 0 <= mono_now - self.last_snapshot_receipt <= .5):
            issues.append("fewer than two synchronized snapshots or latest snapshot stale")
        stream = {}
        for topic, (source, receipt, value) in self.latest.items():
            row = {"receipt_age_s": mono_now - receipt}
            if source is not None:
                row["source_age_s"] = ros_now - source
            kind = self.topics[topic].type
            if kind in (BOOL, DRIVE, SCALAR, JOY):
                row["value"] = value
            if kind == TEXT:
                row["status"] = {k: value[k] for k in ("reason", "valid", "healthy", "health_ready", "latched",
                                                        "stop_reason", "arm_state", "motor_mode",
                                                        "fault", "erpm") if k in value
                                 and (isinstance(value[k], (str, bool)) or (type(value[k]) in (int, float)
                                      and math.isfinite(value[k])))}
            stream[topic] = row
        return issues, {"count": self.count, "last": stream, "invalid_latest": self.invalid,
                        "synchronized_count": self.snapshots}
