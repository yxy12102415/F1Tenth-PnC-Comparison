"""Deterministic safety state machines shared by ROS nodes and offline tests."""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Tuple


NEVER_SEEN_AGE = 1.0e9


@dataclass(frozen=True)
class FinalGuardConfig:
    nominal_timeout: float = 0.12
    scan_timeout: float = 0.12
    odom_timeout: float = 0.12
    planner_timeout: float = 0.12
    belief_timeout: float = 0.12
    deadman_timeout: float = 0.12
    max_target_belief_age: float = 0.25
    max_speed: float = 0.50
    max_steering_angle: float = 0.45
    require_deadman: bool = True
    require_target_belief: bool = True
    real_vehicle_profile: bool = True
    allow_straight_line_fallback: bool = False


@dataclass(frozen=True)
class FinalDecision:
    speed: float
    steering_angle: float
    arm_state: str
    stop_reason: str
    ages: Dict[str, float]
    latched: bool


@dataclass
class FinalGuardCore:
    config: FinalGuardConfig = field(default_factory=FinalGuardConfig)
    arm_requested: bool = False
    latched_reason: Optional[str] = None
    nominal_speed: float = 0.0
    nominal_steering: float = 0.0
    planner_valid: bool = False
    target_belief_age: float = NEVER_SEEN_AGE
    deadman_active: bool = False
    stamps: Dict[str, Optional[float]] = field(default_factory=lambda: {
        'nominal': None,
        'scan': None,
        'odom': None,
        'planner': None,
        'belief': None,
        'deadman': None,
    })

    def set_arm(self, requested: bool) -> None:
        self.arm_requested = bool(requested)

    def reset_latch(self) -> bool:
        """Clear a fault only while explicitly disarmed."""
        if self.arm_requested:
            return False
        self.latched_reason = None
        return True

    def latch(self, reason: str) -> None:
        if self.latched_reason is None:
            self.latched_reason = reason

    def update_nominal(self, speed: float, steering: float, now: float) -> None:
        self.stamps['nominal'] = now
        self.nominal_speed = float(speed)
        self.nominal_steering = float(steering)
        if not math.isfinite(speed) or not math.isfinite(steering):
            self.latch('invalid_nominal_non_finite')
        elif speed < 0.0 or speed > self.config.max_speed:
            self.latch('nominal_speed_out_of_bounds')
        elif abs(steering) > self.config.max_steering_angle:
            self.latch('nominal_steering_out_of_bounds')

    def update_scan(self, valid: bool, now: float) -> None:
        self.stamps['scan'] = now
        if self.arm_requested and not valid:
            self.latch('invalid_scan')

    def update_odom(self, valid: bool, now: float) -> None:
        self.stamps['odom'] = now
        if self.arm_requested and not valid:
            self.latch('invalid_odom_non_finite')

    def update_planner(self, valid: bool, now: float) -> None:
        self.stamps['planner'] = now
        self.planner_valid = bool(valid)
        if self.arm_requested and not valid:
            self.latch('planner_invalid')

    def update_belief(self, age: float, now: float) -> None:
        self.stamps['belief'] = now
        self.target_belief_age = float(age)
        if self.arm_requested and (not math.isfinite(age) or age < 0.0):
            self.latch('invalid_target_belief_age')
        elif self.arm_requested and age > self.config.max_target_belief_age:
            self.latch('target_belief_stale')

    def update_deadman(self, active: bool, now: float) -> None:
        self.stamps['deadman'] = now
        self.deadman_active = bool(active)
        if self.arm_requested and self.config.require_deadman and not active:
            self.latch('teleop_deadman_released')

    def ages(self, now: float) -> Dict[str, float]:
        return {
            key: NEVER_SEEN_AGE if stamp is None else max(0.0, now - stamp)
            for key, stamp in self.stamps.items()
        }

    def _freshness_fault(self, ages: Dict[str, float]) -> Optional[str]:
        limits = (
            ('nominal', self.config.nominal_timeout),
            ('scan', self.config.scan_timeout),
            ('odom', self.config.odom_timeout),
            ('planner', self.config.planner_timeout),
        )
        if self.config.require_target_belief:
            limits += (('belief', self.config.belief_timeout),)
        if self.config.require_deadman:
            limits += (('deadman', self.config.deadman_timeout),)
        for name, limit in limits:
            if ages[name] > limit:
                return f'{name}_timeout'
        return None

    def evaluate(self, now: float) -> FinalDecision:
        ages = self.ages(now)

        if self.config.real_vehicle_profile and self.config.allow_straight_line_fallback:
            self.latch('unsafe_straight_line_fallback_enabled')

        if self.arm_requested:
            freshness_fault = self._freshness_fault(ages)
            if freshness_fault is not None:
                self.latch(freshness_fault)
            if not self.planner_valid:
                self.latch('planner_invalid')
            if (self.config.require_target_belief
                    and self.target_belief_age > self.config.max_target_belief_age):
                self.latch('target_belief_stale')
            if self.config.require_deadman and not self.deadman_active:
                self.latch('teleop_deadman_released')

        if self.latched_reason is not None:
            return FinalDecision(0.0, 0.0, 'LATCHED', self.latched_reason, ages, True)
        if not self.arm_requested:
            return FinalDecision(0.0, 0.0, 'DISARMED', 'disarmed', ages, False)
        return FinalDecision(
            self.nominal_speed,
            self.nominal_steering,
            'ARMED',
            '',
            ages,
            False,
        )


@dataclass(frozen=True)
class StaleCommandConfig:
    final_command_timeout: float = 0.10
    speed_to_erpm_gain: float = 1000.0
    max_abs_rpm: float = 1000.0


@dataclass(frozen=True)
class RpmDecision:
    rpm: float
    stop_reason: str
    input_age: float


@dataclass
class StaleCommandCore:
    config: StaleCommandConfig = field(default_factory=StaleCommandConfig)
    desired_rpm: float = 0.0
    last_command_stamp: Optional[float] = None
    invalid_latched: bool = False

    def update(self, speed: float, now: float) -> None:
        self.last_command_stamp = now
        if not math.isfinite(speed):
            self.desired_rpm = 0.0
            self.invalid_latched = True
            return
        rpm = speed * self.config.speed_to_erpm_gain
        if not math.isfinite(rpm) or abs(rpm) > self.config.max_abs_rpm:
            self.desired_rpm = 0.0
            self.invalid_latched = True
            return
        self.desired_rpm = rpm
        self.invalid_latched = False

    def evaluate(self, now: float) -> RpmDecision:
        age = NEVER_SEEN_AGE if self.last_command_stamp is None else max(
            0.0, now - self.last_command_stamp
        )
        if self.invalid_latched:
            return RpmDecision(0.0, 'invalid_final_command', age)
        if age > self.config.final_command_timeout:
            self.desired_rpm = 0.0
            return RpmDecision(0.0, 'final_command_timeout', age)
        return RpmDecision(self.desired_rpm, '', age)


@dataclass(frozen=True)
class VirtualSinkConfig:
    motor_command_timeout: float = 0.15
    max_abs_rpm: float = 1000.0


@dataclass
class VirtualSinkCore:
    config: VirtualSinkConfig = field(default_factory=VirtualSinkConfig)
    commanded_rpm: float = 0.0
    last_rpm_stamp: Optional[float] = None

    def update(self, rpm: float, now: float) -> None:
        self.last_rpm_stamp = now
        if not math.isfinite(rpm) or abs(rpm) > self.config.max_abs_rpm:
            self.commanded_rpm = 0.0
        else:
            self.commanded_rpm = float(rpm)

    def evaluate(self, now: float) -> RpmDecision:
        age = NEVER_SEEN_AGE if self.last_rpm_stamp is None else max(
            0.0, now - self.last_rpm_stamp
        )
        if age > self.config.motor_command_timeout:
            self.commanded_rpm = 0.0
            return RpmDecision(0.0, 'motor_command_timeout', age)
        return RpmDecision(self.commanded_rpm, '', age)


@dataclass(frozen=True)
class Ps4TestConfig:
    joy_timeout: float = 0.15
    max_speed: float = 0.10
    max_steering_angle: float = 0.27
    axis_deadzone: float = 0.10
    # Bluetooth DS4 mapping verified on this vehicle:
    # right-stick vertical (Ry) drives, left-stick horizontal (X) steers.
    speed_axis: int = 4
    steering_axis: int = 0
    deadman_button: int = 4
    speed_axis_scale: float = -1.0
    min_effective_speed: float = 0.0
    max_acceleration: float = 0.0


@dataclass(frozen=True)
class ChassisDecision:
    speed: float
    steering_angle: float
    active: bool
    stop_reason: str
    input_age: float


@dataclass
class Ps4TestCore:
    """Hold-to-run PS4 command source with neutral-on-arm interlock."""

    config: Ps4TestConfig = field(default_factory=Ps4TestConfig)
    last_joy_stamp: Optional[float] = None
    speed_axis_value: float = 0.0
    steering_axis_value: float = 0.0
    deadman: bool = False
    neutral_seen_while_released: bool = False
    invalid_input: bool = False
    active_direction: int = 0
    direction_change_blocked: bool = False
    output_speed: float = 0.0
    last_evaluate_time: Optional[float] = None

    def _stop(self, now: float, reason: str) -> ChassisDecision:
        self.output_speed = 0.0
        self.last_evaluate_time = now
        return ChassisDecision(0.0, 0.0, False, reason, self._age(now))

    def _age(self, now: float) -> float:
        return NEVER_SEEN_AGE if self.last_joy_stamp is None else max(
            0.0, now - self.last_joy_stamp
        )

    def update(self, axes, buttons, now: float) -> None:
        self.last_joy_stamp = now
        required_axes = max(self.config.speed_axis, self.config.steering_axis)
        if len(axes) <= required_axes or len(buttons) <= self.config.deadman_button:
            self.invalid_input = True
            self.deadman = False
            return
        speed_axis = float(axes[self.config.speed_axis])
        steering_axis = float(axes[self.config.steering_axis])
        if not math.isfinite(speed_axis) or not math.isfinite(steering_axis):
            self.invalid_input = True
            self.deadman = False
            return
        if abs(speed_axis) > 1.01 or abs(steering_axis) > 1.01:
            self.invalid_input = True
            self.deadman = False
            return
        self.invalid_input = False
        self.speed_axis_value = speed_axis
        self.steering_axis_value = steering_axis
        self.deadman = bool(buttons[self.config.deadman_button])
        centered = (
            abs(speed_axis) <= self.config.axis_deadzone
            and abs(steering_axis) <= self.config.axis_deadzone
        )
        if not self.deadman and centered:
            self.neutral_seen_while_released = True
            self.active_direction = 0
            self.direction_change_blocked = False
        elif not self.deadman:
            self.neutral_seen_while_released = False

    def evaluate(self, now: float) -> ChassisDecision:
        age = self._age(now)
        if age > self.config.joy_timeout:
            self.deadman = False
            self.neutral_seen_while_released = False
            return self._stop(now, 'joy_timeout')
        if self.invalid_input:
            return self._stop(now, 'invalid_joy')
        if not self.deadman:
            return self._stop(now, 'deadman_released')
        if not self.neutral_seen_while_released:
            return self._stop(now, 'neutral_arm_interlock')

        def apply_deadzone(value: float) -> float:
            return 0.0 if abs(value) <= self.config.axis_deadzone else value

        speed = (
            self.config.speed_axis_scale
            * apply_deadzone(self.speed_axis_value)
            * self.config.max_speed
        )
        if speed != 0.0:
            speed = math.copysign(
                min(
                    self.config.max_speed,
                    max(abs(speed), self.config.min_effective_speed),
                ),
                speed,
            )
        direction = 1 if speed > 0.0 else (-1 if speed < 0.0 else 0)
        if self.direction_change_blocked:
            return self._stop(now, 'direction_change_interlock')
        if direction and self.active_direction and direction != self.active_direction:
            self.direction_change_blocked = True
            return self._stop(now, 'direction_change_interlock')
        if direction:
            self.active_direction = direction
        if speed == 0.0:
            self.output_speed = 0.0
        elif self.config.max_acceleration > 0.0:
            dt = 0.0 if self.last_evaluate_time is None else max(
                0.0, now - self.last_evaluate_time
            )
            max_delta = self.config.max_acceleration * dt
            delta = speed - self.output_speed
            self.output_speed += max(-max_delta, min(delta, max_delta))
        else:
            self.output_speed = speed
        self.last_evaluate_time = now
        steering = apply_deadzone(
            self.steering_axis_value
        ) * self.config.max_steering_angle
        return ChassisDecision(self.output_speed, steering, True, '', age)


@dataclass(frozen=True)
class HardwareActuatorConfig:
    final_command_timeout: float = 0.10
    speed_to_erpm_gain: float = 7500.0
    max_abs_speed: float = 0.10
    max_abs_rpm: float = 750.0
    steering_gain: float = -1.2135
    steering_offset: float = 0.4304
    max_abs_steering: float = 0.27
    servo_min: float = 0.10
    servo_max: float = 0.76


@dataclass(frozen=True)
class ActuatorDecision:
    rpm: float
    current: float
    motor_mode: str
    servo: float
    stop_reason: str
    input_age: float


@dataclass
class HardwareActuatorCore:
    config: HardwareActuatorConfig = field(default_factory=HardwareActuatorConfig)
    last_command_stamp: Optional[float] = None
    desired_rpm: float = 0.0
    desired_servo: float = 0.4304
    invalid: bool = False

    def update(self, speed: float, steering: float, now: float) -> None:
        self.last_command_stamp = now
        values = (speed, steering)
        if not all(math.isfinite(value) for value in values):
            self.invalid = True
            self.desired_rpm = 0.0
            self.desired_servo = self.config.steering_offset
            return
        rpm = speed * self.config.speed_to_erpm_gain
        servo = steering * self.config.steering_gain + self.config.steering_offset
        # AckermannDrive fields are float32. Allow only enough tolerance for
        # float32 round-trip at an exact configured boundary; published values
        # remain the original bounded command.
        tolerance = 1.0e-6
        valid = (
            abs(speed) <= self.config.max_abs_speed + tolerance
            and abs(rpm) <= self.config.max_abs_rpm + tolerance * abs(
                self.config.speed_to_erpm_gain
            )
            and abs(steering) <= self.config.max_abs_steering + tolerance
            and self.config.servo_min - tolerance <= servo
            and servo <= self.config.servo_max + tolerance
        )
        self.invalid = not valid
        self.desired_rpm = rpm if valid else 0.0
        self.desired_servo = servo if valid else self.config.steering_offset

    def evaluate(self, now: float) -> ActuatorDecision:
        age = NEVER_SEEN_AGE if self.last_command_stamp is None else max(
            0.0, now - self.last_command_stamp
        )
        if self.invalid:
            return ActuatorDecision(
                0.0, 0.0, 'coast', self.config.steering_offset,
                'invalid_final_command', age
            )
        if age > self.config.final_command_timeout:
            self.desired_rpm = 0.0
            self.desired_servo = self.config.steering_offset
            return ActuatorDecision(
                0.0, 0.0, 'coast', self.config.steering_offset,
                'final_command_timeout', age
            )
        # SET_RPM(0) is active closed-loop braking on the VESC and was observed
        # to reverse the unloaded drivetrain briefly. Zero speed therefore uses
        # SET_CURRENT(0), which removes commanded motor torque instead.
        if self.desired_rpm == 0.0:
            return ActuatorDecision(
                0.0, 0.0, 'coast', self.desired_servo, '', age
            )
        return ActuatorDecision(
            self.desired_rpm, 0.0, 'speed', self.desired_servo, '', age
        )


@dataclass(frozen=True)
class WaypointRecorderConfig:
    min_distance: float = 0.10
    min_heading: float = 0.10


@dataclass
class WaypointRecorderCore:
    config: WaypointRecorderConfig = field(default_factory=WaypointRecorderConfig)
    points: List[Tuple[float, float, float]] = field(default_factory=list)

    def add(self, x: float, y: float, yaw: float) -> bool:
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return False
        if self.points:
            last_x, last_y, last_yaw = self.points[-1]
            distance = math.hypot(x - last_x, y - last_y)
            heading = abs(math.atan2(
                math.sin(yaw - last_yaw), math.cos(yaw - last_yaw)
            ))
            if (
                distance < self.config.min_distance
                and heading < self.config.min_heading
            ):
                return False
        self.points.append((float(x), float(y), float(yaw)))
        return True


@dataclass
class InitialPoseGateCore:
    expected_frame: str = 'map'
    accepted: bool = False
    stop_reason: str = 'initial_pose_missing'

    def update(
        self,
        frame_id: str,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float],
        covariance: List[float],
    ) -> bool:
        values = tuple(position) + tuple(orientation) + tuple(covariance)
        if not all(math.isfinite(value) for value in values):
            self.stop_reason = 'initial_pose_non_finite'
            return False
        if frame_id != self.expected_frame:
            self.stop_reason = 'initial_pose_wrong_frame'
            return False
        qx, qy, qz, qw = orientation
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 0.99 or norm > 1.01:
            self.stop_reason = 'initial_pose_invalid_quaternion'
            return False
        if abs(position[2]) > 0.01 or abs(qx) > 0.01 or abs(qy) > 0.01:
            self.stop_reason = 'initial_pose_not_planar'
            return False
        self.accepted = True
        self.stop_reason = ''
        return True
