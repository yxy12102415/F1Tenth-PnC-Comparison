"""Last software owner of VESC motor/servo topics for authorized stand tests."""

import json
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .safety_core import HardwareActuatorConfig, HardwareActuatorCore


class HardwareActuatorGuard(Node):
    def __init__(self) -> None:
        super().__init__('hardware_actuator_guard')
        defaults = {
            'final_topic': '/stage1_hardware/final_command',
            'motor_topic': '/commands/motor/speed',
            'motor_current_topic': '/commands/motor/current',
            'servo_topic': '/commands/servo/position',
            'status_topic': '/stage1_hardware/actuator_status',
            'final_command_timeout': 0.10,
            'speed_to_erpm_gain': 7500.0,
            'max_abs_speed': 0.10,
            'max_abs_rpm': 750.0,
            'steering_gain': -1.2135,
            'steering_offset': 0.4304,
            'max_abs_steering': 0.27,
            'servo_min': 0.10,
            'servo_max': 0.76,
            'publish_rate': 100.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.core = HardwareActuatorCore(HardwareActuatorConfig(
            final_command_timeout=float(self._param('final_command_timeout')),
            speed_to_erpm_gain=float(self._param('speed_to_erpm_gain')),
            max_abs_speed=float(self._param('max_abs_speed')),
            max_abs_rpm=float(self._param('max_abs_rpm')),
            steering_gain=float(self._param('steering_gain')),
            steering_offset=float(self._param('steering_offset')),
            max_abs_steering=float(self._param('max_abs_steering')),
            servo_min=float(self._param('servo_min')),
            servo_max=float(self._param('servo_max')),
        ))
        self.motor_pub = self.create_publisher(
            Float64, str(self._param('motor_topic')), 10
        )
        self.motor_current_pub = self.create_publisher(
            Float64, str(self._param('motor_current_topic')), 10
        )
        self.servo_pub = self.create_publisher(
            Float64, str(self._param('servo_topic')), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self._param('status_topic')), 10
        )
        self.create_subscription(
            AckermannDriveStamped, str(self._param('final_topic')),
            self._command_callback, 10
        )
        rate = max(1.0, float(self._param('publish_rate')))
        self.create_timer(1.0 / rate, self._publish)
        self.last_reason = None
        self.last_rpm_sign = None
        self.last_motor_mode = None

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _command_callback(self, msg: AckermannDriveStamped) -> None:
        self.core.update(
            float(msg.drive.speed), float(msg.drive.steering_angle), time.monotonic()
        )

    def _publish(self) -> None:
        decision = self.core.evaluate(time.monotonic())
        if decision.motor_mode == 'speed':
            self.motor_pub.publish(Float64(data=float(decision.rpm)))
        else:
            self.motor_current_pub.publish(Float64(data=float(decision.current)))
        self.servo_pub.publish(Float64(data=float(decision.servo)))
        self.status_pub.publish(String(data=json.dumps({
            'stop_reason': decision.stop_reason,
            'input_age_sec': round(decision.input_age, 6),
            'rpm': decision.rpm,
            'current': decision.current,
            'motor_mode': decision.motor_mode,
            'servo': decision.servo,
        }, sort_keys=True)))
        rpm_sign = 1 if decision.rpm > 0.0 else (-1 if decision.rpm < 0.0 else 0)
        if rpm_sign != self.last_rpm_sign:
            self.get_logger().info(
                f'ACTUATOR RPM sign={rpm_sign} rpm={decision.rpm:.1f}'
            )
            self.last_rpm_sign = rpm_sign
        if decision.motor_mode != self.last_motor_mode:
            self.get_logger().info(
                f'ACTUATOR motor_mode={decision.motor_mode}'
            )
            self.last_motor_mode = decision.motor_mode
        if decision.stop_reason != self.last_reason:
            if decision.stop_reason:
                self.get_logger().warn(f'ACTUATOR STOP: {decision.stop_reason}')
            else:
                self.get_logger().info('ACTUATOR command fresh')
            self.last_reason = decision.stop_reason

    def publish_stop(self) -> None:
        self.motor_current_pub.publish(Float64(data=0.0))
        self.servo_pub.publish(Float64(data=self.core.config.steering_offset))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareActuatorGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
