"""ROS 2 final-command guard. Defaults are isolated from all vehicle topics."""

import json
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String

from .safety_core import FinalGuardConfig, FinalGuardCore


OFFLINE_PREFIX = '/stage1_offline'


class FinalCommandGuard(Node):
    def __init__(self) -> None:
        super().__init__('final_command_guard')

        defaults = {
            'nominal_topic': OFFLINE_PREFIX + '/drive_nominal',
            'scan_topic': OFFLINE_PREFIX + '/scan',
            'odom_topic': OFFLINE_PREFIX + '/odom',
            'planner_valid_topic': OFFLINE_PREFIX + '/planner_valid',
            'target_belief_age_topic': OFFLINE_PREFIX + '/target_belief_age',
            'deadman_topic': OFFLINE_PREFIX + '/teleop_deadman',
            'arm_topic': OFFLINE_PREFIX + '/arm',
            'reset_topic': OFFLINE_PREFIX + '/reset_latch',
            'final_topic': OFFLINE_PREFIX + '/final_command',
            'status_topic': OFFLINE_PREFIX + '/final_guard_status',
            'nominal_timeout': 0.12,
            'scan_timeout': 0.12,
            'odom_timeout': 0.12,
            'planner_timeout': 0.12,
            'belief_timeout': 0.12,
            'deadman_timeout': 0.12,
            'max_target_belief_age': 0.25,
            'max_speed': 0.50,
            'max_steering_angle': 0.45,
            'require_deadman': True,
            'require_target_belief': True,
            'real_vehicle_profile': True,
            'allow_straight_line_fallback': False,
            'publish_rate': 50.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        config = FinalGuardConfig(
            nominal_timeout=float(self._param('nominal_timeout')),
            scan_timeout=float(self._param('scan_timeout')),
            odom_timeout=float(self._param('odom_timeout')),
            planner_timeout=float(self._param('planner_timeout')),
            belief_timeout=float(self._param('belief_timeout')),
            deadman_timeout=float(self._param('deadman_timeout')),
            max_target_belief_age=float(self._param('max_target_belief_age')),
            max_speed=float(self._param('max_speed')),
            max_steering_angle=float(self._param('max_steering_angle')),
            require_deadman=bool(self._param('require_deadman')),
            require_target_belief=bool(self._param('require_target_belief')),
            real_vehicle_profile=bool(self._param('real_vehicle_profile')),
            allow_straight_line_fallback=bool(
                self._param('allow_straight_line_fallback')
            ),
        )
        self.core = FinalGuardCore(config)

        self.final_pub = self.create_publisher(
            AckermannDriveStamped, str(self._param('final_topic')), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self._param('status_topic')), 10
        )
        self.create_subscription(
            AckermannDriveStamped,
            str(self._param('nominal_topic')),
            self._nominal_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self._param('scan_topic')),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, str(self._param('odom_topic')), self._odom_callback, 10
        )
        self.create_subscription(
            Bool,
            str(self._param('planner_valid_topic')),
            self._planner_callback,
            10,
        )
        self.create_subscription(
            Float64,
            str(self._param('target_belief_age_topic')),
            self._belief_callback,
            10,
        )
        self.create_subscription(
            Bool, str(self._param('deadman_topic')), self._deadman_callback, 10
        )
        self.create_subscription(
            Bool, str(self._param('arm_topic')), self._arm_callback, 10
        )
        self.create_subscription(
            Bool, str(self._param('reset_topic')), self._reset_callback, 10
        )

        publish_rate = max(1.0, float(self._param('publish_rate')))
        self.timer = self.create_timer(1.0 / publish_rate, self._publish)
        self.last_logged_state = None

    def _param(self, name: str):
        return self.get_parameter(name).value

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _nominal_callback(self, msg: AckermannDriveStamped) -> None:
        self.core.update_nominal(
            float(msg.drive.speed), float(msg.drive.steering_angle), self._now()
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        metadata = (
            msg.angle_min,
            msg.angle_max,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )
        valid = all(math.isfinite(float(value)) for value in metadata)
        valid = valid and msg.angle_increment > 0.0 and msg.range_max > msg.range_min
        self.core.update_scan(valid, self._now())

    def _odom_callback(self, msg: Odometry) -> None:
        values = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )
        self.core.update_odom(
            all(math.isfinite(float(value)) for value in values), self._now()
        )

    def _planner_callback(self, msg: Bool) -> None:
        self.core.update_planner(bool(msg.data), self._now())

    def _belief_callback(self, msg: Float64) -> None:
        self.core.update_belief(float(msg.data), self._now())

    def _deadman_callback(self, msg: Bool) -> None:
        self.core.update_deadman(bool(msg.data), self._now())

    def _arm_callback(self, msg: Bool) -> None:
        self.core.set_arm(bool(msg.data))

    def _reset_callback(self, msg: Bool) -> None:
        if msg.data and not self.core.reset_latch():
            self.get_logger().warn('Latch reset rejected while arm is requested')

    def _publish(self) -> None:
        decision = self.core.evaluate(self._now())
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.drive.speed = float(decision.speed)
        command.drive.steering_angle = float(decision.steering_angle)
        command.drive.acceleration = 0.0
        self.final_pub.publish(command)

        status = {
            'arm_state': decision.arm_state,
            'armed': decision.arm_state == 'ARMED',
            'latched': decision.latched,
            'stop_reason': decision.stop_reason,
            'input_age_sec': {
                key: round(value, 6) for key, value in decision.ages.items()
            },
            'real_vehicle_profile': self.core.config.real_vehicle_profile,
            'straight_line_fallback_allowed': (
                self.core.config.allow_straight_line_fallback
            ),
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

        log_state = (decision.arm_state, decision.stop_reason)
        if log_state != self.last_logged_state:
            if decision.arm_state == 'ARMED':
                self.get_logger().info('ARMED: final command forwarding enabled')
            else:
                self.get_logger().warn(
                    f'STOP: state={decision.arm_state} reason={decision.stop_reason}'
                )
            self.last_logged_state = log_state

    def publish_shutdown_stop(self) -> None:
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.drive.speed = 0.0
        command.drive.steering_angle = 0.0
        self.final_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FinalCommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
