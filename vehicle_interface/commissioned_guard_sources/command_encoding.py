"""Encode bounded commands without rounding past a strict receiver limit."""

import math
import struct


def bounded_float32(value: float, limit: float) -> float:
    """Reject invalid inputs; round a valid boundary inward by at most one ULP.

    Ackermann fields are float32, whereas ROS parameter limits are float64.
    This does not clip an already out-of-bounds planner result into validity.
    """
    if not math.isfinite(limit) or limit <= 0.0:
        raise ValueError('invalid command limit')
    if not math.isfinite(value) or abs(value) > limit:
        raise ValueError('invalid or out-of-bounds command')
    try:
        encoded = struct.pack('!f', abs(value))
    except OverflowError as exc:
        raise ValueError('command cannot be represented as float32') from exc
    magnitude = struct.unpack('!f', encoded)[0]
    if magnitude > limit:
        bits = struct.unpack('!I', encoded)[0]
        magnitude = struct.unpack('!f', struct.pack('!I', bits - 1))[0]
    return math.copysign(magnitude, value)


def encode_route_command(speed: float, steering: float,
                         max_speed: float, max_steering: float):
    """Forward-only route commands, ready for lossless float32 serialization."""
    if speed < 0.0:
        raise ValueError('reverse route command')
    return (bounded_float32(speed, max_speed),
            bounded_float32(steering, max_steering))
