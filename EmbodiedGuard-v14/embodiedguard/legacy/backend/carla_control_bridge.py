from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Dict, Mapping


@dataclass
class CarlaControlCommand:
    """A simulator-neutral command that maps directly to CARLA VehicleControl."""

    throttle: float
    steer: float
    brake: float
    reverse: bool = False
    hand_brake: bool = False
    reason: str = "trajectory_control"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def emergency_brake_command(
    reason: str = "runtime_guard_rejected", *, hand_brake: bool = False
) -> CarlaControlCommand:
    """Return the single fail-closed command used at the CARLA boundary."""
    safe_reason = str(reason).strip() or "runtime_guard_rejected"
    return CarlaControlCommand(
        throttle=0.0,
        steer=0.0,
        brake=1.0,
        reverse=False,
        hand_brake=bool(hand_brake),
        reason=safe_reason,
    )


def carla_command_contract_errors(
    command: CarlaControlCommand | Mapping[str, Any] | None,
) -> list[str]:
    """Validate a command immediately before it reaches VehicleControl."""
    if isinstance(command, Mapping):
        try:
            command = CarlaControlCommand(**dict(command))
        except (TypeError, ValueError, OverflowError):
            return ["command_mapping_invalid"]
    if not isinstance(command, CarlaControlCommand):
        return ["command_type_invalid"]

    errors = []
    numeric = {
        "throttle": command.throttle,
        "steer": command.steer,
        "brake": command.brake,
    }
    parsed: dict[str, float] = {}
    for name, value in numeric.items():
        try:
            parsed[name] = float(value)
        except (TypeError, ValueError, OverflowError):
            errors.append(f"{name}_invalid")
            continue
        if not math.isfinite(parsed[name]):
            errors.append(f"{name}_non_finite")

    if "throttle" in parsed and not 0.0 <= parsed["throttle"] <= 1.0:
        errors.append("throttle_out_of_range")
    if "steer" in parsed and not -1.0 <= parsed["steer"] <= 1.0:
        errors.append("steer_out_of_range")
    if "brake" in parsed and not 0.0 <= parsed["brake"] <= 1.0:
        errors.append("brake_out_of_range")
    if parsed.get("throttle", 0.0) > 0.0 and parsed.get("brake", 0.0) > 0.0:
        errors.append("simultaneous_throttle_and_brake")
    if command.hand_brake is True and parsed.get("throttle", 0.0) > 0.0:
        errors.append("throttle_with_hand_brake")
    if type(command.reverse) is not bool:
        errors.append("reverse_not_boolean")
    if type(command.hand_brake) is not bool:
        errors.append("hand_brake_not_boolean")
    if not isinstance(command.reason, str) or not command.reason.strip():
        errors.append("reason_invalid")
    return sorted(set(errors))


def guard_carla_command(
    command: CarlaControlCommand | Mapping[str, Any] | None,
    *,
    failure_reason: str = "invalid_runtime_command",
) -> tuple[CarlaControlCommand, list[str]]:
    """Return a validated command, or a full-brake command plus errors."""
    errors = carla_command_contract_errors(command)
    if errors:
        return emergency_brake_command(failure_reason), errors
    if isinstance(command, Mapping):
        command = CarlaControlCommand(**dict(command))
    return CarlaControlCommand(
        throttle=float(command.throttle),
        steer=float(command.steer),
        brake=float(command.brake),
        reverse=command.reverse,
        hand_brake=command.hand_brake,
        reason=command.reason.strip(),
    ), []


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def ego_left_error_to_carla_steer_error(value: float) -> float:
    """Convert ego-local left-positive error to CARLA right-positive steer."""
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("steering error must be finite")
    return -parsed


def control_hint_to_carla_command(
    control_hint: Mapping[str, Any] | None,
    current_speed_mps: float,
    safety: Mapping[str, Any] | None,
    *,
    speed_kp: float = 0.12,
    brake_kp: float = 0.20,
    lateral_kp: float = 0.08,
    yaw_kp: float = 0.35,
    stop_speed_mps: float = 0.20,
) -> CarlaControlCommand:
    """Map an AutoVLA trajectory hint to a conservative CARLA P-controller.

    This is an execution smoke bridge, not the final PID/MPC controller. A
    rejected or missing safety decision always produces a braking command.
    """

    if not isinstance(safety, Mapping) or safety.get("safe") is not True:
        return emergency_brake_command("safety_rejected")

    try:
        if not isinstance(control_hint, Mapping):
            raise ValueError("control hint must be a mapping")
        target_speed = float(control_hint.get("target_speed_mps", 0.0))
        target_lateral = float(control_hint.get("target_lateral", 0.0))
        target_yaw = float(control_hint.get("target_yaw", 0.0))
        current_speed = float(current_speed_mps)
        gains = [speed_kp, brake_kp, lateral_kp, yaw_kp, stop_speed_mps]
        values = [target_speed, target_lateral, target_yaw, current_speed, *gains]
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("control inputs must be finite")
        if current_speed < 0.0 or any(float(value) < 0.0 for value in gains):
            raise ValueError("speed and controller gains must be non-negative")
    except (TypeError, ValueError, OverflowError):
        return emergency_brake_command("invalid_control_input")

    target_speed = max(0.0, target_speed)
    steer = _clamp(
        ego_left_error_to_carla_steer_error(
            lateral_kp * target_lateral + yaw_kp * target_yaw
        ),
        -0.5,
        0.5,
    )

    if target_speed <= stop_speed_mps:
        return CarlaControlCommand(
            0.0,
            steer,
            1.0,
            hand_brake=True,
            reason="near_stationary_target",
        )

    speed_error = target_speed - current_speed
    if speed_error >= 0.0:
        return CarlaControlCommand(
            throttle=_clamp(speed_kp * speed_error, 0.0, 0.35),
            steer=steer,
            brake=0.0,
            reason="track_target_speed",
        )
    return CarlaControlCommand(
        throttle=0.0,
        steer=steer,
        brake=_clamp(brake_kp * (-speed_error), 0.0, 0.6),
        reason="reduce_to_target_speed",
    )
