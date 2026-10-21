from __future__ import annotations

import copy
import math
from dataclasses import asdict, dataclass
from typing import Any, Dict, Mapping

from embodiedguard.legacy.backend.carla_control_bridge import (
    CarlaControlCommand,
    ego_left_error_to_carla_steer_error,
    emergency_brake_command,
)


CARLA_PID_CONFIG_SCHEMA_VERSION = "autovla.carla_pid_config.v1"


@dataclass(frozen=True)
class PIDAxisConfig:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    derivative_filter: float = 0.25


@dataclass(frozen=True)
class CarlaPIDConfig:
    """Conservative untuned gains for the CARLA engineering baseline."""

    speed: PIDAxisConfig = PIDAxisConfig(0.12, 0.02, 0.01, 20.0)
    lateral: PIDAxisConfig = PIDAxisConfig(0.08, 0.01, 0.01, 2.0)
    yaw: PIDAxisConfig = PIDAxisConfig(0.7, 0.02, 0.01, 1.0)
    stop_speed_mps: float = 0.20
    stationary_target_speed_mps: float = 0.05
    stationary_confirm_cycles: int = 5
    minimum_launch_throttle: float = 0.25
    max_throttle: float = 0.60
    max_brake: float = 0.60
    max_abs_steer: float = 0.50
    max_throttle_rate_per_sec: float = 2.0
    max_brake_rate_per_sec: float = 2.0
    max_steer_rate_per_sec: float = 1.5
    max_abs_target_lateral_m: float = 20.0
    max_abs_target_yaw_rad: float = math.pi
    max_dt_sec: float = 1.0
    lateral_deadband_m: float = 0.05
    steer_full_authority_speed_mps: float = 3.0
    min_effective_max_steer: float = 0.15


def _clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def _finite_float(value: Any, name: str) -> float:
    if type(value) is bool:
        raise ValueError(f"{name} must be numeric, not boolean")
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _validate_axis(config: PIDAxisConfig, name: str) -> None:
    if not isinstance(config, PIDAxisConfig):
        raise ValueError(f"{name} must be PIDAxisConfig")
    values = {
        "kp": config.kp,
        "ki": config.ki,
        "kd": config.kd,
        "integral_limit": config.integral_limit,
        "derivative_filter": config.derivative_filter,
    }
    parsed = {key: _finite_float(value, f"{name}.{key}") for key, value in values.items()}
    if any(parsed[key] < 0.0 for key in ("kp", "ki", "kd", "integral_limit")):
        raise ValueError(f"{name} gains and integral limit must be non-negative")
    if not 0.0 <= parsed["derivative_filter"] <= 1.0:
        raise ValueError(f"{name}.derivative_filter must be in [0, 1]")


class _PIDAxis:
    def __init__(self, config: PIDAxisConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error: float | None = None
        self.filtered_derivative = 0.0
        self._last_step: Dict[str, Any] | None = None

    def step(
        self,
        error: float,
        dt_sec: float,
        *,
        low: float,
        high: float,
        reset_integral_on_sign_change: bool = False,
    ) -> float:
        if (
            reset_integral_on_sign_change
            and self.previous_error is not None
            and error * self.previous_error < 0.0
        ):
            self.integral = 0.0
        raw_derivative = (
            0.0
            if self.previous_error is None
            else (error - self.previous_error) / dt_sec
        )
        alpha = float(self.config.derivative_filter)
        derivative = alpha * raw_derivative + (1.0 - alpha) * self.filtered_derivative
        candidate_integral = _clamp(
            self.integral + error * dt_sec,
            -float(self.config.integral_limit),
            float(self.config.integral_limit),
        )
        candidate_unconstrained = (
            float(self.config.kp) * error
            + float(self.config.ki) * candidate_integral
            + float(self.config.kd) * derivative
        )
        saturated_high = candidate_unconstrained > high and error > 0.0
        saturated_low = candidate_unconstrained < low and error < 0.0
        if not (saturated_high or saturated_low):
            self.integral = candidate_integral
            unconstrained = candidate_unconstrained
        else:
            unconstrained = (
                float(self.config.kp) * error
                + float(self.config.ki) * self.integral
                + float(self.config.kd) * derivative
            )
        self.previous_error = error
        self.filtered_derivative = derivative
        output = _clamp(unconstrained, low, high)
        self._last_step = {
            "error": float(error),
            "dt_sec": float(dt_sec),
            "proportional": float(self.config.kp) * error,
            "integral": float(self.config.ki) * self.integral,
            "derivative": float(self.config.kd) * derivative,
            "integral_state": self.integral,
            "candidate_integral_state": candidate_integral,
            "raw_derivative": raw_derivative,
            "filtered_derivative": derivative,
            "candidate_unconstrained_output": candidate_unconstrained,
            "unconstrained_output": unconstrained,
            "clamped_output": output,
            "saturated_high": bool(saturated_high),
            "saturated_low": bool(saturated_low),
            "anti_windup_applied": bool(saturated_high or saturated_low),
        }
        return output

    def last_step(self) -> Dict[str, Any] | None:
        return copy.deepcopy(self._last_step)

    def state(self) -> Dict[str, Any]:
        return {
            "integral": self.integral,
            "previous_error": self.previous_error,
            "filtered_derivative": self.filtered_derivative,
        }


class CarlaTrajectoryPIDController:
    """Stateful trajectory-hint PID with fail-closed reset semantics.

    This controller is an engineering baseline. Its gains are not presented as
    tuned driving-policy performance and it does not replace the Safety Shield.
    """

    SCHEMA_VERSION = "autovla.carla_pid_controller.v1"

    def __init__(self, config: CarlaPIDConfig | None = None):
        self.config = config or CarlaPIDConfig()
        self._validate_config()
        self._speed = _PIDAxis(self.config.speed)
        self._lateral = _PIDAxis(self.config.lateral)
        self._yaw = _PIDAxis(self.config.yaw)
        self._last_throttle = 0.0
        self._last_brake = 0.0
        self._last_steer = 0.0
        self._stationary_target_cycles = 0
        self._stationary_hold_active = False
        self._last_command_diagnostics: Dict[str, Any] | None = None

    def _validate_config(self) -> None:
        if not isinstance(self.config, CarlaPIDConfig):
            raise ValueError("config must be CarlaPIDConfig")
        for name in ("speed", "lateral", "yaw"):
            _validate_axis(getattr(self.config, name), name)
        positive = (
            "stop_speed_mps",
            "stationary_target_speed_mps",
            "max_throttle",
            "max_brake",
            "max_abs_steer",
            "max_throttle_rate_per_sec",
            "max_brake_rate_per_sec",
            "max_steer_rate_per_sec",
            "max_abs_target_lateral_m",
            "max_abs_target_yaw_rad",
            "max_dt_sec",
        )
        values = {
            name: _finite_float(getattr(self.config, name), name) for name in positive
        }
        for name in (
            "lateral_deadband_m",
            "steer_full_authority_speed_mps",
            "min_effective_max_steer",
        ):
            values[name] = _finite_float(getattr(self.config, name), name)
        if (
            type(self.config.stationary_confirm_cycles) is not int
            or self.config.stationary_confirm_cycles <= 0
        ):
            raise ValueError("stationary_confirm_cycles must be a positive integer")
        minimum_launch_throttle = _finite_float(
            self.config.minimum_launch_throttle,
            "minimum_launch_throttle",
        )
        if (
            minimum_launch_throttle <= 0.0
            or minimum_launch_throttle > values["max_throttle"]
        ):
            raise ValueError(
                "minimum_launch_throttle must be in (0, max_throttle]"
            )
        if values["stop_speed_mps"] < 0.0:
            raise ValueError("stop_speed_mps must be non-negative")
        if values["stationary_target_speed_mps"] < 0.0:
            raise ValueError("stationary_target_speed_mps must be non-negative")
        if (
            values["stationary_target_speed_mps"]
            > values["stop_speed_mps"]
        ):
            raise ValueError(
                "stationary_target_speed_mps must not exceed stop_speed_mps"
            )
        nonnegative = {"stop_speed_mps", "stationary_target_speed_mps"}
        if any(values[name] <= 0.0 for name in positive if name not in nonnegative):
            raise ValueError("controller limits and rates must be positive")
        for name in ("max_throttle", "max_brake", "max_abs_steer"):
            if values[name] > 1.0:
                raise ValueError(f"{name} must be in (0, 1]")
        if values["lateral_deadband_m"] < 0.0:
            raise ValueError("lateral_deadband_m must be non-negative")
        if values["steer_full_authority_speed_mps"] <= 0.0:
            raise ValueError("steer_full_authority_speed_mps must be positive")
        if not 0.0 < values["min_effective_max_steer"] <= values["max_abs_steer"]:
            raise ValueError(
                "min_effective_max_steer must be in (0, max_abs_steer]"
            )

    def reset(self) -> None:
        self._speed.reset()
        self._lateral.reset()
        self._yaw.reset()
        self._last_throttle = 0.0
        self._last_brake = 0.0
        self._last_steer = 0.0
        self._stationary_target_cycles = 0
        self._stationary_hold_active = False
        self._last_command_diagnostics = None

    @staticmethod
    def _rate_limit(target: float, previous: float, rate: float, dt_sec: float) -> float:
        maximum_change = rate * dt_sec
        return _clamp(target, previous - maximum_change, previous + maximum_change)

    def command(
        self,
        control_hint: Mapping[str, Any] | None,
        current_speed_mps: float,
        safety: Mapping[str, Any] | None,
        *,
        dt_sec: float,
    ) -> CarlaControlCommand:
        if not isinstance(safety, Mapping) or safety.get("safe") is not True:
            self.reset()
            self._last_command_diagnostics = {
                "status": "safety_rejected",
                "safety_safe": False,
            }
            return emergency_brake_command("safety_rejected")

        try:
            if not isinstance(control_hint, Mapping):
                raise ValueError("control_hint must be an object")
            required = {"target_speed_mps", "target_lateral", "target_yaw"}
            if not required.issubset(control_hint):
                raise ValueError("control_hint is missing required fields")
            target_speed = _finite_float(
                control_hint["target_speed_mps"], "target_speed_mps"
            )
            target_lateral = _finite_float(
                control_hint["target_lateral"], "target_lateral"
            )
            target_yaw = _finite_float(control_hint["target_yaw"], "target_yaw")
            current_speed = _finite_float(current_speed_mps, "current_speed_mps")
            dt = _finite_float(dt_sec, "dt_sec")
            if target_speed < 0.0 or current_speed < 0.0:
                raise ValueError("target and current speed must be non-negative")
            if not 0.0 < dt <= float(self.config.max_dt_sec):
                raise ValueError("dt_sec is outside the controller contract")
            if abs(target_lateral) > float(self.config.max_abs_target_lateral_m):
                raise ValueError("target_lateral is outside the controller contract")
            if abs(target_yaw) > float(self.config.max_abs_target_yaw_rad):
                raise ValueError("target_yaw is outside the controller contract")
        except (TypeError, ValueError, OverflowError):
            self.reset()
            self._last_command_diagnostics = {
                "status": "invalid_control_input",
                "safety_safe": True,
            }
            return emergency_brake_command("invalid_control_input")

        stop_speed = float(self.config.stop_speed_mps)
        near_stationary_target = (
            target_speed <= float(self.config.stationary_target_speed_mps)
        )
        if near_stationary_target:
            self._stationary_target_cycles += 1
        else:
            self._stationary_target_cycles = 0
            self._stationary_hold_active = False

        if (
            near_stationary_target
            and current_speed <= stop_speed
            and self._stationary_target_cycles
            >= self.config.stationary_confirm_cycles
        ):
            self._speed.reset()
            self._lateral.reset()
            self._yaw.reset()
            self._last_throttle = 0.0
            self._last_brake = 0.0
            self._last_steer = 0.0
            self._stationary_hold_active = True
            self._last_command_diagnostics = {
                "status": "stationary_hold",
                "target_speed_mps": target_speed,
                "current_speed_mps": current_speed,
                "speed_error_mps": target_speed - current_speed,
                "dt_sec": dt,
                "actuation": {
                    "applied_throttle": 0.0,
                    "applied_brake": 1.0,
                    "applied_steer": 0.0,
                    "hand_brake": True,
                },
            }
            return CarlaControlCommand(
                throttle=0.0,
                steer=0.0,
                brake=1.0,
                hand_brake=True,
                reason="near_stationary_target",
            )

        if (
            near_stationary_target
            and current_speed <= stop_speed
            and not self._stationary_hold_active
        ):
            self._last_throttle = 0.0
            self._last_brake = 0.0
            self._last_steer = 0.0
            self._last_command_diagnostics = {
                "status": "stationary_confirmation",
                "target_speed_mps": target_speed,
                "current_speed_mps": current_speed,
                "speed_error_mps": target_speed - current_speed,
                "dt_sec": dt,
                "actuation": {
                    "applied_throttle": 0.0,
                    "applied_brake": 0.0,
                    "applied_steer": 0.0,
                    "hand_brake": False,
                },
            }
            return CarlaControlCommand(
                throttle=0.0,
                steer=0.0,
                brake=0.0,
                hand_brake=False,
                reason="confirm_stationary_target",
            )

        signed_longitudinal = self._speed.step(
            target_speed - current_speed,
            dt,
            low=-float(self.config.max_brake),
            high=float(self.config.max_throttle),
            reset_integral_on_sign_change=True,
        )
        raw_target_throttle = max(0.0, signed_longitudinal)
        raw_target_brake = max(0.0, -signed_longitudinal)
        target_throttle = raw_target_throttle
        target_brake = raw_target_brake
        launch_floor_applied = False
        if (
            not near_stationary_target
            and current_speed <= stop_speed
            and target_throttle > 0.0
        ):
            launch_floor_applied = bool(
                target_throttle < float(self.config.minimum_launch_throttle)
            )
            target_throttle = max(
                target_throttle,
                float(self.config.minimum_launch_throttle),
            )
        if target_throttle > 0.0:
            throttle = self._rate_limit(
                target_throttle,
                0.0 if self._last_brake > 0.0 else self._last_throttle,
                float(self.config.max_throttle_rate_per_sec),
                dt,
            )
            brake = 0.0
        elif target_brake > 0.0:
            throttle = 0.0
            brake = self._rate_limit(
                target_brake,
                0.0 if self._last_throttle > 0.0 else self._last_brake,
                float(self.config.max_brake_rate_per_sec),
                dt,
            )
        else:
            throttle = 0.0
            brake = 0.0

        # 小误差死区:接近车道中心时不做横向修正,避免来回微调
        lateral_error = float(target_lateral)
        lateral_deadband_active = (
            abs(lateral_error) < float(self.config.lateral_deadband_m)
        )
        if lateral_deadband_active:
            lateral_error = 0.0
            self._lateral.reset()
        # 车速自适应转向限幅:低速全权限,高速按比例收紧
        reference_speed = float(self.config.steer_full_authority_speed_mps)
        attenuation = min(
            1.0, reference_speed / max(current_speed, reference_speed)
        )
        effective_max_steer = max(
            float(self.config.min_effective_max_steer),
            float(self.config.max_abs_steer) * attenuation,
        )
        target_steer = _clamp(
            ego_left_error_to_carla_steer_error(
                self._lateral.step(
                    lateral_error,
                    dt,
                    low=-effective_max_steer,
                    high=effective_max_steer,
                    reset_integral_on_sign_change=True,
                )
                + self._yaw.step(
                    target_yaw,
                    dt,
                    low=-effective_max_steer,
                    high=effective_max_steer,
                    reset_integral_on_sign_change=True,
                )
            ),
            -effective_max_steer,
            effective_max_steer,
        )
        rate_limited_steer = self._rate_limit(
            target_steer,
            self._last_steer,
            float(self.config.max_steer_rate_per_sec),
            dt,
        )
        # The speed-dependent authority is a safety boundary on the command
        # sent to CARLA, not merely a target for the slew-rate limiter.  When
        # the authority shrinks, the previous command can lie outside the new
        # interval, so rate limiting alone may otherwise preserve an unsafe
        # over-limit steer for several ticks.
        steer_hard_cap_applied = bool(
            abs(rate_limited_steer) > effective_max_steer + 1e-12
        )
        steer = _clamp(
            rate_limited_steer,
            -effective_max_steer,
            effective_max_steer,
        )
        self._last_throttle = throttle
        self._last_brake = brake
        self._last_steer = steer
        self._last_command_diagnostics = {
            "status": "tracking",
            "target_speed_mps": target_speed,
            "current_speed_mps": current_speed,
            "speed_error_mps": target_speed - current_speed,
            "target_lateral_m": target_lateral,
            "target_yaw_rad": target_yaw,
            "dt_sec": dt,
            "lateral_deadband_active": lateral_deadband_active,
            "effective_max_steer": effective_max_steer,
            "speed_pid": self._speed.last_step(),
            "lateral_pid": self._lateral.last_step(),
            "yaw_pid": self._yaw.last_step(),
            "actuation": {
                "signed_longitudinal_output": signed_longitudinal,
                "raw_target_throttle": raw_target_throttle,
                "raw_target_brake": raw_target_brake,
                "launch_floor_applied": launch_floor_applied,
                "target_throttle_before_rate_limit": target_throttle,
                "target_brake_before_rate_limit": target_brake,
                "target_steer_before_rate_limit": target_steer,
                "rate_limited_steer_before_hard_cap": rate_limited_steer,
                "throttle_rate_limited": bool(
                    abs(throttle - target_throttle) > 1e-12
                ),
                "brake_rate_limited": bool(abs(brake - target_brake) > 1e-12),
                "steer_rate_limited": bool(
                    abs(rate_limited_steer - target_steer) > 1e-12
                ),
                "steer_hard_cap_applied": steer_hard_cap_applied,
                "applied_throttle": throttle,
                "applied_brake": brake,
                "applied_steer": steer,
                "hand_brake": False,
            },
        }
        reason = "track_target_speed" if throttle > 0.0 else "reduce_to_target_speed"
        if throttle == 0.0 and brake == 0.0:
            reason = "hold_target_speed"
        return CarlaControlCommand(
            throttle=throttle,
            steer=steer,
            brake=brake,
            reason=reason,
        )

    def report(self) -> Dict[str, Any]:
        return {
            "schema_version": self.SCHEMA_VERSION,
            "controller": "stateful_pid_engineering_baseline",
            "config": asdict(self.config),
            "state": {
                "speed": self._speed.state(),
                "lateral": self._lateral.state(),
                "yaw": self._yaw.state(),
                "last_throttle": self._last_throttle,
                "last_brake": self._last_brake,
                "last_steer": self._last_steer,
                "stationary_target_cycles": self._stationary_target_cycles,
                "stationary_hold_active": self._stationary_hold_active,
            },
            "last_command": copy.deepcopy(self._last_command_diagnostics),
            "boundary": {
                "trajectory_lateral_axis": "left_positive",
                "trajectory_yaw_axis": "left_turn_positive",
                "carla_steer_axis": "right_positive",
                "gains_tuned_for_official_routes": False,
                "official_control_performance_claimed": False,
                "safety_shield_replaced": False,
            },
        }


def _strict_mapping(
    value: Any, *, name: str, required_keys: set[str]
) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    keys = set(value)
    missing = sorted(required_keys - keys)
    unexpected = sorted(keys - required_keys)
    if missing or unexpected:
        raise ValueError(
            f"{name} keys mismatch: missing={missing}, unexpected={unexpected}"
        )
    return value


def carla_pid_config_from_mapping(value: Any) -> CarlaPIDConfig:
    """Parse the versioned, fail-closed PID configuration contract."""
    record = _strict_mapping(
        value,
        name="pid_config",
        required_keys={"schema_version", "axes", "limits", "boundary"},
    )
    if record["schema_version"] != CARLA_PID_CONFIG_SCHEMA_VERSION:
        raise ValueError("pid_config schema_version mismatch")
    axes = _strict_mapping(
        record["axes"],
        name="pid_config.axes",
        required_keys={"speed", "lateral", "yaw"},
    )
    axis_keys = {"kp", "ki", "kd", "integral_limit", "derivative_filter"}

    def axis(name: str) -> PIDAxisConfig:
        item = _strict_mapping(
            axes[name],
            name=f"pid_config.axes.{name}",
            required_keys=axis_keys,
        )
        return PIDAxisConfig(
            **{
                key: _finite_float(item[key], f"pid_config.axes.{name}.{key}")
                for key in sorted(axis_keys)
            }
        )

    limit_keys = {
        "stop_speed_mps",
        "stationary_target_speed_mps",
        "stationary_confirm_cycles",
        "minimum_launch_throttle",
        "max_throttle",
        "max_brake",
        "max_abs_steer",
        "max_throttle_rate_per_sec",
        "max_brake_rate_per_sec",
        "max_steer_rate_per_sec",
        "max_abs_target_lateral_m",
        "max_abs_target_yaw_rad",
        "max_dt_sec",
    }
    # 新增可选键:缺省用默认值,旧配置仍可解析;未知键依旧拒绝
    optional_limit_keys = {
        "lateral_deadband_m",
        "steer_full_authority_speed_mps",
        "min_effective_max_steer",
    }
    limits_raw = dict(record["limits"])
    optional_limits = {
        key: _finite_float(
            limits_raw.pop(key),
            f"pid_config.limits.{key}",
        )
        for key in sorted(optional_limit_keys)
        if key in limits_raw
    }
    limits = _strict_mapping(
        limits_raw,
        name="pid_config.limits",
        required_keys=limit_keys,
    )
    boundary = _strict_mapping(
        record["boundary"],
        name="pid_config.boundary",
        required_keys={
            "engineering_baseline",
            "official_control_performance_claimed",
        },
    )
    if boundary["engineering_baseline"] is not True:
        raise ValueError("pid_config must declare engineering_baseline=true")
    if boundary["official_control_performance_claimed"] is not False:
        raise ValueError(
            "pid_config cannot claim official control performance"
        )
    numeric_limits = {
        key: _finite_float(limits[key], f"pid_config.limits.{key}")
        for key in sorted(limit_keys - {"stationary_confirm_cycles"})
    }
    stationary_confirm_cycles = limits["stationary_confirm_cycles"]
    if type(stationary_confirm_cycles) is not int:
        raise ValueError(
            "pid_config.limits.stationary_confirm_cycles must be an integer"
        )
    config = CarlaPIDConfig(
        speed=axis("speed"),
        lateral=axis("lateral"),
        yaw=axis("yaw"),
        stationary_confirm_cycles=stationary_confirm_cycles,
        **numeric_limits,
        **optional_limits,
    )
    CarlaTrajectoryPIDController(config)
    return config


def carla_pid_config_to_mapping(config: CarlaPIDConfig) -> Dict[str, Any]:
    """Serialize a validated PID configuration using the versioned schema."""
    controller = CarlaTrajectoryPIDController(config)
    values = asdict(controller.config)
    axes = {name: values.pop(name) for name in ("speed", "lateral", "yaw")}
    return {
        "schema_version": CARLA_PID_CONFIG_SCHEMA_VERSION,
        "axes": axes,
        "limits": values,
        "boundary": {
            "engineering_baseline": True,
            "official_control_performance_claimed": False,
        },
    }
