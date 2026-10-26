from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
from typing import Any

from velocity_control_framework.interfaces import BodyRateThrustCommand

from .state import AltitudeState


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(slots=True)
class AltitudeLoopConfig:
    position_kp: float = 0.10
    velocity_kp: float = 0.0
    integral_ki: float = 0.0
    correction_min: float = -0.045
    correction_max: float = 0.025
    velocity_feedback_enabled: bool = False
    velocity_reference_tracking_enabled: bool = True
    integral_enabled: bool = False


@dataclass(slots=True)
class AttitudeLoopConfig:
    roll_kp: float = 4.0
    pitch_kp: float = 4.0
    yaw_kp: float = 2.0
    # Command signs account for the verified cflib legacy pitch packing behavior.
    roll_command_sign: int = 1
    pitch_command_sign: int = -1
    yaw_command_sign: int = 1
    max_roll_rate_rad_s: float = 0.35
    max_pitch_rate_rad_s: float = 0.35
    max_yaw_rate_rad_s: float = 0.35


@dataclass(slots=True)
class HorizontalLoopConfig:
    enabled: bool = False
    position_kp: float = 0.0
    velocity_kp: float = 0.0
    max_acceleration_m_s2: float = 0.0
    max_tilt_reference_rad: float = 0.0
    gravity_m_s2: float = 9.80665
    body_x_acceleration_to_pitch_sign: int = 1
    body_y_acceleration_to_roll_sign: int = -1


@dataclass(slots=True)
class CascadedPIDConfig:
    schema_version: int = 1
    stage: str = "5B"
    equilibrium_thrust: float = 0.425
    min_thrust: float = 0.380
    max_thrust: float = 0.450
    altitude: AltitudeLoopConfig = field(default_factory=AltitudeLoopConfig)
    attitude: AttitudeLoopConfig = field(default_factory=AttitudeLoopConfig)
    horizontal: HorizontalLoopConfig = field(default_factory=HorizontalLoopConfig)
    hardware_flight_authorized: bool = False
    powered_attitude_validated: bool = False
    horizontal_mapping_hardware_validated: bool = False

    def validate(self) -> None:
        if self.schema_version != 1:
            raise ValueError("unsupported cascaded PID schema_version")
        if self.stage not in {"5B", "5C", "5D"}:
            raise ValueError("stage must be 5B, 5C, or 5D")
        for value in _float_values(asdict(self)):
            if not math.isfinite(value):
                raise ValueError("all cascaded PID numeric settings must be finite")
        if not 0 <= self.min_thrust <= self.equilibrium_thrust <= self.max_thrust <= 1:
            raise ValueError("thrust must satisfy 0 <= min <= equilibrium <= max <= 1")
        if self.altitude.correction_min > self.altitude.correction_max:
            raise ValueError("altitude correction_min must not exceed correction_max")
        if min(self.altitude.position_kp, self.altitude.velocity_kp, self.altitude.integral_ki) < 0:
            raise ValueError("altitude gains must be nonnegative")
        if not isinstance(self.altitude.velocity_reference_tracking_enabled, bool):
            raise ValueError("velocity_reference_tracking_enabled must be boolean")
        effective_min = max(self.altitude.correction_min, self.min_thrust-self.equilibrium_thrust)
        effective_max = min(self.altitude.correction_max, self.max_thrust-self.equilibrium_thrust)
        if effective_min > effective_max:
            raise ValueError("altitude correction and thrust bounds do not overlap")
        if min(self.attitude.roll_kp, self.attitude.pitch_kp, self.attitude.yaw_kp) < 0:
            raise ValueError("attitude gains must be nonnegative")
        if min(self.attitude.max_roll_rate_rad_s, self.attitude.max_pitch_rate_rad_s,
               self.attitude.max_yaw_rate_rad_s) <= 0:
            raise ValueError("attitude rate limits must be positive")
        signs = (self.attitude.roll_command_sign, self.attitude.pitch_command_sign,
                 self.attitude.yaw_command_sign)
        if any(sign not in (-1, 1) for sign in signs):
            raise ValueError("attitude command signs must be -1 or 1")
        if self.horizontal.enabled:
            if min(self.horizontal.position_kp, self.horizontal.velocity_kp) < 0:
                raise ValueError("horizontal gains must be nonnegative")
            if self.horizontal.max_acceleration_m_s2 <= 0 or self.horizontal.max_tilt_reference_rad <= 0:
                raise ValueError("enabled horizontal loop requires positive acceleration and tilt limits")
            if not self.horizontal.max_tilt_reference_rad < math.pi / 2:
                raise ValueError("horizontal tilt reference limit must be below pi/2")
            if self.horizontal.gravity_m_s2 <= 0:
                raise ValueError("gravity must be positive")
            if (self.horizontal.body_x_acceleration_to_pitch_sign not in (-1, 1) or
                    self.horizontal.body_y_acceleration_to_roll_sign not in (-1, 1)):
                raise ValueError("horizontal acceleration-to-attitude signs must be -1 or 1")
        if (not isinstance(self.hardware_flight_authorized, bool) or
                not isinstance(self.powered_attitude_validated, bool) or
                not isinstance(self.horizontal_mapping_hardware_validated, bool)):
            raise ValueError("hardware interlocks must be booleans")
        if self.stage == "5B" and (self.altitude.velocity_feedback_enabled or
                                   self.altitude.integral_enabled or self.horizontal.enabled):
            raise ValueError("Stage 5B permits altitude P and attitude/heading loops only")
        if self.stage == "5C" and self.horizontal.enabled:
            raise ValueError("Stage 5C does not enable horizontal position control")


def _float_values(value: Any):
    if isinstance(value, dict):
        for child in value.values():
            yield from _float_values(child)
    elif isinstance(value, float):
        yield value


@dataclass(frozen=True, slots=True)
class CascadedReference:
    altitude_m: float
    vertical_velocity_m_s: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    position_x_m: float = 0.0
    position_y_m: float = 0.0
    velocity_x_m_s: float = 0.0
    velocity_y_m_s: float = 0.0


@dataclass(frozen=True, slots=True)
class CascadedPIDResult:
    command: BodyRateThrustCommand
    contributions: dict[str, float | int]


class CascadedPIDController:
    """Configurable world-position -> attitude -> body-rate/thrust controller.

    The output intentionally stops at BodyRateThrustCommand. Crazyflie firmware
    retains its body-rate PID and motor mixer.
    """

    def __init__(self, config: CascadedPIDConfig) -> None:
        config.validate()
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.altitude_integral_state = 0.0

    def update(self, state: AltitudeState, reference: CascadedReference,
               dt_s: float) -> CascadedPIDResult:
        numeric = [dt_s, *[getattr(state, name) for name in (
            "altitude_m", "vertical_velocity_m_s", "roll_rad", "pitch_rad",
            "yaw_rad", "position_x_m", "position_y_m", "velocity_x_m_s",
            "velocity_y_m_s")], *[getattr(reference, name) for name in (
            "altitude_m", "vertical_velocity_m_s", "roll_rad", "pitch_rad",
            "yaw_rad", "position_x_m", "position_y_m", "velocity_x_m_s",
            "velocity_y_m_s")]]
        if not all(math.isfinite(value) for value in numeric) or dt_s <= 0:
            raise ValueError("controller inputs and positive dt_s must be finite")

        cfg = self.config
        altitude_error = reference.altitude_m - state.altitude_m
        velocity_reference = (reference.vertical_velocity_m_s
                              if cfg.altitude.velocity_reference_tracking_enabled else 0.0)
        vertical_velocity_error = velocity_reference - state.vertical_velocity_m_s
        altitude_p = cfg.altitude.position_kp * altitude_error
        altitude_v = (cfg.altitude.velocity_kp * vertical_velocity_error
                      if cfg.altitude.velocity_feedback_enabled else 0.0)
        candidate_integral = self.altitude_integral_state + altitude_error * dt_s
        altitude_i = (cfg.altitude.integral_ki * candidate_integral
                      if cfg.altitude.integral_enabled else 0.0)
        correction_raw = altitude_p + altitude_v + altitude_i
        effective_min = max(cfg.altitude.correction_min, cfg.min_thrust-cfg.equilibrium_thrust)
        effective_max = min(cfg.altitude.correction_max, cfg.max_thrust-cfg.equilibrium_thrust)
        correction = _clamp(correction_raw, effective_min, effective_max)
        integral_drive = cfg.altitude.integral_ki * altitude_error
        accept_integral = (correction_raw == correction or
                           (correction_raw > effective_max and integral_drive < 0) or
                           (correction_raw < effective_min and integral_drive > 0))
        if cfg.altitude.integral_enabled and cfg.altitude.integral_ki != 0 and accept_integral:
            self.altitude_integral_state = candidate_integral
        thrust = cfg.equilibrium_thrust + correction

        desired_roll, desired_pitch, horizontal_terms = (
            self._attitude_references(state, reference))
        roll_error = desired_roll - state.roll_rad
        pitch_error = desired_pitch - state.pitch_rad
        yaw_error = _wrap(reference.yaw_rad - state.yaw_rad)
        attitude = cfg.attitude
        roll_raw = attitude.roll_command_sign * attitude.roll_kp * roll_error
        pitch_raw = attitude.pitch_command_sign * attitude.pitch_kp * pitch_error
        yaw_raw = attitude.yaw_command_sign * attitude.yaw_kp * yaw_error
        roll_rate = _clamp(roll_raw, -attitude.max_roll_rate_rad_s, attitude.max_roll_rate_rad_s)
        pitch_rate = _clamp(pitch_raw, -attitude.max_pitch_rate_rad_s, attitude.max_pitch_rate_rad_s)
        yaw_rate = _clamp(yaw_raw, -attitude.max_yaw_rate_rad_s, attitude.max_yaw_rate_rad_s)
        command = BodyRateThrustCommand(roll_rate, pitch_rate, yaw_rate, thrust)
        return CascadedPIDResult(command, {
            "altitude_error_m": altitude_error,
            "altitude_velocity_reference_m_s": velocity_reference,
            "vertical_velocity_error_m_s": vertical_velocity_error,
            "altitude_p": altitude_p, "altitude_velocity": altitude_v,
            "altitude_integral": altitude_i, "altitude_correction_raw": correction_raw,
            "altitude_correction": correction,
            "altitude_saturated": int(correction != correction_raw),
            "integral_update_accepted": int(accept_integral),
            "desired_roll_rad": desired_roll, "desired_pitch_rad": desired_pitch,
            "desired_yaw_rad": reference.yaw_rad,
            **horizontal_terms,
            "roll_error_rad": roll_error, "pitch_error_rad": pitch_error,
            "yaw_error_rad": yaw_error,
            "roll_rate_raw_rad_s": roll_raw, "pitch_rate_raw_rad_s": pitch_raw,
            "yaw_rate_raw_rad_s": yaw_raw,
            "roll_rate_saturated": int(roll_rate != roll_raw),
            "pitch_rate_saturated": int(pitch_rate != pitch_raw),
            "yaw_rate_saturated": int(yaw_rate != yaw_raw),
        })

    def _attitude_references(self, state: AltitudeState, reference: CascadedReference
                             ) -> tuple[float, float, dict[str, float | int]]:
        horizontal = self.config.horizontal
        if not horizontal.enabled:
            return reference.roll_rad, reference.pitch_rad, {
                name: 0.0 for name in (
                    "horizontal_position_error_x_m", "horizontal_position_error_y_m",
                    "horizontal_velocity_error_x_m_s", "horizontal_velocity_error_y_m_s",
                    "horizontal_position_x_contribution_m_s2", "horizontal_position_y_contribution_m_s2",
                    "horizontal_velocity_x_contribution_m_s2", "horizontal_velocity_y_contribution_m_s2",
                    "horizontal_acceleration_x_world_raw_m_s2", "horizontal_acceleration_y_world_raw_m_s2",
                    "horizontal_acceleration_x_world_m_s2", "horizontal_acceleration_y_world_m_s2",
                    "horizontal_acceleration_x_body_m_s2", "horizontal_acceleration_y_body_m_s2")
            } | {"horizontal_acceleration_saturated": 0, "tilt_reference_saturated": 0}
        ex = reference.position_x_m-state.position_x_m
        ey = reference.position_y_m-state.position_y_m
        evx = reference.velocity_x_m_s-state.velocity_x_m_s
        evy = reference.velocity_y_m_s-state.velocity_y_m_s
        position_x = horizontal.position_kp*ex; position_y = horizontal.position_kp*ey
        velocity_x = horizontal.velocity_kp*evx; velocity_y = horizontal.velocity_kp*evy
        ax_raw, ay_raw = position_x+velocity_x, position_y+velocity_y
        ax_world, ay_world = ax_raw, ay_raw
        magnitude = math.hypot(ax_raw, ay_raw)
        acceleration_saturated = magnitude > horizontal.max_acceleration_m_s2
        if magnitude > horizontal.max_acceleration_m_s2:
            scale = horizontal.max_acceleration_m_s2 / magnitude
            ax_world *= scale; ay_world *= scale
        cosine, sine = math.cos(state.yaw_rad), math.sin(state.yaw_rad)
        ax_body = cosine*ax_world + sine*ay_world
        ay_body = -sine*ax_world + cosine*ay_world
        desired_pitch = horizontal.body_x_acceleration_to_pitch_sign * math.atan2(ax_body, horizontal.gravity_m_s2)
        desired_roll = horizontal.body_y_acceleration_to_roll_sign * math.atan2(ay_body, horizontal.gravity_m_s2)
        tilt = math.hypot(desired_roll, desired_pitch)
        tilt_saturated = tilt > horizontal.max_tilt_reference_rad
        if tilt > horizontal.max_tilt_reference_rad:
            scale = horizontal.max_tilt_reference_rad / tilt
            desired_roll *= scale; desired_pitch *= scale
        return desired_roll, desired_pitch, {
            "horizontal_position_error_x_m": ex, "horizontal_position_error_y_m": ey,
            "horizontal_velocity_error_x_m_s": evx, "horizontal_velocity_error_y_m_s": evy,
            "horizontal_position_x_contribution_m_s2": position_x,
            "horizontal_position_y_contribution_m_s2": position_y,
            "horizontal_velocity_x_contribution_m_s2": velocity_x,
            "horizontal_velocity_y_contribution_m_s2": velocity_y,
            "horizontal_acceleration_x_world_raw_m_s2": ax_raw,
            "horizontal_acceleration_y_world_raw_m_s2": ay_raw,
            "horizontal_acceleration_x_world_m_s2": ax_world,
            "horizontal_acceleration_y_world_m_s2": ay_world,
            "horizontal_acceleration_x_body_m_s2": ax_body,
            "horizontal_acceleration_y_body_m_s2": ay_body,
            "horizontal_acceleration_saturated": int(acceleration_saturated),
            "tilt_reference_saturated": int(tilt_saturated),
        }


def load_cascaded_config(path: str | Path) -> CascadedPIDConfig:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    raw["altitude"] = AltitudeLoopConfig(**raw.get("altitude", {}))
    raw["attitude"] = AttitudeLoopConfig(**raw.get("attitude", {}))
    raw["horizontal"] = HorizontalLoopConfig(**raw.get("horizontal", {}))
    config = CascadedPIDConfig(**raw)
    config.validate()
    return config
