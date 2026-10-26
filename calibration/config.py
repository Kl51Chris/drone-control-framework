from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields
import json
import math
from pathlib import Path
from typing import Any


def _finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


@dataclass(slots=True)
class PIDSettings:
    kp: float = 0.25
    ki: float = 0.0
    kd: float = 0.08
    correction_min: float = -0.12
    correction_max: float = 0.12


@dataclass(slots=True)
class SafetySettings:
    min_thrust: float = 0.0
    max_thrust: float = 0.45
    max_state_age_s: float = 0.15
    max_loop_dt_s: float = 0.1
    max_tilt_rad: float = 0.45
    max_altitude_m: float = 1.5
    max_vertical_speed_m_s: float = 1.5
    max_horizontal_displacement_m: float = 0.50
    max_horizontal_speed_m_s: float = 0.75


@dataclass(slots=True)
class AttitudeSettings:
    roll_kp: float = 4.0
    pitch_kp: float = 4.0
    yaw_kp: float = 2.0
    roll_sign: int = -1
    # cflib.send_setpoint_manual() negates pitch before packing the packet.
    # +1 here yields negative feedback in the firmware's legacy pitch frame.
    pitch_sign: int = 1
    yaw_sign: int = -1
    max_rate_rad_s: float = 0.35


@dataclass(slots=True)
class ProfileSettings:
    takeoff_height_m: float = 0.35
    takeoff_rate_m_s: float = 0.15
    hover_duration_s: float = 5.0
    landing_rate_m_s: float = 0.12
    settle_duration_s: float = 1.0


@dataclass(slots=True)
class CalibrationConfig:
    schema_version: int = 1
    uri: str = "radio://0/80/2M/E7E7E7E7E7"
    control_frequency_hz: float = 50.0
    equilibrium_thrust: float = 0.30
    pid: PIDSettings = field(default_factory=PIDSettings)
    safety: SafetySettings = field(default_factory=SafetySettings)
    attitude: AttitudeSettings = field(default_factory=AttitudeSettings)
    profile: ProfileSettings = field(default_factory=ProfileSettings)
    external_altitude_udp_port: int | None = None
    output_directory: str = "calibration/runs"
    # Deliberate interlock: motor-free sign checks do not validate powered attitude hold.
    custom_attitude_hardware_validated: bool = False

    def validate(self) -> None:
        if self.schema_version != 1:
            raise ValueError("unsupported configuration schema_version")
        for section in (self, self.pid, self.safety, self.attitude, self.profile):
            for item in fields(section):
                name, value = item.name, getattr(section, item.name)
                if isinstance(value, float):
                    _finite(name, value)
        if self.control_frequency_hz <= 0:
            raise ValueError("control_frequency_hz must be positive")
        if not 0 <= self.safety.min_thrust <= self.equilibrium_thrust <= self.safety.max_thrust <= 1:
            raise ValueError("thrust must satisfy 0 <= min <= equilibrium <= max <= 1")
        if self.pid.correction_min > self.pid.correction_max:
            raise ValueError("PID correction_min must not exceed correction_max")
        if self.safety.max_state_age_s <= 0 or self.safety.max_loop_dt_s <= 0:
            raise ValueError("state age and loop dt limits must be positive")
        if (self.safety.max_tilt_rad <= 0 or self.safety.max_altitude_m <= 0 or
                self.safety.max_vertical_speed_m_s <= 0 or
                self.safety.max_horizontal_displacement_m <= 0 or
                self.safety.max_horizontal_speed_m_s <= 0):
            raise ValueError("state safety limits must be positive")
        if self.attitude.max_rate_rad_s <= 0:
            raise ValueError("attitude max rate must be positive")
        if self.profile.takeoff_height_m <= 0 or self.profile.takeoff_rate_m_s <= 0 or self.profile.landing_rate_m_s <= 0:
            raise ValueError("profile height and rates must be positive")
        if self.profile.hover_duration_s <= 0 or self.profile.settle_duration_s < 0:
            raise ValueError("hover must be positive and settle must be nonnegative")
        if self.attitude.roll_sign not in (-1, 1) or self.attitude.pitch_sign not in (-1, 1) or self.attitude.yaw_sign not in (-1, 1):
            raise ValueError("attitude signs must be -1 or 1")
        if self.external_altitude_udp_port is not None and not 1 <= self.external_altitude_udp_port <= 65535:
            raise ValueError("external_altitude_udp_port must be a valid UDP port")


def _construct(raw: dict[str, Any]) -> CalibrationConfig:
    data = dict(raw)
    data["pid"] = PIDSettings(**data.get("pid", {}))
    data["safety"] = SafetySettings(**data.get("safety", {}))
    data["attitude"] = AttitudeSettings(**data.get("attitude", {}))
    data["profile"] = ProfileSettings(**data.get("profile", {}))
    config = CalibrationConfig(**data)
    config.validate()
    return config


def load_config(path: str | Path) -> CalibrationConfig:
    with Path(path).open(encoding="utf-8") as stream:
        return _construct(json.load(stream))


def save_config(config: CalibrationConfig, path: str | Path) -> None:
    config.validate()
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as stream:
        json.dump(asdict(config), stream, indent=2, sort_keys=True)
        stream.write("\n")
