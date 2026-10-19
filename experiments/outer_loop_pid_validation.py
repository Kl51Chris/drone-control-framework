from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from enum import IntEnum
import math
from typing import Final

import numpy as np

from velocity_control_framework.controllers import (
    HorizontalCommandMapping,
    OuterLoopPIDConfig,
    OuterLoopPIDController,
    PIDConfig,
)
from velocity_control_framework.interfaces import (
    BodyRateThrustCommand,
    DroneState,
    Reference,
)
from velocity_control_framework.runner import (
    UniversalRunner,
    UniversalRunnerConfig,
)


MAX_POLARITY_OFFSET_M: Final = 0.05
MAX_POLARITY_RATE_RAD_S: Final = math.radians(10.0)
MAX_SLOW_TRAJECTORY_RADIUS_M: Final = 0.25
MIN_SLOW_TRAJECTORY_PERIOD_S: Final = 10.0


class ValidationStage(IntEnum):
    SOFTWARE_PIPELINE = 0
    HORIZONTAL_POLARITY = 1
    EQUILIBRIUM_THRUST = 2
    HORIZONTAL_STABILIZATION = 3
    HORIZONTAL_ALTITUDE = 4
    HEADING = 5
    FIXED_HOVER = 6
    SLOW_TRAJECTORY = 7


@dataclass(frozen=True, slots=True)
class PIDGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0

    @property
    def is_zero(self) -> bool:
        return self.kp == 0.0 and self.ki == 0.0 and self.kd == 0.0


@dataclass(frozen=True, slots=True)
class ValidationExperimentConfig:
    """All operator-selected settings for one validation run."""

    stage: ValidationStage
    fixed_position: tuple[float, float, float] | None = None
    fixed_yaw: float | None = None

    horizontal_mapping: HorizontalCommandMapping | None = None
    equilibrium_thrust: float | None = None
    min_thrust: float | None = None
    max_thrust: float | None = None
    horizontal_rate_limit_rad_s: float | None = None
    heading_rate_limit_rad_s: float | None = None
    altitude_feedback_limit: float | None = None
    roll_attitude_kp: float | None = None
    pitch_attitude_kp: float | None = None
    yaw_attitude_kp: float | None = None
    roll_attitude_to_roll_rate_sign: int | None = None
    pitch_attitude_to_pitch_rate_sign: int | None = None
    yaw_attitude_to_yaw_rate_sign: int | None = None
    attitude_rate_limit_rad_s: float | None = None

    horizontal_x_gains: PIDGains = field(default_factory=PIDGains)
    horizontal_y_gains: PIDGains = field(default_factory=PIDGains)
    altitude_gains: PIDGains = field(default_factory=PIDGains)
    heading_gains: PIDGains = field(default_factory=PIDGains)

    duration_s: float = 10.0
    control_frequency_hz: float = 50.0
    log_frequency_hz: float = 10.0
    state_wait_timeout_s: float = 5.0
    arming_wait_s: float = 1.0

    polarity_offset_m: float | None = None
    ramp_start_thrust: float | None = None
    ramp_duration_s: float | None = None
    hold_duration_s: float | None = None
    trajectory_radius_m: float | None = None
    trajectory_period_s: float | None = None

    # This must be deliberately changed for every physical-stage run.
    physical_test_authorized: bool = False

    def validate(self) -> None:
        """Reject placeholders and incomplete stage configurations."""
        self._validate_finite_values()

        if self.stage is ValidationStage.EQUILIBRIUM_THRUST:
            if not self.physical_test_authorized:
                raise ValueError(
                    "physical_test_authorized must be True for Stage 2"
                )
            required_values = {
                "roll_attitude_kp": self.roll_attitude_kp,
                "pitch_attitude_kp": self.pitch_attitude_kp,
                "yaw_attitude_kp": self.yaw_attitude_kp,
                "roll_attitude_to_roll_rate_sign": (
                    self.roll_attitude_to_roll_rate_sign
                ),
                "pitch_attitude_to_pitch_rate_sign": (
                    self.pitch_attitude_to_pitch_rate_sign
                ),
                "yaw_attitude_to_yaw_rate_sign": (
                    self.yaw_attitude_to_yaw_rate_sign
                ),
                "attitude_rate_limit_rad_s": self.attitude_rate_limit_rad_s,
                "heading_rate_limit_rad_s": self.heading_rate_limit_rad_s,
                "ramp_start_thrust": self.ramp_start_thrust,
                "equilibrium_thrust": self.equilibrium_thrust,
                "ramp_duration_s": self.ramp_duration_s,
                "hold_duration_s": self.hold_duration_s,
                "min_thrust": self.min_thrust,
                "max_thrust": self.max_thrust,
            }
            missing = [
                name for name, value in required_values.items() if value is None
            ]
            if missing:
                raise ValueError(
                    "Stage 2 requires explicit values for: " + ", ".join(missing)
                )
            assert self.roll_attitude_kp is not None
            assert self.pitch_attitude_kp is not None
            assert self.yaw_attitude_kp is not None
            assert self.roll_attitude_to_roll_rate_sign is not None
            assert self.pitch_attitude_to_pitch_rate_sign is not None
            assert self.yaw_attitude_to_yaw_rate_sign is not None
            assert self.attitude_rate_limit_rad_s is not None
            assert self.heading_rate_limit_rad_s is not None
            assert self.ramp_start_thrust is not None
            assert self.equilibrium_thrust is not None
            assert self.ramp_duration_s is not None
            assert self.hold_duration_s is not None
            assert self.min_thrust is not None
            assert self.max_thrust is not None
            for name, gain in (
                ("roll_attitude_kp", self.roll_attitude_kp),
                ("pitch_attitude_kp", self.pitch_attitude_kp),
                ("yaw_attitude_kp", self.yaw_attitude_kp),
            ):
                if gain <= 0.0:
                    raise ValueError(f"{name} must be positive")
            for name, sign in (
                (
                    "roll_attitude_to_roll_rate_sign",
                    self.roll_attitude_to_roll_rate_sign,
                ),
                (
                    "pitch_attitude_to_pitch_rate_sign",
                    self.pitch_attitude_to_pitch_rate_sign,
                ),
                (
                    "yaw_attitude_to_yaw_rate_sign",
                    self.yaw_attitude_to_yaw_rate_sign,
                ),
            ):
                if sign not in (-1, 1):
                    raise ValueError(f"{name} must be either -1 or 1")
            if self.attitude_rate_limit_rad_s <= 0.0:
                raise ValueError("attitude_rate_limit_rad_s must be positive")
            if self.attitude_rate_limit_rad_s > MAX_POLARITY_RATE_RAD_S:
                raise ValueError(
                    "Stage 2 attitude rate limit exceeds its safety cap"
                )
            if self.heading_rate_limit_rad_s <= 0.0:
                raise ValueError("heading_rate_limit_rad_s must be positive")
            if self.heading_rate_limit_rad_s > MAX_POLARITY_RATE_RAD_S:
                raise ValueError(
                    "Stage 2 heading rate limit exceeds its safety cap"
                )
            if self.control_frequency_hz <= 0.0:
                raise ValueError("control_frequency_hz must be positive")
            if not 0.0 <= self.min_thrust <= self.max_thrust <= 1.0:
                raise ValueError("thrust limits must satisfy 0 <= min <= max <= 1")
            if not self.min_thrust <= self.ramp_start_thrust <= self.max_thrust:
                raise ValueError("ramp_start_thrust must be within thrust limits")
            if not self.min_thrust <= self.equilibrium_thrust <= self.max_thrust:
                raise ValueError("equilibrium_thrust must be within thrust limits")
            if self.ramp_start_thrust >= self.equilibrium_thrust:
                raise ValueError(
                    "ramp_start_thrust must be less than equilibrium_thrust"
                )
            if self.ramp_duration_s <= 0.0:
                raise ValueError("ramp_duration_s must be positive")
            if self.hold_duration_s < 0.0:
                raise ValueError("hold_duration_s must be nonnegative")
            return

        if self.horizontal_mapping is None:
            raise ValueError("horizontal_mapping must be configured explicitly")

        if self.stage is ValidationStage.HORIZONTAL_POLARITY:
            if not self.physical_test_authorized:
                raise ValueError(
                    "physical_test_authorized must be True for Stage 1"
                )
            if self.horizontal_rate_limit_rad_s is None:
                raise ValueError(
                    "Stage 1 requires horizontal_rate_limit_rad_s"
                )
            if self.horizontal_rate_limit_rad_s <= 0.0:
                raise ValueError("horizontal_rate_limit_rad_s must be positive")
            if self.horizontal_rate_limit_rad_s > MAX_POLARITY_RATE_RAD_S:
                raise ValueError(
                    "Stage 1 horizontal rate limit exceeds its safety cap"
                )
            if self.polarity_offset_m is None or self.polarity_offset_m <= 0.0:
                raise ValueError("Stage 1 requires a positive polarity_offset_m")
            if self.polarity_offset_m > MAX_POLARITY_OFFSET_M:
                raise ValueError("Stage 1 polarity_offset_m exceeds its safety cap")
            self._require_p_only_horizontal()
            if not self.altitude_gains.is_zero or not self.heading_gains.is_zero:
                raise ValueError("Stage 1 permits only horizontal feedback")
            if self.equilibrium_thrust not in (None, 0.0):
                raise ValueError("Stage 1 requires zero equilibrium thrust")
            return

        if self.fixed_position is None:
            raise ValueError("fixed_position must be configured explicitly")
        if self.fixed_yaw is None:
            raise ValueError("fixed_yaw must be configured explicitly")

        required_values = {
            "equilibrium_thrust": self.equilibrium_thrust,
            "min_thrust": self.min_thrust,
            "max_thrust": self.max_thrust,
            "horizontal_rate_limit_rad_s": self.horizontal_rate_limit_rad_s,
            "heading_rate_limit_rad_s": self.heading_rate_limit_rad_s,
            "altitude_feedback_limit": self.altitude_feedback_limit,
        }
        missing = [name for name, value in required_values.items() if value is None]
        if missing:
            raise ValueError(
                "configure these values explicitly: " + ", ".join(missing)
            )

        assert self.equilibrium_thrust is not None
        assert self.min_thrust is not None
        assert self.max_thrust is not None
        assert self.horizontal_rate_limit_rad_s is not None
        assert self.heading_rate_limit_rad_s is not None
        assert self.altitude_feedback_limit is not None

        if not 0.0 <= self.min_thrust <= self.max_thrust <= 1.0:
            raise ValueError("thrust limits must satisfy 0 <= min <= max <= 1")
        if not self.min_thrust <= self.equilibrium_thrust <= self.max_thrust:
            raise ValueError("equilibrium_thrust must be within thrust limits")

        if self.stage is ValidationStage.SOFTWARE_PIPELINE:
            if not self._all_gains_zero():
                raise ValueError("Stage 0 requires all feedback gains to be zero")
            if self.equilibrium_thrust != 0.0:
                raise ValueError("Stage 0 requires equilibrium_thrust=0.0")
            return

        if self.horizontal_rate_limit_rad_s <= 0.0:
            raise ValueError("horizontal_rate_limit_rad_s must be positive")
        if self.heading_rate_limit_rad_s <= 0.0:
            raise ValueError("heading_rate_limit_rad_s must be positive")
        if self.altitude_feedback_limit <= 0.0:
            raise ValueError("altitude_feedback_limit must be positive")

        if not self.physical_test_authorized:
            raise ValueError(
                "physical_test_authorized must be True for physical stages"
            )

        if self.stage is ValidationStage.HORIZONTAL_STABILIZATION:
            self._require_p_only_horizontal()
            if not self.altitude_gains.is_zero or not self.heading_gains.is_zero:
                raise ValueError("Stage 3 permits only horizontal feedback")
            self._require_positive_equilibrium_thrust()

        elif self.stage is ValidationStage.HORIZONTAL_ALTITUDE:
            self._require_horizontal_and_altitude()
            if not self.heading_gains.is_zero:
                raise ValueError("Stage 4 requires heading gains to remain zero")

        elif self.stage >= ValidationStage.HEADING:
            self._require_horizontal_and_altitude()
            if self.heading_gains.kp <= 0.0:
                raise ValueError("Stages 5-7 require positive heading Kp")
            if self.stage is ValidationStage.HEADING and (
                self.heading_gains.ki != 0.0 or self.heading_gains.kd != 0.0
            ):
                raise ValueError("Stage 5 starts with P-only heading control")

        if self.stage is ValidationStage.SLOW_TRAJECTORY:
            if self.trajectory_radius_m is None or self.trajectory_radius_m <= 0.0:
                raise ValueError("Stage 7 requires positive trajectory_radius_m")
            if self.trajectory_period_s is None or self.trajectory_period_s <= 0.0:
                raise ValueError("Stage 7 requires positive trajectory_period_s")
            if self.trajectory_radius_m > MAX_SLOW_TRAJECTORY_RADIUS_M:
                raise ValueError("Stage 7 trajectory radius exceeds its safety cap")
            if self.trajectory_period_s < MIN_SLOW_TRAJECTORY_PERIOD_S:
                raise ValueError("Stage 7 trajectory period is too short")

    def controller_config(self) -> OuterLoopPIDConfig:
        self.validate()

        if self.stage is ValidationStage.EQUILIBRIUM_THRUST:
            # Stage 2 generates commands locally; this controller is not used.
            return OuterLoopPIDConfig(
                horizontal_mapping=HorizontalCommandMapping(
                    body_x_to_pitch_rate_sign=1,
                    body_y_to_roll_rate_sign=1,
                )
            )

        assert self.horizontal_mapping is not None

        if self.stage is ValidationStage.HORIZONTAL_POLARITY:
            assert self.horizontal_rate_limit_rad_s is not None
            return OuterLoopPIDConfig(
                horizontal_mapping=self.horizontal_mapping,
                horizontal_x=self._pid_config(
                    self.horizontal_x_gains,
                    self.horizontal_rate_limit_rad_s,
                ),
                horizontal_y=self._pid_config(
                    self.horizontal_y_gains,
                    self.horizontal_rate_limit_rad_s,
                ),
                altitude=PIDConfig(),
                heading=PIDConfig(),
                equilibrium_thrust=0.0,
                min_thrust=0.0,
                max_thrust=0.0,
            )

        assert self.equilibrium_thrust is not None
        assert self.min_thrust is not None
        assert self.max_thrust is not None
        assert self.horizontal_rate_limit_rad_s is not None
        assert self.heading_rate_limit_rad_s is not None
        assert self.altitude_feedback_limit is not None

        return OuterLoopPIDConfig(
            horizontal_mapping=self.horizontal_mapping,
            horizontal_x=self._pid_config(
                self.horizontal_x_gains,
                self.horizontal_rate_limit_rad_s,
            ),
            horizontal_y=self._pid_config(
                self.horizontal_y_gains,
                self.horizontal_rate_limit_rad_s,
            ),
            altitude=self._pid_config(
                self.altitude_gains,
                self.altitude_feedback_limit,
            ),
            heading=self._pid_config(
                self.heading_gains,
                self.heading_rate_limit_rad_s,
            ),
            equilibrium_thrust=self.equilibrium_thrust,
            min_thrust=self.min_thrust,
            max_thrust=self.max_thrust,
        )

    def runner_config(self) -> UniversalRunnerConfig:
        return UniversalRunnerConfig(
            control_frequency_hz=self.control_frequency_hz,
            log_frequency_hz=self.log_frequency_hz,
            state_wait_timeout_s=self.state_wait_timeout_s,
            arming_wait_s=self.arming_wait_s,
        )

    @staticmethod
    def _pid_config(gains: PIDGains, limit: float) -> PIDConfig:
        return PIDConfig(
            kp=gains.kp,
            ki=gains.ki,
            kd=gains.kd,
            output_min=-limit,
            output_max=limit,
        )

    def _validate_finite_values(self) -> None:
        numeric_values = {
            "duration_s": self.duration_s,
            "control_frequency_hz": self.control_frequency_hz,
            "log_frequency_hz": self.log_frequency_hz,
            "state_wait_timeout_s": self.state_wait_timeout_s,
            "arming_wait_s": self.arming_wait_s,
        }
        if self.fixed_position is not None:
            if len(self.fixed_position) != 3:
                raise ValueError("fixed_position must contain exactly three values")
            numeric_values.update(
                {
                    "fixed_x": self.fixed_position[0],
                    "fixed_y": self.fixed_position[1],
                    "fixed_z": self.fixed_position[2],
                }
            )
        if self.fixed_yaw is not None:
            numeric_values["fixed_yaw"] = self.fixed_yaw
        for name in (
            "equilibrium_thrust",
            "min_thrust",
            "max_thrust",
            "horizontal_rate_limit_rad_s",
            "heading_rate_limit_rad_s",
            "altitude_feedback_limit",
            "roll_attitude_kp",
            "pitch_attitude_kp",
            "yaw_attitude_kp",
            "attitude_rate_limit_rad_s",
            "polarity_offset_m",
            "ramp_start_thrust",
            "ramp_duration_s",
            "hold_duration_s",
            "trajectory_radius_m",
            "trajectory_period_s",
        ):
            value = getattr(self, name)
            if value is not None:
                numeric_values[name] = value
        for group_name, gains in (
            ("horizontal_x", self.horizontal_x_gains),
            ("horizontal_y", self.horizontal_y_gains),
            ("altitude", self.altitude_gains),
            ("heading", self.heading_gains),
        ):
            numeric_values.update(
                {
                    f"{group_name}.kp": gains.kp,
                    f"{group_name}.ki": gains.ki,
                    f"{group_name}.kd": gains.kd,
                }
            )
        for name, value in numeric_values.items():
            if not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
        if self.duration_s <= 0.0:
            raise ValueError("duration_s must be positive")

    def _all_gains_zero(self) -> bool:
        return all(
            gains.is_zero
            for gains in (
                self.horizontal_x_gains,
                self.horizontal_y_gains,
                self.altitude_gains,
                self.heading_gains,
            )
        )

    def _require_p_only_horizontal(self) -> None:
        for name, gains in (
            ("horizontal_x", self.horizontal_x_gains),
            ("horizontal_y", self.horizontal_y_gains),
        ):
            if gains.kp <= 0.0:
                raise ValueError(f"{name} Kp must be positive")
            if gains.ki != 0.0 or gains.kd != 0.0:
                raise ValueError(f"{name} must be P-only for this stage")

    def _require_positive_equilibrium_thrust(self) -> None:
        assert self.equilibrium_thrust is not None
        if self.equilibrium_thrust <= 0.0:
            raise ValueError("this stage requires positive equilibrium_thrust")

    def _require_horizontal_and_altitude(self) -> None:
        if self.horizontal_x_gains.kp <= 0.0:
            raise ValueError("horizontal_x Kp must be positive")
        if self.horizontal_y_gains.kp <= 0.0:
            raise ValueError("horizontal_y Kp must be positive")
        if self.altitude_gains.kp <= 0.0:
            raise ValueError("altitude Kp must be positive")
        self._require_positive_equilibrium_thrust()


class OuterLoopPIDValidationProgram:
    """Reference selection and controller invocation for validation stages."""

    def __init__(
        self,
        controller: OuterLoopPIDController,
        config: ValidationExperimentConfig,
    ) -> None:
        config.validate()
        self._controller = controller
        self._config = config
        self._start_time: float | None = None
        self._neutral_position: tuple[float, float, float] | None = None
        self._neutral_yaw: float | None = None

    def initialize(self, initial_state: DroneState, start_time: float) -> None:
        self._controller.reset()
        self._start_time = float(start_time)
        if self._config.stage in (
            ValidationStage.HORIZONTAL_POLARITY,
            ValidationStage.EQUILIBRIUM_THRUST,
        ):
            self._neutral_position = tuple(
                float(value) for value in initial_state.position
            )
            self._neutral_yaw = float(initial_state.yaw)
        else:
            assert self._config.fixed_position is not None
            assert self._config.fixed_yaw is not None
            self._neutral_position = self._config.fixed_position
            self._neutral_yaw = self._config.fixed_yaw

    def is_complete(self, current_time: float) -> bool:
        if self._config.stage is ValidationStage.EQUILIBRIUM_THRUST:
            assert self._config.ramp_duration_s is not None
            assert self._config.hold_duration_s is not None
            return self._elapsed(current_time) >= (
                self._config.ramp_duration_s + self._config.hold_duration_s
            )
        return self._elapsed(current_time) >= self._config.duration_s

    def get_phase_name(self, current_time: float) -> str:
        del current_time
        return f"STAGE_{int(self._config.stage)}_{self._config.stage.name}"

    def update(
        self,
        state: DroneState,
        current_time: float,
        dt: float,
    ) -> tuple[Reference, BodyRateThrustCommand]:
        reference = self._reference(current_time)
        if self._config.stage is ValidationStage.EQUILIBRIUM_THRUST:
            command = self._stage_two_command(state, current_time)
        else:
            command = self._controller.update(state, reference, dt)
        return reference, command

    def _stage_two_command(
        self,
        state: DroneState,
        current_time: float,
    ) -> BodyRateThrustCommand:
        assert self._neutral_yaw is not None
        assert self._config.roll_attitude_kp is not None
        assert self._config.pitch_attitude_kp is not None
        assert self._config.yaw_attitude_kp is not None
        assert self._config.roll_attitude_to_roll_rate_sign is not None
        assert self._config.pitch_attitude_to_pitch_rate_sign is not None
        assert self._config.yaw_attitude_to_yaw_rate_sign is not None
        assert self._config.attitude_rate_limit_rad_s is not None
        assert self._config.heading_rate_limit_rad_s is not None

        roll_rate = self._clamp_symmetric(
            self._config.roll_attitude_to_roll_rate_sign
            * self._config.roll_attitude_kp
            * (0.0 - state.roll),
            self._config.attitude_rate_limit_rad_s,
        )
        pitch_rate = self._clamp_symmetric(
            self._config.pitch_attitude_to_pitch_rate_sign
            * self._config.pitch_attitude_kp
            * (0.0 - state.pitch),
            self._config.attitude_rate_limit_rad_s,
        )
        yaw_error = math.atan2(
            math.sin(self._neutral_yaw - state.yaw),
            math.cos(self._neutral_yaw - state.yaw),
        )
        yaw_rate = self._clamp_symmetric(
            self._config.yaw_attitude_to_yaw_rate_sign
            * self._config.yaw_attitude_kp
            * yaw_error,
            self._config.heading_rate_limit_rad_s,
        )
        return BodyRateThrustCommand(
            roll_rate=roll_rate,
            pitch_rate=pitch_rate,
            yaw_rate=yaw_rate,
            thrust=self._stage_two_thrust(current_time),
        )

    @staticmethod
    def _clamp_symmetric(value: float, limit: float) -> float:
        return max(-limit, min(value, limit))

    def _stage_two_thrust(self, current_time: float) -> float:
        elapsed = self._elapsed(current_time)
        assert self._config.ramp_start_thrust is not None
        assert self._config.equilibrium_thrust is not None
        assert self._config.ramp_duration_s is not None
        assert self._config.min_thrust is not None
        assert self._config.max_thrust is not None

        if elapsed < self._config.ramp_duration_s:
            alpha = elapsed / self._config.ramp_duration_s
            thrust = self._config.ramp_start_thrust + alpha * (
                self._config.equilibrium_thrust
                - self._config.ramp_start_thrust
            )
        else:
            thrust = self._config.equilibrium_thrust

        return max(
            self._config.min_thrust,
            min(thrust, self._config.max_thrust),
        )

    def _reference(self, current_time: float) -> Reference:
        elapsed = self._elapsed(current_time)
        if self._neutral_position is None or self._neutral_yaw is None:
            raise RuntimeError("flight program has not been initialized")
        x_ref, y_ref, z_ref = self._neutral_position

        if self._config.stage is ValidationStage.HORIZONTAL_POLARITY:
            x_body, y_body = self._polarity_offset(elapsed)
            cos_yaw = math.cos(self._neutral_yaw)
            sin_yaw = math.sin(self._neutral_yaw)
            x_ref += cos_yaw * x_body - sin_yaw * y_body
            y_ref += sin_yaw * x_body + cos_yaw * y_body

        elif self._config.stage is ValidationStage.SLOW_TRAJECTORY:
            assert self._config.trajectory_radius_m is not None
            assert self._config.trajectory_period_s is not None
            angle = 2.0 * math.pi * min(
                elapsed / self._config.trajectory_period_s,
                1.0,
            )
            radius = self._config.trajectory_radius_m
            # The offset circle begins exactly at the fixed reference.
            x_ref += radius * (math.cos(angle) - 1.0)
            y_ref += radius * math.sin(angle)

        return Reference(
            position=np.array([x_ref, y_ref, z_ref], dtype=np.float64),
            velocity=np.zeros(3, dtype=np.float64),
            yaw=self._neutral_yaw,
            yaw_rate=0.0,
            timestamp=elapsed,
        )

    def _polarity_offset(self, elapsed: float) -> tuple[float, float]:
        assert self._config.polarity_offset_m is not None
        segment = min(int(5.0 * elapsed / self._config.duration_s), 4)
        offset = self._config.polarity_offset_m
        return (
            (offset, 0.0),
            (-offset, 0.0),
            (0.0, offset),
            (0.0, -offset),
            (0.0, 0.0),
        )[segment]

    def _elapsed(self, current_time: float) -> float:
        if self._start_time is None:
            raise RuntimeError("flight program has not been initialized")
        return max(0.0, float(current_time) - self._start_time)


URI: Final = "radio://0/80/2M"

# Intentionally no runnable physical defaults. Replace None with a reviewed
# ValidationExperimentConfig before running this module.
EXPERIMENT_CONFIG = ValidationExperimentConfig(
    stage=ValidationStage.EQUILIBRIUM_THRUST,

    roll_attitude_to_roll_rate_sign=1,
    pitch_attitude_to_pitch_rate_sign=1,
    yaw_attitude_to_yaw_rate_sign=1,

    roll_attitude_kp=1.0,
    pitch_attitude_kp=1.0,
    yaw_attitude_kp=0.2,

    attitude_rate_limit_rad_s=0.1,
    heading_rate_limit_rad_s=0.03,

    ramp_start_thrust=0.30,
    equilibrium_thrust=0.38,
    ramp_duration_s=3.0,
    hold_duration_s=0.5,

    min_thrust=0.0,
    max_thrust=0.42,

    control_frequency_hz=50.0,
    physical_test_authorized=True,
)

def build_stage_zero_config() -> ValidationExperimentConfig:
    """Build the non-flying, zero-output command-pipeline configuration."""
    return ValidationExperimentConfig(
        stage=ValidationStage.SOFTWARE_PIPELINE,
        fixed_position=(0.0, 0.0, 0.0),
        fixed_yaw=0.0,
        # The mapping is non-operative because all gains and limits are zero.
        # It makes no claim about the physical vehicle polarities.
        horizontal_mapping=HorizontalCommandMapping(
            body_x_to_pitch_rate_sign=1,
            body_y_to_roll_rate_sign=1,
        ),
        equilibrium_thrust=0.0,
        min_thrust=0.0,
        max_thrust=0.0,
        horizontal_rate_limit_rad_s=0.0,
        heading_rate_limit_rad_s=0.0,
        altitude_feedback_limit=0.0,
    )


def parse_args(
    argv: list[str] | None = None,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run one staged OuterLoopPIDController validation experiment. "
            "Stages 1-7 require a reviewed EXPERIMENT_CONFIG."
        )
    )
    parser.add_argument(
        "--stage",
        type=int,
        choices=range(8),
        required=True,
        help="validation stage to run (0 is the zero-output pipeline test)",
    )
    return parser.parse_args(argv)


def select_experiment_config(
    stage: ValidationStage,
) -> ValidationExperimentConfig:
    """Select the safe Stage 0 path or a reviewed physical configuration."""
    if stage is ValidationStage.SOFTWARE_PIPELINE:
        config = build_stage_zero_config()
    else:
        if EXPERIMENT_CONFIG is None:
            if stage is ValidationStage.HORIZONTAL_POLARITY:
                required = (
                    "horizontal P gains, horizontal mapping signs, horizontal "
                    "rate limit, polarity offset, duration/runner frequency, "
                    "and physical_test_authorized=True"
                )
            elif stage is ValidationStage.EQUILIBRIUM_THRUST:
                required = (
                    "roll/pitch/yaw attitude P gains, roll/pitch and yaw rate "
                    "limits, ramp start thrust, equilibrium thrust, ramp/hold "
                    "durations, thrust limits, control frequency, and "
                    "physical_test_authorized=True"
                )
            else:
                required = (
                    "reviewed PID gains, horizontal mapping signs, equilibrium "
                    "thrust, thrust limits, body-rate limits, fixed reference, "
                    "and physical_test_authorized=True"
                )
            raise ValueError(
                f"Stage {int(stage)} requires EXPERIMENT_CONFIG with {required}"
            )
        if EXPERIMENT_CONFIG.stage is not stage:
            raise ValueError(
                "--stage does not match EXPERIMENT_CONFIG.stage "
                f"({int(EXPERIMENT_CONFIG.stage)})"
            )
        config = EXPERIMENT_CONFIG

    config.validate()
    return config


def log_step(
    elapsed: float,
    phase: str,
    state: DroneState,
    reference: Reference,
    command: BodyRateThrustCommand,
    dt: float,
) -> None:
    attitude_text = ""
    if phase.startswith("STAGE_2_"):
        attitude_text = (
            f" attitude=({state.roll:+.3f},{state.pitch:+.3f},"
            f"{state.yaw:+.3f})rad"
        )
    print(
        f"[{elapsed:7.2f}s] [{phase}] dt={dt * 1000.0:5.1f}ms"
        f" pos=({state.position[0]:+.3f},{state.position[1]:+.3f},"
        f"{state.position[2]:+.3f})"
        f" ref=({reference.position[0]:+.3f},{reference.position[1]:+.3f},"
        f"{reference.position[2]:+.3f})"
        f"{attitude_text}"
        f" rates=({command.roll_rate:+.3f},{command.pitch_rate:+.3f},"
        f"{command.yaw_rate:+.3f})rad/s thrust={command.thrust:.4f}"
    )


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    stage = ValidationStage(args.stage)

    try:
        config = select_experiment_config(stage)
    except ValueError as error:
        raise SystemExit(f"configuration error: {error}") from error

    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    from velocity_control_framework.backends import (
        CrazyflieCommandAdapter,
        CrazyflieStateProvider,
    )

    cflib.crtp.init_drivers()
    crazyflie = Crazyflie(rw_cache="./cache")

    with SyncCrazyflie(URI, cf=crazyflie) as sync_crazyflie:
        cf = sync_crazyflie.cf
        controller = OuterLoopPIDController(config.controller_config())
        program = OuterLoopPIDValidationProgram(controller, config)
        runner = UniversalRunner[
            DroneState,
            Reference,
            BodyRateThrustCommand,
        ](
            state_provider=CrazyflieStateProvider(cf),
            command_adapter=CrazyflieCommandAdapter(cf),
            flight_program=program,
            arm=lambda: cf.supervisor.send_arming_request(True),
            disarm=lambda: cf.supervisor.send_arming_request(False),
            config=config.runner_config(),
            log_callback=log_step,
        )
        runner.run()


if __name__ == "__main__":
    main()
