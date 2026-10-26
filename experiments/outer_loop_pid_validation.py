from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from enum import Enum, IntEnum
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
MAX_POLARITY_RATE_RAD_S: Final = math.radians(20.0)
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


class StageSixPhase(Enum):
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER"
    LANDING = "LANDING"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True, slots=True)
class StageSixCommandDiagnostics:
    """Stage 6 command components retained for unambiguous logging."""

    position_pid_roll_rate: float
    position_pid_pitch_rate: float
    heading_pid_yaw_rate: float
    roll_attitude_feedback: float
    pitch_attitude_feedback: float
    final_roll_rate: float
    final_pitch_rate: float
    final_yaw_rate: float
    roll_saturated: bool
    pitch_saturated: bool


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

    # Stage 6 values have no flight-ready defaults. The hover height is
    # relative to the altitude measured during initialization; landing returns
    # to that initial altitude.
    hover_height_m: float | None = None
    takeoff_velocity_m_s: float | None = None
    landing_velocity_m_s: float | None = None
    hover_duration_s: float | None = None
    landing_altitude_tolerance_m: float | None = None
    landing_vertical_velocity_tolerance_m_s: float | None = None
    max_horizontal_error_m: float | None = None
    max_altitude_error_m: float | None = None
    max_tilt_rad: float | None = None
    maximum_flight_duration_s: float | None = None
    max_state_age_s: float | None = None
    max_control_loop_dt_s: float | None = None

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

        if self.stage is not ValidationStage.FIXED_HOVER:
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

        if self.stage is ValidationStage.FIXED_HOVER:
            self._validate_stage_six()

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
            "hover_height_m",
            "takeoff_velocity_m_s",
            "landing_velocity_m_s",
            "hover_duration_s",
            "landing_altitude_tolerance_m",
            "landing_vertical_velocity_tolerance_m_s",
            "max_horizontal_error_m",
            "max_altitude_error_m",
            "max_tilt_rad",
            "maximum_flight_duration_s",
            "max_state_age_s",
            "max_control_loop_dt_s",
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

    def _validate_stage_six(self) -> None:
        required_values = {
            "roll_attitude_kp": self.roll_attitude_kp,
            "pitch_attitude_kp": self.pitch_attitude_kp,
            "roll_attitude_to_roll_rate_sign": (
                self.roll_attitude_to_roll_rate_sign
            ),
            "pitch_attitude_to_pitch_rate_sign": (
                self.pitch_attitude_to_pitch_rate_sign
            ),
            "attitude_rate_limit_rad_s": self.attitude_rate_limit_rad_s,
            "hover_height_m": self.hover_height_m,
            "takeoff_velocity_m_s": self.takeoff_velocity_m_s,
            "landing_velocity_m_s": self.landing_velocity_m_s,
            "hover_duration_s": self.hover_duration_s,
            "landing_altitude_tolerance_m": (
                self.landing_altitude_tolerance_m
            ),
            "landing_vertical_velocity_tolerance_m_s": (
                self.landing_vertical_velocity_tolerance_m_s
            ),
            "max_horizontal_error_m": self.max_horizontal_error_m,
            "max_altitude_error_m": self.max_altitude_error_m,
            "max_tilt_rad": self.max_tilt_rad,
            "maximum_flight_duration_s": self.maximum_flight_duration_s,
            "max_state_age_s": self.max_state_age_s,
            "max_control_loop_dt_s": self.max_control_loop_dt_s,
        }
        missing = [
            name for name, value in required_values.items() if value is None
        ]
        if missing:
            raise ValueError(
                "Stage 6 requires explicit values for: " + ", ".join(missing)
            )

        assert self.roll_attitude_kp is not None
        assert self.pitch_attitude_kp is not None
        assert self.roll_attitude_to_roll_rate_sign is not None
        assert self.pitch_attitude_to_pitch_rate_sign is not None
        assert self.attitude_rate_limit_rad_s is not None
        assert self.hover_height_m is not None
        assert self.takeoff_velocity_m_s is not None
        assert self.landing_velocity_m_s is not None
        assert self.hover_duration_s is not None
        assert self.landing_altitude_tolerance_m is not None
        assert self.landing_vertical_velocity_tolerance_m_s is not None
        assert self.max_horizontal_error_m is not None
        assert self.max_altitude_error_m is not None
        assert self.max_tilt_rad is not None
        assert self.maximum_flight_duration_s is not None
        assert self.max_state_age_s is not None
        assert self.max_control_loop_dt_s is not None

        for name, gain in (
            ("roll_attitude_kp", self.roll_attitude_kp),
            ("pitch_attitude_kp", self.pitch_attitude_kp),
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
        ):
            if sign not in (-1, 1):
                raise ValueError(f"{name} must be either -1 or 1")

        positive_values = {
            "attitude_rate_limit_rad_s": self.attitude_rate_limit_rad_s,
            "hover_height_m": self.hover_height_m,
            "takeoff_velocity_m_s": self.takeoff_velocity_m_s,
            "landing_velocity_m_s": self.landing_velocity_m_s,
            "hover_duration_s": self.hover_duration_s,
            "landing_altitude_tolerance_m": (
                self.landing_altitude_tolerance_m
            ),
            "landing_vertical_velocity_tolerance_m_s": (
                self.landing_vertical_velocity_tolerance_m_s
            ),
            "max_horizontal_error_m": self.max_horizontal_error_m,
            "max_altitude_error_m": self.max_altitude_error_m,
            "max_tilt_rad": self.max_tilt_rad,
            "maximum_flight_duration_s": self.maximum_flight_duration_s,
            "max_state_age_s": self.max_state_age_s,
            "max_control_loop_dt_s": self.max_control_loop_dt_s,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")

        if self.attitude_rate_limit_rad_s > MAX_POLARITY_RATE_RAD_S:
            raise ValueError("Stage 6 attitude rate limit exceeds its safety cap")
        if self.max_tilt_rad >= math.pi / 2.0:
            raise ValueError("max_tilt_rad must be less than pi/2")
        if self.landing_altitude_tolerance_m >= self.hover_height_m:
            raise ValueError(
                "landing_altitude_tolerance_m must be less than hover_height_m"
            )

        nominal_duration = (
            1.5 * self.hover_height_m / self.takeoff_velocity_m_s
            + self.hover_duration_s
            + 1.5 * self.hover_height_m / self.landing_velocity_m_s
        )
        if self.maximum_flight_duration_s <= nominal_duration:
            raise ValueError(
                "maximum_flight_duration_s must exceed the nominal Stage 6 "
                "trajectory duration"
            )


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
        self._stage_six_phase: StageSixPhase | None = None
        self._stage_six_landing_altitude: float | None = None
        self._stage_six_diagnostics: StageSixCommandDiagnostics | None = None
        self._stage_six_first_tracking_validation = True

    @property
    def stage_six_diagnostics(self) -> StageSixCommandDiagnostics | None:
        return self._stage_six_diagnostics

    def report_initialization_states(
        self,
        *,
        pre_arming_state: DroneState,
        post_arming_state: DroneState,
    ) -> None:
        """Report and validate the Stage 6 post-arming state transition."""
        if self._config.stage is not ValidationStage.FIXED_HOVER:
            return
        if post_arming_state.timestamp <= pre_arming_state.timestamp:
            raise RuntimeError(
                "Stage 6 did not receive a newer state after the arming wait: "
                f"pre_timestamp={pre_arming_state.timestamp:.6f}s, "
                f"post_timestamp={post_arming_state.timestamp:.6f}s"
            )
        position_delta = post_arming_state.position - pre_arming_state.position
        print(
            "[STAGE 6 INIT]"
            f" pre_arming_position={self._format_vector(pre_arming_state.position)}m"
            f" pre_arming_timestamp={pre_arming_state.timestamp:.6f}s"
            f" post_arming_position={self._format_vector(post_arming_state.position)}m"
            f" post_arming_timestamp={post_arming_state.timestamp:.6f}s"
            f" arming_wait_position_delta={self._format_vector(position_delta)}m"
        )

    def select_initialization_state(
        self,
        *,
        pre_arming_state: DroneState,
        post_arming_state: DroneState,
    ) -> DroneState:
        """Use the refreshed state only for Stage 6 initialization."""
        if self._config.stage is ValidationStage.FIXED_HOVER:
            return post_arming_state
        return pre_arming_state

    def initialize(self, initial_state: DroneState, start_time: float) -> None:
        self._controller.reset()
        self._start_time = float(start_time)
        self._stage_six_diagnostics = None
        self._stage_six_first_tracking_validation = True
        if self._config.stage is ValidationStage.FIXED_HOVER:
            self._validate_stage_six_state(
                initial_state,
                self._start_time,
            )
            self._neutral_position = tuple(
                float(value) for value in initial_state.position
            )
            self._neutral_yaw = float(initial_state.yaw)
            self._stage_six_landing_altitude = float(initial_state.position[2])
            self._stage_six_phase = StageSixPhase.TAKEOFF
            print(
                "[STAGE 6 INIT]"
                f" initial_reference={self._format_vector(initial_state.position)}m"
                f" initial_velocity={self._format_vector(initial_state.velocity)}m/s"
                f" state_timestamp={initial_state.timestamp:.6f}s"
            )
        elif self._config.stage in (
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
        if self._config.stage is ValidationStage.FIXED_HOVER:
            assert self._config.maximum_flight_duration_s is not None
            if self._stage_six_phase is StageSixPhase.COMPLETE:
                return True
            if self._elapsed(current_time) >= self._config.maximum_flight_duration_s:
                raise TimeoutError(
                    "Stage 6 exceeded maximum_flight_duration_s without a "
                    "validated landing"
                )
            return False
        if self._config.stage is ValidationStage.EQUILIBRIUM_THRUST:
            assert self._config.ramp_duration_s is not None
            assert self._config.hold_duration_s is not None
            return self._elapsed(current_time) >= (
                self._config.ramp_duration_s + self._config.hold_duration_s
            )
        return self._elapsed(current_time) >= self._config.duration_s

    def get_phase_name(self, current_time: float) -> str:
        if self._config.stage is ValidationStage.FIXED_HOVER:
            self._update_stage_six_phase(current_time)
            assert self._stage_six_phase is not None
            return f"STAGE_6_{self._stage_six_phase.value}"
        return f"STAGE_{int(self._config.stage)}_{self._config.stage.name}"

    def update(
        self,
        state: DroneState,
        current_time: float,
        dt: float,
    ) -> tuple[Reference, BodyRateThrustCommand]:
        if self._config.stage is ValidationStage.FIXED_HOVER:
            self._validate_stage_six_state(state, current_time)
            self._validate_stage_six_timing(current_time, dt)
        reference = self._reference(current_time)
        if self._config.stage is ValidationStage.FIXED_HOVER:
            self._validate_stage_six_tracking(state, reference, current_time)
        if self._config.stage is ValidationStage.EQUILIBRIUM_THRUST:
            command = self._stage_two_command(state, current_time)
        elif self._config.stage is ValidationStage.FIXED_HOVER:
            command = self._stage_six_command(state, reference, dt)
            if self._stage_six_landing_is_valid(state, current_time):
                self._stage_six_phase = StageSixPhase.COMPLETE
        else:
            command = self._controller.update(state, reference, dt)
        return reference, command

    def _stage_six_command(
        self,
        state: DroneState,
        reference: Reference,
        dt: float,
    ) -> BodyRateThrustCommand:
        """Add Stage 6 attitude damping to the reusable PID command.

        The underlying horizontal PID outputs retain their own saturation and
        conditional-integration behavior. Final saturation after attitude
        feedback cannot undo integral state decisions already made internally.
        """
        assert self._config.roll_attitude_kp is not None
        assert self._config.pitch_attitude_kp is not None
        assert self._config.roll_attitude_to_roll_rate_sign is not None
        assert self._config.pitch_attitude_to_pitch_rate_sign is not None
        assert self._config.attitude_rate_limit_rad_s is not None

        original = self._controller.update(state, reference, dt)
        roll_feedback = (
            self._config.roll_attitude_to_roll_rate_sign
            * self._config.roll_attitude_kp
            * (0.0 - state.roll)
        )
        pitch_feedback = (
            self._config.pitch_attitude_to_pitch_rate_sign
            * self._config.pitch_attitude_kp
            * (0.0 - state.pitch)
        )
        combined_roll = original.roll_rate + roll_feedback
        combined_pitch = original.pitch_rate + pitch_feedback
        final_roll = self._clamp_symmetric(
            combined_roll,
            self._config.attitude_rate_limit_rad_s,
        )
        final_pitch = self._clamp_symmetric(
            combined_pitch,
            self._config.attitude_rate_limit_rad_s,
        )

        self._stage_six_diagnostics = StageSixCommandDiagnostics(
            position_pid_roll_rate=original.roll_rate,
            position_pid_pitch_rate=original.pitch_rate,
            heading_pid_yaw_rate=original.yaw_rate,
            roll_attitude_feedback=roll_feedback,
            pitch_attitude_feedback=pitch_feedback,
            final_roll_rate=final_roll,
            final_pitch_rate=final_pitch,
            final_yaw_rate=original.yaw_rate,
            roll_saturated=(final_roll != combined_roll),
            pitch_saturated=(final_pitch != combined_pitch),
        )
        return BodyRateThrustCommand(
            roll_rate=final_roll,
            pitch_rate=final_pitch,
            yaw_rate=original.yaw_rate,
            thrust=original.thrust,
        )

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

        elif self._config.stage is ValidationStage.FIXED_HOVER:
            self._update_stage_six_phase(current_time)
            z_ref, vz_ref = self._stage_six_vertical_reference(elapsed)
            return Reference(
                position=np.array([x_ref, y_ref, z_ref], dtype=np.float64),
                velocity=np.array([0.0, 0.0, vz_ref], dtype=np.float64),
                yaw=self._neutral_yaw,
                yaw_rate=0.0,
                timestamp=elapsed,
            )

        return Reference(
            position=np.array([x_ref, y_ref, z_ref], dtype=np.float64),
            velocity=np.zeros(3, dtype=np.float64),
            yaw=self._neutral_yaw,
            yaw_rate=0.0,
            timestamp=elapsed,
        )

    def _update_stage_six_phase(self, current_time: float) -> None:
        if self._stage_six_phase is StageSixPhase.COMPLETE:
            return
        elapsed = self._elapsed(current_time)
        takeoff_end, hover_end, _ = self._stage_six_phase_times()
        if elapsed < takeoff_end:
            self._stage_six_phase = StageSixPhase.TAKEOFF
        elif elapsed < hover_end:
            self._stage_six_phase = StageSixPhase.HOVER
        else:
            self._stage_six_phase = StageSixPhase.LANDING

    def _stage_six_phase_times(self) -> tuple[float, float, float]:
        assert self._config.hover_height_m is not None
        assert self._config.takeoff_velocity_m_s is not None
        assert self._config.landing_velocity_m_s is not None
        assert self._config.hover_duration_s is not None
        # Cubic smoothstep peaks at 1.5 times its average velocity, so the
        # factor makes the configured values true peak ascent/descent limits.
        takeoff_end = (
            1.5
            * self._config.hover_height_m
            / self._config.takeoff_velocity_m_s
        )
        hover_end = takeoff_end + self._config.hover_duration_s
        landing_end = (
            hover_end
            + 1.5
            * self._config.hover_height_m
            / self._config.landing_velocity_m_s
        )
        return takeoff_end, hover_end, landing_end

    def _stage_six_vertical_reference(self, elapsed: float) -> tuple[float, float]:
        assert self._stage_six_landing_altitude is not None
        assert self._config.hover_height_m is not None
        takeoff_end, hover_end, landing_end = self._stage_six_phase_times()
        landing_altitude = self._stage_six_landing_altitude
        hover_altitude = landing_altitude + self._config.hover_height_m

        if elapsed < takeoff_end:
            return self._smooth_vertical_segment(
                landing_altitude,
                hover_altitude,
                elapsed,
                takeoff_end,
            )
        if elapsed < hover_end:
            return hover_altitude, 0.0
        return self._smooth_vertical_segment(
            hover_altitude,
            landing_altitude,
            elapsed - hover_end,
            landing_end - hover_end,
        )

    @staticmethod
    def _smooth_vertical_segment(
        start_altitude: float,
        end_altitude: float,
        phase_time: float,
        duration: float,
    ) -> tuple[float, float]:
        s = min(max(phase_time / duration, 0.0), 1.0)
        position_scale = 3.0 * s**2 - 2.0 * s**3
        velocity_scale = 6.0 * s - 6.0 * s**2
        altitude_change = end_altitude - start_altitude
        position = start_altitude + altitude_change * position_scale
        velocity = altitude_change / duration * velocity_scale
        if s >= 1.0:
            velocity = 0.0
        return position, velocity

    def _validate_stage_six_state(
        self,
        state: DroneState,
        current_time: float,
        *,
        check_age: bool = True,
    ) -> None:
        scalar_values = (
            state.roll,
            state.pitch,
            state.yaw,
            state.p,
            state.q,
            state.r,
            state.timestamp,
            current_time,
        )
        if not np.all(np.isfinite(state.position)) or not np.all(
            np.isfinite(state.velocity)
        ) or not all(math.isfinite(float(value)) for value in scalar_values):
            raise RuntimeError("Stage 6 rejected a nonfinite state sample")

        assert self._config.max_state_age_s is not None
        state_age = abs(float(current_time) - state.timestamp)
        if check_age and state_age > self._config.max_state_age_s:
            raise RuntimeError(
                f"Stage 6 rejected stale state data ({state_age:.3f} s old)"
            )

        assert self._config.max_tilt_rad is not None
        if (
            abs(state.roll) > self._config.max_tilt_rad
            or abs(state.pitch) > self._config.max_tilt_rad
        ):
            raise RuntimeError("Stage 6 tilt limit exceeded")

    def _validate_stage_six_timing(self, current_time: float, dt: float) -> None:
        # Raising delegates to UniversalRunner's stop/disarm cleanup. The
        # available interfaces cannot guarantee a controlled emergency landing,
        # so these aborts must not be interpreted as validated touchdown.
        if not math.isfinite(float(dt)) or dt <= 0.0:
            raise RuntimeError("Stage 6 requires a finite positive control-loop dt")
        assert self._config.max_control_loop_dt_s is not None
        if dt > self._config.max_control_loop_dt_s:
            raise TimeoutError("Stage 6 control-loop timeout exceeded")
        assert self._config.maximum_flight_duration_s is not None
        if self._elapsed(current_time) >= self._config.maximum_flight_duration_s:
            raise TimeoutError(
                "Stage 6 exceeded maximum_flight_duration_s without a "
                "validated landing"
            )

    def _validate_stage_six_tracking(
        self,
        state: DroneState,
        reference: Reference,
        current_time: float,
    ) -> None:
        error = reference.position - state.position
        horizontal_error = float(np.linalg.norm(error[:2]))
        assert self._config.max_horizontal_error_m is not None
        assert self._config.max_altitude_error_m is not None
        if self._stage_six_first_tracking_validation:
            assert self._stage_six_phase is not None
            print(
                "[STAGE 6 TRACKING]"
                f" phase={self._stage_six_phase.value}"
                f" elapsed={self._elapsed(current_time):.6f}s"
                f" reference={self._format_vector(reference.position)}m"
                f" measured={self._format_vector(state.position)}m"
                f" measured_timestamp={state.timestamp:.6f}s"
                f" horizontal_error={horizontal_error:.6f}m"
            )
            self._stage_six_first_tracking_validation = False
        if horizontal_error > self._config.max_horizontal_error_m:
            raise RuntimeError(
                "Stage 6 horizontal position error limit exceeded: "
                f"error={horizontal_error:.6f}m, "
                f"limit={self._config.max_horizontal_error_m:.6f}m, "
                f"reference={self._format_vector(reference.position)}m, "
                f"measured={self._format_vector(state.position)}m"
            )
        altitude_error = abs(float(error[2]))
        if altitude_error > self._config.max_altitude_error_m:
            raise RuntimeError(
                "Stage 6 altitude error limit exceeded: "
                f"error={altitude_error:.6f}m, "
                f"limit={self._config.max_altitude_error_m:.6f}m, "
                f"reference_z={reference.position[2]:.6f}m, "
                f"measured_z={state.position[2]:.6f}m"
            )

    @staticmethod
    def _format_vector(vector: np.ndarray) -> str:
        return f"({vector[0]:+.6f},{vector[1]:+.6f},{vector[2]:+.6f})"

    def _stage_six_landing_is_valid(
        self,
        state: DroneState,
        current_time: float,
    ) -> bool:
        assert self._stage_six_landing_altitude is not None
        assert self._config.landing_altitude_tolerance_m is not None
        assert self._config.landing_vertical_velocity_tolerance_m_s is not None
        _, _, landing_end = self._stage_six_phase_times()
        if self._elapsed(current_time) < landing_end:
            return False
        altitude_ok = (
            abs(state.position[2] - self._stage_six_landing_altitude)
            <= self._config.landing_altitude_tolerance_m
        )
        vertical_velocity_ok = (
            abs(state.velocity[2])
            <= self._config.landing_vertical_velocity_tolerance_m_s
        )
        return bool(altitude_ok and vertical_velocity_ok)

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
    stage=ValidationStage.FIXED_HOVER,

    # Horizontal mapping
    horizontal_mapping=HorizontalCommandMapping(
        body_x_to_pitch_rate_sign=1,
        body_y_to_roll_rate_sign=-1,
    ),

    # Temporary software-test gains, NOT flight-tuned
    horizontal_x_gains=PIDGains(kp=1, ki=0.05, kd=0),
    horizontal_y_gains=PIDGains(kp=1, ki=0.05, kd=0),
    altitude_gains=PIDGains(kp=2, ki=1, kd=0.05),
    heading_gains=PIDGains(kp=1, ki=0.0, kd=0.0),

    # Attitude feedback
    roll_attitude_to_roll_rate_sign=1,
    pitch_attitude_to_pitch_rate_sign=1,
    roll_attitude_kp=1.0,
    pitch_attitude_kp=1.0,

    # Output limits
    horizontal_rate_limit_rad_s=0.2,
    attitude_rate_limit_rad_s=0.2,
    heading_rate_limit_rad_s=0.3,
    altitude_feedback_limit=0.04,

    # Historical Stage 2 thrust settings, NOT validated hover thrust
    equilibrium_thrust=0.38,
    min_thrust=0.0,
    max_thrust=0.42,

    # Reference trajectory, for software testing
    hover_height_m=0.3,
    takeoff_velocity_m_s=0.1,
    landing_velocity_m_s=0.1,
    hover_duration_s=3.0,

    landing_altitude_tolerance_m=0.05,
    landing_vertical_velocity_tolerance_m_s=0.05,

    # Software-test safety thresholds
    max_horizontal_error_m=1.0,
    max_altitude_error_m=1.0,
    max_tilt_rad=0.2,
    maximum_flight_duration_s=30.0,
    max_state_age_s=0.2,
    max_control_loop_dt_s=0.1,

    control_frequency_hz=50.0,
    log_frequency_hz=10.0,
    arming_wait_s=1.0,

    # Keep physical operation disabled
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
            elif stage is ValidationStage.FIXED_HOVER:
                required = (
                    "reviewed PID gains and signs, attitude gains and signs, "
                    "rate/thrust limits, Stage 6 trajectory and landing "
                    "tolerances, state/tracking/timing safety limits, and "
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
    stage_six_diagnostics: StageSixCommandDiagnostics | None = None,
) -> None:
    attitude_text = ""
    if phase.startswith("STAGE_2_"):
        attitude_text = (
            f" attitude=({state.roll:+.3f},{state.pitch:+.3f},"
            f"{state.yaw:+.3f})rad"
        )
    stage_six_text = ""
    if phase.startswith("STAGE_6_"):
        position_error = reference.position - state.position
        diagnostics_text = " diagnostics=unavailable"
        if stage_six_diagnostics is not None:
            diagnostics_text = (
                " outer_loop_body_rate_cmd="
                f"({stage_six_diagnostics.position_pid_roll_rate:+.3f},"
                f"{stage_six_diagnostics.position_pid_pitch_rate:+.3f},"
                f"{stage_six_diagnostics.heading_pid_yaw_rate:+.3f})rad/s"
                " attitude_rate_feedback="
                f"({stage_six_diagnostics.roll_attitude_feedback:+.3f},"
                f"{stage_six_diagnostics.pitch_attitude_feedback:+.3f})rad/s"
                " wrapper_saturation="
                f"(roll={stage_six_diagnostics.roll_saturated},"
                f"pitch={stage_six_diagnostics.pitch_saturated})"
            )
        stage_six_text = (
            f" state_timestamp={state.timestamp:.3f}s"
            f" pos_error=({position_error[0]:+.3f},"
            f"{position_error[1]:+.3f},{position_error[2]:+.3f})m"
            f" measured_velocity=({state.velocity[0]:+.3f},"
            f"{state.velocity[1]:+.3f},{state.velocity[2]:+.3f})m/s"
            f" attitude=({state.roll:+.3f},{state.pitch:+.3f},"
            f"{state.yaw:+.3f})rad"
            f" measured_body_rates=({state.p:+.3f},{state.q:+.3f},"
            f"{state.r:+.3f})rad/s"
            f"{diagnostics_text}"
        )
    print(
        f"[{elapsed:7.2f}s] [{phase}] dt={dt * 1000.0:5.1f}ms"
        f" measured_position=({state.position[0]:+.3f},"
        f"{state.position[1]:+.3f},"
        f"{state.position[2]:+.3f})"
        f" reference_position=({reference.position[0]:+.3f},"
        f"{reference.position[1]:+.3f},"
        f"{reference.position[2]:+.3f})"
        f"{attitude_text}"
        f"{stage_six_text}"
        f" commanded_body_rates=({command.roll_rate:+.3f},"
        f"{command.pitch_rate:+.3f},"
        f"{command.yaw_rate:+.3f})rad/s"
        f" normalized_thrust={command.thrust:.4f}"
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
            log_callback=lambda elapsed, phase, state, reference, command, dt: (
                log_step(
                    elapsed,
                    phase,
                    state,
                    reference,
                    command,
                    dt,
                    program.stage_six_diagnostics,
                )
            ),
        )
        runner.run()


if __name__ == "__main__":
    main()
