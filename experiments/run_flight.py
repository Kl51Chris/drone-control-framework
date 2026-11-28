"""Minimal supervised production flight with selectable outer-loop control."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
from experiments.excitation import HoverExcitation
from experiments.measurement_noise import MeasurementNoise
from experiments.inspection import LineInspectionReference, NavigationFault, NavigationFaultConfig
from velocity_control_framework.controllers.outer_loop_deepc import (
    OuterLoopDeePCConfig, OuterLoopDeePCController,
)
from velocity_control_framework.controllers.identified_hover_model import IdentifiedHoverModelConfig
import json
import math
from pathlib import Path
import time
from typing import Callable

import numpy as np

from velocity_control_framework.backends import (
    CrazyflieCommandAdapter,
    CrazyflieStateProvider,
    CrazySimCommandAdapter,
    CrazySimSupervisorAdapter,
)
from velocity_control_framework.controllers import (
    HorizontalAccelerationMapping,
    OuterLoopLQRConfig,
    OuterLoopLQRController,
    OuterLoopPIDConfig,
    OuterLoopPIDController,
)
from velocity_control_framework.backends.crazysim_hlc_lifecycle import (
    CommandOwner,
    CrazySimHLCLifecycle,
)
from velocity_control_framework.controllers.near_hover_model import NearHoverModelConfig
from velocity_control_framework.controllers.outer_loop_mpc import (
    MPCSolveError,
    OuterLoopMPCConfig,
    OuterLoopMPCController,
)
from velocity_control_framework.interfaces import (
    BodyRateThrustCommand,
    Controller,
    DroneState,
    Reference,
)
from velocity_control_framework.references import (
    CircularFlightConfig,
    CircularFlightReference,
    CircularFlightState,
    FlightState,
    VerticalFlightConfig,
    VerticalFlightReference,
)
from velocity_control_framework.runner import UniversalRunner, UniversalRunnerConfig
from velocity_control_framework.logger import FlightLogger, RunConfiguration


CONTROLLER_NAME = "outer-loop-pid"
CONFIGURATION_NAME = "stage5-hardware-candidate"
CRAZYSIM_CONFIGURATION_NAME = "crazysim-cf2x-t350-initial"
CRAZYSIM_CALIBRATED_CONFIGURATION_NAME = "crazysim-cf2x-t350-calibrated"
LQR_CONTROLLER_NAME = "outer-loop-lqr"
MPC_CONTROLLER_NAME = "outer-loop-mpc"
DEEPC_CONTROLLER_NAME = "outer-loop-deepc"
LQR_CONFIGURATION_NAME = "lqr-dfall-initial"
FIXED_HOVER_REFERENCE = "fixed-hover"
CIRCULAR_REFERENCE = "circular"
INSPECTION_REFERENCE = "line-inspection"
DEFAULT_URI = "radio://0/80/2M"
DEFAULT_CRAZYSIM_URI = "udp://127.0.0.1:19850"
HARDWARE_BACKEND = "hardware"
CRAZYSIM_BACKEND = "crazysim"
FIXED_HOVER_MAXIMUM_FLIGHT_DURATION_S = 15.0
CIRCULAR_MAXIMUM_FLIGHT_DURATION_S = 30.0


class HardSafetyViolation(RuntimeError):
    """A hard flight envelope or system-health requirement failed."""


class LandingFailed(HardSafetyViolation):
    """Maximum duration elapsed without estimator-qualified landing."""


class SupervisorArmingError(HardSafetyViolation):
    """The firmware supervisor refused or failed to confirm arming."""


@dataclass(frozen=True, slots=True)
class ProductionSafetyConfig:
    """Reviewed hard limits retained from Stage 5 calibration flights."""

    max_state_age_s: float = 0.15
    max_control_loop_dt_s: float = 0.10
    max_horizontal_displacement_m: float = 2.0
    max_altitude_above_ground_m: float = 1.50
    min_altitude_below_ground_m: float = -0.10
    max_horizontal_speed_m_s: float = 0.75
    max_vertical_speed_m_s: float = 1.50
    max_tilt_rad: float = 0.45
    maximum_flight_duration_s: float = FIXED_HOVER_MAXIMUM_FLIGHT_DURATION_S
    max_arming_position_jump_m: float = 0.15
    landing_altitude_tolerance_m: float = 0.05
    landing_vertical_speed_tolerance_m_s: float = 0.05


@dataclass(frozen=True, slots=True)
class FlightTerminationRecord:
    status: str
    termination_reason: str
    final_estimated_altitude_m: float
    final_height_above_initialized_ground_m: float
    final_vertical_velocity_m_s: float
    final_normalized_thrust: float


class ProductionFlightProgram:
    """Coordinate an existing reference provider and production controller.

    Descent-reference completion changes the phase to ``LANDING_CHECK``. At
    maximum duration, both low estimated height and low vertical speed are
    required. This is not an independent claim of physical touchdown.
    """

    def __init__(
        self,
        controller: Controller,
        reference_name: str,
        *,
        safety: ProductionSafetyConfig | None = None,
        supervisor_status: Callable[[], dict[str, bool]] | None = None,
        vertical_flight_config: VerticalFlightConfig | None = None,
        circular_flight_config: CircularFlightConfig | None = None,
    ) -> None:
        if reference_name not in (FIXED_HOVER_REFERENCE, CIRCULAR_REFERENCE, INSPECTION_REFERENCE):
            raise ValueError(f"unsupported reference provider: {reference_name}")
        self.controller = controller
        self.controller_name = type(controller).__name__
        self.configuration_name = "unspecified"
        self.reference_name = reference_name
        self.safety = safety or ProductionSafetyConfig(
            maximum_flight_duration_s=(
                CIRCULAR_MAXIMUM_FLIGHT_DURATION_S
                if reference_name in (CIRCULAR_REFERENCE, INSPECTION_REFERENCE)
                else FIXED_HOVER_MAXIMUM_FLIGHT_DURATION_S
            )
        )
        self.supervisor_status = supervisor_status
        self.vertical_flight_config = vertical_flight_config
        self.circular_flight_config = circular_flight_config
        self.reference_provider: (
            VerticalFlightReference | CircularFlightReference | None
        ) = None
        self._initial_state: DroneState | None = None
        self._start_time: float | None = None
        self._descent_reference_complete = False
        self._last_state: DroneState | None = None
        self._last_command: BodyRateThrustCommand | None = None
        self.termination_record: FlightTerminationRecord | None = None
        self.use_official_lifecycle = False
        self.official_lifecycle: CrazySimHLCLifecycle | None = None
        self._official_takeoff_end: float | None = None
        self._official_landing_end: float | None = None
        self.step_axis: str | None = None
        self.step_amplitude = 0.03
        self.reference_mode = "constant"
        self.log_root: Path | None = None
        self.excitation: HoverExcitation | None = None
        self._excitation_log: dict = {}
        self.force_solver_failure = False
        self._fallback_reason: str | None = None
        self._tracking_start: float | None = None
        self._previous_input_valid = False
        self.measurement_noise: MeasurementNoise | NavigationFault | None = None
        self._noise_step = 0
        self._noise_log: dict = {}

    @property
    def estimated_landing_confirmed(self) -> bool:
        return bool(self.termination_record and self.termination_record.status == "SUCCESS")

    def report_initialization_states(
        self,
        *,
        pre_arming_state: DroneState,
        post_arming_state: DroneState,
    ) -> None:
        self._validate_finite_state(pre_arming_state)
        self._validate_finite_state(post_arming_state)
        if post_arming_state.timestamp <= pre_arming_state.timestamp:
            raise HardSafetyViolation("post-arming estimator state is not newer")
        position_jump = float(
            np.linalg.norm(post_arming_state.position - pre_arming_state.position)
        )
        if position_jump > self.safety.max_arming_position_jump_m:
            raise HardSafetyViolation(
                "post-arming estimator position discontinuity: "
                f"{position_jump:.3f} m"
            )

    @staticmethod
    def select_initialization_state(
        *,
        pre_arming_state: DroneState,
        post_arming_state: DroneState,
    ) -> DroneState:
        del pre_arming_state
        return post_arming_state

    def initialize(self, initial_state: DroneState, start_time: float) -> None:
        self._noise_step = 0
        self._noise_log = {}
        self._validate_finite_state(initial_state)
        initial_age = float(start_time) - initial_state.timestamp
        if initial_age < -0.01 or initial_age > self.safety.max_state_age_s:
            raise HardSafetyViolation(
                f"initial estimator state age is unsafe: {initial_age:.3f} s"
            )
        self.controller.reset()
        self._initial_state = initial_state
        self._start_time = float(start_time)
        if self.reference_name == FIXED_HOVER_REFERENCE:
            self.reference_provider = VerticalFlightReference(
                initial_position=initial_state.position,
                initial_yaw=initial_state.yaw,
                start_time=start_time,
                config=(
                    self.vertical_flight_config
                    or VerticalFlightConfig(takeoff_height=0.30)
                ),
            )
        else:
            provider = LineInspectionReference if self.reference_name == INSPECTION_REFERENCE else CircularFlightReference
            self.reference_provider = provider(
                initial_position=initial_state.position,
                initial_yaw=initial_state.yaw,
                start_time=start_time,
                config=self.circular_flight_config,
            )

    def is_complete(self, current_time: float) -> bool:
        self._update_descent_completion(current_time)
        if (
            self._official_landing_end is not None
            and current_time >= self._official_landing_end + 0.5
            and self._last_state is not None
            and self._initial_state is not None
            and 0 <= current_time - self._last_state.timestamp <= self.safety.max_state_age_s
            and self._last_state.position[2] - self._initial_state.position[2]
            <= self.safety.landing_altitude_tolerance_m
            and abs(self._last_state.velocity[2]) <= self.safety.landing_vertical_speed_tolerance_m_s
        ):
            self._descent_reference_complete = True
            self._finalize_landing_check()
            if self._last_command is None and self._fallback_reason is None:
                raise LandingFailed("official flight landed without entering body-rate tracking")
            return self.estimated_landing_confirmed
        if self._elapsed(current_time) < self.safety.maximum_flight_duration_s:
            return False
        if self.termination_record is None:
            self._finalize_landing_check()
        if self.estimated_landing_confirmed:
            if self.official_lifecycle is not None and self._last_command is None and self._fallback_reason is None:
                raise LandingFailed("official flight landed without entering body-rate tracking")
            return True
        assert self.termination_record is not None
        raise LandingFailed(self.termination_record.termination_reason)

    def get_phase_name(self, current_time: float) -> str:
        self._update_descent_completion(current_time)
        if self._fallback_reason is not None:
            return "FALLBACK_LANDING"
        if self._descent_reference_complete:
            return "LANDING_CHECK"
        return self._provider().get_state(current_time).name

    def update(
        self,
        state: DroneState,
        current_time: float,
        dt: float,
    ) -> tuple[Reference, BodyRateThrustCommand | None]:
        self._validate_safety(state, current_time, dt)
        reference = self._provider().get_reference(current_time)
        if self.official_lifecycle is not None:
            if (
                self.step_axis is not None and self._tracking_start is not None
                and 0.4 <= current_time - self._tracking_start
                and self._provider().get_state(current_time).name == "HOVER"
            ):
                position = reference.position.copy()
                yaw = reference.yaw
                if self.step_axis == "yaw":
                    yaw += self.step_amplitude
                else:
                    position["xyz".index(self.step_axis)] += self.step_amplitude
                reference = Reference(position, reference.velocity, yaw,
                                      reference.yaw_rate, reference.timestamp)
            return reference, self._update_official(state, reference, current_time, dt)
        command = self.controller.update(state, reference, dt)
        self._last_state = state
        self._last_command = command
        return reference, command

    def _update_official(
        self, state: DroneState, reference: Reference, now: float, dt: float
    ) -> BodyRateThrustCommand | None:
        lifecycle = self.official_lifecycle
        assert lifecycle is not None and self._initial_state is not None
        self._previous_input_valid = lifecycle.owner is CommandOwner.BODY_RATE
        provider = self._provider()
        phase = provider.get_state(now).name
        self._last_state = state
        self._noise_log = {}
        ground_z = float(self._initial_state.position[2])
        if phase == "TAKEOFF" and lifecycle.owner is CommandOwner.GROUND:
            # Leave time for the official loop to settle before tracking.
            duration = provider.config.takeoff_duration - 0.5
            lifecycle.takeoff(ground_z + provider.config.takeoff_height, duration)
            self._official_takeoff_end = now + duration
        elif phase in ("HOVER", "CIRCLE"):
            if lifecycle.owner is CommandOwner.OFFICIAL:
                assert self._official_takeoff_end is not None
                if (
                    now < self._official_takeoff_end + 0.4
                    or abs(float(state.position[2] - reference.position[2])) > 0.05
                    or np.linalg.norm(state.velocity) > 0.10
                    or max(abs(state.roll), abs(state.pitch)) > 0.10
                ):
                    return None
                self.controller.reset()
                self._tracking_start = now
            if lifecycle.owner in (CommandOwner.OFFICIAL, CommandOwner.BODY_RATE):
                try:
                    if (
                        self.force_solver_failure and self._tracking_start is not None
                        and now - self._tracking_start >= 0.40
                        and isinstance(self.controller, OuterLoopMPCController)
                    ):
                        raise MPCSolveError("controlled solver failure")
                    observation=state
                    if self.measurement_noise is not None:
                        observation,self._noise_log=self.measurement_noise.apply(state,self._noise_step)
                        self._noise_step+=1
                    if (
                        self.reference_mode == "preview"
                        and isinstance(self.controller, OuterLoopMPCController)
                        and phase == "CIRCLE"
                    ):
                        profile = provider.config
                        assert self._start_time is not None
                        end = self._start_time + profile.armed_idle_duration + profile.takeoff_duration + profile.circle_duration
                        period = self.controller.config.model.sample_period_s
                        references = tuple(
                            provider.get_reference(min(now + i * period, math.nextafter(end, -math.inf)))
                            for i in range(1, self.controller.config.horizon + 1)
                        )
                        command = self.controller.update_preview(observation, references)
                    else:
                        command = self.controller.update(observation, reference, dt)
                except MPCSolveError as error:
                    if np.linalg.norm(state.velocity) > 0.10 or max(abs(state.roll), abs(state.pitch)) > 0.10:
                        raise HardSafetyViolation("solver failed outside the official recovery envelope") from error
                    self._fallback_reason = str(error)
                    lifecycle.land(ground_z + 0.03, provider.config.landing_duration)
                    self._official_landing_end = now + provider.config.landing_duration
                    return None
                if self.excitation is not None:
                    assert self._tracking_start is not None
                    baseline = np.array([command.roll_rate, command.pitch_rate, command.yaw_rate, command.thrust])
                    injection = self.excitation.sample(now - self._tracking_start)
                    requested = baseline + injection
                    cfg = self.controller.config
                    total = np.clip(requested, [-cfg.max_rate] * 3 + [cfg.min_thrust],
                                    [cfg.max_rate] * 3 + [cfg.max_thrust])
                    self._excitation_log = {
                        "baseline": baseline.tolist(), "injected": injection.tolist(),
                        "saturated": (total != requested).tolist(),
                    }
                    command = BodyRateThrustCommand(*total)
                    self.controller.record_issued_command(command)
                self._last_command = command
                return command
        elif phase == "LANDING" and lifecycle.owner in (
            CommandOwner.OFFICIAL, CommandOwner.BODY_RATE
        ):
            lifecycle.land(ground_z + 0.03, provider.config.landing_duration)
            self._official_landing_end = now + provider.config.landing_duration
        return None

    def get_logging_diagnostics(self) -> dict:
        if self.official_lifecycle is None:
            return {}
        return {
            "measurement_noise": self._noise_log,
            "excitation": self._excitation_log if self.official_lifecycle.owner is CommandOwner.BODY_RATE else {},
            "command_source": self.official_lifecycle.owner.name,
            "fallback_reason": self._fallback_reason,
            "previous_input_valid": self._previous_input_valid,
            "handover_seed": (
                [0.0, 0.0, 0.0, self.controller.config.model.equilibrium_thrust]
                if isinstance(self.controller, OuterLoopMPCController)
                and self.official_lifecycle.owner is CommandOwner.BODY_RATE
                and not self._previous_input_valid else None
            ),
            "mpc": (
                getattr(self.controller, "diagnostics", {})
                if self.official_lifecycle.owner is CommandOwner.BODY_RATE or self._fallback_reason else {}
            ),
        }

    def get_prediction_snapshot(self) -> dict | None:
        if isinstance(self.controller, OuterLoopMPCController):
            return self.controller.prediction
        return None

    def _update_descent_completion(self, current_time: float) -> None:
        state = self._provider().get_state(current_time)
        complete = state in (FlightState.COMPLETE, CircularFlightState.COMPLETE)
        if not complete:
            return
        self._descent_reference_complete = True

    def _finalize_landing_check(self) -> None:
        if not self._descent_reference_complete:
            raise LandingFailed("maximum_duration_before_descent_complete")
        if self._initial_state is None or self._last_state is None:
            raise LandingFailed("landing_check_has_no_observed_state")
        height = float(self._last_state.position[2] - self._initial_state.position[2])
        vertical_velocity = float(self._last_state.velocity[2])
        thrust = float(self._last_command.thrust) if self._last_command else math.nan
        confirmed = (
            height <= self.safety.landing_altitude_tolerance_m
            and abs(vertical_velocity)
            <= self.safety.landing_vertical_speed_tolerance_m_s
        )
        reason = (
            "estimated_landing_criteria_satisfied"
            if confirmed
            else "estimated_landing_criteria_not_satisfied"
        )
        self.termination_record = FlightTerminationRecord(
            status="SUCCESS" if confirmed else "LANDING_FAILED",
            termination_reason=reason,
            final_estimated_altitude_m=float(self._last_state.position[2]),
            final_height_above_initialized_ground_m=height,
            final_vertical_velocity_m_s=vertical_velocity,
            final_normalized_thrust=thrust,
        )
        print("[RESULT] " + json.dumps(asdict(self.termination_record), sort_keys=True))

    def _validate_safety(
        self,
        state: DroneState,
        current_time: float,
        dt: float,
    ) -> None:
        self._validate_finite_state(state)
        self._validate_duration(current_time)
        if not math.isfinite(dt) or dt <= 0.0:
            raise HardSafetyViolation("control-loop dt is not finite and positive")
        if dt > self.safety.max_control_loop_dt_s:
            raise HardSafetyViolation("control-loop deadline exceeded")
        age = current_time - state.timestamp
        if age < -0.01 or age > self.safety.max_state_age_s:
            raise HardSafetyViolation(f"estimator state age is unsafe: {age:.3f} s")
        assert self._initial_state is not None
        displacement = state.position - self._initial_state.position
        horizontal_displacement = float(np.linalg.norm(displacement[:2]))
        if horizontal_displacement > self.safety.max_horizontal_displacement_m:
            raise HardSafetyViolation("horizontal displacement envelope exceeded")
        if not (
            self.safety.min_altitude_below_ground_m
            <= displacement[2]
            <= self.safety.max_altitude_above_ground_m
        ):
            raise HardSafetyViolation("vertical position envelope exceeded")
        horizontal_speed = float(np.linalg.norm(state.velocity[:2]))
        if horizontal_speed > self.safety.max_horizontal_speed_m_s:
            raise HardSafetyViolation("horizontal velocity envelope exceeded")
        if abs(float(state.velocity[2])) > self.safety.max_vertical_speed_m_s:
            raise HardSafetyViolation("vertical velocity envelope exceeded")
        if max(abs(state.roll), abs(state.pitch)) > self.safety.max_tilt_rad:
            raise HardSafetyViolation("attitude envelope exceeded")
        if self.supervisor_status is not None:
            status = self.supervisor_status()
            if (
                not status["can_fly"]
                or not status["is_armed"]
                or status["is_tumbled"]
                or status["is_locked"]
                or status["is_crashed"]
                or status["hl_control_active"]
            ):
                raise HardSafetyViolation(f"unsafe supervisor state: {status}")

    def _validate_duration(self, current_time: float) -> None:
        elapsed = self._elapsed(current_time)
        if elapsed > self.safety.maximum_flight_duration_s:
            raise HardSafetyViolation("maximum flight duration exceeded")

    def _elapsed(self, current_time: float) -> float:
        if self._start_time is None:
            raise RuntimeError("flight program has not been initialized")
        return float(current_time) - self._start_time

    @staticmethod
    def _validate_finite_state(state: DroneState) -> None:
        values = (
            *state.position,
            *state.velocity,
            state.roll,
            state.pitch,
            state.yaw,
            state.p,
            state.q,
            state.r,
            state.timestamp,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise HardSafetyViolation("estimator state contains non-finite values")

    def _provider(self) -> VerticalFlightReference | CircularFlightReference:
        if self.reference_provider is None:
            raise RuntimeError("flight program has not been initialized")
        return self.reference_provider


def select_configuration(name: str) -> OuterLoopPIDConfig | OuterLoopLQRConfig:
    if name == CONFIGURATION_NAME:
        config = OuterLoopPIDConfig.stage5_hardware_candidate()
        if not isinstance(config.horizontal_mapping, HorizontalAccelerationMapping):
            raise RuntimeError("selected configuration activated legacy mapping mode")
        return config
    if name == CRAZYSIM_CONFIGURATION_NAME:
        return OuterLoopPIDConfig.crazysim_cf2x_t350_initial()
    if name == CRAZYSIM_CALIBRATED_CONFIGURATION_NAME:
        return OuterLoopPIDConfig.crazysim_cf2x_t350_calibrated()
    if name == LQR_CONFIGURATION_NAME:
        return OuterLoopLQRConfig.dfall_initial()
    raise ValueError(f"unsupported controller configuration: {name}")


def assemble_program(
    controller_name: str,
    configuration_name: str,
    reference_name: str,
    **program_options,
) -> tuple[
    OuterLoopPIDConfig | OuterLoopLQRConfig,
    OuterLoopPIDController | OuterLoopLQRController,
    ProductionFlightProgram,
]:
    config = select_configuration(configuration_name)
    if configuration_name == CRAZYSIM_CALIBRATED_CONFIGURATION_NAME:
        maximum_duration = (
            CIRCULAR_MAXIMUM_FLIGHT_DURATION_S
            if reference_name in (CIRCULAR_REFERENCE, INSPECTION_REFERENCE)
            else 9.5
        )
        program_options.setdefault(
            "safety",
            ProductionSafetyConfig(
                max_horizontal_displacement_m=0.50,
                max_altitude_above_ground_m=0.60,
                min_altitude_below_ground_m=-0.05,
                max_horizontal_speed_m_s=0.50,
                max_vertical_speed_m_s=0.75,
                max_tilt_rad=0.25,
                maximum_flight_duration_s=maximum_duration,
                max_arming_position_jump_m=0.10,
                landing_altitude_tolerance_m=0.06,
                landing_vertical_speed_tolerance_m_s=0.08,
            ),
        )
        program_options.setdefault(
            "vertical_flight_config",
            VerticalFlightConfig(
                takeoff_height=0.20,
                armed_idle_duration=0.50,
                takeoff_duration=3.0,
                hover_duration=2.0,
                landing_duration=3.0,
            ),
        )
        if reference_name in (CIRCULAR_REFERENCE, INSPECTION_REFERENCE):
            program_options.setdefault(
                "circular_flight_config",
                CircularFlightConfig(
                    takeoff_height=0.20,
                    circle_radius=0.10,
                    circle_revolutions=1.0,
                    armed_idle_duration=0.50,
                    takeoff_duration=3.0,
                    circle_duration=LineInspectionReference.duration_s if reference_name == INSPECTION_REFERENCE else 15.0,
                    landing_duration=3.0,
                ),
            )
    if controller_name == CONTROLLER_NAME and isinstance(config, OuterLoopPIDConfig):
        controller = OuterLoopPIDController(config)
    elif controller_name == LQR_CONTROLLER_NAME and isinstance(config, OuterLoopLQRConfig):
        controller = OuterLoopLQRController(config)
    elif controller_name not in (CONTROLLER_NAME, LQR_CONTROLLER_NAME):
        raise ValueError(f"unsupported controller: {controller_name}")
    else:
        raise ValueError(
            f"configuration {configuration_name} is incompatible with {controller_name}"
        )
    program = ProductionFlightProgram(controller, reference_name, **program_options)
    program.controller_name = controller_name
    program.configuration_name = configuration_name
    return config, controller, program


def _supervisor_status(cf) -> dict[str, bool]:
    supervisor = cf.supervisor
    return {
        name: bool(getattr(supervisor, name))
        for name in (
            "can_be_armed",
            "can_fly",
            "is_armed",
            "is_tumbled",
            "is_locked",
            "is_crashed",
            "hl_control_active",
        )
    }


def request_supervisor_arming(
    cf,
    *,
    timeout_s: float = 1.0,
    poll_interval_s: float = 0.05,
    status_reader: Callable[[], dict[str, bool]] | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
    arming_request: Callable[[bool], None] | None = None,
) -> None:
    """Validate pre-arm state, request arming, and confirm firmware state."""

    read_status = status_reader or (lambda: _supervisor_status(cf))
    before = read_status()
    faults = tuple(
        name for name in ("is_tumbled", "is_locked", "is_crashed") if before[name]
    )
    if not before["can_be_armed"] or faults or before["hl_control_active"]:
        raise SupervisorArmingError(
            f"unsafe pre-arm supervisor state: {before}; faults={faults}"
        )
    if before["is_armed"]:
        raise SupervisorArmingError(
            f"supervisor was already armed before request: {before}"
        )

    request_arming = arming_request or cf.supervisor.send_arming_request
    request_arming(True)
    deadline = clock() + timeout_s
    last_status = before
    try:
        while clock() < deadline:
            sleeper(poll_interval_s)
            last_status = read_status()
            faults = tuple(
                name
                for name in ("is_tumbled", "is_locked", "is_crashed")
                if last_status[name]
            )
            if faults or last_status["hl_control_active"]:
                raise SupervisorArmingError(
                    "supervisor fault while arming: "
                    f"{last_status}; faults={faults}"
                )
            if last_status["is_armed"] and last_status["can_fly"]:
                print(f"[ARM] Firmware confirmed flight readiness: {last_status}")
                return
        raise SupervisorArmingError(
            f"supervisor arming timed out after {timeout_s:.2f} s; "
            f"last_status={last_status}"
        )
    except BaseException:
        # The request may have reached the firmware even if confirmation did not.
        # Explicitly request disarming before propagating the original failure.
        try:
            request_arming(False)
        except BaseException as cleanup_error:
            print(f"[WARN] Disarm after arming failure failed: {cleanup_error}")
        raise


def build_runner(
    cf,
    program: ProductionFlightProgram,
    run_logger: FlightLogger | None = None,
    *,
    backend: str = HARDWARE_BACKEND,
    crazysim_supervisor: CrazySimSupervisorAdapter | None = None,
    crazysim_lifecycle: str | None = None,
    crazysim_outputs: Callable[[], dict[str, float]] | None = None,
) -> UniversalRunner:
    """Assemble existing production backends around UniversalRunner."""

    if backend == CRAZYSIM_BACKEND:
        if (
            crazysim_supervisor is None
            or crazysim_lifecycle != "safe_auto_armed"
            or crazysim_outputs is None
        ):
            raise ValueError(
                "CrazySim runner requires a verified safe auto-armed supervisor"
            )
        program.supervisor_status = crazysim_supervisor.status
    else:
        program.supervisor_status = lambda: _supervisor_status(cf)

    state_provider = CrazyflieStateProvider(
        cf,
        onboard_state_callback=(
            run_logger.record_onboard_state if run_logger is not None else None
        ),
        delete_log_blocks_on_stop=(backend == CRAZYSIM_BACKEND),
    )
    if backend == HARDWARE_BACKEND:
        command_adapter = CrazyflieCommandAdapter(cf)
    elif backend == CRAZYSIM_BACKEND:
        command_adapter = CrazySimCommandAdapter(cf)
    else:
        raise ValueError(f"unsupported backend: {backend}")

    body_rate_adapter = command_adapter
    if program.use_official_lifecycle:
        if backend != CRAZYSIM_BACKEND:
            raise ValueError("official lifecycle is currently CrazySim-only")
        program.official_lifecycle = CrazySimHLCLifecycle(cf, body_rate_adapter)

        class TrackingAdapter:
            def send(self, command: BodyRateThrustCommand) -> None:
                assert program.official_lifecycle is not None
                program.official_lifecycle.send_tracking(command)
                if isinstance(program.controller, OuterLoopMPCController):
                    program.controller.record_issued_command(command)

            def stop(self) -> None:
                assert program.official_lifecycle is not None
                program.official_lifecycle.stop()

        command_adapter = TrackingAdapter()

    def arm() -> None:
        if isinstance(body_rate_adapter, CrazySimCommandAdapter):
            assert crazysim_supervisor is not None
            assert crazysim_outputs is not None
            from tools.validate_crazysim_actuators import validate_safe_startup

            status = crazysim_supervisor.status()
            try:
                lifecycle = validate_safe_startup(status, crazysim_outputs())
            except RuntimeError as error:
                raise SupervisorArmingError(
                    f"unsafe CrazySim command-unlock state: {error}"
                ) from error
            if lifecycle != "safe_auto_armed":
                raise SupervisorArmingError(
                    f"unexpected CrazySim command-unlock lifecycle: {lifecycle}"
                )
            if program.official_lifecycle is None:
                body_rate_adapter.unlock()
        else:
            request_supervisor_arming(cf)

    disarm = (
        (lambda: None)
        if backend == CRAZYSIM_BACKEND
        else (lambda: cf.supervisor.send_arming_request(False))
    )
    emergency_stop = (
        crazysim_supervisor.send_emergency_stop
        if crazysim_supervisor is not None
        else cf.supervisor.send_emergency_stop
    )

    return UniversalRunner[
        DroneState,
        Reference,
        BodyRateThrustCommand,
    ](
        state_provider=state_provider,
        command_adapter=command_adapter,
        flight_program=program,
        arm=arm,
        disarm=disarm,
        emergency_stop=emergency_stop,
        config=UniversalRunnerConfig(),
        log_callback=_log_step,
        run_logger=run_logger,
    )


def _log_step(
    elapsed: float,
    phase: str,
    state: DroneState,
    reference: Reference,
    command: BodyRateThrustCommand,
    dt: float,
) -> None:
    print(
        f"[{elapsed:7.2f}s] [{phase}] dt={dt * 1000.0:5.1f}ms "
        f"position={tuple(float(value) for value in state.position)} "
        f"reference={tuple(float(value) for value in reference.position)} "
        f"rates=({command.roll_rate:+.3f},{command.pitch_rate:+.3f},"
        f"{command.yaw_rate:+.3f})rad/s thrust={command.thrust:.4f}"
    )


def _print_selection(
    config: OuterLoopPIDConfig | OuterLoopLQRConfig,
    controller_name: str,
    configuration_name: str,
    reference_name: str,
    safety: ProductionSafetyConfig,
) -> None:
    print(f"[CONTROLLER] {controller_name}")
    print(f"[CONFIGURATION] {configuration_name}")
    print(f"[REFERENCE] {reference_name}")
    print("[PARAMETERS] " + json.dumps(asdict(config), sort_keys=True))
    print("[SAFETY] " + json.dumps(asdict(safety), sort_keys=True))
    print("[LOG] Powered runs are stored under the project log/ directory")


def validate_backend_uri(backend: str, uri: str) -> None:
    """Reject transport/backend mismatches without discovery or fallback."""
    if backend == HARDWARE_BACKEND:
        if not uri.startswith("radio://"):
            raise ValueError("hardware backend requires a radio:// URI")
        return
    if backend == CRAZYSIM_BACKEND:
        if not uri.startswith("udp://"):
            raise ValueError("crazysim backend requires an udp:// URI")
        return
    raise ValueError(f"unsupported backend: {backend}")


def validate_configuration_backend(backend: str, configuration_name: str) -> None:
    simulation_configurations = {
        CRAZYSIM_CONFIGURATION_NAME,
        CRAZYSIM_CALIBRATED_CONFIGURATION_NAME,
    }
    if backend == HARDWARE_BACKEND and configuration_name in simulation_configurations:
        raise ValueError("CrazySim controller configurations require --backend crazysim")
    if (
        backend == CRAZYSIM_BACKEND
        and configuration_name not in simulation_configurations
    ):
        raise ValueError("CrazySim backend requires a CrazySim-specific configuration")


def resolve_uri(backend: str, uri: str | None) -> str:
    selected = uri or (
        DEFAULT_CRAZYSIM_URI if backend == CRAZYSIM_BACKEND else DEFAULT_URI
    )
    validate_backend_uri(backend, selected)
    return selected


def _run_backend(
    uri: str, program: ProductionFlightProgram, *, backend: str
) -> None:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from tools.crazysim_session import PassiveCrazyflie
    from tools.validate_crazysim_actuators import MotorMonitor, validate_safe_startup

    cflib.crtp.init_drivers()
    crazyflie = (
        PassiveCrazyflie(rw_cache="./cache")
        if backend == CRAZYSIM_BACKEND
        else Crazyflie(rw_cache="./cache")
    )
    with SyncCrazyflie(uri, cf=crazyflie) as link:
        logger_options = {"root": program.log_root} if program.log_root is not None else {}
        run_logger = FlightLogger(build_run_configuration(uri, program, backend=backend), **logger_options)
        print(f"[LOG] {run_logger.directory.resolve()}")
        if backend == HARDWARE_BACKEND:
            build_runner(link.cf, program, run_logger, backend=backend).run()
            return

        supervisor = CrazySimSupervisorAdapter(link.cf)
        motors = MotorMonitor(link.cf)
        runner_invoked = False
        try:
            supervisor.start()
            motors.start()
            lifecycle = validate_safe_startup(
                supervisor.status(), motors.wait_for_first()
            )
            runner = build_runner(
                link.cf,
                program,
                run_logger,
                backend=backend,
                crazysim_supervisor=supervisor,
                crazysim_lifecycle=lifecycle,
                crazysim_outputs=motors.latest,
            )
            runner_invoked = True
            runner.run()
        finally:
            if not runner_invoked:
                run_logger.close("failed", "CrazySim preflight validation failed")
            cleanup_errors = []
            try:
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    outputs = motors.latest()
                    if all(math.isfinite(value) and abs(value) <= 1e-6 for value in outputs.values()):
                        break
                    time.sleep(0.01)
                else:
                    cleanup_errors.append(f"outputs active after STOP: {outputs}")
            except BaseException as error:
                cleanup_errors.append(f"STOP verification failed: {error}")
            for label, monitor in (("motors", motors), ("supervisor", supervisor)):
                try:
                    monitor.stop()
                except BaseException as error:
                    cleanup_errors.append(f"{label} cleanup failed: {error}")
            if cleanup_errors:
                raise RuntimeError("; ".join(cleanup_errors))


def _run_hardware(uri: str, program: ProductionFlightProgram) -> None:
    """Preserve the existing physical-hardware execution entry point."""
    _run_backend(uri, program, backend=HARDWARE_BACKEND)


def build_run_configuration(
    uri: str,
    program: ProductionFlightProgram,
    *,
    backend: str = HARDWARE_BACKEND,
) -> RunConfiguration:
    """Build logger provenance from the fully selected production program."""
    reference_configuration: dict[str, object] = {
        "safety": asdict(program.safety),
        "flight_lifecycle": "hlc" if program.use_official_lifecycle else "controller",
        "step_axis": program.step_axis,
        "step_amplitude": program.step_amplitude if program.step_axis else None,
        "reference_mode": program.reference_mode,
        "measurement_noise": asdict(program.measurement_noise) if isinstance(program.measurement_noise, MeasurementNoise) else None,
        "excitation": (
            {"seed": program.excitation.seed, "axis": program.excitation.axis,
             "max_hz": program.excitation.max_hz, "kind": program.excitation.kind}
            if program.excitation is not None else None
        ),
        "forced_solver_failure": program.force_solver_failure,
    }
    if isinstance(program.measurement_noise, NavigationFault):
        reference_configuration["navigation_fault"] = asdict(program.measurement_noise.config)
        reference_configuration["inspection_distance_m"] = LineInspectionReference.distance_m
    if program.reference_name == FIXED_HOVER_REFERENCE:
        if program.vertical_flight_config is not None:
            reference_configuration["profile"] = asdict(
                program.vertical_flight_config
            )
    elif program.circular_flight_config is not None:
        reference_configuration["profile"] = asdict(program.circular_flight_config)
    return RunConfiguration(
        experiment_name="production-flight",
        reference_type=program.reference_name,
        reference_configuration=reference_configuration,
        controller_type=program.controller_name,
        parameter_set=program.configuration_name,
        controller_configuration=asdict(program.controller.config),
        target_control_frequency_hz=UniversalRunnerConfig().control_frequency_hz,
        hardware=(
            f"CrazySim SITL ({uri})"
            if backend == CRAZYSIM_BACKEND
            else f"Crazyflie ({uri})"
        ),
        state_estimation_source="Crazyflie onboard stateEstimate via passthrough",
        data_sources={
            "backend": backend,
            "transport_uri": uri,
            "controller_state": {
                "description": "DroneState consumed by controller"
            },
            "onboard": {
                "description": "Crazyflie firmware stateEstimate telemetry"
            },
            "external": {
                "available": False,
                "description": "MuJoCo ground truth is not logged separately",
            },
        },
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument(
        "--controller",
        choices=(CONTROLLER_NAME, LQR_CONTROLLER_NAME, MPC_CONTROLLER_NAME, DEEPC_CONTROLLER_NAME),
        default=CONTROLLER_NAME,
    )
    parser.add_argument(
        "--configuration",
        choices=(
            CONFIGURATION_NAME,
            CRAZYSIM_CONFIGURATION_NAME,
            CRAZYSIM_CALIBRATED_CONFIGURATION_NAME,
            LQR_CONFIGURATION_NAME,
        ),
        default=None,
    )
    parser.add_argument(
        "--reference",
        choices=(FIXED_HOVER_REFERENCE, CIRCULAR_REFERENCE, INSPECTION_REFERENCE),
        default=FIXED_HOVER_REFERENCE,
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--backend",
        choices=(HARDWARE_BACKEND, CRAZYSIM_BACKEND),
        default=HARDWARE_BACKEND,
    )
    parser.add_argument("--uri", default=None)
    parser.add_argument("--flight-lifecycle", choices=("controller", "hlc"), default="controller")
    parser.add_argument("--mpc-config", type=Path)
    parser.add_argument("--horizon", type=int)
    parser.add_argument("--mpc-model", choices=("m0","hybrid","joint-fir6","balanced-low-fir25",
                                              "axis-r6-p6-y25-t12","axis-r6-p6-y25-t16",
                                              "axis-r6-p6-y25-t20","axis-r6-p6-y25-t25",
                                              "axis-r10-p10-y16-t12"), default="m0")
    parser.add_argument("--model-evidence", type=Path, default=Path("log/phase3-offline-coordinates/results.json"))
    parser.add_argument("--measurement-noise", choices=("none","iid-small"))
    parser.add_argument("--navigation", choices=("nominal", "position-loss", "position-bias"))
    parser.add_argument("--navigation-scale", type=float)
    parser.add_argument("--navigation-start-step", type=int)
    parser.add_argument("--noise-amplitude", type=float, default=1.0)
    parser.add_argument("--noise-seed", type=int, default=0)
    parser.add_argument("--step-axis", choices=("x", "y", "z", "yaw"))
    parser.add_argument("--step-amplitude", type=float, help="signed meters or yaw radians")
    parser.add_argument("--step-hold", type=float, help="seconds after the unexpected reference change")
    parser.add_argument("--reference-mode", choices=("constant", "preview"), default="constant")
    parser.add_argument("--circle-radius", type=float)
    parser.add_argument("--circle-duration", type=float)
    parser.add_argument("--circle-speed-profile", choices=("smoothstep", "smootherstep"))
    parser.add_argument("--log-root", type=Path)
    parser.add_argument("--excitation-seed", type=int)
    parser.add_argument("--excitation-axis", choices=("all", "roll", "pitch", "yaw", "thrust"), default="all")
    parser.add_argument("--excitation-max-hz", type=float, choices=(8.0, 16.0), default=8.0)
    parser.add_argument("--excitation-kind", choices=("multisine", "filtered"), default="multisine")
    parser.add_argument("--force-solver-failure", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    use_deepc = args.controller == DEEPC_CONTROLLER_NAME
    if args.mpc_model != "m0" and args.controller != MPC_CONTROLLER_NAME:
        raise ValueError("identified model is only available for MPC")
    if args.mpc_model != "m0" and args.excitation_seed is not None:
        raise ValueError("identified candidate does not run data-collection excitation")
    use_mpc = args.controller in (MPC_CONTROLLER_NAME, DEEPC_CONTROLLER_NAME)
    if args.reference == INSPECTION_REFERENCE:
        if not use_mpc or args.backend != CRAZYSIM_BACKEND or args.measurement_noise is not None:
            raise ValueError("line inspection requires a CrazySim predictive controller and its navigation profiles")
    elif any(value is not None for value in (args.navigation, args.navigation_scale, args.navigation_start_step)):
        raise ValueError("navigation profiles require --reference line-inspection")
    if args.measurement_noise is not None and (
        not use_mpc or args.backend != CRAZYSIM_BACKEND or args.excitation_seed is not None
    ):
        raise ValueError("host observation noise requires a CrazySim predictive controller without collection excitation")
    if use_deepc and args.excitation_seed is not None:
        raise ValueError("DeePC prototype does not run data-collection excitation")
    if args.excitation_seed is not None and (
        not use_mpc or args.backend != CRAZYSIM_BACKEND or args.reference != FIXED_HOVER_REFERENCE
        or args.step_axis or args.force_solver_failure or args.excitation_seed < 0
    ):
        raise ValueError("excitation requires a nonnegative seed and CrazySim MPC hover")
    custom_circle = any(value is not None for value in (args.circle_radius, args.circle_duration, args.circle_speed_profile))
    custom_step = args.step_amplitude is not None or args.step_hold is not None
    if custom_step and not args.step_axis:
        raise ValueError("step parameters require --step-axis")
    if custom_circle and args.reference != CIRCULAR_REFERENCE:
        raise ValueError("circle parameters require --reference circular")
    if not use_mpc and (custom_circle or custom_step or args.reference_mode != "constant"):
        raise ValueError("evaluation settings require the MPC controller")
    if use_mpc and args.backend != CRAZYSIM_BACKEND:
        raise ValueError("MPC experiments are CrazySim-only")
    if not use_mpc and (args.mpc_config or args.horizon or args.step_axis or args.force_solver_failure):
        raise ValueError("MPC options require --controller outer-loop-mpc")
    if args.reference != FIXED_HOVER_REFERENCE and (args.step_axis or args.force_solver_failure):
        raise ValueError("step and solver failure require the fixed-hover reference")
    uri = resolve_uri(args.backend, args.uri)
    configuration_name = args.configuration or (
        LQR_CONFIGURATION_NAME
        if args.controller == LQR_CONTROLLER_NAME
        else CRAZYSIM_CALIBRATED_CONFIGURATION_NAME if use_mpc
        else (
            CRAZYSIM_CONFIGURATION_NAME
            if args.backend == CRAZYSIM_BACKEND
            else CONFIGURATION_NAME
        )
    )
    validate_configuration_backend(args.backend, configuration_name)
    config, _, program = assemble_program(
        CONTROLLER_NAME if use_mpc else args.controller,
        configuration_name,
        args.reference,
    )
    program.log_root = args.log_root
    if args.measurement_noise is not None:
        program.measurement_noise=MeasurementNoise(args.measurement_noise,args.noise_amplitude,args.noise_seed)
    if args.reference == INSPECTION_REFERENCE:
        scale = 1.0 if args.navigation_scale is None else args.navigation_scale
        start = 150 if args.navigation_start_step is None else args.navigation_start_step
        if not 0 < abs(scale) <= 2 or not 25 <= start <= 375:
            raise ValueError("inspection fault requires 0 < |scale| <= 2 and start tick 25–375")
        program.measurement_noise = NavigationFault(NavigationFaultConfig(
            args.navigation or "nominal", start_step=start, end_step=start + 200,
            drift_y_m_s=0.005 * scale, bias_y_m=0.025 * scale,
        ))
    if args.flight_lifecycle == "hlc" or use_mpc:
        if (
            args.backend != CRAZYSIM_BACKEND
            or configuration_name != CRAZYSIM_CALIBRATED_CONFIGURATION_NAME
        ):
            raise ValueError("HLC flight requires the calibrated CrazySim configuration")
        program.use_official_lifecycle = True
    if use_mpc:
        parameters = json.loads(args.mpc_config.read_text()) if args.mpc_config else {}
        model = NearHoverModelConfig(**parameters.pop("model", {}))
        if not math.isclose(model.sample_period_s, UniversalRunnerConfig().control_period_s):
            raise ValueError("MPC model period must match the production runner")
        if args.horizon is not None:
            parameters["horizon"] = args.horizon
        if use_deepc:
            config = OuterLoopDeePCConfig(model=model, **parameters)
            program.controller = OuterLoopDeePCController(config)
        else:
            if args.mpc_model != "m0":
                if model != NearHoverModelConfig():
                    raise ValueError("identified model requires the original phase3-v1 physical parameters")
                parameters["identified"] = IdentifiedHoverModelConfig.from_report(args.model_evidence,args.mpc_model)
            config = OuterLoopMPCConfig(model=model, **parameters)
            program.controller = OuterLoopMPCController(config)
        program.controller_name = args.controller
        program.force_solver_failure = args.force_solver_failure
        program.step_axis = args.step_axis
        program.reference_mode = args.reference_mode
        if args.excitation_seed is not None:
            program.excitation = HoverExcitation(args.excitation_seed, args.excitation_axis,
                                                 args.excitation_max_hz, args.excitation_kind)
            program.vertical_flight_config = replace(program.vertical_flight_config, hover_duration=2.4)
        if args.step_axis:
            amplitude = args.step_amplitude
            if amplitude is None:
                amplitude = math.radians(5) if args.step_axis == "yaw" else 0.03
            limit = 0.20 if args.step_axis == "yaw" else 0.10
            if not 0 < abs(amplitude) <= limit:
                raise ValueError("step amplitude is outside the evaluation envelope")
            program.step_amplitude = amplitude
            if args.step_hold is not None:
                if not 0.5 <= args.step_hold <= 2.0:
                    raise ValueError("step hold must fit the existing 9.5-second flight timeout (0.5 to 2.0 seconds)")
                assert program.vertical_flight_config is not None
                program.vertical_flight_config = replace(
                    program.vertical_flight_config, hover_duration=0.4 + args.step_hold
                )
        if custom_circle:
            profile = program.circular_flight_config
            assert profile is not None
            profile = replace(
                profile,
                circle_radius=args.circle_radius if args.circle_radius is not None else profile.circle_radius,
                circle_duration=args.circle_duration if args.circle_duration is not None else profile.circle_duration,
                speed_profile=args.circle_speed_profile or profile.speed_profile,
            )
            peak_speed = 2 * math.pi * profile.circle_radius / profile.circle_duration
            peak_speed *= 1.5 if profile.speed_profile == "smoothstep" else 1.875
            if profile.circle_radius > 0.18 or peak_speed > 0.14 or profile.circle_duration > 22:
                raise ValueError("circle exceeds the nominal evaluation envelope")
            program.circular_flight_config = profile
    _print_selection(
        config,
        args.controller,
        configuration_name,
        args.reference,
        program.safety,
    )
    print(f"[BACKEND] {args.backend}")
    print(f"[URI] {uri}")
    if not args.execute:
        print("[STATUS] inspection_only; no backend connection attempted")
        return
    print("[STATUS] connecting for one supervised flight")
    if args.backend == HARDWARE_BACKEND:
        _run_hardware(uri, program)
    else:
        _run_backend(uri, program, backend=CRAZYSIM_BACKEND)
    if program.estimated_landing_confirmed:
        print(
            "[STATUS] SUCCESS; estimated landing criteria satisfied; "
            "physical touchdown was not independently confirmed"
        )


if __name__ == "__main__":
    main()
