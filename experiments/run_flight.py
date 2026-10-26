"""Minimal supervised production flight for the cascaded outer-loop PID."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Callable

import numpy as np

from velocity_control_framework.backends import (
    CrazyflieCommandAdapter,
    CrazyflieStateProvider,
)
from velocity_control_framework.controllers import (
    HorizontalAccelerationMapping,
    OuterLoopPIDConfig,
    OuterLoopPIDController,
)
from velocity_control_framework.interfaces import (
    BodyRateThrustCommand,
    DroneState,
    Reference,
)
from velocity_control_framework.references import (
    CircularFlightReference,
    CircularFlightState,
    FlightState,
    VerticalFlightConfig,
    VerticalFlightReference,
)
from velocity_control_framework.runner import UniversalRunner, UniversalRunnerConfig


CONTROLLER_NAME = "outer-loop-pid"
CONFIGURATION_NAME = "stage5-hardware-candidate"
FIXED_HOVER_REFERENCE = "fixed-hover"
CIRCULAR_REFERENCE = "circular"
DEFAULT_URI = "radio://0/80/2M"


class HardSafetyViolation(RuntimeError):
    """A hard flight envelope or system-health requirement failed."""


class ProductionLifecycleBlocked(RuntimeError):
    """The requested production lifecycle is not approved for execution."""


class LandingFailed(HardSafetyViolation):
    """Maximum duration elapsed without estimator-qualified landing."""


class SupervisorArmingError(HardSafetyViolation):
    """The firmware supervisor refused or failed to confirm arming."""


@dataclass(frozen=True, slots=True)
class ProductionSafetyConfig:
    """Reviewed hard limits retained from Stage 5 calibration flights."""

    max_state_age_s: float = 0.15
    max_control_loop_dt_s: float = 0.10
    max_horizontal_displacement_m: float = 0.50
    max_altitude_above_ground_m: float = 1.50
    min_altitude_below_ground_m: float = -0.10
    max_horizontal_speed_m_s: float = 0.75
    max_vertical_speed_m_s: float = 1.50
    max_tilt_rad: float = 0.45
    maximum_flight_duration_s: float = 15.0
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
        controller: OuterLoopPIDController,
        reference_name: str,
        *,
        safety: ProductionSafetyConfig | None = None,
        supervisor_status: Callable[[], dict[str, bool]] | None = None,
    ) -> None:
        if reference_name not in (FIXED_HOVER_REFERENCE, CIRCULAR_REFERENCE):
            raise ValueError(f"unsupported reference provider: {reference_name}")
        self.controller = controller
        self.reference_name = reference_name
        self.safety = safety or ProductionSafetyConfig()
        self.supervisor_status = supervisor_status
        self.reference_provider: (
            VerticalFlightReference | CircularFlightReference | None
        ) = None
        self._initial_state: DroneState | None = None
        self._start_time: float | None = None
        self._descent_reference_complete = False
        self._last_state: DroneState | None = None
        self._last_command: BodyRateThrustCommand | None = None
        self.termination_record: FlightTerminationRecord | None = None

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
                config=VerticalFlightConfig(takeoff_height=0.30),
            )
        else:
            self.reference_provider = CircularFlightReference(
                initial_position=initial_state.position,
                initial_yaw=initial_state.yaw,
                start_time=start_time,
            )

    def is_complete(self, current_time: float) -> bool:
        self._update_descent_completion(current_time)
        if self._elapsed(current_time) < self.safety.maximum_flight_duration_s:
            return False
        if self.termination_record is None:
            self._finalize_landing_check()
        if self.estimated_landing_confirmed:
            return True
        assert self.termination_record is not None
        raise LandingFailed(self.termination_record.termination_reason)

    def get_phase_name(self, current_time: float) -> str:
        self._update_descent_completion(current_time)
        if self._descent_reference_complete:
            return "LANDING_CHECK"
        return self._provider().get_state(current_time).name

    def update(
        self,
        state: DroneState,
        current_time: float,
        dt: float,
    ) -> tuple[Reference, BodyRateThrustCommand]:
        self._validate_safety(state, current_time, dt)
        reference = self._provider().get_reference(current_time)
        command = self.controller.update(state, reference, dt)
        self._last_state = state
        self._last_command = command
        return reference, command

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


def select_configuration(name: str) -> OuterLoopPIDConfig:
    if name != CONFIGURATION_NAME:
        raise ValueError(f"unsupported controller configuration: {name}")
    config = OuterLoopPIDConfig.stage5_hardware_candidate()
    if not isinstance(config.horizontal_mapping, HorizontalAccelerationMapping):
        raise RuntimeError("selected configuration activated legacy mapping mode")
    return config


def assemble_program(
    controller_name: str,
    configuration_name: str,
    reference_name: str,
    **program_options,
) -> tuple[OuterLoopPIDConfig, OuterLoopPIDController, ProductionFlightProgram]:
    if controller_name != CONTROLLER_NAME:
        raise ValueError(f"unsupported controller: {controller_name}")
    config = select_configuration(configuration_name)
    controller = OuterLoopPIDController(config)
    program = ProductionFlightProgram(controller, reference_name, **program_options)
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

    cf.supervisor.send_arming_request(True)
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
            cf.supervisor.send_arming_request(False)
        except BaseException as cleanup_error:
            print(f"[WARN] Disarm after arming failure failed: {cleanup_error}")
        raise


def build_runner(cf, program: ProductionFlightProgram) -> UniversalRunner:
    """Assemble existing production backends around UniversalRunner."""

    program.supervisor_status = lambda: _supervisor_status(cf)

    def arm() -> None:
        request_supervisor_arming(cf)

    return UniversalRunner[
        DroneState,
        Reference,
        BodyRateThrustCommand,
    ](
        state_provider=CrazyflieStateProvider(cf),
        command_adapter=CrazyflieCommandAdapter(cf),
        flight_program=program,
        arm=arm,
        disarm=lambda: cf.supervisor.send_arming_request(False),
        emergency_stop=cf.supervisor.send_emergency_stop,
        config=UniversalRunnerConfig(),
        log_callback=_log_step,
    )


def validate_powered_execution(reference_name: str) -> None:
    if reference_name == CIRCULAR_REFERENCE:
        raise ProductionLifecycleBlocked(
            "circular tracking remains disabled until fixed-hover production "
            "validation succeeds"
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


def _print_selection(config: OuterLoopPIDConfig, reference_name: str) -> None:
    print(f"[CONTROLLER] {CONTROLLER_NAME}")
    print(f"[CONFIGURATION] {CONFIGURATION_NAME}")
    print(f"[REFERENCE] {reference_name}")
    print("[PARAMETERS] " + json.dumps(asdict(config), sort_keys=True))
    print("[SAFETY] " + json.dumps(asdict(ProductionSafetyConfig()), sort_keys=True))
    print("[LOG] UniversalRunner console callback; no production run directory")


def _run_hardware(uri: str, program: ProductionFlightProgram) -> None:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    crazyflie = Crazyflie(rw_cache="./cache")
    with SyncCrazyflie(uri, cf=crazyflie) as link:
        build_runner(link.cf, program).run()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=(CONTROLLER_NAME,), default=CONTROLLER_NAME)
    parser.add_argument(
        "--configuration", choices=(CONFIGURATION_NAME,), default=CONFIGURATION_NAME
    )
    parser.add_argument(
        "--reference",
        choices=(FIXED_HOVER_REFERENCE, CIRCULAR_REFERENCE),
        default=FIXED_HOVER_REFERENCE,
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--uri", default=DEFAULT_URI)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    config, _, program = assemble_program(
        args.controller,
        args.configuration,
        args.reference,
    )
    _print_selection(config, args.reference)
    if not args.execute:
        print("[STATUS] inspection_only; no hardware connection attempted")
        return
    try:
        validate_powered_execution(args.reference)
        print("[STATUS] connecting for one supervised flight")
        _run_hardware(args.uri, program)
        if program.estimated_landing_confirmed:
            print(
                "[STATUS] SUCCESS; estimated landing criteria satisfied; "
                "physical touchdown was not independently confirmed"
            )
    except ProductionLifecycleBlocked as error:
        print("[STATUS] blocked")
        print(f"[SHUTDOWN] no hardware connection was opened: {error}")
        raise SystemExit(str(error)) from error


if __name__ == "__main__":
    main()
