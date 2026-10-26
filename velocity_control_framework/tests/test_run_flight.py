from __future__ import annotations

import numpy as np
import pytest
import experiments.run_flight as run_flight

from experiments.run_flight import (
    CIRCULAR_REFERENCE,
    CONFIGURATION_NAME,
    CONTROLLER_NAME,
    FIXED_HOVER_REFERENCE,
    HardSafetyViolation,
    LandingFailed,
    ProductionFlightProgram,
    ProductionLifecycleBlocked,
    SupervisorArmingError,
    assemble_program,
    build_runner,
    main,
    request_supervisor_arming,
    validate_powered_execution,
)
from velocity_control_framework.backends import (
    CrazyflieCommandAdapter,
    CrazyflieStateProvider,
)
from velocity_control_framework.controllers import (
    HorizontalAccelerationMapping,
    OuterLoopPIDConfig,
    OuterLoopPIDController,
)
from velocity_control_framework.interfaces import BodyRateThrustCommand, DroneState
from velocity_control_framework.references import (
    CircularFlightReference,
    VerticalFlightReference,
)
from velocity_control_framework.runner import UniversalRunner
from velocity_control_framework.runner import UniversalRunnerConfig


def initial_state() -> DroneState:
    return DroneState(
        position=np.array([1.0, 2.0, 0.1]),
        velocity=np.zeros(3),
        roll=0.0,
        pitch=0.0,
        yaw=0.3,
        timestamp=10.0,
    )


def test_default_selection_builds_production_cascaded_controller() -> None:
    config, controller, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        FIXED_HOVER_REFERENCE,
    )

    assert isinstance(controller, OuterLoopPIDController)
    assert controller.config is config
    assert config == OuterLoopPIDConfig.stage5_hardware_candidate()
    assert isinstance(config.horizontal_mapping, HorizontalAccelerationMapping)
    assert program.controller is controller


def test_fixed_hover_uses_existing_vertical_reference_provider() -> None:
    _, _, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        FIXED_HOVER_REFERENCE,
    )
    program.initialize(initial_state(), start_time=10.0)

    assert isinstance(program.reference_provider, VerticalFlightReference)
    reference = program.reference_provider.get_reference(10.0)
    np.testing.assert_allclose(reference.position, [1.0, 2.0, 0.1])
    np.testing.assert_allclose(reference.velocity, [0.0, 0.0, 0.0])
    assert reference.yaw == 0.3
    hover_reference = program.reference_provider.get_reference(13.5)
    assert hover_reference.position[2] == pytest.approx(0.4)
    assert program.reference_provider.config.takeoff_height == pytest.approx(0.30)
    assert program.reference_provider.config.armed_idle_duration == pytest.approx(0.5)
    assert program.reference_provider.config.takeoff_duration == pytest.approx(3.0)
    assert program.reference_provider.config.hover_duration == pytest.approx(3.0)
    assert program.reference_provider.config.landing_duration == pytest.approx(3.0)


def test_circular_selection_reuses_existing_circular_provider() -> None:
    _, _, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        CIRCULAR_REFERENCE,
    )
    program.initialize(initial_state(), start_time=10.0)

    assert isinstance(program.reference_provider, CircularFlightReference)


def test_program_calls_controller_once_without_extra_attitude_feedback() -> None:
    expected = BodyRateThrustCommand(0.1, -0.2, 0.3, 0.4)

    class CountingController:
        def __init__(self) -> None:
            self.reset_calls = 0
            self.update_calls = 0

        def reset(self) -> None:
            self.reset_calls += 1

        def update(self, state, reference, dt):
            del state, reference, dt
            self.update_calls += 1
            return expected

    controller = CountingController()
    program = ProductionFlightProgram(controller, FIXED_HOVER_REFERENCE)
    program.initialize(initial_state(), start_time=10.0)

    _, actual = program.update(
        safe_flight_state(position=np.array([1.0, 2.0, 0.1])),
        current_time=10.1,
        dt=0.02,
    )

    assert actual is expected
    assert controller.reset_calls == 1
    assert controller.update_calls == 1


class FakeSupervisor:
    def __init__(self) -> None:
        self.requests: list[bool] = []
        self.can_be_armed = True
        self.can_fly = True
        self.is_armed = False
        self.is_tumbled = False
        self.is_locked = False
        self.is_crashed = False
        self.hl_control_active = False
        self.emergency_stops = 0

    def send_arming_request(self, armed: bool) -> None:
        self.requests.append(armed)
        self.is_armed = armed

    def send_emergency_stop(self) -> None:
        self.emergency_stops += 1


class FakeCrazyflie:
    def __init__(self) -> None:
        self.supervisor = FakeSupervisor()


def supervisor_status(**changes) -> dict[str, bool]:
    status = {
        "can_be_armed": True,
        "can_fly": False,
        "is_armed": False,
        "is_tumbled": False,
        "is_locked": False,
        "is_crashed": False,
        "hl_control_active": False,
    }
    status.update(changes)
    return status


class IncrementingClock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        self.value += 0.05
        return self.value


def test_runner_assembly_uses_production_interfaces_and_arm_callbacks() -> None:
    _, _, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        FIXED_HOVER_REFERENCE,
    )
    cf = FakeCrazyflie()

    runner = build_runner(cf, program)

    assert isinstance(runner, UniversalRunner)
    assert isinstance(runner._state_provider, CrazyflieStateProvider)
    assert isinstance(runner._command_adapter, CrazyflieCommandAdapter)
    assert runner._flight_program is program
    runner._arm()
    runner._disarm()
    assert cf.supervisor.requests == [True, False]


@pytest.mark.parametrize("fault", ["is_locked", "is_crashed", "is_tumbled"])
def test_runner_rejects_unsafe_supervisor_state_before_arming(fault) -> None:
    cf = FakeCrazyflie()
    setattr(cf.supervisor, fault, True)

    with pytest.raises(SupervisorArmingError, match="unsafe pre-arm"):
        request_supervisor_arming(cf)

    assert cf.supervisor.requests == []


def test_prearm_not_yet_can_fly_is_accepted_and_delayed_arming_succeeds() -> None:
    cf = FakeCrazyflie()
    prearm = supervisor_status(can_fly=False, is_armed=False)
    statuses = iter((prearm, prearm, supervisor_status(can_fly=True, is_armed=True)))

    request_supervisor_arming(
        cf,
        status_reader=lambda: next(statuses),
        clock=IncrementingClock(),
        sleeper=lambda _: None,
    )

    assert cf.supervisor.requests == [True]


def test_arming_timeout_requests_disarm() -> None:
    cf = FakeCrazyflie()
    status = supervisor_status(can_fly=False, is_armed=False)

    with pytest.raises(SupervisorArmingError, match="timed out"):
        request_supervisor_arming(
            cf,
            timeout_s=0.15,
            status_reader=lambda: status,
            clock=IncrementingClock(),
            sleeper=lambda _: None,
        )

    assert cf.supervisor.requests == [True, False]


def test_firmware_prearm_refusal_is_not_bypassed() -> None:
    cf = FakeCrazyflie()
    status = supervisor_status(can_be_armed=False)

    with pytest.raises(SupervisorArmingError, match="unsafe pre-arm"):
        request_supervisor_arming(cf, status_reader=lambda: status)

    assert cf.supervisor.requests == []


def test_fixed_execution_is_enabled_but_circular_remains_interlocked() -> None:
    validate_powered_execution(FIXED_HOVER_REFERENCE)
    with pytest.raises(ProductionLifecycleBlocked, match="circular tracking"):
        validate_powered_execution(CIRCULAR_REFERENCE)


def test_landing_check_succeeds_at_maximum_duration() -> None:
    _, _, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        FIXED_HOVER_REFERENCE,
    )
    program.initialize(initial_state(), start_time=10.0)
    landing_state = safe_flight_state(
        position=np.array([1.0, 2.0, 0.14]),
        velocity=np.array([0.0, 0.0, 0.02]),
        timestamp=24.99,
    )
    _, command = program.update(landing_state, 24.99, 0.02)

    assert not program.is_complete(19.5)
    assert program.get_phase_name(19.5) == "LANDING_CHECK"
    assert program.is_complete(25.0)
    assert program.termination_record is not None
    assert program.termination_record.status == "SUCCESS"
    assert program.termination_record.final_estimated_altitude_m == pytest.approx(0.14)
    assert program.termination_record.final_vertical_velocity_m_s == pytest.approx(0.02)
    assert program.termination_record.final_normalized_thrust == pytest.approx(
        command.thrust
    )


def test_low_altitude_alone_does_not_confirm_landing() -> None:
    _, _, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        FIXED_HOVER_REFERENCE,
    )
    program.initialize(initial_state(), start_time=10.0)
    program.update(
        safe_flight_state(
            position=np.array([1.0, 2.0, 0.14]),
            velocity=np.array([0.0, 0.0, 0.06]),
            timestamp=24.99,
        ),
        24.99,
        0.02,
    )

    with pytest.raises(LandingFailed, match="not_satisfied"):
        program.is_complete(25.0)

    assert program.termination_record is not None
    assert program.termination_record.status == "LANDING_FAILED"


def safe_flight_state(**changes) -> DroneState:
    values = {
        "position": np.array([1.0, 2.0, 0.2]),
        "velocity": np.zeros(3),
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.3,
        "timestamp": 10.1,
    }
    values.update(changes)
    return DroneState(**values)


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"position": np.array([1.51, 2.0, 0.2])}, "horizontal displacement"),
        ({"position": np.array([1.0, 2.0, 1.7])}, "vertical position"),
        ({"velocity": np.array([0.76, 0.0, 0.0])}, "horizontal velocity"),
        ({"velocity": np.array([0.0, 0.0, 1.51])}, "vertical velocity"),
        ({"roll": 0.46}, "attitude envelope"),
        ({"timestamp": 9.9}, "state age"),
    ],
)
def test_hard_flight_envelopes_remain_active(changes, message) -> None:
    _, _, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        FIXED_HOVER_REFERENCE,
    )
    origin = initial_state()
    program.initialize(origin, start_time=10.0)

    with pytest.raises(HardSafetyViolation, match=message):
        program.update(safe_flight_state(**changes), 10.1, 0.02)


def test_nonfinite_state_deadline_and_maximum_duration_are_rejected() -> None:
    _, _, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        FIXED_HOVER_REFERENCE,
    )
    program.initialize(initial_state(), start_time=10.0)

    nonfinite = safe_flight_state()
    nonfinite.roll = float("nan")
    with pytest.raises(HardSafetyViolation, match="non-finite"):
        program.update(nonfinite, 10.1, 0.02)
    with pytest.raises(HardSafetyViolation, match="deadline"):
        program.update(safe_flight_state(), 10.1, 0.101)
    with pytest.raises(LandingFailed):
        program.is_complete(25.01)


def test_unsafe_inflight_supervisor_state_is_rejected() -> None:
    status = {
        "can_fly": True,
        "is_armed": True,
        "is_tumbled": False,
        "is_locked": False,
        "is_crashed": False,
        "hl_control_active": False,
    }
    _, _, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        FIXED_HOVER_REFERENCE,
        supervisor_status=lambda: status,
    )
    program.initialize(initial_state(), start_time=10.0)
    status["is_tumbled"] = True

    with pytest.raises(HardSafetyViolation, match="unsafe supervisor"):
        program.update(safe_flight_state(), 10.1, 0.02)


def test_arming_estimator_discontinuity_is_rejected() -> None:
    _, _, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        FIXED_HOVER_REFERENCE,
    )
    before = initial_state()
    after = safe_flight_state(
        position=np.array([1.2, 2.0, 0.1]),
        timestamp=10.1,
    )

    with pytest.raises(HardSafetyViolation, match="position discontinuity"):
        program.report_initialization_states(
            pre_arming_state=before,
            post_arming_state=after,
        )


def test_hard_failure_requests_emergency_stop_then_cleanup() -> None:
    events: list[str] = []

    class Provider:
        def start(self):
            events.append("provider_start")

        def get_state(self):
            return initial_state()

        def stop(self):
            events.append("provider_stop")

    class Adapter:
        def send(self, command):
            del command

        def stop(self):
            events.append("command_stop")

    class FailingProgram:
        def initialize(self, initial_state, start_time):
            del initial_state, start_time

        def is_complete(self, current_time):
            del current_time
            return False

        def get_phase_name(self, current_time):
            del current_time
            return "TAKEOFF"

        def update(self, state, current_time, dt):
            del state, current_time, dt
            raise HardSafetyViolation("test envelope violation")

    runner = UniversalRunner(
        state_provider=Provider(),
        command_adapter=Adapter(),
        flight_program=FailingProgram(),
        arm=lambda: events.append("arm"),
        disarm=lambda: events.append("disarm"),
        emergency_stop=lambda: events.append("emergency_stop"),
        config=UniversalRunnerConfig(arming_wait_s=0.0),
    )

    with pytest.raises(HardSafetyViolation, match="test envelope"):
        runner.run()

    assert events == [
        "provider_start",
        "arm",
        "emergency_stop",
        "command_stop",
        "disarm",
        "provider_stop",
    ]


def test_landing_failure_requests_emergency_stop_then_cleanup() -> None:
    events: list[str] = []

    class Provider:
        def start(self):
            events.append("provider_start")

        def get_state(self):
            return initial_state()

        def stop(self):
            events.append("provider_stop")

    class Adapter:
        def send(self, command):
            del command

        def stop(self):
            events.append("command_stop")

    class LandingFailureProgram:
        def initialize(self, initial_state, start_time):
            del initial_state, start_time

        def is_complete(self, current_time):
            del current_time
            raise LandingFailed("estimated_landing_criteria_not_satisfied")

    runner = UniversalRunner(
        state_provider=Provider(),
        command_adapter=Adapter(),
        flight_program=LandingFailureProgram(),
        arm=lambda: events.append("arm"),
        disarm=lambda: events.append("disarm"),
        emergency_stop=lambda: events.append("emergency_stop"),
        config=UniversalRunnerConfig(arming_wait_s=0.0),
    )

    with pytest.raises(LandingFailed, match="not_satisfied"):
        runner.run()

    assert events == [
        "provider_start",
        "arm",
        "emergency_stop",
        "command_stop",
        "disarm",
        "provider_stop",
    ]


def test_cleanup_after_supervisor_arming_failure() -> None:
    events: list[str] = []
    cf = FakeCrazyflie()
    original_request = cf.supervisor.send_arming_request

    def record_request(armed):
        events.append(f"arm_request_{armed}")
        original_request(armed)

    cf.supervisor.send_arming_request = record_request

    class Provider:
        def start(self):
            events.append("provider_start")

        def get_state(self):
            return initial_state()

        def stop(self):
            events.append("provider_stop")

    class Adapter:
        def stop(self):
            events.append("command_stop")

    class UnusedProgram:
        pass

    status = supervisor_status(can_fly=False, is_armed=False)
    runner = UniversalRunner(
        state_provider=Provider(),
        command_adapter=Adapter(),
        flight_program=UnusedProgram(),
        arm=lambda: request_supervisor_arming(
            cf,
            timeout_s=0.0,
            status_reader=lambda: status,
            clock=IncrementingClock(),
            sleeper=lambda _: None,
        ),
        disarm=lambda: events.append("runner_disarm"),
        config=UniversalRunnerConfig(arming_wait_s=0.0),
    )

    with pytest.raises(SupervisorArmingError, match="timed out"):
        runner.run()

    assert events == [
        "provider_start",
        "arm_request_True",
        "arm_request_False",
        "command_stop",
        "provider_stop",
    ]


def test_inspection_cli_does_not_request_hardware(capsys, monkeypatch) -> None:
    hardware_calls = []
    monkeypatch.setattr(
        run_flight,
        "_run_hardware",
        lambda uri, program: hardware_calls.append((uri, program)),
    )
    main([])

    output = capsys.readouterr().out
    assert "[CONTROLLER] outer-loop-pid" in output
    assert "[REFERENCE] fixed-hover" in output
    assert "inspection_only" in output
    assert hardware_calls == []


def test_execute_has_no_interactive_authorization_prompt(monkeypatch) -> None:
    hardware_calls = []
    monkeypatch.setattr(
        "builtins.input",
        lambda _: pytest.fail("powered execution must not prompt for authorization"),
    )
    monkeypatch.setattr(
        run_flight,
        "_run_hardware",
        lambda uri, program: hardware_calls.append((uri, program)),
    )

    main(["--execute"])

    assert len(hardware_calls) == 1
