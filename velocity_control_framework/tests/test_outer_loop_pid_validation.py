from __future__ import annotations

from dataclasses import replace
import math

import numpy as np
import pytest

from experiments.outer_loop_pid_validation import (
    PIDGains,
    OuterLoopPIDValidationProgram,
    StageSixCommandDiagnostics,
    ValidationExperimentConfig,
    ValidationStage,
    log_step,
)
from velocity_control_framework.controllers import (
    HorizontalCommandMapping,
    OuterLoopPIDController,
)
from velocity_control_framework.interfaces import (
    BodyRateThrustCommand,
    DroneState,
    Reference,
)
from velocity_control_framework.runner import UniversalRunner, UniversalRunnerConfig


def valid_config(stage: ValidationStage) -> ValidationExperimentConfig:
    horizontal_mapping = HorizontalCommandMapping(
        body_x_to_pitch_rate_sign=1,
        body_y_to_roll_rate_sign=-1,
    )
    horizontal_gains = PIDGains()
    altitude_gains = PIDGains()
    heading_gains = PIDGains()
    equilibrium_thrust = 0.0
    polarity_offset_m = None
    ramp_start_thrust = None
    ramp_duration_s = None
    hold_duration_s = None
    roll_attitude_kp = None
    pitch_attitude_kp = None
    yaw_attitude_kp = None
    roll_attitude_to_roll_rate_sign = None
    pitch_attitude_to_pitch_rate_sign = None
    yaw_attitude_to_yaw_rate_sign = None
    attitude_rate_limit_rad_s = None
    trajectory_radius_m = None
    trajectory_period_s = None
    stage_six_values: dict[str, float] = {}

    if stage is ValidationStage.HORIZONTAL_POLARITY:
        horizontal_gains = PIDGains(kp=0.1)
        polarity_offset_m = 0.01
    elif stage is ValidationStage.EQUILIBRIUM_THRUST:
        horizontal_mapping = None
        roll_attitude_kp = 0.2
        pitch_attitude_kp = 0.3
        yaw_attitude_kp = 0.4
        roll_attitude_to_roll_rate_sign = 1
        pitch_attitude_to_pitch_rate_sign = 1
        yaw_attitude_to_yaw_rate_sign = 1
        attitude_rate_limit_rad_s = 0.1
        ramp_start_thrust = 0.1
        equilibrium_thrust = 0.4
        ramp_duration_s = 4.0
        hold_duration_s = 2.0
    elif stage is ValidationStage.HORIZONTAL_STABILIZATION:
        horizontal_gains = PIDGains(kp=0.1)
        equilibrium_thrust = 0.4
    elif stage >= ValidationStage.HORIZONTAL_ALTITUDE:
        horizontal_gains = PIDGains(kp=0.1)
        altitude_gains = PIDGains(kp=0.1)
        equilibrium_thrust = 0.4

    if stage is ValidationStage.FIXED_HOVER:
        roll_attitude_kp = 0.2
        pitch_attitude_kp = 0.3
        roll_attitude_to_roll_rate_sign = 1
        pitch_attitude_to_pitch_rate_sign = -1
        attitude_rate_limit_rad_s = 0.1
        stage_six_values = {
            "hover_height_m": 0.5,
            "takeoff_velocity_m_s": 0.25,
            "landing_velocity_m_s": 0.25,
            "hover_duration_s": 2.0,
            "landing_altitude_tolerance_m": 0.05,
            "landing_vertical_velocity_tolerance_m_s": 0.1,
            "max_horizontal_error_m": 1.0,
            "max_altitude_error_m": 1.0,
            "max_tilt_rad": 0.5,
            "maximum_flight_duration_s": 10.0,
            "max_state_age_s": 0.2,
            "max_control_loop_dt_s": 0.1,
        }

    if stage >= ValidationStage.HEADING:
        heading_gains = PIDGains(kp=0.1)

    if stage is ValidationStage.SLOW_TRAJECTORY:
        trajectory_radius_m = 0.05
        trajectory_period_s = 20.0

    return ValidationExperimentConfig(
        stage=stage,
        fixed_position=(1.0, 2.0, 0.5),
        fixed_yaw=math.pi / 2.0,
        horizontal_mapping=horizontal_mapping,
        equilibrium_thrust=equilibrium_thrust,
        min_thrust=0.0,
        max_thrust=0.6,
        horizontal_rate_limit_rad_s=0.1,
        heading_rate_limit_rad_s=0.1,
        altitude_feedback_limit=0.1,
        horizontal_x_gains=horizontal_gains,
        horizontal_y_gains=horizontal_gains,
        altitude_gains=altitude_gains,
        heading_gains=heading_gains,
        polarity_offset_m=polarity_offset_m,
        ramp_start_thrust=ramp_start_thrust,
        ramp_duration_s=ramp_duration_s,
        hold_duration_s=hold_duration_s,
        roll_attitude_kp=roll_attitude_kp,
        pitch_attitude_kp=pitch_attitude_kp,
        yaw_attitude_kp=yaw_attitude_kp,
        roll_attitude_to_roll_rate_sign=roll_attitude_to_roll_rate_sign,
        pitch_attitude_to_pitch_rate_sign=pitch_attitude_to_pitch_rate_sign,
        yaw_attitude_to_yaw_rate_sign=yaw_attitude_to_yaw_rate_sign,
        attitude_rate_limit_rad_s=attitude_rate_limit_rad_s,
        trajectory_radius_m=trajectory_radius_m,
        trajectory_period_s=trajectory_period_s,
        physical_test_authorized=(stage is not ValidationStage.SOFTWARE_PIPELINE),
        **stage_six_values,
    )


@pytest.mark.parametrize("stage", list(ValidationStage))
def test_each_stage_accepts_complete_configuration(
    stage: ValidationStage,
) -> None:
    valid_config(stage).validate()


def test_physical_stage_rejects_missing_authorization() -> None:
    config = valid_config(ValidationStage.EQUILIBRIUM_THRUST)
    config = ValidationExperimentConfig(
        **{
            field_name: getattr(config, field_name)
            for field_name in config.__dataclass_fields__
            if field_name != "physical_test_authorized"
        },
        physical_test_authorized=False,
    )

    with pytest.raises(ValueError, match="physical_test_authorized"):
        config.validate()


def test_stage_zero_rejects_nonzero_thrust() -> None:
    config = valid_config(ValidationStage.SOFTWARE_PIPELINE)
    config = ValidationExperimentConfig(
        stage=config.stage,
        fixed_position=config.fixed_position,
        fixed_yaw=config.fixed_yaw,
        horizontal_mapping=config.horizontal_mapping,
        equilibrium_thrust=0.1,
        min_thrust=config.min_thrust,
        max_thrust=config.max_thrust,
        horizontal_rate_limit_rad_s=config.horizontal_rate_limit_rad_s,
        heading_rate_limit_rad_s=config.heading_rate_limit_rad_s,
        altitude_feedback_limit=config.altitude_feedback_limit,
    )

    with pytest.raises(ValueError, match="equilibrium_thrust=0.0"):
        config.validate()


def test_stage_one_reference_exercises_body_axis_polarities() -> None:
    config = valid_config(ValidationStage.HORIZONTAL_POLARITY)
    controller = OuterLoopPIDController(config.controller_config())
    program = OuterLoopPIDValidationProgram(controller, config)
    state = DroneState(
        position=np.array(config.fixed_position),
        velocity=np.zeros(3),
        roll=0.0,
        pitch=0.0,
        yaw=config.fixed_yaw,
    )
    program.initialize(state, start_time=10.0)

    reference, command = program.update(
        state=state,
        current_time=10.0,
        dt=0.02,
    )

    # A positive body-x offset at yaw=pi/2 is positive inertial y.
    assert math.isclose(reference.position[0], 1.0, abs_tol=1e-12)
    assert math.isclose(reference.position[1], 2.01, abs_tol=1e-12)
    assert math.isclose(command.roll_rate, 0.0, abs_tol=1e-12)
    assert command.pitch_rate > 0.0
    assert isinstance(command, BodyRateThrustCommand)


def test_controller_config_applies_explicit_command_limits() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)

    controller_config = config.controller_config()

    assert controller_config.horizontal_x.output_min == -0.1
    assert controller_config.horizontal_x.output_max == 0.1
    assert controller_config.heading.output_min == -0.1
    assert controller_config.heading.output_max == 0.1
    assert controller_config.min_thrust == 0.0
    assert controller_config.max_thrust == 0.6
    assert controller_config.equilibrium_thrust == 0.4


def test_stage_two_captures_heading_and_uses_only_attitude_feedback() -> None:
    config = valid_config(ValidationStage.EQUILIBRIUM_THRUST)
    controller_config = config.controller_config()
    controller = OuterLoopPIDController(controller_config)
    program = OuterLoopPIDValidationProgram(controller, config)
    initial_state = DroneState(
        position=np.array([3.0, -2.0, 0.08]),
        velocity=np.zeros(3),
        roll=0.0,
        pitch=0.0,
        yaw=0.25,
    )
    program.initialize(initial_state, start_time=10.0)

    displaced_state = DroneState(
        position=np.array([-20.0, 30.0, 1.0]),
        velocity=np.zeros(3),
        roll=0.2,
        pitch=-0.2,
        yaw=0.15,
    )
    reference, command = program.update(
        state=displaced_state,
        current_time=10.0,
        dt=0.02,
    )

    np.testing.assert_array_equal(reference.position, initial_state.position)
    assert reference.yaw == initial_state.yaw
    assert math.isclose(command.roll_rate, -0.04)
    assert math.isclose(command.pitch_rate, 0.06)
    assert math.isclose(command.yaw_rate, 0.04)
    assert command.thrust == config.ramp_start_thrust

    assert controller_config.horizontal_x.output_max == 0.0
    assert controller_config.horizontal_y.output_max == 0.0
    assert controller_config.altitude.output_min == 0.0
    assert controller_config.altitude.output_max == 0.0
    assert controller_config.heading.output_min == 0.0
    assert controller_config.heading.output_max == 0.0


def test_stage_two_attitude_rate_commands_saturate_and_wrap_yaw() -> None:
    config = valid_config(ValidationStage.EQUILIBRIUM_THRUST)
    controller = OuterLoopPIDController(config.controller_config())
    program = OuterLoopPIDValidationProgram(controller, config)
    initial_state = DroneState(
        position=np.zeros(3),
        velocity=np.zeros(3),
        roll=0.0,
        pitch=0.0,
        yaw=math.pi - 0.1,
    )
    program.initialize(initial_state, start_time=0.0)
    measured_state = DroneState(
        position=np.array([100.0, -100.0, 50.0]),
        velocity=np.ones(3),
        roll=2.0,
        pitch=-2.0,
        yaw=-math.pi + 0.1,
    )

    _, command = program.update(measured_state, current_time=0.0, dt=0.02)

    assert command.roll_rate == -0.1
    assert command.pitch_rate == 0.1
    assert math.isclose(command.yaw_rate, -0.08, abs_tol=1e-12)


@pytest.mark.parametrize(
    ("sign_field", "rate_index"),
    [
        ("roll_attitude_to_roll_rate_sign", 0),
        ("pitch_attitude_to_pitch_rate_sign", 1),
        ("yaw_attitude_to_yaw_rate_sign", 2),
    ],
)
def test_stage_two_attitude_sign_reverses_only_corresponding_rate(
    sign_field: str,
    rate_index: int,
) -> None:
    base_config = valid_config(ValidationStage.EQUILIBRIUM_THRUST)
    initial_state = DroneState(
        position=np.zeros(3),
        velocity=np.zeros(3),
        roll=0.0,
        pitch=0.0,
        yaw=0.3,
    )
    measured_state = DroneState(
        position=np.zeros(3),
        velocity=np.zeros(3),
        roll=0.2,
        pitch=-0.2,
        yaw=0.1,
    )

    def rates(config: ValidationExperimentConfig) -> tuple[float, float, float]:
        program = OuterLoopPIDValidationProgram(
            OuterLoopPIDController(config.controller_config()),
            config,
        )
        program.initialize(initial_state, start_time=0.0)
        _, command = program.update(measured_state, current_time=0.0, dt=0.02)
        return command.roll_rate, command.pitch_rate, command.yaw_rate

    positive_rates = rates(base_config)
    negative_rates = rates(replace(base_config, **{sign_field: -1}))

    for index, (positive, negative) in enumerate(
        zip(positive_rates, negative_rates)
    ):
        if index == rate_index:
            assert negative == -positive
        else:
            assert negative == positive


def test_stage_two_logging_includes_attitude_rates_and_thrust(capsys) -> None:
    state = DroneState(
        position=np.zeros(3),
        velocity=np.zeros(3),
        roll=0.1,
        pitch=-0.2,
        yaw=0.3,
    )
    config = valid_config(ValidationStage.EQUILIBRIUM_THRUST)
    program = OuterLoopPIDValidationProgram(
        OuterLoopPIDController(config.controller_config()),
        config,
    )
    program.initialize(state, start_time=0.0)
    reference, command = program.update(state, current_time=0.0, dt=0.02)

    log_step(0.0, "STAGE_2_EQUILIBRIUM_THRUST", state, reference, command, 0.02)
    output = capsys.readouterr().out

    assert "attitude=(+0.100,-0.200,+0.300)rad" in output
    assert "commanded_body_rates=(-0.020,+0.060,+0.000)rad/s" in output
    assert "thrust=0.1000" in output


def test_stage_two_thrust_ramp_hold_and_completion() -> None:
    config = valid_config(ValidationStage.EQUILIBRIUM_THRUST)
    controller = OuterLoopPIDController(config.controller_config())
    program = OuterLoopPIDValidationProgram(controller, config)
    state = DroneState.zero()
    program.initialize(state, start_time=20.0)

    expected_thrust = (
        (0.0, 0.1),
        (2.0, 0.25),
        (4.0, 0.4),
        (5.5, 0.4),
    )
    for elapsed, expected in expected_thrust:
        _, command = program.update(
            state=state,
            current_time=20.0 + elapsed,
            dt=0.02,
        )
        assert math.isclose(command.thrust, expected)
        assert command.roll_rate == 0.0
        assert command.pitch_rate == 0.0
        assert command.yaw_rate == 0.0
        assert config.min_thrust <= command.thrust <= config.max_thrust

    assert not program.is_complete(25.99)
    assert program.is_complete(26.0)


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"roll_attitude_kp": None}, "roll_attitude_kp"),
        ({"roll_attitude_kp": 0.0}, "roll_attitude_kp must be positive"),
        ({"pitch_attitude_kp": 0.0}, "pitch_attitude_kp must be positive"),
        ({"yaw_attitude_kp": 0.0}, "yaw_attitude_kp must be positive"),
        ({"roll_attitude_to_roll_rate_sign": None}, "roll_attitude_to_roll_rate_sign"),
        ({"roll_attitude_to_roll_rate_sign": 0}, "must be either -1 or 1"),
        ({"pitch_attitude_to_pitch_rate_sign": 2}, "must be either -1 or 1"),
        ({"yaw_attitude_to_yaw_rate_sign": -2}, "must be either -1 or 1"),
        ({"attitude_rate_limit_rad_s": 0.0}, "must be positive"),
        ({"heading_rate_limit_rad_s": 0.0}, "must be positive"),
        ({"ramp_start_thrust": None}, "ramp_start_thrust"),
        ({"ramp_start_thrust": 0.4}, "less than equilibrium_thrust"),
        ({"ramp_start_thrust": -0.1}, "within thrust limits"),
        ({"equilibrium_thrust": 0.7}, "within thrust limits"),
        ({"ramp_duration_s": 0.0}, "must be positive"),
        ({"hold_duration_s": -0.1}, "must be nonnegative"),
        ({"physical_test_authorized": False}, "physical_test_authorized"),
    ],
)
def test_stage_two_rejects_invalid_configuration(
    changes: dict[str, object],
    message: str,
) -> None:
    config = replace(
        valid_config(ValidationStage.EQUILIBRIUM_THRUST),
        **changes,
    )

    with pytest.raises(ValueError, match=message):
        config.validate()


class FixedCommandController:
    def __init__(self, command: BodyRateThrustCommand) -> None:
        self.command = command
        self.update_calls = 0

    def reset(self) -> None:
        pass

    def update(self, state, reference, dt):
        del state, reference, dt
        self.update_calls += 1
        return self.command


def stage_six_state(
    *,
    timestamp: float,
    z: float = 0.0,
    vz: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
) -> DroneState:
    return DroneState(
        position=np.array([1.0, 2.0, z]),
        velocity=np.array([0.0, 0.0, vz]),
        roll=roll,
        pitch=pitch,
        yaw=0.4,
        timestamp=timestamp,
        p=0.01,
        q=-0.02,
        r=0.03,
    )


def test_stage_six_adds_attitude_feedback_and_preserves_yaw_and_thrust() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)
    original = BodyRateThrustCommand(0.05, -0.02, 0.03, 0.45)
    program = OuterLoopPIDValidationProgram(FixedCommandController(original), config)
    state = stage_six_state(timestamp=0.0, roll=0.1, pitch=-0.2)

    command = program._stage_six_command(
        state,
        Reference.hover(1.0, 2.0, 0.0, yaw=0.4),
        0.02,
    )

    assert math.isclose(command.roll_rate, 0.03)
    assert math.isclose(command.pitch_rate, -0.08)
    assert command.yaw_rate == original.yaw_rate
    assert command.thrust == original.thrust
    diagnostics = program.stage_six_diagnostics
    assert diagnostics is not None
    assert diagnostics.position_pid_roll_rate == 0.05
    assert diagnostics.position_pid_pitch_rate == -0.02
    assert diagnostics.heading_pid_yaw_rate == 0.03
    assert math.isclose(diagnostics.roll_attitude_feedback, -0.02)
    assert math.isclose(diagnostics.pitch_attitude_feedback, -0.06)
    assert diagnostics.final_roll_rate == command.roll_rate
    assert diagnostics.final_pitch_rate == command.pitch_rate
    assert diagnostics.final_yaw_rate == 0.03
    assert not diagnostics.roll_saturated
    assert not diagnostics.pitch_saturated


@pytest.mark.parametrize(
    ("sign_field", "rate_index"),
    [
        ("roll_attitude_to_roll_rate_sign", 0),
        ("pitch_attitude_to_pitch_rate_sign", 1),
    ],
)
def test_stage_six_attitude_sign_changes_only_corresponding_axis(
    sign_field: str,
    rate_index: int,
) -> None:
    base = valid_config(ValidationStage.FIXED_HOVER)
    state = stage_six_state(timestamp=0.0, roll=0.1, pitch=-0.1)
    reference = Reference.hover(1.0, 2.0, 0.0, yaw=0.4)

    def command_rates(config):
        zero = BodyRateThrustCommand(0.0, 0.0, 0.0, 0.4)
        program = OuterLoopPIDValidationProgram(FixedCommandController(zero), config)
        command = program._stage_six_command(state, reference, 0.02)
        return command.roll_rate, command.pitch_rate

    positive = command_rates(base)
    reversed_sign = -getattr(base, sign_field)
    negative = command_rates(replace(base, **{sign_field: reversed_sign}))
    for index, (positive_rate, negative_rate) in enumerate(zip(positive, negative)):
        if index == rate_index:
            assert negative_rate == -positive_rate
        else:
            assert negative_rate == positive_rate


def test_stage_six_final_rate_saturation_occurs_after_combination() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)
    original = BodyRateThrustCommand(0.09, -0.09, 0.04, 0.42)
    program = OuterLoopPIDValidationProgram(FixedCommandController(original), config)
    state = stage_six_state(timestamp=0.0, roll=-0.2, pitch=-0.2)

    command = program._stage_six_command(
        state,
        Reference.hover(1.0, 2.0, 0.0, yaw=0.4),
        0.02,
    )

    assert command.roll_rate == 0.1
    assert command.pitch_rate == -0.1
    assert command.yaw_rate == 0.04
    assert command.thrust == 0.42
    assert program.stage_six_diagnostics is not None
    assert program.stage_six_diagnostics.roll_saturated
    assert program.stage_six_diagnostics.pitch_saturated


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"hover_height_m": None}, "hover_height_m"),
        ({"takeoff_velocity_m_s": 0.0}, "takeoff_velocity_m_s must be positive"),
        ({"landing_velocity_m_s": -0.1}, "landing_velocity_m_s must be positive"),
        ({"roll_attitude_kp": 0.0}, "roll_attitude_kp must be positive"),
        ({"pitch_attitude_kp": 0.0}, "pitch_attitude_kp must be positive"),
        ({"roll_attitude_to_roll_rate_sign": 0}, "must be either -1 or 1"),
        ({"pitch_attitude_to_pitch_rate_sign": 2}, "must be either -1 or 1"),
        ({"attitude_rate_limit_rad_s": 0.0}, "must be positive"),
        ({"max_tilt_rad": math.pi / 2.0}, "less than pi/2"),
        ({"landing_altitude_tolerance_m": 0.5}, "less than hover_height_m"),
        ({"maximum_flight_duration_s": 8.0}, "must exceed"),
        ({"physical_test_authorized": False}, "physical_test_authorized"),
    ],
)
def test_stage_six_rejects_invalid_configuration(changes, message) -> None:
    with pytest.raises(ValueError, match=message):
        replace(valid_config(ValidationStage.FIXED_HOVER), **changes).validate()


def test_stage_six_takeoff_hover_landing_and_completion() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)
    program = OuterLoopPIDValidationProgram(
        OuterLoopPIDController(config.controller_config()),
        config,
    )
    initial = stage_six_state(timestamp=10.0)
    program.initialize(initial, start_time=10.0)

    samples = (
        (10.0, 0.0, "STAGE_6_TAKEOFF"),
        (11.5, 0.25, "STAGE_6_TAKEOFF"),
        (13.0, 0.5, "STAGE_6_HOVER"),
        (15.0, 0.5, "STAGE_6_LANDING"),
        (16.5, 0.25, "STAGE_6_LANDING"),
    )
    for current_time, altitude, phase in samples:
        state = stage_six_state(timestamp=current_time, z=altitude)
        reference, _ = program.update(state, current_time, 0.02)
        assert program.get_phase_name(current_time) == phase
        assert math.isclose(reference.position[2], altitude, abs_tol=1e-12)
        assert not program.is_complete(current_time)

    takeoff_midpoint = program._reference(11.5)
    landing_midpoint = program._reference(16.5)
    assert math.isclose(takeoff_midpoint.velocity[2], 0.25)
    assert math.isclose(landing_midpoint.velocity[2], -0.25)

    moving = stage_six_state(timestamp=18.0, z=0.0, vz=0.2)
    program.update(moving, 18.0, 0.02)
    assert not program.is_complete(18.0)

    landed = stage_six_state(timestamp=18.1, z=0.01, vz=0.05)
    program.update(landed, 18.1, 0.02)
    assert program.get_phase_name(18.1) == "STAGE_6_COMPLETE"
    assert program.is_complete(18.1)


def test_stage_six_reference_is_continuous_at_phase_boundaries() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)
    program = OuterLoopPIDValidationProgram(
        OuterLoopPIDController(config.controller_config()),
        config,
    )
    program.initialize(stage_six_state(timestamp=0.0), start_time=0.0)
    epsilon = 1.0e-6

    for boundary in (3.0, 5.0, 8.0):
        before = program._reference(boundary - epsilon)
        at = program._reference(boundary)
        after = program._reference(boundary + epsilon)
        assert abs(before.position[2] - at.position[2]) < 1.0e-9
        assert abs(after.position[2] - at.position[2]) < 1.0e-9
        assert abs(before.velocity[2] - at.velocity[2]) < 2.0e-6
        assert abs(after.velocity[2] - at.velocity[2]) < 2.0e-6


def test_stage_six_rejects_stale_and_nonfinite_state() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)
    program = OuterLoopPIDValidationProgram(
        OuterLoopPIDController(config.controller_config()),
        config,
    )
    program.initialize(stage_six_state(timestamp=0.0), start_time=0.0)

    with pytest.raises(RuntimeError, match="stale"):
        program.update(stage_six_state(timestamp=0.0), 1.0, 0.02)

    nonfinite = stage_six_state(timestamp=0.1)
    nonfinite.position[0] = math.nan
    with pytest.raises(RuntimeError, match="nonfinite"):
        program.update(nonfinite, 0.1, 0.02)


def test_stage_six_rejects_safety_limit_and_timing_violations() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)

    def initialized_program() -> OuterLoopPIDValidationProgram:
        program = OuterLoopPIDValidationProgram(
            OuterLoopPIDController(config.controller_config()),
            config,
        )
        program.initialize(stage_six_state(timestamp=0.0), start_time=0.0)
        return program

    with pytest.raises(RuntimeError, match="tilt limit"):
        initialized_program().update(
            stage_six_state(timestamp=0.1, roll=0.6),
            0.1,
            0.02,
        )

    horizontal_error = stage_six_state(timestamp=0.1)
    horizontal_error.position[0] = 3.0
    with pytest.raises(RuntimeError, match="horizontal position error"):
        initialized_program().update(horizontal_error, 0.1, 0.02)

    with pytest.raises(RuntimeError, match="altitude error"):
        initialized_program().update(
            stage_six_state(timestamp=1.0, z=-1.0),
            1.0,
            0.02,
        )

    with pytest.raises(TimeoutError, match="control-loop timeout"):
        initialized_program().update(
            stage_six_state(timestamp=0.1),
            0.1,
            0.2,
        )

    with pytest.raises(TimeoutError, match="validated landing"):
        initialized_program().is_complete(10.0)


@pytest.mark.parametrize(
    "stage",
    [
        ValidationStage.SOFTWARE_PIPELINE,
        ValidationStage.HORIZONTAL_POLARITY,
        ValidationStage.HORIZONTAL_STABILIZATION,
        ValidationStage.HORIZONTAL_ALTITUDE,
        ValidationStage.HEADING,
        ValidationStage.SLOW_TRAJECTORY,
    ],
)
def test_non_special_stages_keep_direct_controller_path(stage) -> None:
    config = valid_config(stage)
    expected = BodyRateThrustCommand(0.01, 0.02, 0.03, 0.04)
    controller = FixedCommandController(expected)
    program = OuterLoopPIDValidationProgram(controller, config)
    state = DroneState(
        position=np.array([1.0, 2.0, 0.5]),
        velocity=np.zeros(3),
        roll=0.0,
        pitch=0.0,
        yaw=math.pi / 2.0,
        timestamp=0.0,
    )
    program.initialize(state, start_time=0.0)

    _, command = program.update(state, current_time=0.0, dt=0.02)

    assert command == expected
    assert controller.update_calls == 1


def test_stage_six_logging_distinguishes_measured_and_commanded_rates(capsys) -> None:
    state = stage_six_state(timestamp=1.0, z=0.2)
    reference = Reference.hover(1.0, 2.0, 0.25, yaw=0.4)
    command = BodyRateThrustCommand(0.04, -0.05, 0.01, 0.43)
    diagnostics = StageSixCommandDiagnostics(
        position_pid_roll_rate=0.02,
        position_pid_pitch_rate=-0.03,
        heading_pid_yaw_rate=0.01,
        roll_attitude_feedback=0.02,
        pitch_attitude_feedback=-0.02,
        final_roll_rate=0.04,
        final_pitch_rate=-0.05,
        final_yaw_rate=0.01,
        roll_saturated=False,
        pitch_saturated=False,
    )

    log_step(
        1.0,
        "STAGE_6_TAKEOFF",
        state,
        reference,
        command,
        0.02,
        diagnostics,
    )
    output = capsys.readouterr().out

    assert "measured_body_rates=(+0.010,-0.020,+0.030)rad/s" in output
    assert "outer_loop_body_rate_cmd=(+0.020,-0.030,+0.010)rad/s" in output
    assert "attitude_rate_feedback=(+0.020,-0.020)rad/s" in output
    assert "commanded_body_rates=(+0.040,-0.050,+0.010)rad/s" in output
    assert "wrapper_saturation=(roll=False,pitch=False)" in output
    assert "normalized_thrust=0.4300" in output


def test_command_transmission_failure_still_stops_and_disarms() -> None:
    events: list[str] = []

    class Provider:
        def start(self):
            events.append("provider_start")

        def get_state(self):
            return stage_six_state(timestamp=0.0)

        def stop(self):
            events.append("provider_stop")

    class FailingAdapter:
        def send(self, command):
            del command
            raise OSError("simulated command transport failure")

        def stop(self):
            events.append("command_stop")

    class Program:
        def initialize(self, initial_state, start_time):
            del initial_state, start_time

        def is_complete(self, current_time):
            del current_time
            return False

        def get_phase_name(self, current_time):
            del current_time
            return "STAGE_6_TAKEOFF"

        def update(self, state, current_time, dt):
            del state, current_time, dt
            return (
                Reference.hover(1.0, 2.0, 0.0),
                BodyRateThrustCommand.zero(),
            )

    runner = UniversalRunner(
        state_provider=Provider(),
        command_adapter=FailingAdapter(),
        flight_program=Program(),
        arm=lambda: events.append("arm"),
        disarm=lambda: events.append("disarm"),
        config=UniversalRunnerConfig(arming_wait_s=0.0),
    )

    with pytest.raises(OSError, match="transport failure"):
        runner.run()

    assert events == [
        "provider_start",
        "arm",
        "command_stop",
        "disarm",
        "provider_stop",
    ]


class InitializationCaptureProgram:
    def __init__(self) -> None:
        self.pre_arming_state = None
        self.post_arming_state = None
        self.initial_state = None

    def report_initialization_states(
        self,
        *,
        pre_arming_state,
        post_arming_state,
    ) -> None:
        self.pre_arming_state = pre_arming_state
        self.post_arming_state = post_arming_state

    def initialize(self, initial_state, start_time):
        del start_time
        self.initial_state = initial_state

    def select_initialization_state(
        self,
        *,
        pre_arming_state,
        post_arming_state,
    ):
        del pre_arming_state
        return post_arming_state

    def is_complete(self, current_time):
        del current_time
        return True

    def get_phase_name(self, current_time):
        raise AssertionError(f"unexpected phase request at {current_time}")

    def update(self, state, current_time, dt):
        raise AssertionError(f"unexpected update: {state}, {current_time}, {dt}")


class StateSequenceProvider:
    def __init__(self, states) -> None:
        self.states = iter(states)
        self.stopped = False

    def start(self):
        pass

    def get_state(self):
        return next(self.states)

    def stop(self):
        self.stopped = True


class RecordingAdapter:
    def __init__(self) -> None:
        self.commands = []
        self.stopped = False

    def send(self, command):
        self.commands.append(command)

    def stop(self):
        self.stopped = True


@pytest.mark.parametrize("post_x", [1.0, 1.5])
def test_runner_initializes_from_post_arming_state_for_stable_or_jumped_position(
    post_x: float,
) -> None:
    pre = stage_six_state(timestamp=1.0)
    post = stage_six_state(timestamp=2.0)
    post.position[0] = post_x
    provider = StateSequenceProvider([pre, post])
    adapter = RecordingAdapter()
    program = InitializationCaptureProgram()
    runner = UniversalRunner(
        state_provider=provider,
        command_adapter=adapter,
        flight_program=program,
        arm=lambda: None,
        disarm=lambda: None,
        config=UniversalRunnerConfig(arming_wait_s=0.0),
    )

    runner.run()

    assert program.pre_arming_state is pre
    assert program.post_arming_state is post
    assert program.initial_state is post
    assert program.initial_state is not pre
    assert adapter.commands == []
    assert adapter.stopped
    assert provider.stopped


def test_stage_six_reference_uses_post_arming_state_after_position_jump() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)
    program = OuterLoopPIDValidationProgram(
        OuterLoopPIDController(config.controller_config()),
        config,
    )
    pre = stage_six_state(timestamp=100.0)
    post = stage_six_state(timestamp=101.0)
    post.position[:2] = [1.5, 1.7]

    program.report_initialization_states(
        pre_arming_state=pre,
        post_arming_state=post,
    )
    program.initialize(post, start_time=101.0)
    reference, _ = program.update(post, current_time=101.0, dt=0.02)

    np.testing.assert_array_equal(reference.position[:2], post.position[:2])


@pytest.mark.parametrize(
    "stage",
    [
        ValidationStage.SOFTWARE_PIPELINE,
        ValidationStage.HORIZONTAL_POLARITY,
        ValidationStage.EQUILIBRIUM_THRUST,
        ValidationStage.HORIZONTAL_STABILIZATION,
        ValidationStage.HORIZONTAL_ALTITUDE,
        ValidationStage.HEADING,
        ValidationStage.SLOW_TRAJECTORY,
    ],
)
def test_other_stages_preserve_pre_arming_initialization_state(stage) -> None:
    config = valid_config(stage)
    program = OuterLoopPIDValidationProgram(
        OuterLoopPIDController(config.controller_config()),
        config,
    )
    pre = DroneState.zero(timestamp=1.0)
    post = DroneState.zero(timestamp=2.0)
    post.position[0] = 0.5

    selected = program.select_initialization_state(
        pre_arming_state=pre,
        post_arming_state=post,
    )

    assert selected is pre


def test_stage_six_rejects_stale_post_arming_measurement() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)
    program = OuterLoopPIDValidationProgram(
        OuterLoopPIDController(config.controller_config()),
        config,
    )
    pre = stage_six_state(timestamp=1.0)
    post = stage_six_state(timestamp=2.0)
    program.report_initialization_states(
        pre_arming_state=pre,
        post_arming_state=post,
    )

    with pytest.raises(RuntimeError, match="stale state data"):
        program.initialize(post, start_time=3.0)


def test_stage_six_rejects_post_arming_state_without_new_timestamp() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)
    program = OuterLoopPIDValidationProgram(
        OuterLoopPIDController(config.controller_config()),
        config,
    )
    pre = stage_six_state(timestamp=1.0)
    post = stage_six_state(timestamp=1.0)

    with pytest.raises(RuntimeError, match="did not receive a newer state"):
        program.report_initialization_states(
            pre_arming_state=pre,
            post_arming_state=post,
        )


def test_stage_six_initialization_and_first_tracking_diagnostics(capsys) -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)
    program = OuterLoopPIDValidationProgram(
        OuterLoopPIDController(config.controller_config()),
        config,
    )
    pre = stage_six_state(timestamp=10.0)
    post = stage_six_state(timestamp=11.0)
    post.position[:2] = [1.02, 1.98]

    program.report_initialization_states(
        pre_arming_state=pre,
        post_arming_state=post,
    )
    program.initialize(post, start_time=11.0)
    program.update(post, current_time=11.0, dt=0.02)
    output = capsys.readouterr().out

    assert "pre_arming_position=" in output
    assert "pre_arming_timestamp=10.000000s" in output
    assert "post_arming_position=" in output
    assert "post_arming_timestamp=11.000000s" in output
    assert "arming_wait_position_delta=" in output
    assert "initial_reference=" in output
    assert "phase=TAKEOFF" in output
    assert "horizontal_error=0.000000m" in output


def test_stage_six_tracking_exception_reports_values() -> None:
    config = valid_config(ValidationStage.FIXED_HOVER)
    program = OuterLoopPIDValidationProgram(
        OuterLoopPIDController(config.controller_config()),
        config,
    )
    initial = stage_six_state(timestamp=0.0)
    program.initialize(initial, start_time=0.0)
    jumped = stage_six_state(timestamp=0.1)
    jumped.position[0] += 1.1

    with pytest.raises(RuntimeError) as error:
        program.update(jumped, current_time=0.1, dt=0.02)

    message = str(error.value)
    assert "error=1.100000m" in message
    assert "limit=1.000000m" in message
    assert "reference=" in message
    assert "measured=" in message
