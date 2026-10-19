from __future__ import annotations

from dataclasses import replace
import math

import numpy as np
import pytest

from experiments.outer_loop_pid_validation import (
    PIDGains,
    OuterLoopPIDValidationProgram,
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
)


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
    assert "rates=(-0.020,+0.060,+0.000)rad/s" in output
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
