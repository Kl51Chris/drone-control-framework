from __future__ import annotations

import math

import numpy as np
import pytest

from experiments.outer_loop_pid_validation import (
    PIDGains,
    OuterLoopPIDValidationProgram,
    ValidationExperimentConfig,
    ValidationStage,
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
    horizontal_gains = PIDGains()
    altitude_gains = PIDGains()
    heading_gains = PIDGains()
    equilibrium_thrust = 0.0
    polarity_offset_m = None
    trajectory_radius_m = None
    trajectory_period_s = None

    if stage is ValidationStage.HORIZONTAL_POLARITY:
        horizontal_gains = PIDGains(kp=0.1)
        polarity_offset_m = 0.01
    elif stage is ValidationStage.EQUILIBRIUM_THRUST:
        equilibrium_thrust = 0.4
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
        horizontal_mapping=HorizontalCommandMapping(
            body_x_to_pitch_rate_sign=1,
            body_y_to_roll_rate_sign=-1,
        ),
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
