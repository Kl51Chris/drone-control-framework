from __future__ import annotations

import math

import numpy as np
import pytest

from velocity_control_framework.controllers import (
    AttitudeControlConfig,
    HorizontalAccelerationMapping,
    HorizontalCommandMapping,
    HorizontalControlConfig,
    OuterLoopPIDConfig,
    OuterLoopPIDController,
    PIDConfig,
)
from velocity_control_framework.interfaces import DroneState, Reference


def make_state(
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    yaw: float = 0.0,
) -> DroneState:
    return DroneState(
        position=np.array([x, y, z], dtype=np.float64),
        velocity=np.zeros(3, dtype=np.float64),
        roll=0.0,
        pitch=0.0,
        yaw=yaw,
    )


def make_reference(
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    yaw: float = 0.0,
) -> Reference:
    return Reference.hover(x=x, y=y, z=z, yaw=yaw)


def make_config(
    *,
    horizontal_x: PIDConfig | None = None,
    horizontal_y: PIDConfig | None = None,
    altitude: PIDConfig | None = None,
    heading: PIDConfig | None = None,
    equilibrium_thrust: float = 0.4,
    min_thrust: float = 0.0,
    max_thrust: float = 1.0,
    body_x_to_pitch_rate_sign: int = 1,
    body_y_to_roll_rate_sign: int = 1,
) -> OuterLoopPIDConfig:
    return OuterLoopPIDConfig(
        horizontal_mapping=HorizontalCommandMapping(
            body_x_to_pitch_rate_sign=body_x_to_pitch_rate_sign,
            body_y_to_roll_rate_sign=body_y_to_roll_rate_sign,
        ),
        horizontal_x=horizontal_x or PIDConfig(),
        horizontal_y=horizontal_y or PIDConfig(),
        altitude=altitude or PIDConfig(),
        heading=heading or PIDConfig(),
        equilibrium_thrust=equilibrium_thrust,
        min_thrust=min_thrust,
        max_thrust=max_thrust,
    )


def test_zero_tracking_error_produces_zero_rates() -> None:
    controller = OuterLoopPIDController(make_config())

    command = controller.update(
        make_state(x=1.0, y=-2.0, z=0.5, yaw=0.3),
        make_reference(x=1.0, y=-2.0, z=0.5, yaw=0.3),
        dt=0.02,
    )

    assert command.roll_rate == 0.0
    assert command.pitch_rate == 0.0
    assert command.yaw_rate == 0.0


def test_zero_altitude_error_produces_equilibrium_thrust() -> None:
    controller = OuterLoopPIDController(
        make_config(
            altitude=PIDConfig(
                kp=0.2,
                ki=0.1,
                kd=0.1,
                output_min=-0.2,
                output_max=0.2,
            ),
            equilibrium_thrust=0.47,
        )
    )

    command = controller.update(
        make_state(z=0.8),
        make_reference(z=0.8),
        dt=0.02,
    )

    assert command.thrust == 0.47


def test_positive_and_negative_altitude_error_adjust_thrust() -> None:
    config = make_config(
        altitude=PIDConfig(
            kp=0.2,
            output_min=-0.2,
            output_max=0.2,
        ),
        equilibrium_thrust=0.4,
    )

    above_command = OuterLoopPIDController(config).update(
        make_state(z=1.0),
        make_reference(z=1.5),
        dt=0.02,
    )
    below_command = OuterLoopPIDController(config).update(
        make_state(z=1.0),
        make_reference(z=0.5),
        dt=0.02,
    )

    assert math.isclose(above_command.thrust, 0.5)
    assert math.isclose(below_command.thrust, 0.3)


def test_heading_error_wraps_across_pi_boundary() -> None:
    controller = OuterLoopPIDController(
        make_config(
            heading=PIDConfig(
                kp=1.0,
                output_min=-1.0,
                output_max=1.0,
            )
        )
    )

    command = controller.update(
        make_state(yaw=math.pi - 0.1),
        make_reference(yaw=-math.pi + 0.1),
        dt=0.02,
    )

    assert math.isclose(command.yaw_rate, 0.2, abs_tol=1e-12)


def test_inertial_error_rotates_into_body_frame_before_control() -> None:
    controller = OuterLoopPIDController(
        make_config(
            horizontal_x=PIDConfig(
                kp=1.0,
                output_min=-2.0,
                output_max=2.0,
            ),
            horizontal_y=PIDConfig(
                kp=1.0,
                output_min=-2.0,
                output_max=2.0,
            ),
        )
    )

    command = controller.update(
        make_state(yaw=math.pi / 2.0),
        make_reference(x=1.0),
        dt=0.02,
    )

    assert math.isclose(command.pitch_rate, 0.0, abs_tol=1e-12)
    assert math.isclose(command.roll_rate, -1.0, abs_tol=1e-12)


def test_all_commands_saturate_at_configured_limits() -> None:
    controller = OuterLoopPIDController(
        make_config(
            horizontal_x=PIDConfig(
                kp=10.0,
                output_min=-0.2,
                output_max=0.2,
            ),
            horizontal_y=PIDConfig(
                kp=10.0,
                output_min=-0.3,
                output_max=0.3,
            ),
            altitude=PIDConfig(
                kp=10.0,
                output_min=-0.2,
                output_max=0.2,
            ),
            heading=PIDConfig(
                kp=10.0,
                output_min=-0.4,
                output_max=0.4,
            ),
            equilibrium_thrust=0.5,
            min_thrust=0.2,
            max_thrust=0.55,
            body_x_to_pitch_rate_sign=-1,
            body_y_to_roll_rate_sign=1,
        )
    )

    command = controller.update(
        make_state(),
        make_reference(x=10.0, y=10.0, z=10.0, yaw=1.0),
        dt=0.02,
    )

    assert command.roll_rate == 0.3
    assert command.pitch_rate == -0.2
    assert command.yaw_rate == 0.4
    assert command.thrust == 0.55


def test_pid_operates_with_zero_integral_gain() -> None:
    controller = OuterLoopPIDController(
        make_config(
            horizontal_x=PIDConfig(
                kp=0.5,
                ki=0.0,
                kd=0.0,
                output_min=-1.0,
                output_max=1.0,
            )
        )
    )

    first = controller.update(
        make_state(),
        make_reference(x=1.0),
        dt=0.02,
    )
    second = controller.update(
        make_state(),
        make_reference(x=1.0),
        dt=0.02,
    )

    assert first.pitch_rate == 0.5
    assert second.pitch_rate == 0.5


def make_cascaded_config(**changes) -> OuterLoopPIDConfig:
    values = {
        "horizontal_mapping": HorizontalAccelerationMapping(-1, -1),
        "horizontal_x": PIDConfig(0.4, 0.0, 0.8, -0.4, 0.4),
        "horizontal_y": PIDConfig(0.4, 0.0, 0.8, -0.4, 0.4),
        "altitude": PIDConfig(0.06, 0.0, 0.06, -0.045, 0.025),
        "heading": PIDConfig(2.0, 0.0, 0.0, -0.35, 0.35),
        "horizontal": HorizontalControlConfig(0.4, 0.08),
        "attitude": AttitudeControlConfig(),
        "equilibrium_thrust": 0.425,
        "min_thrust": 0.38,
        "max_thrust": 0.45,
    }
    values.update(changes)
    return OuterLoopPIDConfig(**values)


def cascaded_state(
    *,
    position=(0.0, 0.0, 0.0),
    velocity=(0.0, 0.0, 0.0),
    roll=0.0,
    pitch=0.0,
    yaw=0.0,
) -> DroneState:
    return DroneState(
        position=np.array(position, dtype=np.float64),
        velocity=np.array(velocity, dtype=np.float64),
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


def cascaded_reference(
    *,
    position=(0.0, 0.0, 0.0),
    velocity=(0.0, 0.0, 0.0),
    yaw=0.0,
) -> Reference:
    return Reference(
        position=np.array(position, dtype=np.float64),
        velocity=np.array(velocity, dtype=np.float64),
        yaw=yaw,
    )


def test_stage5_candidate_matches_documented_experimental_values() -> None:
    config = OuterLoopPIDConfig.stage5_hardware_candidate()

    assert isinstance(config.horizontal_mapping, HorizontalAccelerationMapping)
    assert config.horizontal_mapping.body_x_acceleration_to_pitch_sign == -1
    assert config.horizontal_mapping.body_y_acceleration_to_roll_sign == -1
    assert config.horizontal_x.kp == 0.4
    assert config.horizontal_x.kd == 0.8
    assert config.altitude.kp == 0.06
    assert config.altitude.kd == 0.06
    assert config.equilibrium_thrust == 0.425


@pytest.mark.parametrize(
    ("reference_position", "state_velocity", "rate_name", "expected_sign"),
    [
        ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), "pitch_rate", 1),
        ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0), "pitch_rate", -1),
        ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0), "roll_rate", -1),
        ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0), "roll_rate", 1),
        ((0.0, 0.0, 0.0), (-0.2, 0.0, 0.0), "pitch_rate", 1),
        ((0.0, 0.0, 0.0), (0.0, -0.2, 0.0), "roll_rate", -1),
    ],
)
def test_cascaded_xy_position_and_velocity_errors_command_restoring_directions(
    reference_position,
    state_velocity,
    rate_name,
    expected_sign,
) -> None:
    controller = OuterLoopPIDController(make_cascaded_config())
    command = controller.update(
        cascaded_state(velocity=state_velocity),
        cascaded_reference(position=reference_position),
        0.02,
    )

    assert math.copysign(1.0, getattr(command, rate_name)) == expected_sign


def test_cascaded_xy_acceleration_is_rotated_world_to_body() -> None:
    controller = OuterLoopPIDController(make_cascaded_config())
    command = controller.update(
        cascaded_state(yaw=math.pi / 2.0),
        cascaded_reference(position=(1.0, 0.0, 0.0)),
        0.02,
    )

    assert math.isclose(command.pitch_rate, 0.0, abs_tol=1e-12)
    assert command.roll_rate > 0.0


def test_attitude_loop_levels_roll_and_uses_cflib_pitch_command_sign() -> None:
    controller = OuterLoopPIDController(make_cascaded_config())
    command = controller.update(
        cascaded_state(roll=0.05, pitch=0.05),
        cascaded_reference(),
        0.02,
    )

    assert math.isclose(command.roll_rate, -0.2)
    assert math.isclose(command.pitch_rate, 0.2)


def test_altitude_pd_uses_zero_reference_velocity_damping_by_default() -> None:
    controller = OuterLoopPIDController(make_cascaded_config())
    command = controller.update(
        cascaded_state(velocity=(0.0, 0.0, 0.1)),
        cascaded_reference(velocity=(0.0, 0.0, 0.1)),
        0.02,
    )

    assert math.isclose(command.thrust, 0.425 - 0.06 * 0.1)


def test_altitude_velocity_reference_tracking_is_configurable() -> None:
    config = make_cascaded_config(altitude_velocity_reference_tracking=True)
    command = OuterLoopPIDController(config).update(
        cascaded_state(velocity=(0.0, 0.0, 0.1)),
        cascaded_reference(velocity=(0.0, 0.0, 0.1)),
        0.02,
    )

    assert command.thrust == config.equilibrium_thrust


def test_cascaded_limits_attitude_rates_and_thrust_authority() -> None:
    controller = OuterLoopPIDController(make_cascaded_config())
    command = controller.update(
        cascaded_state(position=(0.0, 0.0, -10.0)),
        cascaded_reference(position=(10.0, 10.0, 10.0), yaw=2.0),
        0.02,
    )

    assert abs(command.roll_rate) <= 0.35
    assert abs(command.pitch_rate) <= 0.35
    assert command.yaw_rate == 0.35
    assert command.thrust == 0.45


def test_altitude_integrator_does_not_wind_up_against_thrust_limit() -> None:
    config = make_cascaded_config(
        altitude=PIDConfig(0.0, 1.0, 0.0, -1.0, 1.0),
    )
    controller = OuterLoopPIDController(config)

    for _ in range(10):
        saturated = controller.update(
            cascaded_state(),
            cascaded_reference(position=(0.0, 0.0, 1.0)),
            0.1,
        )
        assert saturated.thrust == config.max_thrust

    recovering = controller.update(
        cascaded_state(position=(0.0, 0.0, 0.1)),
        cascaded_reference(),
        0.1,
    )

    assert math.isclose(recovering.thrust, 0.415)
