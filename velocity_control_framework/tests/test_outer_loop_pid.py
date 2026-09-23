from __future__ import annotations

import math

import numpy as np

from velocity_control_framework.controllers import (
    HorizontalCommandMapping,
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
