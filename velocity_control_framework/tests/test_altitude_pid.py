from __future__ import annotations

import math

import numpy as np
import pytest

from velocity_control_framework.controllers import (
    AltitudePIDConfig,
    AltitudePIDController,
)
from velocity_control_framework.interfaces import (
    AttitudeThrustCommand,
    DroneState,
    Reference,
)


def make_state(
    z: float = 0.5,
    vz: float = 0.0,
) -> DroneState:
    return DroneState(
        position=np.array(
            [0.0, 0.0, z],
            dtype=np.float64,
        ),
        velocity=np.array(
            [0.0, 0.0, vz],
            dtype=np.float64,
        ),
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        timestamp=1.0,
    )


def make_reference(
    z: float = 0.5,
    vz: float = 0.0,
    yaw_rate: float = 0.0,
) -> Reference:
    return Reference(
        position=np.array(
            [0.0, 0.0, z],
            dtype=np.float64,
        ),
        velocity=np.array(
            [0.0, 0.0, vz],
            dtype=np.float64,
        ),
        yaw=0.0,
        yaw_rate=yaw_rate,
        timestamp=1.0,
    )


def test_zero_error_returns_hover_thrust() -> None:
    config = AltitudePIDConfig(
        hover_command=31500,
    )
    controller = AltitudePIDController(config)

    command = controller.update(
        state=make_state(z=0.5, vz=0.0),
        reference=make_reference(z=0.5, vz=0.0),
        dt=0.02,
    )

    assert isinstance(
        command,
        AttitudeThrustCommand,
    )

    assert command.roll == 0.0
    assert command.pitch == 0.0
    assert command.yaw_rate == 0.0

    assert command.thrust == pytest.approx(
        31500 / 65535,
    )


def test_positive_altitude_error_increases_thrust() -> None:
    controller = AltitudePIDController()

    command = controller.update(
        state=make_state(z=0.4),
        reference=make_reference(z=0.5),
        dt=0.02,
    )

    assert (
        command.thrust
        > controller.config.hover_thrust
    )


def test_negative_altitude_error_decreases_thrust() -> None:
    controller = AltitudePIDController()

    command = controller.update(
        state=make_state(z=0.6),
        reference=make_reference(z=0.5),
        dt=0.02,
    )

    assert (
        command.thrust
        < controller.config.hover_thrust
    )


def test_upward_velocity_reduces_thrust() -> None:
    controller = AltitudePIDController()

    stationary_command = controller.update(
        state=make_state(z=0.5, vz=0.0),
        reference=make_reference(z=0.5, vz=0.0),
        dt=0.02,
    )

    upward_command = controller.update(
        state=make_state(z=0.5, vz=0.2),
        reference=make_reference(z=0.5, vz=0.0),
        dt=0.02,
    )

    assert (
        upward_command.thrust
        < stationary_command.thrust
    )


def test_downward_velocity_increases_thrust() -> None:
    controller = AltitudePIDController()

    command = controller.update(
        state=make_state(z=0.5, vz=-0.2),
        reference=make_reference(z=0.5, vz=0.0),
        dt=0.02,
    )

    assert (
        command.thrust
        > controller.config.hover_thrust
    )


def test_yaw_rate_is_passed_through() -> None:
    controller = AltitudePIDController()

    command = controller.update(
        state=make_state(),
        reference=make_reference(
            yaw_rate=0.25,
        ),
        dt=0.02,
    )

    assert command.yaw_rate == pytest.approx(0.25)


def test_thrust_is_limited() -> None:
    config = AltitudePIDConfig(
        kp_z=100.0,
        max_thrust=0.60,
    )
    controller = AltitudePIDController(config)

    command = controller.update(
        state=make_state(z=0.0),
        reference=make_reference(z=10.0),
        dt=0.02,
    )

    assert command.thrust == pytest.approx(0.60)


def test_reset_clears_integral() -> None:
    config = AltitudePIDConfig(
        ki_z=0.2,
    )
    controller = AltitudePIDController(config)

    controller.update(
        state=make_state(z=0.4),
        reference=make_reference(z=0.5),
        dt=0.1,
    )

    assert controller.integral_error > 0.0

    controller.reset()

    assert controller.integral_error == 0.0


def test_invalid_dt_is_rejected() -> None:
    controller = AltitudePIDController()

    with pytest.raises(
        ValueError,
        match="dt must be positive",
    ):
        controller.update(
            state=make_state(),
            reference=make_reference(),
            dt=0.0,
        )


def test_nominal_hover_command() -> None:
    controller = AltitudePIDController()

    assert controller.config.hover_command == 31500
    assert controller.config.hover_thrust == pytest.approx(
        31500 / 65535,
    )

    assert math.isclose(
        controller.config.hover_thrust,
        0.4806591897459373,
    )