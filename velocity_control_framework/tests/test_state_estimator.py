from __future__ import annotations

import numpy as np

from velocity_control_framework.controllers import VelocityAltitudePIDController
from velocity_control_framework.estimators import PassthroughStateEstimator
from velocity_control_framework.interfaces import (
    DroneState,
    EstimatedState,
    MeasuredState,
    Reference,
)


def make_estimated_state() -> EstimatedState:
    return EstimatedState(
        position=np.array([1.0, 2.0, 3.0]),
        velocity=np.array([0.1, 0.2, 0.3]),
        roll=0.4,
        pitch=0.5,
        yaw=0.6,
        timestamp=7.0,
        p=0.7,
        q=0.8,
        r=0.9,
    )


def test_passthrough_copies_estimated_state_to_drone_state() -> None:
    estimated = make_estimated_state()

    state = PassthroughStateEstimator().estimate(
        estimated_state=estimated,
        measured_state=MeasuredState(),
    )

    assert isinstance(state, DroneState)
    np.testing.assert_array_equal(state.position, estimated.position)
    np.testing.assert_array_equal(state.velocity, estimated.velocity)
    assert state.roll == estimated.roll
    assert state.pitch == estimated.pitch
    assert state.yaw == estimated.yaw
    assert state.timestamp == estimated.timestamp
    assert (state.p, state.q, state.r) == (
        estimated.p,
        estimated.q,
        estimated.r,
    )

    assert state.position is not estimated.position
    assert state.velocity is not estimated.velocity


def test_passthrough_intentionally_ignores_measured_state() -> None:
    estimated = make_estimated_state()
    measured = MeasuredState(
        position=np.array([-1.0, -2.0, -3.0]),
        velocity=np.array([-0.1, -0.2, -0.3]),
        roll=-0.4,
        pitch=-0.5,
        yaw=-0.6,
        timestamp=8.0,
        p=-0.7,
        q=-0.8,
        r=-0.9,
    )

    state = PassthroughStateEstimator().estimate(estimated, measured)

    np.testing.assert_array_equal(state.position, estimated.position)
    np.testing.assert_array_equal(state.velocity, estimated.velocity)
    assert (state.roll, state.pitch, state.yaw) == (
        estimated.roll,
        estimated.pitch,
        estimated.yaw,
    )
    assert (state.p, state.q, state.r) == (0.7, 0.8, 0.9)
    assert state.timestamp == estimated.timestamp


def test_measured_state_allows_every_field_to_be_unavailable() -> None:
    measured = MeasuredState()

    assert measured.position is None
    assert measured.velocity is None
    assert measured.roll is None
    assert measured.pitch is None
    assert measured.yaw is None
    assert measured.timestamp is None
    assert measured.p is None
    assert measured.q is None
    assert measured.r is None


def test_passthrough_output_preserves_controller_facing_behavior() -> None:
    estimated = EstimatedState(
        position=np.array([0.0, 0.0, 0.5]),
        velocity=np.zeros(3),
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        timestamp=1.0,
        p=1.0,
        q=2.0,
        r=3.0,
    )
    state = PassthroughStateEstimator().estimate(
        estimated,
        MeasuredState(),
    )
    reference = Reference.hover(
        x=0.0,
        y=0.0,
        z=0.5,
        yaw=0.0,
        timestamp=1.0,
    )

    controller = VelocityAltitudePIDController()
    command = controller.update(state, reference, dt=0.02)

    assert command.roll == 0.0
    assert command.pitch == 0.0
    assert command.yaw_rate == 0.0
    assert command.thrust == controller.config.hover_thrust
