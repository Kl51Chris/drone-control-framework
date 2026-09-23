from __future__ import annotations

import numpy as np

from legacy.controllers import VelocityAltitudePIDController
from velocity_control_framework.estimators import PassthroughStateEstimator
from velocity_control_framework.interfaces import (
    EstimatedState,
    MeasuredState,
    Reference,
)


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
