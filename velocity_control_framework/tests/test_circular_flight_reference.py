from __future__ import annotations

import math

import numpy as np
import pytest

from experiments.run_flight import (
    CIRCULAR_MAXIMUM_FLIGHT_DURATION_S,
    CIRCULAR_REFERENCE,
    CONFIGURATION_NAME,
    CONTROLLER_NAME,
    assemble_program,
)
from velocity_control_framework.interfaces import DroneState
from velocity_control_framework.references.circular_flight_reference import (
    CircularFlightConfig,
    CircularFlightReference,
    CircularFlightState,
)


def reference(start_time: float = 0.0) -> CircularFlightReference:
    return CircularFlightReference(
        initial_position=np.array([1.0, 2.0, 0.1]),
        initial_yaw=0.3,
        start_time=start_time,
    )


def test_reviewed_circular_profile_values_remain_unchanged() -> None:
    config = CircularFlightConfig()

    assert config.circle_radius == 0.15
    assert config.circle_revolutions == 1.0
    assert config.armed_idle_duration == 0.5
    assert config.takeoff_duration == 4.0
    assert config.circle_duration == 15.0
    assert config.landing_duration == 4.0


@pytest.mark.parametrize(
    ("elapsed_s", "expected_state"),
    [
        (0.0, CircularFlightState.ARMED_IDLE),
        (0.5 - 1.0e-9, CircularFlightState.ARMED_IDLE),
        (0.5, CircularFlightState.TAKEOFF),
        (4.5 - 1.0e-9, CircularFlightState.TAKEOFF),
        (4.5, CircularFlightState.CIRCLE),
        (19.5 - 1.0e-9, CircularFlightState.CIRCLE),
        (19.5, CircularFlightState.LANDING),
        (23.5 - 1.0e-9, CircularFlightState.LANDING),
        (23.5, CircularFlightState.COMPLETE),
    ],
)
def test_phase_boundaries(elapsed_s, expected_state) -> None:
    assert reference().get_state(elapsed_s) is expected_state


@pytest.mark.parametrize("boundary_s", [0.5, 4.5, 19.5, 23.5])
def test_reference_position_and_velocity_are_continuous(boundary_s) -> None:
    provider = reference()
    epsilon = 1.0e-7
    before = provider.get_reference(boundary_s - epsilon)
    at_boundary = provider.get_reference(boundary_s)

    np.testing.assert_allclose(before.position, at_boundary.position, atol=1.0e-7)
    np.testing.assert_allclose(before.velocity, at_boundary.velocity, atol=1.0e-7)


def test_circle_kinematics_match_smoothstep_profile() -> None:
    provider = reference()
    config = provider.config
    circle_start = config.armed_idle_duration + config.takeoff_duration
    midpoint = circle_start + config.circle_duration / 2.0

    midpoint_reference = provider.get_reference(midpoint)
    expected_speed = (
        3.0
        * math.pi
        * config.circle_radius
        * config.circle_revolutions
        / config.circle_duration
    )
    horizontal_speed = float(np.linalg.norm(midpoint_reference.velocity[:2]))
    assert horizontal_speed == pytest.approx(expected_speed)
    assert horizontal_speed == pytest.approx(0.09424777960769379)

    delta = 1.0e-4
    velocity_before = provider.get_reference(midpoint - delta).velocity[:2]
    velocity_after = provider.get_reference(midpoint + delta).velocity[:2]
    acceleration = (velocity_after - velocity_before) / (2.0 * delta)
    expected_acceleration = config.circle_radius * (
        3.0 * math.pi * config.circle_revolutions / config.circle_duration
    ) ** 2
    assert float(np.linalg.norm(acceleration)) == pytest.approx(
        expected_acceleration, rel=1.0e-7
    )
    assert expected_acceleration == pytest.approx(0.05921762640653615)

    samples = np.linspace(circle_start, circle_start + config.circle_duration, 1001)
    displacements = [
        np.linalg.norm(provider.get_reference(sample).position[:2] - [1.0, 2.0])
        for sample in samples
    ]
    assert max(displacements) == pytest.approx(2.0 * config.circle_radius)


def test_circular_profile_leaves_six_and_a_half_seconds_for_landing_check() -> None:
    _, _, program = assemble_program(
        CONTROLLER_NAME,
        CONFIGURATION_NAME,
        CIRCULAR_REFERENCE,
    )
    program.initialize(
        DroneState(
            position=np.array([1.0, 2.0, 0.1]),
            velocity=np.zeros(3),
            roll=0.0,
            pitch=0.0,
            yaw=0.3,
            timestamp=0.0,
        ),
        start_time=0.0,
    )
    assert isinstance(program.reference_provider, CircularFlightReference)

    nominal_duration = program.reference_provider.total_duration
    allowance = program.safety.maximum_flight_duration_s - nominal_duration

    assert nominal_duration == pytest.approx(23.5)
    assert program.safety.maximum_flight_duration_s == (
        CIRCULAR_MAXIMUM_FLIGHT_DURATION_S
    )
    assert allowance == pytest.approx(6.5)
    assert program.get_phase_name(nominal_duration) == "LANDING_CHECK"
