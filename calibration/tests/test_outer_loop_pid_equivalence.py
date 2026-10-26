"""Numerical contract between calibration and production cascaded PID laws."""

from __future__ import annotations

import math

import numpy as np
import pytest

from calibration.cascaded_pid import (
    AltitudeLoopConfig,
    AttitudeLoopConfig,
    CascadedPIDConfig,
    CascadedPIDController,
    CascadedReference,
    HorizontalLoopConfig,
)
from calibration.state import AltitudeState
from velocity_control_framework.controllers import (
    AttitudeControlConfig,
    HorizontalAccelerationMapping,
    HorizontalControlConfig,
    OuterLoopPIDConfig,
    OuterLoopPIDController,
    PIDConfig,
)
from velocity_control_framework.interfaces import DroneState, Reference


COMMAND_FIELDS = ("roll_rate", "pitch_rate", "yaw_rate", "thrust")
# Both implementations perform the same float64-scale arithmetic. This tight
# tolerance permits harmless operation-order rounding but catches control-law
# or saturation-order drift.
EQUIVALENCE_TOLERANCE = 1e-12


def calibration_config(
    *,
    integral_ki: float = 0.0,
    integral_enabled: bool = False,
    velocity_reference_tracking_enabled: bool = False,
) -> CascadedPIDConfig:
    return CascadedPIDConfig(
        stage="5D",
        equilibrium_thrust=0.425,
        min_thrust=0.38,
        max_thrust=0.45,
        altitude=AltitudeLoopConfig(
            position_kp=0.06,
            velocity_kp=0.06,
            integral_ki=integral_ki,
            correction_min=-0.045,
            correction_max=0.025,
            velocity_feedback_enabled=True,
            velocity_reference_tracking_enabled=(
                velocity_reference_tracking_enabled
            ),
            integral_enabled=integral_enabled,
        ),
        attitude=AttitudeLoopConfig(
            roll_kp=4.0,
            pitch_kp=4.0,
            yaw_kp=2.0,
            roll_command_sign=1,
            pitch_command_sign=-1,
            yaw_command_sign=1,
            max_roll_rate_rad_s=0.35,
            max_pitch_rate_rad_s=0.35,
            max_yaw_rate_rad_s=0.35,
        ),
        horizontal=HorizontalLoopConfig(
            enabled=True,
            position_kp=0.4,
            velocity_kp=0.8,
            max_acceleration_m_s2=0.4,
            max_tilt_reference_rad=0.08,
            gravity_m_s2=9.80665,
            body_x_acceleration_to_pitch_sign=-1,
            body_y_acceleration_to_roll_sign=-1,
        ),
    )


def production_config(config: CascadedPIDConfig) -> OuterLoopPIDConfig:
    altitude = config.altitude
    attitude = config.attitude
    horizontal = config.horizontal
    return OuterLoopPIDConfig(
        horizontal_mapping=HorizontalAccelerationMapping(
            horizontal.body_x_acceleration_to_pitch_sign,
            horizontal.body_y_acceleration_to_roll_sign,
        ),
        horizontal_x=PIDConfig(
            horizontal.position_kp,
            0.0,
            horizontal.velocity_kp,
            -horizontal.max_acceleration_m_s2,
            horizontal.max_acceleration_m_s2,
        ),
        horizontal_y=PIDConfig(
            horizontal.position_kp,
            0.0,
            horizontal.velocity_kp,
            -horizontal.max_acceleration_m_s2,
            horizontal.max_acceleration_m_s2,
        ),
        altitude=PIDConfig(
            altitude.position_kp,
            altitude.integral_ki if altitude.integral_enabled else 0.0,
            altitude.velocity_kp if altitude.velocity_feedback_enabled else 0.0,
            altitude.correction_min,
            altitude.correction_max,
        ),
        heading=PIDConfig(
            attitude.yaw_command_sign * attitude.yaw_kp,
            0.0,
            0.0,
            -attitude.max_yaw_rate_rad_s,
            attitude.max_yaw_rate_rad_s,
        ),
        horizontal=HorizontalControlConfig(
            horizontal.max_acceleration_m_s2,
            horizontal.max_tilt_reference_rad,
            horizontal.gravity_m_s2,
        ),
        attitude=AttitudeControlConfig(
            attitude.roll_kp,
            attitude.pitch_kp,
            attitude.roll_command_sign,
            attitude.pitch_command_sign,
            attitude.max_roll_rate_rad_s,
            attitude.max_pitch_rate_rad_s,
        ),
        altitude_velocity_reference_tracking=(
            altitude.velocity_reference_tracking_enabled
        ),
        equilibrium_thrust=config.equilibrium_thrust,
        min_thrust=config.min_thrust,
        max_thrust=config.max_thrust,
    )


def states(values: dict[str, float]) -> tuple[AltitudeState, DroneState]:
    common = {
        "x": values.get("x", 0.0),
        "y": values.get("y", 0.0),
        "z": values.get("z", 0.0),
        "vx": values.get("vx", 0.0),
        "vy": values.get("vy", 0.0),
        "vz": values.get("vz", 0.0),
        "roll": values.get("roll", 0.0),
        "pitch": values.get("pitch", 0.0),
        "yaw": values.get("yaw", 0.0),
    }
    return (
        AltitudeState(
            altitude_m=common["z"],
            vertical_velocity_m_s=common["vz"],
            roll_rad=common["roll"],
            pitch_rad=common["pitch"],
            yaw_rad=common["yaw"],
            timestamp_s=1.0,
            position_x_m=common["x"],
            position_y_m=common["y"],
            velocity_x_m_s=common["vx"],
            velocity_y_m_s=common["vy"],
        ),
        DroneState(
            position=np.array([common["x"], common["y"], common["z"]]),
            velocity=np.array([common["vx"], common["vy"], common["vz"]]),
            roll=common["roll"],
            pitch=common["pitch"],
            yaw=common["yaw"],
            timestamp=1.0,
        ),
    )


def references(values: dict[str, float]) -> tuple[CascadedReference, Reference]:
    common = {
        "x": values.get("x", 0.0),
        "y": values.get("y", 0.0),
        "z": values.get("z", 0.0),
        "vx": values.get("vx", 0.0),
        "vy": values.get("vy", 0.0),
        "vz": values.get("vz", 0.0),
        "yaw": values.get("yaw", 0.0),
    }
    return (
        CascadedReference(
            altitude_m=common["z"],
            vertical_velocity_m_s=common["vz"],
            yaw_rad=common["yaw"],
            position_x_m=common["x"],
            position_y_m=common["y"],
            velocity_x_m_s=common["vx"],
            velocity_y_m_s=common["vy"],
        ),
        Reference(
            position=np.array([common["x"], common["y"], common["z"]]),
            velocity=np.array([common["vx"], common["vy"], common["vz"]]),
            yaw=common["yaw"],
            timestamp=1.0,
        ),
    )


def assert_commands_equal(calibration_command, production_command) -> None:
    for field in COMMAND_FIELDS:
        assert getattr(production_command, field) == pytest.approx(
            getattr(calibration_command, field),
            rel=EQUIVALENCE_TOLERANCE,
            abs=EQUIVALENCE_TOLERANCE,
        ), field


CASES = [
    ({}, {}),
    ({}, {"x": 0.2}),
    ({}, {"x": -0.2}),
    ({}, {"y": 0.2}),
    ({}, {"y": -0.2}),
    ({"vx": 0.15}, {}),
    ({"vx": -0.15}, {}),
    ({"vy": 0.15}, {}),
    ({"vy": -0.15}, {}),
    ({"yaw": math.pi / 2.0}, {"x": 0.2}),
    ({"yaw": -math.pi / 2.0}, {"x": 0.2}),
    ({"yaw": math.pi - 0.05}, {"yaw": -math.pi + 0.05}),
    ({"roll": 0.04, "pitch": -0.06}, {}),
    ({"z": 0.15, "vz": 0.2}, {"z": 0.1}),
    ({"z": 0.05, "vz": -0.2}, {"z": 0.1}),
    (
        {"x": -0.1, "y": 0.12, "z": 0.04, "vx": 0.08,
         "vy": -0.05, "vz": 0.11, "roll": 0.02, "pitch": -0.03,
         "yaw": 0.7},
        {"x": 0.15, "y": -0.08, "z": 0.12, "vx": -0.04,
         "vy": 0.07, "yaw": -0.4},
    ),
    # Strongly asymmetric errors exercise vector saturation order.
    ({}, {"x": 1.0, "y": 100.0, "z": 10.0, "yaw": 2.0}),
]


@pytest.mark.parametrize(("state_values", "reference_values"), CASES)
def test_production_matches_calibration_for_representative_inputs(
    state_values: dict[str, float],
    reference_values: dict[str, float],
) -> None:
    calibration = calibration_config()
    calibration_controller = CascadedPIDController(calibration)
    production_controller = OuterLoopPIDController(production_config(calibration))
    calibration_state, production_state = states(state_values)
    calibration_reference, production_reference = references(reference_values)

    calibration_command = calibration_controller.update(
        calibration_state,
        calibration_reference,
        0.02,
    ).command
    production_command = production_controller.update(
        production_state,
        production_reference,
        0.02,
    )

    assert_commands_equal(calibration_command, production_command)


@pytest.mark.parametrize("track_reference_velocity", [False, True])
def test_altitude_reference_velocity_setting_is_equivalent(
    track_reference_velocity: bool,
) -> None:
    calibration = calibration_config(
        velocity_reference_tracking_enabled=track_reference_velocity,
    )
    calibration_controller = CascadedPIDController(calibration)
    production_controller = OuterLoopPIDController(production_config(calibration))
    calibration_state, production_state = states({"z": 0.08, "vz": 0.13})
    calibration_reference, production_reference = references(
        {"z": 0.1, "vz": -0.09}
    )

    calibration_command = calibration_controller.update(
        calibration_state,
        calibration_reference,
        0.02,
    ).command
    production_command = production_controller.update(
        production_state,
        production_reference,
        0.02,
    )

    assert_commands_equal(calibration_command, production_command)


def test_saturation_anti_windup_and_controller_history_are_equivalent() -> None:
    calibration = calibration_config(integral_ki=0.03, integral_enabled=True)
    calibration_controller = CascadedPIDController(calibration)
    production_controller = OuterLoopPIDController(production_config(calibration))
    sequence = [
        ({"z": -1.0, "vz": -0.4}, {"z": 1.0}),
        ({"z": -0.8, "vz": -0.2}, {"z": 1.0}),
        ({"z": 0.09, "vz": 0.03}, {"z": 0.1}),
        ({"z": 0.12, "vz": 0.05}, {"z": 0.1}),
        ({"z": 0.11, "vz": -0.02}, {"z": 0.1}),
    ]

    for state_values, reference_values in sequence:
        calibration_state, production_state = states(state_values)
        calibration_reference, production_reference = references(reference_values)
        calibration_command = calibration_controller.update(
            calibration_state,
            calibration_reference,
            0.1,
        ).command
        production_command = production_controller.update(
            production_state,
            production_reference,
            0.1,
        )
        assert_commands_equal(calibration_command, production_command)

    assert production_controller._altitude._integral == pytest.approx(
        calibration_controller.altitude_integral_state,
        rel=EQUIVALENCE_TOLERANCE,
        abs=EQUIVALENCE_TOLERANCE,
    )


def test_production_stage5_factory_matches_calibration_configuration() -> None:
    calibration = calibration_config()
    translated = production_config(calibration)
    candidate = OuterLoopPIDConfig.stage5_hardware_candidate()

    assert candidate == translated
