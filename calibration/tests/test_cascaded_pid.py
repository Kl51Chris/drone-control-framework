from dataclasses import replace
import math

import pytest

from calibration.cascaded_pid import (
    AltitudeLoopConfig, CascadedPIDConfig, CascadedPIDController, CascadedReference,
    HorizontalLoopConfig, load_cascaded_config,
)
from calibration.state import AltitudeState
from velocity_control_framework.interfaces import BodyRateThrustCommand


def state(**changes):
    values = dict(altitude_m=0, vertical_velocity_m_s=0, roll_rad=0, pitch_rad=0,
                  yaw_rad=0, timestamp_s=1, position_x_m=0, position_y_m=0,
                  velocity_x_m_s=0, velocity_y_m_s=0)
    values.update(changes)
    return AltitudeState(**values)


def test_stage5b_produces_body_rate_thrust_and_correct_altitude_sign():
    controller = CascadedPIDController(CascadedPIDConfig())
    result = controller.update(state(altitude_m=.1), CascadedReference(.2), .02)
    assert isinstance(result.command, BodyRateThrustCommand)
    assert result.command.thrust == pytest.approx(.435)
    assert result.contributions["altitude_p"] == pytest.approx(.01)


def test_level_attitude_and_heading_commands_follow_verified_adapter_signs():
    result = CascadedPIDController(CascadedPIDConfig()).update(
        state(roll_rad=.02, pitch_rad=.03, yaw_rad=.04), CascadedReference(0), .02)
    assert result.command.roll_rate < 0
    # Positive measured legacy pitch requires positive API command because cflib negates it.
    assert result.command.pitch_rate > 0
    assert result.command.yaw_rate < 0


def test_attitude_and_altitude_limits_are_independent_and_reported():
    result = CascadedPIDController(CascadedPIDConfig()).update(
        state(altitude_m=-1, roll_rad=1, pitch_rad=-1, yaw_rad=1),
        CascadedReference(1), .02)
    assert result.command.thrust == .45
    assert result.command.roll_rate == -.35
    assert result.command.pitch_rate == -.35
    assert result.command.yaw_rate == -.35
    assert result.contributions["altitude_saturated"] == 1
    assert result.contributions["roll_rate_saturated"] == 1


def test_stage5b_rejects_later_stage_feedback_enablement():
    config = CascadedPIDConfig()
    config.altitude.velocity_feedback_enabled = True
    with pytest.raises(ValueError, match="Stage 5B"):
        config.validate()


def test_stage5c_velocity_feedback_and_integral_anti_windup():
    config = CascadedPIDConfig(stage="5C", altitude=AltitudeLoopConfig(
        position_kp=0, velocity_kp=.1, integral_ki=1,
        correction_min=-.02, correction_max=.02,
        velocity_feedback_enabled=True, integral_enabled=True))
    controller = CascadedPIDController(config)
    saturated = controller.update(state(vertical_velocity_m_s=-1), CascadedReference(1), .1)
    assert saturated.contributions["altitude_velocity"] == pytest.approx(.1)
    assert saturated.command.thrust == pytest.approx(.445)
    assert controller.altitude_integral_state == 0
    controller.update(state(altitude_m=1.01), CascadedReference(1), .1)
    assert controller.altitude_integral_state < 0


def test_stage5d_world_xy_rotates_to_body_and_maps_to_bounded_attitude():
    config = CascadedPIDConfig(stage="5D", horizontal=HorizontalLoopConfig(
        enabled=True, position_kp=2, velocity_kp=0,
        max_acceleration_m_s2=1, max_tilt_reference_rad=.08))
    result = CascadedPIDController(config).update(
        state(yaw_rad=math.pi/2), CascadedReference(0, position_x_m=1), .02)
    assert result.contributions["horizontal_acceleration_x_world_m_s2"] == pytest.approx(1)
    assert result.contributions["horizontal_acceleration_y_body_m_s2"] == pytest.approx(-1)
    assert result.contributions["desired_roll_rad"] == pytest.approx(.08)
    assert abs(result.contributions["desired_pitch_rad"]) < 1e-12


def test_offline_config_is_interlocked_and_loadable():
    config = load_cascaded_config("calibration/stage5b_offline_config.json")
    assert config.stage == "5B"
    assert config.hardware_flight_authorized is False
    assert config.powered_attitude_validated is False


@pytest.mark.parametrize("dt", [0, float("nan")])
def test_invalid_controller_input_rejected(dt):
    with pytest.raises(ValueError):
        CascadedPIDController(CascadedPIDConfig()).update(state(), CascadedReference(0), dt)
