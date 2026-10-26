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


def test_stage5c_positive_velocity_gain_opposes_vertical_velocity_error():
    config=CascadedPIDConfig(stage="5C",altitude=AltitudeLoopConfig(
        position_kp=.1,velocity_kp=.04,velocity_feedback_enabled=True))
    controller=CascadedPIDController(config)
    rising=controller.update(state(vertical_velocity_m_s=.2),CascadedReference(.1),.02)
    falling=controller.update(state(vertical_velocity_m_s=-.2),CascadedReference(.1),.02)
    tracking=controller.update(state(vertical_velocity_m_s=.05),
        CascadedReference(.1,vertical_velocity_m_s=.1),.02)
    assert rising.contributions["altitude_velocity"]==pytest.approx(-.008)
    assert falling.contributions["altitude_velocity"]==pytest.approx(.008)
    assert tracking.contributions["altitude_velocity"]==pytest.approx(.002)


def test_stage5c_can_apply_zero_reference_velocity_damping():
    config=CascadedPIDConfig(stage="5C",altitude=AltitudeLoopConfig(
        position_kp=.1,velocity_kp=.04,velocity_feedback_enabled=True,
        velocity_reference_tracking_enabled=False))
    result=CascadedPIDController(config).update(
        state(vertical_velocity_m_s=.05),CascadedReference(.1,vertical_velocity_m_s=.1),.02)
    assert result.contributions["altitude_velocity_reference_m_s"]==0
    assert result.contributions["vertical_velocity_error_m_s"]==pytest.approx(-.05)
    assert result.contributions["altitude_velocity"]==pytest.approx(-.002)


def test_zero_reference_damping_differs_from_ramp_tracking_only_by_kv_times_reference():
    tracking=CascadedPIDConfig(stage="5C",altitude=AltitudeLoopConfig(
        position_kp=.1,velocity_kp=.04,velocity_feedback_enabled=True))
    zero=CascadedPIDConfig(stage="5C",altitude=AltitudeLoopConfig(
        position_kp=.1,velocity_kp=.04,velocity_feedback_enabled=True,
        velocity_reference_tracking_enabled=False))
    measured=state(altitude_m=.12,vertical_velocity_m_s=.2)
    for reference_velocity,expected_delta in ((.1,-.004),(0,0),(-.08,.0032)):
        reference=CascadedReference(.1,vertical_velocity_m_s=reference_velocity)
        tracking_thrust=CascadedPIDController(tracking).update(measured,reference,.02).command.thrust
        zero_thrust=CascadedPIDController(zero).update(measured,reference,.02).command.thrust
        assert zero_thrust-tracking_thrust==pytest.approx(expected_delta)


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
    assert result.contributions["horizontal_acceleration_saturated"] == 1
    assert result.contributions["tilt_reference_saturated"] == 1


def test_stage5d_position_and_velocity_terms_are_restoring_before_rotation():
    config = CascadedPIDConfig(stage="5D", horizontal=HorizontalLoopConfig(
        enabled=True, position_kp=.4, velocity_kp=.8,
        max_acceleration_m_s2=.4, max_tilt_reference_rad=.08))
    result = CascadedPIDController(config).update(
        state(position_x_m=-.2, position_y_m=.1, velocity_x_m_s=-.1,
              velocity_y_m_s=.05), CascadedReference(0), .02)
    c = result.contributions
    assert c["horizontal_position_x_contribution_m_s2"] > 0
    assert c["horizontal_velocity_x_contribution_m_s2"] > 0
    assert c["horizontal_position_y_contribution_m_s2"] < 0
    assert c["horizontal_velocity_y_contribution_m_s2"] < 0


@pytest.mark.parametrize(
    "position_x_m,velocity_x_m_s,expected_acceleration_sign",
    [(-.2, 0, 1), (.2, 0, -1), (0, .1, -1), (0, -.1, 1)],
)
def test_identified_pitch_mapping_produces_restoring_body_x_acceleration(
        position_x_m, velocity_x_m_s, expected_acceleration_sign):
    """The measured plant mapping is a_body_x/pitch < 0."""
    config = CascadedPIDConfig(stage="5D", horizontal=HorizontalLoopConfig(
        enabled=True, position_kp=.4, velocity_kp=.8,
        max_acceleration_m_s2=.4, max_tilt_reference_rad=.08,
        body_x_acceleration_to_pitch_sign=-1))
    result = CascadedPIDController(config).update(
        state(position_x_m=position_x_m, velocity_x_m_s=velocity_x_m_s),
        CascadedReference(0), .02)
    desired_acceleration = result.contributions["horizontal_acceleration_x_body_m_s2"]
    desired_pitch = result.contributions["desired_pitch_rad"]
    inferred_physical_acceleration = -desired_pitch
    assert math.copysign(1, desired_acceleration) == expected_acceleration_sign
    assert math.copysign(1, inferred_physical_acceleration) == expected_acceleration_sign


@pytest.mark.parametrize(
    "position_y_m,velocity_y_m_s,expected_acceleration_sign",
    [(-.2, 0, 1), (.2, 0, -1), (0, .1, -1), (0, -.1, 1)],
)
def test_identified_roll_mapping_produces_restoring_body_y_acceleration(
        position_y_m, velocity_y_m_s, expected_acceleration_sign):
    """The measured plant mapping is a_body_y/roll < 0."""
    config = CascadedPIDConfig(stage="5D", horizontal=HorizontalLoopConfig(
        enabled=True, position_kp=.4, velocity_kp=.8,
        max_acceleration_m_s2=.4, max_tilt_reference_rad=.08,
        body_y_acceleration_to_roll_sign=-1))
    result = CascadedPIDController(config).update(
        state(position_y_m=position_y_m, velocity_y_m_s=velocity_y_m_s),
        CascadedReference(0), .02)
    desired_acceleration = result.contributions["horizontal_acceleration_y_body_m_s2"]
    desired_roll = result.contributions["desired_roll_rad"]
    inferred_physical_acceleration = -desired_roll
    assert math.copysign(1, desired_acceleration) == expected_acceleration_sign
    assert math.copysign(1, inferred_physical_acceleration) == expected_acceleration_sign


def test_xy_candidate_records_identified_pitch_sign_but_retains_interlock():
    config = load_cascaded_config("calibration/stage5d_xy_first_flight_config.json")
    assert config.horizontal.body_x_acceleration_to_pitch_sign == -1
    assert config.horizontal_mapping_hardware_validated is False
    assert config.hardware_flight_authorized is False


def test_xy_pd_hover_candidate_preserves_altitude_pd_and_validated_mapping():
    config = load_cascaded_config("calibration/stage5d_xy_pd_hover_candidate_config.json")
    assert config.altitude.position_kp == pytest.approx(.06)
    assert config.altitude.velocity_kp == pytest.approx(.06)
    assert config.altitude.velocity_reference_tracking_enabled is False
    assert config.horizontal.body_x_acceleration_to_pitch_sign == -1
    assert config.horizontal.body_y_acceleration_to_roll_sign == -1
    assert config.horizontal_mapping_hardware_validated is True
    assert config.hardware_flight_authorized is False


def test_horizontal_mapping_signs_are_explicit_and_validated():
    config = CascadedPIDConfig(stage="5D", horizontal=HorizontalLoopConfig(
        enabled=True, position_kp=.4, velocity_kp=.8,
        max_acceleration_m_s2=.4, max_tilt_reference_rad=.08,
        body_x_acceleration_to_pitch_sign=0))
    with pytest.raises(ValueError, match="signs must be -1 or 1"):
        config.validate()


def test_offline_config_is_interlocked_and_loadable():
    config = load_cascaded_config("calibration/stage5b_offline_config.json")
    assert config.stage == "5B"
    assert config.hardware_flight_authorized is False
    assert config.powered_attitude_validated is False
    assert config.horizontal_mapping_hardware_validated is False


@pytest.mark.parametrize("dt", [0, float("nan")])
def test_invalid_controller_input_rejected(dt):
    with pytest.raises(ValueError):
        CascadedPIDController(CascadedPIDConfig()).update(state(), CascadedReference(0), dt)
