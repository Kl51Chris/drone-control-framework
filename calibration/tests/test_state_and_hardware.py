import math
from types import SimpleNamespace
import pytest

from calibration.config import AttitudeSettings
from calibration.hardware import CrazyflieHardware, attitude_rates
from calibration.state import AltitudeState, validate_fresh_state


def test_thrust_scaling_and_clamping():
    assert CrazyflieHardware.encode_thrust(.42) == 42
    assert CrazyflieHardware.encode_thrust(-1) == 0
    assert CrazyflieHardware.encode_thrust(2) == 100


def test_state_freshness():
    state = AltitudeState(1, 0, 0, 0, 0, 9.9)
    assert validate_fresh_state(state, 10, .2) == pytest.approx(.1)
    with pytest.raises(TimeoutError):
        validate_fresh_state(state, 11, .2)


def test_nonfinite_and_future_state_rejected():
    with pytest.raises(RuntimeError):
        validate_fresh_state(AltitudeState(math.nan, 0, 0, 0, 0, 1), 1, .2)
    with pytest.raises(RuntimeError):
        validate_fresh_state(AltitudeState(0, 0, 0, 0, 0, 2), 1, .2)


def test_onboard_state_is_used_when_external_udp_is_disabled():
    hardware = CrazyflieHardware("test", external_altitude_port=None)
    hardware._provider = SimpleNamespace(get_state=lambda: SimpleNamespace(
        position=[1, 2, 0.42], velocity=[0, 0, -0.1],
        roll=0.1, pitch=-0.2, yaw=0.3, timestamp=12.0,
    ))
    state = hardware.state()
    assert state.altitude_m == 0.42
    assert state.vertical_velocity_m_s == -0.1
    assert state.source == "crazyflie"


def test_default_attitude_signs_drive_angles_toward_level_and_initial_yaw():
    state = AltitudeState(0, 0, roll_rad=0.05, pitch_rad=-0.05, yaw_rad=0.2, timestamp_s=1)
    roll, pitch, yaw = attitude_rates(state, initial_yaw=0.1, settings=AttitudeSettings())
    assert roll < 0
    # cflib negates API-facing pitch before the firmware sees it.
    assert pitch < 0
    assert -pitch > 0
    assert yaw < 0


def test_attitude_rate_outputs_remain_radians_per_second_and_are_limited():
    state = AltitudeState(0, 0, roll_rad=1, pitch_rad=-1, yaw_rad=1, timestamp_s=1)
    settings = AttitudeSettings(max_rate_rad_s=0.2)
    assert attitude_rates(state, initial_yaw=0, settings=settings) == (-0.2, -0.2, -0.2)
