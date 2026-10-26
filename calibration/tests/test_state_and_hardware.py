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
        p=0.01, q=-0.02, r=0.03,
    ))
    state = hardware.state()
    assert state.altitude_m == 0.42
    assert state.vertical_velocity_m_s == -0.1
    assert state.source == "crazyflie"
    assert (state.body_rate_p_rad_s, state.body_rate_q_rad_s, state.body_rate_r_rad_s) == (0.01, -0.02, 0.03)


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


def test_official_hover_wrapper_uses_zero_horizontal_velocity_and_yaw_rate():
    calls = []
    hardware = CrazyflieHardware("test")
    hardware._cf = SimpleNamespace(commander=SimpleNamespace(
        send_hover_setpoint=lambda *args: calls.append(args)
    ))
    hardware.send_hover_setpoint(0.3)
    assert calls == [(0.0, 0.0, 0.0, 0.3)]


@pytest.mark.parametrize(
    ("value", "expected"),
    [("0", False), ("1", True), (0, False), (1, True)],
)
def test_flow_deck_parameter_numeric_values(value, expected):
    hardware = CrazyflieHardware("test")
    hardware._cf = SimpleNamespace(param=SimpleNamespace(
        values={"deck": {"bcFlow": "0", "bcFlow2": value}}
    ))
    assert hardware.flow_deck_available() is expected


@pytest.mark.parametrize("values", [{}, {"deck": {}}, {"deck": {"unrelated": "1"}}])
def test_flow_deck_missing_parameter_is_not_available(values):
    hardware = CrazyflieHardware("test")
    hardware._cf = SimpleNamespace(param=SimpleNamespace(values=values))
    assert hardware.flow_deck_available() is False


def test_flow_deck_parameter_retrieval_failure_is_fail_closed():
    class BrokenParam:
        @property
        def values(self):
            raise OSError("parameter transport failed")

    hardware = CrazyflieHardware("test")
    hardware._cf = SimpleNamespace(param=BrokenParam())
    with pytest.raises(RuntimeError, match="failed to retrieve Flow Deck parameters"):
        hardware.flow_deck_available()


def test_flow_deck_malformed_parameter_is_rejected():
    hardware = CrazyflieHardware("test")
    hardware._cf = SimpleNamespace(param=SimpleNamespace(
        values={"deck": {"bcFlow2": "enabled"}}
    ))
    with pytest.raises(RuntimeError, match="invalid numeric value"):
        hardware.flow_deck_available()


def test_parameter_readiness_succeeds_after_asynchronous_update():
    responses = iter((False, False, True))
    hardware = CrazyflieHardware("test")
    hardware._link = SimpleNamespace(is_params_updated=lambda: next(responses))
    hardware._wait_for_parameters(timeout_s=.1, poll_interval_s=0)


def test_parameter_readiness_failure_is_bounded():
    hardware = CrazyflieHardware("test")
    hardware._link = SimpleNamespace(is_params_updated=lambda: False)
    with pytest.raises(RuntimeError, match="timed out waiting"):
        hardware._wait_for_parameters(timeout_s=0, poll_interval_s=0)
