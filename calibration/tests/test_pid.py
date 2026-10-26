import pytest

from calibration.config import PIDSettings
from calibration.pid import AltitudePID


def test_positive_up_error_increases_output():
    result = AltitudePID(PIDSettings(kp=2, kd=0, correction_min=-10, correction_max=10)).update(1, .5, .1)
    assert result.error == .5
    assert result.output == 1


def test_discrete_pid_terms():
    pid = AltitudePID(PIDSettings(kp=2, ki=3, kd=4, correction_min=-100, correction_max=100))
    first = pid.update(1, 0, .5)
    second = pid.update(1, .25, .25)
    assert (first.proportional, first.integral, first.derivative) == (2, 1.5, 0)
    assert second.proportional == 1.5
    assert second.integral == pytest.approx(2.0625)
    assert second.derivative == pytest.approx(-4)


def test_conditional_anti_windup_and_recovery():
    pid = AltitudePID(PIDSettings(kp=1, ki=1, kd=0, correction_min=-1, correction_max=1))
    assert pid.update(10, 0, 1).saturated
    assert pid.integral_state == 0
    pid.update(-.5, 0, 1)
    assert pid.integral_state == -.5


def test_invalid_dt_rejected():
    with pytest.raises(ValueError):
        AltitudePID(PIDSettings()).update(1, 0, 0)
