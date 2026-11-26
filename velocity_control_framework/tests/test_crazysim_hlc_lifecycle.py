from types import SimpleNamespace
from unittest.mock import Mock, call

import pytest

from velocity_control_framework.backends.crazysim_hlc_lifecycle import (
    CommandOwner,
    CrazySimHLCLifecycle,
)
from velocity_control_framework.interfaces import BodyRateThrustCommand


def test_ground_unlock_and_bidirectional_handover_order():
    transport = Mock()
    cf = SimpleNamespace(
        commander=transport.commander, high_level_commander=transport.hlc
    )
    lifecycle = CrazySimHLCLifecycle(cf, transport.body_rate)
    command = BodyRateThrustCommand(0.08, 0.0, 0.0, 0.48)

    lifecycle.takeoff(0.35, 3.0)
    lifecycle.send_tracking(command)
    lifecycle.land(0.03, 3.0)

    assert lifecycle.owner is CommandOwner.LANDING
    assert transport.mock_calls == [
        call.body_rate.unlock(),
        call.commander.send_notify_setpoint_stop(),
        call.hlc.takeoff(0.35, 3.0, yaw=None),
        call.body_rate.send(command),
        call.commander.send_notify_setpoint_stop(),
        call.hlc.land(0.03, 3.0, yaw=None),
    ]
    with pytest.raises(RuntimeError):
        lifecycle.send_tracking(command)
    lifecycle.stop()
    assert transport.mock_calls[-2:] == [call.hlc.stop(), call.body_rate.stop()]


def test_official_landing_without_body_rate_does_not_release_priority():
    transport = Mock()
    cf = SimpleNamespace(
        commander=transport.commander, high_level_commander=transport.hlc
    )
    lifecycle = CrazySimHLCLifecycle(cf, transport.body_rate)

    lifecycle.takeoff(0.35, 3.0)
    lifecycle.land(0.03, 3.0)

    transport.commander.send_notify_setpoint_stop.assert_called_once_with()
    transport.body_rate.send.assert_not_called()
