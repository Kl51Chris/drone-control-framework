from __future__ import annotations

import math

from velocity_control_framework.backends import CrazyflieCommandAdapter
from velocity_control_framework.interfaces import BodyRateThrustCommand


class FakeCommander:
    def __init__(self) -> None:
        self.last_setpoint = None
        self.stop_called = False

    def send_setpoint_manual(
        self,
        roll: float,
        pitch: float,
        yaw_rate: float,
        thrust: float,
        rate: bool,
    ) -> None:
        self.last_setpoint = (
            roll,
            pitch,
            yaw_rate,
            thrust,
            rate,
        )

    def send_stop_setpoint(self) -> None:
        self.stop_called = True


class FakeCrazyflie:
    def __init__(self) -> None:
        self.commander = FakeCommander()


def test_crazyflie_command_conversion() -> None:
    cf = FakeCrazyflie()
    adapter = CrazyflieCommandAdapter(cf)

    command = BodyRateThrustCommand(
        roll_rate=math.radians(10.0),
        pitch_rate=math.radians(-5.0),
        yaw_rate=math.radians(30.0),
        thrust=0.5,
    )

    adapter.send(command)

    assert cf.commander.last_setpoint is not None

    roll_rate, pitch_rate, yaw_rate, thrust, rate = (
        cf.commander.last_setpoint
    )

    assert math.isclose(roll_rate, 10.0)
    assert math.isclose(pitch_rate, -5.0)
    assert math.isclose(yaw_rate, 30.0)
    assert math.isclose(thrust, 50.0)
    assert rate is True


def test_crazyflie_stop() -> None:
    cf = FakeCrazyflie()
    adapter = CrazyflieCommandAdapter(cf)

    adapter.stop()

    assert cf.commander.stop_called
