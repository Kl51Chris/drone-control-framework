from __future__ import annotations

import numpy as np

from velocity_control_framework.controllers import DummyController
from velocity_control_framework.interfaces import (
    AttitudeThrustCommand,
    DroneState,
    Reference,
)
from velocity_control_framework.runner.control_loop import run_control_loop

class FakeStateProvider:
    """State provider used only for software integration testing."""

    def __init__(self) -> None:
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True

    def get_state(self) -> DroneState:
        if not self.started:
            raise RuntimeError("State provider has not been started")

        return DroneState(
            position=np.array(
                [0.0, 0.0, 0.5],
                dtype=np.float64,
            ),
            velocity=np.zeros(3, dtype=np.float64),
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            timestamp=1.0,
        )

    def stop(self) -> None:
        self.stopped = True


class RecordingCommandAdapter:
    """Command adapter that records commands instead of sending them."""

    def __init__(self) -> None:
        self.commands: list[AttitudeThrustCommand] = []
        self.stopped = False

    def send(
        self,
        command: AttitudeThrustCommand,
    ) -> None:
        self.commands.append(command)

    def stop(self) -> None:
        self.stopped = True


def test_complete_control_pipeline() -> None:
    state_provider = FakeStateProvider()
    controller = DummyController()
    command_adapter = RecordingCommandAdapter()

    reference = Reference.hover(
        x=0.0,
        y=0.0,
        z=0.5,
        yaw=0.0,
        timestamp=1.0,
    )

    state_provider.start()
    controller.reset()

    state = state_provider.get_state()

    command = controller.update(
        state=state,
        reference=reference,
        dt=0.01,
    )

    command_adapter.send(command)

    command_adapter.stop()
    state_provider.stop()

    assert isinstance(state, DroneState)
    assert isinstance(reference, Reference)
    assert isinstance(command, AttitudeThrustCommand)

    assert len(command_adapter.commands) == 1
    assert command_adapter.commands[0] == command

    assert command.roll == 0.0
    assert command.pitch == 0.0
    assert command.yaw_rate == 0.0
    assert command.thrust == 0.0

    assert command_adapter.stopped
    assert state_provider.stopped
def test_control_loop_with_fake_backend() -> None:
    state_provider = FakeStateProvider()
    command_adapter = RecordingCommandAdapter()
    controller = DummyController()

    def get_reference() -> Reference:
        return Reference.hover(
            x=0.0,
            y=0.0,
            z=0.5,
            yaw=0.0,
            timestamp=1.0,
        )

    run_control_loop(
        state_provider=state_provider,
        command_adapter=command_adapter,
        controller=controller,
        reference_provider=get_reference,
        frequency_hz=20.0,
        duration_s=0.2,
    )

    assert len(command_adapter.commands) >= 2

    for command in command_adapter.commands:
        assert isinstance(
            command,
            AttitudeThrustCommand,
        )
        assert command.thrust == 0.0

    assert command_adapter.stopped
    assert state_provider.stopped