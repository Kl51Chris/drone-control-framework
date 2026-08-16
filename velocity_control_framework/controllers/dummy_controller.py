from __future__ import annotations

from velocity_control_framework.interfaces import (
    AttitudeThrustCommand,
    DroneState,
    Reference,
)


class DummyController:
    """
    Minimal controller used to test the control pipeline.

    It ignores the current state and reference and always returns a
    fixed zero-thrust command.
    """

    def reset(self) -> None:
        """Dummy controller has no internal state."""
        pass

    def update(
        self,
        state: DroneState,
        reference: Reference,
        dt: float,
    ) -> AttitudeThrustCommand:
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        return AttitudeThrustCommand(
            roll=0.0,
            pitch=0.0,
            yaw_rate=0.0,
            thrust=0.0,
        )