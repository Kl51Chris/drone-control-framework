from __future__ import annotations

from typing import Protocol

from velocity_control_framework.interfaces import BodyRateThrustCommand


class CommandAdapter(Protocol):
    """
    Interface for transmitting controller commands to a backend.

    Implementations may send commands to Crazyflie hardware,
    MuJoCo, or another simulation environment.
    """

    def send(
        self,
        command: BodyRateThrustCommand,
    ) -> None:
        """Send one controller command to the backend."""
        ...

    def stop(self) -> None:
        """Request the backend to stop producing thrust."""
        ...
