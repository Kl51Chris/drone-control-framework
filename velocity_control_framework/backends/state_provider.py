from __future__ import annotations

from typing import Protocol

from velocity_control_framework.interfaces import DroneState


class StateProvider(Protocol):
    """
    Interface for obtaining estimated vehicle state.

    Implementations may obtain state from Crazyflie firmware,
    MuJoCo, motion capture, or another backend.
    """

    def start(self) -> None:
        """Start state acquisition."""
        ...

    def get_state(self) -> DroneState:
        """Return the latest available vehicle state."""
        ...

    def stop(self) -> None:
        """Stop state acquisition."""
        ...