from __future__ import annotations

from typing import Protocol

from velocity_control_framework.interfaces import DroneState


class StateProvider(Protocol):
    """
    Interface for obtaining unified controller-facing vehicle state.

    Implementations may acquire source-specific estimated and measured state
    from Crazyflie firmware, MuJoCo, motion capture, or another backend, then
    route those sources through a StateEstimator before returning DroneState.
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
