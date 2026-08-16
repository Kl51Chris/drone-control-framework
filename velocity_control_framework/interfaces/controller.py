from __future__ import annotations

from typing import Protocol, runtime_checkable

from .attitude_thrust_command import AttitudeThrustCommand
from .drone_state import DroneState
from .reference import Reference


@runtime_checkable
class Controller(Protocol):
    """
    Common interface implemented by PID, LQR, and MPC controllers.

    Controllers are pure control components. They must not communicate
    with Crazyflie hardware directly.
    """

    def reset(self) -> None:
        """Reset all controller internal states."""
        ...

    def update(
        self,
        state: DroneState,
        reference: Reference,
        dt: float,
    ) -> AttitudeThrustCommand:
        """
        Compute one attitude-thrust command.

        Args:
            state:
                Current estimated vehicle state.

            reference:
                Current desired reference.

            dt:
                Elapsed controller-update time in seconds.

        Returns:
            AttitudeThrustCommand:
                Desired roll, pitch, yaw rate, and collective thrust.
        """
        ...