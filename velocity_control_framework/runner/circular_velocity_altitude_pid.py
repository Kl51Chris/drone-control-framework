from __future__ import annotations

from collections.abc import Callable

from velocity_control_framework.controllers import (
    VelocityAltitudePIDController,
)
from velocity_control_framework.interfaces import (
    AttitudeThrustCommand,
    DroneState,
    Reference,
)
from velocity_control_framework.references import (
    CircularFlightReference,
    CircularFlightState,
)


CircularReferenceFactory = Callable[
    [
        DroneState,
        float,
    ],
    CircularFlightReference,
]


class CircularVelocityAltitudePIDProgram:
    """
    Adapt CircularFlightReference and
    VelocityAltitudePIDController to UniversalRunner.

    This class contains no controller tuning and no trajectory tuning.
    """

    def __init__(
        self,
        *,
        controller: VelocityAltitudePIDController,
        reference_factory: CircularReferenceFactory,
    ) -> None:
        self._controller = controller
        self._reference_factory = (
            reference_factory
        )

        self._reference: (
            CircularFlightReference | None
        ) = None

    def initialize(
        self,
        initial_state: DroneState,
        start_time: float,
    ) -> None:
        self._controller.reset()

        self._reference = (
            self._reference_factory(
                initial_state,
                start_time,
            )
        )

    def is_complete(
        self,
        current_time: float,
    ) -> bool:
        reference = self._require_reference()

        return (
            reference.get_state(current_time)
            is CircularFlightState.COMPLETE
        )

    def get_phase_name(
        self,
        current_time: float,
    ) -> str:
        reference = self._require_reference()

        return (
            reference
            .get_state(current_time)
            .name
        )

    def update(
        self,
        state: DroneState,
        current_time: float,
        dt: float,
    ) -> tuple[
        Reference,
        AttitudeThrustCommand,
    ]:
        reference_provider = (
            self._require_reference()
        )

        flight_state = (
            reference_provider.get_state(
                current_time
            )
        )

        reference = (
            reference_provider.get_reference(
                current_time
            )
        )

        if (
            flight_state
            is CircularFlightState.ARMED_IDLE
        ):
            # Do not run the altitude controller during armed
            # idle. At zero altitude error the PID would still
            # send the configured hover thrust.
            command = (
                AttitudeThrustCommand.zero()
            )

        else:
            command = self._controller.update(
                state=state,
                reference=reference,
                dt=dt,
            )

        return reference, command

    def _require_reference(
        self,
    ) -> CircularFlightReference:
        if self._reference is None:
            raise RuntimeError(
                "Flight program has not been initialized"
            )

        return self._reference