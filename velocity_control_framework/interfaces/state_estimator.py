from __future__ import annotations

from typing import Protocol

from .drone_state import DroneState
from .estimated_state import EstimatedState
from .measured_state import MeasuredState


class StateEstimator(Protocol):
    """Boundary that combines/selects state sources for controllers."""

    def estimate(
        self,
        estimated_state: EstimatedState,
        measured_state: MeasuredState,
    ) -> DroneState:
        """Produce the best current unified controller-facing state."""
        ...
