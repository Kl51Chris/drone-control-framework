from __future__ import annotations

from velocity_control_framework.interfaces import (
    DroneState,
    EstimatedState,
    MeasuredState,
)


class PassthroughStateEstimator:
    """Construct ``DroneState`` solely from ``EstimatedState``.

    ``MeasuredState`` is accepted to establish the source-combination boundary
    but is intentionally ignored. This is temporary plumbing, not sensor fusion;
    a future estimator can select measured fields without changing controllers.
    """

    def estimate(
        self,
        estimated_state: EstimatedState,
        measured_state: MeasuredState,
    ) -> DroneState:
        del measured_state

        return DroneState(
            position=estimated_state.position.copy(),
            velocity=estimated_state.velocity.copy(),
            roll=estimated_state.roll,
            pitch=estimated_state.pitch,
            yaw=estimated_state.yaw,
            timestamp=estimated_state.timestamp,
            p=estimated_state.p,
            q=estimated_state.q,
            r=estimated_state.r,
        )
