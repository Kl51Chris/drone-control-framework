from __future__ import annotations

import numpy as np

from velocity_control_framework.backends import CrazyflieStateProvider


class FakeCrazyflie:
    pass


def test_provider_builds_unified_state_with_body_rates() -> None:
    provider = CrazyflieStateProvider(FakeCrazyflie())

    provider._on_translation_log_data(
        0,
        {
            "stateEstimate.x": 1.0,
            "stateEstimate.y": 2.0,
            "stateEstimate.z": 3.0,
            "stateEstimate.vx": 0.1,
            "stateEstimate.vy": 0.2,
            "stateEstimate.vz": 0.3,
        },
        None,
    )
    provider._on_attitude_log_data(
        0,
        {
            "stateEstimate.roll": 10.0,
            "stateEstimate.pitch": 20.0,
            "stateEstimate.yaw": 30.0,
        },
        None,
    )
    provider._on_body_rate_log_data(
        0,
        {
            "stateEstimateZ.rateRoll": 100.0,
            "stateEstimateZ.ratePitch": -200.0,
            "stateEstimateZ.rateYaw": 300.0,
        },
        None,
    )

    state = provider.get_state()

    np.testing.assert_allclose(state.position, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(state.velocity, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(
        [state.roll, state.pitch, state.yaw],
        np.deg2rad([10.0, 20.0, 30.0]),
    )
    np.testing.assert_allclose([state.p, state.q, state.r], [0.1, -0.2, 0.3])
