import numpy as np
import pytest

from velocity_control_framework.controllers.near_hover_model import (
    NearHoverModelConfig, discrete_model,
)
from velocity_control_framework.controllers.outer_loop_mpc import (
    OuterLoopMPCConfig, OuterLoopMPCController,
)
from velocity_control_framework.interfaces import DroneState, Reference


def test_model_exact_thrust_zoh_and_observed_rate_signs():
    config = NearHoverModelConfig()
    ad, bd = discrete_model(config)
    dt = config.sample_period_s
    assert ad[0, 3] == pytest.approx(dt)
    assert bd[2, 3] == pytest.approx(config.thrust_gain_m_s2 * dt**2 / 2)
    np.testing.assert_allclose(np.diag(bd[6:9, :3]), [dt, -dt, -dt])


def test_qp_holds_hover_and_respects_bounds_at_unreachable_reference():
    pytest.importorskip("osqp")
    config = OuterLoopMPCConfig(horizon=15)
    controller = OuterLoopMPCController(config)
    state = DroneState.zero()
    equilibrium = controller.update(state, Reference.hover(0, 0, 0), 0.02)
    assert equilibrium.thrust == pytest.approx(config.model.equilibrium_thrust)
    command = controller.update(state, Reference.hover(0, 0, 8), 0.02)
    assert config.min_thrust <= command.thrust <= config.max_thrust
    assert controller.prediction["state"].shape == (16, 9)
    assert controller.diagnostics["status"] == "solved"
    controller.reset()
    assert not controller.prediction


def test_preview_uses_future_targets_and_constant_preview_matches_baseline():
    pytest.importorskip("osqp")
    config = OuterLoopMPCConfig(horizon=5)
    controller = OuterLoopMPCController(config)
    state = DroneState.zero()
    ref = Reference.hover(0, 0, 0)
    baseline = controller.update(state, ref, 0.02)
    controller.reset()
    same = controller.update_preview(state, (ref,) * 5)
    for field in ("roll_rate", "pitch_rate", "yaw_rate", "thrust"):
        assert getattr(same, field) == pytest.approx(getattr(baseline, field), abs=1e-10)
    references = tuple(Reference.hover(0, 0, 0.01 * j) for j in range(1, 6))
    command = controller.update_preview(state, references)
    assert command.thrust > baseline.thrust
    np.testing.assert_allclose(controller.prediction["reference_trajectory"][:, 2],
                               [0.01, 0.02, 0.03, 0.04, 0.05])
