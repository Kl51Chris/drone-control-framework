from pathlib import Path

import numpy as np
import pytest

from velocity_control_framework.controllers.outer_loop_deepc import (
    OuterLoopDeePCConfig, OuterLoopDeePCController,
)
from velocity_control_framework.interfaces import BodyRateThrustCommand, DroneState, Reference


@pytest.mark.parametrize("weight", [0., -1., float("nan"), float("inf")])
def test_initialization_weight_must_be_positive_and_finite(weight):
    with pytest.raises(ValueError, match="initialization weight"):
        OuterLoopDeePCConfig(initialization_weight=weight)


def test_initialization_weight_scales_past_output_hessian_and_logged_penalty():
    if not Path("log/phase3-v1/manifest.json").exists():
        pytest.skip("local phase3-v1 training artifact is not mounted")
    one = OuterLoopDeePCController(OuterLoopDeePCConfig())
    quarter = OuterLoopDeePCController(OuterLoopDeePCConfig(initialization_weight=.25))
    np.testing.assert_allclose(
        one._hessian - quarter._hessian,
        .75 * (one._past_map.T @ one._past_map),
        atol=1e-10, rtol=1e-10,
    )
    for k in range(7):
        state = DroneState.zero(timestamp=1.+.02*k)
        state.position[0] = .0002*k
        quarter.update(state, Reference.hover(0.,0.,0.), .02)
        quarter.record_issued_command(BodyRateThrustCommand(0.,0.,0.,quarter.config.model.equilibrium_thrust))
    slack = quarter.diagnostics["init_slack_rms"]
    assert slack > 0
    assert quarter.diagnostics["initialization_cost"] == pytest.approx(
        .5 * .25 * (9*quarter.config.past-3) * slack**2,
    )
