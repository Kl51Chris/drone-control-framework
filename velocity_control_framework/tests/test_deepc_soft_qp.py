from time import perf_counter

import numpy as np

from velocity_control_framework.controllers.deepc_soft_qp import solve


def test_full_soft_qp_preserves_hard_edges_and_both_output_sides():
    # First state optimum at the soft kink; the next two both reach the
    # hard input box and pay for unavoidable positive/negative slack.
    result, _, residuals = solve(
        np.eye(3), np.eye(3), -np.ones(3), np.ones(3),
        np.full(3, .5), np.array([0., 1.3, -1.3]), np.array([-20., -200., 200.]),
        np.zeros(3), perf_counter() + .2,
    )
    np.testing.assert_allclose(result, [.5, 1., -1., 0., 1.8, 1.8], atol=2e-6)
    assert residuals[0] < 2e-7
    assert residuals[1] < 2e-5


def test_full_soft_qp_rejects_expired_deadline():
    result, _, residuals = solve(
        np.eye(1), np.eye(1), -np.ones(1), np.ones(1),
        np.ones(1), np.zeros(1), np.zeros(1), np.zeros(1), perf_counter(),
    )
    assert result is None and residuals is None
