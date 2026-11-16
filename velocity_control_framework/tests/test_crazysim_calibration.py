from __future__ import annotations

import pytest

from simulation.crazysim_calibration import estimate_cf2x_t350_hover_thrust


def test_cf2x_t350_hover_estimate_matches_model_equations() -> None:
    estimate = estimate_cf2x_t350_hover_thrust()
    assert estimate.weight_n == pytest.approx(0.0379 * 9.81)
    assert estimate.thrust_per_motor_n == pytest.approx(estimate.weight_n / 4.0)
    assert estimate.raw_pwm == pytest.approx(33841.4548125)
    assert estimate.normalized_command == pytest.approx(0.4768186326226526)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"mass_kg": 0.0},
        {"gravity_m_s2": 0.0},
        {"maximum_thrust_per_motor_n": 0.0},
    ],
)
def test_hover_estimate_rejects_invalid_physics(kwargs) -> None:
    with pytest.raises(ValueError):
        estimate_cf2x_t350_hover_thrust(**kwargs)
