import numpy as np
import pytest

from calibration.cascaded_pid import load_cascaded_config
from calibration.vertical_system_id import _fit, identify_vertical_dynamics


def test_integral_model_fit_recovers_known_coefficients():
    rng=np.random.default_rng(7);design=rng.normal(size=(200,3));expected=np.array([20.0,.4,.15])
    response=design@expected
    fitted,rmse,_=_fit([(design,response)])
    assert fitted==pytest.approx(expected)
    assert rmse<1e-12


def test_real_vertical_identification_is_positive_bounded_and_reproducible(tmp_path):
    report=identify_vertical_dynamics(
        "calibration/runs/20261026T064847_690179Z_pitch-response-id",
        "calibration/runs/20261026T065908_616181Z_cascaded-5c",tmp_path,bootstrap_samples=100)
    assert report["selected_delay_s"]==pytest.approx(.10)
    assert report["estimate"]["a_m_s2_per_normalized_thrust"]>0
    assert report["recommended_candidate"]["bootstrap_stable_fraction"]>.95
    for run in report["recommended_candidate"]["historical_state_replay"].values():
        assert run["zero_reference"]["saturation_samples"]==0
        assert .38<run["zero_reference"]["overall_thrust_range"][0]
        assert run["zero_reference"]["overall_thrust_range"][1]<.45
    assert (tmp_path/"vertical_system_id.json").exists()


def test_pole_candidate_changes_only_altitude_design_parameters():
    prior=load_cascaded_config("calibration/stage5c_altitude_pd_zero_velocity_reference_candidate_config.json")
    candidate=load_cascaded_config("calibration/stage5c_altitude_pd_pole_candidate_config.json")
    assert (candidate.altitude.position_kp,candidate.altitude.velocity_kp)==(.06,.06)
    assert candidate.altitude.velocity_reference_tracking_enabled is False
    assert candidate.equilibrium_thrust==prior.equilibrium_thrust
    assert (candidate.min_thrust,candidate.max_thrust)==(prior.min_thrust,prior.max_thrust)
    assert candidate.attitude==prior.attitude and candidate.horizontal==prior.horizontal
