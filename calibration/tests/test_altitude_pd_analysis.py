import csv
import json

import pytest

from calibration.altitude_pd_analysis import analyze_altitude_pd
from calibration.cascaded_pid import load_cascaded_config


RUN="calibration/runs/20261026T064847_690179Z_pitch-response-id"


def test_real_flight_pd_replay_is_bounded_and_reproducible(tmp_path):
    report=analyze_altitude_pd(RUN,tmp_path)
    assert report["sample_count"]==488
    assert report["oscillation"]["estimated_period_s"]==pytest.approx(3.80006,abs=.02)
    assert report["oscillation"]["half_amplitude_m"]==pytest.approx(.12112,abs=.002)
    candidate=report["candidate_velocity_gains"]["0.040"]
    assert candidate["minimum_thrust"]==pytest.approx(.4020619)
    assert candidate["maximum_thrust"]==pytest.approx(.4331083)
    assert candidate["saturation_samples"]==0
    assert (tmp_path/"altitude_pd_replay.csv").exists()
    assert (tmp_path/"altitude_pd_replay.svg").exists()
    saved=json.loads((tmp_path/"altitude_pd_analysis.json").read_text())
    assert "not a closed-loop simulation" in saved["interpretation"]


def test_stage5c_candidate_changes_only_altitude_velocity_feedback():
    p_only=load_cascaded_config("calibration/stage5b_offline_config.json")
    pd=load_cascaded_config("calibration/stage5c_altitude_pd_candidate_config.json")
    assert pd.stage=="5C" and pd.altitude.velocity_feedback_enabled
    assert pd.altitude.velocity_kp==pytest.approx(.04)
    assert pd.altitude.position_kp==p_only.altitude.position_kp
    assert pd.equilibrium_thrust==p_only.equilibrium_thrust
    assert (pd.min_thrust,pd.max_thrust)==(p_only.min_thrust,p_only.max_thrust)
    assert pd.attitude==p_only.attitude and pd.horizontal==p_only.horizontal
    assert pd.altitude.integral_enabled is False


def test_zero_velocity_reference_candidate_preserves_stage5c_gains_and_bounds():
    tracking=load_cascaded_config("calibration/stage5c_altitude_pd_candidate_config.json")
    zero=load_cascaded_config("calibration/stage5c_altitude_pd_zero_velocity_reference_candidate_config.json")
    assert zero.altitude.velocity_reference_tracking_enabled is False
    zero.altitude.velocity_reference_tracking_enabled=True
    assert zero==tracking
