import csv
import json

import pytest

from calibration.config import PIDSettings
from calibration.pid_replay import ReplaySettings, replay_official_hover


def make_run(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "metadata.json").write_text(json.dumps({"mode": "official-hover-baseline"}))
    fields = ["elapsed_s", "dt_s", "phase", "reference_altitude_m", "z_m", "vz_m_s"]
    with (run / "flight_telemetry.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows([
            {"elapsed_s": .02, "dt_s": .02, "phase": "TAKEOFF", "reference_altitude_m": .1, "z_m": 0, "vz_m_s": 0},
            {"elapsed_s": .04, "dt_s": .02, "phase": "TAKEOFF", "reference_altitude_m": .1, "z_m": .2, "vz_m_s": 1},
        ])
    return run


def test_replay_logs_terms_and_both_saturation_layers(tmp_path):
    settings = ReplaySettings(.4, PIDSettings(kp=1, ki=0, kd=0,
        correction_min=-.05, correction_max=.05), .38, .43)
    report = replay_official_hover(make_run(tmp_path), tmp_path / "out", settings)
    assert report["sample_count"] == 2
    assert report["pid_saturation_count"] == 2
    assert report["thrust_saturation_count"] == 2
    with (tmp_path / "out" / "pid_replay.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["p_term"] == "0.1"
    assert rows[0]["predicted_thrust_normalized"] == "0.43"
    assert "does not predict" in report["interpretation"]


def test_replay_rejects_wrong_source_and_invalid_bounds(tmp_path):
    run = make_run(tmp_path)
    (run / "metadata.json").write_text(json.dumps({"mode": "pid"}))
    settings = ReplaySettings(.4, PIDSettings(), .3, .5)
    with pytest.raises(ValueError, match="official-hover"):
        replay_official_hover(run, tmp_path / "out", settings)
    with pytest.raises(ValueError, match="min <= equilibrium"):
        ReplaySettings(.2, PIDSettings(), .3, .5).validate()
