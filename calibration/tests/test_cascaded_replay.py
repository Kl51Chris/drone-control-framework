import json

from calibration.cascaded_pid import load_cascaded_config
from calibration.cascaded_replay import replay_cascaded_official_hover


def test_real_official_hover_replay_records_all_controller_outputs(tmp_path):
    report = replay_cascaded_official_hover(
        "calibration/runs/20261026T045302_143713Z_official-hover-baseline",
        tmp_path, load_cascaded_config("calibration/stage5b_offline_config.json"))
    assert report["sample_count"] == 394
    assert report["controller_stage"] == "5B"
    assert report["saturation_counts"]["altitude_saturated"] == 0
    assert (tmp_path / "cascaded_replay.csv").exists()
    saved = json.loads((tmp_path / "cascaded_replay_report.json").read_text())
    assert "not a closed-loop simulation" in saved["interpretation"]
