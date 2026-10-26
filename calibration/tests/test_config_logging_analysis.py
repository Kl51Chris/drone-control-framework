import csv
import json

import pytest

from calibration.analysis import analyze, export_validated
from calibration.config import CalibrationConfig, load_config, save_config
from calibration.logger import ExperimentLogger, FIELDS


def test_config_round_trip(tmp_path):
    original = CalibrationConfig(equilibrium_thrust=.25)
    path = tmp_path / "config.json"
    save_config(original, path)
    assert load_config(path) == original


def test_logging_analysis_and_export(tmp_path):
    config = CalibrationConfig(output_directory=str(tmp_path))
    logger = ExperimentLogger(tmp_path, "pid", config)
    row = dict.fromkeys(FIELDS, 0)
    row.update(phase="HOVER", state_source="test", dt_s=.02, altitude_reference_m=1, altitude_m=.9, elapsed_s=.02)
    logger.row(row); row["elapsed_s"] = .04; logger.row(row); logger.close("complete")
    metrics = analyze(logger.directory)
    assert metrics["valid"] is True
    assert metrics["rmse_m"] == pytest.approx(.1)
    assert (logger.directory / "response.svg").exists()
    output = tmp_path / "validated.json"
    exported = export_validated(logger.directory, output)
    assert exported["outer_loop_mapping"]["PIDConfig.kp"] == config.pid.kp
    assert json.loads(output.read_text())["hardware_validated"] is False


def test_export_rejects_invalid_run(tmp_path):
    logger = ExperimentLogger(tmp_path, "pid", CalibrationConfig(output_directory=str(tmp_path)))
    logger.close("invalid", "test")
    with pytest.raises(ValueError):
        export_validated(logger.directory, tmp_path / "bad.json")
