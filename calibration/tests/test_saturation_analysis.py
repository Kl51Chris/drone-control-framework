import csv

from calibration.saturation_analysis import analyze_rate_saturation


def test_phase_breakdown_runs_and_pitch_firmware_restoring_sign(tmp_path):
    path = tmp_path / "replay.csv"
    fields = ["elapsed_s", "dt_s", "phase", "desired_yaw_rad", "measured_yaw_rad"]
    for axis in ("roll", "pitch", "yaw"):
        fields += [f"{axis}_error_rad", f"{axis}_rate_raw_rad_s",
                   f"{axis}_rate_saturated", f"command_{axis}_rate_rad_s"]
    base = dict(desired_yaw_rad=0, measured_yaw_rad=0,
                roll_error_rad=.1, roll_rate_raw_rad_s=.4, command_roll_rate_rad_s=.35,
                pitch_error_rad=-.1, pitch_rate_raw_rad_s=.4, command_pitch_rate_rad_s=.35,
                yaw_error_rad=.2, yaw_rate_raw_rad_s=.4, command_yaw_rate_rad_s=.35)
    rows = [
        dict(base, elapsed_s=0, dt_s=.02, phase="TAKEOFF", roll_rate_saturated=1, pitch_rate_saturated=1, yaw_rate_saturated=1),
        dict(base, elapsed_s=.02, dt_s=.02, phase="TAKEOFF", roll_rate_saturated=1, pitch_rate_saturated=0, yaw_rate_saturated=1),
        dict(base, elapsed_s=.04, dt_s=.02, phase="LANDING_CONFIRM", roll_rate_saturated=0, pitch_rate_saturated=0, yaw_rate_saturated=0),
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    report = analyze_rate_saturation(path, tmp_path / "report.json")
    assert report["saturation_sample_counts"]["roll"]["TAKEOFF"] == 2
    assert report["consecutive_saturation_runs"]["roll"][0]["sample_count"] == 2
    assert report["consecutive_saturation_runs"]["roll"][0]["estimated_duration_s"] == .04
    assert report["restoring_direction_check"]["pitch"]["all_restoring"] is True
    assert report["interface"]["duplicated_in_adapter"] is False
