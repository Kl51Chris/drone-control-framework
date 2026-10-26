import csv
import json

import pytest

from calibration.equilibrium_analysis import analyze_equilibrium, raw_to_toolkit_normalized


def test_raw_to_toolkit_normalized_matches_cflib_manual_mapping():
    assert raw_to_toolkit_normalized(10001)==0
    assert raw_to_toolkit_normalized(60000)==1
    assert raw_to_toolkit_normalized(35000.5)==pytest.approx(.5)


def test_analysis_selects_only_stable_fresh_hover_samples_and_writes_artifacts(tmp_path):
    metadata={"onboard_thrust_available":True,"cflib_version":"test"}
    (tmp_path/"metadata.json").write_text(json.dumps(metadata))
    fields=["phase","elapsed_s","reference_altitude_m","z_m","vz_m_s","roll_rad","pitch_rad","onboard_thrust_raw","onboard_thrust_age_s"]
    rows=[
        ["TAKEOFF",0,0.3,0.1,0,0,0,20000,.01],
        ["HOVER",1,0.3,0.2,0,0,0,20000,.01],
        ["HOVER",2,0.3,.29,.01,.01,.02,30000,.01],
        ["HOVER",3,0.3,.31,-.01,.02,.01,32000,.01],
        ["HOVER",4,0.3,.3,0,.01,.01,65000,.06],
    ]
    with (tmp_path/"flight_telemetry.csv").open("w",newline="") as stream:
        writer=csv.writer(stream);writer.writerow(fields);writer.writerows(rows)
    report=analyze_equilibrium(tmp_path)
    assert report["selection"]["stable_samples"]==2
    assert report["measured"]["stabilizer_thrust_raw"]["mean"]==31000
    assert report["inferred"]["toolkit_normalized_equivalent"]["mean"]==pytest.approx((31000-10001)/49999)
    assert (tmp_path/"equilibrium_thrust_report.json").exists()
    assert (tmp_path/"equilibrium_thrust_report.md").exists()
    assert (tmp_path/"equilibrium_thrust_analysis.svg").exists()
