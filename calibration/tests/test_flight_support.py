import json

from calibration.config import CalibrationConfig
from calibration.flight_support import print_run_summary, record_battery_snapshot
from calibration.logger import ExperimentLogger


class BatteryHardware:
    def __init__(self,snapshot): self.snapshot=snapshot
    def battery_snapshot(self): return self.snapshot


def test_battery_snapshot_is_persisted_and_printed(tmp_path):
    logger=ExperimentLogger(tmp_path,"pid",CalibrationConfig(output_directory=str(tmp_path)))
    lines=[]
    snapshot={"success":True,"requested_utc":"r","measurement_utc":"m",
        "measurement_monotonic_s":1.0,"firmware_log_timestamp_ms":5,
        "voltage_v":3.87,"percentage":None,"percentage_source":None,"error":None}
    record_battery_snapshot(BatteryHardware(snapshot),logger,lines.append)
    persisted=json.loads((logger.directory/"metadata.json").read_text())
    assert persisted["battery_snapshot"]==snapshot
    assert lines==["[BATTERY] voltage=3.870 V percentage=unavailable timestamp=m"]
    logger.close("complete")


def test_unavailable_battery_and_exceptional_run_paths_are_printed(tmp_path):
    logger=ExperimentLogger(tmp_path,"pid",CalibrationConfig(output_directory=str(tmp_path)))
    lines=[]
    unavailable={"success":False,"requested_utc":"r","measurement_utc":None,
        "measurement_monotonic_s":None,"firmware_log_timestamp_ms":None,
        "voltage_v":None,"percentage":None,"percentage_source":None,
        "error":"pm.vbat unavailable"}
    record_battery_snapshot(BatteryHardware(unavailable),logger,lines.append)
    logger.safety_event("tilt_limit_exceeded",{})
    logger.close("invalid","hard safety")
    print_run_summary(logger,lines.append)
    assert "[BATTERY] unavailable" in lines[0]
    assert lines[-3].startswith("[RUN] ") and str(logger.directory.resolve()) in lines[-3]
    assert lines[-2].endswith("/telemetry.csv")
    assert lines[-1]=="[RESULT] experiment=pid status=invalid reason=tilt_limit_exceeded"


def test_normal_run_summary_includes_status_and_paths(tmp_path):
    logger=ExperimentLogger(tmp_path,"pid",CalibrationConfig(output_directory=str(tmp_path)))
    logger.close("complete","profile completed");lines=[]
    print_run_summary(logger,lines.append)
    assert lines[-1]=="[RESULT] experiment=pid status=complete reason=profile completed"
    assert all(line.startswith(("[RUN] ","[LOG] ","[RESULT] ")) for line in lines)
