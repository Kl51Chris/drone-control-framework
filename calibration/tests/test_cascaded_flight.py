import csv
from dataclasses import replace
import json

import pytest

from calibration.cascaded_analysis import analyze_cascaded_flight
from calibration.cascaded_flight import CascadedFlightLogger, CascadedStage5BRunner
from calibration.cascaded_pid import CascadedPIDConfig
from calibration.config import CalibrationConfig, ProfileSettings, SafetySettings
from calibration.state import AltitudeState


class Clock:
    def __init__(self): self.now=10.0
    def __call__(self): return self.now
    def sleep(self,value): self.now+=value


class Hardware:
    def __init__(self,clock,unsafe=False,send_error=None):
        self.clock=clock;self.armed=False;self.unsafe=unsafe;self.send_error=send_error
        self.calls=[];self.commands=[]
    def state(self):
        tilt=.5 if self.armed and self.unsafe else .01
        return AltitudeState(0,0,tilt,-.01,0,self.clock(),"test",0,0,0,0,.1,-.2,.3)
    def supervisor_status(self):
        return {"can_be_armed":True,"can_fly":self.armed,"is_armed":self.armed,
            "is_auto_armed":False,"is_flying":self.armed,"is_tumbled":False,
            "is_locked":False,"is_crashed":False,"hl_control_active":False}
    def flow_deck_available(self):return True
    def emergency_stop_available(self):return True
    def protocol_version(self):return 12
    def arm(self):self.calls.append("arm");self.armed=True
    def send_command(self,command):
        self.calls.append("send");self.commands.append(command)
        if self.send_error:
            error=self.send_error;self.send_error=None;raise error
    def supervisor_emergency_stop(self):self.calls.append("supervisor_estop");self.armed=False
    def emergency_stop(self):self.calls.append("stop")
    def disarm(self):self.calls.append("disarm");self.armed=False


def config(tmp_path):
    return CalibrationConfig(control_frequency_hz=10,output_directory=str(tmp_path),
        profile=ProfileSettings(takeoff_height_m=.02,takeoff_rate_m_s=.2,
            hover_duration_s=.1,landing_rate_m_s=.2,settle_duration_s=0),
        safety=SafetySettings(max_loop_dt_s=.2))


def run_parts(tmp_path,hardware,clock):
    cfg=config(tmp_path);cc=CascadedPIDConfig();logger=CascadedFlightLogger(tmp_path,cfg,cc)
    runner=CascadedStage5BRunner(hardware,cfg,cc,logger,clock=clock,sleeper=clock.sleep)
    return runner,logger


def test_normal_profile_logs_full_state_commands_and_finalizes(tmp_path):
    clock=Clock();hardware=Hardware(clock);runner,logger=run_parts(tmp_path,hardware,clock)
    runner.run()
    with (logger.directory/"flight_telemetry.csv").open(newline="") as stream:rows=list(csv.DictReader(stream))
    assert rows and {"TAKEOFF","HOVER","LANDING"}.issubset({r["phase"] for r in rows})
    assert rows[0]["p_rad_s"]=="0.1" and rows[0]["command_thrust_normalized"]
    assert hardware.calls[-2:]==["stop","disarm"]
    assert json.loads((logger.directory/"metadata.json").read_text())["status"]=="complete"
    report=analyze_cascaded_flight(logger.directory)
    assert report["metrics_available"] and (logger.directory/"flight_response.svg").exists()


def test_hard_safety_estops_before_persisting_zero_row_snapshot(tmp_path):
    clock=Clock();hardware=Hardware(clock,unsafe=True);runner,logger=run_parts(tmp_path,hardware,clock)
    with pytest.raises(RuntimeError,match="tilt"):
        runner.run()
    assert hardware.calls[-3:]==["supervisor_estop","stop","disarm"]
    with (logger.directory/"flight_telemetry.csv").open(newline="") as stream: assert len(list(csv.DictReader(stream)))==0
    metadata=json.loads((logger.directory/"metadata.json").read_text())
    assert metadata["safety_event"]["condition"]=="tilt_limit_exceeded"
    assert metadata["safety_event"]["state"]["roll_rad"]==.5
    report=analyze_cascaded_flight(logger.directory)
    assert report["metrics_available"] is False and report["safety_event"]


@pytest.mark.parametrize(("error","status"),[(KeyboardInterrupt(),"interrupted"),(OSError("radio lost"),"aborted")])
def test_interrupt_and_unexpected_send_failure_shutdown_and_finalize(tmp_path,error,status):
    clock=Clock();hardware=Hardware(clock,send_error=error);runner,logger=run_parts(tmp_path,hardware,clock)
    with pytest.raises(BaseException): runner.run()
    metadata=json.loads((logger.directory/"metadata.json").read_text())
    assert metadata["status"]==status
    assert metadata["shutdown"]["stop_setpoint_attempted"]
    assert hardware.calls[-2:]==["stop","disarm"]
    with (logger.directory/"flight_telemetry.csv").open(newline="") as stream: assert len(list(csv.DictReader(stream)))==1
    report=analyze_cascaded_flight(logger.directory)
    assert report["metrics_available"] is True
    assert len(report["controller_contributions_immediately_preceding_failure"])==1


def test_preflight_exception_still_attempts_shutdown_and_finalizes(tmp_path):
    clock=Clock();hardware=Hardware(clock);hardware.flow_deck_available=lambda:False
    runner,logger=run_parts(tmp_path,hardware,clock)
    with pytest.raises(RuntimeError,match="Flow Deck"):runner.run()
    metadata=json.loads((logger.directory/"metadata.json").read_text())
    assert metadata["status"]=="aborted" and hardware.calls==["stop","disarm"]
