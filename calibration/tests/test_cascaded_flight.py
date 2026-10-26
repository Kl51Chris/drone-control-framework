import csv
from dataclasses import replace
import json

import pytest

from calibration.cascaded_analysis import analyze_cascaded_flight
from calibration.cascaded_flight import (CascadedFlightLogger, CascadedStage5BRunner,
    PitchIdentificationSettings, RollIdentificationSettings)
from calibration.cascaded_pid import CascadedPIDConfig
from calibration.pitch_response_analysis import analyze_pitch_response
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
    assert {float(r["reference_vz_m_s"]) for r in rows if r["phase"]=="TAKEOFF"}=={runner.config.profile.takeoff_rate_m_s}
    assert {float(r["reference_vz_m_s"]) for r in rows if r["phase"]=="HOVER"}=={0.0}
    assert {float(r["reference_vz_m_s"]) for r in rows if r["phase"]=="LANDING"}=={-runner.config.profile.landing_rate_m_s}
    assert hardware.calls[-2:]==["stop","disarm"]
    assert json.loads((logger.directory/"metadata.json").read_text())["status"]=="complete"
    report=analyze_cascaded_flight(logger.directory)
    assert report["metrics_available"] and (logger.directory/"flight_response.svg").exists()


def test_landing_confirmation_continues_virtual_descent_until_touchdown_confirmed(tmp_path):
    clock=Clock();hardware=Hardware(clock);runner,logger=run_parts(tmp_path,hardware,clock)
    runner.run()
    with (logger.directory/"flight_telemetry.csv").open(newline="") as stream:
        rows=list(csv.DictReader(stream))
    confirmation=[row for row in rows if row["phase"]=="LANDING_CONFIRM"]
    assert len(confirmation)>=2
    references=[float(row["reference_z_m"]) for row in confirmation]
    assert references[-1] < references[0]
    assert all(float(row["reference_vz_m_s"]) == -runner.config.profile.landing_rate_m_s
               for row in confirmation)
    assert min(float(row["command_thrust_normalized"]) for row in confirmation) >= runner.controller_config.min_thrust
    assert hardware.calls[-2:]==["stop","disarm"]


def test_failed_touchdown_uses_extended_timeout_then_emergency_shutdown(tmp_path):
    class AirborneHardware(Hardware):
        def state(self):
            state=super().state()
            return replace(state,altitude_m=.10 if self.armed else 0,
                           vertical_velocity_m_s=0)

    clock=Clock();hardware=AirborneHardware(clock);runner,logger=run_parts(tmp_path,hardware,clock)
    assert runner.LANDING_TIMEOUT_S==4.0
    with pytest.raises(RuntimeError,match="touchdown was not confirmed"):
        runner.run()
    with (logger.directory/"flight_telemetry.csv").open(newline="") as stream:
        rows=list(csv.DictReader(stream))
    confirmation=[row for row in rows if row["phase"]=="LANDING_CONFIRM"]
    duration=float(confirmation[-1]["elapsed_s"])-float(confirmation[0]["elapsed_s"])
    assert duration>=3.8
    assert float(confirmation[-1]["reference_z_m"]) < float(confirmation[0]["reference_z_m"])
    metadata=json.loads((logger.directory/"metadata.json").read_text())
    assert metadata["safety_event"]["condition"]=="touchdown_not_confirmed"
    assert metadata["status"]=="aborted"
    assert hardware.calls[-3:]==["supervisor_estop","stop","disarm"]


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


def test_unsettled_horizontal_velocity_is_rejected_before_arming(tmp_path):
    class MovingEstimator(Hardware):
        def state(self):
            state=super().state()
            return replace(state,velocity_x_m_s=.2)

    clock=Clock();hardware=MovingEstimator(clock);runner,logger=run_parts(tmp_path,hardware,clock)
    with pytest.raises(RuntimeError,match="preflight horizontal velocity"):
        runner.run()
    assert "arm" not in hardware.calls and "send" not in hardware.calls
    metadata=json.loads((logger.directory/"metadata.json").read_text())
    assert metadata["safety_event"]["condition"]=="preflight_horizontal_velocity_unsettled"
    assert metadata["horizontal_estimator_readiness"]["maximum_horizontal_speed_m_s"]==pytest.approx(.2)


def test_unstable_horizontal_position_is_rejected_before_arming(tmp_path):
    class JumpingPreflightEstimator(Hardware):
        def __init__(self,clock):
            super().__init__(clock);self.samples=0
        def state(self):
            state=super().state();self.samples+=1
            return replace(state,position_x_m=.12 if self.samples%2 else 0)

    clock=Clock();hardware=JumpingPreflightEstimator(clock);runner,logger=run_parts(tmp_path,hardware,clock)
    with pytest.raises(RuntimeError,match="preflight horizontal position"):
        runner.run()
    assert "arm" not in hardware.calls
    metadata=json.loads((logger.directory/"metadata.json").read_text())
    assert metadata["safety_event"]["condition"]=="preflight_horizontal_position_unsettled"


def test_post_arm_velocity_change_is_rejected_before_first_command(tmp_path):
    class MovesAfterArming(Hardware):
        def state(self):
            state=super().state()
            return replace(state,velocity_x_m_s=.2 if self.armed else 0)

    clock=Clock();hardware=MovesAfterArming(clock);runner,logger=run_parts(tmp_path,hardware,clock)
    with pytest.raises(RuntimeError,match="before the first command"):
        runner.run()
    assert "arm" in hardware.calls and "send" not in hardware.calls
    metadata=json.loads((logger.directory/"metadata.json").read_text())
    assert metadata["safety_event"]["condition"]=="post_arm_horizontal_velocity_unsettled"


def test_inflight_position_discontinuity_estops_before_next_command(tmp_path):
    class JumpingEstimator(Hardware):
        def __init__(self,clock):
            super().__init__(clock);self.armed_states=0
        def state(self):
            state=super().state()
            if self.armed:
                self.armed_states+=1
                if self.armed_states>=3:
                    return replace(state,position_x_m=.3)
            return state

    clock=Clock();hardware=JumpingEstimator(clock);runner,logger=run_parts(tmp_path,hardware,clock)
    with pytest.raises(RuntimeError,match="inconsistently"):
        runner.run()
    assert hardware.calls.count("send")==1
    assert hardware.calls[-3:]==["supervisor_estop","stop","disarm"]
    metadata=json.loads((logger.directory/"metadata.json").read_text())
    event=metadata["safety_event"]
    assert event["condition"]=="estimator_position_discontinuity"
    assert event["estimator_continuity"]["position_velocity_residual_m"]==pytest.approx(.3)


def test_pitch_identification_uses_fixed_bipolar_timeline_and_lands_at_2_5_s(tmp_path):
    clock=Clock();hardware=Hardware(clock)
    cfg=config(tmp_path);cfg.profile=replace(cfg.profile,hover_duration_s=4.0)
    cc=CascadedPIDConfig();logger=CascadedFlightLogger(tmp_path,cfg,cc,experiment_mode="pitch-response-id")
    CascadedStage5BRunner(hardware,cfg,cc,logger,clock=clock,sleeper=clock.sleep,
        pitch_identification=PitchIdentificationSettings()).run()
    with (logger.directory/"flight_telemetry.csv").open(newline="") as stream:rows=list(csv.DictReader(stream))
    phases={row["pitch_identification_phase"] for row in rows}
    assert {"SAFETY_ASSESSMENT","NEUTRAL_BEFORE","PITCH_POSITIVE","RECOVERY_POSITIVE",
            "PITCH_NEGATIVE","RECOVERY_NEGATIVE","COMPLETE"}.issubset(phases)
    assert all(float(row["pitch_excitation_reference_rad"])==0 for row in rows if row["phase"]!="HOVER")
    assert max(float(row["pitch_excitation_reference_rad"]) for row in rows)==.02
    assert min(float(row["pitch_excitation_reference_rad"]) for row in rows)==-.02
    metadata=json.loads((logger.directory/"metadata.json").read_text())["pitch_identification"]
    assert metadata["status"]=="complete" and metadata["excitation_performed"] is True
    first_landing=next(row for row in rows if row["phase"]=="LANDING")
    assert 2.5<=float(first_landing["elapsed_s"])<2.61
    assert metadata["pulse_margin_checks"]["positive"]["passed"] is True
    assert metadata["pulse_margin_checks"]["negative"]["passed"] is True


def test_pitch_identification_skips_pulse_without_envelope_margin_and_lands(tmp_path):
    class DriftingHardware(Hardware):
        def arm(self):
            super().arm();self.arm_time=self.clock()
        def state(self):
            state=super().state()
            control_elapsed=self.clock()-getattr(self,"arm_time",self.clock())-.25
            if not self.armed:return state
            if control_elapsed<=.1:x=0;vx=0
            elif control_elapsed<=1.9:x=.2*(control_elapsed-.1);vx=.2
            else:x=.36-.2*(control_elapsed-1.9);vx=-.2
            return replace(state,position_x_m=x,velocity_x_m_s=vx)

    clock=Clock();hardware=DriftingHardware(clock)
    cfg=config(tmp_path);cfg.profile=replace(cfg.profile,hover_duration_s=4.0)
    cc=CascadedPIDConfig();logger=CascadedFlightLogger(tmp_path,cfg,cc,experiment_mode="pitch-response-id")
    CascadedStage5BRunner(hardware,cfg,cc,logger,clock=clock,sleeper=clock.sleep,
        pitch_identification=PitchIdentificationSettings()).run()
    with (logger.directory/"flight_telemetry.csv").open(newline="") as stream:rows=list(csv.DictReader(stream))
    metadata=json.loads((logger.directory/"metadata.json").read_text())
    identification=metadata["pitch_identification"]
    assert identification["status"]=="inconclusive"
    assert identification["excitation_performed"] is False
    assert identification["pulse_margin_checks"]["positive"]["passed"] is True
    assert identification["pulse_margin_checks"]["negative"]["passed"] is False
    assert "SKIPPED_NEGATIVE" in {row["pitch_identification_phase"] for row in rows}
    assert "LANDING" in {row["phase"] for row in rows}
    assert metadata["status"]=="complete" and "safety_event" not in metadata
    assert analyze_pitch_response(logger.directory)["result"]=="INCONCLUSIVE"


def test_pitch_excitation_amplitude_is_configurable_only_within_bound():
    assert PitchIdentificationSettings(excitation_rad=.015).excitation_rad==.015
    with pytest.raises(ValueError,match="conservative validated bound"):
        PitchIdentificationSettings(excitation_rad=.021)


def test_roll_identification_reuses_fixed_timeline_without_pitch_excitation(tmp_path):
    clock=Clock();hardware=Hardware(clock)
    cfg=config(tmp_path);cfg.profile=replace(cfg.profile,hover_duration_s=4.0)
    cc=CascadedPIDConfig();logger=CascadedFlightLogger(
        tmp_path,cfg,cc,experiment_mode="roll-response-id")
    CascadedStage5BRunner(hardware,cfg,cc,logger,clock=clock,sleeper=clock.sleep,
        roll_identification=RollIdentificationSettings()).run()
    with (logger.directory/"flight_telemetry.csv").open(newline="") as stream:
        rows=list(csv.DictReader(stream))
    phases={row["roll_identification_phase"] for row in rows}
    assert {"ROLL_POSITIVE","ROLL_NEGATIVE","RECOVERY_POSITIVE",
            "RECOVERY_NEGATIVE"}.issubset(phases)
    assert max(float(row["roll_excitation_reference_rad"]) for row in rows)==.02
    assert min(float(row["roll_excitation_reference_rad"]) for row in rows)==-.02
    assert all(float(row["pitch_excitation_reference_rad"])==0 for row in rows)
    metadata=json.loads((logger.directory/"metadata.json").read_text())
    assert metadata["roll_identification"]["excitation_performed"] is True
    assert metadata["controller_config"]["horizontal"]["enabled"] is False
