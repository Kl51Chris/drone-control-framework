import csv

import pytest

from calibration.cli import main
from calibration.config import CalibrationConfig
from calibration.official_hover import OfficialHoverLogger, OfficialHoverRunner, OfficialHoverSettings
from calibration.state import AltitudeState


class Clock:
    def __init__(self): self.now = 10.0
    def __call__(self): return self.now
    def sleep(self, duration): self.now += duration


class HoverHardware:
    def __init__(self, clock, *, flow=True, stale_after=None, fail_send_once=False, interrupt_send=False):
        self.clock=clock; self.flow=flow; self.stale_after=stale_after; self.fail_send_once=fail_send_once
        self.interrupt_send=interrupt_send
        self.states=0; self.armed=False; self.hover_calls=[]; self.stop_calls=0; self.notify_calls=0
        self.disarm_calls=0; self.estop_calls=0; self.reference=0.0
    def state(self):
        self.states+=1; timestamp=self.clock()-(1.0 if self.stale_after and self.states>=self.stale_after else 0)
        return AltitudeState(self.reference,0,0.01,-0.01,0,timestamp,"crazyflie",0.01,-0.01,0,0)
    def supervisor_status(self):
        return {"can_be_armed":True,"can_fly":self.armed,"is_armed":self.armed,"is_auto_armed":False,
                "is_flying":self.armed,"is_tumbled":False,"is_locked":False,"is_crashed":False,"hl_control_active":False}
    def flow_deck_available(self): return self.flow
    def emergency_stop_available(self): return True
    def protocol_version(self): return 12
    def start_onboard_thrust_log(self,period_ms=20): return True
    def onboard_thrust_raw(self): return (32000.0,self.clock())
    def arm(self): self.armed=True
    def disarm(self): self.disarm_calls+=1;self.armed=False
    def send_hover_setpoint(self,height_m):
        if self.interrupt_send:
            self.interrupt_send=False;raise KeyboardInterrupt
        if self.fail_send_once:
            self.fail_send_once=False;raise RuntimeError("one command failure")
        self.reference=height_m;self.hover_calls.append(height_m)
    def notify_setpoint_stop(self): self.notify_calls+=1
    def emergency_stop(self): self.stop_calls+=1
    def supervisor_emergency_stop(self): self.estop_calls+=1;self.armed=False


def settings():
    return OfficialHoverSettings(target_altitude_m=.10,hover_duration_s=.20,takeoff_rate_m_s=.5,
        landing_rate_m_s=.5,command_frequency_hz=10,start_height_m=.05,landing_height_m=.03,
        landing_confirmation_s=.1,landing_timeout_s=.5,landing_max_height_m=.06,
        landing_max_vertical_speed_m_s=.08,estimator_check_s=.04)


def test_flight_requires_explicit_cli_authorization():
    with pytest.raises(SystemExit,match="Flight refused"):
        main(["run","official-hover-baseline"])


def test_normal_official_hover_calls_and_state_machine(tmp_path):
    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=HoverHardware(clock)
    logger=OfficialHoverLogger(tmp_path,config,settings())
    metrics=OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep,output=lambda _:None).run()
    assert metrics["performance_pass"] is True
    assert hardware.hover_calls
    assert max(hardware.hover_calls)<=.10
    assert hardware.stop_calls==1 and hardware.notify_calls==1 and hardware.disarm_calls==1
    with (logger.directory/"flight_telemetry.csv").open(newline="") as stream: rows=list(csv.DictReader(stream))
    assert {row["phase"] for row in rows}=={"TAKEOFF","HOVER","LANDING","LANDING_CONFIRM"}
    assert all(row["onboard_thrust_raw"]=="32000.0" for row in rows)
    assert (logger.directory/"metrics.json").exists() and (logger.directory/"flight_response.svg").exists()


def test_preflight_rejects_missing_flow_deck_without_arming(tmp_path):
    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=HoverHardware(clock,flow=False)
    logger=OfficialHoverLogger(tmp_path,config,settings())
    with pytest.raises(RuntimeError,match="Flow Deck"):
        OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep).run()
    assert hardware.armed is False and hardware.hover_calls==[]
    logger.close("invalid","test","not_started")


def test_stale_inflight_state_triggers_supervisor_emergency_stop(tmp_path):
    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=HoverHardware(clock,stale_after=5)
    logger=OfficialHoverLogger(tmp_path,config,settings())
    metrics=OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep).run()
    assert metrics["software_execution_status"]=="aborted"
    assert metrics["landing_status"]=="emergency_motor_shutdown"
    assert hardware.estop_calls==3


def test_recoverable_command_fault_attempts_controlled_landing(tmp_path):
    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=HoverHardware(clock,fail_send_once=True)
    logger=OfficialHoverLogger(tmp_path,config,settings())
    metrics=OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep).run()
    assert metrics["software_execution_status"]=="aborted"
    assert metrics["landing_status"]=="recoverable_controlled_landing_complete"
    assert hardware.stop_calls==1 and hardware.disarm_calls==1


def test_ctrl_c_is_immediate_stop_not_safe_landing(tmp_path):
    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=HoverHardware(clock,interrupt_send=True)
    logger=OfficialHoverLogger(tmp_path,config,settings())
    metrics=OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep).run()
    assert metrics["software_execution_status"]=="interrupted"
    assert metrics["landing_status"]=="immediate_lift_termination"
    assert hardware.stop_calls==1 and hardware.disarm_calls==1


def test_profile_is_offset_from_measured_ground_height(tmp_path):
    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=HoverHardware(clock)
    hardware.reference=.12
    logger=OfficialHoverLogger(tmp_path,config,settings())
    OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep,output=lambda _:None).run()
    assert max(hardware.hover_calls)==pytest.approx(.22)
    assert logger.metadata["ground_altitude_m"]==pytest.approx(.12)


def test_protocol_before_supervisor_port_is_rejected_without_arming(tmp_path):
    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=HoverHardware(clock)
    hardware.protocol_version=lambda:11
    logger=OfficialHoverLogger(tmp_path,config,settings())
    with pytest.raises(RuntimeError,match="protocol version 12"):
        OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep).run()
    assert hardware.armed is False
    logger.close("invalid","test","not_started")


def test_motor_stop_is_withheld_when_touchdown_is_not_confirmed(tmp_path):
    class HoveringHardware(HoverHardware):
        def state(self):
            state=super().state()
            if self.armed and self.hover_calls and max(self.hover_calls)>=.10:
                return AltitudeState(.10,0,0.01,-0.01,0,self.clock(),"crazyflie",0.01,-0.01,0,0)
            return state

    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=HoveringHardware(clock)
    logger=OfficialHoverLogger(tmp_path,config,settings())
    metrics=OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep,output=lambda _:None).run()
    assert metrics["software_execution_status"]=="aborted"
    assert "touchdown was not confirmed" in metrics["termination_reason"]
    assert hardware.stop_calls==0
    assert hardware.estop_calls==3


@pytest.mark.parametrize(
    ("position_x", "velocity_x", "condition", "reason_fragment"),
    [
        (.51, 0.0, "horizontal_displacement_limit_exceeded", "displacement limit exceeded"),
        (0.0, .76, "horizontal_speed_limit_exceeded", "speed limit exceeded"),
    ],
)
def test_first_inflight_horizontal_abort_persists_exact_diagnostics(
        tmp_path, position_x, velocity_x, condition, reason_fragment):
    class UnsafeAfterArming(HoverHardware):
        def state(self):
            state = super().state()
            if self.armed:
                return AltitudeState(
                    state.altitude_m, state.vertical_velocity_m_s, state.roll_rad,
                    state.pitch_rad, state.yaw_rad, state.timestamp_s, state.source,
                    position_x, 0.0, velocity_x, 0.0,
                )
            return state

    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=UnsafeAfterArming(clock)
    logger=OfficialHoverLogger(tmp_path,config,settings())
    metrics=OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep).run()
    event=logger.metadata["safety_event"]
    assert metrics["sample_count"]==0
    assert reason_fragment in metrics["termination_reason"]
    assert event["condition"]==condition
    assert event["state"]["horizontal_displacement_m"]==pytest.approx(
        ((position_x-.01)**2 + .01**2)**.5)
    assert event["state"]["horizontal_speed_m_s"]==pytest.approx(abs(velocity_x))
    assert hardware.estop_calls==3


def test_preflight_records_median_horizontal_origin_and_velocity_summary(tmp_path):
    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=HoverHardware(clock)
    logger=OfficialHoverLogger(tmp_path,config,settings())
    runner=OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep)
    runner._preflight()
    summary=logger.metadata["horizontal_preflight"]
    assert summary["origin_method"].startswith("component-wise median")
    assert summary["origin_x_m"]==pytest.approx(.01)
    assert summary["origin_y_m"]==pytest.approx(-.01)
    assert summary["latest_horizontal_speed_m_s"]==0
    logger.close("invalid","test","not_started")


def test_unsafe_preflight_horizontal_velocity_refuses_to_arm(tmp_path):
    class BadVelocity(HoverHardware):
        def state(self):
            state=super().state()
            return AltitudeState(state.altitude_m,state.vertical_velocity_m_s,state.roll_rad,
                state.pitch_rad,state.yaw_rad,state.timestamp_s,state.source,
                state.position_x_m,state.position_y_m,.76,0)

    clock=Clock();config=CalibrationConfig(output_directory=str(tmp_path));hardware=BadVelocity(clock)
    logger=OfficialHoverLogger(tmp_path,config,settings())
    with pytest.raises(RuntimeError,match="preflight horizontal speed limit exceeded"):
        OfficialHoverRunner(hardware,config,logger,settings(),clock=clock,sleeper=clock.sleep).run()
    assert hardware.armed is False
    assert logger.metadata["safety_event"]["condition"]=="horizontal_speed_limit_exceeded_preflight"
    logger.close("invalid","test","not_started")
