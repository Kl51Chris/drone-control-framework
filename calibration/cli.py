from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path

from .analysis import analyze, export_validated
from .config import load_config
from .experiment import CalibrationRunner, Mode
from .hardware import CrazyflieHardware
from .guided_attitude import GuidedAttitudeRunner
from .logger import ExperimentLogger
from .motor_free_diagnostic import generate_motor_free_report
from .official_hover import OfficialHoverLogger, OfficialHoverRunner, OfficialHoverSettings
from .state_monitor import StateMonitorLogger, StateMonitorRunner, StateMonitorSettings
from .equilibrium_analysis import analyze_equilibrium
from .pid_replay import replay_official_hover, settings_from_config
from .cascaded_pid import load_cascaded_config
from .cascaded_replay import replay_cascaded_official_hover, replay_cascaded_flight
from .cascaded_flight import (CascadedFlightLogger, CascadedStage5BRunner,
                              PitchIdentificationSettings, RollIdentificationSettings)
from .cascaded_analysis import analyze_cascaded_flight
from .pitch_response_analysis import analyze_pitch_response
from .roll_response_analysis import analyze_roll_response
from .altitude_pd_analysis import analyze_altitude_pd
from .vertical_system_id import identify_vertical_dynamics
from .flight_support import record_battery_snapshot, print_run_summary


FLIGHT_CONFIRMATION = "I_UNDERSTAND_PROPELLERS_MAY_SPIN"


def _validate_cascaded_mode_config(mode: Mode, controller_config) -> None:
    """Validate controller-stage compatibility before hardware connection."""
    controller_config.validate()
    if mode in (Mode.PITCH_RESPONSE_ID, Mode.ROLL_RESPONSE_ID):
        if controller_config.stage not in {"5B","5C"}:
            raise SystemExit(f"{mode.value} requires a compatible Stage 5B or Stage 5C controller configuration")
        if controller_config.horizontal.enabled:
            raise SystemExit(f"{mode.value} requires XY control disabled")
    else:
        expected_stage="5D" if mode is Mode.CASCADED_XY else ("5C" if mode is Mode.CASCADED_5C else "5B")
        if controller_config.stage!=expected_stage:
            raise SystemExit(f"{mode.value} requires a Stage {expected_stage} controller configuration")
    if mode is Mode.CASCADED_XY and not controller_config.horizontal_mapping_hardware_validated:
        raise SystemExit(
            "XY flight refused: horizontal_mapping_hardware_validated is false. "
            "Resolve and record physical body-X/pitch and body-Y/roll polarity first.")


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description="Independent Crazyflie altitude calibration toolkit")
    commands = root.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("mode", choices=[mode.value for mode in Mode])
    run.add_argument("--config", default="calibration/default_config.json")
    run.add_argument("--confirm-flight", default="", help=f"required literal: {FLIGHT_CONFIRMATION}")
    report = commands.add_parser("analyze"); report.add_argument("run_directory")
    equilibrium = commands.add_parser("analyze-equilibrium"); equilibrium.add_argument("run_directory")
    export = commands.add_parser("export"); export.add_argument("run_directory"); export.add_argument("destination")
    verify = commands.add_parser("verify-control"); verify.add_argument("run_directory"); verify.add_argument("--config", default="calibration/default_config.json")
    replay = commands.add_parser("replay-pid")
    replay.add_argument("run_directory"); replay.add_argument("output_directory")
    replay.add_argument("--config", default="calibration/default_config.json")
    replay.add_argument("--equilibrium-thrust", type=float)
    replay.add_argument("--kp", type=float); replay.add_argument("--ki", type=float); replay.add_argument("--kd", type=float)
    replay.add_argument("--correction-min", type=float); replay.add_argument("--correction-max", type=float)
    replay.add_argument("--min-thrust", type=float); replay.add_argument("--max-thrust", type=float)
    cascaded = commands.add_parser("replay-cascaded")
    cascaded.add_argument("run_directory"); cascaded.add_argument("output_directory")
    cascaded.add_argument("--controller-config", default="calibration/stage5b_offline_config.json")
    xy_replay = commands.add_parser("replay-cascaded-flight")
    xy_replay.add_argument("run_directory"); xy_replay.add_argument("output_directory")
    xy_replay.add_argument("--controller-config", default="calibration/stage5d_xy_first_flight_config.json")
    flight_analysis = commands.add_parser("analyze-cascaded-flight")
    flight_analysis.add_argument("run_directory")
    pitch_analysis = commands.add_parser("analyze-pitch-response")
    pitch_analysis.add_argument("run_directory")
    roll_analysis = commands.add_parser("analyze-roll-response")
    roll_analysis.add_argument("run_directory")
    altitude_pd = commands.add_parser("analyze-altitude-pd")
    altitude_pd.add_argument("run_directory"); altitude_pd.add_argument("output_directory")
    vertical_id = commands.add_parser("identify-vertical-dynamics")
    vertical_id.add_argument("p_only_run"); vertical_id.add_argument("pd_run"); vertical_id.add_argument("output_directory")
    run.add_argument("--controller-config", default="calibration/stage5b_offline_config.json")
    run.add_argument("--pitch-excitation-rad",type=float,default=.02)
    run.add_argument("--roll-excitation-rad",type=float,default=.02)
    return root


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.command == "analyze":
        print(json.dumps(analyze(args.run_directory), indent=2)); return 0
    if args.command == "analyze-equilibrium":
        print(json.dumps(analyze_equilibrium(args.run_directory), indent=2)); return 0
    if args.command == "export":
        print(json.dumps(export_validated(args.run_directory, args.destination), indent=2)); return 0
    if args.command == "verify-control":
        report = generate_motor_free_report(args.run_directory, args.config)
        print(json.dumps(report, indent=2))
        return 0 if report["overall_offline_result"] == "PASS" else 1
    if args.command == "replay-pid":
        config = load_config(args.config)
        settings = settings_from_config(
            config, equilibrium_thrust=args.equilibrium_thrust,
            kp=args.kp, ki=args.ki, kd=args.kd,
            correction_min=args.correction_min, correction_max=args.correction_max,
            min_thrust=args.min_thrust, max_thrust=args.max_thrust)
        print(json.dumps(replay_official_hover(
            args.run_directory, args.output_directory, settings), indent=2))
        return 0
    if args.command == "replay-cascaded":
        config = load_cascaded_config(args.controller_config)
        print(json.dumps(replay_cascaded_official_hover(
            args.run_directory, args.output_directory, config), indent=2))
        return 0
    if args.command == "analyze-cascaded-flight":
        print(json.dumps(analyze_cascaded_flight(args.run_directory), indent=2)); return 0
    if args.command == "analyze-pitch-response":
        report = analyze_pitch_response(args.run_directory)
        print(json.dumps(report, indent=2)); return 0 if report["result"] == "PASS" else 1
    if args.command == "analyze-roll-response":
        report = analyze_roll_response(args.run_directory)
        print(json.dumps(report, indent=2)); return 0 if report["result"] == "PASS" else 1
    if args.command == "analyze-altitude-pd":
        print(json.dumps(analyze_altitude_pd(args.run_directory,args.output_directory),indent=2)); return 0
    if args.command == "identify-vertical-dynamics":
        print(json.dumps(identify_vertical_dynamics(args.p_only_run,args.pd_run,args.output_directory),indent=2)); return 0
    if args.command == "replay-cascaded-flight":
        config = load_cascaded_config(args.controller_config)
        print(json.dumps(replay_cascaded_flight(
            args.run_directory, args.output_directory, config), indent=2)); return 0
    mode = Mode(args.mode)
    if mode.is_flight and args.confirm_flight != FLIGHT_CONFIRMATION:
        raise SystemExit(f"Flight refused. Pass --confirm-flight {FLIGHT_CONFIRMATION} after completing the safety checklist.")
    config = load_config(args.config)
    if mode is Mode.P_ONLY:
        config.pid = replace(config.pid, ki=0.0, kd=0.0)
    if mode is Mode.STATE_MONITOR:
        settings = StateMonitorSettings()
        logger = StateMonitorLogger(config.output_directory, config, settings)
        # State-monitor deliberately uses onboard z/vz even if a flight config
        # contains an external UDP port.
        hardware = CrazyflieHardware(config.uri, external_altitude_port=None)
        try:
            hardware.connect()
            metrics = StateMonitorRunner(hardware, config, logger, settings).run()
            print(json.dumps(metrics, indent=2))
            print(f"Monitor artifact: {logger.directory}")
            return 0
        except BaseException as exc:
            logger.close("invalid", f"{type(exc).__name__}: {exc}")
            raise
        finally:
            hardware.close()
    if mode is Mode.ATTITUDE_SIGN_MONITOR:
        settings = StateMonitorSettings(frequency_hz=50.0, duration_s=30.0)
        logger = StateMonitorLogger(config.output_directory, config, settings, mode=mode.value)
        hardware = CrazyflieHardware(config.uri, external_altitude_port=None)
        try:
            hardware.connect()
            report = GuidedAttitudeRunner(hardware, config, logger, settings).run()
            print(json.dumps(report, indent=2))
            print(f"Guided monitor artifact: {logger.directory}")
            return 0 if report["overall_result"] == "PASS" else 1
        except BaseException as exc:
            logger.close("invalid", f"{type(exc).__name__}: {exc}")
            raise
        finally:
            hardware.close()
    if mode is Mode.OFFICIAL_HOVER_BASELINE:
        settings = OfficialHoverSettings()
        logger = OfficialHoverLogger(config.output_directory, config, settings)
        hardware = CrazyflieHardware(config.uri, external_altitude_port=None)
        try:
            hardware.connect()
            record_battery_snapshot(hardware, logger)
            metrics = OfficialHoverRunner(hardware, config, logger, settings).run()
            print(json.dumps(metrics, indent=2))
            print(f"Official hover artifact: {logger.directory}")
            return 0 if metrics["performance_pass"] else 1
        except BaseException as exc:
            logger.close("invalid", f"{type(exc).__name__}: {exc}", "not_started_or_unconfirmed")
            raise
        finally:
            hardware.close()
            print_run_summary(logger)
    if mode in (Mode.CASCADED_5B, Mode.CASCADED_5C, Mode.CASCADED_XY,
                Mode.PITCH_RESPONSE_ID, Mode.ROLL_RESPONSE_ID):
        controller_config = load_cascaded_config(args.controller_config)
        _validate_cascaded_mode_config(mode,controller_config)
        identification=(PitchIdentificationSettings(excitation_rad=args.pitch_excitation_rad)
                        if mode is Mode.PITCH_RESPONSE_ID else None)
        roll_identification=(RollIdentificationSettings(excitation_rad=args.roll_excitation_rad)
                             if mode is Mode.ROLL_RESPONSE_ID else None)
        logger = CascadedFlightLogger(config.output_directory, config, controller_config,
                                      experiment_mode=mode.value)
        hardware = CrazyflieHardware(config.uri, external_altitude_port=None)
        runner_started = False
        try:
            hardware.connect()
            record_battery_snapshot(hardware, logger)
            runner_started = True
            CascadedStage5BRunner(hardware, config, controller_config, logger,
                                  pitch_identification=identification,
                                  roll_identification=roll_identification).run()
            metrics = analyze_cascaded_flight(logger.directory)
            print(json.dumps(metrics, indent=2)); print(f"Cascaded flight artifact: {logger.directory}")
            return 0 if metrics["software_execution_status"] == "complete" else 1
        except BaseException as exc:
            if not runner_started:
                # Attempt shutdown before diagnostic disk I/O. During radio loss
                # these calls are best-effort and cannot guarantee motor stop.
                hardware.close()
                logger.diagnostic_event("connection_or_startup_failure", {
                    "type": type(exc).__name__, "message": str(exc)})
                logger.close("aborted", f"startup {type(exc).__name__}: {exc}", {
                    "hardware_close_attempted": True,
                    "motor_shutdown_guaranteed": False,
                })
            raise
        finally:
            hardware.close()
            print_run_summary(logger)
    if mode.is_flight and not config.custom_attitude_hardware_validated:
        raise SystemExit(
            "Custom flight refused: custom_attitude_hardware_validated is false. "
            "Motor-free sign verification is insufficient; record a reviewed powered "
            "attitude-stabilization validation in the per-run config first.")
    logger = ExperimentLogger(config.output_directory, mode.value, config)
    hardware = CrazyflieHardware(config.uri, config.external_altitude_udp_port)
    try:
        hardware.connect()
        record_battery_snapshot(hardware, logger)
        CalibrationRunner(hardware, config, mode, logger).run()
        print(f"Experiment artifact: {logger.directory}")
        return 0
    except BaseException as exc:
        logger.close("invalid", f"{type(exc).__name__}: {exc}")
        raise
    finally:
        hardware.close()
        print_run_summary(logger)
