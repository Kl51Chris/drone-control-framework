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


FLIGHT_CONFIRMATION = "I_UNDERSTAND_PROPELLERS_MAY_SPIN"


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
            metrics = OfficialHoverRunner(hardware, config, logger, settings).run()
            print(json.dumps(metrics, indent=2))
            print(f"Official hover artifact: {logger.directory}")
            return 0 if metrics["performance_pass"] else 1
        except BaseException as exc:
            logger.close("invalid", f"{type(exc).__name__}: {exc}", "not_started_or_unconfirmed")
            raise
        finally:
            hardware.close()
    logger = ExperimentLogger(config.output_directory, mode.value, config)
    hardware = CrazyflieHardware(config.uri, config.external_altitude_udp_port)
    try:
        hardware.connect()
        CalibrationRunner(hardware, config, mode, logger).run()
        print(f"Experiment artifact: {logger.directory}")
        return 0
    except BaseException as exc:
        logger.close("invalid", f"{type(exc).__name__}: {exc}")
        raise
    finally:
        hardware.close()
