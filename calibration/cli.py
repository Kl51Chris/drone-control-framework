from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path

from .analysis import analyze, export_validated
from .config import load_config
from .experiment import CalibrationRunner, Mode
from .hardware import CrazyflieHardware
from .logger import ExperimentLogger


FLIGHT_CONFIRMATION = "I_UNDERSTAND_PROPELLERS_MAY_SPIN"


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description="Independent Crazyflie altitude calibration toolkit")
    commands = root.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("mode", choices=[mode.value for mode in Mode])
    run.add_argument("--config", default="calibration/default_config.json")
    run.add_argument("--confirm-flight", default="", help=f"required literal: {FLIGHT_CONFIRMATION}")
    report = commands.add_parser("analyze"); report.add_argument("run_directory")
    export = commands.add_parser("export"); export.add_argument("run_directory"); export.add_argument("destination")
    return root


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.command == "analyze":
        print(json.dumps(analyze(args.run_directory), indent=2)); return 0
    if args.command == "export":
        print(json.dumps(export_validated(args.run_directory, args.destination), indent=2)); return 0
    mode = Mode(args.mode)
    if mode.is_flight and args.confirm_flight != FLIGHT_CONFIRMATION:
        raise SystemExit(f"Flight refused. Pass --confirm-flight {FLIGHT_CONFIRMATION} after completing the safety checklist.")
    config = load_config(args.config)
    if mode is Mode.P_ONLY:
        config.pid = replace(config.pid, ki=0.0, kd=0.0)
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
