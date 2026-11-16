"""Inspect CrazySim supervisor and commander state without initiating flight."""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable
from typing import Any

from tools.crazysim_session import PassiveCrazyflie
from velocity_control_framework.backends import (
    CrazySimCommandAdapter,
    CrazySimSupervisorAdapter,
)


CRAZYSIM_URI = "udp://127.0.0.1:19850"
COMMAND_LOG_VARIABLES = (
    "ctrltarget.roll",
    "ctrltarget.pitch",
    "ctrltarget.yaw",
    "stabilizer.thrust",
)
MOTOR_LOG_VARIABLES = ("motor.m1", "motor.m2", "motor.m3", "motor.m4")


def validate_crazysim_uri(uri: str) -> None:
    if uri != CRAZYSIM_URI:
        raise ValueError(
            f"diagnostic is restricted to the CrazySim endpoint {CRAZYSIM_URI}"
        )


def supervisor_status(cf: Any) -> dict[str, bool | int]:
    supervisor = cf.supervisor
    bitfield = int(supervisor.read_bitfield())
    return {
        "bitfield": bitfield,
        "can_be_armed": bool(supervisor.can_be_armed),
        "can_fly": bool(supervisor.can_fly),
        "is_armed": bool(supervisor.is_armed),
        "is_flying": bool(supervisor.is_flying),
        "is_tumbled": bool(supervisor.is_tumbled),
        "is_locked": bool(supervisor.is_locked),
        "is_crashed": bool(supervisor.is_crashed),
        "hl_control_active": bool(supervisor.hl_control_active),
    }


def describe_supervisor_status(
    status: dict[str, bool | int],
    sample: dict[str, float | bool] | None = None,
) -> dict[str, Any]:
    bitfield = int(status["bitfield"])
    flags = {
        name: bool(status[name])
        for name in (
            "can_be_armed",
            "is_armed",
            "is_auto_armed",
            "can_fly",
            "is_flying",
            "is_tumbled",
            "is_locked",
            "is_crashed",
            "hl_control_active",
        )
        if name in status
    }
    rejection_flags = [name for name in ("is_armed", "is_flying") if flags[name]]
    return {
        "raw_bitfield_decimal": bitfield,
        "raw_bitfield_hex": f"0x{bitfield:04X}",
        "decoded_flags": flags,
        "command_safety_rejection_flags": rejection_flags,
        "sample": sample,
    }


def inspect_connected(
    cf: Any,
    command_reader: Callable[[Any], dict[str, float]],
    *,
    configure_rate_modes: bool = False,
    verify_stop: bool = False,
    status_reader: Callable[[], dict[str, bool | int]] | None = None,
    sample_reader: Callable[[], dict[str, float | bool]] | None = None,
) -> dict[str, Any]:
    """Inspect one connected SITL instance; no arming or nonzero thrust."""
    if verify_stop and not configure_rate_modes:
        raise ValueError("--verify-stop requires --configure-rate-modes")

    read_status = status_reader or (lambda: supervisor_status(cf))
    before = read_status()
    before_report = describe_supervisor_status(
        before, sample_reader() if sample_reader is not None else None
    )
    rejection_flags = before_report["command_safety_rejection_flags"]
    mutation_requested = configure_rate_modes or verify_stop

    # Reading parameters and log variables is safe while armed. Configuration
    # and STOP remain refused, but the caller receives the raw state first.
    operation_refused = None
    if mutation_requested and rejection_flags:
        operation_refused = {
            "reason": "firmware is armed or flying",
            "triggered_by": rejection_flags,
            "requested_operations": {
                "configure_rate_modes": configure_rate_modes,
                "verify_stop": verify_stop,
            },
        }

    modes = {
        CrazySimCommandAdapter.ROLL_MODE_PARAMETER: int(
            cf.param.get_value(CrazySimCommandAdapter.ROLL_MODE_PARAMETER)
        ),
        CrazySimCommandAdapter.PITCH_MODE_PARAMETER: int(
            cf.param.get_value(CrazySimCommandAdapter.PITCH_MODE_PARAMETER)
        ),
        CrazySimCommandAdapter.YAW_MODE_PARAMETER: int(
            cf.param.get_value(CrazySimCommandAdapter.YAW_MODE_PARAMETER)
        ),
    }
    adapter = None
    if configure_rate_modes and operation_refused is None:
        adapter = CrazySimCommandAdapter(cf)
        modes = {
            name: int(cf.param.get_value(name))
            for name in (
                CrazySimCommandAdapter.ROLL_MODE_PARAMETER,
                CrazySimCommandAdapter.PITCH_MODE_PARAMETER,
                CrazySimCommandAdapter.YAW_MODE_PARAMETER,
            )
        }
    if verify_stop and operation_refused is None:
        assert adapter is not None
        adapter.stop()

    command_state = command_reader(cf)
    after = read_status()
    return {
        "supervisor_before": before_report,
        "rate_modes": modes,
        "commander_state": command_state,
        "stop_setpoint_sent": verify_stop and operation_refused is None,
        "supervisor_after": describe_supervisor_status(
            after, sample_reader() if sample_reader is not None else None
        ),
        "operation_refused": operation_refused,
    }


def read_command_state(cf: Any) -> dict[str, float]:
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncLogger import SyncLogger

    config = LogConfig(name="CrazySimDiagnostic", period_in_ms=100)
    for name in COMMAND_LOG_VARIABLES:
        config.add_variable(name, "float")
    for name in MOTOR_LOG_VARIABLES:
        config.add_variable(name, "uint16_t")
    with SyncLogger(cf, config) as logger:
        _, data, _ = next(logger)
    return {
        name: float(data[name])
        for name in (*COMMAND_LOG_VARIABLES, *MOTOR_LOG_VARIABLES)
    }


def run(
    uri: str,
    *,
    configure_rate_modes: bool = False,
    verify_stop: bool = False,
) -> dict[str, Any]:
    validate_crazysim_uri(uri)
    import cflib.crtp
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    crazyflie = PassiveCrazyflie(rw_cache="./cache")
    with SyncCrazyflie(uri, cf=crazyflie) as link:
        supervisor = CrazySimSupervisorAdapter(link.cf)
        try:
            supervisor.start()
            result = inspect_connected(
                link.cf,
                read_command_state,
                configure_rate_modes=configure_rate_modes,
                verify_stop=verify_stop,
                status_reader=supervisor.status,
                sample_reader=lambda: supervisor.status_with_sample()[1],
            )
            result["supervisor_source"] = "firmware log supervisor.info"
            result["crtp_protocol_version"] = int(
                link.cf.platform.get_protocol_version()
            )
            return result
        finally:
            supervisor.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=CRAZYSIM_URI)
    parser.add_argument(
        "--configure-rate-modes",
        action="store_true",
        help="OPT-IN: write and verify roll/pitch rate-mode parameters",
    )
    parser.add_argument(
        "--verify-stop",
        action="store_true",
        help="OPT-IN: transmit one generic STOP packet; requires rate-mode opt-in",
    )
    args = parser.parse_args()
    try:
        result = run(
            args.uri,
            configure_rate_modes=args.configure_rate_modes,
            verify_stop=args.verify_stop,
        )
    except BaseException as error:
        raise SystemExit(f"CrazySim backend diagnostic failed: {error}") from error
    print(json.dumps(result, indent=2, sort_keys=True))
    if result["operation_refused"] is not None:
        raise SystemExit("requested command/configuration operation was refused")


if __name__ == "__main__":
    main()
