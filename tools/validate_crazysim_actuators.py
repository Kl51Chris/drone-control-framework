"""Explicitly opt-in CrazySim body-rate/thrust actuator diagnostic."""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from experiments.run_flight import request_supervisor_arming
from tools.crazysim_session import PassiveCrazyflie
from tools.diagnose_crazysim_backend import (
    CRAZYSIM_URI,
    supervisor_status,
    validate_crazysim_uri,
)
from velocity_control_framework.backends import (
    CrazyflieStateProvider,
    CrazySimCommandAdapter,
    CrazySimSupervisorAdapter,
)
from velocity_control_framework.interfaces import BodyRateThrustCommand


MAX_DURATION_S = 0.5
MAX_RATE_RAD_S = 0.25
MAX_NORMALIZED_THRUST = 0.20
CONTROL_PERIOD_S = 0.02
MAX_STATE_AGE_S = 0.10
MOTOR_LOG_VARIABLES = (
    "motor.m1", "motor.m2", "motor.m3", "motor.m4", "stabilizer.thrust",
    "ctrltarget.roll", "ctrltarget.pitch", "ctrltarget.yaw",
)
INACTIVE_EPSILON = 1e-6


def build_command(
    axis: str,
    rate_rad_s: float,
    normalized_thrust: float,
) -> BodyRateThrustCommand:
    if axis not in ("roll", "pitch", "yaw", "none"):
        raise ValueError("axis must be roll, pitch, yaw, or none")
    if not math.isfinite(rate_rad_s) or abs(rate_rad_s) > MAX_RATE_RAD_S:
        raise ValueError(f"rate magnitude must not exceed {MAX_RATE_RAD_S} rad/s")
    if axis == "none" and rate_rad_s != 0.0:
        raise ValueError("axis=none requires zero rate")
    if not math.isfinite(normalized_thrust) or not (
        0.0 <= normalized_thrust <= MAX_NORMALIZED_THRUST
    ):
        raise ValueError(
            f"normalized thrust must be within [0, {MAX_NORMALIZED_THRUST}]"
        )
    rates = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    if axis != "none":
        rates[axis] = rate_rad_s
    return BodyRateThrustCommand(
        rates["roll"], rates["pitch"], rates["yaw"], normalized_thrust
    )


def validate_duration(duration_s: float) -> None:
    if not math.isfinite(duration_s) or not (0.0 < duration_s <= MAX_DURATION_S):
        raise ValueError(f"duration must be within (0, {MAX_DURATION_S}] seconds")


def validate_safe_startup(
    status: dict[str, Any], outputs: dict[str, float]
) -> str:
    """Classify a safe disarmed or CrazySim-default auto-armed startup."""
    faults = tuple(
        name for name in ("is_flying", "is_tumbled", "is_locked", "is_crashed")
        if status[name]
    )
    if faults or status["hl_control_active"]:
        raise RuntimeError(f"unsafe initial supervisor state: {status}; faults={faults}")

    nonzero = {
        name: value for name, value in outputs.items()
        if not math.isfinite(value) or abs(value) > INACTIVE_EPSILON
    }
    if nonzero:
        raise RuntimeError(f"startup actuator/controller output is not inactive: {nonzero}")

    if not status["is_armed"]:
        if status["is_auto_armed"] or status["can_fly"] or not status["can_be_armed"]:
            raise RuntimeError(f"unexpected disarmed supervisor state: {status}")
        return "disarmed"

    if (
        status["is_auto_armed"]
        and status["can_fly"]
        and not status["can_be_armed"]
    ):
        return "safe_auto_armed"
    raise RuntimeError(f"unexpected armed supervisor state: {status}")


def validate_authorized_command_phase(status: dict[str, Any]) -> None:
    """Validate supervisor state after the bounded command has been authorized.

    Firmware ``is_flying`` is a motor-above-idle state with hysteresis, not an
    observation that the MuJoCo body left the ground, so either value is valid
    in this phase.
    """
    faults = tuple(
        name for name in ("is_tumbled", "is_locked", "is_crashed")
        if status[name]
    )
    if (
        not status["is_armed"]
        or not status["can_fly"]
        or faults
        or status["hl_control_active"]
    ):
        raise RuntimeError(
            f"unexpected supervisor state during authorized command: "
            f"{status}; faults={faults}"
        )


def safe_cleanup(
    cf: Any,
    adapter: Any | None,
    supervisor: Any | None = None,
    *,
    disarm: bool = True,
) -> list[str]:
    """Best-effort STOP and, only when applicable, explicit disarm."""
    errors = []
    try:
        if adapter is not None:
            adapter.stop()
        else:
            cf.commander.send_stop_setpoint()
    except BaseException as error:
        errors.append(f"stop: {type(error).__name__}: {error}")
    if disarm:
        try:
            if supervisor is None:
                cf.supervisor.send_arming_request(False)
            else:
                supervisor.send_arming_request(False)
        except BaseException as error:
            errors.append(f"disarm: {type(error).__name__}: {error}")
    return errors


class MotorMonitor:
    def __init__(self, cf: Any) -> None:
        from cflib.crazyflie.log import LogConfig

        self._lock = threading.Lock()
        self._latest: dict[str, float] | None = None
        self._config = LogConfig(name="CrazySimMotors", period_in_ms=10)
        for name in MOTOR_LOG_VARIABLES[:4]:
            self._config.add_variable(name, "uint16_t")
        for name in MOTOR_LOG_VARIABLES[4:]:
            self._config.add_variable(name, "float")
        cf.log.add_config(self._config)
        self._config.data_received_cb.add_callback(self._on_data)

    def _on_data(self, _timestamp, data, _config) -> None:
        with self._lock:
            self._latest = {name: float(data[name]) for name in MOTOR_LOG_VARIABLES}

    def start(self) -> None:
        self._config.start()

    def stop(self) -> None:
        self._config.stop()
        self._config.delete()

    def latest(self) -> dict[str, float]:
        with self._lock:
            if self._latest is None:
                raise RuntimeError("no motor telemetry received")
            return dict(self._latest)

    def wait_for_first(self, timeout_s: float = 1.0) -> dict[str, float]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                return self.latest()
            except RuntimeError:
                time.sleep(0.01)
        raise RuntimeError("timed out waiting for motor telemetry")


def run_powered_test(
    uri: str,
    command: BodyRateThrustCommand,
    duration_s: float,
) -> dict[str, Any]:
    validate_crazysim_uri(uri)
    validate_duration(duration_s)
    import cflib.crtp
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    records = []
    cleanup_errors: list[str] = []
    failure: str | None = None
    final_supervisor: dict[str, Any] | None = None
    final_outputs: dict[str, float] | None = None
    startup_lifecycle: str | None = None
    cf = PassiveCrazyflie(rw_cache="./cache")
    with SyncCrazyflie(uri, cf=cf) as link:
        adapter = None
        supervisor = CrazySimSupervisorAdapter(link.cf)
        provider = CrazyflieStateProvider(link.cf, delete_log_blocks_on_stop=True)
        motors = MotorMonitor(link.cf)
        try:
            supervisor.start()
            provider.start()
            motors.start()
            initial = supervisor.status()
            startup_lifecycle = validate_safe_startup(
                initial, motors.wait_for_first()
            )
            # Rate modes are configured and read back only after the supervisor,
            # motors, thrust, and controller targets have passed validation.
            adapter = CrazySimCommandAdapter(link.cf)
            if startup_lifecycle == "disarmed":
                request_supervisor_arming(
                    link.cf,
                    status_reader=supervisor.status,
                    arming_request=supervisor.send_arming_request,
                )
            adapter.unlock()
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                now = time.monotonic()
                state = provider.get_state()
                if now - state.timestamp > MAX_STATE_AGE_S:
                    raise RuntimeError("telemetry became stale during powered diagnostic")
                status = supervisor.status()
                validate_authorized_command_phase(status)
                adapter.send(command)
                records.append({
                    "monotonic_s": now,
                    "command": {
                        "roll_rate_rad_s": command.roll_rate,
                        "pitch_rate_rad_s": command.pitch_rate,
                        "yaw_rate_rad_s": command.yaw_rate,
                        "normalized_thrust": command.thrust,
                        "raw_thrust": adapter._encode_thrust(command.thrust),
                    },
                    "state": {"p": state.p, "q": state.q, "r": state.r},
                    "motors": motors.latest(),
                    "supervisor": status,
                })
                time.sleep(CONTROL_PERIOD_S)
        except BaseException as error:
            failure = f"{type(error).__name__}: {error}"
        finally:
            cleanup_errors = safe_cleanup(
                link.cf,
                adapter,
                supervisor,
                disarm=(startup_lifecycle == "disarmed"),
            )
            try:
                # STOP must produce an inactive motor/control observation. The
                # default auto-armed state itself intentionally remains armed.
                deadline = time.monotonic() + 0.25
                while True:
                    final_outputs = motors.latest()
                    if all(
                        math.isfinite(value) and abs(value) <= INACTIVE_EPSILON
                        for value in final_outputs.values()
                    ):
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError(
                            f"outputs remained active after STOP: {final_outputs}"
                        )
                    time.sleep(0.01)
            except BaseException as error:
                cleanup_errors.append(f"inactive_after_stop: {error}")
            try:
                motors.stop()
            except BaseException as error:
                cleanup_errors.append(f"motor_log_stop: {error}")
            provider.stop()
            try:
                final_supervisor = supervisor.status()
            except BaseException as error:
                cleanup_errors.append(f"final_supervisor: {error}")
            try:
                supervisor.stop()
            except BaseException as error:
                cleanup_errors.append(f"supervisor_log_stop: {error}")
    return {
        "uri": uri,
        "records": records,
        "failure": failure,
        "cleanup_errors": cleanup_errors,
        "final_supervisor": final_supervisor,
        "final_outputs_after_stop": final_outputs,
        "startup_lifecycle": startup_lifecycle,
    }


def write_result(result: dict[str, Any]) -> Path:
    root = Path(__file__).resolve().parents[1] / "log" / "crazysim-diagnostics"
    root.mkdir(parents=True, exist_ok=True)
    path = root / f"{datetime.now().astimezone():%Y%m%d_%H%M%S}_actuator.json"
    path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=CRAZYSIM_URI)
    parser.add_argument("--axis", choices=("roll", "pitch", "yaw", "none"), required=True)
    parser.add_argument("--rate-rad-s", type=float, default=0.0)
    parser.add_argument("--normalized-thrust", type=float, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--execute-powered-test", action="store_true")
    parser.add_argument("--i-understand-motors-may-spin", action="store_true")
    args = parser.parse_args()
    command = build_command(args.axis, args.rate_rad_s, args.normalized_thrust)
    validate_duration(args.duration)
    if not (args.execute_powered_test and args.i_understand_motors_may_spin):
        raise SystemExit(
            "refusing powered test: both --execute-powered-test and "
            "--i-understand-motors-may-spin are required"
        )
    result = run_powered_test(args.uri, command, args.duration)
    path = write_result(result)
    print(json.dumps(result, indent=2, sort_keys=True))
    print(f"[LOG] {path}")
    if result["failure"] or result["cleanup_errors"]:
        raise SystemExit("powered diagnostic failed or cleanup was incomplete")


if __name__ == "__main__":
    main()
