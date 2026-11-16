"""Bounded CrazySim-only high-level takeoff, hover, and landing demo."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from tools.crazysim_session import PassiveCrazyflie
from tools.diagnose_crazysim_backend import CRAZYSIM_URI, validate_crazysim_uri
from tools.validate_crazysim_actuators import MotorMonitor, validate_safe_startup
from velocity_control_framework.backends import (
    CrazyflieStateProvider,
    CrazySimSupervisorAdapter,
)


TAKEOFF_HEIGHT_M = 0.35
LANDING_HEIGHT_M = 0.03
TAKEOFF_DURATION_S = 3.0
HOVER_DURATION_S = 2.0
LANDING_DURATION_S = 3.0
SETTLE_DURATION_S = 0.5
MONITOR_PERIOD_S = 0.05
MAX_TOTAL_DURATION_S = 12.0
MAX_STATE_AGE_S = 0.15
MAX_INITIAL_SPEED_M_S = 0.10
MAX_HORIZONTAL_DISPLACEMENT_M = 0.75
MAX_HEIGHT_ABOVE_START_M = 0.80
MAX_SPEED_M_S = 2.0
MAX_TILT_RAD = 0.8


def validate_stationary_start(state: Any) -> None:
    speed = float(np.linalg.norm(state.velocity))
    if not math.isfinite(speed) or speed > MAX_INITIAL_SPEED_M_S:
        raise RuntimeError(f"CrazySim vehicle is not stationary: speed={speed:.3f} m/s")
    if max(abs(float(state.roll)), abs(float(state.pitch))) > 0.15:
        raise RuntimeError("CrazySim vehicle is not level at startup")


def validate_flight_state(state: Any, initial_position: np.ndarray, now: float) -> None:
    age = now - float(state.timestamp)
    if age < -0.01 or age > MAX_STATE_AGE_S:
        raise RuntimeError(f"state telemetry is stale: {age:.3f} s")
    values = (*state.position, *state.velocity, state.roll, state.pitch, state.yaw)
    if not all(math.isfinite(float(value)) for value in values):
        raise RuntimeError("state telemetry contains non-finite values")
    displacement = np.asarray(state.position, dtype=float) - initial_position
    if float(np.linalg.norm(displacement[:2])) > MAX_HORIZONTAL_DISPLACEMENT_M:
        raise RuntimeError("horizontal flight envelope exceeded")
    if not -0.10 <= float(displacement[2]) <= MAX_HEIGHT_ABOVE_START_M:
        raise RuntimeError("vertical flight envelope exceeded")
    if float(np.linalg.norm(state.velocity)) > MAX_SPEED_M_S:
        raise RuntimeError("flight speed envelope exceeded")
    if max(abs(float(state.roll)), abs(float(state.pitch))) > MAX_TILT_RAD:
        raise RuntimeError("flight tilt envelope exceeded")


def validate_flight_supervisor(status: dict[str, Any]) -> None:
    faults = tuple(
        name for name in ("is_tumbled", "is_locked", "is_crashed")
        if status[name]
    )
    if not status["is_armed"] or not status["can_fly"] or faults:
        raise RuntimeError(f"unsafe flight supervisor state: {status}; faults={faults}")


def stop_commands(cf: Any) -> list[str]:
    """Best-effort high-level STOP followed by generic STOP."""
    errors = []
    try:
        cf.high_level_commander.stop()
    except BaseException as error:
        errors.append(f"high_level_stop: {type(error).__name__}: {error}")
    try:
        cf.commander.send_stop_setpoint()
    except BaseException as error:
        errors.append(f"generic_stop: {type(error).__name__}: {error}")
    return errors


def monitor_phase(
    name: str,
    duration_s: float,
    *,
    provider: CrazyflieStateProvider,
    supervisor: CrazySimSupervisorAdapter,
    motors: MotorMonitor,
    initial_position: np.ndarray,
    experiment_deadline: float,
    records: list[dict[str, Any]],
) -> None:
    phase_deadline = time.monotonic() + duration_s
    while time.monotonic() < phase_deadline:
        now = time.monotonic()
        if now >= experiment_deadline:
            raise RuntimeError("flight demo exceeded its total duration limit")
        state = provider.get_state()
        status = supervisor.status()
        validate_flight_state(state, initial_position, now)
        validate_flight_supervisor(status)
        records.append({
            "monotonic_s": now,
            "phase": name,
            "position_m": [float(value) for value in state.position],
            "velocity_m_s": [float(value) for value in state.velocity],
            "attitude_rad": [float(state.roll), float(state.pitch), float(state.yaw)],
            "supervisor": status,
            "outputs": motors.latest(),
        })
        time.sleep(MONITOR_PERIOD_S)


def run_demo(uri: str) -> dict[str, Any]:
    validate_crazysim_uri(uri)
    import cflib.crtp
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    records: list[dict[str, Any]] = []
    cleanup_errors: list[str] = []
    failure: str | None = None
    completed = False
    cf = PassiveCrazyflie(rw_cache="./cache")
    with SyncCrazyflie(uri, cf=cf) as link:
        provider = CrazyflieStateProvider(link.cf, delete_log_blocks_on_stop=True)
        supervisor = CrazySimSupervisorAdapter(link.cf)
        motors = MotorMonitor(link.cf)
        try:
            provider.start()
            supervisor.start()
            motors.start()
            initial_state = provider.get_state()
            validate_stationary_start(initial_state)
            lifecycle = validate_safe_startup(
                supervisor.status(), motors.wait_for_first()
            )
            if lifecycle != "safe_auto_armed":
                raise RuntimeError(
                    f"high-level demo requires CrazySim's safe auto-armed lifecycle, got {lifecycle}"
                )

            initial_position = np.asarray(initial_state.position, dtype=float)
            takeoff_height = float(initial_position[2]) + TAKEOFF_HEIGHT_M
            landing_height = float(initial_position[2]) + LANDING_HEIGHT_M
            experiment_deadline = time.monotonic() + MAX_TOTAL_DURATION_S

            link.cf.high_level_commander.takeoff(
                takeoff_height, TAKEOFF_DURATION_S, yaw=None
            )
            monitor_phase(
                "TAKEOFF", TAKEOFF_DURATION_S + SETTLE_DURATION_S,
                provider=provider, supervisor=supervisor, motors=motors,
                initial_position=initial_position,
                experiment_deadline=experiment_deadline, records=records,
            )
            monitor_phase(
                "HOVER", HOVER_DURATION_S,
                provider=provider, supervisor=supervisor, motors=motors,
                initial_position=initial_position,
                experiment_deadline=experiment_deadline, records=records,
            )
            link.cf.high_level_commander.land(
                landing_height, LANDING_DURATION_S, yaw=None
            )
            monitor_phase(
                "LANDING", LANDING_DURATION_S + SETTLE_DURATION_S,
                provider=provider, supervisor=supervisor, motors=motors,
                initial_position=initial_position,
                experiment_deadline=experiment_deadline, records=records,
            )
            completed = True
        except BaseException as error:
            failure = f"{type(error).__name__}: {error}"
        finally:
            cleanup_errors.extend(stop_commands(link.cf))
            try:
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    outputs = motors.latest()
                    if all(math.isfinite(value) and abs(value) <= 1e-6 for value in outputs.values()):
                        break
                    time.sleep(0.01)
                else:
                    raise RuntimeError(f"outputs remained active after STOP: {outputs}")
            except BaseException as error:
                cleanup_errors.append(f"stop_verification: {error}")
            for label, component in (
                ("motor_log", motors), ("supervisor_log", supervisor), ("state_log", provider)
            ):
                try:
                    component.stop()
                except BaseException as error:
                    cleanup_errors.append(f"{label}_stop: {error}")
    return {
        "uri": uri,
        "completed": completed,
        "failure": failure,
        "cleanup_errors": cleanup_errors,
        "records": records,
    }


def write_result(result: dict[str, Any]) -> Path:
    root = Path(__file__).resolve().parents[2] / "log" / "crazysim-diagnostics"
    root.mkdir(parents=True, exist_ok=True)
    path = root / f"{datetime.now().astimezone():%Y%m%d_%H%M%S}_flight_demo.json"
    path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=CRAZYSIM_URI)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    validate_crazysim_uri(args.uri)
    if not args.execute:
        print(json.dumps({"uri": args.uri, "status": "inspection_only"}, indent=2))
        return
    result = run_demo(args.uri)
    path = write_result(result)
    print(json.dumps({key: value for key, value in result.items() if key != "records"}, indent=2))
    print(f"[LOG] {path}")
    if not result["completed"] or result["failure"] or result["cleanup_errors"]:
        raise SystemExit("CrazySim flight demo failed or cleanup was incomplete")


if __name__ == "__main__":
    main()
