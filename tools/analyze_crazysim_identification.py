"""Analyze independently restarted near-hover CrazySim pulse runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from velocity_control_framework.controllers.near_hover_model import (
    NearHoverModelConfig,
    discrete_model,
    world_to_fixed_yaw,
)


def load_run(path: Path) -> tuple[dict, list[dict]]:
    run = json.loads(path.read_text(encoding="utf-8"))
    if not run["completed"] or run["failure"] or run["cleanup_errors"]:
        raise ValueError(f"unsuccessful identification run: {path}")
    rows = [row for row in run["records"] if row["phase"] == "BODY_RATE"]
    if len(rows) < 2 or any("input" not in row for row in rows):
        raise ValueError(f"missing timestamped identification samples: {path}")
    return run, rows


def summarize(path: Path) -> dict:
    run, rows = load_run(path)
    send = np.asarray([row["send_completed_monotonic_s"] for row in rows])
    issued = np.asarray([row["input"] for row in rows])
    angles = np.asarray([row["attitude_rad"] for row in rows])
    rates = np.asarray([row["body_rate_rad_s"] for row in rows])
    velocities = np.asarray([row["velocity_m_s"] for row in rows])
    return {
        "run": str(path),
        "axis": run["axis"],
        "direction": run["direction"],
        "samples": len(rows),
        "max_period_ms": float(np.max(np.diff(send)) * 1000),
        "peak_abs_body_rate_rad_s": np.max(np.abs(rates), axis=0).tolist(),
        "attitude_delta_rad": (angles[-1] - angles[0]).tolist(),
        "velocity_delta_m_s": (velocities[-1] - velocities[0]).tolist(),
        "input_min": np.min(issued, axis=0).tolist(),
        "input_max": np.max(issued, axis=0).tolist(),
    }


def validate_model(paths: list[Path]) -> dict:
    config = NearHoverModelConfig()
    ad, bd = discrete_model(config)
    horizons = (15, 25, 40)
    errors = {n: [] for n in horizons}
    persistence = {n: [] for n in horizons}
    for path in paths:
        _, rows = load_run(path)
        observed_at = np.asarray([r["state_timestamp_s"] for r in rows])
        sent_at = np.asarray([r["send_completed_monotonic_s"] for r in rows])
        pos = np.asarray([r["position_m"] for r in rows])
        vel = np.asarray([r["velocity_m_s"] for r in rows])
        angles = np.asarray([r["attitude_rad"] for r in rows])
        rotation = world_to_fixed_yaw(float(angles[0, 2]))
        angles[:, 2] = np.unwrap(angles[:, 2]) - angles[0, 2]
        states = np.column_stack(((pos - pos[0]) @ rotation.T, vel @ rotation.T, angles))
        commands = np.asarray([r["input"] for r in rows])
        commands[:, 3] -= config.equilibrium_thrust
        for n in horizons:
            for origin, command_time in enumerate(sent_at):
                grid = command_time + config.sample_period_s * np.arange(n + 1)
                if grid[-1] > observed_at[-1]:
                    break
                forecast = states[origin].copy()
                for tick in grid[:-1]:
                    index = np.searchsorted(sent_at, tick, side="right") - 1
                    forecast = ad @ forecast + bd @ commands[index]
                actual = np.asarray([
                    np.interp(grid[-1], observed_at, states[:, column])
                    for column in range(states.shape[1])
                ])
                errors[n].append(forecast - actual)
                persistence[n].append(states[origin] - actual)
    return {
        str(n): {
            "samples": len(errors[n]),
            "model_rmse": np.sqrt(np.mean(np.square(errors[n]), axis=0)).tolist() if errors[n] else None,
            "persistence_rmse": np.sqrt(np.mean(np.square(persistence[n]), axis=0)).tolist() if errors[n] else None,
        } for n in horizons
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="*")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--predict", action="store_true")
    mode.add_argument("--benchmark", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not args.runs and not args.benchmark:
        parser.error("provide identification runs or --benchmark")
    if args.benchmark:
        result = benchmark()
    elif args.predict:
        result = validate_model(args.runs)
    else:
        result = [summarize(path) for path in args.runs]
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output is not None:
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")


def benchmark() -> dict:
    from velocity_control_framework.controllers.outer_loop_mpc import (
        MPCSolveError, OuterLoopMPCConfig, OuterLoopMPCController,
    )
    from velocity_control_framework.interfaces import DroneState, Reference

    summaries = {}
    for horizon in (15, 25, 40):
        controller = OuterLoopMPCController(OuterLoopMPCConfig(horizon=horizon))
        durations = []
        failures = 0
        for sample in range(100):
            if sample % 20 == 0:
                controller.reset()
            state = DroneState.zero()
            target = Reference.hover(0.03, -0.03, 0.04 if sample < 50 else 0.3)
            start = perf_counter()
            try:
                controller.update(state, target, dt=0.02)
            except MPCSolveError:
                failures += 1
            durations.append(1000 * (perf_counter() - start))
        summaries[str(horizon)] = {
            "seconds": horizon * 0.02,
            "calls": len(durations),
            "failures": failures,
            "p50_p95_p99_max_ms": np.percentile(durations, [50, 95, 99, 100]).tolist(),
        }
    return summaries


if __name__ == "__main__":
    main()
