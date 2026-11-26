"""Offline plots and per-run statistics for predictive-controller flights."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def analyze(directory: Path) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with (directory / "flight.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    with (directory / "events.csv").open(newline="") as stream:
        events = list(csv.DictReader(stream))
    diagnostics = [json.loads(r["controller_diagnostics_json"]) for r in rows]
    track = np.asarray([bool(r["command_thrust_normalized"]) for r in rows])
    if not np.any(track):
        raise ValueError(f"no body-rate tracking steps in {directory}")
    t = np.asarray([float(r["elapsed_s"]) for r in rows])
    axis = "xyz"
    actual = np.asarray([[float(r[f"estimated_position_{a}_m"]) for a in axis] for r in rows])
    target = np.asarray([[float(r[f"reference_position_{a}_m"]) for a in axis] for r in rows])
    command_keys = ["command_roll_rate_rad_s", "command_pitch_rate_rad_s",
                    "command_yaw_rate_rad_s", "command_thrust_normalized"]
    commands = np.array([[float(r[k]) if r[k] else np.nan for k in command_keys] for r in rows])
    drive = [d.get("mpc", {}) for d, sent in zip(diagnostics, track) if sent]
    solve_ms = np.asarray([1000 * d.get("qp_elapsed_s", np.nan) for d in drive])
    loop_ms = np.asarray([1000 * d.get("timing", {}).get("step_work_s", np.nan) for d in diagnostics])
    slack = np.asarray([d.get("max_slack", np.nan) for d in drive])
    error = actual[track] - target[track]
    model = json.loads((directory / "metadata.json").read_text())
    snapshots = directory / "predictions.jsonl"
    errors: dict[int, list] = {}
    if snapshots.exists():
        capture = np.asarray([float(r["estimated_timestamp_s"]) for r in rows])
        with snapshots.open() as stream:
            for line in stream:
                item = json.loads(line)
                step = item["step_index"]
                yaw = item["yaw"]
                c, s = np.cos(yaw), np.sin(yaw)
                transform = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
                predicted = np.asarray(item["state"])[:, :3] @ transform + item["origin"]
                for ahead in range(1, len(predicted)):
                    when = capture[step] + ahead * 0.02
                    match = int(np.searchsorted(capture, when))
                    if match >= len(rows) or not track[match] or not np.all(track[step:match + 1]):
                        break
                    if capture[match] - when > 0.03:
                        continue
                    measured = [np.interp(when, capture, actual[:, j]) for j in range(3)]
                    errors.setdefault(ahead, []).append(predicted[ahead] - measured)
    prediction_rmse = {j: np.sqrt(np.mean(np.square(v), axis=0)).tolist() for j, v in errors.items()}

    fig, axes = plt.subplots(5, 1, figsize=(11, 13))
    for j, a in enumerate(axis):
        trace = axes[0].plot(t[track], actual[track, j], label=f"{a} actual")[0]
        axes[0].plot(t[track], target[track, j], "--", color=trace.get_color(), label=f"{a} reference")
    axes[0].set(ylabel="Tracking [m]")
    axes[0].legend(ncol=3)
    for j, label in enumerate(("roll", "pitch", "yaw", "thrust")):
        axes[1].plot(t, commands[:, j], label=label)
    axes[1].set(ylabel="Rate [rad/s] / thrust [norm.]")
    axes[1].legend()
    axes[2].plot(t[track], solve_ms, label="QP")
    axes[2].plot(t, loop_ms, label="control and send")
    axes[2].set(ylabel="Elapsed [ms]")
    axes[2].legend()
    axes[3].plot(t[track], slack)
    axes[3].set(ylabel="Maximum predicted slack", xlabel="Flight time [s]")
    if errors:
        for j, a in enumerate(axis):
            axes[4].plot(list(prediction_rmse), [v[j] for v in prediction_rmse.values()], label=a)
        axes[4].legend()
    axes[4].set(ylabel="Online position RMSE [m]", xlabel="Prediction step")
    fig.tight_layout()
    fig.savefig(directory / "predictive-performance.png", dpi=130)
    plt.close(fig)
    finite_solve = solve_ms[np.isfinite(solve_ms)]
    finite_loop = loop_ms[np.isfinite(loop_ms)]
    return {
        "run": str(directory), "run_id": model["run_id"],
        "tracking_steps": int(np.count_nonzero(track)),
        "tracking_rmse_m": np.sqrt(np.mean(error**2, axis=0)).tolist(),
        "tracking_absolute_p95_m": np.percentile(np.abs(error), 95, axis=0).tolist(),
        "tracking_peak_absolute_m": np.max(np.abs(error), axis=0).tolist(),
        "predicted_soft_activation_steps": sum(d.get("soft_boundary_count", 0) > 0 for d in drive),
        "predicted_input_activation_steps": sum(d.get("active_input_count", 0) > 0 for d in drive),
        "max_predicted_slack": float(np.nanmax(slack)) if np.any(np.isfinite(slack)) else None,
        "timing_percentiles": [50, 95, 99, 100],
        "solve_ms": np.percentile(finite_solve, [50, 95, 99, 100]).tolist() if finite_solve.size else None,
        "control_send_ms": np.percentile(finite_loop, [50, 95, 99, 100]).tolist() if finite_loop.size else None,
        "deadline_misses": sum(e["event"] == "control_deadline_missed" for e in events),
        "fallback_reasons": sorted({d["fallback_reason"] for d in diagnostics if d.get("fallback_reason")}),
        "prediction_snapshots_dropped": model.get("prediction_snapshots_dropped"),
        "online_prediction_position_rmse_m": prediction_rmse,
        "online_prediction_samples": {j: len(v) for j, v in errors.items()},
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+")
    args = parser.parse_args()
    for run in args.runs:
        result = json.dumps(analyze(run), indent=2)
        (run / "predictive-performance.json").write_text(result + "\n", encoding="utf-8")
        print(result)


if __name__ == "__main__":
    main()
