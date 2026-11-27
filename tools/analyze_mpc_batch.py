"""Score independent CrazySim MPC runs and keep failures in the summary."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from velocity_control_framework.controllers.near_hover_model import (
    NearHoverModelConfig, discrete_model, world_to_fixed_yaw,
)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def values(rows, fields):
    return np.array([[float(row[field]) for field in fields] for row in rows])


def describe(run):
    result = json.loads((run / "experiment.json").read_text())
    row = {"run": str(run), **result["case"], "status": result["status"]}
    flight = next(run.glob("*/flight.csv"), None)
    if flight is None or not (run / "sim_truth.csv").is_file():
        return row, None
    truth = read_csv(run / "sim_truth.csv")
    flight_rows = read_csv(flight)
    rows = [r for r in flight_rows if r["command_thrust_normalized"]]
    if len(truth) < 2 or len(rows) < 2:
        return row, None
    if len({r["agent"] for r in truth}) != 1:
        raise ValueError("analysis expects a single simulated vehicle")
    tt = values(truth, ["monotonic_s"])[:, 0]
    t = values(rows, ["estimated_timestamp_s"])[:, 0]
    if np.any(np.diff(tt) <= 0) or t[0] < tt[0] or t[-1] > tt[-1]:
        raise ValueError("truth clock does not cover tracking observations")
    index = np.clip(np.searchsorted(tt, t), 1, len(tt) - 1)
    bracket = tt[index] - tt[index - 1]
    if np.max(bracket) > 0.05:
        raise ValueError("truth interpolation gap exceeds 50 ms")
    raw_pose = values(truth, ["x", "y", "z"])
    raw_angles = Rotation.from_quat(values(truth, ["qx", "qy", "qz", "qw"])).as_euler("xyz")
    raw_yaw = raw_angles[:, 2]
    # Crazyflie framework Euler yaw and MuJoCo world yaw share the same sign.
    truth_xyz = np.column_stack([np.interp(t, tt, raw_pose[:, j]) for j in range(3)])
    truth_yaw = np.interp(t, tt, np.unwrap(raw_yaw))
    measured = values(rows, [f"estimated_position_{a}_m" for a in "xyz"] + ["estimated_yaw_rad"])
    target = values(rows, [f"reference_position_{a}_m" for a in "xyz"] + ["reference_yaw_rad"])
    actual = np.column_stack((truth_xyz, truth_yaw))

    def difference(a, b):
        error = a - b
        error[:, 3] = (error[:, 3] + np.pi) % (2 * np.pi) - np.pi
        return error

    tracking = difference(actual, target)
    estimation = difference(measured, actual)
    diagnostics = [json.loads(r["controller_diagnostics_json"]) for r in rows]
    controls = values(rows, ["command_roll_rate_rad_s", "command_pitch_rate_rad_s",
                             "command_yaw_rate_rad_s", "command_thrust_normalized"])
    meta = json.loads((flight.parent / "metadata.json").read_text())
    controller = meta["controller_configuration"]
    config = NearHoverModelConfig(**controller["model"])
    delta = controls - [0, 0, 0, config.equilibrium_thrust]
    # Evaluate the physics model from simulator state with actually issued inputs.
    # Crazyflie estimated pitch follows the legacy negative physical pitch convention.
    raw_angles[:, 1] *= -1
    raw_angles[:, 2] = np.unwrap(raw_angles[:, 2])
    raw_state = np.column_stack((raw_pose, values(truth, ["vx", "vy", "vz"]), raw_angles))
    raw_state[:, :3] -= actual[0, :3]
    rotation = world_to_fixed_yaw(float(measured[0, 3]))
    raw_state[:, :3] = raw_state[:, :3] @ rotation.T
    raw_state[:, 3:6] = raw_state[:, 3:6] @ rotation.T
    raw_state[:, 8] -= measured[0, 3]
    sent_at = np.asarray([d["timing"]["send_completed_monotonic_s"] for d in diagnostics])
    ad, bd = discrete_model(config)
    model_errors = {}
    for depth in (1, 15, 25, 40):
        errors = []
        for start in sent_at:
            end = start + depth * config.sample_period_s
            if end >= sent_at[-1]:
                break
            x = np.array([np.interp(start, tt, channel) for channel in raw_state.T])
            for k in range(depth):
                tick = start + k * config.sample_period_s
                command = np.searchsorted(sent_at, tick, side="right") - 1
                x = ad @ x + bd @ delta[command]
            y = np.array([np.interp(end, tt, channel) for channel in raw_state.T])
            errors.append(x - y)
        if errors:
            model_errors[str(depth)] = {
                "windows": len(errors), "rmse": np.sqrt(np.mean(np.square(errors), axis=0)).tolist()
            }
    events = read_csv(flight.parent / "events.csv")
    fallback = [e for e in events if e["event"] == "phase_changed" and
                json.loads(e["details_json"]).get("phase") == "FALLBACK_LANDING"]
    solve = [d.get("mpc", {}).get("qp_elapsed_s", np.nan) * 1000 for d in diagnostics]
    solver_ms = np.asarray(solve)
    deadline = sum(e["event"] == "control_deadline_missed" for e in events)
    hard_inputs = int(np.count_nonzero(
        np.any(np.abs(controls[:, :3]) > controller["max_rate"] + 1e-8, axis=1)
        | (controls[:, 3] < controller["min_thrust"] - 1e-8)
        | (controls[:, 3] > controller["max_thrust"] + 1e-8)
    ))
    loop = np.asarray([d.get("timing", {}).get("step_work_s", np.nan) * 1000 for d in diagnostics])
    observed_angles = values(rows, ["estimated_roll_rad", "estimated_pitch_rad", "estimated_yaw_rad"])
    true_angles = np.column_stack([np.interp(t, tt, raw_angles[:, j]) for j in range(3)])
    attitude_error = (observed_angles - true_angles + np.pi) % (2 * np.pi) - np.pi
    safety = meta["reference"]["configuration"]["safety"]
    start_time = float(flight_rows[0]["estimated_timestamp_s"])
    ground = np.asarray([np.interp(start_time, tt, channel) for channel in raw_pose.T])
    displacement = truth_xyz - ground
    truth_velocity = np.column_stack([
        np.interp(t, tt, channel) for channel in values(truth, ["vx", "vy", "vz"]).T
    ])
    hard_truth = int(np.count_nonzero(
        (np.linalg.norm(displacement[:, :2], axis=1) > safety["max_horizontal_displacement_m"])
        | (displacement[:, 2] > safety["max_altitude_above_ground_m"])
        | (displacement[:, 2] < safety["min_altitude_below_ground_m"])
        | (np.linalg.norm(truth_velocity[:, :2], axis=1) > safety["max_horizontal_speed_m_s"])
        | (np.abs(truth_velocity[:, 2]) > safety["max_vertical_speed_m_s"])
        | (np.max(np.abs(true_angles[:, :2]), axis=1) > safety["max_tilt_rad"])
    ))
    row.update(
        reference=meta["reference"],
        controller=controller,
        tracking_rmse_xyz_yaw=np.sqrt(np.mean(tracking**2, axis=0)).tolist(),
        tracking_p95_xyz_yaw=np.percentile(np.abs(tracking), 95, axis=0).tolist(),
        estimation_rmse_xyz_yaw=np.sqrt(np.mean(estimation**2, axis=0)).tolist(),
        attitude_estimation_rmse_rad=np.sqrt(np.mean(attitude_error**2, axis=0)).tolist(),
        truth_bracket_p99_ms=float(1000 * np.percentile(bracket, 99)),
        truth_observer_p99_ms=float(1000 * np.percentile(values(truth, ["observer_s"]), 99)),
        truth_real_time_factor=float((float(truth[-1]["sim_s"]) - float(truth[0]["sim_s"])) / (tt[-1] - tt[0])),
        actual_input_model_rmse=model_errors,
        effort=np.mean(delta**2, axis=0).tolist(),
        increment_rms=np.sqrt(np.mean(np.diff(controls, axis=0)**2, axis=0)).tolist(),
        deadline_misses=deadline,
        fallback=len(fallback),
        solver_statuses=sorted({d.get("mpc", {}).get("status", "unavailable") for d in diagnostics}),
        hard_input_violation_steps=hard_inputs,
        truth_envelope_violation_tracking_steps=hard_truth,
        solver_p99_ms=float(np.nanpercentile(solver_ms, 99)),
        solver_ms_percentiles=np.nanpercentile(solver_ms, [50, 95, 99, 100]).tolist(),
        loop_ms_percentiles=np.nanpercentile(loop, [50, 95, 99, 100]).tolist(),
        max_slack=max((d.get("mpc", {}).get("max_slack", 0) for d in diagnostics), default=0),
        soft_activation_steps=sum(d.get("mpc", {}).get("soft_boundary_count", 0) > 0 for d in diagnostics),
        input_activation_steps=sum(d.get("mpc", {}).get("active_input_count", 0) > 0 for d in diagnostics),
        first_sent_input=controls[0].tolist(),
        feasible=row["status"] == "completed" and not fallback and not any((deadline, hard_inputs, hard_truth)),
    )
    if meta["reference"]["type"] == "circular":
        radius = meta["reference"]["configuration"]["profile"]["circle_radius"]
        center = target[0, :2] - [radius, 0]
        desired_angle = np.unwrap(np.arctan2(*(target[:, :2] - center).T[::-1]))
        real_angle = np.unwrap(np.arctan2(*(actual[:, :2] - center).T[::-1]))
        # References are evaluated at each runner tick. Asynchronous observation
        # timestamps can repeat, so use the strictly increasing reference clock.
        reference_clock = values(rows, ["monotonic_s"])[:, 0]
        rate = np.gradient(desired_angle, reference_clock)
        active = rate > 0.5 * np.max(rate)
        if np.any(active):
            row["phase_lag_s"] = float(np.median((desired_angle[active] - real_angle[active]) / rate[active]))
    step = meta["reference"]["configuration"].get("step_axis")
    if step:
        j = ("x", "y", "z", "yaw").index(step)
        changes = np.flatnonzero(np.abs(np.diff(target[:, j])) > 1e-5)
        if changes.size:
            start = changes[0] + 1
            tolerance = max(abs(target[start, j] - target[start - 1, j]) * 0.05, 0.002)
            outside = np.flatnonzero(abs(tracking[start:, j]) > tolerance)
            candidate = start + int(outside[-1]) + 1 if outside.size else start
            row["settling_s"] = (
                float(t[candidate] - t[start])
                if candidate < len(t) and t[-1] - t[candidate] >= 0.2 else None
            )
            row["steady_error"] = float(np.mean(tracking[start:, j][-20:]))
            row["settling_tolerance"] = float(tolerance)
            row["step_observed_hold_s"] = float(t[-1] - t[start])
            row["settled"] = row["settling_s"] is not None
    return row, (t - t[0], target, measured, actual, controls, diagnostics)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch", type=Path)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    summaries = []
    for case_path in sorted(args.batch.glob("*/experiment.json")):
        try:
            row, traces = describe(case_path.parent)
        except (ValueError, KeyError, OSError) as error:
            raw = json.loads(case_path.read_text())
            row, traces = {**raw["case"], "status": raw["status"], "analysis_error": str(error)}, None
        summaries.append(row)
        if traces is None:
            continue
        t, target, measured, actual, controls, diagnostics = traces
        fig, axes = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
        for j, label in enumerate(("X [m]", "Y [m]", "Z [m]", "Yaw [rad]")):
            axes[j].plot(t, target[:, j], "--", label="reference")
            axes[j].plot(t, measured[:, j], label="estimated")
            axes[j].plot(t, actual[:, j], label="truth")
            axes[j].set_ylabel(label)
        axes[0].legend()
        axes[4].plot(t, controls)
        axes[4].set_ylabel("Command")
        axes[5].plot(t, [d.get("mpc", {}).get("max_slack", 0) for d in diagnostics])
        axes[5].set(ylabel="Maximum slack", xlabel="Tracking time [s]")
        fig.tight_layout()
        fig.savefig(case_path.parent / "truth-tracking.png")
        plt.close(fig)

    (args.batch / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    groups = {}
    for row in summaries:
        key = (row["name"], row.get("mode"), row.get("horizon"))
        groups.setdefault(key, []).append(row)
    aggregate = []
    for (name, mode, horizon), runs in groups.items():
        available = [r for r in runs if "tracking_rmse_xyz_yaw" in r]
        feasible = [r for r in available if r["feasible"]]
        entry = {"name": name, "mode": mode, "horizon": horizon,
                 "runs": len(runs), "scored": len(available), "feasible": len(feasible)}
        for metric in ("tracking_rmse_xyz_yaw", "estimation_rmse_xyz_yaw", "phase_lag_s",
                       "solver_p99_ms", "settling_s", "steady_error", "effort", "increment_rms",
                       "deadline_misses", "fallback", "soft_activation_steps", "input_activation_steps"):
            samples = [run[metric] for run in available if run.get(metric) is not None]
            if samples:
                entry[metric] = {"n": len(samples), "mean": np.mean(samples, axis=0).tolist(),
                                 "std": np.std(samples, axis=0).tolist()}
        aggregate.append(entry)
    (args.batch / "aggregate.json").write_text(json.dumps(aggregate, indent=2) + "\n")
    scored = [r for r in summaries if "tracking_rmse_xyz_yaw" in r]
    if scored:
        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        indices = np.arange(1, len(scored) + 1)
        axes[0].plot(indices, [np.linalg.norm(r["tracking_rmse_xyz_yaw"][:3]) for r in scored], "o")
        axes[1].plot(indices, [r["solver_p99_ms"] for r in scored], "o")
        axes[2].plot(indices, [r.get("phase_lag_s", np.nan) for r in scored], "o")
        axes[0].set_ylabel("Position RMSE [m]")
        axes[1].set_ylabel("QP P99 [ms]")
        axes[2].set(ylabel="Phase lag [s]", xlabel="Scored run index")
        fig.tight_layout()
        fig.savefig(args.batch / "cross-run.png")
        plt.close(fig)
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        for metric, ax, label in zip(
            ("tracking_rmse_xyz_yaw", "solver_p99_ms", "phase_lag_s"), axes,
            ("Position RMSE [m]", "QP P99 [ms]", "Phase lag [s]"),
        ):
            for j, entry in enumerate(aggregate):
                for run in groups[(entry["name"], entry["mode"], entry["horizon"])]:
                    if run.get(metric) is None:
                        continue
                    value = (np.linalg.norm(run[metric][:3]) if metric == "tracking_rmse_xyz_yaw"
                             else run[metric])
                    ax.scatter(j, value, color="tab:blue" if run["feasible"] else "tab:red")
            ax.set_ylabel(label)
        axes[-1].set_xticks(range(len(aggregate)), [
            f"{e['name']}\n{e['mode']}, N={e['horizon']}\n{e['feasible']}/{e['runs']} feasible"
            for e in aggregate
        ], rotation=30, ha="right")
        fig.tight_layout()
        fig.savefig(args.batch / "group-comparison.png")
        plt.close(fig)


if __name__ == "__main__":
    main()
