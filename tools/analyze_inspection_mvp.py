"""Score the small CrazySim line-inspection matrix and retain partial results."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from tools.analyze_mpc_batch import describe, read_csv, values
from velocity_control_framework.backends.crazysim_command_adapter import CrazySimCommandAdapter as Adapter


CONTROLLERS = ("joint-fir6", "t6-ref")
COLORS = {"joint-fir6": "#2864A0", "t6-ref": "#00866A"}


def csv_output(path, rows):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def score_run(run, case, settings):
    row = dict(controller=case["baseline"], scenario=case["navigation"], run=str(run),
               status="not_run", samples=0, recovery_status="unavailable")
    if not (run / "experiment.json").is_file():
        return row, []
    experiment = json.loads((run / "experiment.json").read_text())
    row.update(status=experiment["status"], exit_code=experiment.get("exit_code"))
    flight = next(run.glob("*/flight.csv"), None)
    if flight is None:
        row["issue"] = experiment.get("error", "flight telemetry absent")
        return row, []
    meta = json.loads(flight.with_name("metadata.json").read_text())
    fault = meta["reference"]["configuration"]["navigation_fault"]
    rows = [r for r in read_csv(flight) if r["command_thrust_normalized"]]
    diagnostics = [json.loads(r["controller_diagnostics_json"]) for r in rows]
    events = read_csv(flight.with_name("events.csv"))
    row.update(
        samples=len(rows), landing_status=meta["status"],
        termination=meta.get("termination_reason"),
        fallback=sum(e["event"] == "phase_changed" and
                     json.loads(e["details_json"]).get("phase") == "FALLBACK_LANDING" for e in events),
        deadline_misses=sum(e["event"] == "control_deadline_missed" for e in events),
    )
    if fault["profile"] == "nominal":
        row["recovery_status"] = "not_applicable"
    if not rows:
        row["issue"] = "No issued tracking commands"
        return row, []
    steps = np.array([d["measurement_noise"]["step"] for d in diagnostics])
    active = np.array([d["measurement_noise"]["active"] for d in diagnostics])
    expected_active = (steps >= fault["start_step"]) & (steps < fault["end_step"])
    if fault["profile"] == "nominal":
        expected_active[:] = False
    if not np.array_equal(active, expected_active):
        raise ValueError(f"Inconsistent navigation log: {run}")
    row["fault_ticks_observed"] = int(active.sum())
    qp_ms = np.array([d["mpc"]["qp_elapsed_s"] * 1000 for d in diagnostics])
    row.update(solver_p99_ms=float(np.percentile(qp_ms, 99)), solver_max_ms=float(qp_ms.max()),
               qp_over_budget=int(np.count_nonzero(qp_ms > 12)))
    try:
        base, traces = describe(run)
    except (ValueError, KeyError, IndexError) as error:
        row["issue"] = str(error)
        return row, []
    if traces is None:
        row["issue"] = "Insufficient aligned truth and tracking telemetry"
        return row, []
    time, target, _, truth, commands, _ = traces
    error = np.linalg.norm(truth[:, :3] - target[:, :3], axis=1)
    config = meta["controller_configuration"]
    commands[:, 3] = [(Adapter._encode_thrust(u) - Adapter.RAW_THRUST_MIN) /
                      (Adapter.RAW_THRUST_MAX - Adapter.RAW_THRUST_MIN) for u in commands[:, 3]]
    delta = commands - [0, 0, 0, config["model"]["equilibrium_thrust"]]
    dt = values(rows, ["control_dt_s"])[:, 0]
    full = len(rows) >= settings["expected_minimum_tracking_ticks"]
    row.update(tracking_complete=full, rmse_m=float(np.sqrt(np.mean(error**2))),
               max_error_m=float(np.max(error)),
               effort_per_s=float(np.average(np.square(delta) @ config["r"], weights=dt)),
               hard_input_violations=base["hard_input_violation_steps"],
               truth_envelope_violations=base["truth_envelope_violation_tracking_steps"],
               soft_constraint_ticks=base["soft_activation_steps"],
               runner_p99_ms=base["loop_ms_percentiles"][2])
    row["violation_free_completion"] = bool(full and row["status"] == "completed"
        and row["landing_status"] == "completed" and not any(row[k] for k in
        ("fallback", "deadline_misses", "hard_input_violations",
         "truth_envelope_violations", "qp_over_budget")))
    if fault["profile"] != "nominal":
        recovery = np.flatnonzero(steps >= fault["end_step"])
        hold = settings["recovery_hold_ticks"]
        if len(recovery) >= hold:
            first = int(recovery[0])
            for i in range(first, len(error) - hold + 1):
                if np.all(np.diff(steps[i:i + hold]) == 1) and np.all(
                        error[i:i + hold] <= settings["recovery_error_m"]):
                    row.update(recovery_s=float(time[i] - time[first]), recovery_status="recovered")
                    break
            else:
                row["recovery_status"] = "not_recovered" if full else "censored_partial"
        else:
            row["recovery_status"] = "censored_partial"
    data = [dict(controller=row["controller"], scenario=row["scenario"], step=int(steps[i]),
                 time_s=float(time[i]), fault_active=bool(active[i]), error_m=float(error[i]),
                 reference_x_m=float(target[i, 0]), truth_x_m=float(truth[i, 0]),
                 reference_y_m=float(target[i, 1]), truth_y_m=float(truth[i, 1]),
                 qp_ms=float(qp_ms[i])) for i in range(len(rows))]
    return row, data


def decision(rows, settings):
    matrix = []
    for controller in CONTROLLERS:
        cases = [r for r in rows if r["controller"] == controller]
        entry = dict(controller=controller)
        total = 0
        for key, weight in settings["weights"].items():
            scores = []
            for r in cases:
                if key == "recovery_s" and r["scenario"] not in settings["recovery_profiles"]:
                    continue
                if key == "violation_free_completion":
                    scores.append(100 * bool(r.get(key)))
                    continue
                value = r.get(key)
                if key in ("rmse_m", "max_error_m", "effort_per_s") and not r.get("tracking_complete"):
                    value = None
                limit = settings["zero_score_at"][key]
                scores.append(0 if value is None else 100 * max(0, 1 - value / limit))
            entry[key + "_score"] = round(float(np.mean(scores)), 3)
            total += float(np.mean(scores)) * weight / 100
        entry["weighted_total"] = round(total, 3)
        matrix.append(entry)
    return matrix


def figures(out, rows, traces, scenarios):
    fig, axes = plt.subplots(2, len(scenarios), figsize=(11, 5), sharex="col", sharey="row")
    for j, scenario in enumerate(scenarios):
        axes[0, j].set_title(scenario)
        for controller in CONTROLLERS:
            data = [r for r in traces if (r["controller"], r["scenario"]) == (controller, scenario)]
            if not data:
                continue
            t = [r["time_s"] for r in data]
            active = [r["time_s"] for r in data if r["fault_active"]]
            color = COLORS[controller]
            style = "--" if controller == "t6-ref" else "-"
            axes[0, j].plot(t, [r["error_m"] * 1000 for r in data], style, color=color, label=controller)
            axes[1, j].plot(t, [r["qp_ms"] for r in data], style, color=color)
            if active and controller == "joint-fir6":
                for ax in axes[:, j]:
                    ax.axvspan(active[0], active[-1], color="0.85", zorder=-1)
        axes[1, j].axhline(12, color="0.5", linewidth=0.8)
        axes[1, j].set_xlabel("Tracking time (s)")
    axes[0, 0].set_ylabel("3D truth error (mm)")
    axes[1, 0].set_ylabel("QP elapsed (ms)")
    axes[0, 0].legend(fontsize=8)
    fig.tight_layout()
    for suffix in ("png", "svg"):
        fig.savefig(out / f"tracking-and-solver.{suffix}", dpi=300)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    for r in rows:
        if r.get("rmse_m") is not None:
            x = scenarios.index(r["scenario"])
            controller = r["controller"]
            ax.scatter(x + (-0.08 if controller == "joint-fir6" else 0.08), r["rmse_m"] * 1000,
                       color=COLORS[controller], marker="o" if r.get("violation_free_completion") else "x")
    ax.set_xticks(range(len(scenarios)), scenarios)
    ax.set_ylabel("3D truth RMSE (mm)")
    ax.set_title("MPC (blue), DeePC (green); × marks a failed completion gate")
    fig.tight_layout()
    for suffix in ("png", "svg"):
        fig.savefig(out / f"scenario-rmse.{suffix}", dpi=300)
    plt.close(fig)

    fig, axes = plt.subplots(1, len(scenarios), figsize=(11, 3.4), sharex=True, sharey=True)
    for ax, scenario in zip(axes, scenarios):
        shown_reference = False
        for controller in CONTROLLERS:
            data = [r for r in traces if (r["controller"], r["scenario"]) == (controller, scenario)]
            if not data:
                continue
            x0, y0 = data[0]["reference_x_m"], data[0]["reference_y_m"]
            if not shown_reference:
                ax.plot([(r["reference_x_m"] - x0) * 1000 for r in data],
                        [(r["reference_y_m"] - y0) * 1000 for r in data],
                        ":", color="0.25", label="Reference")
                shown_reference = True
            ax.plot([(r["truth_x_m"] - x0) * 1000 for r in data],
                    [(r["truth_y_m"] - y0) * 1000 for r in data],
                    "--" if controller == "t6-ref" else "-", color=COLORS[controller],
                    label=controller)
        ax.set(title=scenario, xlabel="Along-line X (mm)")
    axes[0].set_ylabel("Cross-track Y (mm)")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    for suffix in ("png", "svg"):
        fig.savefig(out / f"inspection-trajectory.{suffix}", dpi=300)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch", type=Path)
    out = parser.parse_args().batch
    settings = json.loads((out / "scoring.json").read_text())
    rows, traces = [], []
    for controller in CONTROLLERS:
        cases = json.loads((out / f"{controller}-matrix.json").read_text())
        for i, case in enumerate(cases, start=1):
            row, data = score_run(out / controller / f"{i:03d}", case, settings)
            rows.append(row)
            traces.extend(data)
    csv_output(out / "comparison.csv", rows)
    if traces:
        csv_output(out / "tracking-timeseries.csv", traces)
        figures(out, rows, traces, settings["profiles"])
    csv_output(out / "decision-matrix.csv", decision(rows, settings))


if __name__ == "__main__":
    main()
