"""Apply the frozen MVP score and independently check the experimental logs."""

import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt

from tools.analyze_inspection_mvp import COLORS, csv_output, decision, read_csv, score_run, values


METRICS = ("rmse_m", "max_error_m", "recovery_s", "effort_per_s", "solver_p99_ms")
PREFERENCES = {
    "tracking": [40, 15, 20, 15, 5, 5],
    "reliability": [10, 5, 40, 10, 10, 25],
    "recovery": [20, 10, 20, 35, 5, 10],
    "effort": [15, 5, 20, 10, 35, 15],
}


def verify(run, case, row):
    """Recompute RMSE via vectorized linear interpolation and reproduce position faults."""
    flight = next(run.glob("*/flight.csv"), None)
    if flight is None or not row.get("samples"):
        return {"checked": False}
    cfg = json.loads(flight.with_name("metadata.json").read_text())
    controller = cfg["controller_configuration"]
    assert controller["horizon"] == 25
    if case["baseline"] == "t6-ref":
        assert (controller["past"], controller["regularization"], controller["initialization_weight"]) == (6, .001, .25)
    else:
        assert controller["identified"]["variant"] == "joint-fir6"
    ref_cfg = cfg["reference"]["configuration"]
    fault = ref_cfg["navigation_fault"]
    scale = case["scale"]
    assert (fault["profile"], fault["start_step"], fault["end_step"]) == (
        case["navigation"], case["start_step"], case["start_step"] + 200)
    assert (fault["drift_y_m_s"], fault["bias_y_m"]) == (.005 * scale, .025 * scale)

    rows = [r for r in read_csv(flight) if r["command_thrust_normalized"]]
    ts = values(rows, ["estimated_timestamp_s"])[:, 0]
    velocity = values(rows, [f"estimated_velocity_{a}_m_s" for a in "xyz"])
    estimator = values(rows, [f"estimated_position_{a}_m" for a in "xyz"])
    nav = [json.loads(r["controller_diagnostics_json"])["measurement_noise"] for r in rows]
    np.testing.assert_allclose(estimator, [n["raw_position"] for n in nav], rtol=0, atol=1e-10)
    used = np.asarray([n["used_position"] for n in nav])
    previous = None
    for i, n in enumerate(nav):
        expected = estimator[i].copy()
        if n["active"] and fault["profile"] == "position-loss":
            if previous is None or n["step"] != nav[previous]["step"] + 1:
                previous = i
                continue
            dt = max(0.0, ts[i] - ts[previous])
            expected = used[previous] + velocity[i] * dt
            expected[1] += fault["drift_y_m_s"] * dt
        elif n["active"]:
            expected[1] += fault["bias_y_m"]
        np.testing.assert_allclose(used[i], expected, rtol=0, atol=1e-10)
        previous = i

    if row.get("rmse_m") is None:
        return {"checked": True, "fault_samples": len(rows), "truth_checked": False}
    sim = values(read_csv(run / "sim_truth.csv"), ["monotonic_s", "x", "y", "z"])
    hi = np.searchsorted(sim[:, 0], ts).clip(1, len(sim) - 1)
    lo = hi - 1
    fraction = (ts - sim[lo, 0]) / (sim[hi, 0] - sim[lo, 0])
    xyz = sim[lo, 1:] + fraction[:, None] * (sim[hi, 1:] - sim[lo, 1:])
    target = values(rows, [f"reference_position_{a}_m" for a in "xyz"])
    errors_squared = np.sum((xyz - target)**2, axis=1)
    rmse = float(np.sqrt(errors_squared.mean()))
    np.testing.assert_allclose([rmse, np.sqrt(errors_squared.max())],
                               [row["rmse_m"], row["max_error_m"]], atol=1e-12)
    return {"checked": True, "fault_samples": len(rows), "truth_checked": True,
            "independent_rmse_m": rmse}


def condition(row):
    return row["scenario"], row["scale"], row["start_step"]


def summarize(rows, metrics):
    return {field: dict(n=len(sample), median=float(np.median(sample)),
                        min=float(min(sample)), max=float(max(sample)))
            for field in metrics
            if (sample := [r[field] for r in rows if r.get(field) is not None])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("iteration", type=Path)
    parser.add_argument("output", type=Path, help="New analysis directory")
    args = parser.parse_args()
    out = args.output
    out.mkdir()
    root = args.iteration
    scoring = root / "scoring.json"
    if not scoring.is_file():
        scoring = root.parent / "scoring.json"
    if not scoring.is_file():
        scoring = Path(__file__).resolve().parent / "config" / "scoring.json"
    settings = json.loads(scoring.read_text())
    (out / "scoring.json").write_bytes(scoring.read_bytes())
    plan = json.loads((root / "plan.json").read_text())
    rows, checks = [], []
    for i, case in enumerate(plan["cases"], 1):
        run = root / f"{i:03d}"
        row, trace = score_run(run, case, settings)
        row.update(scale=case["scale"], start_step=case["start_step"], repetition=case["repetition"])
        start = case["start_step"]
        for label, left, right in (("pre", 0, start), ("window", start, start + 200),
                                   ("post", start + 200, 600)):
            portion = [x for x in trace if left <= x["step"] < right]
            row[f"{label}_ticks"] = len(portion)
            if portion:
                row[f"{label}_rmse_m"] = float(np.sqrt(np.mean([x["error_m"]**2 for x in portion])))
                row[f"{label}_qp_p99_ms"] = float(np.percentile([x["qp_ms"] for x in portion], 99))
        if case["navigation"] != "nominal":
            followup = [x["time_s"] for x in trace if x["step"] >= start + 200]
            if followup:
                row["recovery_followup_s"] = followup[-1] - followup[0]
            row["recovery_right_censored"] = row["recovery_status"] != "recovered"
        flight = next(run.glob("*/flight.csv"), None)
        if flight is not None:
            for logged in read_csv(flight):
                info = json.loads(logged["controller_diagnostics_json"])
                if reason := info.get("fallback_reason"):
                    row["fallback_reason"] = reason
                    row["first_fallback_elapsed_s"] = float(logged["elapsed_s"])
                    nav = info.get("measurement_noise", {})
                    row["fallback_attempt_tick"] = nav.get("step")
                    if "step" in nav and case["navigation"] != "nominal":
                        row["fallback_ticks_from_fault_end"] = nav["step"] - (start + 200)
                    row["tracking_steps_before_fallback"] = row["samples"]
                    break
        check = verify(run, case, row)
        checks.append(dict(run=run.name, **check))
        rows.append(row)
    assert all(row["status"] != "not_run" for row in rows)
    csv_output(out / "comparison.csv", rows)
    matrix = decision(rows, settings)
    csv_output(out / "decision-matrix.csv", matrix)
    (out / "verification.json").write_text(json.dumps(checks, indent=2) + "\n")
    sensitivity = []
    for name, weights in {"original": list(settings["weights"].values()), **PREFERENCES}.items():
        varied = {**settings, "weights": dict(zip(settings["weights"], weights))}
        mpc, deepc = decision(rows, varied)
        sensitivity.append(dict(weights=name, joint_fir6=mpc["weighted_total"], t6_ref=deepc["weighted_total"],
                                deepc_minus_mpc=round(deepc["weighted_total"] - mpc["weighted_total"], 3)))
    csv_output(out / "weight-sensitivity.csv", sensitivity)
    pairs = defaultdict(dict)
    for row in rows:
        pairs[(*condition(row), row["repetition"])][row["controller"]] = row
    deltas = []
    for (scenario, scale, start, rep), pair in pairs.items():
        mpc, deepc = pair["joint-fir6"], pair["t6-ref"]
        deltas.append(dict(scenario=scenario, scale=scale, start_step=start, repetition=rep,
                           **{field: deepc[field] - mpc[field] for field in METRICS
                              if deepc.get(field) is not None and mpc.get(field) is not None
                              and (field not in ("rmse_m", "max_error_m", "effort_per_s")
                                   or (deepc.get("tracking_complete") and mpc.get("tracking_complete")))}))
    csv_output(out / "paired-differences-deepc-minus-mpc.csv", deltas)
    conditions = list(dict.fromkeys(condition(r) for r in rows))
    summary = []
    for key in conditions:
        subset = [r for r in rows if condition(r) == key]
        data = {}
        for controller in COLORS:
            ctrl = [r for r in subset if r["controller"] == controller]
            data[controller] = dict(n=len(ctrl), passed=sum(bool(r.get("violation_free_completion")) for r in ctrl),
                                    metrics=summarize(ctrl, (*METRICS, "pre_rmse_m", "window_rmse_m", "post_rmse_m",
                                                             "pre_qp_p99_ms", "window_qp_p99_ms", "post_qp_p99_ms")))
        summary.append(dict(scenario=key[0], scale=key[1], start_step=key[2], **data,
                            paired_delta=summarize([r for r in deltas if condition(r) == key], METRICS)))
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    fig, axes = plt.subplots(2, 2, figsize=(max(10, len(conditions)*1.8), 7))
    for ax, field, factor, label in zip(axes.flat, ("rmse_m", "recovery_s", "effort_per_s", "solver_p99_ms"),
                                       (1000, 1, 1, 1), ("3D RMSE (mm)", "Recovery (s)",
                                       "Weighted effort per second", "QP P99 (ms)")):
        for i, key in enumerate(conditions):
            for c, (controller, color) in enumerate(COLORS.items()):
                points = [r for r in rows if condition(r) == key and r["controller"] == controller]
                jitter = np.linspace(-.045, .045, len(points))
                for j, r in enumerate(points):
                    if r.get(field) is not None:
                        ax.scatter(i + (c*2-1)*.13 + jitter[j], factor*r[field], color=color, alpha=.8,
                                   marker="o" if r.get("violation_free_completion") else "x")
        ax.set_xticks(range(len(conditions)), [f"{k[0].replace('position-', '')}\n{k[1]:+g}, step {k[2]}"
                                               for k in conditions], fontsize=8)
        ax.set_ylabel(label)
        ax.grid(alpha=.25, axis="y")
    fig.suptitle("Joint FIR6 (blue) | T6-ref (green) | one point per run")
    fig.tight_layout()
    for suffix in ("png", "svg"):
        fig.savefig(out / f"comparison.{suffix}", dpi=300)
    print(json.dumps(dict(version=plan["version"], runs=len(rows),
                          verified_truth=sum(c.get("truth_checked", False) for c in checks),
                          qualified=sum(bool(r.get("violation_free_completion")) for r in rows),
                          decision=matrix, sensitivity=sensitivity), indent=2))


if __name__ == "__main__":
    main()
