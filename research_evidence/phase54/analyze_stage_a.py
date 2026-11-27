"""Phase 5.4 Stage A analysis: preregistered gates, figures, Markdown + HTML report.

Usage (repo root, PYTHONPATH=.):
  .venv/bin/python research_evidence/phase54/analyze_stage_a.py log/phase54/stage-a research_evidence/phase54/stage-a

Reads only Stage A run directories and the unchanged scorer outputs stored in
summary.json. Implements plan.md §3–§6 exactly; thresholds are constants here.
"""
import base64
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.shared_dataset import read_csv  # noqa: E402

CONTROLLERS = ["t6-ref", "t6-a", "t6-b"]
LABEL = {"t6-ref": "DeePC T6-ref (ρ=.001, β=.25)", "t6-a": "DeePC T6-A (ρ=.01, β=.0625)",
         "t6-b": "DeePC T6-B (ρ=.01, β=.125)"}
SHORT = {"t6-ref": "T6-ref", "t6-a": "T6-A", "t6-b": "T6-B"}
# DeePC family = green dashed (protocol); configurations differ by lightness + marker.
COLOR = {"t6-ref": "#00866A", "t6-a": "#5BB89F", "t6-b": "#0A4D3E"}
MARKER = {"t6-ref": "o", "t6-a": "s", "t6-b": "^"}
SCENES = ["hover", "yaw7-pos", "yaw7-neg", "z4-pos", "z4-neg", "circle6"]
SCENE_MARK = dict(zip(SCENES, ["o", "^", "v", "P", "X", "D"]))
F_P, F_YAW, F_C = 0.5e-3, 1.0e-3, 1.0e-3
BUDGET_MS = 12.0
plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": "#dddddd", "grid.linewidth": .5,
                     "svg.fonttype": "none"})


# ---------------------------------------------------------------- data
def load(stage_dir):
    rows = json.loads((stage_dir / "summary.json").read_text())
    final = {}
    for r in rows:  # later attempt (infrastructure retry) supersedes, first kept in `attempts`
        final.setdefault(r["case"]["name"], []).append(r)
    runs = []
    for name, attempts in final.items():
        r = dict(attempts[-1])
        r["attempts"] = [dict(path=a["path"], status=a["status"], infrastructure_failure=a.get("infrastructure_failure"))
                         for a in attempts]
        ct = r.get("common_tracking")
        if ct:
            e = ct["truth_rmse_xyz_yaw"]
            r["P"] = math.sqrt(e[0] ** 2 + e[1] ** 2 + e[2] ** 2)
            r["yaw"] = e[3]
            r["C"] = ct["physical_cost_s"]["total"]
        runs.append(r)
    return rows, runs


def reason(r):
    if r.get("eligible"):
        return "eligible"
    out = []
    if r["status"] != "completed": out.append(f"status={r['status']}")
    if r.get("analysis_error"): out.append("analysis_error")
    if r.get("scoring_ineligible"): out.append("incomplete scoring window")
    if r.get("fallback"): out.append(f"fallback ({r.get('fallback_reason')})")
    if r.get("deadline_misses"): out.append(f"{r['deadline_misses']} deadline miss")
    if r.get("hard_input_violation_steps"): out.append("hard input violation")
    if r.get("truth_envelope_violation_tracking_steps"): out.append("truth envelope violation")
    if r.get("qp_overruns"): out.append(f"{len(r['qp_overruns'])} QP>12 ms")
    return "; ".join(out) or "ineligible"


def trajectory(run_path):
    run = ROOT / run_path
    flight = next(run.glob("*/flight.csv"))
    rows = [r for r in read_csv(flight) if r["command_thrust_normalized"]]
    truth = read_csv(run / "sim_truth.csv")
    tt = np.array([float(r["monotonic_s"]) for r in truth])
    t = np.array([float(r["estimated_timestamp_s"]) for r in rows])
    q = np.array([[float(r[k]) for k in ("qx", "qy", "qz", "qw")] for r in truth])
    yaw = np.unwrap(Rotation.from_quat(q).as_euler("xyz")[:, 2])
    tr = np.column_stack([np.interp(t, tt, [float(r[k]) for r in truth]) for k in "xyz"] + [np.interp(t, tt, yaw)])
    ref = np.array([[float(r[f"reference_position_{a}_m"]) for a in "xyz"] + [float(r["reference_yaw_rad"])] for r in rows])
    u = np.array([[float(r[k]) for k in ("command_roll_rate_rad_s", "command_pitch_rate_rad_s",
                                         "command_yaw_rate_rad_s", "command_thrust_normalized")] for r in rows])
    diag = [json.loads(r["controller_diagnostics_json"]).get("mpc", {}) for r in rows]
    bridge = sum(d.get("active_controller") in {"mpc_history_bridge", "m0_history_bridge"} for d in diag)
    tick = np.array([float(r["elapsed_s"]) for r in rows])
    return dict(t=tick - tick[0], truth=tr, ref=ref, u=u, bridge=bridge)


def steady_yaw(r):
    """Signed mean truth yaw error over the last 25 ticks of the 90-tick step window (diagnostic)."""
    if not r.get("common_tracking") or not r["case"]["scene"].startswith("yaw"):
        return None
    tj = trajectory(r["path"])
    ref = tj["ref"][:, 3]
    onset = int(np.flatnonzero(np.abs(ref - ref[0]) > .001)[0])
    i = np.arange(onset + 65, onset + 90)
    err = (tj["truth"][i, 3] - ref[i] + np.pi) % (2 * np.pi) - np.pi
    return float(err.mean())


# ---------------------------------------------------------------- gates
def gm(x):
    return float(np.exp(np.mean(np.log(x)))) if len(x) else None


def ratio_gate(pairs, floor, threshold, improvement=False):
    """plan.md §5. pairs = [(cand, ref)]. Returns dict with pass and audit fields."""
    pairs = [(c, r) for c, r in pairs if c is not None and r is not None and np.isfinite(c) and np.isfinite(r)]
    elig = [(c, r) for c, r in pairs if r >= floor]
    small = [(c, r) for c, r in pairs if r < floor]
    out = dict(n_pairs=len(pairs), n_ratio_eligible=len(elig),
               near_zero_pairs=[dict(cand=c, ref=r, diff=c - r) for c, r in small],
               mean_abs_diff=float(np.mean([c - r for c, r in pairs])) if pairs else None)
    small_ok = all(c <= r + floor for c, r in small)
    if len(elig) * 2 >= len(pairs) and elig:
        stat = gm([c / r for c, r in elig]); out["rule"] = "geometric mean of ratio-eligible pairs"
    else:
        mr = float(np.mean([r for _, r in pairs])) if pairs else 0.0
        mc = float(np.mean([c for c, _ in pairs])) if pairs else 0.0
        if mr >= floor:
            stat = mc / mr; out["rule"] = "ratio of arithmetic means (<50% ratio-eligible)"
        else:
            out.update(rule="absolute (mean reference below floor)", statistic=None,
                       passed=(not improvement) and mc <= mr + floor)
            return out
    out["statistic"] = stat
    out["passed"] = bool(stat <= threshold and (improvement or small_ok))
    if not improvement:
        out["near_zero_absolute_ok"] = small_ok
    return out


def pairs_for(runs, cand, level, metric, scene=None):
    idx = {(r["case"]["candidate"], r["case"]["scene"], r["case"]["level"], r["case"]["seed"]): r for r in runs}
    out = []
    for (who, sc, lv, seed), r in idx.items():
        if who != "t6-ref" or lv != level or (scene and sc != scene):
            continue
        c = idx.get((cand, sc, lv, seed))
        out.append(((c or {}).get(metric) if c and c.get("eligible") else None,
                    r.get(metric) if r.get("eligible") else None, sc, seed))
    return out


def evaluate(runs):
    res = {}
    ref_ok = all(r["eligible"] for r in runs if r["case"]["candidate"] == "t6-ref")
    for cand in ("t6-a", "t6-b"):
        mine = [r for r in runs if r["case"]["candidate"] == cand]
        g = {}
        g["G1"] = dict(passed=len(mine) == 24 and all(r["eligible"] for r in mine),
                       eligible=sum(r["eligible"] for r in mine), total=len(mine))
        l1 = pairs_for(runs, cand, 1, "P")
        p2 = [(c, r) for c, r, *_ in l1]
        g["G2a"] = ratio_gate(p2, F_P, 0.90, improvement=True)
        wins = sum(1 for c, r in p2 if c is not None and r is not None and c < r)
        g["G2b"] = dict(passed=wins >= 12, wins=wins, of=len(p2))
        per = {}
        for sc in SCENES:
            sp = [(c, r) for c, r, s, _ in l1 if s == sc and c is not None and r is not None]
            el = [(c, r) for c, r in sp if r >= F_P]
            per[sc] = gm([c / r for c, r in el]) if el else (
                "mean_diff<=0" if sp and np.mean([c - r for c, r in sp]) <= 0 else None)
        ok = sum(1 for v in per.values() if (isinstance(v, float) and v <= 1.0) or v == "mean_diff<=0")
        g["G2c"] = dict(passed=ok >= 5, scenes_ok=ok, per_scene_gm=per)
        g["G3"] = ratio_gate([(c, r) for c, r, *_ in pairs_for(runs, cand, 1, "C")], F_C, 1.10)
        g["G4"] = ratio_gate([(c, r) for c, r, *_ in pairs_for(runs, cand, 1, "yaw")], F_YAW, 1.10)
        g5p = ratio_gate([(c, r) for c, r, *_ in pairs_for(runs, cand, 0, "P")], F_P, 1.10)
        g5c = ratio_gate([(c, r) for c, r, *_ in pairs_for(runs, cand, 0, "C")], F_C, 1.10)
        g["G5"] = dict(passed=bool(g5p.get("passed") and g5c.get("passed")), P=g5p, C=g5c)
        g["all_passed"] = all(bool(v.get("passed")) for k, v in g.items() if k.startswith("G"))
        res[cand] = g
    passing = sorted([c for c in res if res[c]["all_passed"]], key=lambda c: res[c]["G2a"]["statistic"])
    if not ref_ok:
        rec = dict(recommendation=None, reason="t6-ref has an ineligible run; no automatic recommendation (plan §6)")
    elif not passing:
        rec = dict(recommendation="t6-ref", reason="no candidate passed all gates")
    else:
        top = passing[0]
        if len(passing) == 2 and abs(res[passing[0]]["G2a"]["statistic"] - res[passing[1]]["G2a"]["statistic"]) < .02:
            top = "t6-b"
            why = "both passed; G2a statistics differ by <0.02, larger beta preferred"
        else:
            why = "lowest G2a statistic among passing candidates"
        rec = dict(recommendation=top, reason=why)
    rec["t6_ref_all_eligible"] = ref_ok
    return res, rec


# ---------------------------------------------------------------- figures
def save(fig, out, name, data, script_note=""):
    fig.savefig(out / f"{name}.png", dpi=300, bbox_inches="tight")
    fig.savefig(out / f"{name}.svg", bbox_inches="tight")
    plt.close(fig)
    if data:
        with open(out / f"{name}.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(data[0]))
            w.writeheader(); w.writerows(data)


def fail_x(ax, x, y, **kw):
    ax.scatter(x, y, marker="x", color="#C0392B", s=40, zorder=5, **kw)


def fig_paired(runs, out):
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.5), sharex=True, sharey=True)
    data = []
    lim = [math.inf, 0]
    for ax, cand in zip(axes, ("t6-a", "t6-b")):
        for c, r, sc, seed in pairs_for(runs, cand, 1, "P"):
            data.append(dict(candidate=cand, scene=sc, seed=seed, level=1, ref_P_mm=None if r is None else r * 1e3,
                             cand_P_mm=None if c is None else c * 1e3, status="paired" if c and r else "missing/ineligible"))
            if c is None or r is None:
                continue
            ax.scatter(r * 1e3, c * 1e3, marker=SCENE_MARK[sc], s=36, facecolor=COLOR[cand],
                       edgecolor="white", linewidth=.8, zorder=3)
            lim = [min(lim[0], r * 1e3, c * 1e3), max(lim[1], r * 1e3, c * 1e3)]
        scored = sum(1 for c, r, *_ in pairs_for(runs, cand, 1, "P") if c is not None and r is not None)
        ax.set_title(f"{SHORT[cand]} vs T6-ref · L1, {scored}/18 pairs scored")
        ax.set_xlabel("T6-ref 3D truth RMSE [mm]")
    if lim[1] == 0:
        lim = [1, 10]
    lo, hi = lim[0] * .8, lim[1] * 1.15
    for ax in axes:
        ax.plot([lo, hi], [lo, hi], color="#555", lw=1, label="y = x")
        ax.plot([lo, hi], [.9 * lo, .9 * hi], color="#999", lw=1, ls=":", label="10% better")
        ax.set_xscale("log"); ax.set_yscale("log")
        ax.set_xlim(lo, hi); ax.set_ylim(lo, hi); ax.set_aspect("equal")
    axes[0].set_ylabel("Candidate 3D truth RMSE [mm]")
    handles = [plt.Line2D([], [], marker=SCENE_MARK[s], ls="", color="#666", label=s) for s in SCENES]
    fig.legend(handles=handles + axes[1].get_legend_handles_labels()[0], fontsize=6.5, loc="lower center",
               ncol=8, frameon=False, bbox_to_anchor=(.5, -.06))
    fig.suptitle("Stage A paired L1 tracking (below diagonal = candidate better; log axes). "
                 "Unscored pair: run 054 (T6-A, circle6, seed 54162, fallback).", fontsize=8)
    save(fig, out, "A1_paired_L1_tracking", data)


def fig_scenes(runs, out):
    metrics = [("P", 1e3, "3D truth position RMSE [mm]"), ("yaw", 1e3, "Truth yaw RMSE [mrad]"),
               ("C", 1, "Common physical cost [1/s]")]
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 7.4), sharex=True)
    data = []
    for ax, (m, k, lab) in zip(axes, metrics):
        for j, sc in enumerate(SCENES):
            for i, who in enumerate(CONTROLLERS):
                x0 = j + (i - 1) * .25
                for r in runs:
                    if r["case"]["scene"] != sc or r["case"]["candidate"] != who:
                        continue
                    lv = r["case"]["level"]
                    x = x0 + (-.06 if lv == 0 else .04)
                    if r.get(m) is None:
                        fail_x(ax, x, ax.get_ylim()[0] if ax.has_data() else 0)
                        continue
                    v = r[m] * k
                    ax.scatter(x, v, marker=MARKER[who], s=22, zorder=3, facecolor="white" if lv == 0 else COLOR[who],
                               edgecolor=COLOR[who], linewidth=1)
                    if not r["eligible"]:
                        fail_x(ax, x, v)
                    data.append(dict(metric=m, scene=sc, controller=who, level=lv, seed=r["case"]["seed"],
                                     value=v, eligible=r["eligible"]))
                vals = [r[m] * k for r in runs if r["case"]["scene"] == sc and r["case"]["candidate"] == who
                        and r["case"]["level"] == 1 and r.get(m) is not None]
                if vals:
                    ax.hlines(np.median(vals), x0 - .09, x0 + .11, color=COLOR[who], lw=1.5)
        ax.set_ylabel(lab)
        ax.set_yscale("log")
    axes[-1].set_xticks(range(len(SCENES)), SCENES)
    handles = [plt.Line2D([], [], marker=MARKER[w], ls="", markerfacecolor=COLOR[w], markeredgecolor=COLOR[w],
                          label=LABEL[w]) for w in CONTROLLERS]
    handles += [plt.Line2D([], [], marker="o", ls="", markerfacecolor="white", markeredgecolor="#555", label="L0 (1 run)"),
                plt.Line2D([], [], marker="o", ls="", color="#555", label="L1 (3 seeds, bar = median)"),
                plt.Line2D([], [], marker="x", ls="", color="#C0392B", label="ineligible run")]
    fig.legend(handles=handles, fontsize=6.5, ncol=3, frameon=False, loc="upper center", bbox_to_anchor=(.5, .965))
    fig.suptitle("Stage A per-scene results, every run shown (n = 1 L0 + 3 L1 per controller and scene; "
                 "red × at panel floor = no score)", fontsize=8)
    save(fig, out, "A2_per_scene_runs", data)


def fig_cost(runs, out):
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.2), sharey=True)
    data = []
    parts = ["tracking", "effort", "increment"]
    shade = {"tracking": 1.0, "effort": .6, "increment": .3}
    for ax, lv in zip(axes, (0, 1)):
        for i, who in enumerate(CONTROLLERS):
            el = [r for r in runs if r["case"]["candidate"] == who and r["case"]["level"] == lv and r.get("common_tracking")]
            bottom = 0
            for p in parts:
                v = float(np.mean([r["common_tracking"]["physical_cost_s"][p] for r in el])) if el else 0
                ax.bar(i, v, bottom=bottom, color=COLOR[who], alpha=shade[p], width=.6, edgecolor="white", linewidth=1)
                data.append(dict(level=lv, controller=who, part=p, mean_cost_per_s=v, n_scored=len(el)))
                bottom += v
            ax.scatter(np.full(len(el), i + .38), [r["C"] for r in el], s=8, color="#444", zorder=3)
            ax.text(i, bottom, f"n={len(el)}", ha="center", va="bottom", fontsize=6.5)
        ax.set_xticks(range(3), [SHORT[w] for w in CONTROLLERS])
        ax.set_title(f"L{lv}: mean over scored runs, dots = per-run total")
    axes[0].set_ylabel("Common physical cost [1/s] (original Q/R/S)")
    handles = [plt.Rectangle((0, 0), 1, 1, color="#00866A", alpha=shade[p], label=p) for p in parts]
    axes[1].legend(handles=handles, fontsize=6.5, frameon=False)
    save(fig, out, "A3_physical_cost_decomposition", data)


def fig_solver(runs, out):
    fig, ax = plt.subplots(figsize=(5.2, 3.2))
    data = []
    for who in CONTROLLERS:
        ts = []
        for r in runs:
            if r["case"]["candidate"] != who:
                continue
            ticks = json.loads((ROOT / r["path"] / "ticks.json").read_text())
            ts += [t["qp_elapsed_s"] * 1e3 for t in ticks if t.get("qp_elapsed_s") is not None and t.get("active") == "deepc"]
        ts = np.sort(ts)
        if not len(ts):
            continue
        ax.step(ts, np.arange(1, len(ts) + 1) / len(ts), where="post", color=COLOR[who], ls="--", lw=1.5,
                label=f"{LABEL[who]} · {len(ts)} ticks, max {ts[-1]:.2f} ms")
        over = int((ts > BUDGET_MS).sum())
        data.append(dict(controller=who, ticks=len(ts), p50=np.percentile(ts, 50), p95=np.percentile(ts, 95),
                         p99=np.percentile(ts, 99), max=ts[-1], over_12ms=over))
    ax.axvline(BUDGET_MS, color="#C0392B", lw=1, label="12 ms acceptance budget")
    ax.set_xscale("log"); ax.set_xlabel("DeePC QP time per tick [ms] (all 24 runs each)"); ax.set_ylabel("ECDF")
    ax.legend(fontsize=6.5, frameon=False, loc="lower right")
    save(fig, out, "A4_solver_timing_ecdf", data)


def fig_noise(runs, out):
    """Per-scene L1/L0 ratio (mean of eligible L1 seeds over the single L0 run); no cross-scene pooling."""
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0), sharey=True)
    data = []
    for ax, (m, lab) in zip(axes, [("P", "3D truth RMSE: L1 mean / L0"), ("C", "Physical cost: L1 mean / L0")]):
        for i, who in enumerate(CONTROLLERS):
            for j, sc in enumerate(SCENES):
                l0 = [r[m] for r in runs if r["case"]["candidate"] == who and r["case"]["scene"] == sc
                      and r["case"]["level"] == 0 and r.get(m) is not None]
                l1 = [r[m] for r in runs if r["case"]["candidate"] == who and r["case"]["scene"] == sc
                      and r["case"]["level"] == 1 and r.get(m) is not None]
                if not l0 or not l1:
                    continue
                ratio = float(np.mean(l1)) / l0[0]
                ax.scatter(j + (i - 1) * .22, ratio, marker=MARKER[who], s=26, color=COLOR[who], zorder=3,
                           label=SHORT[who] if j == 0 else None)
                data.append(dict(metric=m, controller=who, scene=sc, L0=l0[0], L1_mean=float(np.mean(l1)),
                                 n_L1=len(l1), ratio=ratio))
        ax.axhline(1, color="#555", lw=1)
        ax.set_yscale("log"); ax.set_title(lab, fontsize=8)
        ax.set_xticks(range(len(SCENES)), SCENES, rotation=30, fontsize=7)
    axes[0].set_ylabel("ratio (1 = no change from L0)")
    axes[0].legend(fontsize=6.5, frameon=False)
    fig.suptitle("Sensitivity to host iid measurement noise, per scene (L0 n=1, L1 n=3 seeds; A circle6 n=2)", fontsize=8)
    save(fig, out, "A5_noise_level", data)


def fig_internals(runs, out):
    keys = [("regularization_cost", "g penalty λ_g‖g‖²/2"), ("initialization_cost", "init penalty β‖e‖²/2"),
            ("g_norm", "‖g‖"), ("init_slack_rms", "init residual RMS (normalized)")]
    fig, axes = plt.subplots(1, 4, figsize=(7.2, 2.8))
    data = []
    for ax, (k, lab) in zip(axes, keys):
        box, pos, cols = [], [], []
        for i, who in enumerate(CONTROLLERS):
            for j, lv in enumerate((0, 1)):
                v = []
                for r in runs:
                    if r["case"]["candidate"] == who and r["case"]["level"] == lv:
                        v += [t[k] for t in json.loads((ROOT / r["path"] / "ticks.json").read_text())
                              if t.get("active") == "deepc" and t.get(k) is not None]
                v = np.asarray(v)
                v = v[v > 0] if k != "init_slack_rms" else v
                box.append(v if len(v) else [np.nan]); pos.append(i * 2.4 + j); cols.append((who, lv))
                if len(v):
                    data.append(dict(quantity=k, controller=who, level=lv, ticks=len(v),
                                     p50=float(np.percentile(v, 50)), p95=float(np.percentile(v, 95)), max=float(v.max())))
        b = ax.boxplot(box, positions=pos, widths=.8, showfliers=False, patch_artist=True, medianprops=dict(color="#222222"))
        for patch, (who, lv) in zip(b["boxes"], cols):
            patch.set(facecolor=COLOR[who] if lv else "white", edgecolor=COLOR[who])
        ax.set_yscale("log"); ax.set_title(lab, fontsize=7)
        ax.set_xticks([i * 2.4 + .5 for i in range(3)], [SHORT[w] for w in CONTROLLERS], fontsize=6.5)
    fig.suptitle("DeePC internal terms per tick (open box L0, filled L1; whiskers 1.5 IQR, outliers hidden, values in data CSV)", fontsize=7.5)
    save(fig, out, "A6_deepc_internal_terms", data)


def fig_timeseries(runs, out, scene, rep_seed):
    sel = {r["case"]["candidate"]: r for r in runs if r["case"]["scene"] == scene and r["case"]["seed"] == rep_seed}
    circle = scene.startswith("circle")
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 6.4), sharex=not circle)
    data = []
    if circle:
        axes[0].remove(); axes[0] = fig.add_subplot(3, 1, 1)
    for who in CONTROLLERS:
        r = sel.get(who)
        if not r or not (ROOT / r["path"]).exists():
            continue
        try:
            tj = trajectory(r["path"])
        except (StopIteration, OSError, ValueError, KeyError):
            continue
        t, tr, ref, u = tj["t"], tj["truth"], tj["ref"], tj["u"]
        tr0 = tr - [*tr[0, :3] - ref[0, :3], 0]  # display only: align truth origin to takeover reference
        if circle:
            axes[0].plot(tr0[:, 0] * 1e3, tr0[:, 1] * 1e3, color=COLOR[who], ls="--", lw=1, label=f"{SHORT[who]} truth")
            axes[1].plot(t, (tr0[:, 0] - ref[:, 0]) * 1e3, color=COLOR[who], ls="--", lw=1)
        else:
            j = 3 if scene.startswith("yaw") else 2
            k = 1e3
            axes[0].plot(t, tr0[:, j] * k, color=COLOR[who], ls="--", lw=1, label=f"{SHORT[who]} truth")
            axes[1].plot(t, (tr0[:, 0]) * 1e3, color=COLOR[who], ls="--", lw=1)
        axes[2].plot(t, u[:, 2 if scene.startswith("yaw") else 3 if scene.startswith("z") else 0],
                     color=COLOR[who], ls="--", lw=1, label=SHORT[who])
        axes[2].axvline(t[tj["bridge"]] if tj["bridge"] < len(t) else t[-1], color=COLOR[who], lw=.5, alpha=.6)
        for n in range(len(t)):
            data.append(dict(controller=who, seed=rep_seed, t_s=t[n], ref_x=ref[n, 0], ref_y=ref[n, 1], ref_z=ref[n, 2],
                             ref_yaw=ref[n, 3], truth_x=tr0[n, 0], truth_y=tr0[n, 1], truth_z=tr0[n, 2], truth_yaw=tr0[n, 3],
                             u_roll=u[n, 0], u_pitch=u[n, 1], u_yaw=u[n, 2], u_thrust=u[n, 3], eligible=r["eligible"]))
    ref_r = next((sel[w] for w in CONTROLLERS if w in sel), None)
    if ref_r:
        tj = trajectory(ref_r["path"])
        if circle:
            axes[0].plot(tj["ref"][:, 0] * 1e3, tj["ref"][:, 1] * 1e3, color="black", lw=1, label="reference")
            axes[0].set_aspect("equal"); axes[0].set_xlabel("x [mm]"); axes[0].set_ylabel("y [mm]")
            axes[1].set_ylabel("x error [mm]")
        else:
            j = 3 if scene.startswith("yaw") else 2
            axes[0].plot(tj["t"], tj["ref"][:, j] * 1e3, color="black", lw=1, label="reference")
            axes[0].set_ylabel("yaw [mrad]" if j == 3 else "z [mm]")
            axes[1].set_ylabel("x (cross-axis) [mm]")
    axes[0].legend(fontsize=6.5, frameon=False)
    axes[2].set_ylabel({"yaw": "yaw-rate cmd [rad/s]", "z4-": "thrust cmd [normalized]"}.get(scene[:3], "roll-rate cmd [rad/s]")
                       if not scene.startswith("z") else "thrust cmd [normalized]")
    axes[2].set_xlabel("time since first body-rate command [s] (thin vertical = bridge end)")
    fig.suptitle(f"{scene}, L1 seed {rep_seed}: reference vs MuJoCo truth and issued input (DeePC configs)", fontsize=8)
    save(fig, out, f"A7_timeseries_{scene}", data)


# ---------------------------------------------------------------- report
def table(rows, header):
    s = "| " + " | ".join(header) + " |\n|" + "---|" * len(header) + "\n"
    return s + "".join("| " + " | ".join(str(x) for x in r) + " |\n" for r in rows)


def fmt(x, k=1, d=3):
    return "—" if x is None else f"{x * k:.{d}f}"


def main():
    stage, out = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    raw, runs = load(stage)
    for r in runs:
        r["steady_yaw"] = steady_yaw(r) if r.get("eligible") else None
    gates, rec = evaluate(runs)
    fig_paired(runs, out); fig_scenes(runs, out); fig_cost(runs, out); fig_solver(runs, out)
    fig_noise(runs, out); fig_internals(runs, out)
    fig_timeseries(runs, out, "yaw7-pos", 54121); fig_timeseries(runs, out, "circle6", 54161)
    per_run = []
    for r in sorted(runs, key=lambda r: r["path"]):
        ct = r.get("common_tracking") or {}
        pc = ct.get("physical_cost_s", {})
        per_run.append(dict(run=r["path"], name=r["case"]["name"], scene=r["case"]["scene"], level=r["case"]["level"],
                            seed=r["case"]["seed"], controller=r["case"]["candidate"], status=r["status"],
                            eligible=r["eligible"], reason=reason(r), attempts=len(r["attempts"]),
                            P_mm=fmt(r.get("P"), 1e3), x_mm=fmt((ct.get("truth_rmse_xyz_yaw") or [None])[0], 1e3),
                            y_mm=fmt((ct.get("truth_rmse_xyz_yaw") or [None, None])[1], 1e3),
                            z_mm=fmt((ct.get("truth_rmse_xyz_yaw") or [None] * 3)[2], 1e3),
                            yaw_mrad=fmt(r.get("yaw"), 1e3), cost_total=fmt(pc.get("total"), 1, 5),
                            cost_tracking=fmt(pc.get("tracking"), 1, 5), cost_effort=fmt(pc.get("effort"), 1, 5),
                            cost_increment=fmt(pc.get("increment"), 1, 5),
                            solver_p99_ms=fmt((ct.get("solver_ms") or [None] * 3)[2], 1, 2),
                            solver_max_ms=fmt((ct.get("solver_ms") or [None] * 4)[3], 1, 2),
                            runner_work_max_ms=fmt((ct.get("runner_work_ms") or [None] * 4)[3], 1, 2),
                            deadline_misses=r.get("deadline_misses"), fallback=r.get("fallback"),
                            qp_over_12ms=len(r.get("qp_overruns", [])), soft_ticks=ct.get("soft_activation_ticks"),
                            bridge_s=fmt((r.get("bridge") or {}).get("duration_s"), 1, 2),
                            steady_yaw_mrad=fmt(r.get("steady_yaw"), 1e3, 2)))
    with open(out / "stage_a_runs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(per_run[0])); w.writeheader(); w.writerows(per_run)
    (out / "stage_a_gates.json").write_text(json.dumps(dict(gates=gates, recommendation=rec), indent=2, default=float))
    (out / "stage_a_attempts.json").write_text(json.dumps(
        [dict(path=r["path"], name=r["case"]["name"], status=r["status"], attempt=r.get("attempt"),
              infrastructure_failure=r.get("infrastructure_failure"), eligible=r.get("eligible")) for r in raw], indent=2))
    print(json.dumps(dict(recommendation=rec, eligible=f"{sum(r['eligible'] for r in runs)}/{len(runs)}",
                          attempts=len(raw)), indent=2))


if __name__ == "__main__":
    main()
