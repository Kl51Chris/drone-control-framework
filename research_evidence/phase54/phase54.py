"""Phase 5.4 registration, preflight and Stage A execution.

Subcommands (run from the repository root with PYTHONPATH=.):
  register  <dir>          write configs/ and registered.json (refuses to overwrite)
  lambdas   <dir>          construct Stage A DeePC controllers and check lambda_g
  stage-a   <dir> <out>    execute the registered Stage A matrix (72 runs)

The scorer in log/phase53/phase51_score.py is imported unchanged.
Stage B is registered only; this script has no Stage B execution path.
"""
import argparse
import json
import signal
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "log/phase53"))

BASE = dict(q=[120, 120, 200, 12, 12, 24, 6, 6, 10], r=[2, 2, 1, 12], s=[2, 2, 1, 12])
DEEPC = {
    "t6-ref": dict(past=6, regularization=0.001, initialization_weight=0.25),
    "t6-a": dict(past=6, regularization=0.01, initialization_weight=0.0625),
    "t6-b": dict(past=6, regularization=0.01, initialization_weight=0.125),
    "t12-ref": dict(past=12, regularization=0.001, initialization_weight=0.25),
}
MPC = ("joint-fir6", "m0")
EXPECTED_LAMBDA = {"t6-ref": 0.0133075422, "t6-a": 1.33075422, "t6-b": 1.33075422}
YAW7, YAW6 = 0.12217304763960307, 0.10471975511965977


def step(axis, amplitude):
    return ["--step-axis", axis, "--step-amplitude", repr(amplitude), "--step-hold", "2"]


def circle(radius):
    return ["--reference", "circular", "--circle-radius", repr(radius),
            "--circle-duration", "12", "--reference-mode", "preview"]


SCENES_A = [("hover", []), ("yaw7-pos", step("yaw", YAW7)), ("yaw7-neg", step("yaw", -YAW7)),
            ("z4-pos", step("z", 0.04)), ("z4-neg", step("z", -0.04)), ("circle6", circle(0.06))]
SCENES_B = [("hover", []), ("yaw6-pos", step("yaw", YAW6)), ("yaw6-neg", step("yaw", -YAW6)),
            ("z5-pos", step("z", 0.05)), ("z5-neg", step("z", -0.05)), ("circle5", circle(0.05))]


def case(stage, scene_no, scene, extra, level, seed, repeat, who, config_dir):
    mpc = who in MPC
    args = ["--horizon", "25", "--mpc-config", f"{config_dir}/{who}.json"]
    if mpc:
        args += ["--mpc-model", who]
    args += [*extra, "--measurement-noise", "none" if level == 0 else "iid-small",
             "--noise-amplitude", str(level), "--noise-seed", str(seed)]
    return dict(stage=stage, name=f"{scene}-{who}-L{level}-{seed}", scene=scene, scene_no=scene_no,
                level=level, seed=seed, repeat=repeat, candidate=who,
                controller="outer-loop-mpc" if mpc else "outer-loop-deepc", args=args)


def blocks(scenes, l0_seed, l1_prefix, l2=()):
    out = [(i, s, a, 0, l0_seed, 0) for i, (s, a) in enumerate(scenes, 1)]
    for rep in (1, 2, 3):
        out += [(i, s, a, 1, int(f"{l1_prefix}{i}{rep}"), rep) for i, (s, a) in enumerate(scenes, 1)]
    for name, seed in l2:
        i, (s, a) = next((i, x) for i, x in enumerate(scenes, 1) if x[0] == name)
        out.append((i, s, a, 2, seed, 1))
    return out


def matrix(stage, scenes, controllers, l0, prefix, l2, config_dir):
    rows = []
    for b, (i, s, a, level, seed, rep) in enumerate(blocks(scenes, l0, prefix, l2)):
        order = controllers[b % len(controllers):] + controllers[:b % len(controllers)]
        rows += [case(stage, i, s, a, level, seed, rep, who, config_dir) for who in order]
    return rows


def register(directory):
    directory.mkdir()
    configs = directory / "configs"
    configs.mkdir()
    for who in [*DEEPC, *MPC]:
        (configs / f"{who}.json").write_text(json.dumps({**BASE, **DEEPC.get(who, {})}, indent=2))
    cfg = str(configs.relative_to(ROOT))
    stage_a = matrix("A", SCENES_A, ["t6-ref", "t6-a", "t6-b"], 54100, "541", (), cfg)
    stage_b = matrix("B", SCENES_B, ["joint-fir6", "m0", "DEEPC_SELECTED", "t12-ref"], 54500, "545",
                     (("yaw6-neg", 54630), ("circle5", 54660)), cfg)
    assert len(stage_a) == 72 and len(stage_b) == 104
    (directory / "registered.json").write_text(json.dumps(dict(
        stage_a=stage_a, stage_b=stage_b,
        stage_b_note="NOT AUTHORIZED. DEEPC_SELECTED resolves to the user-approved Stage A choice.",
        evaluation=BASE), indent=2))
    print("registered", len(stage_a), "Stage A and", len(stage_b), "Stage B runs")


def lambdas(directory):
    from velocity_control_framework.controllers.outer_loop_deepc import (
        OuterLoopDeePCConfig, OuterLoopDeePCController)
    from velocity_control_framework.controllers.near_hover_model import NearHoverModelConfig
    out, ok = {}, True
    for who, expected in EXPECTED_LAMBDA.items():
        params = json.loads((directory / "configs" / f"{who}.json").read_text())
        controller = OuterLoopDeePCController(
            OuterLoopDeePCConfig(model=NearHoverModelConfig(), horizon=25, **params))
        value = float(controller._lambda)
        rel = abs(value - expected) / expected
        out[who] = dict(lambda_g=value, expected=expected, relative_error=rel, pass_=rel <= 1e-6)
        ok &= rel <= 1e-6
    (directory / "preflight-lambda.json").write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    sys.exit(0 if ok else 1)


def has_takeoff(run):
    for events in run.glob("*/events.csv"):
        if '"phase": "TAKEOFF"' in events.read_text():
            return True
    return False


def audit(run, c):
    from phase51_score import score
    from tools.shared_dataset import read_csv
    raw = json.loads((run / "experiment.json").read_text())
    result = dict(case=c, status=raw["status"], path=str(run.relative_to(ROOT)), takeoff=has_takeoff(run))
    try:
        result.update(score(run, BASE))
    except (OSError, ValueError, KeyError, IndexError, StopIteration) as exc:
        result["analysis_error"] = repr(exc)
    flight = next(run.glob("*/flight.csv"), None)
    overruns, ticks = [], []
    if flight is not None:
        for row in read_csv(flight):
            if not row["command_thrust_normalized"]:
                continue
            d = json.loads(row["controller_diagnostics_json"]).get("mpc", {})
            if d.get("qp_elapsed_s", 0.0) > 0.012:
                overruns.append(dict(step_index=int(row["step_index"]), qp_elapsed_s=d["qp_elapsed_s"]))
            ticks.append(dict(step_index=int(row["step_index"]), active=d.get("active_controller"),
                              **{k: d.get(k) for k in ("qp_elapsed_s", "regularization_cost",
                                 "initialization_cost", "g_norm", "init_slack_rms", "slack_cost",
                                 "soft_boundary_count", "active_input_count", "solver_path", "status")},
                              **{k: float(row[k]) for k in ("command_roll_rate_rad_s", "command_pitch_rate_rad_s",
                                 "command_yaw_rate_rad_s", "command_thrust_normalized")}))
    result["qp_overruns"] = overruns
    (run / "ticks.json").write_text(json.dumps(ticks))
    result["eligible"] = bool(result.get("common_tracking") and result["status"] == "completed"
                              and result.get("fallback", 1) == 0 and result.get("deadline_misses", 1) == 0
                              and result.get("hard_input_violation_steps", 1) == 0
                              and result.get("truth_envelope_violation_tracking_steps", 1) == 0
                              and not overruns)
    return result


def stage_a(directory, output, port):
    from tools.run_crazysim_batch import run_case
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    registered = json.loads((directory / "registered.json").read_text())["stage_a"]
    output.mkdir()
    (output / "matrix.json").write_text(json.dumps(registered, indent=2))
    sim, rows, retries = Path("/home/chris/CrazySim"), [], 0
    for i, c in enumerate(registered, 1):
        attempt = 0
        while True:
            run = output / (f"{i:03d}" if attempt == 0 else f"{i:03d}-retry{attempt}")
            run.mkdir()
            run_case(run, c, sim, False, port, c["controller"])
            result = audit(run, c)
            result["attempt"] = attempt
            rows.append(result)
            infra = not result["takeoff"]
            result["infrastructure_failure"] = infra
            print(f"{i:03d}{'' if not attempt else ' retry'} {c['name']} {result['status']} "
                  f"{'eligible' if result['eligible'] else 'INELIGIBLE'} "
                  f"{result.get('fallback_reason') or result.get('analysis_error') or ''}", flush=True)
            (output / "summary.json").write_text(json.dumps(rows, indent=2, default=float))
            if infra and attempt == 0:
                retries += 1
                if retries > 10:
                    print("STOP: more than 10 infrastructure retries", flush=True)
                    return
                attempt = 1
                continue
            break
    final = [r for r in rows if not (r["infrastructure_failure"] and r["attempt"] == 0 and
             any(q["case"] == r["case"] and q["attempt"] == 1 for q in rows))]
    print(f"stage A complete: {sum(r['eligible'] for r in final)}/{len(final)} eligible, "
          f"{retries} infrastructure retries", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=("register", "lambdas", "stage-a"))
    p.add_argument("directory", type=Path)
    p.add_argument("output", type=Path, nargs="?")
    p.add_argument("--port", type=int, default=26540)
    a = p.parse_args()
    if a.command == "register":
        register(a.directory.resolve())
    elif a.command == "lambdas":
        lambdas(a.directory.resolve())
    else:
        stage_a(a.directory.resolve(), a.output.resolve(), a.port)


if __name__ == "__main__":
    main()
