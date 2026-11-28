# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Modular Crazyflie control framework for comparing outer-loop controllers (PID, LQR, MPC, DeePC) on real hardware and in CrazySim (MuJoCo SITL). The PC sends **body-rate + normalized collective thrust** commands. The firmware keeps the inner body-rate PID and the motor mixer.

`agent.md` holds the full project rules. Its main points are summarized below. Note that it says `docs/`, but the directory is actually `doc/`.

## Commands

Always use the venv interpreter. There is no `pyproject.toml` or `setup.py`. Dependencies come from `requirements-ubuntu-lock.txt` (or `requirements-wsl-lock.txt` on WSL).

```bash
# Main test suites (do not run bare `pytest` at the repo root: MuJoCo/ROS assets break collection)
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest velocity_control_framework/tests calibration/tests

# Single test
.venv/bin/python -m pytest velocity_control_framework/tests/test_outer_loop_mpc.py::test_name -q

# Inspection only: resolves and prints the config, no connection (safe)
.venv/bin/python -m experiments.run_flight --controller outer-loop-pid --configuration stage5-hardware-candidate --reference fixed-hover

# CrazySim flight (needs running SITL firmware + MuJoCo; see doc/crazysim-hlc-testing.md)
.venv/bin/python -m experiments.run_flight --backend crazysim --controller outer-loop-mpc --flight-lifecycle hlc --execute

# Batch SITL runs (Phase 4) and log analysis
.venv/bin/python -m tools.run_crazysim_batch <matrix.json> <out_dir> --controller outer-loop-deepc --port 25550
.venv/bin/python -m tools.analyze_flight_log <run-dir>
.venv/bin/python -m tools.analyze_predictive_flight <run-dir>
```

MPC and DeePC enforce a ~12 ms QP solve time limit, and some tests depend on it. Without `OPENBLAS_NUM_THREADS=1`, `test_phase4_controllers.py` can fail with `MPCSolveError: run time limit reached`. That failure comes from timing on the machine and does not by itself mean the logic is broken.

## Architecture

The data flow is `StateProvider → StateEstimator → DroneState` + `Reference` → `Controller.update(state, ref, dt) → BodyRateThrustCommand` → `CommandAdapter`.

- `velocity_control_framework/interfaces/` holds the public contracts: `DroneState`, `EstimatedState`/`MeasuredState`, `Reference`, `Controller`, `BodyRateThrustCommand`, `StateEstimator`. All of them use SI units (radians, rad/s, m, s) and thrust normalized to [0, 1], and they reject non-finite values. Changing any of these is an architectural change: update every user, the tests, `doc/architecture.md` and `doc/control-design.md` together.
- `controllers/` contains `outer_loop_pid`, `outer_loop_lqr`, `outer_loop_mpc` (OSQP, 9-state near-hover model, 20 ms step, default horizon 25) and `outer_loop_deepc` (Hankel built from the `log/phase3-v1` training split). `near_hover_model.py` / `identified_hover_model.py` provide the prediction models for MPC (`--mpc-model m0|hybrid|joint-fir6`).
- `backends/` is the only place that may touch hardware or simulator APIs: Crazyflie radio (`crazyflie_*`), CrazySim UDP (`crazysim_*`), and the official HighLevelCommander takeoff/landing handover (`crazysim_hlc_lifecycle.py`). Unit conversion (rad/s → deg/s, thrust → firmware encoding) happens here.
- `runner/universal_hardware_runner.py` owns timing and lifecycle: start state → arm → fixed-rate loop (~50 Hz; missed deadlines are dropped, not caught up) → in `finally`, stop commands, disarm, stop state, finalize the logger. It delegates reference and command generation to a `FlightProgram`.
- `experiments/run_flight.py` is the single production CLI. It assembles controller + configuration + reference + backend into a `ProductionFlightProgram` and owns the safety envelope, landing checks and `FALLBACK_LANDING`. `--execute` is required to fly. Without it the command is inspection only.
- With `--flight-lifecycle hlc` (CrazySim), the firmware's HLC handles takeoff and landing, and the selected controller only owns the tracking segment. HLC phases log blank body-rate fields.
- `calibration/` is a separate system-identification and staged-validation toolkit with its own tests. Equivalence tests check that the production PID matches the calibration PID.
- `tools/` contains diagnostics, SITL batch runners, dataset builders (`shared_dataset.py`, `phase3_matrix.py`) and analyzers. They are outside the control path.
- `legacy/` is historical only. Never import it from active code and do not modify it unless asked.
- Flight runs write to `log/<timestamp>_<reference>_<controller>_<config>_.../` (`metadata.json`, `flight.csv`, `events.csv`, `predictions.jsonl`). This directory is gitignored but also holds datasets and matrices that the DeePC and Phase 3/4 tooling read.

## Rules from agent.md

- Controllers must never call Crazyflie or cflib APIs, open connections, or read raw log variables. References must not contain controller-specific logic. Experiments stay thin, and reusable logic moves into the framework.
- Hardware safety: never run `--execute` against real hardware, arm, take off, or raise thrust/rate/attitude limits or change e-stop behavior unless explicitly asked. CrazySim runs are the safe place for flight testing.
- Never claim flight validation from software tests. Hardware landing currently ends in `LANDING_FAILED` (estimator-qualified landing criteria not met).
- Update the relevant `doc/*.md` in the same change when interfaces, units, signs or control behavior change.
- Research protocol (`doc/shared-dataset.md`): MPC identification and DeePC must use the same train/validation/test split. MuJoCo ground truth is for offline scoring only and must never be fed back to a controller online.

## Git workflow

- Do not stage, commit or push automatically when a task finishes. Leave all modifications in the working tree so the user can review them in VS Code and organize commits themselves.
- Read-only Git operations (`status`, `diff`, `log`, `show`, `blame`, etc.) are always allowed.
- Any operation that changes the index, commit history, branch state or remote (`add`, `commit`, `push`, `reset`, `rebase`, `merge`, `checkout`/`switch` of branches, `stash`, `tag`, etc.) requires the user's explicit approval for that specific operation.
- If a commit is scientifically necessary (e.g. freezing a preregistration before running experiments), explain why and request approval before committing. Keep such commits separate from result commits.
- This policy applies to edits of this file too: do not commit CLAUDE.md changes without approval.
