# Phase 4 CrazySim controller prototypes

Use Python dependencies from the Phase 2 CrazySim guide and keep
`OPENBLAS_NUM_THREADS=1` when comparing solver timing. The examples here
use isolated, sequential SITL instances. There is no physical-hardware
entry point for the new predictive options.

## Prototype configuration

DeePC reads training episodes from the local `log/phase3-v1` manifest.
It skips every validation and test episode before reading the NPZ files.
The model uses 25 future steps, 6 observed past steps, and window-local
position in a fixed yaw frame. It enforces past and future input equalities
in a reduced subspace with 16 retained output-side directions.
Past-output mismatch is soft. The coefficient penalty uses the 1e-3
development factor times the input equality's largest singular value,
squared. This normalization is specified here because the earlier offline
probe used the combined regressor norm. Physical Q/R/S and physical input
and state bounds are shared with M0. Position bounds are translated from
the fixed takeover box. The runner's world-frame hard envelope remains
independent of the changing anchor.

The QP first tries its unconstrained minimizer, then its input-box-only
minimizer. It accepts either only if it also satisfies the state box. It
otherwise runs the full output-slack QP. The unmodified 12 ms solve limit
and runner deadline remain in effect. Six actual send callbacks initialize
DeePC with an MPC bridge. A solver or history failure uses the existing HLC
recovery path when the raw observations satisfy the recovery envelope.

MPC stays takeover-relative. `--mpc-model m0` is the default.
`hybrid` and `joint-fir6` use training-fit coefficients from
`log/phase3-offline-coordinates/results.json`. The QP forecast augments nine
physical states with five issued input vectors and a constant affine
component. The first five sends are by M0, avoiding synthetic HLC history.
The applied thrust history uses reconstructed firmware encoding. Neither
controller receives MuJoCo truth.

## Reproduction

The checked run matrices are saved automatically with each original batch.
Use a fresh output directory and a free firmware/UDP port pair.

```bash
cd /home/chris/drone-project
export OPENBLAS_NUM_THREADS=1
.venv/bin/python -m tools.run_crazysim_batch log/phase4-deepc-hover/matrix.json log/my-deepc-hover --controller outer-loop-deepc --port 25550
.venv/bin/python -m tools.run_crazysim_batch log/phase4-deepc-steps/matrix.json log/my-deepc-steps --controller outer-loop-deepc --port 25550
.venv/bin/python -m tools.run_crazysim_batch log/phase4-deepc-circle/matrix.json log/my-deepc-circle --controller outer-loop-deepc --port 25550
.venv/bin/python -m tools.run_crazysim_batch log/phase4-deepc-fallback/matrix.json log/my-deepc-fallback --controller outer-loop-deepc --port 25550
.venv/bin/python -m tools.run_crazysim_batch log/phase4-mpc-models/matrix.json log/my-mpc-models --port 25650
```

The default batch mode is headless. Add `--gui` to view one supervised
simulation at a time. Existing independent run logs record resolved
controller parameters, sent commands, raw estimated state, MuJoCo truth,
solver and history status, and prediction snapshots.

## Host measurement noise

The prototype profile `iid-small` scales independent Gaussian deviations
of 1 mm position, 5 mm/s velocity, 1 mrad angles and 0.01 rad/s body rates.
`--noise-amplitude` is a multiplier, and `--noise-seed` defines the same
realization per tracking step across both controllers. All upstream
measurements, HLC behavior and safety checks use the original estimator
path. DeePC's own history uses the perturbed observation. Zero amplitude
passes the original object through exactly. The per-step diagnostics log
raw, applied delta, and used estimate. Raw truth remains independent.
The CrazySim native IMU/barometer noise switch is a distinct scenario and
is disabled for the Phase 4 smoke tests.

```bash
.venv/bin/python -m tools.run_crazysim_batch log/phase4-noise-mpc/matrix.json log/my-noise-mpc --port 25750
.venv/bin/python -m tools.run_crazysim_batch log/phase4-noise-deepc/matrix.json log/my-noise-deepc --controller outer-loop-deepc --port 25750
.venv/bin/python -m tools.phase4_evidence log/my-phase4-summary log/my-deepc-hover log/my-deepc-steps log/my-deepc-circle log/my-deepc-fallback log/my-mpc-models log/my-noise-mpc log/my-noise-deepc
```

The summary reports independent truth tracking and timing plus online
truth-vs-plan prediction error. That online quantity includes future
replanning, unlike held-out replay with matched future inputs.

## Phase 4 results and scope

Six nominal DeePC cases successfully completed HLC takeoff, six confirmed
MPC bridge packets, DeePC tracking, and HLC landing. The tested cases
were hover, +3 cm X/Y/Z, +5 degree yaw, and a previewed 0.04 m-radius,
12 s circle. An additional flight exercised controlled solver failure and
official landing. This injected exception validated fallback, not a native
OSQP failure mode. There were no nominal deadline misses, unexpected
fallbacks or measured hard envelope violations. The slow circle scored
4.24 mm truth XYZ RMSE and 0.54 ms run-level QP P99.

| MPC prototype | Truth XYZ RMSE, hover / X step / slow circle | QP P99, hover / X step / slow circle |
| --- | --- | --- |
| M0 | 7.90 / 16.97 / 4.53 mm | 1.41 / 2.05 / 0.92 ms |
| Hybrid | 8.58 / 17.62 / 5.51 mm | 2.72 / 2.81 / 1.65 ms |
| Joint FIR6 | 8.51 / 17.59 / 5.06 mm | 2.85 / 2.54 / 1.21 ms |

All nine MPC runs completed without fallback or a missed deadline. The
identified variants were initialized by five M0 sends. The better held-out
prediction of Joint FIR6 did not transfer to improved tracking in these
single-run comparisons. Retain M0 as the default and Joint FIR6 as the main
identified challenger for controlled, repeated comparisons. Hybrid remains
a structural and timing comparison point.

Both controllers completed zero-amplitude and nonzero-amplitude noise
hover. Each zero run logged 100 exact raw/used observation matches. The
paired nonzero runs logged 100 exactly matching indexed noise vectors.
DeePC normalized initialization slack P95 was 0.013 in the zero trial
and 1.219 with noise. This is a functional smoke test, not evidence for
choosing a robust regularization parameter.

Scaled input Hankel ranks at depth 40, 60, and 61 were 160, 240, and 244,
respectively, at the relative 1e-4 threshold. Depth 40 uses nominal order
9; the deeper checks cover the 29 dynamic-state / 30 affine-augmented FIR
representation. They do not identify the actual plant order.

An offline stress check under an artificial 3 mm X soft bound caused the
full state-slack QP to time out at the 12 ms budget. Neither bounds nor
deadlines were loosened for flight. The reduced QP and fallback are
adequate for the observed nominal cases, but sustained active output
constraints are a remaining solver gate before expanding the envelope.
There is no evidence from these small scenarios that v2 collection is
necessary. Broader velocity or combined yaw/position tracking,
robustness, and final dataset/model/cost freeze remain outside this result.

The original artifacts are under `log/phase4-*` with aggregate evidence
in `log/phase4-summary`. The regressions cover FIR recursion, the six-send
bridge and bounded outputs, and deterministic/zero noise. A future common
tuning stage should match Q/R/S, horizon, constraint and observation profiles
and repeat the nominal pairs before choosing controller-specific penalties
or an identified model. Keep GNSS degradation out of tuning.
