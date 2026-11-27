# Phase 5.2 shared measurement-noise protocol

## Development settings

Joint FIR6 is the identified MPC candidate, M0 is the physics-based
reference, and DeePC uses T_ini=6 or 12. Both run N=25 at 50 Hz with the
existing Q/R/S, physical constraints and 12 ms QP budget. DeePC builds its
predictor from phase3-v1 training only. Its development regularization
factor is 1e-3 and its retained latent dimension is 16.

## Injection and pairing

The existing host wrapper injects independent zero-mean Gaussian noise
into the outer controller's estimated observation. Position, velocity,
Euler attitude and reported body rates receive independent per-axis
draws. The 9-state controllers use position, velocity and Euler
attitude. The reported gyro channels are also perturbed and logged.
The observation timestamp is preserved; yaw is wrapped after addition.

| Level | Existing CLI profile / amplitude | Position σ, mm | Velocity σ, mm/s | Attitude σ, mrad | Body-rate σ, mrad/s |
| --- | --- | ---: | ---: | ---: | ---: |
| L0 | none / 0 | 0 | 0 | 0 | 0 |
| L1 | iid-small / 1 | 1 | 5 | 1 | 10 |
| L2 | iid-small / 2 | 2 | 10 | 2 | 20 |
| L3 | iid-small / 3 | 3 | 15 | 3 | 30 |

Noise index zero is the first body-rate update after the HLC settle gate.
It advances on each attempted controller update, including the history
bridge. Each draw is keyed by the scenario seed and index using NumPy
SeedSequence and PCG64. Paired runs share the indexed noise vector; their
raw estimates and vehicle trajectories can differ. DeePC stores the
perturbed observation and the actually issued commands. In its moving
position frame an earlier position perturbation is differenced against
the current anchor perturbation. This transformation can correlate
entries within a history even though the original draws are iid.

CrazySim's optional native IMU/barometer noise is separate and disabled
in this pilot. The normal SITL sensor packets and onboard estimator still
run. The host perturbation does not change the firmware estimator, HLC,
independent safety checks, or the MuJoCo scoring stream. These levels
probe outer-loop measurement sensitivity. Formal GNSS degradation will
need its own experiment design and remains outside the tuning set.

## Registered pilot

The pilot uses L0 and L2. L1 and L3 define additional diagnostic levels.
It first validates nominal T_ini=12 in hover, signed yaw/Z steps and a
previewed circle. It then pairs four controllers in L0 hover/circle and
in all six L2 scenarios. All flights use isolated firmware and simulator
instances. The reference, initial-condition protocol and seeds are fixed
in the local pilot matrices. The Phase 5.1 truth scoring windows, metrics
and safety criteria are reused. All attempts and any allowed retry are
recorded separately.

Run from the repository root with the Phase 5.2 artifacts present:

```bash
export OPENBLAS_NUM_THREADS=1 PYTHONPATH=. PYTHONDONTWRITEBYTECODE=1
.venv/bin/python log/phase52/run_plan.py log/phase52 log/phase52-replay --port 26261
.venv/bin/python log/phase52/audit_noise.py log/phase52-replay
.venv/bin/python log/phase52/summarize.py log/phase52-replay --plan log/phase52
```

Original registration, commands, raw logs and derived diagnostics live
in the local `log/phase52` directory. Use a fresh replay directory. The
summarizer reads the original plan manifests, so when summarizing a
replay pass the original plan directory with `--plan`.

## Phase 5.3 finite tuning configuration

DeePC's JSON controller configuration accepts `regularization` and
`initialization_weight` separately. The implemented coefficient penalty
is `0.5 * (regularization * sigma_max(H_u_scaled))**2 * ||g||²`.
The initialization penalty is `0.5 * initialization_weight * ||e_ini||²`,
where `e_ini` contains the retained normalized past-output residuals.
The nominal defaults remain 0.001 and 1.0. The physical Q/R/S and the
predicted output slack penalty are separate terms.

In an iid position channel, re-anchoring gives older past entries
`epsilon_old - epsilon_now`. Their variance is twice the input noise
variance, and their within-window pairwise correlation is 0.5 because
they share the anchor. The current position is identically zero and is
excluded from the retained past-output rows.

The local `log/phase53/plan.md` fixes the finite tuning budget, seeds
and eligibility criteria. The shared-weight experiment computes all
realized physical costs under the original evaluation Q/R/S while
changing controller-side slew weights. Phase 5.1 scoring windows, safety
checks and event definitions are reused.
