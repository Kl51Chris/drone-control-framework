# Phase 5.4 · Preregistered independent closed-loop confirmation

Registered 2026-11-27 (America/Chicago) before any Phase 5.4 flight.
User authorization: Stage A registration, preflight and execution. Stage B
is registered here but **not authorized for execution**. Parameter freeze,
GNSS experiments and any additional parameter search are not authorized.
The Experimental Integrity rules in doc/research-handover.md §8 apply.

No document named "Phase 5.4 Plan" with Stage A/B text exists in the
repository. The Stage A/B split below follows the user's written
authorization of 2026-11-27 and the handover §6 row for Phase 5.4.

## 1. Fixed conditions (both stages)

- Dataset phase3-v1 with its existing split. DeePC Hankel uses the 11 train
  episodes only. The development-validation and sealed test episodes are
  not opened by any Phase 5.4 script.
- N=25 (0.50 s), 50 Hz host loop, 12 ms solver acceptance budget.
- Controller and evaluation Q/R/S: Q=diag(120,120,200,12,12,24,6,6,10),
  R=diag(2,2,1,12), S=diag(2,2,1,12). Physical input limits, soft output
  box, CrazySim hard safety envelope, HLC lifecycle, takeover/bridge rules
  and reference policy (preview for known circles, no advance notice for
  steps) unchanged.
- Measurement noise: existing `iid-small` generator, L0 = amplitude 0
  (`--measurement-noise none`), L1 = amplitude 1, L2 = amplitude 2. One
  seed per (scene, level, repeat) block is shared by every controller in
  the block (paired noise vector by update index).
- Scorer: `log/phase53/phase51_score.py` and `tools/analyze_mpc_batch.py`
  imported unchanged. Scoring windows unchanged: hover 75 ticks from tick
  20, step 90 ticks from the first reference change, circle 560 ticks from
  tick 20. No scorer, acceptance test or threshold is modified.

## 2. Stage A · Closed-loop candidate screening (authorized)

Controllers (DeePC, T_ini=6, N=25, 16 latent directions):

| Label | rho | beta | Role |
|---|---:|---:|---|
| t6-ref | 0.001 | 0.25 | Current closed-loop reference |
| t6-a | 0.01 | 0.0625 | Phase 5.3.1 candidate A |
| t6-b | 0.01 | 0.125 | Phase 5.3.1 candidate B |

Scenes (no amplitude below was used in any tuning stage; Phase 5.1–5.3
used ±5° yaw, ±0.03 m Z and a 4 cm / 12 s circle):

| No. | Scene | Arguments |
|---:|---|---|
| 1 | hover | default fixed hover |
| 2 | yaw7-pos | `--step-axis yaw --step-amplitude 0.12217304763960307 --step-hold 2` |
| 3 | yaw7-neg | `--step-axis yaw --step-amplitude -0.12217304763960307 --step-hold 2` |
| 4 | z4-pos | `--step-axis z --step-amplitude 0.04 --step-hold 2` |
| 5 | z4-neg | `--step-axis z --step-amplitude -0.04 --step-hold 2` |
| 6 | circle6 | `--reference circular --circle-radius 0.06 --circle-duration 12 --reference-mode preview` |

Seeds (none previously used; prior seeds: 47, 52010–52015, 53010–53031,
653100–653103, excitation 100–3014):

- L0: seed 54100 for all scenes (label only; noise amplitude is zero).
- L1: seed `541{scene}{repeat}`, repeat 1–3, i.e. 54111…54163.

Matrix: 3 controllers × 6 scenes × (1 L0 + 3 L1) = **72 runs**. Block order
is L0, L1 repeat 1, repeat 2, repeat 3; scenes 1–6 within each; controller
order inside block *i* (0-based) is [t6-ref, t6-a, t6-b] rotated left by
*i* mod 3.

## 3. Run eligibility

A run is eligible only if all hold:

1. batch status `completed` and the scorer returns `common_tracking`
   (full fixed scoring window, no `scoring_ineligible`);
2. no HLC fallback (`fallback == 0`);
3. **no runner deadline miss** (`deadline_misses == 0`, counted over the
   whole flight from `control_deadline_missed` events);
4. no hard input violation and no truth-envelope violation;
5. no QP with `qp_elapsed_s > 0.012` on any host-issued tick.

Ineligible runs stay in every denominator with their status, partial
trajectory and reason. They are never imputed as zero or as success.

## 4. Metrics

- Primary P: 3D truth position RMSE over the window,
  √(rx² + ry² + rz²) from `truth_rmse_xyz_yaw` (m; reported in mm).
- Yaw: truth yaw RMSE (rad; reported in mrad).
- C: common physical cost per second, total and tracking/effort/increment.
- Diagnostics (not gated): solver and runner timing percentiles, bridge
  duration, soft-output and input activation ticks, per-tick
  regularization cost, initialization cost, g norm, initialization residual
  (`init_slack_rms`), actually issued inputs, signed steady yaw error (mean
  truth yaw error over the last 25 ticks of the step window).

Absolute values and paired absolute differences (candidate − reference) are
reported for every metric alongside ratios.

## 5. Near-zero and zero denominators

Floors F fixed from Phase 5.3 magnitudes before any Phase 5.4 data:
F_P = 0.5 mm, F_yaw = 1.0 mrad, F_C = 1.0e-3 s⁻¹ (Phase 5.3 minima were
≈2.7 mm, ≈0.4 mrad, ≈2.2e-3 s⁻¹).

- A pair is *ratio-eligible* when the reference value ≥ F and both values
  are finite. Ratios use only ratio-eligible pairs.
- A non-ratio-eligible pair (including zero denominators) is judged by
  absolute difference; for a "not worse" gate it must satisfy
  candidate ≤ reference + F, otherwise that gate fails.
- If fewer than half of the pairs in a gate are ratio-eligible, the
  geometric-mean criterion is replaced by the ratio of arithmetic means
  over all pairs; if the mean reference is itself < F, a "not worse" gate
  uses mean(candidate) ≤ mean(reference) + F and the improvement gate G2a
  fails (improvement cannot be demonstrated).
- Win counts use the sign of the absolute difference; exact ties are not
  wins.

## 6. Candidate replacement gates (Stage A)

For X ∈ {t6-a, t6-b}, paired with t6-ref on identical (scene, level, seed):

- G1 feasibility: all 24 L0+L1 runs of X eligible.
- G2 L1 tracking (18 pairs): (a) geometric mean of P ratios ≤ 0.90;
  (b) X wins ≥ 12/18 pairs; (c) scene geometric mean ratio ≤ 1.0 in
  ≥ 5/6 scenes (a scene with no ratio-eligible pair uses mean difference ≤ 0).
- G3 L1 cost: geometric mean of C ratios ≤ 1.10.
- G4 L1 yaw: geometric mean of yaw-RMSE ratios ≤ 1.10.
- G5 L0 no degradation (6 pairs): P and C geometric mean ratios ≤ 1.10 each.

Decision: if t6-ref has any ineligible run, no automatic recommendation is
made; results go to the user. Otherwise rank passing candidates by G2a
ratio; if the top two differ by < 0.02, prefer the larger β (t6-b). If
none passes, recommend keeping t6-ref. The 10% bands are development
selection criteria, not statistical significance. The Stage A outcome is a
recommendation only; no freeze.

## 7. Stage B · Independent confirmation (registered, NOT executed)

Controllers: Joint FIR6 (MPC), M0 (MPC), the DeePC configuration
recommended by Stage A *and approved by the user* (t6-ref if none passes),
t12-ref (T_ini=12, ρ=0.001, β=0.25). No ranking or tuning in Stage B.

New reference parameters not used in Stage A or any earlier tuning, all
inside the existing scoring windows and safety envelope:

| No. | Scene | Arguments |
|---:|---|---|
| 1 | hover | default fixed hover |
| 2 | yaw6-pos | step yaw +0.10471975511965977 rad, hold 2 s |
| 3 | yaw6-neg | step yaw −0.10471975511965977 rad, hold 2 s |
| 4 | z5-pos | step z +0.05 m, hold 2 s |
| 5 | z5-neg | step z −0.05 m, hold 2 s |
| 6 | circle5 | circular, radius 0.05 m, 12 s, preview |

The 12 s circle keeps the 560-tick window inside the reference; a longer
circle would not be covered by the unchanged scorer, so it is not used.

Seeds (fresh, disjoint from Stage A): L0 54500; L1 `545{scene}{repeat}`
(54511…54563); L2 yaw6-neg 54630, circle5 54660.
Matrix: 4 × (6 L0 + 18 L1 + 2 L2) = **104 runs**. L2 is pressure evidence
only and does not participate in any parameter choice. Stage B results are
not used for further tuning. Joint FIR6 signed steady yaw error is reported
per sign; |error| > 20% of the step amplitude in ≥ 2/3 seeds of a sign is
recorded as an unresolved limitation (no re-identification).

## 8. Preflight (stop on any failure)

1. Record `git status`, HEAD, Python/NumPy/SciPy/OSQP versions.
2. `research_evidence/verify_archive.py` passes.
3. `OPENBLAS_NUM_THREADS=1 pytest velocity_control_framework/tests calibration/tests`
   passes; record count (Phase 5.3 record: 429).
4. Construct each Stage A DeePC controller from its registered JSON and
   record λ_g. t6-ref must match 0.0133075422 and t6-a/t6-b must match
   1.33075422 to relative 1e-6.
5. Inspection-only resolve of one run per scene confirms arguments.

## 9. Retries and stopping

Only an infrastructure failure may be retried: a run with no
`phase_changed TAKEOFF` event (simulator/firmware/port start failure).
At most one retry per run with the same seed and conditions; the first
attempt is kept and labeled. More than 10 infrastructure retries stops the
stage. Control, solver, constraint or safety failures are never retried.
No additional seeds, scenes or parameter values.

## 10. Reporting

Visualization-first report (doc/research-reporting-protocol.md) as
Markdown plus a self-contained local HTML file, with PNG (300 DPI), SVG,
figure CSV/JSON and plotting script per figure. Every attempt, failure and
per-tick DeePC penalty/residual/input trace is retained. Evidence indices
are updated after Stage A.
