# Shared-Dataset Same-Cost roadmap

1. Phase 3 produces a versioned offline near-hover body-rate dataset. MPC
   system identification and DeePC Hankel construction use the same training
   episodes, validation episodes and untouched final test split. Historical
   development runs and excitation pilots stay outside that split.
2. Freeze a common reference policy, Q/R/S, horizon, constraints, sampling
   rate and available measurements. Both controllers can optimize their own
   internal parameters using a comparable budget. Tune on shared nominal and
   stochastic-noise environments. GNSS degradation is excluded from tuning.
3. Freeze the model, dataset version and controller configurations before the
   formal test. Evaluate on common conditions, including any separately
   defined degradation tests. MuJoCo ground truth is an offline scoring
   channel and is unavailable as online controller feedback.

The existing MPC is a development baseline. Phase 3 does not tune final
weights or implement online DeePC.

## Existing dataset audit

Body-rate-only samples from the final Phase 1/2 runs were screened using a
depth-44 input Hankel with no cross-run columns, thrust centered at the
nominal equilibrium, and a relative singular-value threshold of 1e-4.
The 176-row results below use physical units before channel scaling.

| Source | Data and coverage | Development use | Shared admission |
| --- | --- | --- | --- |
| Phase 1 pulses | Eight single-axis positive/negative episodes. About 42 body-rate samples per run. Host timing and low-level commands are present. Independent MuJoCo truth was not logged. | Command sign, delay, physical-model checks | Exclude from the formal split |
| Phase 1 hover | One 100-sample episode. Rank 44/176. Almost all activity is in thrust | Hover offset and transient checks | Exclude |
| Phase 2 signed steps | 24 runs, 2,880 samples, rank 175/176. Step-dominated spectrum; one target axis per run. P99 tick deviation 0.36 ms | Model and direction sanity checks with truth | Exclude. Used in baseline evaluation |
| Phase 2 reference comparison circles | Six runs, 4,500 samples, rank 159/176. Yaw standard deviation about 0.00013 rad/s. P99 tick deviation 0.33 ms | Low-frequency tracking and coordinate checks | Exclude. Used to choose preview |
| Phase 2 horizon sweep | 18 runs, 7,830 samples, rank 153/176. P99 tick deviation 0.27 ms. Mixed circles and X steps | Development prediction and timing | Exclude. Used for horizon selection |

Phase 2 has independent raw truth and controller logs. Around-bound predictive
input activation appeared on 90 nominal tracking steps, and on zero steps in
the reference comparison and the horizon sweep summaries. The activation
metric uses the solver proximity tolerance; it should not be interpreted as
hardware saturation. High numerical rank alone does not establish independent
wideband excitation. None of the above is an independent final test.

## Collection protocol

Use the existing CrazySim batch runner and HLC lifecycle. Each case starts
independent simulator and firmware processes. Seed controls the injected
sequence. The simulator uses the same default nominal environment and the
existing 9.5 s timeout. MPC stabilizes a 2.4 s hover. The pilot injects after
0.2 s for up to 96 ticks, downstream of MPC and upstream of the existing
command bounds. It records baseline, injected increment, clipping and final
host-issued command. MPC uses the issued command in the next slew objective.

The four peak injection amplitudes are 0.045 rad/s for each rate and 0.009
normalized thrust. Multisine is useful for attributing response to known tones
and observing feedback at those frequencies. Its short episodes can leave a
weak Hankel tail. Independent binary streams passed through a seven-tap FIR
spread excitation across more frequency bins without increasing peak input.
The filter's cutoff is a design parameter, not a strict spectral truncation.
Closed-loop feedback can still lower excitation relative to the total signal.
The tools report both sent-minus-baseline and injection-to-total correlation.

PID/LQR are retained as alternate development controllers. Nominal MPC proved
adequate for these short acquisitions, so no controller retuning is needed
to collect the first operating point. Sequential SITL reset is preferred
because clock jitter and I/O coverage are part of the admission checks.

## Versioned schema

The raw case directory holds `experiment.json`, truth, flight/state/event
logs, and simulator output. The processed directory contains
`manifest.json`, per-episode NPZ, training Hankel NPZ, quality metrics and
figures. The manifest contains run IDs, experiment seeds, parameters,
versions and the split. The processed row stores host `tick`, observation
timestamp, send-completion timestamp, `dt`, validity and saturation. It also
stores host command, reconstructed raw thrust, baseline, injection, measured
body rates, nine-channel estimator state and independent truth.

`u` is the final host-issued rate command and quantization-adjusted thrust
minus 0.47681863. Firmware receipt is unobserved. `y` and truth are in the
same fixed initial-yaw frame, with position relative to the initial estimated
pose, velocity rotated into that frame, legacy-sign pitch and wrapped
relative yaw. The raw estimator state remains unchanged. Truth is aligned
to the observation clock by interpolation. Host ticks are used as the
nominal 20 ms index after jitter screening; the untouched clocks remain in
the episode to support finer delay analysis.

Hankel columns use matched (u[k],y[k+1]) pairs. Per-episode blocks are
concatenated horizontally and never include HLC/landing or adjacent episodes.
The PE diagnostics evaluate orders 40/44/55/59 for N=25/40, T_ini=6/10
and a model-order candidate of nine. Validation uses the same held-out
future issued inputs for physics, regularized behavioral prediction and
persistence. Both predictors see estimated state only; separate scoring
against estimator output and truth reveals the distinction.

## Candidate admission gates

- Complete episodes, no fallback or missed control deadline, max tick
  deviation <=5 ms, P99 deviation <=2 ms, observation-to-send age <=40 ms.
  Truth interpolation gap <=30 ms. All admitted samples finite.
- Near-hover truth stays within 0.15 m of initial pose and 0.12 rad in the
  three attitude coordinates. Clip fraction <=1%.
- For the selected horizon and past window, require numerical PE rank at
  relative 1e-4, inspect the full spectrum and the normalized input
  correlation. Compare actual-input spectra and rank against smaller sets.
- Require held-out data-driven prediction to improve on persistence where
  outputs are excited and inspect failure by channel/horizon. Evaluate
  stability across independent validation episodes before freezing.
  The test split remains sealed during this stage.

## Reproduction

```bash
cd /home/chris/drone-project
export OPENBLAS_NUM_THREADS=1
.venv/bin/python -m tools.phase3_matrix pilot > /tmp/phase3-pilot.json
.venv/bin/python -m tools.run_crazysim_batch /tmp/phase3-pilot.json log/my-phase3-pilot --port 25150
.venv/bin/python -m tools.shared_dataset log/my-phase3-pilot
```

The matrix stages `wide` and `filtered` reproduce the pilot variants.
After the pilot passes, generate `dataset`, run it in its own directory
and process it with:

```bash
.venv/bin/python -m tools.phase3_matrix dataset > /tmp/phase3-collection.json
.venv/bin/python -m tools.run_crazysim_batch /tmp/phase3-collection.json log/my-phase3-raw --port 25450
.venv/bin/python -m tools.shared_dataset log/my-phase3-raw --output log/my-phase3-v1 --version phase3-v1 --t-ini 6
```

PSD, excitation versus applied input, coverage, Hankel spectrum and
validation curves are written alongside the processed manifest.

## Phase 3 v1 acceptance report

Raw captures are in `log/phase3-shared-raw`. Processed artifacts are under
`log/phase3-v1`. Both paths are local to this checkout. All eighteen
flights completed with zero simulator-log drops. Episode 009 (seed 3008)
had an observation-to-send age of 41.73 ms at one step and was rejected
against the unchanged 40 ms gate. Its original data remain available.
The admitted split is **11 train / 3 validation / 3 test**, 120 samples per
episode (2,040 total). The three test episodes were screened for integrity
and not used in prediction scoring or parameter selection.

| Depth | Effective rank | Training columns | Relative minimum singular value |
| --- | --- | --- | --- |
| 40 | 160/160 | 891 | 0.00816 |
| 44 | 176/176 | 847 | 0.00755 |
| 55 | 220/220 | 726 | 0.00583 |
| 59 | 236/236 | 682 | 0.00550 |

Training P99 tick deviation was 0.37 ms and observation age was 11.81 ms.
There were no repeated observations or clipped training steps. Maximum
absolute off-diagonal input correlation was 0.061. Thrust
injection-to-total correlation was 0.319. The MPC hover correction still
dominates part of that channel, but the additive excitation was present
in every issued command where requested. Truth position coverage was
approximately ±3 mm horizontally and -22 to 0 mm vertically relative to
the body-rate handover origin. This is a local hover dataset.

With T_ini=6, the same held-out future host inputs are used for both
predictors. Final-step XYZ RMSE norms:

| Predictor | Estimate 25 / 40 steps | Truth 25 / 40 steps |
| --- | --- | --- |
| Physics nominal | 9.80 / 18.35 mm | 9.89 / 18.31 mm |
| Physics train-fit vertical | 7.72 / 15.38 mm | 7.76 / 15.32 mm |
| Regularized behavioral | 0.69 / 1.22 mm | 0.73 / 1.20 mm |
| Persistence | 4.82 / 6.71 mm | 4.86 / 6.72 mm |

Behavioral prediction improves on persistence for all nine channels on
both scoring streams at both terminal steps. The corresponding 40-step
truth error is 2.40 / 1.25 / 1.20 mm with 4 / 8 / 11 training episodes.
PE is already full at four episodes, so the learning curve gives evidence
about useful data volume that rank alone misses. T_ini=10 is also reported,
with a different valid-window count.

**Disposition:** accepted as the shared local development dataset. The
nominal nine-state physics model does not pass the broadband open-loop
position benchmark. Fitting only thrust on the training split reduced the
error, but did not beat persistence and left horizontal error unchanged.
The fitted gain was 5.44 versus the nominal 14.495, with hover thrust
0.476907. Treat it as an effective fit under the present timing,
closed-loop feedback, and unmodeled tracking dynamics. Leave the production
MPC unchanged. Investigate state/send alignment, inner-loop response and
model residuals with existing train/validation data before deciding whether
to augment the model or collect a targeted dataset. Formal freeze requires
a model decision and held-out evaluation beyond this single nominal hover
operating region.
