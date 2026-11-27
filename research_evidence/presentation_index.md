# Jeremy technical meeting · presentation and final report evidence

Use one protocol per comparison. Where a figure contains different protocols, the panels are labeled separately. Figures are available as PNG and SVG with the same names. Figure numbers are stable references for slides and report drafts.

## Main presentation figures

| ID and figure | Result to show | Supported conclusion | Inference to avoid | Source |
|---|---|---|---|---|
| F01 [MPC model trade-offs](figures/mpc_model_tradeoffs.png) | Phase 4.3 implemented size, terminal validation RMSE and realized physical cost are shown in separate panels for five models. | Joint FIR6 is a compact identified controller with useful closed-loop results. Longer FIRs lowered replay error. | A larger augmented state is the true plant order, or a smaller offline error guarantees lower closed-loop cost. | [FIR validation](../log/phase43-diagnosis/orders.json) · [Actual costs](../log/phase43-diagnosis/physical.json) |
| F02 [Reference preview](figures/reference_preview.png) | Phase 2 matched N25 circles, three runs per mode. X/Y error 16.0/14.2 to 4.6/4.1 mm. | Known-reference preview lowers measured tracking lag in this protocol. | An unannounced reference change is available early, or preview fixes unrelated estimator/model error. | [Pair scores](../log/mpc-phase2-compare/summary.json) |
| F03 [Potential of window-local coordinates](figures/deepc_coordinates.png) | Historical circles replayed with the same issued future inputs. N25 and N40 are separate panels, with terminal XYZ RMSE norm against the estimator. | Window-local position reduced extrapolation error on these trajectories. | Closed-loop stability, simulator-truth RMSE or immunity to anchored measurement noise. | [Replay arrays](../log/phase3-offline-coordinates/results.json) |
| F04 [Penalty sensitivity](figures/lambda_beta_sensitivity.png) | All six settings and all 18 screening flights. Separate L0 hover, L1 hover and L1 circle. | Reducing the initialization penalty changed realized physical cost more than the tested increase in g penalty. | A one-parameter lambda sweep or a globally optimal tuning result. | [All 40 rows](regularization_trials.csv) · [Pre-registration](../log/phase53/plan.md) |
| F05 [Nominal and noisy controller performance](figures/nominal_vs_noisy.png) | Phase 5.2 L0/L2 hover and circle under matched seed groups. A red × marks the T6 circle fallback. The second failure was T12 negative yaw, outside the plotted scenes. | Under these original development parameters DeePC was measurably more sensitive. | A ranking of the later tuned candidates, an achieved cost of zero for the failed condition, or GNSS-fault performance. | [Original runs and partial results](../log/phase52/summary.json) · [Negative-yaw fallback](../log/phase52/c/010) · [Circle fallback](../log/phase52/c/022) |

For a short talk, start with F02/F01 as the baseline design, then F03/F04, and close with F05 and the realtime limitations. Report Phase 5.3 stress separately using F09.

## Phase 5.4 Stage A figures

Single protocol: Phase 5.4 Stage A (DeePC T6 only, 72 runs, L0 n=1 and L1 n=3 seeds per scene). Not comparable across phases without labeling. PNG, SVG, figure CSV and script are kept side by side.

| ID and figure | Result to show | Supported conclusion | Inference to avoid | Source |
|---|---|---|---|---|
| F10 [Paired L1 tracking](phase54/stage-a/A1_paired_L1_tracking.png) | Candidate vs T6-ref 3D truth RMSE per pair; 054 unscored. | Strong regularization helps hover/yaw-step position, hurts the preview circle about 3.9×. | Candidates are globally better or worse, or statistical significance. | [Runs](phase54/stage-a/stage_a_runs.csv) · [Gates](phase54/stage-a/stage_a_gates.json) |
| F11 [Per-scene runs](phase54/stage-a/A2_per_scene_runs.png) | Every run, position/yaw/cost per scene, ineligible marked. | Scene dependence of the trade-off. | Yaw-step yaw RMSE separates candidates (it is shared non-convergence). | [Data](phase54/stage-a/A2_per_scene_runs.csv) |
| F12 [Cost decomposition](phase54/stage-a/A3_physical_cost_decomposition.png) | Tracking/effort/increment under original Q/R/S. | A/B remove noise-driven input chatter. | Lower tracking cost. | [Data](phase54/stage-a/A3_physical_cost_decomposition.csv) |
| F13 [Time-domain response](phase54/stage-a/A7_timeseries_circle6.png) | Circle shrinkage under A/B; [yaw step](phase54/stage-a/A7_timeseries_yaw7-pos.png) stalls near 35 mrad of 122 mrad for all three. | Closed-loop cost of strong ridge on persistent motion. | Cause of the yaw stall is established. | [Circle data](phase54/stage-a/A7_timeseries_circle6.csv) · [Yaw data](phase54/stage-a/A7_timeseries_yaw7-pos.csv) |

Appendix: [solver ECDF](phase54/stage-a/A4_solver_timing_ecdf.png) (no QP > 12 ms; 054 was a host stall, not the solver), [noise sensitivity per scene](phase54/stage-a/A5_noise_level.png), [DeePC internal terms](phase54/stage-a/A6_deepc_internal_terms.png). Full report: [Markdown](phase54/stage-a/report.md) · [HTML](phase54/stage-a/report.html).

## Appendix figures

| ID and figure | Result to show | Supported conclusion | Inference to avoid | Source |
|---|---|---|---|---|
| F06 [Prediction horizon](figures/prediction_horizon.png) | Phase 2 complete preview-circle rerun, N15/25/40. Mean run-level truth RMSE and QP P99. | The tested M0 horizon trades accuracy for computation. | Shared DeePC feasibility at N40 or reuse of the interrupted attempt as a replicate. | [Complete paired sweep](../log/mpc-phase2-horizon-repeat/summary.json) · [Interrupted log](../log/mpc-phase2-horizon) |
| F07 [Hankel singular spectrum and PE](figures/hankel_spectrum_PE.png) | Train-only relative singular values and rank/row counts at four depths. | Numerical full row rank at the registered cutoff in the near-hover dataset. | Physical plant-order identification, generalization to new motion, or independent test evidence. | [Quality summary](../log/phase3-v1/quality.json) |
| F08 [Initialization window](figures/initialization_horizon.png) | Phase 5.1 matched validation replay, terminal truth Z/yaw and joint conditioning. | Longer initialization improves these two open-loop errors while conditioning worsens. | T25 is better in flight or a yaw RMSE in mrad can be compared directly with Z RMSE in mm. | [Offline replay](../log/phase51/offline.json) · [Closed-loop comparison](../log/phase51/report.md) |
| F09 [Solver timing](figures/solver_timing.png) | Phase 5.1 raw cold/warm synthetic timings and distinct P5.2/P5.3 closed-loop percentiles. | Phase 5.2 had real budget misses. The registered finalist P5.3 stress had no budget misses. | The grouped bars form a paired estimate of solver improvement or prove sustained active-output reliability. | [Synthetic samples](../log/phase51/stress.json) · [P5.2](../log/phase52/summary.json) · [P5.3](../log/phase53/results-audit.json) |
| Historical [FIR axis response and DC gain](../log/phase43-diagnosis/orders.png) | Axis-specific fit orders and frequency-domain diagnosis. | The compact identified model has known low-frequency limitations. | A direct steady-state actuator measurement. | [FIR arrays](../log/phase43-diagnosis/orders.json) |
| Historical [Shared slew penalty sweep](../log/phase53/tuning-comparison.png) | Original scientific plot of L0/L1 circular trials, with the original evaluation weights. | Doubling S lowered mean physical cost while mean XY RMSE rose. | Independently tuned controller weights, or a Q or R sweep. | [Shared decisions](../log/phase53/runs/decisions.json) |
| Pending GNSS outage and bias tracking/recovery | Not yet verified. No results or illustration masquerading as data. | Pending formal degradation protocol and runs. | Inferring GNSS recovery time from iid measurement noise. | [Pending registry](final_configuration.yaml) |

The current files can support tables, figures and citations in technical documents. Preserve the original units and report the sample denominator when converting them to slides.

## Phase 5.3.1 appendix figures

| ID and figure | Result to show | Supported conclusion | Inference to avoid | Source |
|---|---|---|---|---|
| F10 [L0 heatmap](figures/cv_l0_heatmap.png) | All 25 nominal cells, marked reference and two finalists | Strongest relative regularization has a nominal RMSE trade-off | Treating RMSE as signed bias | [Grid values](phase531/candidate_metrics.csv) |
| F11 [L1 heatmap](figures/cv_l1_heatmap.png) | All 25 cells on the primary noisy seed | Best XYZ prediction reaches the tested upper ρ boundary | Locating a global optimum or justifying further search | [Grid values](phase531/candidate_metrics.csv) |
| F12 [Validation curves](figures/cv_curves.png) | L0 and L1a curves by initialization weight | Relative ridge strength changes the offline ranking with noise | Independent additive effects of λ_g and β in conditional prediction | [Fold-level data](phase531/fold_metrics.csv) |
| F13 [Nominal/noisy trade-off](figures/cv_nominal_noisy_tradeoff.png) | XYZ prediction trade-off by fixed groups | Both finalists improve mean L1 error while preserving L0 gate | Forecasting closed-loop tracking or physical cost | [Grid values](phase531/candidate_metrics.csv) |
| F14 [Initialization and g](figures/cv_initialization_and_g.png) | Normalized residual and coefficient norm, on matched L1a | Stronger relative ridge reduces g while residual can rise | Treating normalized residual as a measured physical error | [Diagnostics](phase531/candidate_metrics.csv) |
| F15 [Fold stability](figures/cv_fold_stability.png) | Five folds shown for each of two paired L1 seeds | Both alternatives improve in every recorded fold on both seeds | Treating the two seeds as independent new flights or declaring final controller superiority | [Fold-level data](phase531/fold_metrics.csv) |
