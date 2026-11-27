# Phase 5.3 penalty sensitivity and selection

All 40 original experiments are in [regularization_trials.csv](regularization_trials.csv). The screening subset contains 18 flights across six settings. There were no failed Phase 5.3 flights. The four settings that did not advance are retained. Earlier Phase 5.2 failures remain in the experiment index. The complete Phase 5.3 archive includes run-level files and every configuration template. Templates for unselected confirmation candidates are distinguishable from executed experiments.

The objective uses `0.5 λ_g ||g||²` and `0.5 β ||e_ini||²`, with `λ_g = (ρ σ_max(H_u_scaled))²`. The initial residual is evaluated on retained normalized output rows of the history. The older Phase 3 offline coordinate replay uses its own ridge convention. The physical Q/R/S terms and physical output slack are separate. See [technical definition](../log/phase53/report.md).

| T_ini | ρ | β | Actual λ_g | L1 hover cost/s | L1 circle cost/s | Mean L1 cost/s | Outcome |
|---:|---:|---:|---:|---:|---:|---:|---|
| 6 | .001 | 1 | .0133075422 | .12162 | .13661 | .12912 | Screened |
| 6 | .003 | 1 | .1197678798 | .09971 | .10979 | .10475 | Screened |
| 6 | .001 | .25 | .0133075422 | .02962 | .02636 | .02799 | Confirmed, provisional primary |
| 12 | .001 | 1 | .0156936665 | .18560 | .22572 | .20566 | Screened |
| 12 | .003 | 1 | .1412429985 | .15374 | .18403 | .16889 | Screened |
| 12 | .001 | .25 | .0156936665 | .04014 | .04973 | .04494 | Confirmed, provisional comparator |

Costs use original shared evaluation Q/R/S and the registered formal scoring windows. CrazySim, shared N25 and phase3-v1 are held fixed. Bridge and formal windows are identified separately in the raw source. Noise profile, seed, component costs, per-axis truth RMSE, initialization residual, g norm, window and full-run timing and fallback are recorded by experiment ID. Second-seed confirmation was 6/6 and L2 pressure was 4/4. Both finalists retain original S after the independent shared-weight gate. The formal experiment configuration remains provisional.

[Screening figure](figures/lambda_beta_sensitivity.png) · [Registry](parameter_registry.csv) · [Original Phase 5.3 audit](../log/phase53/results-audit.json)
