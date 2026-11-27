# Phase 5.3 finite noise-aware tuning registration

Fixed development configuration is phase3-v1, N=25, 50 Hz, 12 ms,
the Phase 5.2 measurement noise protocol and seeds derived by a fixed
scenario table below, physical limits, HLC lifecycle, reference preview
policy and Phase 5.1 scoring windows. Joint FIR6 and M0 are the only MPC
variants. DeePC uses T_ini 6/12 and the existing latent dimension.

Candidates per T_ini: baseline (rho=0.001, init weight=1), g3
(rho=0.003, init weight=1), init-quarter (rho=0.001, init weight=0.25).
No combined candidate and no additional settings after observing data.
Use unchanged physical Q/R/S for the first three stages.

1. Screen (18 runs): each candidate in L0 hover, L1 hover and L1 circle.
   Nominal/RNG seed 53010 for hover and 53011 for circle. Use the same
   existing 4 cm / 12 s previewed circle and default 2 s hover.
   A candidate must complete all original fixed-score windows without
   fallback, deadline events, or any existing hard constraint failure.
   Rank the feasible candidates separately per T by the mean fixed
   physical cost per second over both L1 scenes. Ties use the candidate
   declaration order. Retain one per T. If neither passes, stop.
2. Confirm (six runs maximum): each retained candidate uses L1 hover
   seed 53020 and L0/L1 circle seed 53021. A candidate must pass all
   three cases to continue. No substitution or additional repeat.
3. L2 pressure (four runs maximum): confirmed candidates use new fixed
   seed 53030 on negative 5-degree yaw (2s hold) and 53031 on the circle.
   A fallback or deadline is recorded as a pressure failure. This stress
   stage does not rerank candidates or change the L0/L1 development set.
4. Shared weights (12 new runs maximum): Q and R stay at the existing
   values; compare S vs 2*S at the common L0/L1 circle seed 53021. For
   default S, reuse each confirmed DeePC run from stage 2 and run the
   missing two MPC controls. For 2*S use exactly the same controllers
   and conditions. All variants always use identical physical weights
   in each configuration. Evaluate both weight sets with the original
   fixed common physical Q/R/S and unchanged scorer/windows.
   Recommend S2 only if all S2 cases are eligible, average fixed cost
   is strictly lower, and no controller has worse mean XY RMSE over
   L0/L1. Use original S if it is feasible and those conditions are not
   met. If neither setting meets the full gate, report no selected
   common tuning configuration.

Maximum 40 original SITL trials. No retries in this budget, and all
failures/incomplete outcomes remain in stages and original denominators.
A skipped adaptive stage must explicitly state the failed eligibility
gate. Do not rerun a candidate for solver/constraint/quality failure.
Preflight and end-of-stage tests use the unchanged existing definitions.
The formal independent experiment still needs new reserved seeds and
scenarios after development choices. No final test or GNSS runs.
