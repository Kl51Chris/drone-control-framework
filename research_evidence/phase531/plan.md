# Phase 5.3.1 · Preregistered offline validation

Only the 11 manifest train entries may be opened. Original development
validation and locked test files are excluded before any NPZ load.
Sort the training manifest by processed filename, permute indices using
NumPy PCG64 seed 653100 and split into groups of 3/2/2/2/2. Freeze the
whole-episode folds in folds.json. No grouping of overlapping columns.
Every held-out origin is k=25 through len(y)-N-1, as in Phase 5.1.

N=25, T_ini=6, 50 Hz, 16 retained latent directions and the unchanged
production model, scaling, Q/R/S and output/input limits. Build the
existing production reduced model per fold via a temporary filtered
manifest of training-side episodes, and reuse the production training
blocks for diagnostics. Production input equality must be well conditioned
at relative 1e-4 and depth N+T_ini+9 must have full row rank at that cutoff.
Order-16/29 PE and joint conditioning are diagnostic. Keep failures and
stop inference if a required fold fails. Consider leave-one-episode-out
only after a failed five-fold gate, without silently dropping any fold.

For each of the 25 rho x beta combinations use L0 and two complete L1
replicates, primary seed 653101 and sensitivity seed 653102. The L0
setting is the existing zero-amplitude profile. Episode-specific streams
are spawned deterministically by SeedSequence([replicate_seed, sorted
episode ordinal]) and share exact observations across candidates.
Inject the existing iid-small profile on observed history in physical
units before window-local anchoring. Train Hankel columns and truth remain
clean. Actual past/future issued inputs are fixed offline. As in Phase 5.1,
the conditional predictor minimizes past-fit plus g penalty in the fixed
future-input affine subspace. No future tracking target, optimizer policy,
or inequality enforcement is used to fit predictions to withheld truth.
Prediction diagnostics never feed back to a controller.

Primary metric is equal-episode mean of XYZ full-horizon RMS integrating
axes and steps in metres (reported in mm). Keep per-axis RMS and all
terminal scores. Average within each episode first. Fold means are
equal-episode and the reported fold SD is sample SD, with worst fold.
Velocity and attitude are separate physical groups in m/s and rad.
Penalty values and timings are diagnostic and do not enter rankings.

Compare with rho=.001 and beta=.25. Advance at most two additional
candidates. Require at least 10% mean L1 XYZ improvement for each seed
and a positive L1 XYZ change in at least four of five folds for each seed.
Require improvement in at least two of the three physical L1 groups
(XYZ, velocity, attitude), including XYZ, and no group mean more than 10%
worse for either seed. L0 XYZ, velocity and attitude group means may not
degrade by more than 10%. Sort candidates passing all gates by primary
L1 XYZ mean, then ascending rho and beta. The 10% bands are development
selection criteria, not flight tolerances or statistical significance.
Compare the current point with its four axial grid neighbors and keep
boundary winners visible. Use L2 seed 653103 only for the current point
and up to two selected alternatives. L2 is diagnostic and does not
rerank the choices. No additional seeds, parameter grid extensions, CV
leakage, or closed-loop flights.
