# Phase 5.4 execution incidents

## I1 · Infrastructure-failure detector false positive (2026-11-27)

- **What happened.** `has_takeoff()` in phase54.py searched events.csv for the
  literal text `"phase": "TAKEOFF"`. events.csv stores details as CSV-quoted JSON
  (`""phase"": ""TAKEOFF""`), so every run was classified as an infrastructure
  failure and immediately re-flown once with the same seed and conditions.
- **Affected.** Registered runs 001–004 (first attempts) completed normally and
  reached TAKEOFF. Spurious duplicates `001-retry1`, `002-retry1`, `003-retry1`
  completed; `004-retry1` was interrupted by the operator (SIGTERM) before takeoff
  when the bug was noticed. Batch log: `log/phase54/stage-a-attempt1.out`; original
  summary: `log/phase54/stage-a/summary-attempt1-detector-bug.json`.
- **Classification.** None of 001–004 was an infrastructure failure, so under
  plan §9 no retry was permitted. The first attempts are the registered runs and
  are the only ones scored. The four `*-retry1` directories are retained unmodified,
  labeled *spurious duplicate (detector bug)*, and excluded from all gates and
  figures. No result was inspected before the decision; the classification follows
  from the TAKEOFF event alone.
- **Fix.** Parse events.csv with `csv.DictReader` + `json.loads` and test
  `phase_changed` / `TAKEOFF`. Verified on 001, 001-retry1, 004 (True) and
  004-retry1 (False). Added `--resume`: existing completed first attempts are
  re-audited (not re-flown); execution continues from run 005. Genuine retries are
  now named `*-infra-retry1`. No matrix, seed, scorer, gate or threshold change.
