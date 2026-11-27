# Research evidence archive

This directory indexes CrazySim research through Phase 5.3 and provides portable analysis snapshots and presentation figures. Use [presentation_index.md](presentation_index.md) for Jeremy's meeting, [decision_log.md](decision_log.md) for the technical narrative, and [final_configuration.yaml](final_configuration.yaml) for the provisional development configuration.

## Files and evidence status

| File | Use |
|---|---|
| [parameter_registry.csv](parameter_registry.csv) | Per-parameter mathematical definition, units, options, current status, experiment IDs and caveats |
| [experiment_index.csv](experiment_index.csv) | Batch and single-flight index plus offline analyses, with separate process and scientific outcome fields |
| [regularization_trials.csv](regularization_trials.csv) | Every original Phase 5.3 attempt, including the MPC controls and the nonadvancing settings. Tracking, cost, slack, g norm and timing retain their reported scope. |
| [regularization_comparison.md](regularization_comparison.md) | Six screening settings, precise parameter definitions and the selection outcome |
| [decision_log.md](decision_log.md) | Problem, hypothesis, experiment, evidence, decision and limitation |
| [presentation_index.md](presentation_index.md) | Main and appendix figures, supporting claims, protocol boundaries and source pointers |
| [final_configuration.yaml](final_configuration.yaml) | Provisional recommendations, code defaults, and unset GNSS parameters |

The experiment index distinguishes reported feasible outcomes, failed or incomplete cases, intentional and unexpected HLC fallback, dataset admission/rejection, and missing feasibility. Standalone MPC logs around Phase 1 are grouped by their recorded controller and timestamp. Older context flights without phase provenance have an explicit unverified phase label. A recorded Git hash identifies the recorded HEAD, with working-tree changes documented in the corresponding phase audits. Absent fields are marked unavailable. Three sealed raw episode entries are listed from the manifest only. Their raw and processed files were left unopened.

A historical reproduction field with three commands is a JSON object containing the separately recorded firmware, simulator, and flight commands. Preserve it as evidence. To reproduce, use the phase instructions and a fresh output directory with a free port. The original commands may target the original log directory. This archive creation performed only file reads and derivations.

## Retention and integrity

Phase 5.3 has a complete local compressed copy at [archive/phase53-complete.tar.gz](archive/phase53-complete.tar.gz), including every raw run, matrix, script, config, audit and derived metric. It is Git-ignored because of size. Keep it with the per-file and archive checksums in the laboratory backup. The original log tree remains in place. Older historical raw run paths are indexed in place. Their files and any existing native figures are still local to the checkout. Selected figure-source summaries are also in the small `snapshots/` tree, with their original relative paths.

From the repository root:

```bash
sha256sum -c research_evidence/archive/phase53-original-files.sha256
cd research_evidence/archive
sha256sum -c phase53-complete.tar.gz.sha256
```

The complete archive can be restored to a new empty directory with `tar -xzf`. For display-only regeneration, run `.venv/bin/python research_evidence/build_figures.py` from the repo. The plotting script prefers the frozen summary snapshots and does not load dataset episodes. The CSV builder reads original log metadata and scored summaries from the project root. It reads the sealed paths only from the manifest to filter them out.

## Maintenance after each Phase

从 Phase 5.4 起按[研究报告协议](../doc/research-reporting-protocol.md)生成并归档图表、图表源数据和复现脚本。

1. Preserve raw runs, failures, aborts, configs, matrices, summary scope, recorded Git/dirty state and reproduction commands.
2. Append or regenerate the experiment index. Verify run denominators, durable paths and explicit reasons for exclusions. Keep the Phase 5.3 archive and old comparisons immutable.
3. Add each decision with its candidate values, fixed conditions, metric definition, evidence IDs and limitation. Record superseded settings and their replacement. Keep the status provisional until formal approval and independent confirmation.
4. Add new figures and source pointers to the presentation index. Draw comparisons within their registered cohort and retain failed or incomplete cases.
5. Update the configuration YAML without changing historical thresholds or the sealed split. GNSS injection, timing, bias, recovery and paired seed definitions remain pending until separately registered.
