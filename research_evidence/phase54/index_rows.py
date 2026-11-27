"""Idempotently add Phase 5.4 Stage A rows to research_evidence/experiment_index.csv.

Usage (repo root): .venv/bin/python research_evidence/phase54/index_rows.py
Removes existing rows whose phase starts with "5.4" and re-appends them from
log/phase54/stage-a (registered runs) plus the four unscored incident duplicates.
Rerun after research_evidence/build_index.py, which does not know Phase 5.4.
"""
import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
INDEX = ROOT / "research_evidence/experiment_index.csv"
STAGE = ROOT / "log/phase54/stage-a"
SOURCE = "research_evidence/phase54/stage-a/stage_a_runs.csv"


def git(run):
    meta = next(run.glob("*/metadata.json"), None)
    return json.loads(meta.read_text()).get("git_commit", "unavailable") if meta else "unavailable"


def row(run, case, status, outcome, fallback, deadline, reason):
    raw = json.loads((run / "experiment.json").read_text())
    level = case["level"]
    return dict(
        phase="5.4-A", experiment_id=f"phase54/stage-a/{run.name}", experiment_name=case["name"], evidence_type="flight",
        controller=case["controller"], configuration=json.dumps(dict(case=case), separators=(",", ":")),
        dataset_version="phase3-v1",
        noise_profile=json.dumps({"amplitude": float(level), "profile": "none" if level == 0 else "iid-small"},
                                 separators=(",", ":")),
        noise_seed=str(case["seed"]), excitation_seed="unavailable", process_status=status,
        success_failure_status=outcome, fallback_count=str(fallback), runner_deadline_misses=str(deadline),
        status_reason=reason, original_log_path=str(run.relative_to(ROOT)), recorded_git_commit=git(run),
        reproduction_command=json.dumps({k: " ".join(raw[k]) for k in ("firmware", "simulator", "flight")},
                                        separators=(",", ":")),
        supporting_source=SOURCE)


def main():
    with INDEX.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fields, rows = reader.fieldnames, [r for r in reader if not r["phase"].startswith("5.4")]
    summary = json.loads((STAGE / "summary.json").read_text())
    new = []
    for r in summary:
        run = ROOT / r["path"]
        if r.get("eligible"):
            outcome, why = "passed_feasibility", "unavailable"
        elif r.get("fallback"):
            outcome, why = "unexpected_fallback", f"{r.get('fallback_reason')}; deadline_misses={r.get('deadline_misses')}"
        else:
            outcome, why = "failed", "ineligible under plan section 3"
        new.append(row(run, r["case"], r["status"], outcome, r.get("fallback", "unavailable"),
                       r.get("deadline_misses", "unavailable"), why))
    for run in sorted(STAGE.glob("*-retry1")):
        case = json.loads((run / "experiment.json").read_text())["case"]
        status = json.loads((run / "experiment.json").read_text())["status"]
        new.append(row(run, case, status, "failed" if status != "completed" else "completed",
                       "unavailable", "unavailable",
                       "unscored spurious duplicate from detector incident I1 (research_evidence/phase54/incidents.md)"))
    with INDEX.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader(); w.writerows(rows + new)
    print(f"kept {len(rows)} rows, added {len(new)} Phase 5.4 rows")


if __name__ == "__main__":
    main()
