"""Combine decision matrices with version labels, maintaining original denominators."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path


def records(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_table(path, table):
    fields = list(dict.fromkeys(key for row in table for key in row))
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(table)


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("output", type=Path, help="Study root with v02 through v05 runs and analysis")
parser.add_argument("--mvp-analysis", type=Path, default=Path("log/inspection-mvp-hkd41J"))
args = parser.parse_args()
out = args.output
mvp = args.mvp_analysis
designs = [("0.1", "pilot", mvp, None),
           ("0.2", "repeatability", out / "v02-analysis", out / "v02"),
           ("0.3", "fault direction", out / "v03-analysis", out / "v03"),
           ("0.4", "fault strength", out / "v04-analysis", out / "v04"),
           ("0.5", "fault timing", out / "v05-analysis", out / "v05")]
matrix, failures = [], []
elapsed = 0
actual = 0
for version, design, analysis, raw in designs:
    if not (analysis / "decision-matrix.csv").exists():
        continue
    rows = records(analysis / "comparison.csv")
    if raw is not None:
        batches = [json.loads(line) for line in (raw / "runs.jsonl").read_text().splitlines()]
        assert len(batches) == len(rows)
        assert (raw / "cleanup.txt").exists()
        actual += len(rows)
        elapsed += sum(run["elapsed_s"] for run in batches)
    for score in records(analysis / "decision-matrix.csv"):
        subset = [r for r in rows if r["controller"] == score["controller"]]
        passed = sum(r.get("violation_free_completion") == "True" for r in subset)
        matrix.append(dict(version=version, design=design, runs=len(subset), qualified=passed, **score))
    for row in rows:
        if row.get("violation_free_completion") != "True":
            failures.append(dict(version=version, **row))
write_table(out / "decision-matrices-by-version.csv", matrix)
if failures:
    write_table(out / "ineligible-runs.csv", failures)
data = dict(additional_runs=actual, summed_serial_case_seconds=round(elapsed, 3),
            new_ineligible=len(failures), failure_counts=Counter(r["controller"] for r in failures))
(out / "session-counts.json").write_text(json.dumps(data, indent=2) + "\n")
print(json.dumps(data, indent=2))
