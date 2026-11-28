"""Explain matrix gaps, retaining the primary failure-inclusive scores."""

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import statistics


def utility(row, key, normalization):
    if key == "violation_free_completion":
        return 100.0 if row[key] == "True" else 0.0
    value = row.get(key)
    return max(0.0, 100 * (1 - float(value)/normalization[key])) if value else 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("analysis", type=Path)
    args = parser.parse_args()
    directory = args.analysis
    scoring = directory / "scoring.json"
    if not scoring.is_file():
        scoring = directory.parent / "scoring.json"
    settings = json.loads(scoring.read_text())
    with (directory / "comparison.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    with (directory / "decision-matrix.csv").open() as stream:
        official = list(csv.DictReader(stream))

    pairs = defaultdict(dict)
    for row in rows:
        pairs[(row["scenario"], row["scale"], row["start_step"], row["repetition"])][row["controller"]] = row
    survivors = defaultdict(list)
    for condition, pair in pairs.items():
        if all(r["violation_free_completion"] == "True" for r in pair.values()):
            survivors[condition[:3]].append(pair)
    with (directory / "score-attribution.csv").open("x", newline="") as stream:
        w = csv.writer(stream)
        w.writerow(["criterion", "weight_percent", "official_deepc_minus_mpc_points",
                    "matched_complete_deepc_minus_mpc_points", "matched_complete_conditions",
                    "matched_complete_pairs"])
        for key, weight in settings["weights"].items():
            score = key + "_score"
            diff = float(official[1][score]) - float(official[0][score])
            paired = []
            pair_count = 0
            for condition, subset in survivors.items():
                if key == "recovery_s" and condition[0] not in settings["recovery_profiles"]:
                    continue
                delta = [utility(p["t6-ref"], key, settings["zero_score_at"]) -
                         utility(p["joint-fir6"], key, settings["zero_score_at"]) for p in subset]
                paired.append(statistics.mean(delta))
                pair_count += len(delta)
            conditional = weight / 100 * statistics.mean(paired) if paired else ""
            w.writerow([key, weight, weight / 100 * diff, conditional, len(paired), pair_count])


if __name__ == "__main__":
    main()
