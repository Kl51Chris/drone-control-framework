"""Describe within-block changes between conditions without pooling versions."""

import argparse
from collections import defaultdict
import csv
from pathlib import Path
import statistics


METRICS = ("rmse_m", "max_error_m", "recovery_s", "effort_per_s", "solver_p99_ms")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("analysis", type=Path)
    args = parser.parse_args()
    with (args.analysis / "comparison.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    groups = defaultdict(dict)
    for r in rows:
        key = r["controller"], r["scenario"], int(r["repetition"])
        groups[key][(float(r["scale"]), int(r["start_step"]))] = r
    effects = defaultdict(list)
    for (controller, scenario, _), group in groups.items():
        contrasts = []
        for strength in (1, 2):
            contrasts.append((f"reverse@{strength}", (strength, 150), (-strength, 150)))
        for sign in (1, -1):
            contrasts.append((f"double@{sign:+}", (sign, 150), (2 * sign, 150)))
            for strength in (1, 2):
                scale = sign * strength
                contrasts.append((f"late@{scale:+}", (scale, 150), (scale, 300)))
        for label, base, alternate in contrasts:
            if base not in group or alternate not in group:
                continue
            both = group[base], group[alternate]
            for metric in METRICS:
                if metric in ("rmse_m", "max_error_m", "effort_per_s") and any(
                    r["tracking_complete"] != "True" for r in both
                ):
                    continue
                if all(r.get(metric) for r in both):
                    effects[controller, scenario, label, metric].append(
                        float(both[1][metric]) - float(both[0][metric])
                    )
    fields = ["controller", "scenario", "contrast", "metric", "n", "median", "min", "max"]
    with (args.analysis / "condition-contrasts.csv").open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for key, data in sorted(effects.items()):
            writer.writerow([*key, len(data), statistics.median(data), min(data), max(data)])


if __name__ == "__main__":
    main()
