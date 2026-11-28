"""Summarize issued and rejected QP traces around the two fault transitions."""

import argparse
import csv
import json
from pathlib import Path
import re

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("iteration", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    plan = json.loads((args.iteration / "plan.json").read_text())
    fields = ["run", "controller", "profile", "scale", "start", "repetition", "edge", "accepted_ticks",
              "accepted_max_ms", "rejected_offset_from_edge", "rejected_elapsed_ms", "failure"]
    with args.output.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for i, case in enumerate(plan["cases"], start=1):
            if case["navigation"] == "nominal":
                continue
            folder = args.iteration / f"{i:03d}"
            log = next(folder.glob("*/flight.csv"), None)
            if log is None:
                continue
            with log.open(newline="") as flight:
                records = [(row, json.loads(row["controller_diagnostics_json"])) for row in csv.DictReader(flight)]
            for edge, tick in (("onset", case["start_step"]), ("restoration", case["start_step"] + 200)):
                accepted, rejected = [], None
                for row, diag in records:
                    step = diag.get("measurement_noise", {}).get("step")
                    if step is None or not tick - 10 <= step <= tick + 10:
                        continue
                    if row["command_thrust_normalized"]:
                        accepted.append(diag["mpc"]["qp_elapsed_s"] * 1000)
                    if rejected is None and (failure := diag.get("fallback_reason")):
                        match = re.search(r"elapsed=([\d.]+)s", failure)
                        rejected = dict(rejected_offset_from_edge=step - tick, failure=failure)
                        if match:
                            rejected["rejected_elapsed_ms"] = float(match.group(1)) * 1000
                writer.writerow(dict(run=folder.name, controller=case["baseline"], profile=case["navigation"],
                                     scale=case["scale"], start=case["start_step"], repetition=case["repetition"],
                                     edge=edge, accepted_ticks=len(accepted),
                                     accepted_max_ms=float(np.max(accepted)) if accepted else "",
                                     **(rejected or {})))


if __name__ == "__main__":
    main()
