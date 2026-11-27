"""Generate the isolated Phase 3 excitation matrix."""

import argparse
import json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("pilot", "wide", "filtered", "dataset"))
    args = parser.parse_args()
    if args.stage == "pilot":
        cases = [(axis, 100 + i, "pilot", "8") for i, axis in enumerate(
            ("roll", "pitch", "yaw", "thrust", "all"))]
    elif args.stage == "wide":
        cases = [("all", 201, "pilot", "16"), ("all", 202, "pilot", "16")]
    elif args.stage == "filtered":
        cases = [("all", 301 + i, "pilot", "16") for i in range(4)]
    else:
        cases = [("all", 3000 + i, split, "16") for i, split in
                 enumerate(["train"] * 12 + ["validation"] * 3 + ["test"] * 3)]
    print(json.dumps([
        {"name": f"excite-{axis}-{seed}", "split": split, "seed": seed,
         "args": ["--excitation-seed", str(seed), "--excitation-axis", axis,
                  "--excitation-max-hz", bandwidth, "--excitation-kind",
                  "filtered" if args.stage in ("filtered", "dataset") else "multisine"]}
        for axis, seed, split, bandwidth in cases
    ], indent=2))


if __name__ == "__main__":
    main()
