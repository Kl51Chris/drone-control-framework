"""Print a reproducible Phase 2 experiment matrix as JSON."""

import argparse
import json
import math


def build(stage, mode, repeats):
    circle = ["--reference", "circular"]
    scenarios = {
        "baseline-circle": circle,
        "faster-circle": circle + ["--circle-duration", "10"],
        "scaled-circle": circle + ["--circle-radius", "0.08", "--circle-duration", "12"],
    }
    cases = []

    def add(name, repeat, args, **tags):
        cases.append({"name": name, "repeat": repeat, **tags, "args": args})

    for repeat in range(1, repeats + 1):
        if stage == "compare":
            modes = ("constant", "preview") if repeat % 2 else ("preview", "constant")
            for reference_mode in modes:
                add("baseline-circle", repeat,
                    circle + ["--reference-mode", reference_mode, "--horizon", "25"],
                    mode=reference_mode, horizon=25)
        elif stage == "nominal":
            for axis in ("x", "y", "z", "yaw"):
                for sign in ((1, -1) if repeat % 2 else (-1, 1)):
                    amplitude = math.radians(5) if axis == "yaw" else 0.03
                    add(f"{axis}{'+' if sign > 0 else '-'}", repeat,
                        ["--step-axis", axis, "--step-amplitude", str(sign * amplitude),
                         "--step-hold", "2", "--reference-mode", mode],
                        mode=mode, horizon=25)
            for name in ("faster-circle", "scaled-circle"):
                add(name, repeat, scenarios[name] + ["--reference-mode", mode],
                    mode=mode, horizon=25)
        else:
            horizons = (15, 25, 40)
            offset = (repeat - 1) % len(horizons)
            for horizon in (*horizons[offset:], *horizons[:offset]):
                options = ["--reference-mode", mode, "--horizon", str(horizon)]
                add("baseline-circle", repeat, circle + options, mode=mode, horizon=horizon)
                add("x+", repeat, ["--step-axis", "x", "--step-hold", "2"] + options,
                    mode=mode, horizon=horizon)
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("compare", "nominal", "horizon"))
    parser.add_argument("--mode", choices=("constant", "preview"), default="preview")
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("repeats must be positive")
    print(json.dumps(build(args.stage, args.mode, args.repeats), indent=2))


if __name__ == "__main__":
    main()
