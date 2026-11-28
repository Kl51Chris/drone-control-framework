"""Generate a deterministic, balanced controller order for a registered sensitivity design."""

import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("version", choices=("0.1", "0.2", "0.3", "0.4", "0.5"))
parser.add_argument("output", type=Path)
parser.add_argument("--timing-profile", choices=("position-loss", "position-bias"),
                    default="position-bias")
parser.add_argument("--timing-scale", type=int, choices=(1, 2), default=2)
args = parser.parse_args()
controller_names = {"joint-fir6": "outer-loop-mpc", "t6-ref": "outer-loop-deepc"}
profiles = ("position-loss", "position-bias")
config = Path(__file__).resolve().parent / "config"
designs = {
    "0.1": (1, [("nominal", 1, 150), *((p, 1, 150) for p in profiles)],
            "MVP scenarios, replayed with the study runner."),
    "0.2": (4, [("nominal", 1, 150), *((p, 1, 150) for p in profiles)],
            "Independent cold starts under the original conditions. Estimate repeatability; retain MVP as a separate discovery sample."),
    "0.3": (4, [(p, s, 150) for p in profiles for s in (1, -1)],
            "Compare both cross-track directions in each repeat block at the original strength; no nominal runs."),
    "0.4": (2, [(p, s, 150) for p in profiles for s in (1, -1, 2, -2)],
            "Compare original and double strength within each repeat block in both directions; descriptive small-sample stress."),
    "0.5": (3, [(args.timing_profile, args.timing_scale, start) for start in (150, 300)],
            "Compare straight-travel and reversal onset for the selected fault in three paired blocks. Preserve magnitude and duration; late recovery has shorter follow-up."),
}
repeats, conditions, question = designs[args.version]
rows = []
for rep in range(1, repeats + 1):
    ordered = conditions[rep % len(conditions):] + conditions[:rep % len(conditions)]
    for profile, scale, start in ordered:
        order = list(controller_names)
        if rep % 2 == 0:
            order.reverse()
        for ctrl in order:
            argv = ["--reference", "line-inspection", "--reference-mode", "preview", "--horizon", "25",
                    "--mpc-config", str(config / f"{ctrl}.json"), "--navigation", profile]
            if scale != 1:
                argv += ["--navigation-scale", str(scale)]
            if start != 150:
                argv += ["--navigation-start-step", str(start)]
            if ctrl == "joint-fir6":
                argv += ["--mpc-model", ctrl]
            rows.append(dict(name=f"{args.version}-{rep}-{profile}-{scale:+g}-{start}-{ctrl}",
                             baseline=ctrl, navigation=profile, scale=scale, start_step=start,
                             repetition=rep, controller=controller_names[ctrl], args=argv))
with args.output.open("x") as stream:
    json.dump(dict(version=args.version, question=question,
                   evaluation="Identical v0.1 scoring and fixed normalization; analyze like conditions within repeat blocks. No outcome-based reruns.",
                   cases=rows), stream, indent=2)
    stream.write("\n")
