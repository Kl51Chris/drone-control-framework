"""Execute a locked inspection matrix serially using the existing case runner."""

import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy
import osqp
import scipy

from tools.run_crazysim_batch import run_case


def check_idle(port):
    if subprocess.run(["pgrep", "-x", "cf2"], capture_output=True).returncode == 0:
        raise RuntimeError("An existing firmware process is active")
    if subprocess.run(["pgrep", "-af", r"^([^ ]*/)?python[^ ]* .*crazysim"],
                      capture_output=True).returncode == 0:
        raise RuntimeError("An existing simulator process is active")
    sockets = subprocess.check_output(["ss", "-H", "-lntu"], text=True)
    for line in sockets.splitlines():
        if line.split()[4].rsplit(":", 1)[-1] in (str(port), str(port - 100)):
            raise RuntimeError(f"Simulation port already bound: {line}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--sim-root", type=Path, default=Path("/home/chris/CrazySim"))
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    out = args.output.resolve()
    port = 25550
    sim = args.sim_root.resolve()
    config = Path(__file__).resolve().parent / "config"
    for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS"):
        os.environ[variable] = "1"
    for relative in (".venv/bin/python", "crazyflie-firmware/sitl_make/build/cf2",
                     "crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py"):
        if not (sim / relative).is_file():
            parser.error(f"missing CrazySim dependency: {sim / relative}")
    for case in plan["cases"]:
        argv = case["args"]
        dependency = Path(argv[argv.index("--mpc-config") + 1])
        if not dependency.is_file():
            parser.error(f"missing controller configuration: {dependency}")
    for name in ("log/phase3-v1/manifest.json", "log/phase3-offline-coordinates/results.json"):
        if not Path(name).is_file():
            parser.error(f"missing experiment input: {name}")
    check_idle(port)
    out.mkdir()
    (out / "plan.json").write_bytes(args.plan.read_bytes())
    (out / "scoring.json").write_bytes((config / "scoring.json").read_bytes())
    sources = ["experiments/inspection.py", "experiments/run_flight.py",
               "log/phase3-v1/manifest.json", "log/phase3-offline-coordinates/results.json"]
    sources.extend(str(path) for path in (
        Path(__file__), config / "scoring.json", Path("tools/analyze_inspection_mvp.py"),
        Path("tools/run_crazysim_batch.py"), Path("tools/crazysim_truth.py")
    ))
    provenance = dict(git=subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
                      source_hashes={p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in sources},
                      config_hashes={
                          str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in sorted({Path(case["args"][case["args"].index("--mpc-config") + 1])
                                              for case in plan["cases"]})
                      },
                      sim_git=subprocess.check_output(["git", "-C", sim, "rev-parse", "HEAD"], text=True).strip(),
                      firmware_git=subprocess.check_output(["git", "-C", sim / "crazyflie-firmware", "rev-parse", "HEAD"], text=True).strip(),
                      python=sys.version, numpy=numpy.__version__, scipy=scipy.__version__, osqp=osqp.__version__,
                      openblas_threads=1, omp_threads=1, rng="none; deterministic fault")
    (out / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    notes = out / "runs.jsonl"
    for i, case in enumerate(plan["cases"], 1):
        check_idle(port)
        dest = out / f"{i:03d}"
        dest.mkdir()
        started = time.monotonic()
        status = run_case(dest, case, sim, False, port, case["controller"])
        check_idle(port)
        flight = next(dest.glob("*/flight.csv"), None)
        entry = dict(directory=dest.name, name=case["name"], status=status,
                     elapsed_s=round(time.monotonic() - started, 3), flight_log=bool(flight))
        with notes.open("a") as stream:
            stream.write(json.dumps(entry) + "\n")
        print(f"{i}/{len(plan['cases'])} {entry}", flush=True)
        if not flight:
            raise RuntimeError(f"No flight log for {case['name']}; inspect simulator prerequisites")
    (out / "cleanup.txt").write_text("All experiment processes stopped; both ports free.\n")


def terminate(signum, frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, terminate)
    main()
