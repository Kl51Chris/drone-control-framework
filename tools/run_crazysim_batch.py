"""Run a JSON matrix of independent CrazySim MPC flights in sequence."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def stop(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def run_case(root, case, sim, gui, port, controller="outer-loop-mpc"):
    repo = Path(__file__).resolve().parents[1]
    firmware = sim / "crazyflie-firmware"
    simulator = firmware / "tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py"
    env = {**os.environ, "OPENBLAS_NUM_THREADS": "1", "PYTHONDONTWRITEBYTECODE": "1"}
    fw_args = [str(firmware / "sitl_make/build/cf2"), str(port)]
    sim_args = [
        str(sim / ".venv/bin/python"), str(repo / "tools/crazysim_truth.py"),
        str(simulator), str(root / "sim_truth.csv"),
        "--port", str(port), "--model-type", "cf2x_T350", "--dt", "0.001",
        *(["--vis"] if gui else []), "--", "0,0",
    ]
    flight_args = [
        sys.executable, "-m", "experiments.run_flight", "--backend", "crazysim",
        "--controller", controller, "--execute", "--log-root", str(root),
        "--uri", f"udp://127.0.0.1:{port - 100}",
        *case["args"],
    ]
    forbidden = {"--backend", "--controller", "--uri", "--log-root", "--execute"}
    if any(arg.split("=")[0] in forbidden for arg in case["args"]):
        raise ValueError("matrix cannot override simulator isolation or logging")
    result = {"case": case, "firmware": fw_args, "simulator": sim_args,
              "flight": flight_args, "status": "started"}
    metadata = root / "experiment.json"
    metadata.write_text(json.dumps(result, indent=2))
    processes = []
    try:
        with (root / "firmware.log").open("w") as fw_log, (root / "simulator.log").open("w") as sim_log, (root / "flight.log").open("w") as flight_log:
            processes.append(subprocess.Popen(fw_args, cwd=root, stdout=fw_log, stderr=subprocess.STDOUT))
            processes.append(subprocess.Popen(sim_args, cwd=root, stdout=sim_log, stderr=subprocess.STDOUT))
            time.sleep(4)
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("SITL exited during startup")
            flight = subprocess.Popen(flight_args, cwd=repo, env=env, stdout=flight_log, stderr=subprocess.STDOUT)
            processes.append(flight)
            result["exit_code"] = flight.wait(timeout=45)
            result["status"] = "completed" if result["exit_code"] == 0 else "failed"
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        result.update(status="failed", error=str(error))
    finally:
        for process in reversed(processes):
            stop(process)
        if result["status"] == "started":
            result["status"] = "interrupted"
        metadata.write_text(json.dumps(result, indent=2))
    return result["status"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matrix", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--sim-root", type=Path, default=Path("/home/chris/CrazySim"))
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--controller", choices=("outer-loop-mpc","outer-loop-deepc"), default="outer-loop-mpc")
    parser.add_argument("--port", type=int, default=24950, help="SITL firmware port (CFLib uses port minus 100)")
    args = parser.parse_args()
    def terminate(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    matrix = json.loads(args.matrix.read_text())
    root = args.output.resolve()
    root.mkdir(parents=True)
    (root / "matrix.json").write_text(json.dumps(matrix, indent=2))
    failures = 0
    for index, case in enumerate(matrix, 1):
        directory = root / f"{index:03d}"
        directory.mkdir()
        status = run_case(directory, case, args.sim_root.resolve(), args.gui, args.port, args.controller)
        print(f"{index}: {case['name']}: {status}", flush=True)
        failures += status != "completed"
    return int(bool(failures))


if __name__ == "__main__":
    sys.exit(main())
