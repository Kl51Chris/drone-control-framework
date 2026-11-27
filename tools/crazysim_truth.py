"""Launch the local CrazySim with a read-only MuJoCo truth observer."""

import argparse
import csv
import importlib
from pathlib import Path
import queue
import signal
import sys
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulator", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("simulator_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    sys.path.insert(0, str(args.simulator.resolve().parent))
    sim = importlib.import_module(args.simulator.stem)
    samples = queue.Queue(maxsize=512)
    last = {}
    dropped = 0
    original = sim.DroneAgent.read_pose
    stream = args.output.open("x", newline="", encoding="utf-8")
    writer = csv.writer(stream)
    writer.writerow(("agent", "sim_s", "monotonic_s", "x", "y", "z", "qx", "qy", "qz", "qw",
                     "vx", "vy", "vz", "p", "q", "r", "observer_s"))

    def write_samples():
        while (row := samples.get()) is not None:
            writer.writerow(row)

    worker = threading.Thread(target=write_samples)
    worker.start()

    def observe(agent):
        pose = original(agent)
        sim_time = float(agent.data.time)
        key = int(agent._body_id)
        if sim_time + 1e-9 >= last.get(key, 0.0):
            nonlocal dropped
            started = time.monotonic()
            last[key] = sim_time + 0.01
            address = agent._qvel_adr
            velocity = agent.data.qvel[address:address + 6].copy().tolist()
            try:
                samples.put_nowait((key, sim_time, started, *pose[0], *pose[1], *velocity,
                                     time.monotonic() - started))
            except queue.Full:
                dropped += 1
        return pose

    def terminate(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    sim.DroneAgent.read_pose = observe
    sys.argv = [str(args.simulator), *args.simulator_args]
    try:
        sim.main()
    except KeyboardInterrupt:
        pass
    finally:
        samples.put(None)
        worker.join()
        stream.close()
        print(f"ground truth samples dropped: {dropped}", flush=True)


if __name__ == "__main__":
    main()
