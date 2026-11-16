"""Passively validate CrazySim firmware telemetry without sending commands."""

from __future__ import annotations

import argparse
import json
import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from velocity_control_framework.backends import CrazyflieStateProvider


DEFAULT_URI = "udp://127.0.0.1:19850"


def collect(uri: str, duration_s: float) -> dict[str, float | int | str]:
    if not uri.startswith("udp://"):
        raise ValueError("CrazySim telemetry validation requires an udp:// URI")
    if duration_s <= 0.0:
        raise ValueError("duration must be positive")

    cflib.crtp.init_drivers()
    crazyflie = Crazyflie(rw_cache="./cache")
    timestamps: list[float] = []
    sample_count = 0
    with SyncCrazyflie(uri, cf=crazyflie) as link:
        provider = CrazyflieStateProvider(link.cf, delete_log_blocks_on_stop=True)
        try:
            provider.start()
            started = time.monotonic()
            deadline = started + duration_s
            last_timestamp: float | None = None
            while time.monotonic() < deadline:
                state = provider.get_state()
                values = (
                    *state.position, *state.velocity, state.roll, state.pitch,
                    state.yaw, state.p, state.q, state.r, state.timestamp,
                )
                if not all(math.isfinite(float(value)) for value in values):
                    raise RuntimeError("CrazySim telemetry contains a non-finite value")
                if state.timestamp != last_timestamp:
                    timestamps.append(state.timestamp)
                    sample_count += 1
                    last_timestamp = state.timestamp
                time.sleep(0.001)
        finally:
            provider.stop()

    elapsed = time.monotonic() - started
    gaps = [later - earlier for earlier, later in zip(timestamps, timestamps[1:])]
    return {
        "uri": uri,
        "duration_s": elapsed,
        "updates": sample_count,
        "update_frequency_hz": sample_count / elapsed if elapsed else 0.0,
        "maximum_receive_gap_s": max(gaps, default=0.0),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=DEFAULT_URI)
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()
    try:
        print(json.dumps(collect(args.uri, args.duration), indent=2, sort_keys=True))
    except BaseException as error:
        raise SystemExit(f"CrazySim telemetry validation failed: {error}") from error


if __name__ == "__main__":
    main()
