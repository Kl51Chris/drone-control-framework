"""Test two passive CFLib sessions against one surviving CrazySim process."""

from __future__ import annotations

import argparse
import json
import socket
import time
from collections.abc import Callable
from typing import Any

from tools.crazysim_session import PassiveCrazyflie
from tools.diagnose_crazysim_backend import CRAZYSIM_URI, validate_crazysim_uri
from velocity_control_framework.backends import CrazyflieStateProvider


def udp_endpoint_responds(host: str = "127.0.0.1", port: int = 19850) -> bool:
    """Use CrazySim's documented 0xff probe; it is not a commander packet."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.settimeout(0.25)
        probe.sendto(b"\xff", (host, port))
        data, _ = probe.recvfrom(64)
        return data == b"\xff"
    except (OSError, socket.timeout):
        return False
    finally:
        probe.close()


def run_cycle(uri: str, cycle: int) -> dict[str, Any]:
    import cflib.crtp
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    cf = PassiveCrazyflie(rw_cache="./cache")
    started = time.monotonic()
    with SyncCrazyflie(uri, cf=cf) as link:
        provider = CrazyflieStateProvider(link.cf, delete_log_blocks_on_stop=True)
        try:
            provider.start()
            state = provider.get_state()
            return {
                "cycle": cycle,
                "connected": True,
                "connection_and_state_s": time.monotonic() - started,
                "state_timestamp": state.timestamp,
            }
        finally:
            provider.stop()


def run_two_cycles(
    uri: str,
    cycle_runner: Callable[[str, int], dict[str, Any]] = run_cycle,
    endpoint_probe: Callable[[], bool] = udp_endpoint_responds,
    settle_s: float = 0.25,
    sleeper: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    validate_crazysim_uri(uri)
    results = []
    for cycle in (1, 2):
        if cycle == 2 and settle_s > 0.0:
            # CrazySim has no disconnect signal and retains bounded forwarding
            # queues. Let late replies drain to the closed first client before
            # the second client's first packet replaces the destination.
            sleeper(settle_s)
        try:
            results.append(cycle_runner(uri, cycle))
        except BaseException as error:
            alive = endpoint_probe()
            results.append({
                "cycle": cycle,
                "connected": False,
                "error": f"{type(error).__name__}: {error}",
                "udp_bridge_responded_after_failure": alive,
            })
            return {
                "uri": uri,
                "cycles": results,
                "classification": (
                    "bridge_alive_but_cflib_session_failed"
                    if alive else "simulator_or_udp_bridge_not_running"
                ),
            }
    return {"uri": uri, "cycles": results, "classification": "reconnect_supported"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=CRAZYSIM_URI)
    args = parser.parse_args()
    print(json.dumps(run_two_cycles(args.uri), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
