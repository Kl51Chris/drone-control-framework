"""Explicit, separate CrazySim supervisor emergency-stop diagnostic."""

from __future__ import annotations

import argparse
import json
from typing import Any

from tools.diagnose_crazysim_backend import (
    CRAZYSIM_URI,
    supervisor_status,
    validate_crazysim_uri,
)
from tools.crazysim_session import PassiveCrazyflie
from velocity_control_framework.backends import CrazySimSupervisorAdapter


def execute_connected(cf: Any, supervisor: Any | None = None) -> dict[str, Any]:
    read_status = supervisor.status if supervisor is not None else lambda: supervisor_status(cf)
    before = read_status()
    if before["is_armed"] or before["is_flying"]:
        raise RuntimeError("emergency-stop diagnostic requires disarmed SITL")
    try:
        cf.commander.send_stop_setpoint()
        if supervisor is None:
            cf.supervisor.send_emergency_stop()
        else:
            supervisor.send_emergency_stop()
        after = read_status()
        return {"supervisor_before": before, "supervisor_after": after}
    finally:
        try:
            cf.commander.send_stop_setpoint()
        finally:
            if supervisor is None:
                cf.supervisor.send_arming_request(False)
            else:
                supervisor.send_arming_request(False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=CRAZYSIM_URI)
    parser.add_argument("--execute-emergency-stop-test", action="store_true")
    parser.add_argument("--i-understand-sitl-may-lock", action="store_true")
    args = parser.parse_args()
    validate_crazysim_uri(args.uri)
    if not (args.execute_emergency_stop_test and args.i_understand_sitl_may_lock):
        raise SystemExit(
            "refusing emergency-stop test: both explicit acknowledgement flags are required"
        )
    import cflib.crtp
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    with SyncCrazyflie(args.uri, cf=PassiveCrazyflie(rw_cache="./cache")) as link:
        supervisor = CrazySimSupervisorAdapter(link.cf)
        try:
            supervisor.start()
            result = execute_connected(link.cf, supervisor)
        finally:
            supervisor.stop()
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
