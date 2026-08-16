from __future__ import annotations

import time

import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from velocity_control_framework.backends import (
    CrazyflieStateProvider,
)


URI = "radio://0/80/2M"

FREQUENCY_HZ = 10.0
DURATION_S = 10.0


def format_vector(vector: np.ndarray) -> str:
    return (
        f"[{vector[0]: .4f}, "
        f"{vector[1]: .4f}, "
        f"{vector[2]: .4f}]"
    )


def main() -> None:
    cflib.crtp.init_drivers()

    cf = Crazyflie(
        rw_cache="./cache",
    )

    with SyncCrazyflie(URI, cf=cf) as scf:
        print(f"Connected to {URI}")

        state_provider = CrazyflieStateProvider(
            scf.cf
        )

        period_s = 1.0 / FREQUENCY_HZ
        end_time = time.monotonic() + DURATION_S

        try:
            print("Starting estimator-state logging...")

            state_provider.start()

            print("First complete state received")
            print()
            print(
                "time       "
                "position [m]                    "
                "velocity [m/s]                  "
                "roll     pitch    yaw"
            )

            while time.monotonic() < end_time:
                loop_start = time.monotonic()

                state = state_provider.get_state()

                print(
                    f"{state.timestamp:10.3f}  "
                    f"{format_vector(state.position)}  "
                    f"{format_vector(state.velocity)}  "
                    f"{np.rad2deg(state.roll):8.3f} "
                    f"{np.rad2deg(state.pitch):8.3f} "
                    f"{np.rad2deg(state.yaw):8.3f}"
                )

                elapsed = time.monotonic() - loop_start
                sleep_time = period_s - elapsed

                if sleep_time > 0.0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nState check interrupted")

        finally:
            state_provider.stop()
            print("State provider stopped")


if __name__ == "__main__":
    main()