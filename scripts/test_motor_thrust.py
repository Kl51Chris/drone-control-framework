import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


URI = "radio://0/80/2M"

# Conservative first test.
# send_setpoint thrust range is 0 to 65535.
TEST_THRUST = 10_000

COMMAND_RATE_HZ = 100
RAMP_TIME_S = 1.0
HOLD_TIME_S = 2.0

# Refuse suspiciously large values in this bench-test script.
MAX_ALLOWED_TEST_THRUST = 15_000


def send_setpoint(cf, thrust: int) -> None:
    """Send a level attitude-rate setpoint with the specified collective thrust."""
    cf.commander.send_setpoint(
        roll=0.0,
        pitch=0.0,
        yawrate=0.0,
        thrust=thrust,
    )


def stop_and_disarm(cf) -> None:
    """Repeatedly stop the commander, then explicitly disarm."""
    print("[STOP] Sending stop setpoints...")

    for _ in range(20):
        try:
            cf.commander.send_stop_setpoint()
        except Exception:
            pass
        time.sleep(0.01)

    print("[DISARM] Sending disarm request...")

    try:
        cf.supervisor.send_arming_request(False)
        time.sleep(0.5)
    except Exception as exc:
        print(f"[WARN] Disarm request failed: {exc}")


def main() -> None:
    if not 0 <= TEST_THRUST <= MAX_ALLOWED_TEST_THRUST:
        raise ValueError(
            f"TEST_THRUST must be between 0 and "
            f"{MAX_ALLOWED_TEST_THRUST}; got {TEST_THRUST}"
        )

    period_s = 1.0 / COMMAND_RATE_HZ
    ramp_steps = max(1, int(RAMP_TIME_S * COMMAND_RATE_HZ))
    hold_steps = max(1, int(HOLD_TIME_S * COMMAND_RATE_HZ))

    cflib.crtp.init_drivers()

    print(f"[INFO] Connecting to {URI}")

    with SyncCrazyflie(
        URI,
        cf=Crazyflie(rw_cache="./cache"),
    ) as scf:
        cf = scf.cf

        print("[OK] Connected")
        time.sleep(1.0)

        print(f"[STATE] Before arm: {cf.supervisor.read_state_list()}")

        if not cf.supervisor.can_be_armed:
            raise RuntimeError(
                "Crazyflie cannot currently be armed. "
                f"Supervisor state: {cf.supervisor.read_state_list()}"
            )

        print("[ARM] Sending arming request...")
        cf.supervisor.send_arming_request(True)
        time.sleep(0.8)

        print(f"[STATE] After arm: {cf.supervisor.read_state_list()}")

        if not cf.supervisor.is_armed:
            raise RuntimeError("Arming request was not accepted.")

        try:
            print("[COMMANDER] Sending zero-thrust setpoints...")

            # Establish the low-level commander stream before applying thrust.
            for _ in range(20):
                send_setpoint(cf, 0)
                time.sleep(period_s)

            print(f"[RAMP] Increasing command to {TEST_THRUST}...")

            for step in range(ramp_steps):
                fraction = (step + 1) / ramp_steps
                thrust = int(TEST_THRUST * fraction)

                send_setpoint(cf, thrust)
                time.sleep(period_s)

            print(f"[HOLD] Holding {TEST_THRUST} for {HOLD_TIME_S:.1f} s...")

            for _ in range(hold_steps):
                send_setpoint(cf, TEST_THRUST)
                time.sleep(period_s)

            print("[RAMP] Returning thrust command to zero...")

            for step in range(ramp_steps):
                fraction = 1.0 - ((step + 1) / ramp_steps)
                thrust = max(0, int(TEST_THRUST * fraction))

                send_setpoint(cf, thrust)
                time.sleep(period_s)

            print("[DONE] Motor-thrust command test completed.")

        except KeyboardInterrupt:
            print("\n[CTRL+C] Interrupt received.")

        finally:
            stop_and_disarm(cf)
            print(f"[STATE] Final: {cf.supervisor.read_state_list()}")


if __name__ == "__main__":
    main()
