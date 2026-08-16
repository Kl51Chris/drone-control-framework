# scripts/estop_test.py

import time
import cflib.crtp

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


URI = "radio://0/80/2M"


def normal_stop(cf):
    print("[STOP] Sending commander stop setpoints...")
    for _ in range(50):
        try:
            cf.commander.send_stop_setpoint()
        except Exception as e:
            print(f"[WARN] send_stop_setpoint failed: {e}")
        time.sleep(0.01)


def hard_estop(cf):
    print("[E-STOP] Sending supervisor emergency stop.")
    print("[E-STOP] This latches until reboot.")
    try:
        cf.supervisor.send_emergency_stop()
    except Exception as e:
        print(f"[WARN] supervisor emergency stop failed: {e}")

    normal_stop(cf)


def main():
    cflib.crtp.init_drivers()

    print(f"[INFO] Connecting to {URI}")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print("[OK] Connected")
        print("[TEST] Props should be OFF for this test.")
        print("[TEST] Press Ctrl+C now. The script should enter the stop path.")
        print("[TEST] Waiting...")

        try:
            while True:
                time.sleep(0.25)
        except KeyboardInterrupt:
            print("\n[CTRL+C] KeyboardInterrupt received.")
            normal_stop(cf)
            print("[OK] Stop path tested.")


if __name__ == "__main__":
    main()
