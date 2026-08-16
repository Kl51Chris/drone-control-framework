# scripts/first_hover_flowdeck.py

import time
import sys
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander


URI = "radio://0/80/2M"

DEFAULT_HEIGHT_M = 0.20
HOVER_TIME_S = 2.0

# False: Ctrl+C uses commander stop only.
# True: Ctrl+C uses supervisor emergency stop, which latches until reboot.
HARD_ESTOP_ON_CTRL_C = False


def get_param_safe(cf, name, timeout=2.0):
    done = Event()
    result = {"value": None, "error": None}

    def cb(_name, value):
        result["value"] = value
        done.set()

    try:
        group, var = name.split(".", 1)
        cf.param.add_update_callback(group=group, name=var, cb=cb)
        cf.param.request_param_update(name)
        done.wait(timeout)
        return result["value"]
    except Exception:
        return None


def normal_stop(cf):
    print("[STOP] Sending commander stop setpoints.")
    for _ in range(50):
        try:
            cf.commander.send_stop_setpoint()
        except Exception:
            pass
        time.sleep(0.01)


def hard_estop(cf):
    print("[E-STOP] Sending supervisor emergency stop. This latches until reboot.")
    try:
        cf.supervisor.send_emergency_stop()
    except Exception as e:
        print(f"[WARN] supervisor emergency stop failed: {e}")
    normal_stop(cf)


def stop_on_interrupt(cf):
    if HARD_ESTOP_ON_CTRL_C:
        hard_estop(cf)
    else:
        normal_stop(cf)


def check_flow_deck_hint(cf):
    print("[CHECK] Checking Flow deck detection hints...")
    candidates = ["deck.bcFlow2", "deck.bcFlow", "deck.bcZRanger2", "deck.bcZRanger"]

    active = []
    for p in candidates:
        value = get_param_safe(cf, p)
        if value is not None:
            print(f"[PARAM] {p} = {value}")
            if str(value) not in ["0", "False", "false", ""]:
                active.append(p)

    if active:
        print(f"[OK] Active deck-related params: {active}")
        return True

    print("[WARN] Could not confirm active Flow deck from params.")
    print("[WARN] If preflight_check_flow.py showed range/flow logs, you may still proceed cautiously.")
    print("[WARN] Otherwise do NOT fly.")
    return False


def main():
    cflib.crtp.init_drivers()

    print(f"[INFO] Connecting to {URI}")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print("[OK] Connected")

        time.sleep(1.0)

        deck_ok = check_flow_deck_hint(cf)
        if not deck_ok:
            print("[ABORT] Flow deck was not confidently detected.")
            print("[ABORT] Run scripts/preflight_check_flow.py and confirm range.zrange / motion logs first.")
            sys.exit(1)

        print("\n[FLIGHT] First hover test")
        print(f"[FLIGHT] Target height: {DEFAULT_HEIGHT_M:.2f} m")
        print(f"[FLIGHT] Hover time: {HOVER_TIME_S:.1f} s")
        print("[FLIGHT] Keep one hand ready for Ctrl+C.")
        print("[FLIGHT] Taking off in 3 seconds...")
        time.sleep(3.0)

        try:
            with MotionCommander(scf, default_height=DEFAULT_HEIGHT_M) as mc:
                print("[FLIGHT] Takeoff command sent.")
                time.sleep(HOVER_TIME_S)

                print("[FLIGHT] Landing.")
                mc.land()
                time.sleep(1.0)

            print("[DONE] First hover test completed.")

        except KeyboardInterrupt:
            print("\n[CTRL+C] Interrupt received.")
            stop_on_interrupt(cf)
            print("[DONE] Stop path executed after Ctrl+C.")

        except Exception as e:
            print(f"\n[ERROR] Flight exception: {e}")
            normal_stop(cf)
            print("[DONE] Stop path executed after exception.")
            raise


if __name__ == "__main__":
    main()
