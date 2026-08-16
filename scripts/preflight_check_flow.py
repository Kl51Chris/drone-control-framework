# scripts/preflight_check_flow.py

import time
import sys
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


URI = "radio://0/80/2M"


def get_param_safe(cf, name, timeout=2.0):
    done = Event()
    result = {"value": None, "error": None}

    def cb(_name, value):
        result["value"] = value
        done.set()

    def err_cb(_name, msg):
        result["error"] = msg
        done.set()

    try:
        cf.param.add_update_callback(group=name.split(".")[0],
                                     name=name.split(".")[1],
                                     cb=cb)
        cf.param.request_param_update(name)
        done.wait(timeout)
        return result["value"], result["error"]
    except Exception as e:
        return None, str(e)


def add_var_safe(logconf, name, vartype):
    try:
        logconf.add_variable(name, vartype)
        return True
    except Exception as e:
        print(f"[WARN] Could not add log variable {name}: {e}")
        return False


def stop_safely(cf):
    for _ in range(20):
        try:
            cf.commander.send_stop_setpoint()
        except Exception:
            pass
        time.sleep(0.01)


def main():
    cflib.crtp.init_drivers()

    print(f"[INFO] Connecting to {URI}")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print("[OK] Connected")

        print("[INFO] Waiting briefly for params...")
        time.sleep(1.0)

        # Flow deck detection params. Names may differ across firmware versions.
        print("\n[CHECK] Deck parameters")
        candidates = [
            "deck.bcFlow2",
            "deck.bcFlow",
            "deck.bcZRanger2",
            "deck.bcZRanger",
        ]

        found_any_deck_signal = False
        for p in candidates:
            value, err = get_param_safe(cf, p)
            if value is not None:
                print(f"[PARAM] {p} = {value}")
                if str(value) not in ["0", "False", "false", ""]:
                    found_any_deck_signal = True
            else:
                print(f"[WARN] Could not read {p}: {err}")

        if found_any_deck_signal:
            print("[OK] At least one Flow/Z-ranger related deck parameter is active.")
        else:
            print("[WARN] No active Flow/Z-ranger deck parameter detected.")
            print("       This may mean the deck is not detected, firmware is old, or parameter names differ.")
            print("       Use cfclient console/Parameters tab to confirm deck detection if needed.")

        print("\n[CHECK] Logging basic state")
        logconf = LogConfig(name="Preflight", period_in_ms=100)

        # Stabilizer attitude
        add_var_safe(logconf, "stabilizer.roll", "float")
        add_var_safe(logconf, "stabilizer.pitch", "float")
        add_var_safe(logconf, "stabilizer.yaw", "float")

        # Battery
        add_var_safe(logconf, "pm.vbat", "float")

        # Range / flow variables. These can vary depending on firmware.
        add_var_safe(logconf, "range.zrange", "uint16_t")
        add_var_safe(logconf, "motion.deltaX", "int16_t")
        add_var_safe(logconf, "motion.deltaY", "int16_t")

        samples = []

        def log_data(timestamp, data, logconf):
            samples.append(data)
            msg = "[LOG] "
            for key, value in data.items():
                msg += f"{key}={value} "
            print(msg)

        def log_error(logconf, msg):
            print(f"[ERROR] Log error: {msg}")

        cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_data)
        logconf.error_cb.add_callback(log_error)

        try:
            logconf.start()
            print("[INFO] Collecting 30 samples. Move the drone gently by hand while props are OFF.")
            print("[INFO] Do not arm. Do not spin motors.")
            time.sleep(3.0)
            logconf.stop()
        finally:
            stop_safely(cf)

        print("\n[SUMMARY]")
        if len(samples) > 0:
            print(f"[OK] Received {len(samples)} log samples.")
        else:
            print("[FAIL] No log samples received.")
            sys.exit(1)

        last = samples[-1]
        if "pm.vbat" in last:
            print(f"[INFO] Battery voltage last sample: {last['pm.vbat']:.2f} V")
            if last["pm.vbat"] < 3.6:
                print("[WARN] Battery looks low. Charge before flight.")

        if "range.zrange" in last:
            print(f"[INFO] range.zrange last sample: {last['range.zrange']}")
            print("[NOTE] range.zrange is usually in mm-like units depending on firmware path.")
            print("       Put the drone on a non-reflective floor and check that this changes with height.")

        print("[DONE] Preflight check completed. No flight commands were sent.")


if __name__ == "__main__":
    main()
