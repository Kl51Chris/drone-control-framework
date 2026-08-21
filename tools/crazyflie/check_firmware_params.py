import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "radio://0/80/2M"

def read_param(cf, name, timeout=2.0):
    done = Event()
    result = {"value": None}

    def cb(_name, value):
        result["value"] = value
        done.set()

    try:
        group, var = name.split(".", 1)
        cf.param.add_update_callback(group=group, name=var, cb=cb)
        cf.param.request_param_update(name)
        done.wait(timeout)
        return result["value"]
    except Exception as e:
        return f"ERROR: {e}"

def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        time.sleep(1.0)

        candidates = [
            "firmware.revision0",
            "firmware.revision1",
            "firmware.modified",
            "firmware.hash",
            "platform.name",
            "deck.bcFlow2",
            "deck.bcFlow",
        ]

        for p in candidates:
            print(f"{p}: {read_param(cf, p)}")

if __name__ == "__main__":
    main()
