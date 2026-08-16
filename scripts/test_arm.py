import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "radio://0/80/2M"


def print_state(cf, title=""):
    states = cf.supervisor.read_state_list()

    print("\n======================")
    print(title)
    print("======================")

    print("States:", states)
    print("Can arm :", cf.supervisor.can_be_armed)
    print("Can fly :", cf.supervisor.can_fly)
    print("Armed   :", cf.supervisor.is_armed)
    print("Flying  :", cf.supervisor.is_flying)
    print("Locked  :", cf.supervisor.is_locked)
    print("Crash   :", cf.supervisor.is_crashed)
    print()


def main():
    cflib.crtp.init_drivers()

    print(f"Connecting to {URI}")

    with SyncCrazyflie(
        URI,
        cf=Crazyflie(rw_cache="./cache"),
    ) as scf:

        cf = scf.cf

        print("Connected.")
        time.sleep(1)

        print_state(cf, "Before Arm")

        print("Sending ARM request...")
        cf.supervisor.send_arming_request(True)

        time.sleep(1.0)

        print_state(cf, "After Arm")

        print("Waiting 5 seconds...")
        time.sleep(5)

        print("Disarming...")
        cf.supervisor.send_arming_request(False)

        time.sleep(1)

        print_state(cf, "After Disarm")


if __name__ == "__main__":
    main()
