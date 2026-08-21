import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "radio://0/80/2M"

logging.basicConfig(level=logging.INFO)

def main():
    cflib.crtp.init_drivers()

    print(f"Connecting to {URI} ...")

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        print("Connected!")
        print("Waiting for parameters...")
        scf.wait_for_params()
        print("Parameters downloaded. Link is alive.")

        # 保持连接 5 秒，方便你观察 drone LED
        for i in range(5):
            print(f"Link alive: {i + 1}/5")
            time.sleep(1)

    print("Disconnected.")

if __name__ == "__main__":
    main()
