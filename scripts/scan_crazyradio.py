import cflib.crtp

cflib.crtp.init_drivers()
print("Scanning interfaces...")

available = cflib.crtp.scan_interfaces()

if available:
    for uri, info in available:
        print(uri, info)
else:
    print("No Crazyflie found.")
    print("This can mean: radio works, but drone is off / wrong channel / not bound.")
