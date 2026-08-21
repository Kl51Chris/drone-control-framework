import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


URI = "radio://0/80/2M"

TARGET_HEIGHT_M = 0.30
HOVER_DURATION_S = 5.0
COMMAND_PERIOD_S = 0.05


def reset_estimator(cf):
    """Reset the onboard Kalman estimator."""
    print("[INFO] Resetting estimator...")

    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)

    cf.param.set_value("kalman.resetEstimation", "0")
    time.sleep(2.0)

    print("[INFO] Estimator reset complete.")


def arm(cf):
    """Request arming through the Crazyflie supervisor."""
    print("[INFO] Sending ARM request...")

    cf.supervisor.send_arming_request(True)
    time.sleep(1.0)

    print("[INFO] ARM request sent.")


def disarm(cf):
    """Request normal disarming through the Crazyflie supervisor."""
    print("[INFO] Sending DISARM request...")

    try:
        # First stop the active commander setpoint.
        cf.commander.send_stop_setpoint()
        time.sleep(0.1)

        # Then request disarming.
        cf.supervisor.send_arming_request(False)
        time.sleep(0.2)

        print("[INFO] DISARM request sent.")

    except Exception as error:
        print(f"[WARN] Could not complete normal disarm: {error}")


def emergency_stop(cf):
    """
    Trigger the supervisor-level emergency stop.

    WARNING:
    This is latching. The Crazyflie must be rebooted before it can fly again.
    """
    print()
    print("======================================")
    print("!!! SUPERVISOR EMERGENCY STOP !!!")
    print("!!! MOTORS WILL STOP IMMEDIATELY  !!!")
    print("======================================")

    try:
        # Send several times because the command has no acknowledgement.
        for _ in range(3):
            cf.supervisor.send_emergency_stop()
            time.sleep(0.05)

        print("[EMERGENCY] E-stop command sent.")
        print("[EMERGENCY] Reboot the Crazyflie before the next test.")

    except Exception as error:
        print(f"[ERROR] Could not send supervisor E-stop: {error}")
        print("[EMERGENCY] Disconnect the battery immediately.")


def send_hover_command(cf, height_m):
    """Send one hover setpoint."""
    cf.commander.send_hover_setpoint(
        0.0,       # vx, body-frame forward velocity in m/s
        0.0,       # vy, body-frame sideways velocity in m/s
        0.0,       # yaw rate in degrees/s
        height_m,  # height above the surface in m
    )


def takeoff(cf, target_height_m=TARGET_HEIGHT_M):
    """Ramp the height command from 5 cm to the target height."""
    print(f"[INFO] Taking off to {target_height_m:.2f} m...")

    height_steps = [
        0.05,
        0.10,
        0.15,
        0.20,
        0.25,
        target_height_m,
    ]

    for height_m in height_steps:
        for _ in range(10):
            send_hover_command(cf, height_m)
            time.sleep(COMMAND_PERIOD_S)

    print("[INFO] Takeoff ramp complete.")


def hover(
    cf,
    height_m=TARGET_HEIGHT_M,
    duration_s=HOVER_DURATION_S,
):
    """Hold the requested hover height."""
    print(
        f"[INFO] Hovering at {height_m:.2f} m "
        f"for {duration_s:.1f} seconds..."
    )

    steps = int(duration_s / COMMAND_PERIOD_S)

    for _ in range(steps):
        send_hover_command(cf, height_m)
        time.sleep(COMMAND_PERIOD_S)

    print("[INFO] Hover complete.")


def land(cf, start_height_m=TARGET_HEIGHT_M):
    """Ramp the requested height down and stop the motors."""
    print("[INFO] Landing...")

    height_steps = [
        start_height_m,
        0.25,
        0.20,
        0.15,
        0.10,
        0.05,
    ]

    for height_m in height_steps:
        for _ in range(10):
            send_hover_command(cf, height_m)
            time.sleep(COMMAND_PERIOD_S)

    # Stop producing thrust after the landing ramp.
    cf.commander.send_stop_setpoint()
    time.sleep(0.2)

    # Lower this commander's setpoint priority.
    cf.commander.send_notify_setpoint_stop()
    time.sleep(0.1)

    print("[INFO] Landing sequence complete.")


def countdown(seconds=5):
    """Provide time to abort before connecting and arming."""
    print("======================================")
    print("Crazyflie hover test")
    print("Keep one hand near the battery plug.")
    print("Press Ctrl+C for SUPERVISOR E-STOP.")
    print("======================================")

    for remaining in range(seconds, 0, -1):
        print(f"[INFO] Starting in {remaining}...")
        time.sleep(1)


def run_hover_test():
    cflib.crtp.init_drivers()

    countdown(5)

    print(f"[INFO] Connecting to {URI}...")

    with SyncCrazyflie(
        URI,
        cf=Crazyflie(rw_cache="./cache"),
    ) as scf:

        cf = scf.cf
        armed = False
        emergency_stop_triggered = False

        print("[OK] Connected.")

        try:
            reset_estimator(cf)

            print()
            print("[WARNING] ARMING THE CRAZYFLIE")
            print("[WARNING] Propellers may begin moving.")
            print()

            arm(cf)
            armed = True

            takeoff(cf)
            hover(cf)
            land(cf)

            disarm(cf)
            armed = False

            print("[INFO] Hover test finished normally.")

        except KeyboardInterrupt:
            emergency_stop_triggered = True

            print("\n[INFO] Ctrl+C detected.")
            emergency_stop(cf)

            # E-stop itself disables the motors.
            armed = False

        except Exception as error:
            emergency_stop_triggered = True

            print(f"\n[ERROR] Flight test failed: {error}")
            emergency_stop(cf)

            armed = False

        finally:
            # Only do a normal disarm when an E-stop was not triggered.
            # After E-stop, the supervisor is latched until reboot.
            if armed and not emergency_stop_triggered:
                disarm(cf)

            print("[INFO] Closing Crazyflie connection.")


if __name__ == "__main__":
    try:
        run_hover_test()

    except KeyboardInterrupt:
        # This catches Ctrl+C during countdown or before connection.
        print("\n[INFO] Test aborted before flight.")

    except Exception as error:
        # This catches connection failures where no cf object exists.
        print(f"[ERROR] Could not run hover test: {error}")

    finally:
        print("[INFO] Program exited.")