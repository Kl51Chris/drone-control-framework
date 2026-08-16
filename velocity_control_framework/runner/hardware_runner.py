from __future__ import annotations

import time
import math
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from velocity_control_framework.backends import (
    CrazyflieCommandAdapter,
    CrazyflieStateProvider,
)
from velocity_control_framework.controllers import (
    VelocityAltitudePIDConfig,
    VelocityAltitudePIDController,
)
from velocity_control_framework.interfaces import AttitudeThrustCommand
from velocity_control_framework.references import (
    FlightState,
    VerticalFlightConfig,
    VerticalFlightReference,
)


URI = uri_helper.uri_from_env(
    default="radio://0/80/2M"
)

CONTROL_FREQUENCY_HZ = 50.0
CONTROL_PERIOD_S = 1.0 / CONTROL_FREQUENCY_HZ

LOG_FREQUENCY_HZ = 10.0
LOG_PERIOD_S = 1.0 / LOG_FREQUENCY_HZ

STATE_WAIT_TIMEOUT_S = 5.0
ARMING_WAIT_S = 1.0


def wait_for_first_state(
    state_provider: CrazyflieStateProvider,
    timeout_s: float,
):
    """
    Wait until the StateProvider has received one complete state.

    Raises:
        TimeoutError:
            If no state is available within timeout_s.
    """
    deadline = time.monotonic() + timeout_s
    last_error: RuntimeError | None = None

    while time.monotonic() < deadline:
        try:
            return state_provider.get_state()
        except RuntimeError as error:
            last_error = error
            time.sleep(0.05)

    raise TimeoutError(
        "No valid Crazyflie state was received "
        f"within {timeout_s:.1f} seconds"
    ) from last_error


def run_vertical_flight(
    crazyflie,
) -> None:
    """
    Execute one idle-takeoff-hover-landing sequence.

    All flight commands use AttitudeThrustCommand and send_setpoint().
    """
    state_provider = CrazyflieStateProvider(
        crazyflie
    )

    command_adapter = CrazyflieCommandAdapter(
        crazyflie
    )

    controller = VelocityAltitudePIDController(
    VelocityAltitudePIDConfig(
        kp_vx=1.0,
        kp_vy=1.0,

        kp_z=0.8,
        ki_z=0.2,
        kd_z=0.25,

        hover_command=31500,

        max_tilt=math.radians(8.0),

        min_thrust=0.0,
        max_thrust=0.62,

        integral_limit_z=0.50,
    )
)

    flight_config = VerticalFlightConfig(
        takeoff_height=0.50,
        armed_idle_duration=0.50,
        takeoff_duration=3.00,
        hover_duration=6.00,
        landing_duration=3.00,
    )

    state_provider.start()

    is_armed = False

    try:
        print("[STATE] Waiting for estimator data")

        initial_state = wait_for_first_state(
            state_provider=state_provider,
            timeout_s=STATE_WAIT_TIMEOUT_S,
        )

        print(
            "[STATE] Initial state received:"
            f" z={initial_state.position[2]:.3f} m,"
            f" vz={initial_state.velocity[2]:.3f} m/s,"
            f" yaw={initial_state.yaw:.3f} rad"
        )

        # Arm through the supervisor. Arming remains a runner
        # responsibility, not a controller responsibility.
        crazyflie.supervisor.send_arming_request(True)
        is_armed = True

        print("[STATE] Arming requested")
        time.sleep(ARMING_WAIT_S)

        controller.reset()

        flight_start_time = time.monotonic()

        reference_plan = VerticalFlightReference(
            initial_position=initial_state.position,
            initial_yaw=initial_state.yaw,
            start_time=flight_start_time,
            config=flight_config,
        )

        print(
            "[PLAN]"
            f" initial_z={reference_plan.initial_altitude:.3f} m,"
            f" target_z={reference_plan.target_altitude:.3f} m"
        )

        previous_time = flight_start_time
        next_deadline = flight_start_time

        previous_flight_state: FlightState | None = None
        last_log_time = flight_start_time

        while True:
            now = time.monotonic()

            flight_state = reference_plan.get_state(now)

            if flight_state is not previous_flight_state:
                print(f"[STATE] {flight_state.name}")
                previous_flight_state = flight_state

            if flight_state is FlightState.COMPLETE:
                break

            state = state_provider.get_state()
            reference = reference_plan.get_reference(now)

            dt = now - previous_time
            previous_time = now

            if dt <= 0.0:
                dt = CONTROL_PERIOD_S

            if flight_state is FlightState.ARMED_IDLE:
                # Required zero-thrust command before ordinary
                # low-level thrust commands are accepted.
                command = AttitudeThrustCommand.zero()
            else:
                command = controller.update(
                    state=state,
                    reference=reference,
                    dt=dt,
                )

            command_adapter.send(command)

            if now - last_log_time >= LOG_PERIOD_S:
                elapsed = now - flight_start_time

                print(
                    f"[{elapsed:6.2f}s]"
                    f" [{flight_state.name:11}]"
                    f" vx={state.velocity[0]:+.3f}"
                    f" vy={state.velocity[1]:+.3f}"
                    f" roll_cmd={math.degrees(command.roll):+.2f}deg"
                    f" pitch_cmd={math.degrees(command.pitch):+.2f}deg"
                    f" roll={math.degrees(state.roll):+.2f}deg"
                    f" pitch={math.degrees(state.pitch):+.2f}deg"
                    f" z={state.position[2]:+.3f}"
                    f" z_ref={reference.position[2]:+.3f}"
                    f" vz={state.velocity[2]:+.3f}"
                    f" vz_ref={reference.velocity[2]:+.3f}"
                    f" thrust={command.thrust:.4f}"
                )

                last_log_time = now

            next_deadline += CONTROL_PERIOD_S
            sleep_duration = (
                next_deadline
                - time.monotonic()
            )

            if sleep_duration > 0.0:
                time.sleep(sleep_duration)
            else:
                # Do not run several delayed iterations back-to-back.
                next_deadline = time.monotonic()

        print("\n[STATE] Flight plan complete")

    finally:
        print("\n[STOP] Stopping command output")

        try:
            command_adapter.stop()
        finally:
            if is_armed:
                crazyflie.supervisor.send_arming_request(
                    False
                )
                print("[STOP] Disarming requested")

            state_provider.stop()
            print("[STOP] State provider stopped")


def main() -> None:
    cflib.crtp.init_drivers()

    print(f"[CONNECT] Connecting to {URI}")

    crazyflie = Crazyflie(
        rw_cache="./cache"
    )

    with SyncCrazyflie(
        URI,
        cf=crazyflie,
    ) as sync_crazyflie:
        print("[CONNECT] Connected")

        run_vertical_flight(
            sync_crazyflie.cf
        )

    print("[CONNECT] Disconnected")


if __name__ == "__main__":
    main()