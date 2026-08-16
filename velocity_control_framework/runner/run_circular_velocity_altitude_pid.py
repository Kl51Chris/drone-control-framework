from __future__ import annotations

import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import (
    SyncCrazyflie,
)
from cflib.utils import uri_helper

from velocity_control_framework.backends import (
    CrazyflieCommandAdapter,
    CrazyflieStateProvider,
)
from velocity_control_framework.controllers import (
    VelocityAltitudePIDConfig,
    VelocityAltitudePIDController,
)
from velocity_control_framework.runner import (
    CircularVelocityAltitudePIDProgram,
)
from velocity_control_framework.references import (
    CircularFlightConfig,
    CircularFlightReference,
)
from velocity_control_framework.runner import (
    UniversalRunner,
    UniversalRunnerConfig,
)


URI = uri_helper.uri_from_env(
    default="radio://0/80/2M"
)


def build_controller(
) -> VelocityAltitudePIDController:
    """
    Controller tuning belongs to the controller construction,
    not to UniversalRunner.
    """
    config = VelocityAltitudePIDConfig(
        kp_vx=1.0,
        kp_vy=1.0,

        kp_z=1.1,
        ki_z=0.2,
        kd_z=0.55,

        hover_command=31500,

        max_tilt=math.radians(8.0),

        min_thrust=0.0,
        max_thrust=0.62,

        integral_limit_z=0.50,
    )

    return VelocityAltitudePIDController(
        config=config
    )


def build_circular_reference(
    initial_state,
    start_time: float,
) -> CircularFlightReference:
    """
    Trajectory tuning belongs to the reference construction,
    not to UniversalRunner.
    """
    config = CircularFlightConfig(
        takeoff_height=0.30,

        circle_radius=0.25,
        circle_revolutions=1.0,

        armed_idle_duration=0.50,
        takeoff_duration=10.00,
        circle_duration=15.00,
        landing_duration=4.00,
    )

    return CircularFlightReference(
        initial_position=(
            initial_state.position
        ),
        initial_yaw=initial_state.yaw,
        start_time=start_time,
        config=config,
    )


def log_control_step(
    elapsed,
    phase,
    state,
    reference,
    command,
    dt,
) -> None:
    print(
        f"[{elapsed:7.2f}s]"
        f" [{phase:10}]"
        f" dt={dt * 1000.0:5.1f}ms"

        f" x={state.position[0]:+.3f}"
        f" y={state.position[1]:+.3f}"
        f" z={state.position[2]:+.3f}"

        f" x_ref={reference.position[0]:+.3f}"
        f" y_ref={reference.position[1]:+.3f}"
        f" z_ref={reference.position[2]:+.3f}"

        f" vx={state.velocity[0]:+.3f}"
        f" vy={state.velocity[1]:+.3f}"
        f" vz={state.velocity[2]:+.3f}"

        f" vx_ref={reference.velocity[0]:+.3f}"
        f" vy_ref={reference.velocity[1]:+.3f}"
        f" vz_ref={reference.velocity[2]:+.3f}"

        f" roll_cmd="
        f"{math.degrees(command.roll):+.2f}deg"

        f" pitch_cmd="
        f"{math.degrees(command.pitch):+.2f}deg"

        f" thrust={command.thrust:.4f}"
    )


def run_flight(
    crazyflie,
) -> None:
    state_provider = (
        CrazyflieStateProvider(
            crazyflie
        )
    )

    command_adapter = (
        CrazyflieCommandAdapter(
            crazyflie
        )
    )

    controller = build_controller()

    flight_program = (
        CircularVelocityAltitudePIDProgram(
            controller=controller,
            reference_factory=(
                build_circular_reference
            ),
        )
    )

    runner = UniversalRunner(
        state_provider=state_provider,
        command_adapter=command_adapter,
        flight_program=flight_program,

        arm=lambda: (
            crazyflie
            .supervisor
            .send_arming_request(True)
        ),

        disarm=lambda: (
            crazyflie
            .supervisor
            .send_arming_request(False)
        ),

        config=UniversalRunnerConfig(
            control_frequency_hz=50.0,
            log_frequency_hz=10.0,
            state_wait_timeout_s=5.0,
            arming_wait_s=1.0,
        ),

        log_callback=log_control_step,
    )

    runner.run()


def main() -> None:
    cflib.crtp.init_drivers()

    print(
        f"[CONNECT] Connecting to {URI}"
    )

    crazyflie = Crazyflie(
        rw_cache="./cache"
    )

    with SyncCrazyflie(
        URI,
        cf=crazyflie,
    ) as sync_crazyflie:
        print(
            "[CONNECT] Connected"
        )

        run_flight(
            sync_crazyflie.cf
        )

    print(
        "[CONNECT] Disconnected"
    )


if __name__ == "__main__":
    main()