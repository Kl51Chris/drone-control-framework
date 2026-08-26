from __future__ import annotations

import time
from collections.abc import Callable

from velocity_control_framework.backends import CommandAdapter, StateProvider
from velocity_control_framework.interfaces import Controller, Reference


def run_control_loop(
    state_provider: StateProvider,
    command_adapter: CommandAdapter,
    controller: Controller,
    reference_provider: Callable[[], Reference],
    frequency_hz: float,
    duration_s: float | None = None,
) -> None:
    """
    Run the backend-independent controller loop.

    The loop contains no Crazyflie-, PID-, LQR-, or MPC-specific logic.
    """
    if frequency_hz <= 0.0:
        raise ValueError("frequency_hz must be positive")

    if duration_s is not None and duration_s <= 0.0:
        raise ValueError("duration_s must be positive")

    period = 1.0 / frequency_hz

    state_provider.start()
    controller.reset()

    start_time = time.monotonic()
    previous_time = start_time
    next_time = start_time

    try:
        while True:
            now = time.monotonic()

            if duration_s is not None and now - start_time >= duration_s:
                break

            state = state_provider.get_state()
            reference = reference_provider()

            dt = now - previous_time
            previous_time = now

            if dt <= 0.0:
                dt = period

            command = controller.update(
                state=state,
                reference=reference,
                dt=dt,
            )

            command_adapter.send(command)

            next_time += period
            sleep_time = next_time - time.monotonic()

            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                # The loop missed its deadline. Reset rather than trying
                # to execute multiple iterations immediately.
                next_time = time.monotonic()

    finally:
        command_adapter.stop()
        state_provider.stop()
