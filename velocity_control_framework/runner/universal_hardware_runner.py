from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable, Generic, Mapping, Protocol, TypeVar


StateT = TypeVar("StateT")
ReferenceT = TypeVar("ReferenceT")
CommandT = TypeVar("CommandT")


class StateProviderProtocol(
    Protocol[StateT],
):
    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def get_state(self) -> StateT:
        ...


class CommandAdapterProtocol(
    Protocol[CommandT],
):
    def send(
        self,
        command: CommandT,
    ) -> None:
        ...

    def stop(self) -> None:
        ...


class FlightProgramProtocol(
    Protocol[
        StateT,
        ReferenceT,
        CommandT,
    ],
):
    """
    Adapter between a reference, controller, and runner.

    A flight program owns no radio or timing loop. It only
    coordinates reference generation and controller execution.
    """

    def initialize(
        self,
        initial_state: StateT,
        start_time: float,
    ) -> None:
        ...

    def is_complete(
        self,
        current_time: float,
    ) -> bool:
        ...

    def get_phase_name(
        self,
        current_time: float,
    ) -> str:
        ...

    def update(
        self,
        state: StateT,
        current_time: float,
        dt: float,
    ) -> tuple[
        ReferenceT,
        CommandT,
    ]:
        ...


LogCallback = Callable[
    [
        float,
        str,
        StateT,
        ReferenceT,
        CommandT,
        float,
    ],
    None,
]


@dataclass(frozen=True, slots=True)
class UniversalRunnerConfig:
    """
    Runtime configuration only.

    Controller gains and trajectory parameters do not belong here.
    """

    control_frequency_hz: float = 50.0
    log_frequency_hz: float = 10.0

    state_wait_timeout_s: float = 5.0
    arming_wait_s: float = 1.0

    def __post_init__(self) -> None:
        if self.control_frequency_hz <= 0.0:
            raise ValueError(
                "control_frequency_hz must be positive"
            )

        if self.log_frequency_hz < 0.0:
            raise ValueError(
                "log_frequency_hz must be nonnegative"
            )

        if self.state_wait_timeout_s <= 0.0:
            raise ValueError(
                "state_wait_timeout_s must be positive"
            )

        if self.arming_wait_s < 0.0:
            raise ValueError(
                "arming_wait_s must be nonnegative"
            )

    @property
    def control_period_s(self) -> float:
        return 1.0 / self.control_frequency_hz

    @property
    def log_period_s(self) -> float:
        if self.log_frequency_hz == 0.0:
            return float("inf")

        return 1.0 / self.log_frequency_hz


class UniversalRunner(
    Generic[
        StateT,
        ReferenceT,
        CommandT,
    ],
):
    """
    Generic real-time flight runner.

    Responsibilities:
        - start and stop state acquisition
        - wait for the first valid state
        - arm and disarm
        - maintain the control-loop clock
        - run a supplied FlightProgram
        - send commands
        - invoke optional logging
        - stop safely after completion or exceptions

    It does not know:
        - which controller is used
        - controller gains
        - which reference is used
        - trajectory dimensions
        - flight-phase definitions
        - how commands are calculated
    """

    def __init__(
        self,
        *,
        state_provider: StateProviderProtocol[
            StateT
        ],
        command_adapter: CommandAdapterProtocol[
            CommandT
        ],
        flight_program: FlightProgramProtocol[
            StateT,
            ReferenceT,
            CommandT,
        ],
        arm: Callable[[], None],
        disarm: Callable[[], None],
        emergency_stop: Callable[[], None] | None = None,
        config: UniversalRunnerConfig | None = None,
        log_callback: LogCallback[
            StateT,
            ReferenceT,
            CommandT,
        ]
        | None = None,
        run_logger: Any | None = None,
    ) -> None:
        self._state_provider = state_provider
        self._command_adapter = command_adapter
        self._flight_program = flight_program

        self._arm = arm
        self._disarm = disarm
        self._emergency_stop = emergency_stop

        self._config = (
            config
            if config is not None
            else UniversalRunnerConfig()
        )

        self._log_callback = log_callback
        self._run_logger = run_logger

    def wait_for_first_state(
        self,
    ) -> StateT:
        deadline = (
            time.monotonic()
            + self._config.state_wait_timeout_s
        )

        last_error: RuntimeError | None = None

        while time.monotonic() < deadline:
            try:
                return self._state_provider.get_state()
            except RuntimeError as error:
                last_error = error
                time.sleep(0.05)

        raise TimeoutError(
            "No valid state was received within "
            f"{self._config.state_wait_timeout_s:.1f} seconds"
        ) from last_error

    def run(self) -> None:
        provider_started = False
        is_armed = False
        run_status = "completed"
        termination_reason: str | None = None

        try:
            self._state_provider.start()
            provider_started = True
            if self._run_logger is not None:
                self._run_logger.event("state_provider_started")

            print(
                "[STATE] Waiting for estimator data"
            )

            pre_arming_state = (
                self.wait_for_first_state()
            )

            print(
                "[STATE] Initial state received"
            )
            if self._run_logger is not None:
                self._run_logger.event("initial_state_received")

            self._arm()
            is_armed = True
            if self._run_logger is not None:
                self._run_logger.event("arming_requested")

            print(
                "[STATE] Arming requested"
            )

            if self._config.arming_wait_s > 0.0:
                time.sleep(
                    self._config.arming_wait_s
                )

            post_arming_state = (
                self._state_provider
                .get_state()
            )
            start_time = time.monotonic()

            if self._run_logger is not None:
                self._run_logger.mark_running(start_time)

            initialization_reporter = getattr(
                self._flight_program,
                "report_initialization_states",
                None,
            )
            if initialization_reporter is not None:
                initialization_reporter(
                    pre_arming_state=pre_arming_state,
                    post_arming_state=post_arming_state,
                )

            initialization_state = pre_arming_state
            initialization_selector = getattr(
                self._flight_program,
                "select_initialization_state",
                None,
            )
            if initialization_selector is not None:
                initialization_state = initialization_selector(
                    pre_arming_state=pre_arming_state,
                    post_arming_state=post_arming_state,
                )

            self._flight_program.initialize(
                initial_state=initialization_state,
                start_time=start_time,
            )

            previous_time = start_time
            next_deadline = start_time
            last_log_time = start_time

            previous_phase: str | None = None

            while True:
                now = time.monotonic()

                if self._flight_program.is_complete(
                    now
                ):
                    break

                phase_name = (
                    self._flight_program
                    .get_phase_name(now)
                )

                if phase_name != previous_phase:
                    print(
                        f"[PHASE] {phase_name}"
                    )
                    previous_phase = phase_name
                    if self._run_logger is not None:
                        self._run_logger.event(
                            "phase_changed", {"phase": phase_name},
                            monotonic_s=now,
                        )

                state = (
                    self._state_provider
                    .get_state()
                )

                dt = now - previous_time
                previous_time = now

                if dt <= 0.0:
                    dt = (
                        self._config
                        .control_period_s
                    )

                reference, command = (
                    self._flight_program.update(
                        state=state,
                        current_time=now,
                        dt=dt,
                    )
                )

                self._command_adapter.send(
                    command
                )

                if self._run_logger is not None:
                    diagnostics: Mapping[str, Any] | None = None
                    diagnostics_provider = getattr(
                        self._flight_program, "get_logging_diagnostics", None
                    )
                    if diagnostics_provider is not None:
                        try:
                            diagnostics = diagnostics_provider()
                        except Exception as error:
                            self._run_logger.event(
                                "diagnostics_unavailable",
                                {"error": f"{type(error).__name__}: {error}"},
                                monotonic_s=now,
                            )
                    source_context_provider = getattr(
                        self._state_provider, "get_logging_context", None
                    )
                    source_context = (
                        source_context_provider()
                        if source_context_provider is not None else None
                    )
                    self._run_logger.log_flight(
                        now - start_time, now, phase_name, state, reference,
                        command, dt, diagnostics=diagnostics,
                        source_context=source_context,
                    )

                should_log = (
                    self._log_callback is not None
                    and now - last_log_time
                    >= self._config.log_period_s
                )

                if should_log:
                    elapsed = now - start_time

                    self._log_callback(
                        elapsed,
                        phase_name,
                        state,
                        reference,
                        command,
                        dt,
                    )

                    last_log_time = now

                next_deadline += (
                    self._config.control_period_s
                )

                sleep_duration = (
                    next_deadline
                    - time.monotonic()
                )

                if sleep_duration > 0.0:
                    time.sleep(
                        sleep_duration
                    )
                else:
                    # Drop missed deadlines instead of running
                    # multiple control steps back-to-back.
                    next_deadline = (
                        time.monotonic()
                    )

            print(
                "[STATE] Flight program complete"
            )

        except KeyboardInterrupt:
            run_status = "aborted"
            termination_reason = "KeyboardInterrupt"
            print(
                "\n[STOP] Keyboard interrupt received"
            )

        except BaseException as error:
            run_status = "failed"
            termination_reason = f"{type(error).__name__}: {error}"
            if is_armed and self._emergency_stop is not None:
                try:
                    self._emergency_stop()
                    print("[STOP] Emergency stop requested")
                except BaseException as error:
                    print(f"[WARN] Emergency stop failed: {error}")
            raise

        finally:
            print(
                "[STOP] Stopping command output"
            )

            try:
                self._command_adapter.stop()

            finally:
                if is_armed:
                    try:
                        self._disarm()

                        print(
                            "[STOP] Disarming requested"
                        )

                    except Exception as error:
                        print(
                            "[WARN] Disarming failed:"
                            f" {error}"
                        )

                if provider_started:
                    self._state_provider.stop()

                    print(
                        "[STOP] State provider stopped"
                    )

                if self._run_logger is not None:
                    self._run_logger.close(run_status, termination_reason)
