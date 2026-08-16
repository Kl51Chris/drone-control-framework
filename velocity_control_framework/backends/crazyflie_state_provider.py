from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from velocity_control_framework.interfaces import DroneState


class CrazyflieStateProvider:
    """
    Obtain the latest estimator state from Crazyflie log callbacks.

    The provider owns:
        - Crazyflie estimator-state log configuration
        - Crazyflie log callbacks
        - conversion into DroneState
        - thread-safe storage of the latest state
        - propagation of logging errors to the caller

    Coordinate and unit conventions:
        position:
            Crazyflie stateEstimate world-frame position [x, y, z],
            in meters.

        velocity:
            Crazyflie stateEstimate world-frame velocity [vx, vy, vz],
            in meters per second.

        roll:
            Crazyflie estimator roll angle, converted from degrees
            to radians.

        pitch:
            Crazyflie legacy pitch sign convention, converted from
            degrees to radians. No additional sign inversion is applied.

        yaw:
            Crazyflie estimator yaw angle, converted from degrees
            to radians.

        timestamp:
            Host monotonic time at which the latest complete state
            was assembled, in seconds.
    """

    def __init__(self, crazyflie: Any) -> None:
        self._cf = crazyflie

        self._lock = threading.Lock()
        self._state_ready = threading.Event()

        self._latest_state: DroneState | None = None
        self._error: RuntimeError | None = None

        self._translation_log_config: Any | None = None
        self._attitude_log_config: Any | None = None

        self._latest_position: np.ndarray | None = None
        self._latest_velocity: np.ndarray | None = None
        self._latest_attitude: np.ndarray | None = None

        self._running = False

    def start(self) -> None:
        """
        Configure and start Crazyflie estimator-state logging.

        Translation and attitude are placed in separate LogConfig
        blocks to stay within the Crazyflie log-packet size limit.
        """
        if self._running:
            return

        with self._lock:
            self._latest_state = None
            self._error = None

            self._latest_position = None
            self._latest_velocity = None
            self._latest_attitude = None
        self._state_ready.clear()
        
        self._translation_log_config = (
            self._create_translation_log_config()
        )
        self._attitude_log_config = (
            self._create_attitude_log_config()
        )

        self._cf.log.add_config(
            self._translation_log_config
        )
        self._cf.log.add_config(
            self._attitude_log_config
        )

        self._translation_log_config.data_received_cb.add_callback(
            self._on_translation_log_data
        )
        self._translation_log_config.error_cb.add_callback(
            self._on_log_error
        )

        self._attitude_log_config.data_received_cb.add_callback(
            self._on_attitude_log_data
        )
        self._attitude_log_config.error_cb.add_callback(
            self._on_log_error
        )

        try:
            self._translation_log_config.start()
            self._attitude_log_config.start()
            self._running = True

            state_ready = self._state_ready.wait(
                timeout=2.0
            )

            if not state_ready:
                raise RuntimeError(
                    "Timed out waiting for the first complete "
                    "Crazyflie state"
                )

            with self._lock:
                if self._error is not None:
                    raise RuntimeError(
                        str(self._error)
                    ) from self._error

                if self._latest_state is None:
                    raise RuntimeError(
                        "Crazyflie state wait completed, but no "
                        "complete state is available"
                    )
        except Exception:
            self.stop()
            raise

        self._running = True

    def get_state(self) -> DroneState:
        """
        Return a copy of the latest complete DroneState.

        Raises:
            RuntimeError:
                If a Crazyflie logging error has occurred.

            RuntimeError:
                If no complete state sample has been received yet.
        """
        with self._lock:
            if self._error is not None:
                raise RuntimeError(str(self._error)) from self._error

            if self._latest_state is None:
                raise RuntimeError(
                    "No complete Crazyflie state has been "
                    "received yet"
                )

            state = self._latest_state

            return DroneState(
                position=state.position.copy(),
                velocity=state.velocity.copy(),
                roll=state.roll,
                pitch=state.pitch,
                yaw=state.yaw,
                timestamp=state.timestamp,
            )

    def stop(self) -> None:
        """Stop all Crazyflie estimator-state log blocks."""
        configs = (
            self._translation_log_config,
            self._attitude_log_config,
        )

        for config in configs:
            if config is None:
                continue

            try:
                config.stop()
            except Exception:
                # Shutdown should continue even if one log block
                # has already stopped or the link is unavailable.
                pass

        self._running = False

    @staticmethod
    def _create_translation_log_config() -> Any:
        """
        Create the position and velocity log configuration.

        Six float variables require 24 bytes, which fits inside one
        Crazyflie log block.
        """
        from cflib.crazyflie.log import LogConfig

        config = LogConfig(
            name="EstimatorTranslation",
            period_in_ms=10,
        )

        config.add_variable(
            "stateEstimate.x",
            "float",
        )
        config.add_variable(
            "stateEstimate.y",
            "float",
        )
        config.add_variable(
            "stateEstimate.z",
            "float",
        )

        config.add_variable(
            "stateEstimate.vx",
            "float",
        )
        config.add_variable(
            "stateEstimate.vy",
            "float",
        )
        config.add_variable(
            "stateEstimate.vz",
            "float",
        )

        return config

    @staticmethod
    def _create_attitude_log_config() -> Any:
        """
        Create the roll, pitch, and yaw log configuration.

        Crazyflie reports these angles in degrees. They are converted
        to radians when received.
        """
        from cflib.crazyflie.log import LogConfig

        config = LogConfig(
            name="EstimatorAttitude",
            period_in_ms=10,
        )

        config.add_variable(
            "stateEstimate.roll",
            "float",
        )
        config.add_variable(
            "stateEstimate.pitch",
            "float",
        )
        config.add_variable(
            "stateEstimate.yaw",
            "float",
        )

        return config

    def _on_translation_log_data(
        self,
        timestamp: int,
        data: dict[str, float],
        log_config: Any,
    ) -> None:
        """
        Store one Crazyflie position and velocity log packet.
        """
        del timestamp
        del log_config

        position = np.array(
            [
                data["stateEstimate.x"],
                data["stateEstimate.y"],
                data["stateEstimate.z"],
            ],
            dtype=np.float64,
        )

        velocity = np.array(
            [
                data["stateEstimate.vx"],
                data["stateEstimate.vy"],
                data["stateEstimate.vz"],
            ],
            dtype=np.float64,
        )

        with self._lock:
            self._latest_position = position
            self._latest_velocity = velocity
            self._try_build_state_locked()

    def _on_attitude_log_data(
        self,
        timestamp: int,
        data: dict[str, float],
        log_config: Any,
    ) -> None:
        """
        Store one Crazyflie roll, pitch, and yaw log packet.

        Pitch uses the Crazyflie legacy sign convention. No extra
        sign inversion is applied here.
        """
        del timestamp
        del log_config

        attitude = np.array(
            [
                np.deg2rad(
                    data["stateEstimate.roll"]
                ),
                np.deg2rad(
                    data["stateEstimate.pitch"]
                ),
                np.deg2rad(
                    data["stateEstimate.yaw"]
                ),
            ],
            dtype=np.float64,
        )

        with self._lock:
            self._latest_attitude = attitude
            self._try_build_state_locked()

    def _try_build_state_locked(self) -> None:
        """
        Build a complete DroneState when all required components exist.

        This method must only be called while self._lock is held.
        """
        if self._latest_position is None:
            return

        if self._latest_velocity is None:
            return

        if self._latest_attitude is None:
            return

        self._latest_state = DroneState(
            position=self._latest_position.copy(),
            velocity=self._latest_velocity.copy(),
            roll=float(self._latest_attitude[0]),
            pitch=float(self._latest_attitude[1]),
            yaw=float(self._latest_attitude[2]),
            timestamp=time.monotonic(),
        )
        self._state_ready.set()
    def _on_log_error(
        self,
        log_config: Any,
        message: str,
    ) -> None:
        """
        Store a logging error for propagation through get_state().

        The callback may execute in a cflib worker thread, so it must
        not raise directly from this method.
        """
        config_name = getattr(
            log_config,
            "name",
            "unknown",
        )

        error = RuntimeError(
            f"Crazyflie log error in "
            f"{config_name}: {message}"
        )

        with self._lock:
            self._error = error

        self._state_ready.set()