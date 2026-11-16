from __future__ import annotations

import threading
import time
from typing import Any, Callable


class CrazySimSupervisorAdapter:
    """Compatibility access to the supervisor in CrazySim's CRTP-v7 SITL.

    CFLib does not query supervisor state below CRTP v12, while this firmware
    exposes the same status layout as the ``supervisor.info`` log variable.
    The SITL build also disables the legacy platform arming command, so the
    adapter uses the firmware's legacy ``system.arm`` parameter with explicit
    safety checks and readback.
    """

    SUPPORTED_PROTOCOL_VERSION = 7
    STATUS_LOG = "supervisor.info"
    ARM_PARAMETER = "system.arm"
    MAX_STATUS_AGE_S = 0.2

    def __init__(
        self,
        crazyflie: Any,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._cf = crazyflie
        self._clock = clock
        self._lock = threading.Lock()
        self._bitfield: int | None = None
        self._received_at: float | None = None
        self._first_sample = threading.Event()
        self._config: Any | None = None

        version = int(self._cf.platform.get_protocol_version())
        if version != self.SUPPORTED_PROTOCOL_VERSION:
            raise RuntimeError(
                "CrazySim supervisor compatibility requires CRTP protocol "
                f"version {self.SUPPORTED_PROTOCOL_VERSION}, got {version}"
            )

    @staticmethod
    def decode(bitfield: int) -> dict[str, bool | int]:
        if not 0 <= int(bitfield) <= 0xFFFF:
            raise ValueError("supervisor.info must be an unsigned 16-bit value")
        value = int(bitfield)
        return {
            "bitfield": value,
            "can_be_armed": bool(value & (1 << 0)),
            "is_armed": bool(value & (1 << 1)),
            "is_auto_armed": bool(value & (1 << 2)),
            "can_fly": bool(value & (1 << 3)),
            "is_flying": bool(value & (1 << 4)),
            "is_tumbled": bool(value & (1 << 5)),
            "is_locked": bool(value & (1 << 6)),
            "is_crashed": bool(value & (1 << 7)),
            # CrazySim's v7 firmware only publishes bits 0 through 7.
            "hl_control_active": False,
        }

    def start(self, timeout_s: float = 1.0) -> None:
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if self._config is not None:
            raise RuntimeError("CrazySim supervisor monitor is already started")

        from cflib.crazyflie.log import LogConfig

        config = LogConfig(name="CrazySimSupervisor", period_in_ms=10)
        config.add_variable(self.STATUS_LOG, "uint16_t")
        self._config = config
        try:
            self._cf.log.add_config(config)
            config.data_received_cb.add_callback(self._on_data)
            config.start()
            if not self._first_sample.wait(timeout_s):
                raise RuntimeError("timed out waiting for supervisor.info telemetry")
        except BaseException:
            self.stop()
            raise

    def _on_data(self, _timestamp: int, data: dict[str, Any], _config: Any) -> None:
        value = int(data[self.STATUS_LOG])
        with self._lock:
            self._bitfield = value
            self._received_at = self._clock()
        self._first_sample.set()

    def status(self) -> dict[str, bool | int]:
        status, _ = self.status_with_sample()
        return status

    def status_with_sample(self) -> tuple[dict[str, bool | int], dict[str, float | bool]]:
        """Return decoded state and evidence that it came from a fresh log sample."""
        with self._lock:
            bitfield = self._bitfield
            received_at = self._received_at
        if bitfield is None or received_at is None:
            raise RuntimeError("no supervisor.info telemetry has been received")
        age = self._clock() - received_at
        if age < -0.01 or age > self.MAX_STATUS_AGE_S:
            raise RuntimeError(f"supervisor.info telemetry is stale: {age:.3f} s")
        return self.decode(bitfield), {
            "age_s": age,
            "fresh": True,
            "maximum_age_s": self.MAX_STATUS_AGE_S,
        }

    def send_arming_request(self, do_arm: bool) -> None:
        status = self.status()
        if not do_arm and status["is_auto_armed"]:
            raise RuntimeError(
                "CrazySim auto-arm is compile-time firmware behavior and cannot "
                "be cleared by the legacy system.arm parameter"
            )
        if do_arm:
            faults = tuple(
                name
                for name in ("is_tumbled", "is_locked", "is_crashed")
                if status[name]
            )
            if not status["can_be_armed"] or faults or status["is_armed"]:
                raise RuntimeError(
                    f"refusing legacy CrazySim arm request: {status}; faults={faults}"
                )
        requested = "1" if do_arm else "0"
        self._cf.param.set_value(self.ARM_PARAMETER, requested)
        readback = int(self._cf.param.get_value(self.ARM_PARAMETER))
        if readback != int(do_arm):
            # Best-effort fail closed if an arm write could have been accepted.
            if do_arm:
                try:
                    self._cf.param.set_value(self.ARM_PARAMETER, "0")
                except BaseException:
                    pass
            raise RuntimeError(
                f"CrazySim {self.ARM_PARAMETER} readback mismatch: {readback}"
            )

    def send_emergency_stop(self) -> None:
        # CFLib's legacy localization-port encoding is supported by this firmware.
        self._cf.supervisor.send_emergency_stop()

    def stop(self) -> None:
        config = self._config
        self._config = None
        if config is None:
            return
        try:
            config.stop()
        finally:
            config.delete()
