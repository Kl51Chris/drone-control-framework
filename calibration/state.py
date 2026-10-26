from __future__ import annotations

from dataclasses import dataclass, fields
import json
import math
import socket
import threading
import time


@dataclass(frozen=True, slots=True)
class AltitudeState:
    altitude_m: float
    vertical_velocity_m_s: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    timestamp_s: float
    source: str = "crazyflie"


def validate_fresh_state(state: AltitudeState, now_s: float, max_age_s: float) -> float:
    values = (getattr(state, item.name) for item in fields(state))
    if any(not math.isfinite(v) for v in values if isinstance(v, float)):
        raise RuntimeError("state contains a non-finite value")
    age = now_s - state.timestamp_s
    if age < -0.01:
        raise RuntimeError("state timestamp is in the future")
    if age > max_age_s:
        raise TimeoutError(f"state is stale ({age:.3f} s > {max_age_s:.3f} s)")
    return age


class ExternalAltitudeReceiver:
    """Receive newline-independent UDP JSON: {\"altitude_m\": number}."""

    def __init__(self, port: int) -> None:
        self.port = port
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._sample: tuple[float, float, float] | None = None

    def start(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", self.port))
        sock.settimeout(0.1)
        self._socket = sock
        self._thread = threading.Thread(target=self._receive, daemon=True)
        self._thread.start()

    def _receive(self) -> None:
        previous: tuple[float, float] | None = None
        while not self._stop.is_set():
            try:
                payload, _ = self._socket.recvfrom(4096)  # type: ignore[union-attr]
            except socket.timeout:
                continue
            received = time.monotonic()
            try:
                altitude = float(json.loads(payload)["altitude_m"])
                if not math.isfinite(altitude):
                    continue
            except (ValueError, TypeError, KeyError, json.JSONDecodeError):
                continue
            velocity = 0.0
            if previous is not None and received > previous[1]:
                velocity = (altitude - previous[0]) / (received - previous[1])
            previous = (altitude, received)
            with self._lock:
                self._sample = (altitude, velocity, received)

    def latest(self) -> tuple[float, float, float]:
        with self._lock:
            if self._sample is None:
                raise RuntimeError("no external altitude sample received")
            return self._sample

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self._socket is not None:
            self._socket.close()
