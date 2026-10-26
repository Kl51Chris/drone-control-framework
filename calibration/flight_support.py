from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Protocol


class FlightLogger(Protocol):
    directory: Path
    metadata: dict[str, Any]
    def persist_metadata(self) -> None: ...


def record_battery_snapshot(hardware: Any, logger: FlightLogger,
                            output: Callable[[str], None] = print) -> dict[str, Any]:
    """Persist and display a best-effort preflight battery measurement."""
    try:
        snapshot = hardware.battery_snapshot()
    except Exception as exc:
        snapshot = {
            "success": False, "requested_utc": None, "measurement_utc": None,
            "measurement_monotonic_s": None, "firmware_log_timestamp_ms": None,
            "voltage_v": None, "percentage": None, "percentage_source": None,
            "error": f"{type(exc).__name__}: {exc}",
        }
    logger.metadata["battery_snapshot"] = snapshot
    logger.persist_metadata()
    if snapshot.get("success"):
        percentage = (f'{snapshot["percentage"]:.1f}%' if snapshot.get("percentage") is not None
                      else "unavailable")
        output(f'[BATTERY] voltage={snapshot["voltage_v"]:.3f} V '
               f'percentage={percentage} timestamp={snapshot["measurement_utc"]}')
    else:
        output(f'[BATTERY] unavailable error={snapshot.get("error") or "unknown"}')
    return snapshot


def print_run_summary(logger: FlightLogger, output: Callable[[str], None] = print) -> None:
    """Print stable artifact locations and final state without changing exceptions."""
    directory = Path(logger.directory).resolve()
    telemetry = directory / ("flight_telemetry.csv" if (directory/"flight_telemetry.csv").exists()
                             else "telemetry.csv")
    metadata = logger.metadata
    status = metadata.get("status", "unknown")
    mode = metadata.get("mode", directory.name)
    reason = (metadata.get("safety_event") or {}).get("condition")
    if reason is None:
        reason = metadata.get("termination_reason", metadata.get("reason", ""))
    output(f"[RUN] {directory}")
    output(f"[LOG] {telemetry}")
    output(f"[RESULT] experiment={mode} status={status} reason={reason or 'none'}")
