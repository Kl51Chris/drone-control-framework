from __future__ import annotations

import csv
import json
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .config import CalibrationConfig


FIELDS = ["elapsed_s", "monotonic_s", "phase", "state_source", "state_age_s", "dt_s", "altitude_reference_m", "altitude_m", "vertical_velocity_m_s", "x_m", "y_m", "vx_m_s", "vy_m_s", "horizontal_displacement_m", "horizontal_speed_m_s", "roll_rad", "pitch_rad", "yaw_rad", "error_m", "p_term", "i_term", "d_term", "pid_unsaturated", "pid_output", "thrust_normalized", "thrust_percent", "saturated"]


class ExperimentLogger:
    def __init__(self, root: str | Path, mode: str, config: CalibrationConfig) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.directory = Path(root) / f"{stamp}_{mode}"
        self.directory.mkdir(parents=True, exist_ok=False)
        self._stream = (self.directory / "telemetry.csv").open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._stream, fieldnames=FIELDS)
        self._writer.writeheader()
        metadata = {"schema_version": 1, "mode": mode, "started_utc": datetime.now(timezone.utc).isoformat(), "status": "running", "config": asdict(config)}
        self._write_metadata(metadata)
        self.metadata = metadata

    def _write_metadata(self, data: dict[str, Any]) -> None:
        with (self.directory / "metadata.json").open("w", encoding="utf-8") as stream:
            json.dump(data, stream, indent=2, sort_keys=True)
            stream.write("\n")

    def row(self, values: dict[str, Any]) -> None:
        self._writer.writerow(values)
        self._stream.flush()

    def persist_metadata(self) -> None:
        self._write_metadata(self.metadata)

    def safety_event(self, condition: str, values: dict[str, Any]) -> None:
        """Persist fault evidence before shutdown can disrupt the link/process."""
        self.metadata["safety_event"] = {
            "condition": condition,
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
            **values,
        }
        self._write_metadata(self.metadata)

    def close(self, status: str, reason: str = "") -> None:
        if self._stream.closed:
            return
        self._stream.close()
        self.metadata.update(status=status, reason=reason, ended_utc=datetime.now(timezone.utc).isoformat())
        self._write_metadata(self.metadata)
