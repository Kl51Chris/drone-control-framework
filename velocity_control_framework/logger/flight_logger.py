from __future__ import annotations

import csv
import json
import queue
import re
import subprocess
import threading
import uuid
from dataclasses import asdict, dataclass, is_dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LOG_ROOT = PROJECT_ROOT / "log"

STATE_FIELDS = (
    "position_x_m", "position_y_m", "position_z_m",
    "velocity_x_m_s", "velocity_y_m_s", "velocity_z_m_s",
    "roll_rad", "pitch_rad", "yaw_rad",
    "body_rate_p_rad_s", "body_rate_q_rad_s", "body_rate_r_rad_s",
)

FLIGHT_FIELDS = (
    "step_index", "elapsed_s", "monotonic_s", "control_dt_s", "phase",
    "reference_timestamp_s", "reference_position_x_m",
    "reference_position_y_m", "reference_position_z_m",
    "reference_velocity_x_m_s", "reference_velocity_y_m_s",
    "reference_velocity_z_m_s", "reference_yaw_rad",
    "reference_yaw_rate_rad_s", "estimated_timestamp_s",
    *tuple(f"estimated_{name}" for name in STATE_FIELDS),
    "command_roll_rate_rad_s", "command_pitch_rate_rad_s",
    "command_yaw_rate_rad_s", "command_thrust_normalized",
    "safety_status", "onboard_available", "onboard_sequence",
    "external_available", "external_sequence", "controller_diagnostics_json",
)

SOURCE_FIELDS = (
    "sequence", "source_timestamp_s", "reception_monotonic_s", "available",
    *STATE_FIELDS,
)

EVENT_FIELDS = (
    "sequence", "timestamp", "monotonic_s", "elapsed_s", "event", "details_json",
)


@dataclass(frozen=True, slots=True)
class RunConfiguration:
    experiment_name: str
    reference_type: str
    reference_configuration: Mapping[str, Any]
    controller_type: str
    parameter_set: str
    controller_configuration: Mapping[str, Any]
    target_control_frequency_hz: float
    hardware: str | None = None
    state_estimation_source: str | None = None
    data_sources: Mapping[str, Any] | None = None


def _slug(value: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9-]+", "-", value.strip()).strip("-").lower()
    return slug or "unnamed"


def _json_value(value: Any) -> Any:
    if is_dataclass(value) and not isinstance(value, type):
        return _json_value(asdict(value))
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    return value


def _git_commit() -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT,
            check=True, capture_output=True, text=True, timeout=2,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


class FlightLogger:
    """Non-blocking, controller-independent logger for one flight run."""

    def __init__(
        self,
        config: RunConfiguration,
        root: str | Path = DEFAULT_LOG_ROOT,
        *,
        now: datetime | None = None,
        run_id: str | None = None,
    ) -> None:
        started = (now or datetime.now().astimezone())
        if started.tzinfo is None:
            raise ValueError("logging start time must include a timezone")
        self.run_id = run_id or str(uuid.uuid4())
        name = "_".join((
            started.strftime("%Y%m%d_%H%M%S"),
            _slug(config.reference_type), _slug(config.controller_type),
            _slug(config.parameter_set), _slug(config.experiment_name),
        ))
        root_path = Path(root)
        root_path.mkdir(parents=True, exist_ok=True)
        self.directory = root_path / name
        try:
            self.directory.mkdir(exist_ok=False)
        except FileExistsError:
            self.directory = root_path / f"{name}_{self.run_id[:8]}"
            self.directory.mkdir(exist_ok=False)

        self._started = started
        self._start_monotonic: float | None = None
        self._step_sequence = 0
        self._event_sequence = 0
        self._source_sequences = {"onboard": 0, "external": 0}
        self._latest_source_sequences: dict[str, int | None] = {
            "onboard": None, "external": None,
        }
        self._sequence_lock = threading.Lock()
        self._queue: queue.Queue[tuple[str, dict[str, Any]] | None] = queue.Queue()
        self._closed = False
        self._worker_error: BaseException | None = None

        source_info = config.data_sources or {
            "controller_state": {"description": "DroneState consumed by controller"},
            "onboard": {"description": "Crazyflie onboard stateEstimate telemetry"},
            "external": {"description": "optional external measurement provider"},
            "raw_imu": {"available": False, "reason": "not exposed by active telemetry"},
        }
        self.metadata: dict[str, Any] = {
            "schema_version": 1,
            "run_id": self.run_id,
            "timestamp": started.isoformat(),
            "started_at": started.isoformat(),
            "ended_at": None,
            "experiment_name": config.experiment_name,
            "reference": {
                "type": config.reference_type,
                "configuration": _json_value(config.reference_configuration),
            },
            "controller_type": config.controller_type,
            "parameter_set": config.parameter_set,
            "controller_configuration": _json_value(config.controller_configuration),
            "target_control_frequency_hz": float(config.target_control_frequency_hz),
            "git_commit": _git_commit(),
            "hardware": config.hardware,
            "state_estimation_source": config.state_estimation_source,
            "data_sources": _json_value(source_info),
            "status": "initialized",
            "termination_reason": None,
        }
        self._write_metadata()

        self._thread = threading.Thread(
            target=self._writer_main, name=f"flight-log-{self.run_id[:8]}", daemon=True
        )
        self._thread.start()
        self.event("logger_initialized", {"run_directory": str(self.directory)})

    def _write_metadata(self) -> None:
        temporary = self.directory / ".metadata.json.tmp"
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(self.metadata, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        temporary.replace(self.directory / "metadata.json")

    def _writer_main(self) -> None:
        specs = {
            "flight": ("flight.csv", FLIGHT_FIELDS),
            "onboard": ("onboard_state.csv", SOURCE_FIELDS),
            "external": ("external_state.csv", SOURCE_FIELDS),
            "event": ("events.csv", EVENT_FIELDS),
        }
        streams: dict[str, Any] = {}
        writers: dict[str, csv.DictWriter] = {}
        try:
            for key, (filename, fields) in specs.items():
                stream = (self.directory / filename).open("w", newline="", encoding="utf-8")
                writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
                writer.writeheader()
                streams[key] = stream
                writers[key] = writer
            while True:
                item = self._queue.get()
                try:
                    if item is None:
                        break
                    kind, row = item
                    writers[kind].writerow(row)
                finally:
                    self._queue.task_done()
            for stream in streams.values():
                stream.flush()
        except BaseException as error:
            self._worker_error = error
        finally:
            for stream in streams.values():
                stream.close()

    def _enqueue(self, kind: str, row: dict[str, Any]) -> None:
        if self._closed:
            return
        self._queue.put_nowait((kind, row))

    @staticmethod
    def _state_values(state: Any) -> dict[str, Any]:
        if state is None:
            return {field: None for field in STATE_FIELDS}
        position = getattr(state, "position", None)
        velocity = getattr(state, "velocity", None)
        return {
            "position_x_m": None if position is None else position[0],
            "position_y_m": None if position is None else position[1],
            "position_z_m": None if position is None else position[2],
            "velocity_x_m_s": None if velocity is None else velocity[0],
            "velocity_y_m_s": None if velocity is None else velocity[1],
            "velocity_z_m_s": None if velocity is None else velocity[2],
            "roll_rad": getattr(state, "roll", None),
            "pitch_rad": getattr(state, "pitch", None),
            "yaw_rad": getattr(state, "yaw", None),
            "body_rate_p_rad_s": getattr(state, "p", None),
            "body_rate_q_rad_s": getattr(state, "q", None),
            "body_rate_r_rad_s": getattr(state, "r", None),
        }

    def mark_running(self, monotonic_s: float) -> None:
        self._start_monotonic = monotonic_s
        self.metadata["status"] = "running"
        self._write_metadata()
        self.event("run_started", monotonic_s=monotonic_s)

    def log_flight(
        self, elapsed_s: float, monotonic_s: float, phase: str, state: Any,
        reference: Any, command: Any, dt: float, *, safety_status: str = "nominal",
        diagnostics: Mapping[str, Any] | None = None,
        source_context: Mapping[str, Any] | None = None,
    ) -> None:
        source_context = source_context or {}
        with self._sequence_lock:
            onboard_sequence = source_context.get(
                "onboard_sequence", self._latest_source_sequences["onboard"]
            )
            external_sequence = source_context.get(
                "external_sequence", self._latest_source_sequences["external"]
            )
            step_sequence = self._step_sequence
            self._step_sequence += 1
        values = self._state_values(state)
        row = {
            "step_index": step_sequence, "elapsed_s": elapsed_s,
            "monotonic_s": monotonic_s, "control_dt_s": dt, "phase": phase,
            "reference_timestamp_s": getattr(reference, "timestamp", None),
            "estimated_timestamp_s": getattr(state, "timestamp", None),
            "command_roll_rate_rad_s": getattr(command, "roll_rate", None),
            "command_pitch_rate_rad_s": getattr(command, "pitch_rate", None),
            "command_yaw_rate_rad_s": getattr(command, "yaw_rate", None),
            "command_thrust_normalized": getattr(command, "thrust", None),
            "safety_status": safety_status,
            "onboard_available": onboard_sequence is not None,
            "onboard_sequence": onboard_sequence,
            "external_available": external_sequence is not None,
            "external_sequence": external_sequence,
            "controller_diagnostics_json": json.dumps(_json_value(diagnostics or {}), sort_keys=True),
        }
        for key, value in values.items():
            row[f"estimated_{key}"] = value
        for vector_name in ("position", "velocity"):
            vector = getattr(reference, vector_name, None)
            for index, axis in enumerate("xyz"):
                row[f"reference_{vector_name}_{axis}_{'m' if vector_name == 'position' else 'm_s'}"] = (
                    None if vector is None else vector[index]
                )
        row["reference_yaw_rad"] = getattr(reference, "yaw", None)
        row["reference_yaw_rate_rad_s"] = getattr(reference, "yaw_rate", None)
        self._enqueue("flight", row)

    def _record_source(
        self, source: str, state: Any, *, source_timestamp_s: float | None,
        reception_monotonic_s: float, available: bool,
    ) -> int:
        with self._sequence_lock:
            sequence = self._source_sequences[source]
            self._source_sequences[source] += 1
            self._latest_source_sequences[source] = sequence
        row = {
            "sequence": sequence, "source_timestamp_s": source_timestamp_s,
            "reception_monotonic_s": reception_monotonic_s, "available": available,
            **self._state_values(state),
        }
        self._enqueue(source, row)
        return sequence

    def record_onboard_state(
        self, state: Any, *, source_timestamp_s: float | None,
        reception_monotonic_s: float,
    ) -> int:
        return self._record_source(
            "onboard", state, source_timestamp_s=source_timestamp_s,
            reception_monotonic_s=reception_monotonic_s, available=True,
        )

    def record_external_state(
        self, state: Any | None, *, source_timestamp_s: float | None,
        reception_monotonic_s: float, available: bool = True,
    ) -> int:
        return self._record_source(
            "external", state, source_timestamp_s=source_timestamp_s,
            reception_monotonic_s=reception_monotonic_s, available=available,
        )

    def event(
        self, name: str, details: Mapping[str, Any] | None = None, *,
        monotonic_s: float | None = None,
    ) -> None:
        elapsed = None
        if monotonic_s is not None and self._start_monotonic is not None:
            elapsed = monotonic_s - self._start_monotonic
        with self._sequence_lock:
            sequence = self._event_sequence
            self._event_sequence += 1
        row = {
            "sequence": sequence,
            "timestamp": datetime.now().astimezone().isoformat(),
            "monotonic_s": monotonic_s, "elapsed_s": elapsed, "event": name,
            "details_json": json.dumps(_json_value(details or {}), sort_keys=True),
        }
        self._enqueue("event", row)

    def close(self, status: str = "completed", reason: str | None = None) -> None:
        if self._closed:
            return
        self.event("run_finished", {"status": status, "reason": reason})
        self._closed = True
        self._queue.put_nowait(None)
        self._thread.join()
        self.metadata.update(
            status=status, termination_reason=reason,
            ended_at=datetime.now().astimezone().isoformat(),
        )
        if self._worker_error is not None:
            self.metadata["logging_error"] = repr(self._worker_error)
        self._write_metadata()
        if self._worker_error is not None:
            raise RuntimeError("flight log writer failed") from self._worker_error

    def __enter__(self) -> FlightLogger:
        return self

    def __exit__(self, exc_type: Any, exc: BaseException | None, traceback: Any) -> None:
        del exc_type, traceback
        self.close("completed" if exc is None else "failed", None if exc is None else repr(exc))
