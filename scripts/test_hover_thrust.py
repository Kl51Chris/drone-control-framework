from __future__ import annotations

import csv
import statistics
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


URI = "radio://0/80/2M"

TARGET_HEIGHT_M = 0.30
HOVER_DURATION_S = 5.0
COMMAND_PERIOD_S = 0.05

# Crazyflie log period must be an integer number of milliseconds.
LOG_PERIOD_MS = 50

OUTPUT_DIRECTORY = Path("hover_thrust_logs")


@dataclass
class HoverSample:
    """One sample recorded during the 5-second hover interval."""

    elapsed_s: float
    thrust: float
    height_m: float | None
    vertical_velocity_m_s: float | None
    battery_voltage_v: float | None


class HoverThrustLogger:
    """
    Log the onboard controller thrust during the steady hover interval.

    Only samples collected while recording_enabled is True are saved.
    The takeoff and landing ramps are therefore excluded from the reported
    hover-thrust statistics.
    """

    def __init__(self, cf: Crazyflie) -> None:
        self.cf = cf
        self.log_config: LogConfig | None = None

        self.samples: list[HoverSample] = []

        self.recording_enabled = False
        self.recording_start_time: float | None = None

        self.available_variables: set[str] = set()

    def _load_available_variables(self) -> None:
        """
        Read the logging table of contents received from the Crazyflie.

        The exact set of variables can vary with firmware builds.
        """
        toc = self.cf.log.toc

        if toc is None:
            raise RuntimeError(
                "Crazyflie logging TOC is unavailable."
            )

        toc_data = getattr(toc, "toc", None)

        if not isinstance(toc_data, dict):
            raise RuntimeError(
                "Could not read Crazyflie logging TOC."
            )

        for group_name, group_variables in toc_data.items():
            if not isinstance(group_variables, dict):
                continue

            for variable_name in group_variables:
                self.available_variables.add(
                    f"{group_name}.{variable_name}"
                )

    def _add_optional_variable(
        self,
        log_config: LogConfig,
        variable_name: str,
    ) -> bool:
        """Add a variable only when it exists in the current firmware."""
        if variable_name not in self.available_variables:
            print(
                f"[WARN] Log variable unavailable: {variable_name}"
            )
            return False

        # No explicit type is supplied. CFLib uses the variable's native
        # type from the logging table of contents.
        log_config.add_variable(variable_name)

        print(f"[OK] Logging variable: {variable_name}")
        return True

    def start(self) -> None:
        """Configure and start the Crazyflie log block."""
        self._load_available_variables()

        if "stabilizer.thrust" not in self.available_variables:
            raise RuntimeError(
                "Required log variable stabilizer.thrust is not available. "
                "Check the firmware log TOC before flying."
            )

        log_config = LogConfig(
            name="HoverThrust",
            period_in_ms=LOG_PERIOD_MS,
        )

        self._add_optional_variable(
            log_config,
            "stabilizer.thrust",
        )
        self._add_optional_variable(
            log_config,
            "stateEstimate.z",
        )
        self._add_optional_variable(
            log_config,
            "stateEstimate.vz",
        )
        self._add_optional_variable(
            log_config,
            "pm.vbat",
        )

        log_config.data_received_cb.add_callback(
            self._on_log_data
        )
        log_config.error_cb.add_callback(
            self._on_log_error
        )

        self.cf.log.add_config(log_config)

        if not log_config.valid:
            raise RuntimeError(
                "Hover thrust log configuration is invalid."
            )

        log_config.start()
        self.log_config = log_config

        print("[INFO] Hover thrust logger started.")

    def stop(self) -> None:
        """Stop the Crazyflie log block."""
        self.recording_enabled = False

        if self.log_config is not None:
            try:
                self.log_config.stop()
            except Exception as error:
                print(
                    f"[WARN] Could not stop log block cleanly: "
                    f"{error}"
                )

        print("[INFO] Hover thrust logger stopped.")

    def begin_hover_recording(self) -> None:
        """Begin saving samples for the steady hover experiment."""
        self.samples.clear()
        self.recording_start_time = time.monotonic()
        self.recording_enabled = True

        print("[INFO] Recording hover thrust now.")

    def end_hover_recording(self) -> None:
        """Stop saving samples."""
        self.recording_enabled = False

        print(
            f"[INFO] Recorded {len(self.samples)} hover samples."
        )

    def _on_log_data(
        self,
        timestamp: int,
        data: dict[str, Any],
        log_config: LogConfig,
    ) -> None:
        """Receive one asynchronous Crazyflie log packet."""
        del timestamp
        del log_config

        if not self.recording_enabled:
            return

        if self.recording_start_time is None:
            return

        thrust_value = data.get("stabilizer.thrust")

        if thrust_value is None:
            return

        elapsed_s = (
            time.monotonic() - self.recording_start_time
        )

        sample = HoverSample(
            elapsed_s=elapsed_s,
            thrust=float(thrust_value),
            height_m=self._optional_float(
                data.get("stateEstimate.z")
            ),
            vertical_velocity_m_s=self._optional_float(
                data.get("stateEstimate.vz")
            ),
            battery_voltage_v=self._optional_float(
                data.get("pm.vbat")
            ),
        )

        self.samples.append(sample)

    @staticmethod
    def _optional_float(value: Any) -> float | None:
        if value is None:
            return None

        return float(value)

    @staticmethod
    def _on_log_error(
        log_config: LogConfig,
        message: str,
    ) -> None:
        print(
            f"[ERROR] Log block {log_config.name}: {message}"
        )

    def save_csv(self) -> Path:
        """Save all steady-hover samples to a timestamped CSV file."""
        if not self.samples:
            raise RuntimeError(
                "No hover samples were recorded."
            )

        OUTPUT_DIRECTORY.mkdir(
            parents=True,
            exist_ok=True,
        )

        timestamp_text = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        output_path = (
            OUTPUT_DIRECTORY
            / f"hover_thrust_{timestamp_text}.csv"
        )

        with output_path.open(
            "w",
            newline="",
            encoding="utf-8",
        ) as csv_file:
            writer = csv.writer(csv_file)

            writer.writerow(
                [
                    "elapsed_s",
                    "stabilizer_thrust",
                    "height_m",
                    "vertical_velocity_m_s",
                    "battery_voltage_v",
                ]
            )

            for sample in self.samples:
                writer.writerow(
                    [
                        f"{sample.elapsed_s:.6f}",
                        f"{sample.thrust:.6f}",
                        self._format_optional(
                            sample.height_m
                        ),
                        self._format_optional(
                            sample.vertical_velocity_m_s
                        ),
                        self._format_optional(
                            sample.battery_voltage_v
                        ),
                    ]
                )

        return output_path

    @staticmethod
    def _format_optional(
        value: float | None,
    ) -> str:
        if value is None:
            return ""

        return f"{value:.6f}"

    def print_summary(
        self,
        target_height_m: float,
    ) -> None:
        """Print hover-thrust and altitude statistics."""
        if not self.samples:
            print("[WARN] No hover samples to summarize.")
            return

        thrust_values = [
            sample.thrust
            for sample in self.samples
        ]

        mean_thrust = statistics.fmean(thrust_values)
        median_thrust = statistics.median(thrust_values)

        if len(thrust_values) >= 2:
            thrust_std = statistics.stdev(thrust_values)
        else:
            thrust_std = 0.0

        print()
        print("======================================")
        print("HOVER THRUST RESULT")
        print("======================================")
        print(
            f"Samples:              {len(thrust_values)}"
        )
        print(
            f"Mean thrust:          {mean_thrust:.2f}"
        )
        print(
            f"Median thrust:        {median_thrust:.2f}"
        )
        print(
            f"Thrust standard dev:  {thrust_std:.2f}"
        )
        print(
            f"Minimum thrust:       {min(thrust_values):.2f}"
        )
        print(
            f"Maximum thrust:       {max(thrust_values):.2f}"
        )

        height_values = [
            sample.height_m
            for sample in self.samples
            if sample.height_m is not None
        ]

        if height_values:
            mean_height = statistics.fmean(height_values)
            mean_height_error = (
                mean_height - target_height_m
            )

            print(
                f"Mean measured height: {mean_height:.4f} m"
            )
            print(
                f"Mean height error:    "
                f"{mean_height_error:+.4f} m"
            )

        battery_values = [
            sample.battery_voltage_v
            for sample in self.samples
            if sample.battery_voltage_v is not None
        ]

        if battery_values:
            print(
                f"Mean battery voltage: "
                f"{statistics.fmean(battery_values):.3f} V"
            )

        print("======================================")


def reset_estimator(cf: Crazyflie) -> None:
    """Reset the onboard Kalman estimator."""
    print("[INFO] Resetting estimator...")

    cf.param.set_value(
        "kalman.resetEstimation",
        "1",
    )
    time.sleep(0.1)

    cf.param.set_value(
        "kalman.resetEstimation",
        "0",
    )
    time.sleep(2.0)

    print("[INFO] Estimator reset complete.")


def arm(cf: Crazyflie) -> None:
    """Request arming through the Crazyflie supervisor."""
    print("[INFO] Sending ARM request...")

    cf.supervisor.send_arming_request(True)
    time.sleep(1.0)

    print("[INFO] ARM request sent.")


def disarm(cf: Crazyflie) -> None:
    """Request normal disarming through the supervisor."""
    print("[INFO] Sending DISARM request...")

    try:
        cf.commander.send_stop_setpoint()
        time.sleep(0.1)

        cf.supervisor.send_arming_request(False)
        time.sleep(0.2)

        print("[INFO] DISARM request sent.")

    except Exception as error:
        print(
            f"[WARN] Could not complete normal disarm: "
            f"{error}"
        )


def emergency_stop(cf: Crazyflie) -> None:
    """
    Trigger the latching supervisor emergency stop.

    The Crazyflie must be rebooted before flying again.
    """
    print()
    print("======================================")
    print("!!! SUPERVISOR EMERGENCY STOP !!!")
    print("!!! MOTORS WILL STOP IMMEDIATELY  !!!")
    print("======================================")

    try:
        for _ in range(3):
            cf.supervisor.send_emergency_stop()
            time.sleep(0.05)

        print("[EMERGENCY] E-stop command sent.")
        print(
            "[EMERGENCY] Reboot the Crazyflie "
            "before the next test."
        )

    except Exception as error:
        print(
            f"[ERROR] Could not send E-stop: {error}"
        )
        print(
            "[EMERGENCY] Disconnect the battery "
            "immediately."
        )


def send_hover_command(
    cf: Crazyflie,
    height_m: float,
) -> None:
    """Send one body-frame hover setpoint."""
    cf.commander.send_hover_setpoint(
        0.0,       # vx, body-frame m/s
        0.0,       # vy, body-frame m/s
        0.0,       # yaw rate, degrees/s
        height_m,  # distance to surface, meters
    )


def hold_height(
    cf: Crazyflie,
    height_m: float,
    duration_s: float,
) -> None:
    """Continuously send one fixed-height hover command."""
    end_time = time.monotonic() + duration_s

    while time.monotonic() < end_time:
        send_hover_command(
            cf,
            height_m,
        )
        time.sleep(COMMAND_PERIOD_S)


def takeoff(
    cf: Crazyflie,
    target_height_m: float,
) -> None:
    """
    Ramp to the experiment height.

    This data is not included in the hover-thrust statistics.
    """
    print(
        f"[INFO] Taking off to "
        f"{target_height_m:.2f} m..."
    )

    ramp_duration_s = 2.0
    ramp_steps = int(
        ramp_duration_s / COMMAND_PERIOD_S
    )

    start_height_m = 0.05

    for step in range(ramp_steps):
        interpolation = (
            (step + 1) / ramp_steps
        )

        commanded_height_m = (
            start_height_m
            + interpolation
            * (target_height_m - start_height_m)
        )

        send_hover_command(
            cf,
            commanded_height_m,
        )
        time.sleep(COMMAND_PERIOD_S)

    # Brief settling time before the measured five-second interval.
    print("[INFO] Allowing height controller to settle...")
    hold_height(
        cf,
        target_height_m,
        duration_s=1.0,
    )

    print("[INFO] Takeoff and settling complete.")


def measured_hover(
    cf: Crazyflie,
    logger: HoverThrustLogger,
    height_m: float,
    duration_s: float,
) -> None:
    """Hold one height and record exactly this experiment interval."""
    print()
    print(
        f"[INFO] Beginning measured hover at "
        f"{height_m:.2f} m for {duration_s:.1f} s."
    )

    logger.begin_hover_recording()

    try:
        hold_height(
            cf,
            height_m,
            duration_s,
        )

    finally:
        logger.end_hover_recording()

    print("[INFO] Measured hover complete.")


def land(
    cf: Crazyflie,
    start_height_m: float,
) -> None:
    """Ramp down from the experiment height."""
    print("[INFO] Landing...")

    ramp_duration_s = 2.0
    ramp_steps = int(
        ramp_duration_s / COMMAND_PERIOD_S
    )

    final_height_m = 0.05

    for step in range(ramp_steps):
        interpolation = (
            (step + 1) / ramp_steps
        )

        commanded_height_m = (
            start_height_m
            + interpolation
            * (final_height_m - start_height_m)
        )

        send_hover_command(
            cf,
            commanded_height_m,
        )
        time.sleep(COMMAND_PERIOD_S)

    # Continue sending the low height briefly so the supervisor
    # has time to recognize that the vehicle has landed.
    hold_height(
        cf,
        final_height_m,
        duration_s=0.5,
    )

    cf.commander.send_stop_setpoint()
    time.sleep(0.2)

    cf.commander.send_notify_setpoint_stop()
    time.sleep(0.2)

    print("[INFO] Landing sequence complete.")


def countdown(seconds: int = 5) -> None:
    """Provide time to abort before connecting and arming."""
    print("======================================")
    print("Crazyflie hover-thrust experiment")
    print(f"Target height: {TARGET_HEIGHT_M:.2f} m")
    print(f"Measured time: {HOVER_DURATION_S:.1f} s")
    print("Keep one hand near the battery plug.")
    print("Press Ctrl+C for SUPERVISOR E-STOP.")
    print("======================================")

    for remaining in range(
        seconds,
        0,
        -1,
    ):
        print(
            f"[INFO] Starting in {remaining}..."
        )
        time.sleep(1)


def run_hover_thrust_test() -> None:
    """Run one fixed-height hover-thrust experiment."""
    cflib.crtp.init_drivers()

    countdown(5)

    print(f"[INFO] Connecting to {URI}...")

    with SyncCrazyflie(
        URI,
        cf=Crazyflie(rw_cache="./cache"),
    ) as scf:
        cf = scf.cf

        armed = False
        emergency_stop_triggered = False

        thrust_logger = HoverThrustLogger(cf)

        print("[OK] Connected.")

        try:
            reset_estimator(cf)

            # Configure logging before arming. This also verifies that
            # stabilizer.thrust exists before the propellers move.
            thrust_logger.start()

            print()
            print("[WARNING] ARMING THE CRAZYFLIE")
            print("[WARNING] Propellers may begin moving.")
            print()

            arm(cf)
            armed = True

            takeoff(
                cf,
                TARGET_HEIGHT_M,
            )

            measured_hover(
                cf,
                thrust_logger,
                TARGET_HEIGHT_M,
                HOVER_DURATION_S,
            )

            land(
                cf,
                TARGET_HEIGHT_M,
            )

            disarm(cf)
            armed = False

            thrust_logger.stop()

            thrust_logger.print_summary(
                TARGET_HEIGHT_M
            )

            output_path = thrust_logger.save_csv()

            print(
                f"[OK] CSV saved to: "
                f"{output_path.resolve()}"
            )
            print(
                "[INFO] Hover-thrust experiment "
                "finished normally."
            )

        except KeyboardInterrupt:
            emergency_stop_triggered = True

            print("\n[INFO] Ctrl+C detected.")
            emergency_stop(cf)

            armed = False

        except Exception as error:
            emergency_stop_triggered = True

            print(
                f"\n[ERROR] Flight test failed: "
                f"{error}"
            )
            emergency_stop(cf)

            armed = False

        finally:
            thrust_logger.stop()

            if (
                armed
                and not emergency_stop_triggered
            ):
                disarm(cf)

            # Preserve partial measurements when the hover was interrupted.
            if thrust_logger.samples:
                try:
                    partial_path = (
                        thrust_logger.save_csv()
                    )
                    print(
                        f"[INFO] Recorded data saved to: "
                        f"{partial_path.resolve()}"
                    )
                except Exception as error:
                    print(
                        f"[WARN] Could not save data: "
                        f"{error}"
                    )

            print(
                "[INFO] Closing Crazyflie connection."
            )


if __name__ == "__main__":
    try:
        run_hover_thrust_test()

    except KeyboardInterrupt:
        print(
            "\n[INFO] Test aborted before flight."
        )

    except Exception as error:
        print(
            f"[ERROR] Could not run hover test: "
            f"{error}"
        )

    finally:
        print("[INFO] Program exited.")