from __future__ import annotations

from velocity_control_framework.interfaces import BodyRateThrustCommand


class PrintingCommandAdapter:
    """
    Print controller commands without sending them to hardware.

    Intended for integration and dry-run testing.
    """

    def send(
        self,
        command: BodyRateThrustCommand,
    ) -> None:
        print(
            "command:"
            f" roll_rate={command.roll_rate:.4f},"
            f" pitch_rate={command.pitch_rate:.4f},"
            f" yaw_rate={command.yaw_rate:.4f},"
            f" thrust={command.thrust:.4f}"
        )

    def stop(self) -> None:
        print("command adapter stopped")
