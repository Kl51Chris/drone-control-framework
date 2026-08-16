from __future__ import annotations

from velocity_control_framework.interfaces import AttitudeThrustCommand


class PrintingCommandAdapter:
    """
    Print controller commands without sending them to hardware.

    Intended for integration and dry-run testing.
    """

    def send(
        self,
        command: AttitudeThrustCommand,
    ) -> None:
        print(
            "command:"
            f" roll={command.roll:.4f},"
            f" pitch={command.pitch:.4f},"
            f" yaw_rate={command.yaw_rate:.4f},"
            f" thrust={command.thrust:.4f}"
        )

    def stop(self) -> None:
        print("command adapter stopped")