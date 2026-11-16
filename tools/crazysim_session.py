"""CFLib session helpers for command-free CrazySim diagnostics."""

from __future__ import annotations

from cflib.crazyflie import Crazyflie, State


class PassiveCrazyflie(Crazyflie):
    """Crazyflie whose close path does not send CFLib's legacy zero setpoint.

    CFLib's standard ``close_link`` sends ``send_setpoint(0, 0, 0, 0)`` before
    closing. That packet unlocks the legacy RPYT thrust path, so passive
    diagnostics use this narrowly scoped close implementation instead.
    """

    def close_link(self) -> None:
        if self.link is not None:
            self.link.close()
            self.link = None
        if self.incoming:
            callbacks = list(self.incoming.cb)
            if self.incoming.is_alive():
                self.incoming.stop()
                self.incoming.join(timeout=1)
            self.incoming = type(self.incoming)(self)
            self.incoming.cb = callbacks
            self.incoming.daemon = True
        self._answer_patterns = {}
        self.disconnected.call(self.link_uri)
        self.state = State.DISCONNECTED
