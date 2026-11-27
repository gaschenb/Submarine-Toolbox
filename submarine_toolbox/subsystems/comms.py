"""Communications: shore broadcast schedule, VLF floating wire (receive
only, shallow), SATCOM/HF via comms mast at PD (two-way).

Shore transmits a numbered broadcast on a fixed schedule; how far behind
the boat's inbox is measures its connectivity discipline. Outgoing
traffic queues until a two-way path exists.
"""

from __future__ import annotations

from submarine_toolbox.core.subsystem import Subsystem

class Comms(Subsystem):
    name = "comms"

    BROADCAST_PERIOD = 600.0
    VLF_DEPTH = 50.0
    VLF_DELAY = 60.0

    def __init__(self, sim):
        super().__init__(sim)
        self.wire_deployed = False
        self.received = 0
        self.outbox = 0
        self.sent = 0
        self._last_path = "none"

    def queue_message(self):
        self.outbox += 1

    def update(self, dt: float) -> None:
        t = self.sim.time
        n_broadcast = int(t // self.BROADCAST_PERIOD)
        depth = self.get("state.depth", 99.0)

        mast_link = self.get("masts.comms", False) and depth <= 18.0
        vlf_link = self.wire_deployed and depth <= self.VLF_DEPTH

        if mast_link:
            self.received = n_broadcast
            self.sent += self.outbox
            self.outbox = 0
            self._last_path = "SATCOM/HF"
        elif vlf_link:
            avail = int(max(t - self.VLF_DELAY, 0.0)
                        // self.BROADCAST_PERIOD)
            self.received = max(self.received, avail)
            self._last_path = "VLF wire"
        else:
            self._last_path = "none"

        self.put("comms.missed", n_broadcast - self.received)
        self.put("comms.received", self.received)
        self.put("comms.outbox", self.outbox)
        self.put("comms.sent", self.sent)
        self.put("comms.path", self._last_path)
        self.put("comms.wire", self.wire_deployed)
