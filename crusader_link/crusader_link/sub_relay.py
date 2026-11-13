"""sub_relay: the rules for carrying Graey (the UUV) through the boat. No ROS, no sockets,
no pymavlink: rfd_link_node owns the radio and the UDP socket and asks this what to do.

WHY THE BOAT. Graey keys the RFD mesh only at the surface, with its antenna at the waterline:
it reaches the boat, on the water beside it, far better than the dock. The boat already has
the 5.8 GHz link to the OCS laptop, so it carries Graey's frames the rest of the way:

  up    every frame heard on the radio from one of Graey's systems (its Cube and its
        companion) -> one UDP datagram to fleet_link on the OCS laptop. Whole and unchanged,
        so the laptop sees exactly what its own radio would have heard. Costs no air.
  down  what the laptop sends back for Graey's companion (RunStart, mission requests) -> the
        radio, whole and unchanged, so Graey sees the OCS's own system id and answers it.

Nothing is decoded, re-stamped or invented here: a frame is relayed or it is not.

THE DOWN LEG KEYS THE SHARED AIR, so it is the narrow one. Only a TUNNEL, only from the
systems named in `down_sources` (the OCS adapter and the Graey laptop client), only addressed
to Graey's companion, and under a byte budget: the same mesh carries the BOAT packet Ekko
needs for Task 1. Everything refused is counted by reason, never queued for later.
"""
import ipaddress

#: fleet.toml on the OCS laptop: [ocs] sysid (the adapter) and the Graey laptop client.
DOWN_SOURCES = (250, 254)
DOWN_BYTES_PER_S = 600.0      # sustained down-leg budget; a RunStart is ~30 bytes on the air
DOWN_BURST_BYTES = 2400.0     # one mission document's chunks, back to back


def parse_target(text):
    """'192.168.8.255:14573' -> ('192.168.8.255', 14573); '' -> None (the relay is off).

    An IPv4 literal only: a name would be resolved on the radio thread. A subnet broadcast
    address is fine (the socket is opened for it) and means no laptop address is typed here.
    Raises ValueError with the reason for anything else, so a typo stops the node at startup
    instead of relaying into nowhere."""
    text = str(text or "").strip()
    if not text:
        return None
    host, sep, port = text.rpartition(":")
    if not sep or not host:
        raise ValueError("sub_relay_target %r is not ip:port" % text)
    try:
        address = ipaddress.IPv4Address(host)
        number = int(port)
    except ValueError:
        raise ValueError("sub_relay_target %r is not an IPv4 address and a port" % text) from None
    if not 1 <= number <= 65535:
        raise ValueError("sub_relay_target port %d is outside 1-65535" % number)
    if address.is_multicast or address.is_unspecified:
        raise ValueError("sub_relay_target %s cannot be the OCS laptop" % host)
    third = int(str(address).split(".")[2])
    if str(address).startswith("192.168.") and 65 <= third <= 68:
        raise ValueError("sub_relay_target %s is a RoboCommand course network: the boat "
                         "never speaks there" % host)
    return str(address), number


class SubRelay:
    """Which frames cross, in which direction, and how much air the down leg may spend."""

    def __init__(self, sub_sysids, companion_sysid, *, down_sources=DOWN_SOURCES,
                 bytes_per_s=DOWN_BYTES_PER_S, burst_bytes=DOWN_BURST_BYTES):
        self.sub_sysids = frozenset(int(s) for s in sub_sysids)
        self.companion = int(companion_sysid)
        if self.companion not in self.sub_sysids:
            raise ValueError("sub_companion_sysid %d must be one of sub_sysids %s"
                             % (self.companion, sorted(self.sub_sysids)))
        self.down_sources = frozenset(int(s) for s in down_sources)
        if self.down_sources & self.sub_sysids:
            raise ValueError("a system cannot be both Graey and the OCS: a frame would loop")
        self.rate, self.burst = float(bytes_per_s), float(burst_bytes)
        self._tokens, self._filled = self.burst, None
        self.up_frames = self.up_bytes = self.down_frames = self.down_bytes = 0
        self.refused = {}
        self.last_up = None

    def up(self, src_system, nbytes, now):
        """Radio -> laptop? True for Graey's own systems, and nothing else on the mesh."""
        if int(src_system) not in self.sub_sysids:
            return False
        self.up_frames += 1
        self.up_bytes += int(nbytes)
        self.last_up = now
        return True

    def down(self, src_system, msg_type, target_system, nbytes, now):
        """Laptop -> radio? None when the frame may be keyed, else the reason it is refused."""
        why = self._down_reason(src_system, msg_type, target_system, nbytes, now)
        if why is None:
            self.down_frames += 1
            self.down_bytes += int(nbytes)
        else:
            self.refused[why] = self.refused.get(why, 0) + 1
        return why

    def _down_reason(self, src_system, msg_type, target_system, nbytes, now):
        if msg_type != "TUNNEL":
            return "not a TUNNEL"
        if int(src_system) not in self.down_sources:
            return "source is not the OCS"
        if int(target_system) != self.companion:
            return "not addressed to Graey's companion"
        if self._filled is not None:
            self._tokens = min(self.burst, self._tokens + max(0.0, now - self._filled) * self.rate)
        self._filled = now
        if nbytes > self._tokens:
            return "over the air budget"
        self._tokens -= nbytes
        return None

    def heard(self, now, within_s=3.0):
        """Is Graey on the radio right now? (its frames reach the boat)"""
        return self.last_up is not None and now - self.last_up <= within_s

    def summary(self):
        refused = ", ".join("%s x%d" % kv for kv in sorted(self.refused.items())) or "none"
        return "up %d frames (%d B), down %d frames (%d B), refused: %s" % (
            self.up_frames, self.up_bytes, self.down_frames, self.down_bytes, refused)
