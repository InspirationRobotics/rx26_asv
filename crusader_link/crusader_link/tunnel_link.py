# tunnel_link.py -- a byte copy, below this header, of rx26_uav
#   uav_common/uav_common/boat_link.py   at rx26_uav commit 2ab6f93
# Every radio on the mesh packs and unpacks with the same bytes, and a ROS package cannot
# import across repos, so this is a copy. Do not edit below the header: re-copy the file and
# update the commit. test_rfd_link.py fails when it drifts.
"""boat_link -- the bytes Ekko and Crusader exchange over the RFD900ux mesh.

PURE: struct packing and the bookkeeping around it; no ROS, no MAVLink import,
no other uav_common module (fake_crusader.py runs on a bare laptop with this
file beside it). Every radio on the mesh (Ekko, the ground laptop, Crusader, a
laptop standing in for Crusader) packs and unpacks with THIS file, so the ends
cannot drift apart.

CARRIED BY MAVLINK `TUNNEL` (128 bytes of vendor payload per message): the mesh
also carries QGC and the autopilots' own telemetry, and Ekko's Pixhawk parses
every byte it hears, so anything we send must be a MAVLink frame. Each frame
costs about 17 bytes of envelope, which is why each packet below carries
everything of its kind at once rather than one fact per packet.

NO FIELD NAMES ON THE AIR: the order of the bytes IS the meaning. The shapes (v3):

  POSITIONS  drone -> all   map(1) count(1), then per buoy  slot(1) id(2)
             lat(4) lon(4). Sent when a buoy's light is first decided, again
             whenever its mapped position moves by MOVE_M, and the WHOLE map
             every REFRESH_S -- so a listener that joins late, restarts, or
             lost a packet has the map within one refresh, acknowledged or not.
  LIGHTS     drone -> all   every second: map(1), the confirmed gate or exit
             (two slots, 0 = none), then ONE BYTE per buoy -- slot in the top 5
             bits, light in the bottom 3. It is the WHOLE current map: a slot
             missing from it is a buoy that is no longer on the map, and every
             receiver drops it.
  BOAT       boat -> all    every second: lat(4) lon(4) activity(1) target
             slot(1), a 32-bit mask of the slots whose positions the boat HOLDS
             (that mask is the acknowledgement), and optionally map(1) -- which
             map those slots belong to.
  TEST       anyone -> all: a text line the ground station's Radio tab puts on
             the air so an operator can watch one known frame cross it.
             NEITHER VEHICLE ACTS ON IT.
  GNSS       drone -> all   every second while fresh: mode(1) satellites(1)
             h-accuracy cm(2) v-accuracy cm(2), from Ekko's Septentrio ITSELF.
             For the ground laptop: the autopilot cannot say "HAS" (ArduPilot
             4.7 has no fix type for the receiver's PPP mode), and the laptop's
             page only knows what crosses this link. NOTHING ACTS ON IT.

THE MAP NUMBER, AND WHY v2 NEEDED IT. v2 sent each buoy's position ONCE, keyed
by buoy_mapper's tracker id -- and the tracker restarts its ids at B1 on every
"Clear buoys". At the park on 26 Sep 2026 the manual flights had used B1-B10,
the map was cleared, and the autonomous search then found ten buoys of which six
(B1, B2, B3, B5, B6, B9) had ids the sender had already sent: their positions
never went out, and their lights went out under the OLD buoys' slots. The ground
laptop showed 4 of 10; a boat that had heard the first map would have put the
new lights on the old positions. Now every map gets a number, slots restart per
map, and a receiver that hears a new number throws the old map away. The v2
payload types (32770, 32771) are retired rather than re-shaped, so an old
decoder ignores v3 packets instead of misreading them.

SLOTS, NOT TRACKER IDS, ON THE AIR. buoy_mapper numbers every track it ever
starts, false ones included, so ids are not 1-10 in a real session. Each buoy
gets a radio slot 1-31 within its map the first time its position is sent, and
the POSITIONS record carries its real id, so every receiver still labels it
"B7". A slot is never reused within one map, so a merged-away buoy's number can
never come back meaning someone else.
"""
import math
import os
import struct

#: TUNNEL payload types (vendor range, above 32767). Retired numbers are never
#: reused, so an old sender cannot be misread as a new one:
#:   32769  v1 whole-map packet        32770 / 32771  v2 positions / lights
PAYLOAD_BOAT = 32768         # boat -> all
PAYLOAD_POSITIONS = 32772    # drone -> all (v3: carries the map number)
PAYLOAD_LIGHTS = 32773       # drone -> all (v3: carries the map number)
PAYLOAD_GNSS = 32774         # drone -> all: the main GNSS receiver's own status
PAYLOAD_TEST = 33022         # 0x80FE, anyone -> anyone: a text line nobody acts on
RETIRED = {32769: "v1 whole map", 32770: "v2 positions", 32771: "v2 lights"}
MAX_PAYLOAD = 128

#: The receiver's PVT mode, as the Septentrio numbers it (SBF PVTGeodetic Mode
#: bits 0-3) and as the GNSS packet carries it. 10 is PPP; the only PPP service
#: Ekko's mosaic-G5 P8 is permitted is Galileo HAS, so it is shown as "HAS".
GNSS_MODES = {0: "no fix", 1: "standalone", 2: "DGNSS", 3: "fixed position",
              4: "RTK fixed", 5: "RTK float", 6: "SBAS", 7: "moving-base RTK fixed",
              8: "moving-base RTK float", 10: "HAS"}
_GNSS = struct.Struct("<BBHH")         # mode, satellites, h/v accuracy cm = 6 bytes

#: Light states, by code. Index IS the wire value; append only, never reorder.
#: Three bits: at most eight.
STATES = ("UNKNOWN", "OFF", "FLASHING_RED", "FLASHING_GREEN", "FLASHING_BLUE",
          "SOLID_BLUE")
CODE = {name: i for i, name in enumerate(STATES)}

#: What the boat says it is doing. Same values as uav_msgs/BoatState.
ACTIVITY = ("unknown", "holding", "circling the entry buoy", "transiting",
            "circling the exit buoy", "done")

MAX_SLOT = 31                          # five bits
_POS = struct.Struct("<BHii")          # slot, id, lat 1e-7, lon 1e-7 = 11 bytes
_BOAT = struct.Struct("<iiBBI")        # lat, lon, activity, target slot, acked = 14
MAX_POSITIONS = (MAX_PAYLOAD - 2) // _POS.size   # 11 to a packet


def pad(payload):
    """A TUNNEL payload field is always 128 bytes; payload_length says how much
    of it is real. (MAVLink 2 trims the trailing zeros off the air.)"""
    body = bytearray(MAX_PAYLOAD)
    body[:len(payload)] = payload
    return bytes(body)


def body(msg):
    """The real bytes of a received TUNNEL, without the padding."""
    return bytes(msg.payload)[:msg.payload_length]


def _metres(a, b):
    """Ground distance between two (lat, lon), flat-earth: fine for metres."""
    dn = (a[0] - b[0]) * 111320.0
    de = (a[1] - b[1]) * 111320.0 * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


# ------------------------------------------------------------------ packets

def pack_positions(records, epoch):
    """[(slot, buoy_id, lat, lon)] for map `epoch` -> [payload, ...],
    MAX_POSITIONS per packet."""
    out = []
    records = list(records)
    for i in range(0, len(records), MAX_POSITIONS):
        chunk = records[i:i + MAX_POSITIONS]
        buf = bytearray([int(epoch) & 0xFF, len(chunk)])
        for slot, bid, lat, lon in chunk:
            buf += _POS.pack(int(slot), int(bid) & 0xFFFF,
                             int(round(lat * 1e7)), int(round(lon * 1e7)))
        out.append(bytes(buf))
    return out


def unpack_positions(payload):
    """-> {"epoch": map number or None, "buoys": [{"slot", "id", "lat", "lon"}]}"""
    raw = bytes(payload)
    if len(raw) < 2:
        return {"epoch": None, "buoys": []}
    epoch, n = raw[0], raw[1]
    out = []
    for i in range(min(n, MAX_POSITIONS)):
        off = 2 + i * _POS.size
        if off + _POS.size > len(raw):
            break
        slot, bid, lat, lon = _POS.unpack_from(raw, off)
        out.append({"slot": slot, "id": bid, "lat": lat / 1e7, "lon": lon / 1e7})
    return {"epoch": epoch, "buoys": out}


def pack_lights(lights, confirmed=(), *, epoch):
    """{slot: label} + confirmed slots ([red, green], [exit] or []) for map
    `epoch` -> payload. `lights` is the WHOLE map: whatever is missing from it
    every receiver drops."""
    c = [int(s) for s in list(confirmed)[:2]]
    buf = bytearray([int(epoch) & 0xFF] + c + [0] * (2 - len(c)))
    for slot in sorted(lights):
        buf.append((int(slot) & MAX_SLOT) << 3 | CODE.get(lights[slot], 0))
    return bytes(buf)


def unpack_lights(payload):
    """-> {"epoch", "confirmed": [slots], "lights": {slot: label}}"""
    raw = bytes(payload)
    if len(raw) < 3:
        return {"epoch": None, "confirmed": [], "lights": {}}
    confirmed = [s for s in raw[1:3] if s]
    lights = {}
    for b in raw[3:]:
        slot, code = b >> 3, b & 0x07
        if slot:
            lights[slot] = STATES[code] if code < len(STATES) else "UNKNOWN"
    return {"epoch": raw[0], "confirmed": confirmed, "lights": lights}


def pack_boat(lat, lon, activity, target_slot=0, acked=(), epoch=None):
    """-> payload. `acked`: the slots whose positions this boat holds; `epoch`:
    the map they belong to (Receiver.epoch). A boat that leaves `epoch` out still
    works, but its acknowledgements are only trusted a few seconds after a new
    map starts -- see Sender.ACK_GRACE_S."""
    mask = 0
    for s in acked:
        if 1 <= s <= MAX_SLOT:
            mask |= 1 << s
    out = _BOAT.pack(int(round(lat * 1e7)), int(round(lon * 1e7)),
                     int(activity) & 0xFF, int(target_slot) & 0xFF, mask)
    if epoch:
        out += bytes([int(epoch) & 0xFF])
    return out


def unpack_boat(payload):
    raw = bytes(payload)
    lat, lon, activity, target, mask = _BOAT.unpack(raw[:_BOAT.size])
    epoch = raw[_BOAT.size] if len(raw) > _BOAT.size and raw[_BOAT.size] else None
    return {"lat": lat / 1e7, "lon": lon / 1e7, "activity": activity,
            "target_slot": target, "epoch": epoch,
            "acked": {s for s in range(1, MAX_SLOT + 1) if mask & (1 << s)}}


# ------------------------------------------------------------------ Ekko's side

class Sender:
    """What Ekko puts on the air, and when. Pure: the caller passes the clock.

    feed(buoys, confirmed_ids, now, map_id) with the map as
    [(id, lat, lon, label)], the ids the search confirms, and WHICH map this is
    (buoy_mapper's export stem: it changes on every clear); boat_heard(acked,
    now, epoch) with each boat report; then due(now) -> [(payload_type,
    payload)] to transmit.
    """

    LIGHTS_PERIOD_S = 1.0
    #: How long to wait for the boat to acknowledge a position before sending
    #: it again. Only while a boat is actually being heard.
    RESEND_S = 3.0
    #: The whole map, to everyone, this often, acknowledged or not. This is what
    #: lets the ground laptop (which never acknowledges) and a boat that starts
    #: listening late have every buoy. 10 buoys is one packet: ~0.1 kbit/s.
    REFRESH_S = 10.0
    #: A buoy whose mapped position has moved this far since it was last sent is
    #: sent again. The mapper keeps refining positions after the light is
    #: decided; v2 froze them at that first moment. Well under the 3 m gate
    #: spacing, well over the jitter of a converged track.
    MOVE_M = 0.25
    #: ...but not more often than this per buoy, so a track still settling does
    #: not put a packet on the air every second.
    MIN_MOVE_GAP_S = 2.0
    BOAT_HEARD_S = 5.0
    #: After a new map starts, a boat report WITHOUT a map number may still be
    #: describing the old map's slots. Its acknowledgements are not trusted
    #: until this long after the change; a report that carries the number is
    #: trusted, or ignored, at once.
    ACK_GRACE_S = 3.0

    def __init__(self, first_epoch=None):
        # A random first number, so a power cycle mid-session is a new map to
        # every receiver even though this process has forgotten the last one.
        self._next_epoch = int(first_epoch) if first_epoch else 1 + os.urandom(1)[0] % 255
        self.epoch = 0             # 0 = no map yet: nothing goes on the air
        self.map_id = None
        self._map_t = None
        self.boat_t = None
        self.lights_t = None
        self.refresh_t = None
        self.maps = 0              # how many maps this process has started
        self._reset()

    def _reset(self):
        self.slots = {}            # buoy id -> slot, THIS map
        self.pos = {}              # slot -> (id, lat, lon), latest from the mapper
        self.sent = {}             # slot -> (lat, lon, when) last put on the air
        self.pending = set()       # slots whose position is due now
        self.lights = {}           # slot -> label
        self.confirmed = []        # slots
        self.acked = set()
        self.overflow = []         # buoy ids with no slot left in this map
        self._next_slot = 1

    def new_map(self, map_id, now):
        """Forget the old map and start numbering again under a new map number."""
        self._reset()
        self.map_id = map_id
        self.epoch = self._next_epoch
        self._next_epoch = self._next_epoch % 255 + 1
        self._map_t = now
        self.lights_t = None       # tell every receiver at once
        self.refresh_t = now
        self.maps += 1

    def slot_of(self, buoy_id):
        return self.slots.get(buoy_id)

    def id_of(self, slot):
        return next((b for b, s in self.slots.items() if s == slot), None)

    def feed(self, buoys, confirmed_ids, now, map_id=None):
        if self.epoch == 0 or map_id != self.map_id:
            self.new_map(map_id, now)
        present = set()
        for bid, lat, lon, label in buoys:
            slot = self.slots.get(bid)
            if slot is None:
                if label == "UNKNOWN":
                    continue          # not decided yet: nothing worth a position
                if self._next_slot > MAX_SLOT:
                    if bid not in self.overflow:
                        self.overflow.append(bid)
                    continue
                slot = self._next_slot
                self._next_slot += 1
                self.slots[bid] = slot
                self.pending.add(slot)
            self.pos[slot] = (bid, lat, lon)
            last = self.sent.get(slot)
            if (last is not None and slot not in self.pending
                    and now - last[2] >= self.MIN_MOVE_GAP_S
                    and _metres((lat, lon), last[:2]) >= self.MOVE_M):
                self.pending.add(slot)
            self.lights[slot] = label
            present.add(slot)
        # A buoy the mapper no longer has -- merged into another track, or
        # deleted -- leaves the air: absent from LIGHTS, every receiver drops it.
        for bid, slot in list(self.slots.items()):
            if slot not in present:
                del self.slots[bid]
                for held in (self.pos, self.sent, self.lights):
                    held.pop(slot, None)
                self.pending.discard(slot)
                self.acked.discard(slot)
        self.confirmed = [self.slots[b] for b in confirmed_ids if b in self.slots][:2]

    def boat_heard(self, acked_slots, now, epoch=None):
        self.boat_t = now
        if epoch is not None and epoch != self.epoch:
            return                # acknowledging a map that is not this one
        if epoch is None and (self._map_t is None
                              or now - self._map_t < self.ACK_GRACE_S):
            return                # may still be the old map's slots
        self.acked = set(acked_slots) & set(self.pos)

    def boat_listening(self, now):
        return self.boat_t is not None and now - self.boat_t <= self.BOAT_HEARD_S

    def due(self, now):
        out = []
        if self.epoch == 0:
            return out
        send = set(self.pending)
        if self.boat_listening(now):
            send |= {s for s, (_la, _lo, t) in self.sent.items()
                     if s not in self.acked and now - t >= self.RESEND_S}
        if self.pos and (self.refresh_t is None
                         or now - self.refresh_t >= self.REFRESH_S):
            send |= set(self.pos)
            self.refresh_t = now
        records = []
        for s in sorted(send):
            if s in self.pos:
                bid, lat, lon = self.pos[s]
                records.append((s, bid, lat, lon))
                self.sent[s] = (lat, lon, now)
        out += [(PAYLOAD_POSITIONS, p) for p in pack_positions(records, self.epoch)]
        self.pending = set()
        # Every period, even for an EMPTY map: that is how a Clear reaches the
        # receivers -- a new map number with no buoys in it.
        if self.lights_t is None or now - self.lights_t >= self.LIGHTS_PERIOD_S:
            self.lights_t = now
            out.append((PAYLOAD_LIGHTS, pack_lights(self.lights, self.confirmed,
                                                    epoch=self.epoch)))
        return out


# ------------------------------------------------------------------ the boat's side

class Receiver:
    """What a boat (or any laptop on the mesh) builds from what it hears."""

    def __init__(self):
        self.epoch = None          # the map being held
        self.positions = {}        # slot -> {"slot","id","lat","lon"}
        self.lights = {}           # slot -> label
        self.confirmed_slots = []
        self.maps = 0              # how many maps have been heard

    def _map(self, epoch):
        """A packet from another map: everything held belongs to the old one."""
        if epoch != self.epoch:
            self.epoch = epoch
            self.positions = {}
            self.lights = {}
            self.confirmed_slots = []
            self.maps += 1

    def hear(self, payload_type, payload):
        """Feed one TUNNEL payload. -> the packet kind it was, or None."""
        if payload_type == PAYLOAD_POSITIONS:
            rep = unpack_positions(payload)
            if rep["epoch"] is None:
                return None
            self._map(rep["epoch"])
            for rec in rep["buoys"]:
                self.positions[rec["slot"]] = rec
            return "positions"
        if payload_type == PAYLOAD_LIGHTS:
            rep = unpack_lights(payload)
            if rep["epoch"] is None:
                return None
            self._map(rep["epoch"])
            self.lights = rep["lights"]
            self.confirmed_slots = rep["confirmed"]
            # LIGHTS is the whole map: a held position it no longer lists is a
            # buoy Ekko merged or dropped.
            for s in [s for s in self.positions if s not in self.lights]:
                del self.positions[s]
            return "lights"
        return None

    def acked(self):
        """The slots whose positions are held: sent back as the acknowledgement
        (with self.epoch, so Ekko knows which map they belong to)."""
        return set(self.positions)

    def slot_of(self, buoy_id):
        """The slot a buoy id travels as, or 0 when its position is not held."""
        return next((s for s, rec in self.positions.items() if rec["id"] == buoy_id), 0)

    def buoys(self):
        """[{"id", "lat", "lon", "label", "slot"}] for every buoy whose position
        is held. A light heard for a slot with no position yet is not a buoy
        anyone can steer by."""
        return [dict(rec, label=self.lights.get(s, "UNKNOWN"))
                for s, rec in sorted(self.positions.items())]

    def confirmed(self):
        """The confirmed gate or exit as buoy ids (not slots), or []."""
        ids = [self.positions[s]["id"] for s in self.confirmed_slots if s in self.positions]
        return ids if len(ids) == len(self.confirmed_slots) else []


def pack_test(text):
    """A PAYLOAD_TEST line, cut to one TUNNEL. ASCII, so it reads the same in
    QGC's MAVLink Inspector as on the Radio tab."""
    return str(text).encode("ascii", "replace")[:MAX_PAYLOAD]


def _cm(m):
    return 65535 if m is None else max(0, min(65534, int(round(m * 100))))


def pack_gnss(mode, satellites, h_acc_m, v_acc_m):
    """-> payload. Unknowns (None) travel as the "unknown" values, 255 and
    65535, so a receiver shows a blank rather than a made-up number."""
    return _GNSS.pack(int(mode) & 0xFF, 255 if satellites is None else min(254, int(satellites)),
                      _cm(h_acc_m), _cm(v_acc_m))


def unpack_gnss(payload):
    """-> {"mode", "mode_name", "satellites", "h_acc_m", "v_acc_m"}, or None if
    the payload is too short to be one."""
    raw = bytes(payload)
    if len(raw) < _GNSS.size:
        return None
    mode, nsv, h, v = _GNSS.unpack_from(raw)
    return {"mode": mode, "mode_name": GNSS_MODES.get(mode, "mode %d" % mode),
            "satellites": None if nsv == 255 else nsv,
            "h_acc_m": None if h == 65535 else h / 100.0,
            "v_acc_m": None if v == 65535 else v / 100.0}


def describe(payload_type, payload):
    """(name, one line) for a TUNNEL payload, for the Radio tab.

    NEVER RAISES. A packet that does not decode is still something that crossed
    the link, and the Radio tab is exactly where it needs to show up -- a
    describer that threw would hide the one frame worth looking at.
    """
    raw = bytes(payload)
    name = "TUNNEL_0x%04X" % (int(payload_type) & 0xFFFF)
    try:
        if payload_type == PAYLOAD_BOAT:
            b = unpack_boat(raw)
            doing = (ACTIVITY[b["activity"]] if b["activity"] < len(ACTIVITY)
                     else "activity %d" % b["activity"])
            # The ack mask is the whole point of the boat's packet: say how many
            # positions it holds, not just where it is.
            target = ", wants slot %d" % b["target_slot"] if b["target_slot"] else ""
            of_map = " of map %d" % b["epoch"] if b["epoch"] else ""
            return "BOAT", "%.7f %.7f, %s%s, holds %d position(s)%s" % (
                b["lat"], b["lon"], doing, target, len(b["acked"]), of_map)
        if payload_type == PAYLOAD_POSITIONS:
            rep = unpack_positions(raw)
            if not rep["buoys"]:
                return "POSITIONS", "empty"
            return "POSITIONS", "map %d, %d buoy(s): %s" % (
                rep["epoch"], len(rep["buoys"]),
                ", ".join("B%d as slot %d" % (r["id"], r["slot"]) for r in rep["buoys"]))
        if payload_type == PAYLOAD_LIGHTS:
            rep = unpack_lights(raw)
            c = rep["confirmed"]
            if len(c) == 2:
                confirmed = "gate red slot %d / green slot %d" % (c[0], c[1])
            elif c:
                confirmed = "exit slot %d" % c[0]
            else:
                confirmed = "nothing"
            return "LIGHTS", "map %s, %d light(s), confirmed %s" % (
                rep["epoch"], len(rep["lights"]), confirmed)
        if payload_type == PAYLOAD_TEST:
            return "TEST", raw.decode("ascii", "replace")
        if payload_type == PAYLOAD_GNSS:
            g = unpack_gnss(raw)
            if g is None:
                return "GNSS", "too short (%d bytes)" % len(raw)
            acc = lambda m: "?" if m is None else "%.2f m" % m
            return "GNSS", "%s, %s sats, accuracy %s H / %s V" % (
                g["mode_name"], "?" if g["satellites"] is None else g["satellites"],
                acc(g["h_acc_m"]), acc(g["v_acc_m"]))
        if payload_type in RETIRED:
            return name, "%s (retired format, %d bytes): update the sender" % (
                RETIRED[payload_type], len(raw))
    except Exception as e:                     # noqa: BLE001 -- see docstring
        return name, "does not decode (%d bytes): %s" % (len(raw), e)
    return name, "%d bytes, not a format boat_link knows" % len(raw)
