#!/usr/bin/env python3
"""uav_link — the aircraft's half of the Task 1 radio, with no ROS in it.

Shared by the two things that stand in for Ekko:

    bench_uav.py                    a scripted run, no browser
    bench_world_model.py --uav      the same radio, driven from the GUI

EXTRACTED RATHER THAN COPIED, for the reason mjpeg_server.py was: two copies of
a handshake drift, and only one of them gets the fix. The gate protocol has
exactly one subtle rule in it (below) and it must not exist twice.

THE HANDSHAKE, and the one rule that is easy to get wrong:

    boat  -> RXL_USV_REACHED_GATE(seq)      "gate cleared, what next?"
    here  -> RXL_NEXT_BUOY_SET(seq, r, g)   "red r to starboard, green g to port"
    here  -> RXL_NEXT_BUOY_SET(seq, 255,255) when the gates run out

    A REPEATED seq MUST GET THE SAME ANSWER. On a lossy radio the boat re-asks
    when our reply is lost, and answering a repeat with the NEXT pair would skip
    a gate -- the boat would drive past one it never cleared and nothing in any
    log would look wrong. `_answered` is what makes a re-ask idempotent.

This module never decides where the boat goes. It transmits what it is given
and answers what it is asked; the passage layout comes from the caller.
"""
import collections
import threading
import time

from crusader_common import geo
from crusader_link import rxl_codec


def offsets_to_latlon(east_m, north_m, anchor):
    """(east, north) metres from `anchor` -> (lat, lon).

    Straight through crusader_common.geo, which owns M_PER_DEG. This file had
    its own copy of that constant for about an hour; geo.py warns by name
    against exactly that -- "a target the USV reports and the UAV re-derives
    must land in the same place" -- and a bench that places buoys with a
    different earth radius from the boat that reads them is the drift it means.
    """
    return geo.xy_to_latlon(east_m, north_m, anchor)


class UavLink:
    """One radio, one gate list, one log. Thread-safe; poll() from anywhere."""

    def __init__(self, endpoint="udpout:127.0.0.1:14555", source_system=200,
                 on_log=None):
        # source_system must differ from the boat's (42) or an inspector files
        # both vehicles' traffic under one tree.
        self.conn = rxl_codec.connect(endpoint, source_system=source_system)
        self.endpoint = endpoint
        self._lock = threading.Lock()
        self._gates = []                 # [(red_id, green_id)], in order
        self._answered = {}              # seq -> (red, green)
        self._served = 0
        self._log = collections.deque(maxlen=40)
        self._plan = None                # last (buoys, entry, exit) transmitted
        self._odd_pairs = set()          # pairs already complained about
        self._sent = 0
        # The deque is for the GUI, which re-reads the whole tail each poll. A
        # script cannot follow a BOUNDED deque by index -- once it wraps, the
        # index means a different line -- so a caller that wants every line as
        # it happens passes a callback instead.
        self._on_log = on_log

    # ------------------------------------------------------------- outbound

    def set_gates(self, gates):
        """Replace the gate order. [(id, id), ...] in the order the boat meets them.

        The pair may be given either way round: _reorder_gates puts it into
        (red, green) from the plan, because which buoy is red is a fact about
        the buoy and not about the order somebody clicked or typed it.

        Resets what has been served, so editing the order mid-run starts the
        sequence again rather than continuing into a list that changed
        underneath it.
        """
        with self._lock:
            self._gates = [(int(r), int(g)) for r, g in gates]
            self._served = 0
            self._answered.clear()
            self._odd_pairs.clear()
            self._reorder_gates()
            self._say("gate order set: %s" % (self._gates or "none"))

    def _reorder_gates(self):
        """Put every pair into (red, green) order, read off the plan.

        Caller holds self._lock.

        WHY THIS EXISTS, and why the failure was invisible. gateWaypoints on the
        boat derives the crossing heading from bearing(green -> red) - 90. That
        is a pure function of the pair, with no reference to the boat, the entry
        or the exit -- which is exactly what makes it robust when GPS yaw is
        unresolved, and also what makes an INVERTED pair produce a heading 180
        degrees out instead of an error.

        2026-09-13, SITL: the page authored pairs in click order, the operator
        clicked green-then-red, and the aircraft said "red 3, green 1" about a
        green 3 and a red 1. The boat crossed the first gate correctly, turned
        around, and came back through it with RED TO PORT. Every line in every
        log was an [INFO] and none of them was wrong.

        It only ever SWAPS. A pair that is not one red and one green is left
        exactly as authored and said out loud once: dropping a gate silently
        would change the mission, and quietly guessing at one is what caused
        this in the first place.
        """
        if self._plan is None:
            return                       # nothing to check against yet
        colour = {int(i): int(c) for i, _la, _lo, c in self._plan[0]}
        red_c = rxl_codec.BEACON_FLASHING_RED
        green_c = rxl_codec.BEACON_FLASHING_GREEN
        fixed = []
        for n, (red, green) in enumerate(self._gates, 1):
            cr, cg = colour.get(red), colour.get(green)
            if (cr, cg) == (green_c, red_c):
                # Self-limiting: after the swap the pair reads (red, green), so
                # the 0.2 Hz retransmit that calls this again says nothing.
                self._say("gate %d: %d is green and %d is red -- pair swapped"
                          % (n, red, green))
                red, green = green, red
            elif (cr, cg) != (red_c, green_c) and (red, green) not in self._odd_pairs:
                self._odd_pairs.add((red, green))
                self._say("gate %d: %s %d + %s %d is not a red-green pair; "
                          "sending it as authored"
                          % (n, _beacon(cr), red, _beacon(cg), green))
            fixed.append((red, green))

        # A CORRECTED PAIR INVALIDATES THE ANSWER ALREADY GIVEN FOR IT.
        #
        # _answered exists so a repeated seq gets the same reply and a lost
        # reply cannot skip a gate. That guarantee is about the RADIO, not about
        # the content: if the pair itself has changed -- because the plan now
        # says a different buoy is the red one, which is exactly what a
        # Disruptive recolour does -- then repeating the old answer hands the
        # boat the order we just established is wrong. The boat re-asks the same
        # seq after PlanChanged halts the leg, so without this the recolour
        # would not reach the gate being driven.
        #
        # Only pairs that actually changed are dropped. A plain retransmission
        # changes nothing and clears nothing.
        changed = {old_p for old_p, new_p in zip(self._gates, fixed) if old_p != new_p}
        if changed:
            stale = [q for q, ans in self._answered.items() if ans in changed]
            for q in stale:
                del self._answered[q]
            if stale:
                self._say("gate %s: answer withdrawn, the pair changed"
                          % ", ".join(str(q) for q in sorted(stale)))
        self._gates = fixed

    def send_plan(self, buoys, entry, exit_, quiet=False):
        """Transmit the whole passage.

        buoys : [(id, lat, lon, beacon)] -- id must be the INDEX, because that
                is what RXL_SAFE_PASSAGE means by an id and what a gate pair
                names.
        entry, exit_ : (lat, lon)
        """
        if len(buoys) > rxl_codec.MAX_BUOYS:
            raise ValueError("%d buoys; the message carries %d"
                             % (len(buoys), rxl_codec.MAX_BUOYS))
        lat, lon, col = [], [], []
        for _id, la, lo, c in buoys:
            lat.append(rxl_codec.e7(la))
            lon.append(rxl_codec.e7(lo))
            col.append(int(c))
        pad = rxl_codec.MAX_BUOYS - len(buoys)
        with self._lock:
            self.conn.mav.rxl_safe_passage_send(
                int(time.time() * 1e3) & 0xFFFFFFFF,
                rxl_codec.e7(entry[0]), rxl_codec.e7(entry[1]),
                rxl_codec.e7(exit_[0]), rxl_codec.e7(exit_[1]),
                len(buoys), lat + [0] * pad, lon + [0] * pad, col + [0] * pad)
            self._plan = (list(buoys), entry, exit_)
            self._reorder_gates()        # first chance to check, if gates came first
            self._sent += 1
            # A routine retransmission is the link WORKING, not news. Logging
            # every one at 0.2 Hz buries the gate handshake -- the thing the
            # panel exists to show -- under a minute of identical lines. The
            # count is in status() for anyone who wants it.
            if not quiet:
                self._say("transmitted %d buoys (send #%d)"
                          % (len(buoys), self._sent))

    def resend(self):
        """Retransmit the last plan unchanged, the way a real aircraft would.

        The boat does NOT treat this as a re-tasking: rxl_link_node versions the
        plan by CONTENT, so an identical retransmission does not bump the
        version and PlanChanged stays quiet.
        """
        with self._lock:
            p = self._plan
        if p is None:
            return False
        self.send_plan(*p, quiet=True)
        return True

    # -------------------------------------------------------------- inbound

    def poll(self, budget=32):
        """Answer any gate requests waiting on the socket. Returns how many.

        Non-blocking. `budget` caps one call so a flood cannot hold a GUI
        thread; whatever is left is still there next time.
        """
        served = 0
        for _ in range(budget):
            with self._lock:
                msg = self.conn.recv_match(blocking=False)
            if msg is None:
                break
            d = rxl_codec.decode(msg)
            if d is None or d["msg"] != "USV_REACHED_GATE":
                continue
            self._answer(d["gate_seq"])
            served += 1
        return served

    def _answer(self, seq):
        with self._lock:
            if seq in self._answered:
                red, green = self._answered[seq]      # idempotent: see header
                self._say("gate %d re-asked -> same answer (%s, %s)"
                          % (seq, _name(red), _name(green)))
            elif self._served < len(self._gates):
                red, green = self._gates[self._served]
                self._served += 1
                self._answered[seq] = (red, green)
                self._say("gate %d -> red %d, green %d" % (seq, red, green))
            else:
                red = green = rxl_codec.NO_BUOY
                self._answered[seq] = (red, green)
                self._say("gate %d -> PASSAGE COMPLETE (255/255)" % seq)
            self.conn.mav.rxl_next_buoy_set_send(
                int(time.time() * 1e3) & 0xFFFFFFFF, seq, [red, green])

    # --------------------------------------------------------------- status

    def status(self):
        with self._lock:
            return {
                "endpoint": self.endpoint,
                "gates": list(self._gates),
                "served": self._served,
                "sent": self._sent,
                "has_plan": self._plan is not None,
                "log": list(self._log),
            }

    def _say(self, line):
        """Call with the lock held."""
        self._log.append("%s  %s" % (time.strftime("%H:%M:%S"), line))
        if self._on_log:
            self._on_log(line)


def _name(v):
    return "none" if v == rxl_codec.NO_BUOY else str(v)


_BEACONS = {rxl_codec.BEACON_UNKNOWN: "unknown", rxl_codec.BEACON_OFF: "unlit",
            rxl_codec.BEACON_FLASHING_RED: "red",
            rxl_codec.BEACON_FLASHING_GREEN: "green",
            rxl_codec.BEACON_FLASHING_BLUE: "entry",
            rxl_codec.BEACON_STEADY_BLUE: "exit"}


def _beacon(v):
    return "no-such-buoy" if v is None else _BEACONS.get(v, "beacon %s" % v)
