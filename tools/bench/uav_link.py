#!/usr/bin/env python3
"""uav_link — the aircraft's half of the Task 1 radio, with no ROS in it.

Shared by the two things that stand in for Ekko:

    bench_uav.py                    a scripted run, no browser
    bench_world_model.py --uav      the same radio, driven from the GUI

Extracted rather than copied: two copies of a handshake drift, and only one
gets the fix.

THE HANDSHAKE:

    here  -> RXL_SAFE_PASSAGE               ten buoys, positions and colours
    boat  -> RXL_USV_REACHED_GATE(seq)      "past a checkpoint -- still current?"
    here  -> RXL_SAFE_PASSAGE               the field again, as it is NOW
    here  -> RXL_NEXT_BUOY_SET(seq, -, -)   "confirmed, as of your gate seq"

    SEQ COUNTS CHECKPOINTS, NOT GATES. The boat asks once after circling the
    ENTRY buoy and once after each gate, so seq 1 is the entry and seq 2 is
    gate 1. The wire carries only a number, so these log lines say
    "checkpoint" and leave naming it to the boat. The message id still says
    GATE because renaming an on-air id breaks whichever vehicle is not updated.

    THE AIRCRAFT DOES NOT ASSIGN GATES. The boat pairs the buoys and picks the
    order itself (nav::planPassage). RXL_NEXT_BUOY_SET is only the
    acknowledgement; its two buoy_id fields are sent as NO_BUOY and the boat
    ignores them (and logs it if an older aircraft fills them in).

    A REPEATED seq MUST GET THE SAME ANSWER, immediately: on a lossy radio the
    boat re-asks when our reply is lost. `_answered` makes that work.

    CONFIRMING IS AN OPERATOR ACTION, because the point of the confirmation is
    that a human may be about to change the field. poll() records the request
    and confirm() answers it; scripts pass auto_confirm=True.

This module never decides where the boat goes; the passage layout comes from
the caller.
"""
import collections
import threading
import time

from crusader_common import geo
from crusader_link import rxl_codec


def offsets_to_latlon(east_m, north_m, anchor):
    """(east, north) metres from `anchor` -> (lat, lon).

    Through crusader_common.geo, which owns M_PER_DEG: a bench that places
    buoys with a different earth radius from the boat that reads them puts
    them somewhere else.
    """
    return geo.xy_to_latlon(east_m, north_m, anchor)


class UavLink:
    """One radio, one gate list, one log. Thread-safe; poll() from anywhere."""

    def __init__(self, endpoint="udpout:127.0.0.1:14555", source_system=200,
                 on_log=None, auto_confirm=False):
        # source_system must differ from the boat's (42) or an inspector files
        # both vehicles' traffic under one tree.
        self.conn = rxl_codec.connect(endpoint, source_system=source_system)
        self.endpoint = endpoint
        self._lock = threading.Lock()
        self._answered = set()           # seqs already confirmed, for the re-ask
        self._pending = None             # seq the boat is waiting on, or None
        self._confirmed = 0
        self._auto_confirm = auto_confirm
        self._log = collections.deque(maxlen=40)
        self._plan = None                # last (buoys, entry, exit) transmitted
        self._sent = 0
        # The deque is for the GUI, which re-reads the tail each poll. A bounded
        # deque cannot be followed by index once it wraps, so a caller that
        # wants every line passes a callback instead.
        self._on_log = on_log

    # ------------------------------------------------------------- outbound

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
            self._sent += 1
            # Routine retransmissions are not logged: at 0.2 Hz they bury the
            # gate handshake the panel exists to show. The count is in status().
            if not quiet:
                self._say("transmitted %d buoys (send #%d)"
                          % (len(buoys), self._sent))

    def resend(self):
        """Retransmit the last plan unchanged, the way a real aircraft would.

        Not a re-tasking: rxl_link_node versions the plan by CONTENT, so
        PlanChanged stays quiet.
        """
        with self._lock:
            p = self._plan
        if p is None:
            return False
        self.send_plan(*p, quiet=True)
        return True

    # -------------------------------------------------------------- inbound

    def poll(self, budget=32):
        """Take any confirmation requests off the socket. Returns how many.

        Non-blocking. `budget` caps one call so a flood cannot hold a GUI
        thread; whatever is left is still there next time.
        """
        got = 0
        for _ in range(budget):
            with self._lock:
                msg = self.conn.recv_match(blocking=False)
            if msg is None:
                break
            d = rxl_codec.decode(msg)
            if d is None or d["msg"] != "USV_REACHED_GATE":
                continue
            got += 1
            self._request(d["gate_seq"])
        return got

    def _request(self, seq):
        """Handle one confirmation request.

        Split out of poll() so a bench can drive the protocol with no socket
        peer.

        A seq already confirmed is answered now: that is the lost-reply case,
        and waiting on the operator again would turn every dropped packet into
        a stall at a gate. A new seq waits for confirm().
        """
        with self._lock:
            if seq in self._answered:
                self._ack_locked(seq)
                self._say("checkpoint %d re-asked -> confirmed again" % seq)
                return
            if self._pending != seq:
                self._pending = seq
                self._say("checkpoint %d: the boat is asking for confirmation" % seq)
            auto = self._auto_confirm
        if auto:
            self.confirm()

    def confirm(self, buoys=None, entry=None, exit_=None):
        """Answer the outstanding request: the field as it is NOW, then the ack.

        THE FIELD GOES FIRST. The boat treats the ack as permission to drive on,
        so an ack that overtook the new positions would let it leave on the old
        ones.

        Pass buoys/entry/exit_ to change the field in the same breath ("recolour
        a buoy, then confirm"); omit them to repeat the last field unchanged.
        """
        with self._lock:
            seq = self._pending
            if seq is None:
                return "nothing is waiting on a confirmation"
            plan = self._plan

        if buoys is not None:
            self.send_plan(buoys, entry, exit_, quiet=True)
        elif plan is not None:
            self.send_plan(*plan, quiet=True)

        with self._lock:
            if self._pending != seq:            # a fresh request overtook us
                return "checkpoint %d was superseded" % seq
            self._ack_locked(seq)
            self._answered.add(seq)
            self._pending = None
            self._confirmed += 1
            self._say("checkpoint %d confirmed, field sent (%d buoys)"
                      % (seq, len(self._plan[0]) if self._plan else 0))
        return "checkpoint %d confirmed" % seq

    def rewind(self):
        """Forget which sequence numbers have been confirmed.

        For a fresh run against the same radio. Without it the next run's seq 1
        is answered instantly as a lost-reply repeat, and the operator never
        gets to change the field before the boat drives on.
        """
        with self._lock:
            self._answered.clear()
            self._pending = None
            self._confirmed = 0
            self._say("rewound: no checkpoint is confirmed")

    def _ack_locked(self, seq):
        """Put RXL_NEXT_BUOY_SET on the wire. Call with the lock held.

        The buoy_id fields are NO_BUOY on purpose; see the header.
        """
        self.conn.mav.rxl_next_buoy_set_send(
            int(time.time() * 1e3) & 0xFFFFFFFF, seq,
            [rxl_codec.NO_BUOY, rxl_codec.NO_BUOY])

    # --------------------------------------------------------------- status

    def status(self):
        with self._lock:
            return {
                "endpoint": self.endpoint,
                "pending": self._pending,
                "confirmed": self._confirmed,
                "auto_confirm": self._auto_confirm,
                "sent": self._sent,
                "has_plan": self._plan is not None,
                "log": list(self._log),
            }

    def _say(self, line):
        """Call with the lock held."""
        self._log.append("%s  %s" % (time.strftime("%H:%M:%S"), line))
        if self._on_log:
            self._on_log(line)
