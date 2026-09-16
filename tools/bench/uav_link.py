#!/usr/bin/env python3
"""uav_link — the aircraft's half of the Task 1 radio, with no ROS in it.

Shared by the two things that stand in for Ekko:

    bench_uav.py                    a scripted run, no browser
    bench_world_model.py --uav      the same radio, driven from the GUI

EXTRACTED RATHER THAN COPIED, for the reason mjpeg_server.py was: two copies of
a handshake drift, and only one of them gets the fix. The gate protocol has
exactly one subtle rule in it (below) and it must not exist twice.

THE HANDSHAKE, and the one rule that is easy to get wrong:

    here  -> RXL_SAFE_PASSAGE               ten buoys, positions and colours
    boat  -> RXL_USV_REACHED_GATE(seq)      "past a checkpoint -- still current?"
    here  -> RXL_SAFE_PASSAGE               the field again, as it is NOW
    here  -> RXL_NEXT_BUOY_SET(seq, -, -)   "confirmed, as of your gate seq"

    SEQ COUNTS CHECKPOINTS, NOT GATES. The boat asks once after circling the
    ENTRY buoy and once after each gate, so seq 1 is the entry and seq 2 is
    gate 1. Nothing here can tell them apart -- the wire carries only a number --
    which is why these log lines say "checkpoint" and leave naming it to the
    boat, which knows. The message id still says GATE because it is on the air
    between two vehicles and renaming it would break the one that was not
    updated.

    THE AIRCRAFT NO LONGER ASSIGNS GATES. It transmits ten buoys with their
    colours; the boat pairs them and picks the order itself (nav::planPassage).
    RXL_NEXT_BUOY_SET survives only as the acknowledgement -- its two buoy_id
    fields are vestigial and are sent as NO_BUOY. The boat ignores them, and
    says so in a log line if an older aircraft fills them in.

    A REPEATED seq MUST GET THE SAME ANSWER. On a lossy radio the boat re-asks
    when our reply is lost, and the re-ask must be answered immediately rather
    than waiting on the operator again. `_answered` is what makes that work.

    CONFIRMING IS AN OPERATOR ACTION, not an automatic one, because the whole
    point of the confirmation is that a human may be about to change the field.
    poll() records the request and confirm() answers it. Scripts that want the
    old automatic behaviour pass auto_confirm=True.

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
        # The deque is for the GUI, which re-reads the whole tail each poll. A
        # script cannot follow a BOUNDED deque by index -- once it wraps, the
        # index means a different line -- so a caller that wants every line as
        # it happens passes a callback instead.
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
        """Take any confirmation requests off the socket. Returns how many.

        Non-blocking. `budget` caps one call so a flood cannot hold a GUI
        thread; whatever is left is still there next time.

        A request for a seq already confirmed is re-answered HERE and now: that
        is the lost-reply case and it must not wait on the operator a second
        time. A new seq is recorded as pending and waits for confirm().
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
        peer, and so the re-ask rule below exists once rather than twice.

        A seq already confirmed is answered HERE and now: that is the lost-reply
        case, and making it wait on the operator again would turn every dropped
        packet into a stall at a gate. A new seq waits for confirm().
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

        THE FIELD GOES FIRST, and the order is the whole point. The boat treats
        the acknowledgement as permission to drive on, so an ack that overtook
        the new positions would let it leave on the old ones. Sending the plan
        first means that by the time the ack lands the boat has already re-planned
        against the new field.

        Pass buoys/entry/exit_ to change the field in the same breath -- that is
        what "recolour a buoy, then confirm" does from the page. Omit them and
        the last transmitted field is repeated unchanged.
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

        For starting a fresh run against the same radio. Without it the next
        run's first request -- seq 1 again -- is answered instantly from the
        cache as a lost-reply repeat, and the operator never gets the chance to
        change the field before the boat drives on.
        """
        with self._lock:
            self._answered.clear()
            self._pending = None
            self._confirmed = 0
            self._say("rewound: no checkpoint is confirmed")

    def _ack_locked(self, seq):
        """Put RXL_NEXT_BUOY_SET on the wire. Call with the lock held.

        The two buoy_id fields are NO_BUOY on purpose: the boat plans its own
        gate order, so there is nothing to assign. See the header.
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
