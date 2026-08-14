"""StreamCache — a last-known value that knows how old it is.

The bug this exists to prevent: telemetry_bridge cached the last frame from
each MAVLink stream and republished it at 20 Hz with `header.stamp = now`,
with no check that MAVProxy was still delivering. When the gateway died the
last frame was rebroadcast forever, timestamped as if it had just arrived.

That is worse than publishing nothing, because it defeats the consumers built
to notice silence. rc_heartbeat_watchdog judges the RC link by the PWM on
/crsd/rc_channels and the gateway by the arrival of ANY bridge topic; both
timers were held fresh by the very staleness they exist to detect, so its
force-disarm and its `gateway_down_in_loss` branch became unreachable. And
because the header stamp lied, nothing downstream could tell either:
frame_transform kept rotating body detections against a frozen pose, writing
obstacles into the world grid at coordinates the boat had long since left.

So: a cached value is only ever handed out with an explicit freshness check,
and going stale is a loud, one-shot event rather than silence. `now` is a
caller-supplied monotonic seconds value so tests drive time deterministically.

No ROS imports; unit-tested.
"""


class StreamCache:
    """Holds the newest value seen on one stream plus its receipt time.

    Deliberately has no "just give me the value" accessor: every read passes a
    timestamp and gets None when the data is too old. Making the stale case
    unrepresentable is the point — the original defect was a plain attribute
    that read the same whether it was 20 ms or 20 minutes old.
    """

    __slots__ = ("timeout_s", "value", "stamp", "recv_t", "_was_stale")

    def __init__(self, timeout_s: float):
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = timeout_s
        self.value = None
        self.stamp = None        # transport-side timestamp captured AT RECEIPT
        self.recv_t = None       # monotonic seconds at receipt
        self._was_stale = False

    # ---- write ----

    def set(self, value, now: float, stamp=None):
        """Record a value received at monotonic time `now`.

        `stamp` is whatever the caller wants to republish as the message's own
        timestamp — captured here, at receipt, rather than at publish time, so
        a republished value carries the age it actually has.
        """
        self.value = value
        self.stamp = stamp
        self.recv_t = now
        self._was_stale = False

    # ---- read ----

    def get(self, now: float):
        """The value if it is still fresh, else None. The only accessor."""
        return self.value if self.fresh(now) else None

    def fresh(self, now: float) -> bool:
        return self.recv_t is not None and (now - self.recv_t) < self.timeout_s

    def age(self, now: float):
        """Seconds since receipt, or None if nothing has ever arrived."""
        return None if self.recv_t is None else now - self.recv_t

    def went_stale(self, now: float) -> bool:
        """One-shot rising edge of staleness, for logging.

        True exactly once per fresh -> stale transition, so a dead stream
        produces one loud line rather than a throttled stream of them or, worse,
        nothing at all. Never fires for a stream that has not yet had a first
        value: startup silence is not a dropout (the same distinction
        rc_heartbeat_core draws by seeding last_bridge_msg "down").
        """
        if self.recv_t is None or self.fresh(now):
            return False
        if self._was_stale:
            return False
        self._was_stale = True
        return True
