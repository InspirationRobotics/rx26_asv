"""light_core — a buoy's beacon state from every look the boat has had at it.

No ROS, no clock: every sample carries its own time, so a test can drive a 1 s ON / 1 s OFF
beacon in microseconds.

WHY THE LIGHT HISTORY LIVES ON THE WORLD TRACK (Ekko's way: rx26_uav
uav_perception/buoy_tracker.py). Flashing is a property of a buoy over seconds. camera_node
used to decide it on its IMAGE track (IoU from frame to frame): every turn, every frame the
buoy left, every missed box restarted that track, its six-second window and its four-second
wait, and red and green publish no colour at all until they are seen flashing. Ekko keeps the
samples on the buoy's GROUND track instead, so looking away and coming back adds to what was
already seen. target_tracker's earth-anchored tracks are that ground track, so camera_node now
sends each frame's reading (reading label, below) and the state is decided here.

WHAT DECIDES, kept from camera_node's FlashTracker, which was right about it:

  1. Does it move like a LIGHT? Anti-correlation at half the flash period (handbook 3.3.2:
     on 1 s, off 1 s). Scale-free, so a faint green flash passes as easily as a bright red
     one, and a classifier hedging at random scores near zero. Ekko asks "lit share in the
     middle and at least two changes", which hedging noise passes at 10 Hz.
  2. Not periodic: the mean lit score decides SOLID or OFF.
  3. In the middle and not periodic: undecided. The dead bands are deliberate: mistaking
     the solid-blue EXIT for the flashing-blue ENTRY sends the boat to the wrong end.

WHAT IS EKKO'S: samples pooled over every look inside a window (`window_s`), watch time
counted only across gaps of at most MAX_GAP_S (time spent looking elsewhere is not watching),
and the periodicity pooled over each look's own pairs, never across a gap. A decision is
STICKY: a look that settles nothing leaves the last answer standing, and changing it takes
SWITCH_EVALS agreeing evaluations in a row, so one noisy evaluation cannot flip ENTRY and
EXIT. Ekko locks outright; here a sustained new answer still wins, which Disruptive needs.
"""
import math
import re
from collections import deque

# ---------------------------------------------------------------- the reading label
# camera_node -> target_tracker, one per classified box per frame (buoy_model, when
# light_raw_labels is on): lit<NN>_<colour>_<shape>, NN = round(100 * (1 - P(off))), colour =
# the most likely LIT class whatever NN is. A box too small to classify comes as a bare
# <shape>. oak_detector_core.reading_label writes it; change both or neither.
READING = re.compile(r"^lit(\d{1,3})_([a-z]+)_([a-z0-9]+)$")


def parse_reading(label):
    """-> (lit_score 0..1, colour, shape) for a reading label, None for any other label."""
    m = READING.match(label or "")
    if m is None:
        return None
    return min(1.0, int(m.group(1)) / 100.0), m.group(2), m.group(3)


# ---------------------------------------------------------------- shaping, not knobs
PERIOD_S = 2.0          # handbook 3.3.2: on 1 s, off 1 s
SOLID_MIN = 0.70        # FlashTracker's dead bands, unchanged
OFF_MAX = 0.30
LIT_ABOVE = 0.5         # a sample whose colour counts toward the colour vote
MAX_GAP_S = 1.0         # Ekko's max_sample_gap_s: a longer gap ends a look
MIN_SAMPLES = 12        # Ekko's min_samples
MIN_PAIRED_S = 3.0      # lag pairs the periodicity needs (FlashTracker: 1.5 periods of span)
GRID_HZ = 10.0          # each look is resampled onto this grid before correlating
SWITCH_EVALS = 10       # agreeing evaluations in a row to set or change a decision
MAX_SAMPLES = 3000      # 5 min at 10 Hz; the window is what decides

FLASHING, SOLID, OFF = "flashing", "solid", "off"


def looks(samples, max_gap=MAX_GAP_S):
    """Time-ordered samples split wherever the gap exceeds max_gap: one list per look."""
    out = []
    for s in samples:
        if out and s[0] - out[-1][-1][0] <= max_gap:
            out[-1].append(s)
        else:
            out.append([s])
    return out


def _resample(look, n):
    """The look's lit scores on a GRID_HZ grid from its first sample, n points."""
    t0 = look[0][0]
    vals, j = [], 0
    for i in range(n):
        t = t0 + i / GRID_HZ
        while j + 1 < len(look) and look[j + 1][0] <= t:
            j += 1
        if j + 1 < len(look):
            (ta, a), (tb, b) = look[j][:2], look[j + 1][:2]
            vals.append(a + (b - a) * (t - ta) / (tb - ta) if tb > ta else a)
        else:
            vals.append(look[j][1])
    return vals


def periodicity(samples, period_s=PERIOD_S):
    """-> anti-correlation of the lit score at half the flash period, 0..1, pooled over looks.

    Each look is resampled on its own (interpolating across a gap would invent the samples
    a flash is judged on); the lag pairs of every look are then pooled around one mean, so
    three two-second looks count like one six-second look. 0.0 with under MIN_PAIRED_S of
    pairs: no verdict rather than a guess.
    """
    k = int(round(period_s / 2.0 * GRID_HZ))
    grids = []
    for look in looks(samples):
        n = int((look[-1][0] - look[0][0]) * GRID_HZ) + 1
        if n > k:
            grids.append(_resample(look, n))
    pairs = sum(len(g) - k for g in grids)
    if pairs < MIN_PAIRED_S * GRID_HZ:
        return 0.0
    every = [v for g in grids for v in g]
    mean = sum(every) / len(every)
    sab = saa = sbb = 0.0
    for g in grids:
        for i in range(len(g) - k):
            a, b = g[i] - mean, g[i + k] - mean
            sab += a * b
            saa += a * a
            sbb += b * b
    if saa * sbb < 1e-12:                   # flat: no swing at all
        return 0.0
    return max(0.0, -sab / math.sqrt(saa * sbb))


class LightConfig:
    """What a track's light is judged with. target_tracker fills it from TrackerParams."""

    def __init__(self, window_s=30.0, min_watch_s=4.0, corr_min=0.45,
                 min_colour_votes=5, min_colour_ratio=0.6, must_flash=("red", "green")):
        self.window_s = float(window_s)
        self.min_watch_s = float(min_watch_s)
        self.corr_min = float(corr_min)
        self.min_colour_votes = int(min_colour_votes)
        self.min_colour_ratio = float(min_colour_ratio)
        self.must_flash = tuple(must_flash)


def evidence(samples, cfg):
    """The numbers a decision is made from, over the last cfg.window_s of samples."""
    if not samples:
        return {"samples": 0, "watch_s": 0.0, "duty": None, "period": 0.0,
                "colour": None, "colour_share": 0.0}
    cut = samples[-1][0] - cfg.window_s
    win = [s for s in samples if s[0] >= cut]
    watch = sum(b[0] - a[0] for a, b in zip(win, win[1:]) if b[0] - a[0] <= MAX_GAP_S)
    counts = {}
    for _t, score, colour in win:
        if score > LIT_ABOVE and colour:
            counts[colour] = counts.get(colour, 0) + 1
    colour, share = None, 0.0
    if counts:
        ranked = sorted(counts.items(), key=lambda kv: kv[1], reverse=True)
        top, n_top = ranked[0]
        share = n_top / float(sum(counts.values()))
        tie = len(ranked) > 1 and ranked[1][1] == n_top
        if not tie and n_top >= cfg.min_colour_votes and share >= cfg.min_colour_ratio:
            colour = top
    return {"samples": len(win), "watch_s": watch,
            "duty": sum(s[1] for s in win) / len(win),
            "period": periodicity(win), "colour": colour, "colour_share": share}


def judge(ev, cfg):
    """-> (state, colour) from evidence, or None when it settles nothing. See the docstring."""
    if ev["samples"] < MIN_SAMPLES or ev["watch_s"] < cfg.min_watch_s:
        return None
    if ev["period"] >= cfg.corr_min:
        state = FLASHING
    elif ev["duty"] >= SOLID_MIN:
        state = SOLID
    elif ev["duty"] <= OFF_MAX:
        return OFF, None
    else:
        return None
    if ev["colour"] is None:
        return None                         # a light of no settled colour: keep watching
    return state, ev["colour"]


class LightHistory:
    """One world track's light readings and its sticky decision."""

    def __init__(self):
        self.samples = deque(maxlen=MAX_SAMPLES)    # (t, lit score, colour)
        self.decided = None                         # (state, colour) or None
        self.ev = evidence([], LightConfig())
        self._candidate, self._streak = None, 0
        self._dirty = False

    def add(self, t, score, colour):
        if self.samples and t < self.samples[-1][0]:
            return                                  # out of order: never rewrite history
        self.samples.append((float(t), float(score), colour))
        self._dirty = True

    def clear(self):
        self.__init__()

    def update(self, cfg):
        """Re-judge if a sample arrived since the last call; returns the sticky decision."""
        if not self._dirty:
            return self.decided
        self._dirty = False
        self.ev = evidence(self.samples, cfg)
        verdict = judge(self.ev, cfg)
        if verdict is None or verdict == self.decided:
            self._candidate, self._streak = None, 0
            return self.decided
        if verdict == self._candidate:
            self._streak += 1
        else:
            self._candidate, self._streak = verdict, 1
        if self._streak >= SWITCH_EVALS:
            self.decided = verdict
            self._candidate, self._streak = None, 0
        return self.decided


def light_label(decided, shape, must_flash=("red", "green")):
    """The published name, in buoy_model's vocabulary (bt_runner's beaconFromLabel reads it):

        flashing   flash_red_diamond
        solid      blue_diamond (red and green: unknown, Task 1 never shows them solid)
        off        off_diamond
        undecided  unknown_diamond
    """
    if decided is None:
        return "unknown_" + shape
    state, colour = decided
    if state == OFF:
        return "off_" + shape
    if state == FLASHING:
        return "flash_%s_%s" % (colour, shape)
    if colour in must_flash:
        return "unknown_" + shape
    return "%s_%s" % (colour, shape)
