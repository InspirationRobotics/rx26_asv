"""record_settings — what the Record tab saved for AUTO-recording, and the rules for when
a Mission-tab run starts and stops a recording by itself.

Two pure pieces, no ROS, tested off-boat:

  Store         the settings JSON ({auto, frame_hz, camera, topics}), written atomically.
                A missing or corrupt file gives the defaults and a warning, never a crash.
  AutoRecorder  the state machine. gcs_bridge feeds it events and carries out the actions
                it returns; it never touches a recorder itself.

AutoRecorder rules:
  * A recording starts only on the transition INTO RUNNING of a goal the Mission tab sent
    (`mission_started`). WAITING_RUN_START does not start it; a Task 1 / Task 3 panel
    mission never does (those never call `mission_started`).
  * Only if `auto` is on and nothing is recording. A recording the operator started is
    never started over and never stopped by a mission.
  * The recording it starts is OWNED (by session name). It is stopped when that mission
    leaves RUNNING for any reason (IDLE, result, cancel, abort, bt_runner gone), and only
    while the active session is still the owned one.
  * The operator takes it over by stopping it (it is not restarted for that mission) or by
    pressing a manual "record start" while it runs (`manual_start`: it is now theirs and
    runs until they stop it).
  * A Mission-tab start whose RUNNING arrives while the previous mission's auto recording
    is still owned: stop it, then start a fresh one (two actions, in that order).
  * A Mission-tab start never seen leaving IDLE within PENDING_S is forgotten.
"""
import json
import os

RUNNING, WAITING, IDLE = "RUNNING", "WAITING_RUN_START", "IDLE"
FRAME_HZ_RANGE = (0.05, 30.0)
DEFAULTS = {"auto": True, "frame_hz": 1.0, "camera": True, "topics": []}
PENDING_S = 60.0


def validate(raw):
    """(settings, error): `raw` merged over the defaults, or None and why it is refused."""
    if not isinstance(raw, dict):
        return None, "settings must be an object"
    out = dict(DEFAULTS, **{k: raw[k] for k in DEFAULTS if k in raw})
    for key in ("auto", "camera"):
        if not isinstance(out[key], bool):
            return None, f"{key} must be true or false"
    hz = out["frame_hz"]
    if isinstance(hz, bool) or not isinstance(hz, (int, float)) or hz != hz:
        return None, "frame_hz must be a number"
    if not FRAME_HZ_RANGE[0] <= hz <= FRAME_HZ_RANGE[1]:
        return None, f"frame_hz must be {FRAME_HZ_RANGE[0]:g}-{FRAME_HZ_RANGE[1]:g}"
    out["frame_hz"] = float(hz)
    topics = out["topics"]
    if not isinstance(topics, list) or not all(isinstance(t, str) and t for t in topics):
        return None, "topics must be a list of topic names"
    out["topics"] = list(dict.fromkeys(topics))
    return out, ""


class Store:
    """The settings file. `warning` says why the defaults are in use, "" when they are not."""

    def __init__(self, path):
        self.path = os.path.expanduser(path)
        self.warning = ""
        self.settings = dict(DEFAULTS, topics=[])
        try:
            with open(self.path, encoding="utf-8") as f:
                raw = json.load(f)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as e:
            self.warning = f"{self.path} unreadable ({e}): auto-record defaults in use"
            return
        settings, err = validate(raw)
        if settings is None:
            self.warning = f"{self.path} invalid ({err}): auto-record defaults in use"
        else:
            self.settings = settings

    def get(self):
        return dict(self.settings, topics=list(self.settings["topics"]))

    def set(self, raw):
        """Validate, save atomically, keep. Returns (settings, error).

        Raises:
          OSError: the file could not be written (nothing is kept).
        """
        settings, err = validate(raw)
        if settings is None:
            return None, err
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(settings, f, indent=1)
        os.replace(tmp, self.path)
        self.settings, self.warning = settings, ""
        return self.get(), ""


class AutoRecorder:
    """Feed it events; carry out the ("start", mission) / ("stop", mission) it returns.

    After a "start" the caller reports `started(session, mission)` or `start_failed()`.
    """

    def __init__(self):
        self.pending = None         # mission name the Mission tab sent, not yet recorded
        self._pending_t = 0.0
        self._seen = False          # bt_status has shown the pending goal since it was sent
        self.owned = None           # session name this class started
        self.mission = None         # ... and for which mission

    @property
    def active(self):
        """The mission whose auto recording is running, else None (the status frame's `auto`)."""
        return self.mission if self.owned else None

    def mission_started(self, name, now):
        self.pending, self._pending_t, self._seen = name, now, False

    def manual_start(self):
        """The operator pressed "record start": whatever runs is theirs now."""
        self.owned = self.mission = None

    def started(self, session, mission):
        self.owned, self.mission = session, mission

    def start_failed(self):
        self.pending = None         # one refusal per mission: no retry every tick

    def update(self, now, state, name, recording, auto_on):
        """One bt_status look. `recording` is the active session's name or None.

        Returns a list of ("start" | "stop", mission) in the order to carry them out.
        """
        actions = []
        running = state == RUNNING
        if self.owned and recording != self.owned:
            self.owned = self.mission = None        # the operator stopped it (or replaced it)
        elif self.owned and not (running and name == self.mission):
            actions.append(("stop", self.mission))
            self.owned = self.mission = None
            recording = None                        # the caller stops it before any start
        if self.pending is not None:
            if name == self.pending and state in (RUNNING, WAITING):
                self._seen = True
            if running and name == self.pending:
                if auto_on and recording is None:
                    actions.append(("start", self.pending))
                self.pending = None
            elif (self._seen and state == IDLE) or now - self._pending_t > PENDING_S and not self._seen:
                self.pending = None
        return actions
