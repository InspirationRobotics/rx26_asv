"""mission_api — the /mission/* endpoints, as one implementation both servers call.

No ROS imports: gcs_node (the Jetson / sim) and tools/scripts/gcs_laptop.py (the laptop)
both build one of these, and the tests drive it with fakes. Two halves:

  MissionLib  the five library endpoints (catalogue, list, load, validate, save), a thin
              wrapper over mission_lib with the payload checks the HTTP layer owes it.
              It reads and writes only the two directories it is given.
  MissionRun  start / stop / status, for the one process that can reach the BT runner.
              The ROS pieces (set the runner's tree_file, send the SafePassage goal,
              cancel it) come in as a "port"; this class owns the rules around them.

THE RULES MissionRun HOLDS, and holds again on every request whatever the page showed:

  * One mission at a time. A start while one is starting, running or being cancelled is
    refused, never queued.
  * The XML is validated here before anything is written or sent. A page that skipped
    its own validation cannot start a broken tree.
  * The tree file is written, the runner is told about it, and only then is the goal
    sent, because the runner reads tree_file when it accepts a goal.
  * A refusal keeps its reason and surfaces in /mission/status, so a start that never
    reached the boat is told apart from one that ran and failed.
  * HTTP never waits long on ROS: the launch runs in a worker thread and the request
    waits for it a few seconds at most, then says the boat has not answered yet.
"""
import math
import os
import re
import threading
import time

LIB_PATHS = ("/mission/catalogue", "/mission/list", "/mission/load",
             "/mission/validate", "/mission/save")
RUN_PATHS = ("/mission/start", "/mission/stop", "/mission/status")

#: SafePassage.Goal.tier values, by the names the page offers.
TIERS = {"core": 0, "advanced": 1, "disruptive": 2}
#: SafePassage.Result.outcome values.
OUTCOMES = {0: "SUCCESS", 1: "TIMEOUT", 2: "NO_ENTRY", 3: "CANCELLED",
            4: "NOT_AUTONOMOUS", 5: "FAULT"}

MAX_XML = 1 << 20
MAX_TIMEOUT_S = 86400.0
ACTIVE = ("starting", "running", "cancelling")


def _bad(message, **extra):
    return dict(extra, ok=False, message=message)


def _text(payload, key, limit=MAX_XML):
    value = payload.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"'{key}' is missing or not text")
    if len(value) > limit:
        raise ValueError(f"'{key}' is too long")
    return value


_DEVICE = re.compile(r"(?i)con|prn|aux|nul|com[1-9]|lpt[1-9]")


def safe_name(name):
    """A file stem from a mission name: nothing that can leave the run directory, and not a
    Windows device name (CON.xml is the console there)."""
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(name or "")).strip("._")[:60]
    return "mission" if not stem or _DEVICE.fullmatch(stem) else stem


class MissionLib:
    """The library endpoints over one bt_dir (subtrees, pre-built missions) and one
    missions_dir (the operator's saved ones). `lib` is mission_lib, imported on first use so a
    missing library fails the mission endpoints and nothing else."""

    def __init__(self, bt_dir, missions_dir, lib=None):
        self.bt_dir = bt_dir
        self.missions_dir = missions_dir
        self._lib = lib
        self._cat = None
        self._lock = threading.Lock()

    def lib(self):
        if self._lib is None:
            from crusader_groundstation import mission_lib
            self._lib = mission_lib
        return self._lib

    def catalogue(self):
        """Read afresh each time the page asks (a subtree edited on disk shows up on the next
        tab visit); validate and save reuse the last one rather than re-parsing every call."""
        cat = self.lib().catalogue(self.bt_dir)
        with self._lock:
            self._cat = cat
        return cat

    def _cached(self):
        with self._lock:
            cat = self._cat
        return cat if cat is not None else self.catalogue()

    def handle(self, path, payload):
        """One library endpoint -> a dict with at least {ok, message}. Never raises."""
        try:
            return self._handle(path, payload)
        except ValueError as e:
            return _bad(str(e))
        except Exception as e:              # noqa: BLE001 - one bad mission file must not 500 the page
            return _bad(f"mission library failed on {path}: {type(e).__name__}: {e}")

    def _handle(self, path, payload):
        lib = self.lib()
        if path == "/mission/catalogue":
            return dict(self.catalogue(), ok=True, message="")
        if path == "/mission/list":
            return {"ok": True, "message": "",
                    "missions": lib.list_missions(self.bt_dir, self.missions_dir)}
        if path == "/mission/load":
            name, source = _text(payload, "name", 200), payload.get("source")
            if source not in ("prebuilt", "saved"):
                raise ValueError("'source' must be 'prebuilt' or 'saved'")
            return dict(lib.load_mission(self.bt_dir, self.missions_dir, name, source))
        if path == "/mission/validate":
            res = dict(lib.validate(_text(payload, "xml"), self._cached()))
            errors = res.get("errors") or []
            res.setdefault("message", "" if res.get("ok") else
                           f"{len(errors)} error(s): {errors[0] if errors else '?'}")
            return res
        if path == "/mission/save":
            name, xml = _text(payload, "name", 200), _text(payload, "xml")
            return dict(lib.save_mission(self.missions_dir, name, xml, self._cached(),
                                         overwrite=bool(payload.get("overwrite"))))
        return _bad(f"unknown mission action {path}")


class MissionRun:
    """Start, stop and report one mission through a goal port.

    port.send(tier, timeout_s, on_feedback, on_result) -> (ok, reason): sends the goal and
        returns once the runner accepted or refused it; on_feedback(dict) and on_result(dict)
        arrive later from another thread. port.cancel() -> (ok, reason).
    set_tree_file(path) -> (ok, reason): points the runner at the file it should load.
    """

    def __init__(self, lib, run_dir, set_tree_file, port, logger=None,
                 clock=time.monotonic, wait_s=3.0):
        self.lib, self.run_dir = lib, run_dir
        self._set_tree_file, self._port = set_tree_file, port
        self._log = logger or (lambda _m: None)
        self._clock, self._wait_s = clock, wait_s
        self._lock = threading.Lock()
        self._s = {"state": "idle", "name": "", "tier": "", "t0": None, "file": "",
                   "feedback": None, "fb_t": None, "result": None}

    # ---- the requests ----

    def handle(self, path, payload):
        try:
            if path == "/mission/start":
                return self.start(payload)
            if path == "/mission/stop":
                return self.stop()
            if path == "/mission/status":
                return self.status()
        except ValueError as e:
            return _bad(str(e))
        except Exception as e:              # noqa: BLE001
            return _bad(f"{path} failed: {type(e).__name__}: {e}")
        return _bad(f"unknown mission action {path}")

    def start(self, payload):
        xml = _text(payload, "xml")
        tier = payload.get("tier")
        if tier not in TIERS:
            raise ValueError(f"'tier' must be one of {', '.join(TIERS)}")
        try:
            timeout_s = float(payload.get("timeout_s"))
        except (TypeError, ValueError):
            raise ValueError("'timeout_s' must be a number") from None
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= MAX_TIMEOUT_S:
            raise ValueError(f"'timeout_s' must be between 0 and {MAX_TIMEOUT_S:.0f}")
        name = str(payload.get("name") or "mission")[:120]

        with self._lock:                    # claimed before validating: two clicks, one start
            if self._s["state"] in ACTIVE:
                return _bad(f"a mission is already {self._s['state']} ({self._s['name']}): "
                            "stop it first")
            self._s.update(state="starting", name=name, tier=tier, t0=self._clock(), file="",
                           feedback=None, fb_t=None, result=None)
        check = self.lib.handle("/mission/validate", {"xml": xml})
        if not check.get("ok"):
            errors = check.get("errors") or []
            return self._refused("; ".join(errors[:3]) if errors else check.get("message", "invalid"),
                                 "the mission does not validate", errors=errors,
                                 warnings=check.get("warnings") or [])
        try:
            path = self._write(name, xml)
        except OSError as e:
            return self._refused(str(e), "cannot write the mission file")
        with self._lock:
            self._s["file"] = path
        done = threading.Event()
        box = {}

        def launch():
            try:
                box["r"] = self._launch(path, TIERS[tier], timeout_s)
            except Exception as e:          # noqa: BLE001
                box["r"] = self._refused(f"{type(e).__name__}: {e}", "the launch failed")
            done.set()
        threading.Thread(target=launch, daemon=True).start()
        if done.wait(self._wait_s):
            return dict(box["r"], warnings=check.get("warnings") or [])
        return {"ok": True, "message": "starting: the boat has not answered yet; the status line "
                                       "says when it has", "warnings": check.get("warnings") or []}

    def _write(self, name, xml):
        os.makedirs(self.run_dir, exist_ok=True)
        path = os.path.join(self.run_dir, safe_name(name) + ".xml")
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8", newline="\n") as f:
            f.write(xml)
        os.replace(tmp, path)               # the runner never reads half a file
        return path

    def _launch(self, path, tier_code, timeout_s):
        ok, why = self._set_tree_file(path)
        if not ok:
            return self._refused(why, "the BT runner would not take the mission file")
        ok, why = self._port.send(tier_code, timeout_s, self._on_feedback, self._on_result)
        if not ok:
            return self._refused(why, "the BT runner did not accept the goal")
        with self._lock:
            if self._s["state"] == "starting":      # a very fast result has already moved it on
                self._s["state"] = "running"
            name = self._s["name"]
        self._log(f"mission '{name}' started from {path}")
        return {"ok": True, "message": f"started {name}"}

    def _refused(self, why, headline, **extra):
        with self._lock:
            self._s.update(state="idle", result={"outcome": "NOT STARTED", "code": None,
                                                 "detail": f"{headline}: {why}", "elapsed_s": None})
        self._log(f"mission start refused: {headline}: {why}")
        return _bad(f"{headline}: {why}", **extra)

    def stop(self):
        with self._lock:
            if self._s["state"] not in ACTIVE:
                return _bad("no mission is running")
            self._s["state"] = "cancelling"
        ok, why = self._port.cancel()
        if not ok:
            with self._lock:
                if self._s["state"] == "cancelling":
                    self._s["state"] = "running"
            return _bad(f"the cancel did not go through: {why}")
        return {"ok": True, "message": "cancel requested"}

    # ---- from the runner ----

    def _on_feedback(self, fb):
        with self._lock:
            self._s["feedback"], self._s["fb_t"] = dict(fb), self._clock()

    def _on_result(self, res):
        with self._lock:
            t0 = self._s["t0"]
            res = dict(res)
            if res.get("code") in OUTCOMES:
                res.setdefault("outcome", OUTCOMES[res["code"]])
            res.setdefault("elapsed_s", None if t0 is None else round(self._clock() - t0, 1))
            self._s.update(state="idle", result=res)
        self._log(f"mission finished: {res.get('outcome')} {res.get('detail', '')}")

    def status(self):
        with self._lock:
            s = dict(self._s)
            now = self._clock()
        fb = None
        if s["feedback"] is not None and s["state"] in ACTIVE:
            fb = dict(s["feedback"], age_s=round(now - s["fb_t"], 1))
        return {"ok": True, "message": "", "active": s["state"] in ACTIVE, "state": s["state"],
                "name": s["name"], "tier": s["tier"], "file": s["file"],
                "elapsed_s": round(now - s["t0"], 1) if s["state"] in ACTIVE and s["t0"] is not None else None,
                "feedback": fb, "result": s["result"]}
