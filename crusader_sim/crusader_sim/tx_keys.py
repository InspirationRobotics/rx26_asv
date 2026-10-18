"""tx_keys — the pilot's transmitter on the keyboard, for the Gazebo sim: `tx keys`.

    python3 -m crusader_sim.tx_keys        (crusader_sim/scripts/tx.sh keys)

Runs in: WSL, next to crusader_sim.sim_transmitter, whose RC channels go into the
autopilot exactly as the RadioMaster's receiver does on the boat. So a key here is
a switch there: SC changes the mode, SB e-stops the motors, SD trips the autonomy
drop latch (the tree lets go), SE runs the pump. The sticks drive the boat in
MANUAL whenever the tree is not overriding them.

    SC  m MANUAL   h HOLD   a AUTO          g GUIDED (the ground station's button)
    SD  d  drop latch switch up/down        SB  e  e-stop on/off      SE  p  pump on/off
    r arm   x disarm
    sticks: arrows = throttle / steer, z c = slide left / right, space = centre all
    q quit (sticks centred; the switches stay where they are)

The autopilot's side (mode, armed, the drop latch) comes from crusader_sim.fcu_watch
in the crsd-sim container, once a second.
"""
import curses
import json
import os
import subprocess
import threading
import time

from crusader_sim import sim_transmitter as tx

CONTAINER = os.environ.get("RX26_CONTAINER", "crsd-sim")
ROS = "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && "
TRIM = {1: 1489, 3: 1495, 4: 1495}       # = sim_transmitter IDLE: steer, throttle, lateral
STEP_US = 25
MAX_DEFLECT = 300


def ctl(msg):
    try:
        return tx.control(msg)
    except SystemExit as e:              # control() exits when nothing answers
        return {"error": str(e)}


def set_ch(ch, us):
    return ctl({"op": "set", "ch": ch, "us": int(us)})


class Keys:
    def __init__(self):
        self.fcu = {"mode": None, "armed": None, "dropped": None, "stale": True}
        self.note = "ready"
        self.chans = {}
        self.lock = threading.Lock()
        self.watch = None
        threading.Thread(target=self._watch_loop, daemon=True).start()

    # --- the autopilot's side, from the container ---
    def _watch_loop(self):
        while True:
            try:
                self.watch = subprocess.Popen(
                    ["docker", "exec", "-i", CONTAINER, "bash", "-lc", ROS + "python3 -u -m crusader_sim.fcu_watch"],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                for line in self.watch.stdout:
                    try:
                        with self.lock:
                            self.fcu = json.loads(line)
                    except ValueError:
                        pass
            except OSError:
                pass
            time.sleep(2.0)

    def in_rig(self, label, cmd):
        def run():
            self.say(f"{label}...")
            r = subprocess.run(["docker", "exec", "-i", CONTAINER, "bash", "-lc", ROS + cmd],
                               capture_output=True, text=True)
            out = (r.stdout.strip().splitlines() or [r.stderr.strip() or "done"])[-1]
            self.say(f"{label}: {out[:60]}")
        threading.Thread(target=run, daemon=True).start()

    def say(self, s):
        with self.lock:
            self.note = s

    # --- the switches and sticks ---
    def flick_mode(self, name):
        """SC acts on a CHANGE: through another position first, as a pilot would."""
        other = "hold" if name != "hold" else "manual"
        def run():
            set_ch(8, tx.NAMED["mode"][1][other])
            time.sleep(0.6)
            set_ch(8, tx.NAMED["mode"][1][name])
            self.say(f"SC -> {name.upper()}")
        threading.Thread(target=run, daemon=True).start()

    def toggle(self, what):
        ch, table = tx.NAMED[what]
        cur = self.chans.get(str(ch), self.chans.get(ch))
        on = table["on"]
        new = table["off"] if cur == on else on
        set_ch(ch, new)
        self.say(f"{what} {'ON' if new == on else 'off'}")

    def stick(self, ch, delta):
        cur = int(self.chans.get(str(ch), TRIM[ch]))
        new = max(TRIM[ch] - MAX_DEFLECT, min(TRIM[ch] + MAX_DEFLECT, cur + delta))
        set_ch(ch, new)

    def centre(self):
        for ch, us in TRIM.items():
            set_ch(ch, us)
        self.say("sticks centred")

    def refresh(self):
        s = ctl({"op": "show"})
        if "error" not in s:
            self.chans = s
        return s


def label(chans, ch, table):
    us = chans.get(str(ch))
    for k, v in table.items():
        if us == v:
            return k
    return f"{us} us"


def draw(scr, k):
    scr.erase()
    c = k.chans
    with k.lock:
        f, note = dict(k.fcu), k.note
    lines = [
        "SIM TRANSMITTER  (crusader_sim.tx_keys)                          q quit",
        "",
        f"  SC mode switch : {label(c, 8, tx.NAMED['mode'][1]).upper():8s}   m MANUAL  h HOLD  a AUTO   (g GUIDED)",
        f"  SD drop latch  : {label(c, 9, tx.NAMED['drop'][1]).upper():8s}   d toggle   (on = the tree lets go)",
        f"  SB e-stop      : {label(c, 7, tx.NAMED['estop'][1]).upper():8s}   e toggle   (motors only)",
        f"  SE pump        : {label(c, 10, tx.NAMED['pump'][1]).upper():8s}   p toggle   (SB does not stop it)",
        "",
        "  sticks (us from trim)   arrows: throttle / steer   z c: slide   space: centre",
        f"    throttle {int(c.get('3', TRIM[3])) - TRIM[3]:+5d}   steer {int(c.get('1', TRIM[1])) - TRIM[1]:+5d}"
        f"   lateral {int(c.get('4', TRIM[4])) - TRIM[4]:+5d}",
        "",
        f"  AUTOPILOT: mode {f.get('mode') or '?'}   armed {f.get('armed')}   drop latch "
        f"{'TRIPPED' if f.get('dropped') else 'clear' if f.get('dropped') is False else '?'}"
        + ("   (no status: is the sim up?)" if f.get("stale") else ""),
        "  r arm   x disarm",
        "",
        f"  > {note}",
    ]
    h, w = scr.getmaxyx()
    for i, ln in enumerate(lines[:h - 1]):
        scr.addstr(i, 0, ln[:w - 1])
    scr.refresh()


def main_loop(scr):
    curses.curs_set(0)
    scr.nodelay(True)
    scr.timeout(200)
    k = Keys()
    if "error" in k.refresh():
        k.say("no sim_transmitter answering: is the sim up (sim up)?")
    while True:
        k.refresh()
        draw(scr, k)
        ch = scr.getch()
        if ch == -1:
            continue
        if ch in (ord("q"), ord("Q")):
            k.centre()
            break
        elif ch == ord("m"):
            k.flick_mode("manual")
        elif ch == ord("h"):
            k.flick_mode("hold")
        elif ch == ord("a"):
            k.flick_mode("auto")
        elif ch == ord("g"):
            k.in_rig("GUIDED", "ros2 topic pub --once /crsd/set_mode std_msgs/msg/String '{data: GUIDED}' "
                               ">/dev/null && echo sent")
        elif ch == ord("d"):
            k.toggle("drop")
        elif ch == ord("e"):
            k.toggle("estop")
        elif ch == ord("p"):
            k.toggle("pump")
        elif ch == ord("r"):
            k.in_rig("arm", "python3 -c 'from crusader_sim.operator_tools import arm; print(arm()[1])'")
        elif ch == ord("x"):
            k.in_rig("disarm", "python3 -c \"from pymavlink import mavutil; "
                               "m = mavutil.mavlink_connection('udpin:127.0.0.1:14550', source_system=255); "
                               "m.wait_heartbeat(timeout=10); m.arducopter_disarm(); "
                               "a = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3); "
                               "print('disarmed' if a is not None and a.result == 0 else 'no ACK')\"")
        elif ch == curses.KEY_UP:
            k.stick(3, +STEP_US)
        elif ch == curses.KEY_DOWN:
            k.stick(3, -STEP_US)
        elif ch == curses.KEY_RIGHT:
            k.stick(1, +STEP_US)
        elif ch == curses.KEY_LEFT:
            k.stick(1, -STEP_US)
        elif ch == ord("c"):
            k.stick(4, +STEP_US)
        elif ch == ord("z"):
            k.stick(4, -STEP_US)
        elif ch == ord(" "):
            k.centre()
    if k.watch is not None:
        k.watch.terminate()


def main():
    curses.wrapper(main_loop)


if __name__ == "__main__":
    main()
