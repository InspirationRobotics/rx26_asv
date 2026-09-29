"""sim_transmitter — the RadioMaster, for SITL: raw RC into ArduPilot's UDP RC port.

    python3 -m crusader_sim.sim_transmitter                 # run (gz_sim_up.sh does this)
    python3 -m crusader_sim.sim_transmitter set estop on    # SB to e-stop
    python3 -m crusader_sim.sim_transmitter set mode manual # SC: manual | hold | auto
    python3 -m crusader_sim.sim_transmitter set drop on     # autonomy-drop switch
    python3 -m crusader_sim.sim_transmitter set ch 3 1600   # any channel, raw us
    python3 -m crusader_sim.sim_transmitter show

Runs in: WSL, next to SITL. Not a ROS node: it is the pilot's hand, so it lives
BELOW MAVLink, exactly where the real receiver does. That is what makes
RC_CHANNELS, the e-stop, the mode switch and the autonomy-drop latch behave in
the sim the way they do on the water, and it is why /crsd/rc_override (an
RC_CHANNELS_OVERRIDE on top of these values) exercises the same path it does on
the boat.

WHY IT EXISTS. SITL's own default "transmitter" (AP_RCProtocol_UDP.cpp:17-26)
holds ch7 at 1000 us. On this boat ch7 is SB with RC7_OPTION=165, so 1000 us is
EMERGENCY STOP: the vehicle arms, and every motor sits at neutral. It also puts
ch8 (MODE_CH) at 1800 = MODE6 = AUTO. This sends the boat's real idle state
instead.

Channel roles (crusader_params.yaml + working_crusader.params):
    1 steer   3 throttle   4 lateral        (override_channels [1, 3, 4])
    7 SB e-stop, RC7_OPTION=165: <1200 e-stop (bridge estop_threshold), mid run
    8 SC mode switch, MODE_CH=8: MODE1 MANUAL, MODE4 HOLD, MODE6 AUTO
    9 autonomy-drop, telemetry_bridge drop_channel=9: >=1700 trips the latch
"""
import argparse
import json
import os
import socket
import struct
import sys
import time

SITL_RC_PORT = 5501          # AP_HAL_SITL RCIN_PORT (instance 0)
CTRL_PORT = 5509             # this process's own control port (localhost only)
RATE_HZ = 50.0

IDLE = {1: 1500, 2: 1500, 3: 1500, 4: 1500, 5: 1500, 6: 1000,
        7: 1500,             # SB mid: run, not e-stopped, not arming
        8: 1500,             # SC mid: MODE4 = HOLD
        9: 1000,             # autonomy-drop: not dropped
        10: 1500, 11: 1500, 12: 1500, 13: 1500, 14: 1500, 15: 1500, 16: 1500}

NAMED = {
    "estop": (7, {"on": 1000, "off": 1500}),
    "mode": (8, {"manual": 1000, "hold": 1500, "auto": 1900}),
    "drop": (9, {"on": 1900, "off": 1000}),
}


def run(host):
    chans = dict(IDLE)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctl.bind(("127.0.0.1", CTRL_PORT))
    ctl.setblocking(False)
    period = 1.0 / RATE_HZ
    print(f"sim_transmitter -> udp {host}:{SITL_RC_PORT} at {RATE_HZ:.0f} Hz; "
          f"control on 127.0.0.1:{CTRL_PORT}", flush=True)
    while True:
        try:
            data, addr = ctl.recvfrom(512)
            cmd = json.loads(data.decode())
            if cmd.get("op") == "set":
                chans[int(cmd["ch"])] = int(cmd["us"])
                print(f"ch{cmd['ch']} = {cmd['us']}", flush=True)
            ctl.sendto(json.dumps(chans).encode(), addr)
        except BlockingIOError:
            pass
        except (ValueError, KeyError) as e:
            print(f"bad control message: {e}", flush=True)
        tx.sendto(struct.pack("<16H", *[chans[i] for i in range(1, 17)]),
                  (host, SITL_RC_PORT))
        time.sleep(period)


def control(msg):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1.0)
    s.sendto(json.dumps(msg).encode(), ("127.0.0.1", CTRL_PORT))
    try:
        return json.loads(s.recvfrom(2048)[0].decode())
    except socket.timeout:
        sys.exit("no sim_transmitter running (nothing answered on "
                 f"127.0.0.1:{CTRL_PORT})")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd")
    r = sub.add_parser("run")
    r.add_argument("--host", default="127.0.0.1")
    st = sub.add_parser("set")
    st.add_argument("what", help="estop | mode | drop | ch")
    st.add_argument("args", nargs="+")
    sub.add_parser("show")
    a = ap.parse_args()
    if a.cmd in (None, "run"):
        run(getattr(a, "host", "127.0.0.1"))
    elif a.cmd == "show":
        print(json.dumps(control({"op": "show"}), indent=0))
    else:
        if a.what == "ch":
            ch, us = int(a.args[0]), int(a.args[1])
        else:
            ch, table = NAMED[a.what]
            us = table[a.args[0].lower()]
        state = control({"op": "set", "ch": ch, "us": us})
        print(f"ch{ch} -> {us} us   (now: {state})")


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    main()
