# `ocs` — the shore laptop's half of the RoboCommand link

```bash
python -m venv .venv && .venv/Scripts/python -m pip install -r requirements.txt
bash make_protos.sh                      # generates rx_bridge/gen/ from a pinned SHA
.venv/Scripts/python -m unittest discover -s tests -t .
.venv/Scripts/python -m rx_bridge --config bridge.toml
```

**State: USV only, and not yet run against a real broker.** The rules, the wire
format and the validator are tested (35 tests). The MQTT plumbing has never held
a live connection to anything — see [What is actually proven](#what-is-actually-proven).

This is **not** a ROS package and does not live on the boat. It is a plain Python
program for the laptop on shore, and that separation is the point: the handbook
permits the team exactly one OCS connection to RoboCommand and forbids vehicles
from reaching it at all, so the process holding that connection must be the one
thing that is definitely not on a vehicle.

`crusader_groundstation` is the boat's web page and runs on the Jetson. It is not
this and cannot become this.

---

## The network, and the one question everyone asks

**Do not plug the RoboNation server into the team router — neither the LAN port
nor the WAN port.** It belongs on its own segment.

- **LAN port** puts it on `192.168.8.0/24` alongside the Jetson. The boat can
  reach the broker directly, which is the prohibited topology, and — worse —
  everything will appear to work. You will not find out until the course.
- **WAN port** is the RoboBoat arrangement, and it is worse rather than better:
  the router NATs the boat out to it, so vehicles still reach RoboCommand, but
  now through a translation layer that hides what is happening.

Both violate the same clause. Vehicles must not connect directly to the
RoboCommand network or broker, and an OCS may not route, bridge or expose its
other networks through the RoboCommand-facing interface.

What the competition hands you is an RJ-45 cable and a DHCP lease on the course
subnet. Reproduce that:

```
  Jetson ─── wifi ───┐
                     │  192.168.8.0/24   (team router — the sim box is NOT on it)
      OCS laptop ────┘
           │
           │  ethernet, straight cable, no router in the middle
           │  192.168.65.0/24  ← DHCP served by the sim box
           │
      sim box  192.168.65.2   (docker compose from robonation/robocommand)
```

The sim box runs a DHCP server on that link (`dnsmasq` on Linux is four lines)
handing out `192.168.65.100–200`, and holds `.2` itself. Matching the real Alpha
course addressing is free and means the only thing that changes on competition
day is which cable you plug in.

If the sim box is Windows or you would rather not run `dnsmasq`, use a **second
router** for the course segment instead — the sim box and the OCS ethernet both
on its LAN, and nothing connecting it to the team router. A separate broadcast
domain is what matters; which box serves DHCP does not.

Three properties this setup buys you, none of which the router arrangement can:

1. **DHCP is exercised.** The handbook forbids a static address on that
   interface, and "we always ran it static on the bench" is how you discover at
   Alpha that your bridge hardcodes an IP.
2. **The prohibition becomes testable.** From the Jetson, `ping 192.168.65.2`
   must fail. If it succeeds, you are non-compliant and you now know.
3. **`ip_forward=0` starts meaning something.** With two real interfaces on the
   OCS, forgetting to disable forwarding is a live bug you can catch, rather
   than a rule you assert in a document.

Set `net.ipv4.ip_forward=0` on the OCS and verify it. That single sysctl is what
keeps the laptop an endpoint rather than a router.

> Many laptops have no RJ-45 port. If yours needs a USB-C dongle, buy two
> identical ones and check the interface name survives a replug — the bridge
> selects its interface by name.

---

## Running the whole thing on a desk

Three terminals, no boat, no radio:

```bash
# 1. RoboNation's own stub — broker, retained RxCourse, decoded message logs
git clone https://github.com/robonation/robocommand
cd robocommand/RobotX_2026 && docker compose up --build
```

```bash
# 2. the bridge
cd ocs && .venv/Scripts/python -m rx_bridge --config bridge.toml
```

```bash
# 3. a USV that does not exist
cd ocs && .venv/Scripts/python fake_vehicle.py --host 127.0.0.1
```

Then at the `rx>` prompt: `declare`, watch `test_client.py`'s log fill with
heartbeats, send a `RunStart` from the stub, and `status` to see the state
machine advance.

The full procedure -- pass/fail matrix, the two-machine network build, and a
troubleshooting table -- is [docs/G3_robocommand_bench.md](../docs/G3_robocommand_bench.md).

**If the bridge sits in `CONNECTED` and never advances**, the stub is not
publishing a retained `RxCourse`, and nothing else can proceed until it does:

```bash
.venv/Scripts/python fake_course.py --host 127.0.0.1
```

We did **not** write our own RoboCommand stub. RoboNation ships one, along with
`test_client.py`, which is a reference OCS — the thing to diff our wire output
against when a report is rejected.

### Proving the safety nets work

Two faults come from hardware states you cannot ask for on a bench, so
`fake_vehicle.py` fakes them. Run each and watch the bridge refuse the frame:

```bash
.venv/Scripts/python fake_vehicle.py --nan       # DROP ... heading_deg: NaN
.venv/Scripts/python fake_vehicle.py --unknown   # DROP ... current_task: UNKNOWN
```

A net you have never seen catch anything is a net you are guessing about.

---

## The decisions worth arguing with

**`sent_at` is stamped by the bridge, at publish, never by the vehicle.** The
vehicle sets its own clock into the field, and the bridge overwrites it —
keeping the original only to measure staleness. A Jetson whose NTP has drifted
therefore cannot corrupt the timestamp RoboCommand judges us on; the worst it
can do is trigger a `STALE` warning.

**Sequence numbers are fsync'd before the message goes out.** RxReport.seq is
per vehicle. Hold it in memory and a crash at minute nine restarts it at zero,
which on RoboCommand's side is indistinguishable from a replayed stream. The
ordering — durable first, then publish — is the entire design; the fsync per
message is noise on an SSD at 2 Hz.

**The rate governor reserves tokens for heartbeats.** 5 msg/s is a cap and 2 Hz
is an obligation, and they collide the moment task reports start flowing. A
plain FIFO bucket lets a burst of task traffic push a vehicle below its mandated
heartbeat rate. Reports may only spend down to a floor; heartbeats may spend the
floor.

*Whether the 5/s cap is per-vehicle or per-team is genuinely ambiguous in the
handbook* — read as per-team, three vehicles at the mandated 2 Hz breach it on
heartbeats alone (6 > 5), which cannot be intended. We meter per vehicle and
have asked RoboNation. If the answer is per-team, share one `Governor` across
vehicles and nothing else changes.

**We do not re-declare on reconnect.** `RunStart` carries the `declaration_seq`
it is answering. A reflexive re-declare after a dropped connection mints a new
seq and orphans the `RunStart` we are waiting for, and the run never starts.
`runstate.py` remembers that it was already declared.

**Heartbeats go out at QoS 0, everything else at QoS 1.** A retried stale
position is worse than a missing one and a 2 Hz stream self-heals in 500 ms;
an event-shaped report that vanishes costs points. *This is the call to revisit
for the UAV*, whose heartbeats are relayed for Singapore Network Remote ID — but
that is not this package's problem yet.

**Task 4 commands will never actuate anything.** Inbound `RxCommand` reaches the
mission executive as advisory input. The repo's standing rule is that the RC
e-stop is the only safety path and WiFi is a convenience; a shore network that
can move the boat inverts it. Losing the RoboCommand link likewise changes no
vehicle behaviour — there is deliberately no comms failsafe.

**The generated protobuf is committed.** `protoc` then never has to exist on the
Jetson or in the `asv` container, and RoboNation cannot renumber a field
underneath us between now and Sarasota. `make_protos.sh` pins the SHA; bump it
on purpose, read the diff, re-run the tests.

---

## Toolchain: Edition 2024 will bite you

The schemas declare `edition = "2024"`, not `proto3`. Consequences that cost
real time to rediscover:

| | |
|---|---|
| `protobuf` 5.x | **cannot load these descriptors at all.** Needs 6.x+ |
| `libprotoc` 29 and below | `Edition 2024 is later than the maximum supported edition 2023` |
| Ubuntu 22.04 `protobuf-compiler` | libprotoc 3.12 — hopeless |
| `grpcio-tools` 1.71.x | carries libprotoc 29.0 — fails |
| `grpcio-tools` 1.83.0 | carries libprotoc 35.1 — **works** |

So `make_protos.sh` prefers the `protoc` inside `.venv` over any system one.
`requirements.txt` pins the combination that is known to work.

`FieldDescriptor.label` was also removed in protobuf 7 — `validate.py` uses
`is_repeated`.

---

## What is actually proven

| | |
|---|---|
| `runstate` `seqstore` `governor` `config` | **tested**, 24 cases, no dependencies |
| `validate`, wire format, RunDeclaration, RunStart | **tested**, 11 cases against generated protobuf |
| codegen from a pinned SHA | **run**, `f6457fa` |
| `bridge.py` MQTT plumbing | **imports only.** Never held a live connection |
| `fake_vehicle.py` | **imports only.** Never published to a broker |
| the network topology above | **not built** |

The next thing to do is step 2 of the bench sequence: bring up the RoboNation
stub and run all three processes together. Everything above it is untested
plumbing until that happens.

## Layout

```
ocs/
  bridge.toml          configuration; bare keys MUST precede every [section]
  make_protos.sh       codegen from a pinned robocommand SHA
  fake_vehicle.py      a 2 Hz USV that does not exist, plus its two fault modes
  fake_course.py       a retained RxCourse, for when the stub does not send one
  rx_bridge/
    runstate.py        the run's state machine.  no I/O
    seqstore.py        durable per-vehicle sequence counters
    governor.py        5/s token bucket with a heartbeat floor.  no clock
    validate.py        no UNKNOWN enums, no NaN
    config.py          TOML, validated on load
    wirelog.py         every frame both directions, raw bytes kept
    bridge.py          the two MQTT clients and the hot path
    proto.py           the only module that touches sys.path
    gen/               generated; committed; see make_protos.sh
  tests/
    test_core.py       runs on a bare laptop
    test_wire.py       needs the venv and gen/; skips cleanly without them
```
