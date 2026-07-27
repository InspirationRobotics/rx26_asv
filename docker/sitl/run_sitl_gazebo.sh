#!/usr/bin/env bash
# Launch Rover SITL with Gazebo as the physics backend — the OmniX-capable path.
#
# WHAT THIS FIXES
# ---------------
# run_sitl.sh's documented limitation:
#
#   "SITL's boat model ('motorboat' frame) does not model OmniX lateral thrust —
#    GUIDED behavior, AVOID_*, WP_* logic are exact (same firmware), strafe
#    dynamics are not. dp_hold-style RC-override mechanisms get logic-level
#    testing only; their dynamics are bench/field territory."
#
# Crusader is FRAME_TYPE=2 (OmniX): four thrusters in an X, genuinely holonomic.
# dp_hold is THE lateral-hold path (MANUAL + RC override, because GUIDED cannot
# strafe on this frame), and today its dynamics can only be checked on the water.
#
# This script keeps EVERYTHING about the firmware identical — same ArduRover
# 4.6.3, same extracted tunable params, same MAVProxy topology on 14550/14551 —
# and swaps only the FDM: Gazebo drives a real four-thruster OmniX model instead
# of SITL's built-in motorboat. Lateral thrust becomes real, so dp_hold and
# station-keeping get dynamics-level testing.
#
# WHAT STAYS THE SAME
#   - firmware, params, MAVProxy ports, scenario JSON, evaluator, metrics
#   - `--backend gazebo` episodes are directly comparable to `--backend sitl`
#
# USAGE
#   RX26_GZ_WORLD=tools/sim/worlds/mission1_transit.sdf docker/sitl/run_sitl_gazebo.sh
#   SCENARIO=orchestrator/scenarios/mission1_transit.json docker/sitl/run_sitl_gazebo.sh
#
# Then, in another shell:
#   RX26_SITL_OK=1 python3 orchestrator/run_episode.py \
#       --scenario orchestrator/scenarios/mission1_transit.json \
#       --backend gazebo --out /tmp/ep.json
set -euo pipefail

SITL_DIR="${SITL_DIR:-/root/ardupilot}"
WS="${WS:-/root/robotx_ws}"
PARAMS_SRC="${PARAMS_SRC:-$WS/working_crusader_params.params}"
GZ_PLUGIN_DIR="${GZ_PLUGIN_DIR:-/root/ardupilot_gazebo/build}"
GZ_MODEL_DIR="${GZ_MODEL_DIR:-$WS/tools/sim/models}"

# --- world + origin, both derived from the scenario -------------------------- #
# HOME_LOC is NOT hardcoded here. run_sitl.sh pins 32.7020,-117.2510 with the
# comment "matches scenario origins" — true today, but it is a second copy of a
# number that lives in the scenario JSON, and the day a scenario moves to the
# real venue the two silently disagree. Read it from the scenario instead.
SCENARIO="${SCENARIO:-}"
RX26_GZ_WORLD="${RX26_GZ_WORLD:-}"

if [[ -n "$SCENARIO" ]]; then
  NAME=$(python3 -c "import json,sys;print(json.load(open('$SCENARIO'))['name'])")
  RX26_GZ_WORLD="${RX26_GZ_WORLD:-$WS/tools/sim/worlds/${NAME}.sdf}"
  HOME_LOC=$(python3 -c "
import json; o=json.load(open('$SCENARIO'))['origin']; print(f'{o[0]},{o[1]},0,0')")
  echo "scenario : $SCENARIO"
elif [[ -n "$RX26_GZ_WORLD" ]]; then
  HOME_LOC=$(python3 -c "
import re,sys
t=open('$RX26_GZ_WORLD').read()
la=re.search(r'<latitude_deg>([-0-9.]+)',t).group(1)
lo=re.search(r'<longitude_deg>([-0-9.]+)',t).group(1)
print(f'{la},{lo},0,0')")
else
  echo "ERROR: set SCENARIO=<scenario.json> or RX26_GZ_WORLD=<world.sdf>" >&2
  exit 2
fi

if [[ ! -f "$RX26_GZ_WORLD" ]]; then
  echo "ERROR: world not found: $RX26_GZ_WORLD" >&2
  echo "Generate it:  python3 tools/sim/scenario_to_world.py --all" >&2
  exit 2
fi

echo "world    : $RX26_GZ_WORLD"
echo "home     : $HOME_LOC   (from the scenario — not hardcoded)"

# --- params: identical extraction to run_sitl.sh ----------------------------- #
SITL_PARM=/tmp/crusader_sitl.parm
python3 "$WS/tools/scripts/extract_sitl_params.py" "$PARAMS_SRC" > "$SITL_PARM"
echo "SITL tunables:"; cat "$SITL_PARM"

# --- Gazebo ------------------------------------------------------------------ #
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_PLUGIN_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${GZ_MODEL_DIR}:$(dirname "$RX26_GZ_WORLD"):${GZ_SIM_RESOURCE_PATH:-}"

if [[ ! -d "$GZ_PLUGIN_DIR" ]]; then
  echo "ERROR: ardupilot_gazebo plugin not built at $GZ_PLUGIN_DIR" >&2
  echo "Run:  docker/sitl/install_gazebo.sh" >&2
  exit 2
fi

HEADLESS="${HEADLESS:-1}"
GZ_ARGS=(-v4 -r "$RX26_GZ_WORLD")
[[ "$HEADLESS" == "1" ]] && GZ_ARGS=(-s "${GZ_ARGS[@]}")   # server only, for CI

echo
echo "starting Gazebo (headless=$HEADLESS)…"
gz sim "${GZ_ARGS[@]}" &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true' EXIT INT TERM

# Wait for the plugin to bind the FDM port. GazeboBackend probes this same port
# and refuses to run if nothing holds it — without that guard SITL silently
# falls back to the motorboat FDM and reports strafe results that are not real.
echo -n "waiting for ardupilot_gazebo on udp/9002 "
for _ in $(seq 1 40); do
  if python3 -c "
import socket,sys
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
try: s.bind(('127.0.0.1',9002)); sys.exit(1)   # free -> not up yet
except OSError: sys.exit(0)                    # in use -> Gazebo has it
finally: s.close()"; then echo " ok"; break; fi
  echo -n "."; sleep 0.5
done

# --- SITL: same firmware, same params, JSON FDM instead of motorboat --------- #
cd "$SITL_DIR"
exec Tools/autotest/sim_vehicle.py \
  -v Rover -f JSON \
  --add-param-file="$SITL_PARM" \
  -l "$HOME_LOC" \
  --no-rebuild \
  --out 127.0.0.1:14550 --out 127.0.0.1:14551 \
  "$@"
