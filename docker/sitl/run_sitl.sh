#!/usr/bin/env bash
# Launch Rover SITL with Crusader's tunables, MAVProxy attached exactly as on the boat
# (SITL owns the "serial", MAVProxy rebroadcasts on 14550/14551 — same topology).
#
# KNOWN LIMITATION (plan §4.6): SITL's boat model ("motorboat" frame) does not model
# OmniX lateral thrust — GUIDED behavior, AVOID_*, WP_* logic are exact (same firmware),
# strafe dynamics are not. dp_hold-style RC-override mechanisms get logic-level testing
# only; their dynamics are bench/field territory.
set -euo pipefail

SITL_DIR="${SITL_DIR:-/root/ardupilot}"
WS="${WS:-/root/robotx_ws}"
PARAMS_SRC="${PARAMS_SRC:-$WS/working_crusader_params.params}"
HOME_LOC="${HOME_LOC:-32.7020,-117.2510,0,0}"   # matches scenario origins

# Extract only the tunable subset of the boat params (param_guard's TUNABLE list) —
# hardware-specific params (SERVO mapping, GPS ports, arming) stay SITL-default.
SITL_PARM=/tmp/crusader_sitl.parm
python3 "$WS/tools/scripts/extract_sitl_params.py" "$PARAMS_SRC" > "$SITL_PARM"
echo "SITL tunables:"; cat "$SITL_PARM"

cd "$SITL_DIR"
exec Tools/autotest/sim_vehicle.py \
  -v Rover -f motorboat \
  --add-param-file="$SITL_PARM" \
  -l "$HOME_LOC" \
  --no-rebuild \
  --out 127.0.0.1:14550 --out 127.0.0.1:14551 \
  "$@"
