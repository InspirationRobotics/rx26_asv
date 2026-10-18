#!/bin/bash
# Sync the Windows checkout into the WSL sim workspace, stripping CR.
#
# Runs INSIDE WSL. The WSL clone at ~/robotx_ws/src/rx26_asv is a separate checkout whose git HEAD
# lags the Windows one (content is copied in as working-tree changes), so do NOT trust `git log`
# there to say whether it is current. Compare content.
#
# WHY WHOLE-TREE. A hand-kept file list kept missing things (crusader_params.yaml, which crashed
# target_tracker with KeyError: 'use_lidar' and looked like a code bug).
#
# THE EXTENSION LIST BELOW IS THE SAME TRAP ONE LEVEL DOWN: *.txt was missing, so
# tools/sitl/rig_processes.txt (the sim rig's kill list) never reached WSL and nodes_down.sh
# reported "0 stopped, none left" with every node still running. '*.txt' now covers CMakeLists.txt
# too. Before adding a file type, check it is not bulk: this copies whatever it matches on every
# bring-up.
MODE="${1:-report}"
SRC="${RX26_WIN_SRC:-/mnt/c/Users/chase/OneDrive/Documents/GitHub/RobotX_2026/Boat/rx26_asv}"
DST="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"

cd "$SRC" || { echo "no Windows checkout at $SRC" >&2; exit 1; }

mapfile -t FILES < <(
  find . \( -path ./.git -o -name build -o -name install -o -name log \
            -o -name __pycache__ -o -name '.pytest_cache' \) -prune -o \
       -type f \( -name '*.py' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \
                  -o -name '*.xml' -o -name '*.yaml' -o -name '*.sh' \
                  -o -name '*.msg' -o -name '*.action' -o -name '*.srv' \
                  -o -name '*.txt' -o -name '*.md' \
                  -o -name '*.parm' -o -name '*.params' -o -name '*.Dockerfile' \
                  -o -name 'setup.cfg' -o -path '*/resource/*' \) -print \
  | sed 's|^\./||' | sort
)

diffc=0; newc=0; samec=0
for f in "${FILES[@]}"; do
  w=$(tr -d '\r' < "$SRC/$f" | md5sum | cut -d' ' -f1)
  if [ ! -f "$DST/$f" ]; then
    printf 'NEW   %s\n' "$f"; newc=$((newc+1))
    [ "$MODE" = sync ] && { mkdir -p "$(dirname "$DST/$f")"; tr -d '\r' < "$SRC/$f" > "$DST/$f"; }
    continue
  fi
  l=$(tr -d '\r' < "$DST/$f" | md5sum | cut -d' ' -f1)
  if [ "$w" != "$l" ]; then
    printf 'DIFF  %s\n' "$f"; diffc=$((diffc+1))
    # Written in place rather than cp -p, so an existing executable bit lives.
    [ "$MODE" = sync ] && tr -d '\r' < "$SRC/$f" > "$DST/$f"
  else
    samec=$((samec+1))
  fi
done

echo
echo "same=$samec  diff=$diffc  new=$newc   (mode=$MODE)"
[ "$MODE" = sync ] && echo "synced Windows -> WSL, CR stripped"
exit 0
