# sim_aliases.sh - the water's short commands, for the Gazebo sim. In WSL2 Ubuntu-22.04:
#
#   bash /mnt/c/.../rx26_asv/crusader_sim/scripts/sim_aliases.sh --install    # once
#
# then in any NEW terminal (or: source ~/.bash_aliases):
#
#   sim up              the boat on the water (Gazebo, the autopilot, the boat's stack)
#   tx arm; tx manual   the pilot: arm, SC to MANUAL (tx keys: the whole transmitter)
#   tx guided; sim wp right   GUIDED, and one waypoint 5 m off the right bay (or QGC)
#   t3 start            the Task 3 launch, the SAME script as on the Jetson (leave it running)
#   t3 go [wp] | t3 steps | t3 log | slip | cal ...   the same as on the Jetson
#   sim log | sim robocommand | sim lights   the referee's side, live
#   sim wind 6 90 | sim current 0.2 0 | sim calm | sim verdict | sim down
#
# t3 / slip / cal run tools/scripts/task3/ in the crsd-sim container, exactly as the
# Jetson's aliases run them in the asv container; CRSD_SIM=1 is the only difference
# (t3 start adds sim:=true: the sim is the camera, the tree may move the simulated
# boat). docs/T3_sim_commands.md has the two side by side.
_rx26_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_rx26_task3=/root/robotx_ws/src/rx26_asv/tools/scripts/task3

if [ "${1:-}" = "--install" ]; then
  line="[ -f \"$_rx26_here/sim_aliases.sh\" ] && . \"$_rx26_here/sim_aliases.sh\""
  touch ~/.bash_aliases
  grep -qF "$_rx26_here/sim_aliases.sh" ~/.bash_aliases || echo "$line" >> ~/.bash_aliases
  grep -q "bash_aliases" ~/.bashrc || echo '[ -f ~/.bash_aliases ] && . ~/.bash_aliases' >> ~/.bashrc
  echo "installed: open a new terminal (or: source ~/.bash_aliases), then: sim up"
  return 0 2>/dev/null || exit 0
fi

alias sim="bash '$_rx26_here/sim.sh'"
alias tx="bash '$_rx26_here/tx.sh'"
alias t3="docker exec -it -e CRSD_SIM=1 crsd-sim bash $_rx26_task3/t3.sh"
alias slip="docker exec -it -e CRSD_SIM=1 crsd-sim bash $_rx26_task3/slip.sh"
alias cal="docker exec -it -e CRSD_SIM=1 crsd-sim bash $_rx26_task3/cal.sh"
