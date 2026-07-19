# APF advisory output (plan §3.2 — ADVISORY ONLY, never a setpoint).
# The active task node (gate_navigator wrapper / dp_hold) blends this into its
# own output; the mission planner watches at_risk/stalled for escape decisions.
std_msgs/Header header
float64 corrected_lat      # corrected goal (lat/lon so GUIDED tasks use it directly)
float64 corrected_lon
float32 speed_scale        # [min_scale..1] multiply cruise speed by this
float32 repulsion          # |repulsive vector|; 0 = no correction active
bool potential_trap        # APF trap detected (tangential bias engaged)
string progress_state      # "ok" | "at_risk" | "stalled" (ProgressMonitor)
string[] sources           # contributing obstacle sources: perception/comms/moving
