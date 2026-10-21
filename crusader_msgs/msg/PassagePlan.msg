# The Task 1 passage as the UAV sees it, on /crsd/passage_plan: rfd_link_node from Ekko's
# frames, or the ground station's MANUAL field. Stamped at receipt (the vehicles share no
# clock). plan_version moves only when the content changes. ENTRY / EXIT are (0, 0) when the
# plan has none; no plan at all is published as nothing.
std_msgs/Header header
uint32 plan_version
float64 entry_latitude
float64 entry_longitude
float64 exit_latitude
float64 exit_longitude
PassageBuoy[] buoys
