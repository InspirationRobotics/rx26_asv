# What the boat is doing, as one message: the autonomy state and the task.
#
# Published by mission_planner and consumed by ocs_client, which passes these
# two strings straight into the OCS heartbeat. That is why they are STRINGS and
# not an enum of our own: the values are the protobuf enum names from
# RoboNation's schema (common.proto RobotState, rx_common.proto RxTask), so
# they travel end to end with no mapping layer to get out of step.
#
# The planner is the ONLY node that decides these. ocs_client used to derive
# state itself from armed+autonomy_active; two nodes answering the same question
# is how they eventually answer it differently.
std_msgs/Header header

# STATE_AUTO | STATE_MANUAL | STATE_KILLED.
# Never STATE_UNKNOWN: it is the proto zero value, the OCS validator refuses it,
# and it would claim "the reporter forgot" rather than "the boat does not know".
# When the state is not knowable the planner publishes nothing at all and
# ocs_client lets the heartbeat lapse, which is the truthful signal.
string state

# TASK_NONE | TASK_SAFE_PASSAGE | TASK_INFRA_SURVEY_REPAIR |
# TASK_COORDINATED_LOGISTICS | TASK_DYNAMIC_INCIDENT.
# TASK_UNKNOWN is never sent -- the handbook forbids it outright.
string task

# Why the state is what it is, in words: "armed in GUIDED", "disarmed",
# "RC kill active". Goes to the operator page. A state without a reason is a
# thing you debug by guessing.
string reason

# Whether a mission is executing. On the boat this is currently always false;
# it exists because the aircraft needs it -- a UAV in LOITER under CV control is
# autonomous, and the mode alone cannot tell you that.
bool mission_active

# Which mission, when one is running. Empty otherwise.
string mission_name
