# The Task 1 passage as the UAV sees it. Published by rxl_link_node on
# /crsd/passage_plan, decoded from RXL_SAFE_PASSAGE (MAVLink id 42011).
#
# WHY THIS IS NOT TrackedTargetArray. Two reasons, and both bite.
#
#   1. PROVENANCE. TrackedTargetArray means "my sensors saw this". This means
#      "the aircraft told me this". At Advanced and Disruptive tier those are
#      different claims with different authority, and a Context field that
#      cannot tell them apart cannot implement "the UAV wins".
#   2. VERSIONING. Disruptive tier is defined by the passage CHANGING during the
#      transit. A plan needs to be able to supersede the previous one, and
#      TrackedTargetArray has no notion of that.
#
# THE STAMP IS RECEIPT TIME, not the aircraft's clock. The two vehicles do not
# share a time base, so a UAV-side timestamp cannot be aged against the boat's
# clock without a sync that does not exist yet. What the boat actually needs to
# know is "how long since I last heard a plan", and receipt time answers that
# honestly. bt_runner_node ages this the same way it ages pose, and the tree
# refuses to transit on a stale one: a dead radio must stop the boat, not leave
# it driving a plan it can no longer confirm.
std_msgs/Header header

# Bumps every time the UAV changes anything. The boat does not diff the plan; it
# watches this number, and a change halts the leg in flight so the new plan is
# used from the next tick. Wrapping is not a concern at 0.2 Hz.
uint32 plan_version

# ENTRY (flashing blue, circled clockwise) and EXIT (steady blue, circled
# anticlockwise), handbook 3.3.2. These come from the plan rather than from the
# boat's own perception because the EXIT is typically far beyond camera range
# for the whole first half of the transit — in SITL on 2026-09-09 the boat never
# saw it at all, and nav::nextWaypoint fell back on the live heading and threw
# away every gate buoy as astern.
float64 entry_latitude
float64 entry_longitude
float64 exit_latitude
float64 exit_longitude

# At most 10 per handbook 3.3.2. Sent whole on every update, never as a delta:
# a delta stream over a lossy radio needs the receiver to have caught every
# previous frame, and RXL_SAFE_PASSAGE at 0.2 Hz is cheap enough that it does
# not have to.
PassageBuoy[] buoys
