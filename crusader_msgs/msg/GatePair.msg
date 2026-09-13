# The next gate, named by the UAV. Published by rxl_link_node on
# /crsd/next_gate, decoded from RXL_NEXT_BUOY_SET (MAVLink id 42019).
#
# THE BOAT DOES NOT PAIR BUOYS. nav_math.hpp declines to infer gates from a
# buoy field on purpose: the handbook constrains each RED/GREEN buoy
# individually, says nothing about pairs, and an inferred pairing fails badly
# when one of a pair is missed. Above Core tier it is not an inference at all —
# the aircraft can see the whole passage from above and simply says which two.
std_msgs/Header header

# Which gate this answers. The boat publishes gate_reached(seq) and waits for
# the reply carrying the SAME seq. Without it a retransmission on a lossy radio
# is indistinguishable from the answer to the next request, and the boat would
# drive the wrong gate while everything looked healthy.
uint8 gate_seq

# The pair, by PassageBuoy.id. Order carries the whole meaning — red passes to
# starboard, green to port — so these are named rather than being an array of
# two, because a positional pair is the easy way to get a boat to drive the
# wrong side of a buoy.
#
# NO_BUOY (255) means the UAV could not fill this slot. BOTH set to NO_BUOY is
# not a failure: it is the end-of-passage signal, and it is the ONLY thing that
# ends the transit loop.
#
# That signal has to be explicit. Remove it and "the UAV has not answered yet"
# and "there are no more gates" are both silence, and on a lossy radio silence
# is the one thing you cannot read. The boat must not guess its way out of the
# field on proximity to the exit either: it has to clear every gate first, and
# the exit buoy can sit well within any sane arrival radius of the last one.
uint8 NO_BUOY=255
uint8 red_id
uint8 green_id
