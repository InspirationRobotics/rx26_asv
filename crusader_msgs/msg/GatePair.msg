# The aircraft's ACKNOWLEDGEMENT of a confirmation request. Published by
# rxl_link_node on /crsd/next_gate, decoded from RXL_NEXT_BUOY_SET (id 42019).
#
# IT NO LONGER NAMES A GATE, and the name is kept only because the wire message
# is. The boat receives ten buoys with their colours in RXL_SAFE_PASSAGE, pairs
# them itself and picks the order (nav::planPassage); having crossed one it asks
# "is the rest of the field still where you said it was?" and this is the yes.
#
# The aircraft answers by retransmitting the whole field and then sending this.
# The field goes first on purpose: the boat treats the acknowledgement as
# permission to drive on, so an ack that overtook the new positions would let it
# leave on the old ones.
std_msgs/Header header

# Which request this answers. The boat publishes gate_reached(seq) and waits for
# the reply carrying the SAME seq. Without it a retransmission on a lossy radio
# is indistinguishable from the answer to the next request, and the boat would
# drive on believing a stale field had been confirmed.
uint8 gate_seq

# VESTIGIAL. These carried the pair the aircraft assigned, back when it did the
# assigning. They are sent as NO_BUOY and the boat IGNORES them.
#
# Kept rather than removed because the message flies between two vehicles whose
# software is updated separately: a boat that quietly drove whatever an older
# aircraft happened to put here would be worse than one that ignores it.
# bt_runner logs a throttled warning if either is set, so a mismatched aircraft
# is visible rather than silent.
uint8 NO_BUOY=255
uint8 red_id
uint8 green_id
