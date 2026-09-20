# One MAVLink frame that crossed the RFD900 mesh, as rxl_link_node sent it or
# heard it. Published on /crsd/radio/traffic for the ground station's Radio tab.
#
# A RECORD, NOT A CHANNEL. Nothing may act on this topic. It is published after
# the fact by the one node that owns the radio, so an operator can see what went
# out and what came in; acting on it would be a second path off the boat with
# none of rxl_link_node's checks on it.
#
# Field for field the same as rx26_uav's uav_msgs/RadioFrame, so the two
# vehicles' Radio tabs stay one design and a later shared package is a move,
# not a rewrite.
std_msgs/Header header    # when rxl_link_node sent or received the frame

uint8 DIR_TX=1
uint8 DIR_RX=2
uint8 direction

uint8 src_system          # who sent it; for TX, rxl_link_node's own id
uint8 src_component
uint8 dst_system          # who it is addressed to; 0 = broadcast, or no target field

string msg_name           # SAFE_PASSAGE, BUOY_MAP, TEST, TUNNEL_0x80xx, or the MAVLink name
uint16 payload_type       # the TUNNEL payload_type; 0 for a native message
string summary            # one line, decoded where the boat knows the format
uint16 frame_bytes        # bytes on the air, MAVLink framing included
