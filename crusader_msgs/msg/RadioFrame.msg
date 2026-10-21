# One MAVLink frame that crossed the RFD900 mesh, as rfd_link_node sent or heard it. A record
# for the ground station's Radio tab: nothing acts on it.
std_msgs/Header header    # when rfd_link_node sent or received it
uint8 DIR_TX=1
uint8 DIR_RX=2
uint8 direction
uint8 src_system
uint8 src_component
uint8 dst_system          # 0 = broadcast, or no target field
string msg_name           # BOAT, POSITIONS, LIGHTS, TEST, ... or the MAVLink name
uint16 payload_type       # the TUNNEL payload_type; 0 for a native message
string summary            # one line, decoded where the boat knows the format
uint16 frame_bytes        # bytes on the air, MAVLink framing included
