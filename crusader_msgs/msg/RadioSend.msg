# A TUNNEL frame for rfd_link_node to put on the RFD900 mesh.
uint8 target_system   # 0 = broadcast
uint16 payload_type
uint8[] payload       # at most 128 bytes
