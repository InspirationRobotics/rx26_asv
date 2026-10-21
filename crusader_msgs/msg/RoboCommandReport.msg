# One RxReport body for ocs_client to send to the OCS.
std_msgs/Header header
string kind        # the RxReport oneof name: safe_passage, docking, firefighting, resource_delivery, ...
string body_json   # that message as protobuf JSON (rx_reports.proto field names)
