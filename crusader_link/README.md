# crusader_link

`rfd_link_node`: the boat's end of the RFD900 mesh to Ekko (MAVLink TUNNEL, v3 payloads).

| Topic | Direction |
|---|---|
| `/crsd/passage_plan` (PassagePlan) | out, on each of Ekko's LIGHTS frames; nothing when Ekko is silent |
| `/crsd/radio/traffic` (RadioFrame) | out, every frame sent or heard |
| `/crsd/pose` | in: the BOAT packet to Ekko, 1 Hz, only from a fresh pose |
| `/crsd/radio/send` (RadioSend), `/crsd/radio/send_test` (Trigger) | in |

`tunnel_link.py` is a byte copy of rx26_uav's `boat_link.py` (the header names the commit);
`ekko_map.py` turns Ekko's frames into the plan. `rfd_endpoint` is the serial radio, or
`udpin:127.0.0.1:14555` in the sim. Tests: `python3 -m pytest crusader_link/test`.
