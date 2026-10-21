# crusader_msgs

The typed contracts between nodes: `msg/` and `action/RunMission.action`. Each file's header
says who produces it, who consumes it and on which topic.

Add the file **and** its line in `CMakeLists.txt`: an unlisted file is silently not generated,
and the import fails at node start, on the boat. A changed field needs the whole stack rebuilt
(`tools/scripts/rebuild.sh`) and every node restarted: a mismatched message is a silent
deserialisation failure. Dependencies stay `std_msgs` and `action_msgs`. Messages for
capabilities the repo does not have are not kept for later.
