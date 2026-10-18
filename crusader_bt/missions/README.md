# missions/ - custom missions saved from the ground station's Mission tab

One `<name>.xml` per mission, BehaviorTree.CPP 4 XML, written by `mission_lib.save_mission`.
Name: letters, digits, `_` and `-`, up to 64. The picker lists these as "saved"; the files in
`../behavior_trees/` are "pre-built". Edit one by loading it in the tab, not by hand.

* Self-contained: every tree it uses is inlined, with no `<include>`, so the file is the record
  of what ran. Every `<SubTree>` carries `_autoremap="true"`.
* Its first XML comment, if it starts `MISSION:`, is the one-line description in the picker.
* Commit the ones worth keeping; a saved mission is plain XML and diffs like any file.
