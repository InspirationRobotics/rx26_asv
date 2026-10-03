#!/usr/bin/env python3
"""export_engines — build oak_detector's two TensorRT engines ON the Jetson.

Runs INSIDE the asv container, where oak_detector loads them:

    docker exec -it asv bash
    python3 /root/robotx_ws/src/rx26_asv/tools/scripts/export_engines.py \
        --det /root/robotx_ws/models/crusader_det_yolo26n.pt \
        --cls /root/robotx_ws/models/crusader_led_cls.pt

Copy the two .pt files to the host's ~/robotx_ws/models/ first (that folder is
/root/robotx_ws/models/ inside asv). Expect several minutes per engine.

WHY ON THE JETSON. A .engine is built for one GPU and one TensorRT version. One
built on the laptop, or in a different container, fails to load or loads wrong.
Build it in the same container that will run it.

WHAT IT DOES, in order. It stops at the first thing that is wrong.
  1. Prints the versions it can read (L4T, torch, CUDA, TensorRT, ultralytics,
     onnx, protobuf).
  2. Loads each .pt and checks its task and its class ORDER against
     crusader_params.yaml's det_labels / cls_labels. oak_detector maps class index
     to name by that list, so a different order paints every buoy the wrong
     colour, and nothing downstream can tell.
  3. Exports. The detector is a static batch of 1 at 640, matching
     oak_detector's predict(imgsz=(640, 640)). The classifier is FP16 with a
     DYNAMIC batch of up to 8 at cls_imgsz. With a static batch of 8, ultralytics
     refuses a call with 1-7 crops, and the node's one-crop fallback would fail
     the same way.
  4. Smoke-tests each new engine: one blank frame for the detector, then 3 crops
     and 1 crop for the classifier.
  5. Installs them at det_engine / cls_engine from the YAML. An engine already
     there is RENAMED to <name>.bak-<stamp>, never overwritten.

It never restarts anything. Restart oak_detector from the GCS Nodes tab, then
read its "engines ready" log line.

PROTOBUF. The asv image pins protobuf 5.29.x (Dockerfile, "Nothing here may move
protobuf"). ultralytics pip-installs a missing export dependency (onnx,
onnxslim) by itself, and that can move protobuf in a running container where no
build guard is watching. So this script turns auto-install OFF. A missing
package then fails loudly. It also compares protobuf before and after the
export and says so if the version moved.
"""
import argparse
import datetime
import os
import shutil
import sys

# Must be set before ultralytics is imported: it reads this once, at import.
os.environ["YOLO_AUTOINSTALL"] = "false"

DEFAULT_PARAMS = "/root/robotx_ws/src/rx26_asv/crusader_bringup/config/crusader_params.yaml"
CLS_MAX_BATCH = 8


def die(msg):
    print("*** " + msg, file=sys.stderr, flush=True)
    sys.exit(1)


def find_key(tree, key):
    """The first value stored under `key` anywhere in a nested YAML document."""
    if isinstance(tree, dict):
        if key in tree:
            return tree[key]
        for v in tree.values():
            found = find_key(v, key)
            if found is not None:
                return found
    return None


def detector_params(path):
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f)
    out = {k: find_key(doc, k) for k in
           ("det_engine", "cls_engine", "det_labels", "cls_labels", "cls_imgsz")}
    missing = [k for k, v in out.items() if v is None]
    if missing:
        die("%s has no %s for oak_detector" % (path, ", ".join(missing)))
    return out


def module_version(name):
    try:
        mod = __import__(name)
        return getattr(mod, "__version__", "?")
    except Exception as e:
        return "MISSING (%s)" % type(e).__name__


def protobuf_version():
    try:
        import google.protobuf
        return google.protobuf.__version__
    except Exception:
        return None


def print_versions():
    try:
        with open("/etc/nv_tegra_release") as f:
            l4t = f.readline().strip()
    except OSError:
        l4t = "not readable here (run `cat /etc/nv_tegra_release` on the host)"
    print("L4T:         " + l4t)
    print("python:      " + sys.version.split()[0])
    for name in ("torch", "tensorrt", "ultralytics", "onnx", "onnxslim"):
        print("%-12s %s" % (name + ":", module_version(name)))
    print("protobuf:    %s" % (protobuf_version() or "MISSING"))
    try:
        import torch
        print("CUDA:        %s" % (torch.cuda.is_available() and torch.cuda.get_device_name(0)))
    except Exception as e:
        print("CUDA:        unknown (%s)" % type(e).__name__)


def load_checked(pt, task, labels):
    """Load a .pt and refuse it unless its task and class order match the node."""
    from ultralytics import YOLO
    if not os.path.isfile(pt):
        die("no such file: " + pt)
    try:
        model = YOLO(pt)
    except Exception as e:
        die("ultralytics cannot load %s (%s: %s). An ultralytics too old for this "
            "architecture fails exactly like this. Fallback detector: YOLO11n, "
            "Boat/crusader_vision/runs/det_yolo11n/weights/best.pt." % (pt, type(e).__name__, e))
    if model.task != task:
        die("%s is a %s model, oak_detector needs %s" % (pt, model.task, task))
    names = [model.names[i] for i in sorted(model.names)]
    if names != list(labels):
        die("%s classes %s != crusader_params.yaml %s. oak_detector maps index to "
            "name by the YAML, so this order would mislabel every detection."
            % (pt, names, list(labels)))
    print("ok: %s  task %s  classes %s" % (os.path.basename(pt), task, names))
    return model


def export(model, **kw):
    print("exporting: %s" % kw, flush=True)
    path = model.export(format="engine", device=0, half=True, **kw)
    if not path or not os.path.isfile(str(path)):
        die("export returned no engine file (%r)" % (path,))
    return str(path)


def smoke(engine, task, imgsz, batches):
    """Run the engine on blank input of each batch size, as oak_detector calls it."""
    import numpy as np
    from ultralytics import YOLO
    model = YOLO(engine, task=task)
    for n in batches:
        frames = [np.zeros((imgsz, imgsz, 3), dtype=np.uint8) for _ in range(n)]
        model.predict(frames if n > 1 else frames[0], imgsz=imgsz, verbose=False)
        print("ok: %s runs a batch of %d" % (os.path.basename(engine), n))


def install(built, target, stamp):
    if os.path.abspath(built) == os.path.abspath(target):
        return
    if os.path.exists(target):
        backup = "%s.bak-%s" % (target, stamp)
        os.rename(target, backup)
        print("kept the old engine as " + backup)
    shutil.move(built, target)
    print("installed " + target)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--det", required=True, help="detector .pt (circle, diamond)")
    ap.add_argument("--cls", required=True, help="LED classifier .pt (blue, green, off, red)")
    ap.add_argument("--params", default=DEFAULT_PARAMS, help="crusader_params.yaml")
    ap.add_argument("--check-only", action="store_true",
                    help="versions and the .pt checks only; export nothing")
    ap.add_argument("--no-install", action="store_true",
                    help="build and smoke-test only; leave the engines next to the .pt files")
    a = ap.parse_args()

    p = detector_params(a.params)
    print_versions()
    proto_before = protobuf_version()

    det = load_checked(a.det, "detect", p["det_labels"])
    cls = load_checked(a.cls, "classify", p["cls_labels"])
    if a.check_only:
        print("check only: both models load and match the YAML; nothing exported")
        return

    det_engine = export(det, imgsz=640, batch=1)
    cls_engine = export(cls, imgsz=int(p["cls_imgsz"]), batch=CLS_MAX_BATCH, dynamic=True)

    proto_after = protobuf_version()
    if proto_after != proto_before:
        print("*** protobuf MOVED during export: %s -> %s. The detector stack pins 5.29; "
              "tell the team before anything else runs." % (proto_before, proto_after),
              file=sys.stderr)

    smoke(det_engine, "detect", 640, [1])
    smoke(cls_engine, "classify", int(p["cls_imgsz"]), [3, 1])

    if a.no_install:
        print("built (not installed): %s, %s" % (det_engine, cls_engine))
        return
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    install(det_engine, p["det_engine"], stamp)
    install(cls_engine, p["cls_engine"], stamp)
    print("done. Restart oak_detector from the GCS Nodes tab and look for its "
          "'engines ready' line; the old engines are the .bak-%s files." % stamp)


if __name__ == "__main__":
    main()
