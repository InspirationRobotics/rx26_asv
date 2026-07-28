#!/usr/bin/env python3
"""train_buoy — fine-tune the buoy detector on Crusader footage + export the engine.

Runs on the Jetson (in the asv container) or any CUDA machine for training;
ENGINE EXPORT MUST HAPPEN ON THE TARGET JETSON — .engine files are compiled
per-GPU/per-JetPack (CLAUDE.md). Train elsewhere if convenient, then copy the
.pt here and run with --export-only.

Usage:
  # fine-tune from the V16 weights and export:
  python3 train_buoy.py --data ~/datasets/buoys_v1/dataset.yaml \
      --base /root/robotx_ws/models/buoy_v16.pt --epochs 60

  # regenerate engine only (e.g. after a JetPack upgrade):
  python3 train_buoy.py --export-only --weights runs/detect/train/weights/best.pt

Every run writes training_manifest.json next to the weights: dataset sha, base
weights, git sha, epochs — the G2 sign-off records this manifest.
"""
import argparse
import json
import subprocess
import time
from pathlib import Path


def git_sha():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True).stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def export_engine(weights: str):
    from ultralytics import YOLO
    model = YOLO(weights)
    path = model.export(format="engine", device=0, half=True)
    print(f"engine exported: {path}")
    print("Deploy: copy to /root/robotx_ws/models/ and update perception_node's "
          "engine_path param; preflight checks loadability.")
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data", help="dataset.yaml from prep_dataset.py")
    ap.add_argument("--base", default="buoy_v16.pt", help="base weights to fine-tune")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--export-only", action="store_true")
    ap.add_argument("--weights", help="weights for --export-only")
    args = ap.parse_args()

    if args.export_only:
        if not args.weights:
            ap.error("--export-only requires --weights")
        export_engine(args.weights)
        return

    if not args.data:
        ap.error("--data required for training")

    from ultralytics import YOLO
    model = YOLO(args.base)
    results = model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz,
                          patience=15)
    best = Path(results.save_dir) / "weights" / "best.pt"

    manifest = {
        "base": args.base, "data": args.data, "epochs": args.epochs,
        "imgsz": args.imgsz, "git_sha": git_sha(), "t": time.time(),
        "dataset_sha": None, "best": str(best),
    }
    ds_manifest = Path(args.data).parent / "manifest.json"
    if ds_manifest.exists():
        manifest["dataset_sha"] = json.loads(ds_manifest.read_text()).get("dataset_sha")
    (best.parent / "training_manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"best weights: {best}")

    print("\nNext: eval_regression.py against the held-out set, THEN export:")
    print(f"  python3 eval_regression.py --weights {best} --regression <dataset>/regression")
    print(f"  python3 train_buoy.py --export-only --weights {best}")


if __name__ == "__main__":
    main()
