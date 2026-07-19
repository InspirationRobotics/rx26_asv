#!/usr/bin/env python3
"""prep_dataset — build the buoy retrain dataset from labeled footage sessions.

Input: session dirs produced by tools/scripts/collect_footage.py, labeled in
YOLO format (a `labels/` subdir per session with one .txt per frame, plus a
`classes.txt`).

Split rules (leakage guard): frames are captured at ~2 Hz, so adjacent frames
are near-duplicates — splitting at frame level would leak train into val and
inflate every metric. Splits are therefore BY SESSION:
  --holdout  sessions -> regression/ (the held-out G2 set; never trained on)
  --val      sessions -> val
  everything else     -> train

Output layout (ultralytics-standard):
  <out>/images/{train,val}/  <out>/labels/{train,val}/  <out>/regression/
  <out>/dataset.yaml         <out>/manifest.json  (sessions, counts, sha)

Usage:
  python3 prep_dataset.py --out ~/datasets/buoys_v1 \
      --sessions ~/footage/20260801 ~/footage/20260808 ~/footage/20260815 \
      --val 20260808 --holdout 20260815
"""
import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path


def frames_and_labels(session: Path):
    labels = session / "labels"
    if not labels.is_dir():
        sys.exit(f"ERROR: {session} has no labels/ dir — label it first")
    pairs = []
    for txt in sorted(labels.glob("*.txt")):
        if txt.name == "classes.txt":
            continue
        img = session / (txt.stem + ".jpg")
        if img.exists():
            pairs.append((img, txt))
    return pairs


def copy_split(pairs, out: Path, split: str, session_name: str):
    (out / "images" / split).mkdir(parents=True, exist_ok=True)
    (out / "labels" / split).mkdir(parents=True, exist_ok=True)
    for img, txt in pairs:
        stem = f"{session_name}_{img.stem}"
        shutil.copy2(img, out / "images" / split / f"{stem}.jpg")
        shutil.copy2(txt, out / "labels" / split / f"{stem}.txt")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    ap.add_argument("--sessions", nargs="+", required=True)
    ap.add_argument("--val", nargs="*", default=[],
                    help="session basenames for the val split")
    ap.add_argument("--holdout", nargs="*", default=[],
                    help="session basenames held out as the regression set")
    args = ap.parse_args()

    out = Path(args.out)
    sessions = [Path(s) for s in args.sessions]
    classes = None
    manifest = {"sessions": {}, "splits": {"train": 0, "val": 0, "regression": 0}}

    for sess in sessions:
        name = sess.name
        cfile = sess / "labels" / "classes.txt"
        if cfile.exists():
            cls = cfile.read_text().split()
            if classes is None:
                classes = cls
            elif classes != cls:
                sys.exit(f"ERROR: class list mismatch in {sess} — all sessions "
                         "must be labeled against the same classes.txt")
        pairs = frames_and_labels(sess)
        if name in args.holdout:
            split = "regression"
            dst = out / "regression" / name
            dst.mkdir(parents=True, exist_ok=True)
            for img, txt in pairs:
                shutil.copy2(img, dst / img.name)
                shutil.copy2(txt, dst / txt.name)
        else:
            split = "val" if name in args.val else "train"
            copy_split(pairs, out, split, name)
        manifest["sessions"][name] = {"split": split, "frames": len(pairs)}
        manifest["splits"][split] += len(pairs)
        print(f"{name}: {len(pairs)} frames -> {split}")

    if classes is None:
        sys.exit("ERROR: no classes.txt found in any session")
    if manifest["splits"]["regression"] == 0:
        print("WARNING: no --holdout session — G2 regression eval will have "
              "nothing to run against. Strongly recommended to hold one out.")

    yaml = (f"path: {out.resolve()}\ntrain: images/train\nval: images/val\n"
            f"names:\n" + "".join(f"  {i}: {c}\n" for i, c in enumerate(classes)))
    (out / "dataset.yaml").write_text(yaml)
    manifest["classes"] = classes
    manifest["dataset_sha"] = hashlib.sha256(yaml.encode()).hexdigest()[:12]
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"\ndataset at {out} — train {manifest['splits']['train']}, "
          f"val {manifest['splits']['val']}, "
          f"regression {manifest['splits']['regression']}")


if __name__ == "__main__":
    main()
