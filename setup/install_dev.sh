#!/usr/bin/env bash
# ============================================================================
# setup/install_dev.sh — DEV MACHINE setup (Linux/macOS laptop, NOT the Jetson)
#
# There is very little to install: this repo's off-boat surface is the config
# guards and the param tooling. No ROS, no Docker, no hardware required. Node
# code is built and run on the Jetson, inside the `asv` container.
#
# Usage:   bash setup/install_dev.sh              # create .venv + install + verify
#          bash setup/install_dev.sh --no-verify  # skip the verification run
# ============================================================================
set -euo pipefail
cd "$(dirname "$0")/.."   # repo root

RUN_CHECKS=1
[[ "${1:-}" == "--no-verify" ]] && RUN_CHECKS=0

echo "== [1/3] Python virtual environment (.venv) =="
python3 -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
pip install --upgrade pip

echo "== [2/3] Dependencies (matches .github/workflows/ci.yml) =="
pip install pyyaml
# Optional, only for talking to a live vehicle from the laptop:
#   pip install pymavlink       (tools/scripts/param_guard.py --live)
# Optional, only for tools/oak_view.py against a RAW (uncompressed) image
# topic; the default compressed topic needs no image libraries at all:
#   pip install opencv-python numpy

if [[ "$RUN_CHECKS" == "1" ]]; then
  echo "== [3/3] Verify: config + param baseline guards =="
  python3 tools/scripts/check_config.py
else
  echo "== [3/3] skipped (--no-verify) =="
fi

echo
echo "Done. Activate with:  source .venv/bin/activate"
echo "Next steps: docs/SETUP_GUIDE.md."
