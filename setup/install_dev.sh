#!/usr/bin/env bash
# ============================================================================
# setup/install_dev.sh — DEV MACHINE setup (Linux/macOS laptop, NOT the Jetson)
#
# Installs everything needed to run the orchestrator (autoresearch harness),
# the kinematic-backend episode runner, and the full pytest suite — no ROS,
# no Docker, no hardware required. This is the entry point for new developers.
#
# Usage:   bash setup/install_dev.sh            # create .venv + install + test
#          bash setup/install_dev.sh --no-test  # skip the verification run
# ============================================================================
set -euo pipefail
cd "$(dirname "$0")/.."   # repo root

RUN_TESTS=1
[[ "${1:-}" == "--no-test" ]] && RUN_TESTS=0

echo "== [1/3] Python virtual environment (.venv) =="
python3 -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
pip install --upgrade pip

echo "== [2/3] Dependencies (orchestrator + tests; matches .github/workflows/ci.yml) =="
pip install pytest numpy pyyaml
# Optional, only needed for --proposer llm runs (Level 1 LLM / Level 2 rounds):
#   pip install anthropic
# Optional, only needed for tools/training/ (model retraining, not dev-loop):
#   pip install ultralytics

if [[ "$RUN_TESTS" == "1" ]]; then
  echo "== [3/3] Verify: full test suite + Gate G0 smoke episode =="
  python -m pytest orchestrator/tests tests -q
  python orchestrator/run_episode.py \
      --scenario orchestrator/scenarios/mission1_transit.json \
      --backend kinematic --seed 0 --out /tmp/ep_smoke.json
  python - <<'EOF'
import json
m = json.load(open("/tmp/ep_smoke.json"))
assert m["schema"] == "rx26-episode-metrics/1", m["schema"]
print("dev setup verified — G0 smoke OK, partial credit:",
      m["objective3"]["partial_credit"])
EOF
else
  echo "== [3/3] skipped (--no-test) =="
fi

echo
echo "Done. Activate with:  source .venv/bin/activate"
echo "Next steps: docs/SETUP_GUIDE.md §B (Run)."
