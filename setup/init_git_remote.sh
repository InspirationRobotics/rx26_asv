#!/usr/bin/env bash
# ============================================================================
# setup/init_git_remote.sh — one-time git init + remote wiring for this repo
#
# Idempotent: safe to re-run; it only creates what is missing.
#
# Usage:
#   bash setup/init_git_remote.sh                                  # init only
#   bash setup/init_git_remote.sh git@github.com:ORG/rx26_asv.git   # + remote
#   bash setup/init_git_remote.sh <url> --push                     # + first push
#
# NOTE (merge path): the live boat repo is github.com/chrismartin018/robotx_2026
# (private, lives on the Jetson). If this checkout is destined to MERGE into
# that repo rather than stand alone, do NOT push here — follow the rsync merge
# procedure in README_PHASE0.md instead.
# ============================================================================
set -euo pipefail
cd "$(dirname "$0")/.."   # repo root

REMOTE_URL="${1:-}"
DO_PUSH="${2:-}"

echo "== [1/3] git init (branch: main) =="
if [[ -d .git ]]; then
  echo "   already a git repo — skipping init"
else
  git init -b main
fi

echo "== [2/3] initial commit =="
if git rev-parse HEAD >/dev/null 2>&1; then
  echo "   history exists — skipping initial commit"
else
  git add -A
  git commit -m "Initial commit: RX26 Phase 0-5 scaffold (interfaces, orchestrator, nodes, tools, setup, docs)"
fi

echo "== [3/3] remote =="
if [[ -z "$REMOTE_URL" ]]; then
  echo "   no URL given. Wire one later with:"
  echo "     git remote add origin <url> && git push -u origin main"
  exit 0
fi
if git remote get-url origin >/dev/null 2>&1; then
  echo "   'origin' already set to: $(git remote get-url origin) — not changing"
else
  git remote add origin "$REMOTE_URL"
  echo "   origin -> $REMOTE_URL"
fi
if [[ "$DO_PUSH" == "--push" ]]; then
  git push -u origin main
else
  echo "   push when ready:  git push -u origin main"
fi
