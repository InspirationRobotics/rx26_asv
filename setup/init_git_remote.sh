#!/usr/bin/env bash
# ============================================================================
# setup/init_git_remote.sh — one-time git init + remote wiring for this repo
#
# Idempotent: safe to re-run; it only creates what is missing.
#
# Usage:
#   bash setup/init_git_remote.sh                                  # init only
#   bash setup/init_git_remote.sh git@github.com:InspirationRobotics/rx26_asv.git
#   bash setup/init_git_remote.sh <url> --push                     # + first push
#
# Canonical remote: github.com/InspirationRobotics/rx26_asv (private).
# This repo is standalone — it is not a fork and does not merge into any other
# tree. On the Jetson it is cloned to `~/robotx_ws/src/rx26_asv` — one package
# source inside a colcon workspace, NOT the workspace root. This script is only
# for bootstrapping a fresh checkout that has no git yet.
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
  git commit -m "Initial commit: rx26_asv (interfaces, nodes, tools, setup, docs)"
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
