#!/usr/bin/env bash
# Fetch the upstream XGO Duck runtime (controller, policies, web UI) at a pinned commit.
# The Pi port runs those files unchanged; only the STM32 firmware is replaced.
set -euo pipefail
cd "$(dirname "$0")"
REPO=https://github.com/LuwuDynamics/xgoduck_runtime_arduino.git
COMMIT=${1:-9dc36787bc61b102c2da0f97c9c758e639e90ab8}
if [ ! -d upstream/.git ]; then
  git init -q upstream
  git -C upstream remote add origin "$REPO"
fi
git -C upstream fetch -q --depth 1 origin "$COMMIT"
git -C upstream checkout -q --detach FETCH_HEAD
echo "upstream runtime at $(git -C upstream rev-parse --short HEAD)"
