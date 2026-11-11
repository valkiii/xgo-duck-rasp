#!/usr/bin/env bash
# Copy this folder to the Pi. Never deletes anything there and never touches
# upstream/ (it holds data/zero_pos.json, your calibration).
#   ./deploy.sh pi@xgoduck.local
set -euo pipefail
cd "$(dirname "$0")"
TARGET=${1:?usage: ./deploy.sh user@host}
rsync -az --exclude .venv --exclude upstream --exclude __pycache__ --exclude .pytest_cache \
  ./ "$TARGET:xgo-duck/"
echo "Copied to $TARGET:~/xgo-duck. First time: ssh $TARGET 'cd xgo-duck && ./install_pi.sh'"
