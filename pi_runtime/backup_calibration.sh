#!/usr/bin/env bash
# Copy the Pi's calibration (zero_pos.json) to ../calibration/backups/ with a timestamp.
# Read-only on the Pi (scp download only). Usage: ./backup_calibration.sh [user@host]
set -euo pipefail
cd "$(dirname "$0")"
TARGET=${1:-valkiii_zero@192.168.2.87}
DEST=../calibration/backups
STAMP=$(date +%Y%m%d-%H%M%S)
mkdir -p "$DEST"
if scp -q -o ConnectTimeout=8 -o BatchMode=yes "$TARGET:xgo-duck/upstream/data/zero_pos.json" "$DEST/zero_pos-$STAMP.json"; then
  echo "saved $DEST/zero_pos-$STAMP.json"
else
  rm -f "$DEST/zero_pos-$STAMP.json"   # local partial file only
  echo "no backup: could not copy ~/xgo-duck/upstream/data/zero_pos.json from $TARGET (file missing or Pi unreachable)" >&2
  exit 1
fi
