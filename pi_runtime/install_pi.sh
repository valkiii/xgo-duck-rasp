#!/usr/bin/env bash
# One-time setup on the Pi Zero 2 W (Raspberry Pi OS Lite, 64-bit).
#   ./install_pi.sh            install dependencies and the upstream runtime
#   ./install_pi.sh --service  also start the runtime automatically at boot
set -euo pipefail
cd "$(dirname "$0")"
DIR=$(pwd)

if [ "$(uname -m)" != "aarch64" ]; then
  echo "This needs the 64-bit Raspberry Pi OS (aarch64): onnxruntime has no 32-bit ARM wheels." >&2
  exit 1
fi

sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y python3-venv python3-dev git i2c-tools

# I2C on, at 400 kHz like the firmware's Wire.setClock(400000).
sudo raspi-config nonint do_i2c 0
CONFIG=/boot/firmware/config.txt; [ -f "$CONFIG" ] || CONFIG=/boot/config.txt
grep -q '^dtparam=i2c_arm_baudrate' "$CONFIG" || echo 'dtparam=i2c_arm_baudrate=400000' | sudo tee -a "$CONFIG" >/dev/null

# Serial (URT-2) and I2C access without sudo.
sudo usermod -aG dialout,i2c "$USER"

python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt
./fetch_upstream.sh

if [ "${1:-}" = "--service" ]; then
  sed -e "s|@USER@|$USER|g" -e "s|@DIR@|$DIR|g" xgo-duck.service | sudo tee /etc/systemd/system/xgo-duck.service >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl enable xgo-duck.service
  echo "Service installed: it starts at boot (sudo systemctl start xgo-duck to start now)."
fi

echo "Done. Reboot once so the I2C speed and group changes apply: sudo reboot"
