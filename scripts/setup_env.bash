#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

if [ ! -d ".venv" ]; then
  python3 -m venv --system-site-packages .venv
fi

source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo "[setup_env] Host .venv setup complete (numpy<2 + system ROS 2 Jazzy)."
