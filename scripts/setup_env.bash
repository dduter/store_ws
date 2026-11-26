#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "[setup_env] Workspace: ${WS_DIR}"

if [ ! -d "${WS_DIR}/.venv" ]; then
  python3 -m venv --system-site-packages "${WS_DIR}/.venv"
fi

# shellcheck disable=SC1091
source "${WS_DIR}/.venv/bin/activate"
pip install --upgrade pip
pip install -r "${WS_DIR}/requirements.txt"

echo "[setup_env] Done. Activate with: source ${WS_DIR}/.venv/bin/activate"
