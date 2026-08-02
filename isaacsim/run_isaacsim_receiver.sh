#!/usr/bin/env bash
# Isaac Sim receiver launcher.
# Run isaacsim_joint_receiver.py via the official Isaac Sim python.sh.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_ROOT="${ISAACSIM_ROOT:-/home/seeed/IsaacSim/_build/linux-x86_64/release}"
ISAACSIM_PYTHON="${ISAACSIM_ROOT}/python.sh"

if [[ ! -f "${ISAACSIM_PYTHON}" ]]; then
  echo "[error] Isaac Sim python.sh not found: ${ISAACSIM_PYTHON}" >&2
  echo "[hint] please set ISAACSIM_ROOT to your Isaac Sim runtime directory" >&2
  exit 1
fi

if [[ -z "${REBOT_ASSET_ROOT:-}" ]]; then
  echo "[error] REBOT_ASSET_ROOT is not set" >&2
  echo "[hint] point it at a directory containing usd/RS-rebot-dev-arm/, e.g. the reBot-Isaacsim repo root" >&2
  echo "[hint] export REBOT_ASSET_ROOT=/path/to/reBot-Isaacsim" >&2
  exit 1
fi

exec bash "${ISAACSIM_PYTHON}" "${SCRIPT_DIR}/isaacsim_joint_receiver.py" "$@"
