#!/usr/bin/env bash
# Isaac Sim 接收端启动脚本 / Isaac Sim receiver launcher.
# 使用 Isaac 官方 python.sh 运行 isaacsim_joint_receiver.py。
# Run isaacsim_joint_receiver.py via the official Isaac Sim python.sh.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_ROOT="${ISAACSIM_ROOT:-/home/seeed/IsaacSim/_build/linux-x86_64/release}"
ISAACSIM_PYTHON="${ISAACSIM_ROOT}/python.sh"

if [[ ! -f "${ISAACSIM_PYTHON}" ]]; then
  echo "[error] 未找到 Isaac Sim python.sh: ${ISAACSIM_PYTHON} / Isaac Sim python.sh not found: ${ISAACSIM_PYTHON}" >&2
  echo "[hint] 请设置 ISAACSIM_ROOT 环境变量指向 Isaac Sim 运行目录 / please set ISAACSIM_ROOT to your Isaac Sim runtime directory" >&2
  exit 1
fi

if [[ -z "${REBOT_ASSET_ROOT:-}" ]]; then
  echo "[error] 未设置 REBOT_ASSET_ROOT / REBOT_ASSET_ROOT is not set" >&2
  echo "[hint] 请指向包含 usd/RS-rebot-dev-arm/ 的目录，例如 reBot-Isaacsim 仓库根目录 /" >&2
  echo "[hint] point it at a directory containing usd/RS-rebot-dev-arm/, e.g. the reBot-Isaacsim repo root" >&2
  echo "[hint] export REBOT_ASSET_ROOT=/path/to/reBot-Isaacsim" >&2
  exit 1
fi

exec bash "${ISAACSIM_PYTHON}" "${SCRIPT_DIR}/isaacsim_joint_receiver.py" "$@"
