#!/usr/bin/env bash
# Prepares assets/SDK libraries in this checkout, then builds the C++ runtime.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
revision=e6b879df4b96daaed30ef10fe02650d29048fa9b
vendor_path="$project_root/.vendor/MuJoCo-LiDAR"
if [[ ! -d "$vendor_path" ]]; then
  mkdir -p "$vendor_path"
  git -C "$vendor_path" init
  git -C "$vendor_path" remote add origin https://github.com/discoverse-dev/MuJoCo-LiDAR.git
  git -C "$vendor_path" fetch --depth 1 origin "$revision"
  git -C "$vendor_path" checkout --detach FETCH_HEAD
fi
if [[ "$(git -C "$vendor_path" rev-parse HEAD)" != "$revision" ]]; then
  echo "MuJoCo-LiDAR version differs from the tested revision: $revision" >&2
  exit 1
fi
if [[ ! -x .venv/bin/python ]]; then
  uv venv --python /usr/bin/python3.10 .venv
fi
uv pip install --python .venv/bin/python -r mujoco_sim/requirements.lock
uv pip install --python .venv/bin/python --no-deps -e "$vendor_path" -e mujoco_sim
.venv/bin/python mujoco_sim/scripts/bootstrap_rl_sar_assets.py --manifest mujoco_sim/assets/go2_manifest.json
.venv/bin/python mujoco_sim/scripts/bootstrap_rl_sar_assets.py --manifest mujoco_sim/assets/go2w_manifest.json
./mujoco_sim_cpp/setup.sh
echo "Ready: ./mujoco_sim/run.sh --robot go2 --cmd .3 0 0 --duration 10"
