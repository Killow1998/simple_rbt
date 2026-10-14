#!/usr/bin/env bash
# Prepare the locked SDK libraries, Python oracle and model assets in this checkout.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
# A staging worktree may borrow these directories read-only from another checkout.
if [[ -L .venv || -L .vendor ]]; then
  echo "SDK preparation requires checkout-local .venv and .vendor; use ./setup.sh --native-only for borrowed dependencies." >&2
  exit 1
fi
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
uv pip install --python .venv/bin/python -r reference/python/requirements.lock
uv pip install --python .venv/bin/python --no-deps -e "$vendor_path" -e reference/python
.venv/bin/python tools/bootstrap_rl_sar_assets.py --manifest assets/manifests/go2_manifest.json
.venv/bin/python tools/bootstrap_rl_sar_assets.py --manifest assets/manifests/go2w_manifest.json
echo "SDK and assets prepared."
