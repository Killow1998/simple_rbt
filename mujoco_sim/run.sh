#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$project_root/mujoco_sim_cpp/run.sh" "$@"
