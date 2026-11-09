#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
profile=${1:-up}
if [[ "$profile" == --help || "$profile" == -h ]]; then
  echo 'Usage: run_supermimic.sh [PROFILE] [--check]'
  echo 'Profiles: flat, up, down, slope, rubble, incline-up, incline-down, up-l1..up-l5, down-l1..down-l5, mapping'
  echo 'SIMPLE_RBT_BUNDLE selects a portable bundle; SIMPLE_RBT_BUILD selects a development build.'
  echo 'W/S forward/back, A/D lateral, Q/E yaw. Release, Space or manual focus loss stops input.'
  echo 'vx/vy/yaw rate only; no height or pitch commands / 不支持高度或俯仰目标。'
  exit 0
fi
if (( $# > 2 )) || [[ ${2:-} != '' && ${2:-} != --check ]]; then
  echo 'Use --help for supported arguments.' >&2; exit 2
fi
bundle=${SIMPLE_RBT_BUNDLE:-${XDG_DATA_HOME:-$HOME/.local/share}/simple-rbt/bundles/go2w-supermimic}
build=${SIMPLE_RBT_BUILD:-$project_root/build/integration}
if [[ -x "$build/simple-rbt-console" ]]; then
  console="$build/simple-rbt-console"; core="$build/simple-rbt-sim"
else
  console=$(command -v sprobo); core=$(command -v simple-rbt-sim)
fi
args=(--worker "$core" --bundle "$bundle" --profile "$profile")
if [[ ${2:-} == --check ]]; then args+=(--check-dependencies); fi
exec "$console" "${args[@]}"
