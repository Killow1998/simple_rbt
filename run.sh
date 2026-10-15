#!/usr/bin/env bash
# Optional development convenience; both installed executables run directly.
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_directory="${SIMPLE_RBT_BUILD_DIR:-$project_root/build/desktop}"
if [[ "${1:-}" == --console ]]; then
  shift
  exec "$build_directory/simple-rbt-console" --worker "$build_directory/simple-rbt-sim" --assets "$project_root/assets" "$@"
fi
exec "$build_directory/simple-rbt-sim" --project-root "$project_root" "$@"
