#!/usr/bin/env bash
# Build CPU-only and CUDA-capable variants without requiring a GPU.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
version=${1:-$(cat packaging/VERSION)}
[[ "$version" == "$(cat packaging/VERSION)" ]] || \
  { printf 'Version must match packaging/VERSION\n' >&2; exit 2; }
mkdir -p output/ci
export PATH="$HOME/.local/bin:$PATH"
./tools/setup-sdk.sh --minimal
python3 tools/prepare_visual_assets.py
python3 tools/setup-gui.py
cmake -S . -B build/ci-cpu -DCMAKE_BUILD_TYPE=Release \
  -DSIMPLE_RBT_CUDA=OFF -DSIMPLE_RBT_DESKTOP=OFF
cmake --build build/ci-cpu --parallel "${SPROBO_BUILD_JOBS:-2}"
ctest --test-dir build/ci-cpu --output-on-failure > output/ci/ctest-cpu.txt 2>&1 || \
  { cat output/ci/ctest-cpu.txt; exit 1; }
# Exercise the real C++ wire, not only mocked Go transports. Runtime GUI builds
# remain CGO_ENABLED=0; the race detector requires cgo only while testing.
go_version=$(python3 -c 'import json; print(json.load(open("third_party/mygo/manifest.json"))["go"]["version"])')
go="$root/build/toolchains/$go_version/go/bin/go"
g++ -std=c++17 -O2 -pthread -I include -I third_party tests/ipc/ros_fixture.cpp \
  ipc/codec.cpp ipc/socket.cpp ipc/client.cpp ipc/server.cpp src/robots.cpp \
  -o output/ci/ipc-fixture
(
  cd console
  export CGO_ENABLED=1 GOTOOLCHAIN=local
  export GOMODCACHE="$root/build/go-modules" GOCACHE="$root/build/go-cache"
  export SIMPLE_RBT_CPP_WORKER="$root/build/ci-cpu/simple-rbt-sim"
  export SIMPLE_RBT_CPP_IPC_FIXTURE="$root/output/ci/ipc-fixture"
  export LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
  "$go" -C "$root/.vendor/mygo" test -v ./internal/text -run '^TestPango'
  "$go" test -race -v ./...
  "$go" vet ./...
) > output/ci/go-race-vet.txt 2>&1 || \
  { cat output/ci/go-race-vet.txt; exit 1; }
export CUDACXX=${CUDACXX:-/usr/local/cuda-12.9/bin/nvcc}
export CUDAHOSTCXX=${CUDAHOSTCXX:-$(command -v "${CXX:-c++}")}
cmake -S . -B build/ci-release -DCMAKE_BUILD_TYPE=Release \
  -DSIMPLE_RBT_CUDA=ON -DSIMPLE_RBT_DESKTOP=ON
cmake --build build/ci-release --parallel "${SPROBO_BUILD_JOBS:-2}"
ctest --test-dir build/ci-release --output-on-failure > output/ci/ctest-release.txt 2>&1 || \
  { cat output/ci/ctest-release.txt; exit 1; }
python3 tools/package_deb.py --build-dir build/ci-release --version "$version"
python3 - "$version" <<'PY'
import json,sys
from pathlib import Path
report=json.loads(Path(f'output/packages/sprobo_{sys.argv[1]}_amd64.json').read_text())
assert report['cuda_enabled'], 'Release must preserve CUDA acceleration and CPU fallback'
PY
