#!/usr/bin/env bash
# Test real installed UI events and analytic RGB/depth through software EGL.
set -euo pipefail
[[ "${GITHUB_ACTIONS:-}" == true ]] || \
  { printf 'This installer check runs only on an ephemeral GitHub Actions runner. Use extracted binaries locally.\n' >&2; exit 2; }
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
version=${1:-$(cat packaging/VERSION)}
sudo -n apt-get install -y --no-install-recommends "./output/packages/sprobo_${version}_amd64.deb"
mkdir -p output/ci
export LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
xvfb-run -a -s '-screen 0 1600x1100x24' /bin/bash -c '
  exec python3 tools/test_console_native.py --display "$DISPLAY" \
    --console /usr/bin/sprobo --core /usr/bin/simple-rbt-sim \
    --assets /usr/share/simple-rbt/assets --backend cpu --output output/ci/native-input
'
xvfb-run -a -s '-screen 0 1600x1100x24' /bin/bash -c '
  exec python3 tools/test_console_workspace.py --display "$DISPLAY" \
    --console /usr/bin/sprobo --core /usr/bin/simple-rbt-sim \
    --assets /usr/share/simple-rbt/assets --output output/ci/workspace
'
python3 - <<'PY'
from pathlib import Path
import subprocess
sdk=next(Path('.venv/lib').glob('python*/site-packages/mujoco'))
library=next(sdk.glob('libmujoco.so.*'))
subprocess.run(['g++','-std=c++17','-O2','tests/ci_camera_fixture.cpp',
 '-I'+str(sdk/'include'),str(library),'-Wl,-rpath,'+str(library.parent.resolve()),
 '-o','output/ci/camera-fixture'],check=True)
PY
output/ci/camera-fixture output/ci/camera
build/ci-release/camera_render_consistency > output/ci/camera/render-consistency.txt
build/ci-release/camera_snapshot > output/ci/camera/snapshot-consistency.txt
build/ci-release/sensor_devices --render > output/ci/camera/sensor-devices.txt
build/ci-release/camera_oracle output/ci/camera/model.mjb output/ci/camera/state.bin \
  output/ci/camera/pose.bin output/ci/camera/render 80 60 160 80 64 90
.venv/bin/python - <<'PY'
import json
from pathlib import Path
import numpy as np
base=Path('output/ci/camera')
depth=np.fromfile(base/'render-perspective-depth.f32',dtype=np.float32).reshape(60,80)
rgb=np.fromfile(base/'render-perspective-rgb.u8',dtype=np.uint8).reshape(60,80,3)
value=float(depth[30,40])
assert abs(value-2)<.001, f'Analytic wall center depth must be 2m; got {value}'
assert int(rgb[30,40,0])>int(rgb[30,40,1]), 'Known red wall must dominate green'
assert not np.isnan(depth).any() and not (depth<0).any()
meta=json.loads((base/'render-metadata.json').read_text())
assert meta['physical_state_and_model_visuals_unchanged'] and meta['repeat_depth_identical']
(base/'analytic-check.json').write_text(json.dumps({'passed':True,'expected_depth_m':2,
 'actual_depth_m':value,'rgb_center':rgb[30,40].tolist(),'oracle':meta},indent=2)+'\n')
PY
sudo -n apt-get purge -y sprobo
