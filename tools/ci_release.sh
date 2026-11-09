#!/usr/bin/env bash
# Called only after all build and installed-package checks passed.
set -euo pipefail
version=${1:?Pass the tested package version}
commit=${2:?Pass the tested Git commit}
directory=${3:?Pass the downloaded artifact directory}
tag="v$version"
git check-ref-format "refs/tags/$tag"
if gh release view "$tag" >/dev/null 2>&1; then
  printf 'Release %s already exists; immutable assets are preserved.\n' "$tag"
  exit 0
fi
if git show-ref --verify --quiet "refs/tags/$tag"; then
  [[ $(git rev-parse "$tag^{commit}") == "$commit" ]] || \
    { printf 'Existing tag does not identify the tested commit\n' >&2; exit 1; }
fi
package="sprobo_${version}_amd64.deb"
(cd "$directory" && sha256sum --check "$package.sha256")
notes=$(mktemp)
trap 'rm -f "$notes"' EXIT
cat > "$notes" <<EOF
SpRobo launcher for the simple_rbt framework — Ubuntu 22.04 amd64.
SpRobo 启动器与 simple_rbt 框架，支持 Ubuntu 22.04 amd64。

Install / 安装：sudo apt install ./$package
Launch / 启动：sprobo
Headless framework / 无头框架：simple-rbt-sim

The release includes the arm and quadrotor, scene editing, independent ROS 1/ROS 2 adapters, and a hardware depth-sampling correction. Light/dark appearance and English/Simplified Chinese are selectable and remembered.
本版本包含机械臂、四旋翼、场景编辑、独立 ROS 1／ROS 2 适配器，以及硬件深度采样修复。可切换并保存浅色／深色与英文／简体中文。

The optional Go2W SuperMimic controller requires a separately prepared model and policy. Its tracked mode uses ideal simulated velocity feedback; very-low-speed tracking remains approximate.
可选 Go2W SuperMimic 控制器需要单独准备模型和策略。tracked 模式使用理想仿真速度反馈；极低速跟踪仍有误差。

CUDA acceleration and automatic CPU fallback are included. CI builds both CPU-only and CUDA-capable variants, then verifies installation, all registered robots on CPU, the native window, normal shutdown and removal in a clean Ubuntu container. A GPU-free CI runner does not validate physical GPU performance.
包含 CUDA 加速和自动 CPU 回退。CI 检查两种构建，并在干净 Ubuntu 容器中验收安装、全部注册机器人、原生窗口、正常关闭及卸载；无显卡 runner 不作为宿主 GPU 性能验收。

Tested source commit: $commit
EOF
gh release create "$tag" "$directory/$package" "$directory/$package.sha256" \
  "$directory/sprobo_${version}_amd64.json" --target "$commit" \
  --title "SpRobo $tag · simple_rbt" --notes-file "$notes"
