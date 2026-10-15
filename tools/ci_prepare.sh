#!/usr/bin/env bash
# Dependencies for an ephemeral public Ubuntu 22.04 amd64 CI runner.
set -euo pipefail
[[ "${GITHUB_ACTIONS:-}" == true ]] || \
  { printf 'This dependency installer runs only on an ephemeral GitHub Actions runner.\n' >&2; exit 2; }
source /etc/os-release
[[ "$ID:$VERSION_ID:$(uname -m)" == ubuntu:22.04:x86_64 ]]
as_root() { if [[ $(id -u) == 0 ]]; then "$@"; else sudo -n "$@"; fi; }
as_root apt-get -o Acquire::Retries=3 update
as_root apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
  build-essential cmake git pkg-config curl ca-certificates python3 python3-pip \
  python3-venv ripgrep libegl1-mesa-dev libgl1-mesa-dev libglfw3-dev \
  libpng-dev libssl-dev libtinyxml2-dev libx11-dev \
  libgtk-3-0 libepoxy0 libpango-1.0-0 libpangocairo-1.0-0 fonts-wqy-microhei \
  xvfb xauth libxtst6
python3 -m pip install --user --disable-pip-version-check uv==0.12.19
if [[ -n "${GITHUB_PATH:-}" ]]; then printf '%s\n' "$HOME/.local/bin" >> "$GITHUB_PATH"; fi
temporary=$(mktemp -d)
trap 'rm -rf "$temporary"' EXIT
curl --fail --location --retry 3 \
  https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
  --output "$temporary/cuda-keyring.deb"
printf '%s  %s\n' d93190d50b98ad4699ff40f4f7af50f16a76dac3bb8da1eaaf366d47898ff8df \
  "$temporary/cuda-keyring.deb" | sha256sum --check
as_root dpkg -i "$temporary/cuda-keyring.deb"
as_root apt-get -o Acquire::Retries=3 update
# Compiler and runtime headers only: no driver package or GPU probe.
as_root apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
  cuda-nvcc-12-9 cuda-cudart-dev-12-9
if [[ -n "${GITHUB_PATH:-}" ]]; then printf '%s\n' /usr/local/cuda-12.9/bin >> "$GITHUB_PATH"; fi
