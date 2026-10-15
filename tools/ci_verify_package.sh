#!/usr/bin/env bash
# Run the installed artifact in a clean Ubuntu image without network or GPU.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
version=${1:-$(cat packaging/VERSION)}
evidence="$root/output/ci/installed"
mkdir -p "$evidence"
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic packaging/capture_x11.cpp \
  -o output/ci/capture-x11 -lX11 -lpng
docker_local=(env -u DOCKER_HOST -u DOCKER_CONTEXT -u DOCKER_TLS \
  -u DOCKER_TLS_VERIFY -u DOCKER_CERT_PATH docker --host=unix:///var/run/docker.sock)
"${docker_local[@]}" info --format '{{.ServerVersion}}'
build_args=(--build-arg HTTP_PROXY --build-arg HTTPS_PROXY \
  --build-arg http_proxy --build-arg https_proxy)
validation_image=${SPROBO_VALIDATION_IMAGE:-sprobo-package-check:ubuntu22.04}
if [[ -z "${SPROBO_VALIDATION_IMAGE:-}" ]]; then
  "${docker_local[@]}" build "${build_args[@]}" -f packaging/Dockerfile.validation \
    -t "$validation_image" packaging
fi
"${docker_local[@]}" image inspect "$validation_image" --format '{{.Id}}' \
  > "$evidence/validation-image.txt"
"${docker_local[@]}" run --rm --network none \
  --mount "type=bind,source=$root/output/packages,target=/packages,readonly" \
  --mount "type=bind,source=$root/packaging/verify_install.sh,target=/verify_install.sh,readonly" \
  --mount "type=bind,source=$root/output/ci/capture-x11,target=/capture-x11,readonly" \
  --mount "type=bind,source=$evidence,target=/evidence" \
  "$validation_image" /bin/sh /verify_install.sh \
  "/packages/sprobo_${version}_amd64.deb" /evidence /capture-x11
