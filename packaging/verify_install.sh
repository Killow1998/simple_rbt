#!/bin/sh
# Independent install/uninstall acceptance in a fresh Ubuntu validation image.
set -eu
package=${1:?Pass the mounted .deb path}
evidence=${2:-/tmp/sprobo-package-check}
capture_tool=${3:?Pass the mounted preparation-only X11 capture executable}
mkdir -p "$evidence"
dpkg -i "$package" > "$evidence/install.log"
dpkg-query -W sprobo > "$evidence/package-version.txt"
dpkg-query -L sprobo > "$evidence/installed-files.txt"
desktop-file-validate /usr/share/applications/sprobo.desktop
for app in simple-rbt-sim sprobo simple-rbt-tutorial-config; do
    if ! ldd "/usr/bin/$app" > "$evidence/$app-libraries.txt" 2>&1; then
        if readelf -d "/usr/bin/$app" | grep -q '(NEEDED)'; then
            cat "$evidence/$app-libraries.txt" >&2
            exit 1
        fi
        printf 'Static executable; no ELF shared library dependency\n' > "$evidence/$app-libraries.txt"
    fi
    if grep -q 'not found' "$evidence/$app-libraries.txt"; then
        cat "$evidence/$app-libraries.txt" >&2
        exit 1
    fi
    if grep -E '/home/|\.venv|libpython|libroscpp|librclcpp' "$evidence/$app-libraries.txt"; then
        echo "Host runtime leaked a source/Python/ROS dependency" >&2
        exit 1
    fi
done
test ! -d /home/user/Projects/simple_rbt
test ! -d /opt/ros
if command -v python3 >/dev/null 2>&1; then
    echo "Validation image unexpectedly contains Python" >&2
    exit 1
fi
mkdir -p /tmp/sprobo-user/config /tmp/sprobo-user/state \
    /tmp/sprobo-user/cache /tmp/sprobo-user/runtime
chown -R 65534:65534 /tmp/sprobo-user
chmod 0700 /tmp/sprobo-user/runtime
cd /tmp
setpriv --reuid=65534 --regid=65534 --clear-groups \
    /usr/bin/simple-rbt-sim --help > "$evidence/sim-help.txt"
/usr/bin/simple-rbt-sim --list-robots > "$evidence/robot-registry.json"
robots=$(jq -er '.[].name' "$evidence/robot-registry.json")
jq -e 'length > 0 and ((map(.name) | length) == (map(.name) | unique | length))' \
    "$evidence/robot-registry.json" >/dev/null
for robot in $robots; do
    setpriv --reuid=65534 --regid=65534 --clear-groups \
        /usr/bin/simple-rbt-sim --robot "$robot" --sensors physics --backend cpu \
        --settle 0 --duration 0.05 --summary-only \
        --output "/tmp/sprobo-user/physics-$robot" \
        > "$evidence/physics-$robot.txt" 2> "$evidence/physics-$robot-stderr.txt"
done
setpriv --reuid=65534 --regid=65534 --clear-groups \
    env XDG_CONFIG_HOME=/tmp/sprobo-user/config XDG_STATE_HOME=/tmp/sprobo-user/state \
    XDG_CACHE_HOME=/tmp/sprobo-user/cache XDG_RUNTIME_DIR=/tmp/sprobo-user/runtime \
    /usr/bin/sprobo --check-dependencies > "$evidence/dependencies.json"
jq -e '.ready == true' "$evidence/dependencies.json" >/dev/null
setpriv --reuid=65534 --regid=65534 --clear-groups \
    env XDG_CONFIG_HOME=/tmp/sprobo-user/config \
    XDG_STATE_HOME=/tmp/sprobo-user/state XDG_CACHE_HOME=/tmp/sprobo-user/cache \
    XDG_RUNTIME_DIR=/tmp/sprobo-user/runtime \
    /usr/bin/sprobo --smoke-test \
    > "$evidence/console-smoke.txt" 2> "$evidence/console-smoke-stderr.txt"
cat > /tmp/sprobo-user/package-session.json <<'JSON'
{"robot":"ranger_mini","backend":"cpu","camera_hz":2,
 "sensors":{"mid360":false,"camera":false,"depth":false,"panorama":false},
 "adapters":{"ros1":false,"ros2":false}}
JSON
setpriv --reuid=65534 --regid=65534 --clear-groups \
    env XDG_CONFIG_HOME=/tmp/sprobo-user/config XDG_STATE_HOME=/tmp/sprobo-user/state \
    XDG_CACHE_HOME=/tmp/sprobo-user/cache XDG_RUNTIME_DIR=/tmp/sprobo-user/runtime \
    LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless \
    timeout 45s xvfb-run -a -s '-screen 0 1360x1000x24' \
    /bin/sh -c '
      /usr/bin/sprobo --config /tmp/sprobo-user/package-session.json &
      console_pid=$!
      sleep 12
      "$1" /tmp/sprobo-user/installed-native-console.png
      wait "$console_pid"
    ' native-window-check "$capture_tool" \
    > "$evidence/native-console.txt" 2> "$evidence/native-console-stderr.txt"
test -s /tmp/sprobo-user/installed-native-console.png
cp /tmp/sprobo-user/installed-native-console.png "$evidence/installed-native-console.png"
chmod 0644 "$evidence/installed-native-console.png"
for comm in /proc/[0-9]*/comm; do
    if [ "$(cat "$comm" 2>/dev/null || true)" = simple-rbt-sim ]; then
        echo 'Simulator worker survived normal native window close' >&2
        exit 1
    fi
done
printf 'preserve-user-settings\n' > /tmp/sprobo-user/config/user-state-marker
dpkg --purge sprobo > "$evidence/uninstall.log"
test ! -e /usr/bin/simple-rbt-sim
test ! -e /usr/bin/sprobo
test ! -e /usr/bin/simple-rbt-tutorial-config
test ! -e /usr/share/applications/sprobo.desktop
test ! -d /usr/share/simple-rbt
test ! -d /usr/lib/simple-rbt
test -f /tmp/sprobo-user/config/user-state-marker
printf 'install, non-root native window, all registered CPU models, dependency isolation and uninstall passed\n'
