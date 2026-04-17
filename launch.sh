#!/bin/bash
#
# Launch MindCloud World Fly simulator
#
# Usage:
#   ./launch.sh          - Firefox + NVIDIA RTX 5060 + WebHID bridge (recommended)
#   ./launch.sh chrome   - Chrome + Intel GPU (WebHID native, slower GPU)
#
# The HTTP server and WebHID bridge run as children of THIS shell. Press Ctrl+C
# here (or close this terminal) to stop both servers together. Any pre-existing
# server instances are killed on start so you never end up with stale processes.
#
# The browser is launched detached — closing this terminal does NOT close it.
#
# First-time setup (run once as root):
#   sudo bash setup_udev.sh
#

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HTTP_PORT=8080
HID_PORT=8766
BROWSER="${1:-firefox}"

HTTP_PID=""
HID_PID=""
_cleaned=0

cleanup() {
    # Idempotent: Ctrl+C fires INT trap which calls `exit`, which in turn fires
    # EXIT trap. We only want to run the teardown once.
    [[ "$_cleaned" = "1" ]] && return
    _cleaned=1
    echo ""
    echo "Shutting down servers..."
    # Polite SIGTERM first
    [[ -n "$HTTP_PID" ]] && kill -TERM "$HTTP_PID" 2>/dev/null || true
    [[ -n "$HID_PID"  ]] && kill -TERM "$HID_PID"  2>/dev/null || true
    # Small grace period, then force-kill anything still alive
    sleep 0.4
    [[ -n "$HTTP_PID" ]] && kill -KILL "$HTTP_PID" 2>/dev/null || true
    [[ -n "$HID_PID"  ]] && kill -KILL "$HID_PID"  2>/dev/null || true
    wait 2>/dev/null || true
    echo "All services stopped."
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

# ── Wipe any pre-existing server instances so this launch owns them ──────────
if pgrep -f "$SCRIPT_DIR/serve.py" > /dev/null 2>&1; then
    echo "  Stopping previous HTTP server..."
    pkill -f "$SCRIPT_DIR/serve.py" || true
fi
if pgrep -f "$SCRIPT_DIR/hid_server.py" > /dev/null 2>&1; then
    echo "  Stopping previous WebHID bridge..."
    pkill -f "$SCRIPT_DIR/hid_server.py" || true
fi
# Give the kernel a moment to release the ports
sleep 0.3

# ── HTTP server ──────────────────────────────────────────────────────────────
echo "Starting HTTP server on port $HTTP_PORT..."
python3 "$SCRIPT_DIR/serve.py" $HTTP_PORT &
HTTP_PID=$!
sleep 0.3
if ! kill -0 "$HTTP_PID" 2>/dev/null; then
    echo "ERROR: HTTP server failed to start" >&2
    exit 1
fi
echo "  HTTP server ready (pid $HTTP_PID)"

# ── WebHID bridge server ─────────────────────────────────────────────────────
echo "Starting WebHID bridge on port $HID_PORT..."
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libhidapi-hidraw.so.0 \
    python3 "$SCRIPT_DIR/hid_server.py" &
HID_PID=$!
sleep 0.3
if ! kill -0 "$HID_PID" 2>/dev/null; then
    echo "ERROR: WebHID bridge failed to start" >&2
    exit 1
fi
echo "  WebHID bridge ready (pid $HID_PID)"

# ── Browser ──────────────────────────────────────────────────────────────────
# Launch browser fully detached: nohup + disown. Closing this terminal must
# NOT close the browser (user might want to keep playing after stopping servers).
if [ "$BROWSER" = "chrome" ]; then
    # Chrome: native WebHID, Intel GPU (slower rendering)
    echo "Opening Chrome (native WebHID, Intel GPU)..."
    __GLX_VENDOR_LIBRARY_NAME=mesa nohup /opt/google/chrome/chrome "http://localhost:$HTTP_PORT" \
        >/dev/null 2>&1 &
    disown
else
    # Firefox: NVIDIA RTX 5060 via PRIME offload + WebHID via bridge polyfill
    echo "Opening Firefox (NVIDIA GPU + WebHID bridge)..."
    __NV_PRIME_RENDER_OFFLOAD=1 \
    __GLX_VENDOR_LIBRARY_NAME=nvidia \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    MOZ_WEBRENDER=1 \
        nohup firefox "http://localhost:$HTTP_PORT" >/dev/null 2>&1 &
    disown
fi

cat <<EOF

Simulator:     http://localhost:$HTTP_PORT
WebHID bridge: ws://localhost:$HID_PORT

To connect RC transmitter: Settings (Tab) → Connect HID Device

Press Ctrl+C here (or close this terminal) to stop both servers.
EOF

# Block until either server dies or the user interrupts. `wait -n` returns when
# any child exits; if one server crashes we also tear the other down.
wait -n "$HTTP_PID" "$HID_PID" 2>/dev/null || true
echo "A server exited unexpectedly."
exit 1
