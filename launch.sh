#!/bin/bash
#
# Launch MindCloud World Fly simulator
#
# Usage:
#   ./launch.sh          - Chrome + NVIDIA GPU (recommended)
#   ./launch.sh no-open  - start only the HTTP server
#
# The HTTP server runs as a child of THIS shell. Press Ctrl+C here (or close
# this terminal) to stop it. Any pre-existing server instance is killed on
# start so you never end up with a stale process.
#
# The browser is launched detached — closing this terminal does NOT close it.
#

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HTTP_PORT=8080
BROWSER_MODE="${1:-chrome}"

HTTP_PID=""
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
    # Small grace period, then force-kill anything still alive
    sleep 0.4
    [[ -n "$HTTP_PID" ]] && kill -KILL "$HTTP_PID" 2>/dev/null || true
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

# ── Browser ──────────────────────────────────────────────────────────────────
# Launch browser fully detached: nohup + disown. Closing this terminal must
# NOT close the browser (user might want to keep playing after stopping servers).
if [ "$BROWSER_MODE" != "no-open" ]; then
    CHROME_BIN="${CHROME_BIN:-}"
    if [ -z "$CHROME_BIN" ]; then
        for candidate in /opt/google/chrome/chrome google-chrome google-chrome-stable chromium chromium-browser; do
            if command -v "$candidate" >/dev/null 2>&1; then
                CHROME_BIN="$(command -v "$candidate")"
                break
            elif [ -x "$candidate" ]; then
                CHROME_BIN="$candidate"
                break
            fi
        done
    fi

    if [ -z "$CHROME_BIN" ]; then
        echo "ERROR: Chrome/Chromium not found. Install google-chrome-stable or chromium." >&2
        exit 1
    fi

    echo "Opening Chrome with NVIDIA GPU acceleration..."
    __NV_PRIME_RENDER_OFFLOAD=1 \
    __GLX_VENDOR_LIBRARY_NAME=nvidia \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
        nohup "$CHROME_BIN" \
            --enable-gpu-rasterization \
            --ignore-gpu-blocklist \
            "http://localhost:$HTTP_PORT" >/dev/null 2>&1 &
    disown
else
    echo "Browser launch skipped (-- no-open)."
fi

cat <<EOF

Simulator:     http://localhost:$HTTP_PORT

Gamepad:       Chrome Gamepad API
RC HID:        Chrome WebHID (Settings / Tab → Connect HID)

Press Ctrl+C here (or close this terminal) to stop the server.
EOF

wait "$HTTP_PID" 2>/dev/null || true
echo "HTTP server exited."
exit 1
