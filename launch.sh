#!/bin/bash
#
# Launch MindCloud World Fly simulator
#
# Usage:
#   ./launch.sh          - Firefox + NVIDIA RTX 5060 + WebHID bridge (recommended)
#   ./launch.sh chrome   - Chrome + Intel GPU (WebHID native, slower GPU)
#
# First-time setup (run once as root):
#   sudo bash setup_udev.sh
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HTTP_PORT=8080
HID_PORT=8766
BROWSER="${1:-firefox}"

# ── HTTP server ──────────────────────────────────────────────────────────────
if ! pgrep -f "serve.py" > /dev/null; then
    echo "Starting HTTP server on port $HTTP_PORT..."
    python3 "$SCRIPT_DIR/serve.py" $HTTP_PORT &
    sleep 1
    echo "  HTTP server ready"
else
    echo "  HTTP server already running"
fi

# ── WebHID bridge server ─────────────────────────────────────────────────────
# Kill stale bridge process if port is not actually listening
if pgrep -f "hid_server.py" > /dev/null && ! ss -tlnp 2>/dev/null | grep -q ":$HID_PORT "; then
    echo "  Killing stale WebHID bridge process..."
    pkill -f "hid_server.py"
    sleep 0.5
fi

if ! ss -tlnp 2>/dev/null | grep -q ":$HID_PORT "; then
    echo "Starting WebHID bridge on port $HID_PORT..."
    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libhidapi-hidraw.so.0 python3 "$SCRIPT_DIR/hid_server.py" &
    sleep 1
    echo "  WebHID bridge ready"
else
    echo "  WebHID bridge already running"
fi

# ── Browser ──────────────────────────────────────────────────────────────────
if [ "$BROWSER" = "chrome" ]; then
    # Chrome: native WebHID, Intel GPU (slower rendering)
    # __GLX_VENDOR_LIBRARY_NAME=mesa fixes GLVND vendor selection in on-demand mode
    echo "Opening Chrome (native WebHID, Intel GPU)..."
    __GLX_VENDOR_LIBRARY_NAME=mesa /opt/google/chrome/chrome "http://localhost:$HTTP_PORT" &
else
    # Firefox: NVIDIA RTX 5060 via PRIME offload + WebHID via bridge polyfill
    # __EGL_VENDOR_LIBRARY_FILENAMES: route EGL calls to NVIDIA (driver 590+ supports EGL+PRIME)
    # MOZ_WEBRENDER=1: force WebRender GPU compositor (bypass Firefox driver blocklist)
    echo "Opening Firefox (NVIDIA GPU + WebHID bridge)..."
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json MOZ_WEBRENDER=1 firefox "http://localhost:$HTTP_PORT" &
fi

echo ""
echo "Simulator: http://localhost:$HTTP_PORT"
echo "WebHID bridge: ws://localhost:$HID_PORT"
echo ""
echo "To connect RC transmitter: Settings (Tab) → Connect HID Device"
