#!/usr/bin/env bash
# restart_all.sh - force-restart all MindCloud_World_Fly containers
#
#   Force-stop and rebuild the following containers (Docker images are NOT rebuilt):
#     1. DA360 depth service    mindcloud-da360-api    http://127.0.0.1:5688
#     2. YOPO avoidance backend  mindcloud-yopo-api     http://127.0.0.1:5689
#     3. Main flight process     google-tiles-flight     http://127.0.0.1:8080
#
#   Usage:
#     ./restart_all.sh                    # restart all three containers
#     ./restart_all.sh --no-main          # restart DA360 + YOPO only (keep main flight)
#     ./restart_all.sh --no-da360         # restart YOPO + main flight only (keep DA360)
#     ./restart_all.sh --no-yopo          # restart DA360 + main flight only (keep YOPO)
#
#   Notes:
#     - Both backend containers mount their server scripts read-only, so changes to
#       the Python take effect after restarting the container (no image rebuild).
#     - The main flight container runs with --detach; the frontend JS is mounted
#       read-only, so hard-refresh the browser (Ctrl+F5) to load the latest frontend.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── Arguments ──
DO_DA360=1
DO_YOPO=1
DO_MAIN=1
for arg in "$@"; do
    case "$arg" in
        --no-da360) DO_DA360=0 ;;
        --no-yopo)  DO_YOPO=0 ;;
        --no-main)  DO_MAIN=0 ;;
        -h|--help)
            grep '^#' "$0" | sed -n '2,/^ *Usage:/p' | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Unknown argument: $arg (use --help for usage)" >&2; exit 1 ;;
    esac
done

DA360_NAME="mindcloud-da360-api"
YOPO_NAME="mindcloud-yopo-api"
MAIN_NAME="google-tiles-flight"

echo "==============================================="
echo " MindCloud_World_Fly one-shot restart"
echo " Restarting: DA360=$DO_DA360 YOPO=$DO_YOPO main flight=$DO_MAIN"
echo "==============================================="

# ── Stop the container and wait for it to release its port ──
stop_wait() {
    local name="$1"
    if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "$name"; then
        echo "  Stopping $name ..."
        docker rm -fv "$name" >/dev/null 2>&1 || true
        for _ in $(seq 1 20); do
            docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$name" || break
            sleep 0.5
        done
    fi
}

wait_health() {
    local url="$1" timeout="$2" label="$3"
    local waited=0
    while [ "$waited" -lt "$timeout" ]; do
        if curl -s -m 2 "$url" >/dev/null 2>&1; then
            echo "  [OK] $label ready"
            return 0
        fi
        sleep 2
        waited=$((waited + 2))
    done
    echo "  [WARN] $label not ready after $timeout seconds" >&2
    return 1
}

# ── 1. DA360 ──
if [ "$DO_DA360" = "1" ]; then
    echo "[1/3] Restarting the DA360 depth service ..."
    stop_wait "$DA360_NAME"
    DA360_DETACH=1 DA360_NO_WARMUP=1 ./scripts/start_da360_api.sh large >/tmp/restart_da360.log 2>&1 &
    DA360_PID=$!
    wait_health "http://127.0.0.1:5688/health" 120 DA360 || true
fi

# ── 2. YOPO ──
if [ "$DO_YOPO" = "1" ]; then
    echo "[2/3] Restarting the YOPO avoidance backend ..."
    stop_wait "$YOPO_NAME"
    # Deploys YOPO_40 by default (wc=8, learning-based avoidance) with TensorRT acceleration
    # (YOPO_USE_TRT=1). If the engine is missing it still starts normally (falling back to
    # PyTorch eager) and prints a hint on how to generate it.
    # To switch models, point YOPO_MODEL_PATH below at the corresponding checkpoint.
    # Default cruise speed YOPO_VELOCITY=15.0 (vel_max ~= 15 m/s, acc_max ~= 37.5 m/s^2):
    # measured comparison -- 8.0 cruises at only ~7 m/s (too slow, the source of the 0.5 m/s the
    # user reported); 16.0 has its trajectory endpoints amplified by the network to ~76 m/s (the
    # real culprit behind the "suddenly flying very fast" bursts); 15.0 cruises at ~12-15 m/s in
    # the middle segments with ~16-19 m/s endpoints, fast yet controllable. On the client side
    # yopoPosErrMaxV was raised to 15 (unlocking the position loop's cruise ceiling) and
    # yopoMaxSpd=15 clamps as a second safety net against lurching.
    # Set YOPO_VELOCITY=16~18 for more speed (mind the burst risk), or drop it to 12 for more
    # stability.
    # YOPO_CTRL_TIME_SCALE: the "fast forward" factor for ctrl_time after the command continuity
    # fix. 1 = follow the network-planned speed exactly (vel_max ~= 15 -> cruise <= 15 m/s).
    # Note: SCALE > 1 pushes the commanded speed to vel_max*SCALE (e.g. 2 -> ~30 m/s); although
    # YOPO_SPEED_CAP=15 clamps it back to 15, it makes the planned position run ahead and leaves
    # the drone permanently lagging behind the trajectory, so it stays at 1 by default. Speed
    # requests should be met by tuning YOPO_VELOCITY, not by inflating SCALE.
    TRT_ENGINE="$SCRIPT_DIR/asset/yopo-trt/yopo_trt.pth"
    if [ ! -f "$TRT_ENGINE" ]; then
        echo "  [WARN] TensorRT engine not found: $TRT_ENGINE" >&2
        echo "         YOPO will run slowly on PyTorch eager. To build the engine:" >&2
        echo "         docker exec $YOPO_NAME python /opt/mindcloud-yopo/scripts/yopo_trt_transfer.py" >&2
    fi
    YOPO_DETACH=1 \
        YOPO_MODEL_PATH="$SCRIPT_DIR/third_party/yopo/saved/YOPO_40/epoch50.pth" \
        YOPO_USE_TRT=1 \
        YOPO_VELOCITY=15.0 \
        YOPO_CTRL_TIME_SCALE=1 \
        ./scripts/start_yopo_api.sh >/tmp/restart_yopo.log 2>&1 &
    YOPO_PID=$!
    wait_health "http://127.0.0.1:5689/yopo/status" 120 YOPO || true
fi

# ── 3. Main flight ──
if [ "$DO_MAIN" = "1" ]; then
    echo "[3/3] Restarting the main flight process ..."
    stop_wait "$MAIN_NAME"
    DETACH=1 \
        nohup bash "$SCRIPT_DIR/launch.sh" --detach >/tmp/restart_main.log 2>&1 &
    sleep 4
    wait_health "http://127.0.0.1:8080/" 60 "main flight" || true
fi

# Clean up background logging processes (if anything is following in the foreground)
wait 2>/dev/null || true

echo "==============================================="
echo " All ready:"
docker ps --format '  {{.Names}}\t{{.Status}}\t{{.Ports}}' \
    | grep -E "$MAIN_NAME|$DA360_NAME|$YOPO_NAME" || echo "  (no container running)"
echo " Simulator: http://127.0.0.1:8080  (press Ctrl+F5 to hard-refresh the latest frontend)"
echo "==============================================="
