#!/usr/bin/env bash
set -Eeuo pipefail

# start_yopo_api.sh
#   Build (if necessary) and run the YOPO navigation backend.
#
#   Usage:
#       ./scripts/start_yopo_api.sh                 # docker mode, auto-build if image missing
#       YOPO_MODE=local ./scripts/start_yopo_api.sh # run Python directly
#       YOPO_FORCE_BUILD=1 ./scripts/start_yopo_api.sh # force rebuild docker image
#
#   Important environment variables:
#       YOPO_MODE          docker|local (default docker)
#       YOPO_IMAGE         docker image tag (default mindcloud-yopo:latest)
#       YOPO_MODEL_PATH    path to YOPO checkpoint (default third_party/yopo/saved/YOPO_18/epoch20.pth)
#       YOPO_PORT          host port exposed (default 5689)
#       YOPO_FORCE_BUILD   1=always rebuild image, 0=use cached image if present (default 0)
#       YOPO_GPUS          docker --gpus value, or "none" for CPU (default all)
#       YOPO_DETACH        1=run container in background (default 0)
#
#   Note on depth images:
#       DA360 uses a 360 equirectangular RGB image and runs a depth-estimation
#       model server (port 5688).  YOPO_360 expects a 192x384 ERP panorama
#       depth map in metres (encoding '32FC1') plus a uint8 validity mask
#       (255=valid).  DA360's raw output is already ERP, so it is resized
#       directly to 192x384 instead of being reprojected into a pinhole.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODE="${YOPO_MODE:-docker}"
IMAGE="${YOPO_IMAGE:-mindcloud-yopo:latest}"
NAME="${YOPO_CONTAINER_NAME:-mindcloud-yopo-api}"
PORT="${YOPO_PORT:-5689}"
MODEL_PATH="${YOPO_MODEL_PATH:-$PROJECT_ROOT/third_party/yopo/saved/YOPO_18/epoch20.pth}"
BASE_IMAGE="${YOPO_BASE_IMAGE:-pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime}"
BUILD_NETWORK="${YOPO_BUILD_NETWORK:-host}"
BUILD_RETRIES="${YOPO_BUILD_RETRIES:-3}"
MOUNT_SERVER="${YOPO_MOUNT_SERVER:-1}"
if ! [[ "$BUILD_RETRIES" =~ ^[0-9]+$ ]] || (( BUILD_RETRIES < 1 )); then
    BUILD_RETRIES=1
fi
SERVER_SHA="$(sha256sum "$SCRIPT_DIR/yopo_server.py" | awk '{print $1}')"

# Docker build arguments
build_args=(
    --pull=false
)
FORWARDED_PROXY_BUILD_ARGS=0
if [[ -n "$BUILD_NETWORK" ]]; then
    build_args+=(--network "$BUILD_NETWORK")
fi

# Disable pip proxy inside the build if host proxy is broken; user can override
# by setting YOPO_PIP_NO_PROXY=0.
YOPO_PIP_NO_PROXY="${YOPO_PIP_NO_PROXY:-0}"
if [[ "$YOPO_PIP_NO_PROXY" == "1" ]]; then
    build_args+=(--build-arg "HTTP_PROXY=")
    build_args+=(--build-arg "http_proxy=")
    build_args+=(--build-arg "HTTPS_PROXY=")
    build_args+=(--build-arg "https_proxy=")
fi

add_proxy_build_arg() {
    local name="$1"
    local value="$2"
    if [[ -n "$value" ]]; then
        build_args+=(--build-arg "$name=$value")
        FORWARDED_PROXY_BUILD_ARGS=1
    fi
}

add_proxy_build_arg_pair() {
    local upper_name="$1"
    local lower_name="$2"
    local upper_value="${!upper_name:-}"
    local lower_value="${!lower_name:-}"
    add_proxy_build_arg "$upper_name" "${upper_value:-$lower_value}"
    add_proxy_build_arg "$lower_name" "${lower_value:-$upper_value}"
}

# Only forward proxy args if the user explicitly allows it
if [[ "$YOPO_PIP_NO_PROXY" != "1" ]]; then
    add_proxy_build_arg_pair HTTP_PROXY http_proxy
    add_proxy_build_arg_pair HTTPS_PROXY https_proxy
    add_proxy_build_arg_pair FTP_PROXY ftp_proxy
    add_proxy_build_arg_pair ALL_PROXY all_proxy
    add_proxy_build_arg_pair NO_PROXY no_proxy
fi

if [[ ! -s "$MODEL_PATH" ]]; then
    echo "ERROR: YOPO model not found at: $MODEL_PATH" >&2
    echo "       Please ensure the model file exists or set YOPO_MODEL_PATH." >&2
    exit 1
fi

echo "YOPO model: $MODEL_PATH"
echo "YOPO mode:  $MODE"

MODEL_PATH="$(readlink -f "$MODEL_PATH")"
MODEL_BASENAME="$(basename "$MODEL_PATH")"

# Local mode: run Python script directly
if [[ "$MODE" == "local" ]]; then
    PYTHON_BIN="${YOPO_PYTHON:-python3}"
    exec "$PYTHON_BIN" "$SCRIPT_DIR/yopo_server.py" \
        --model-path "$MODEL_PATH" \
        --port "$PORT" \
        --host 0.0.0.0 \
        --verbose
fi

# Docker mode
command -v docker >/dev/null 2>&1 || {
    echo "Docker is required for YOPO_MODE=docker." >&2
    exit 1
}

docker info >/dev/null 2>&1 || {
    echo "Cannot access Docker daemon." >&2
    exit 1
}

# Build image if needed
build_ok=0
if [[ "${YOPO_FORCE_BUILD:-0}" == "1" ]]; then
    echo "YOPO_FORCE_BUILD=1: rebuilding Docker image $IMAGE ..."
elif docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Using existing YOPO image $IMAGE."
    echo "  To force a rebuild, run: YOPO_FORCE_BUILD=1 $0"
    build_ok=1
else
    echo "YOPO image $IMAGE not found; building now ..."
fi

if [[ "$build_ok" != "1" ]]; then
    for ((attempt = 1; attempt <= BUILD_RETRIES; attempt++)); do
        echo "Docker build attempt $attempt/$BUILD_RETRIES ..."
        if docker build "${build_args[@]}" \
            --network=host \
            --build-arg "YOPO_BASE_IMAGE=$BASE_IMAGE" \
    --build-arg "YOPO_SERVER_SHA=$SERVER_SHA" \
    -f "$PROJECT_ROOT/Dockerfile.yopo" \
    -t "$IMAGE" \
    "$PROJECT_ROOT"; then
            build_ok=1
            echo "Docker image $IMAGE built successfully."
            break
        fi
        if (( attempt < BUILD_RETRIES )); then
            echo "WARNING: YOPO image build failed; retrying ($attempt/$BUILD_RETRIES)..." >&2
            sleep 2
        fi
    done
fi

if [[ "$build_ok" != "1" ]]; then
    echo "ERROR: failed to build $IMAGE from $BASE_IMAGE." >&2
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    exit 1
fi

docker rm -f "$NAME" >/dev/null 2>&1 || true

gpu_args=()
if [[ "${YOPO_GPUS:-all}" != "none" ]]; then
    gpu_args=(--gpus "${YOPO_GPUS:-all}")
fi

run_args=(
    --rm
    --name "$NAME"
    -p "$PORT:5689"
    -e "YOPO_NO_WARMUP=${YOPO_NO_WARMUP:-0}"
    -v "$MODEL_PATH:/models/$MODEL_BASENAME:ro"
)

if [[ "$MOUNT_SERVER" == "1" ]]; then
    run_args+=(-v "$SCRIPT_DIR/yopo_server.py:/opt/mindcloud-yopo/scripts/yopo_server.py:ro")
fi

# Mount YOPO source code for model imports
YOPO_SRC_DIR="${YOPO_SRC_DIR:-$PROJECT_ROOT/third_party/yopo}"
if [[ -d "$YOPO_SRC_DIR" ]]; then
    run_args+=(-v "$YOPO_SRC_DIR:/opt/mindcloud-yopo/third_party/yopo:ro")
else
    echo "WARNING: YOPO source directory not found at $YOPO_SRC_DIR" >&2
    echo "The YOPO server will look for the YOPO module in third_party/yopo relative to the server path." >&2
fi

if [[ "${YOPO_DETACH:-0}" == "1" ]]; then
    run_args=(-d "${run_args[@]}")
fi

echo "Starting YOPO container $NAME on host port $PORT -> container port 5689 ..."
if [[ "${YOPO_DETACH:-0}" == "1" ]]; then
    echo "Container is running in detached mode. Stop it later with: docker rm -f $NAME"
fi

exec docker run "${gpu_args[@]}" "${run_args[@]}" "$IMAGE" \
    python /opt/mindcloud-yopo/scripts/yopo_server.py \
        --model-path "/models/$MODEL_BASENAME" \
        --host 0.0.0.0 \
        --port 5689 \
        --verbose