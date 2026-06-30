#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODE="${DA360_MODE:-docker}"
IMAGE="${DA360_IMAGE:-mindcloud-da360:latest}"
NAME="${DA360_CONTAINER_NAME:-mindcloud-da360-api}"
PORT="${DA360_PORT:-5688}"
MODEL_NAME="${DA360_MODEL:-small}"
MODEL_PATH="${DA360_MODEL_PATH:-$PROJECT_ROOT/third_party/DA360/checkpoints/DA360_${MODEL_NAME}.pth}"
BASE_IMAGE="${DA360_BASE_IMAGE:-pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime}"
BUILD_RETRIES="${DA360_BUILD_RETRIES:-3}"
if ! [[ "$BUILD_RETRIES" =~ ^[0-9]+$ ]] || (( BUILD_RETRIES < 1 )); then
    BUILD_RETRIES=1
fi
SERVER_SHA="$(sha256sum "$SCRIPT_DIR/da360_server.py" | awk '{print $1}')"

image_label() {
    docker image inspect --format "{{ index .Config.Labels \"$1\" }}" "$IMAGE" 2>/dev/null || true
}

image_server_sha_from_file() {
    docker run --rm --entrypoint sha256sum "$IMAGE" /opt/mindcloud-da360/scripts/da360_server.py 2>/dev/null \
        | awk '{print $1}' \
        || true
}

if [[ ! -s "$MODEL_PATH" ]]; then
    if [[ -n "${DA360_MODEL_PATH:-}" ]]; then
        echo "DA360_MODEL_PATH does not exist: $MODEL_PATH" >&2
        exit 1
    fi
    "$SCRIPT_DIR/download_da360_model.sh" "$MODEL_NAME"
fi

MODEL_PATH="$(readlink -f "$MODEL_PATH")"
MODEL_BASENAME="$(basename "$MODEL_PATH")"

if [[ "$MODE" == "local" ]]; then
    PYTHON_BIN="${DA360_PYTHON:-python3}"
    exec "$PYTHON_BIN" "$SCRIPT_DIR/da360_server.py" --model-path "$MODEL_PATH" --port "$PORT"
fi

command -v docker >/dev/null 2>&1 || {
    echo "Docker is required for DA360_MODE=docker." >&2
    exit 1
}

docker info >/dev/null 2>&1 || {
    echo "Cannot access Docker daemon." >&2
    exit 1
}

build_ok=0
existing_server_sha="$(image_label "mindcloud.da360.server_sha")"
if [[ "${DA360_FORCE_BUILD:-0}" != "1" && "$existing_server_sha" != "$SERVER_SHA" ]] &&
    docker image inspect "$IMAGE" >/dev/null 2>&1; then
    existing_server_sha="$(image_server_sha_from_file)"
fi
if [[ "${DA360_FORCE_BUILD:-0}" != "1" && "$existing_server_sha" == "$SERVER_SHA" ]]; then
    echo "DA360 image $IMAGE already contains the current server script; skipping rebuild."
    build_ok=1
else
    for ((attempt = 1; attempt <= BUILD_RETRIES; attempt++)); do
        if docker build --pull=false \
            --build-arg "DA360_BASE_IMAGE=$BASE_IMAGE" \
            --build-arg "DA360_SERVER_SHA=$SERVER_SHA" \
            -f "$PROJECT_ROOT/Dockerfile.da360" \
            -t "$IMAGE" \
            "$PROJECT_ROOT"; then
            build_ok=1
            break
        fi
        if (( attempt < BUILD_RETRIES )); then
            echo "WARNING: DA360 image build failed; retrying ($attempt/$BUILD_RETRIES)..." >&2
            sleep 2
        fi
    done
    if [[ "$build_ok" != "1" ]] && docker image inspect "$IMAGE" >/dev/null 2>&1; then
        existing_server_sha="$(image_server_sha_from_file)"
        if [[ "$existing_server_sha" == "$SERVER_SHA" ]]; then
            echo "WARNING: failed to rebuild $IMAGE, but the existing local image contains the current server script; using it." >&2
            build_ok=1
        fi
    fi
fi

if [[ "$build_ok" != "1" ]]; then
    if [[ "${DA360_ALLOW_STALE_IMAGE:-0}" == "1" ]] && docker image inspect "$IMAGE" >/dev/null 2>&1; then
        echo "WARNING: failed to rebuild $IMAGE; DA360_ALLOW_STALE_IMAGE=1, using the existing local image." >&2
    else
        echo "ERROR: failed to build $IMAGE from $BASE_IMAGE." >&2
        echo "Not starting a stale DA360 container because the frontend/server protocol may be incompatible." >&2
        echo "If Docker Hub timed out, retry this command after the network recovers, or set DA360_BASE_IMAGE to a local/mirror image." >&2
        docker rm -f "$NAME" >/dev/null 2>&1 || true
        exit 1
    fi
fi
docker rm -f "$NAME" >/dev/null 2>&1 || true

gpu_args=()
if [[ "${DA360_GPUS:-all}" != "none" ]]; then
    gpu_args=(--gpus "${DA360_GPUS:-all}")
fi

run_args=(
    --rm
    --name "$NAME"
    -p "$PORT:5688"
    -e "DA360_NO_WARMUP=${DA360_NO_WARMUP:-0}"
    -e "DA360_INPUT_SCALE=${DA360_INPUT_SCALE:-0.65}"
    -e "DA360_INPUT_WIDTH=${DA360_INPUT_WIDTH:-0}"
    -e "DA360_INPUT_HEIGHT=${DA360_INPUT_HEIGHT:-0}"
    -e "DA360_OUTPUT_FORMAT=${DA360_OUTPUT_FORMAT:-jpeg}"
    -e "DA360_JPEG_QUALITY=${DA360_JPEG_QUALITY:-72}"
    -e "DA360_AMP=${DA360_AMP:-1}"
    -e "DA360_CHANNELS_LAST=${DA360_CHANNELS_LAST:-1}"
    -e "DA360_TORCH_COMPILE=${DA360_TORCH_COMPILE:-0}"
    -v "$MODEL_PATH:/models/$MODEL_BASENAME:ro"
)

if [[ "${DA360_DETACH:-0}" == "1" ]]; then
    run_args=(-d "${run_args[@]}")
fi

exec docker run "${gpu_args[@]}" "${run_args[@]}" "$IMAGE" \
    python /opt/mindcloud-da360/scripts/da360_server.py \
        --model-path "/models/$MODEL_BASENAME" \
        --host 0.0.0.0 \
        --port 5688
