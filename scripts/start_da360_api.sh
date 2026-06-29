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

if ! docker build --pull=false -f "$PROJECT_ROOT/Dockerfile.da360" -t "$IMAGE" "$PROJECT_ROOT"; then
    if docker image inspect "$IMAGE" >/dev/null 2>&1; then
        echo "WARNING: failed to rebuild $IMAGE; using the existing local image." >&2
    else
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
