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
#       YOPO_MODEL_PATH    path to YOPO checkpoint (default third_party/yopo/saved/YOPO_40/epoch50.pth)
#       YOPO_PORT          host port exposed (default 5689)
#       YOPO_FORCE_BUILD   1=always rebuild image, 0=use cached image if present (default 0)
#       YOPO_GPUS          docker --gpus value, or "none" for CPU (default all)
#       YOPO_DETACH        1=run container in background (default 0)
#       YOPO_USE_TRT        1=enable TensorRT engine acceleration (default empty=off)
#       YOPO_TRT_PATH       path to TensorRT engine inside container
#                          (default /opt/mindcloud-yopo/trt/yopo_trt.pth)
#
#   Note on depth images:
#       DA360 uses a 360 equirectangular RGB image and runs a depth-estimation
#       model server (port 5688).  YOPO 原版 expects a 192x384 ERP panorama
#       depth map in metres (encoding '32FC1') plus a uint8 validity mask
#       (255=valid).  DA360's raw output is already ERP, so it is resized
#       directly to 192x384 instead of being reprojected into a pinhole.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODE="${YOPO_MODE:-docker}"
IMAGE="${YOPO_IMAGE:-mindcloud-yopo:latest}"
NAME="${YOPO_CONTAINER_NAME:-mindcloud-yopo-api}"
PORT="${YOPO_PORT:-5689}"
MODEL_PATH="${YOPO_MODEL_PATH:-$PROJECT_ROOT/third_party/yopo/saved/YOPO_40/epoch50.pth}"
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

# ── TensorRT 自动启用 ──────────────────────────────────────────────
# 引擎文件存在且用户未显式设置 YOPO_USE_TRT 时, 默认启用 TRT 加速, 使
# restart_all.sh / launch.sh 等任意启动入口一键即可跑在加速模式。
# 显式 YOPO_USE_TRT=0 可强制回退 PyTorch eager。
TRT_ENGINE_HOST="$PROJECT_ROOT/asset/yopo-trt/yopo_trt.pth"
if [[ -z "${YOPO_USE_TRT:-}" ]]; then
    if [[ -f "$TRT_ENGINE_HOST" ]]; then
        YOPO_USE_TRT=1
        echo "YOPO TRT: 检测到引擎 $TRT_ENGINE_HOST → 自动启用 YOPO_USE_TRT=1 (推理加速)"
    else
        YOPO_USE_TRT=0
        echo "YOPO TRT: 未找到引擎 $TRT_ENGINE_HOST (运行 scripts/yopo_trt_transfer.py 生成) → 回退 PyTorch eager"
    fi
fi
export YOPO_USE_TRT

echo "YOPO model: $MODEL_PATH"
echo "YOPO mode:  $MODE"
echo "YOPO TRT:   YOPO_USE_TRT=$YOPO_USE_TRT"

MODEL_PATH="$(readlink -f "$MODEL_PATH")"
MODEL_BASENAME="$(basename "$MODEL_PATH")"

# Local mode: run Python script directly (no docker -> 重启最快, 推荐用于调参迭代)
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
    docker rm -fv "$NAME" >/dev/null 2>&1 || true
    exit 1
fi

docker rm -fv "$NAME" >/dev/null 2>&1 || true

gpu_args=()
if [[ "${YOPO_GPUS:-all}" != "none" ]]; then
    gpu_args=(--gpus "${YOPO_GPUS:-all}")
fi

run_args=(
    --rm
    --name "$NAME"
    -p "$PORT:5689"
    -p 5690:5690
    -e "YOPO_NO_WARMUP=${YOPO_NO_WARMUP:-0}"
    # 转发速度/时间缩放相关环境变量(否则在 docker 模式下设置无效, 只能改 yaml 重启)
    -e "YOPO_VELOCITY=${YOPO_VELOCITY:-}"
    -e "YOPO_CTRL_TIME_SCALE=${YOPO_CTRL_TIME_SCALE:-}"
    # 轨迹末端外推时长(秒), 修重规划间隔内的指令冻结锯齿; 默认 2.0 (见 yopo_server.py)
    -e "YOPO_TRAJ_EXTEND_S=${YOPO_TRAJ_EXTEND_S:-}"
    # TensorRT 加速开关与引擎路径: YOPO_USE_TRT=1 时加载 YOPO_TRT_PATH 指向的引擎
    -e "YOPO_USE_TRT=${YOPO_USE_TRT:-}"
    -e "YOPO_TRT_PATH=${YOPO_TRT_PATH:-/opt/mindcloud-yopo/trt/yopo_trt.pth}"
    -v "$MODEL_PATH:/models/$MODEL_BASENAME:ro"
    # TensorRT 引擎持久化目录(重规划加速权重, 跨容器重建保留)
    -v "$PROJECT_ROOT/asset/yopo-trt:/opt/mindcloud-yopo/trt:rw"
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

# ── 自动构建 TensorRT 引擎 (启用 TRT 但宿主引擎缺失时) ──
# 在容器内用 GPU 把当前模型固化为引擎, 存到 asset/yopo-trt (挂载 rw), 使
# restart_all.sh / launch.sh 等任意入口一键即可获得 TRT 加速, 无需手动预处理。
if [[ "${YOPO_USE_TRT:-0}" == "1" && "${MODE}" != "local" ]]; then
    if [[ ! -f "$TRT_ENGINE_HOST" ]]; then
        echo "YOPO TRT: 未找到引擎 $TRT_ENGINE_HOST, 尝试在容器内用 GPU 自动构建 ..."
        docker run --rm "${gpu_args[@]}" \
            -v "$SCRIPT_DIR/yopo_trt_transfer.py:/opt/mindcloud-yopo/scripts/yopo_trt_transfer.py:ro" \
            -v "$YOPO_SRC_DIR:/opt/mindcloud-yopo/third_party/yopo:ro" \
            -v "$MODEL_PATH:/models/$MODEL_BASENAME:ro" \
            -v "$PROJECT_ROOT/asset/yopo-trt:/opt/mindcloud-yopo/trt:rw" \
            "$IMAGE" python /opt/mindcloud-yopo/scripts/yopo_trt_transfer.py \
                --model "/models/$MODEL_BASENAME" \
                --out /opt/mindcloud-yopo/trt/yopo_trt.pth \
        && echo "YOPO TRT: 引擎构建成功 -> $TRT_ENGINE_HOST" \
        || echo "YOPO TRT: 自动构建失败 (容器可能缺少 tensorrt/GPU), 将回退 PyTorch eager" >&2
    fi
fi

echo "Starting YOPO container $NAME on host port $PORT -> container port 5689 ..."
if [[ "${YOPO_DETACH:-0}" == "1" ]]; then
    echo "Container is running in detached mode. Stop it later with: docker rm -fv $NAME"
fi

exec docker run "${gpu_args[@]}" "${run_args[@]}" "$IMAGE" \
    python /opt/mindcloud-yopo/scripts/yopo_server.py \
        --model-path "/models/$MODEL_BASENAME" \
        --host 0.0.0.0 \
        --port 5689 \
        --ws-port 5690 \
        --verbose