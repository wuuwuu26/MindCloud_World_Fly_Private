#!/usr/bin/env bash
# restart_all.sh — 一键强制重启 MindCloud_World_Fly 全部容器
#
#   强制停止并重建以下容器(不重建 Docker 镜像):
#     1. DA360 深度服务   mindcloud-da360-api    http://127.0.0.1:5688
#     2. YOPO 避障后端     mindcloud-yopo-api     http://127.0.0.1:5689
#     3. 主飞行进程       google-tiles-flight     http://127.0.0.1:8080
#
#   用法:
#     ./restart_all.sh                    # 重启全部三个容器
#     ./restart_all.sh --no-main          # 只重启 DA360 + YOPO (主飞行保留)
#     ./restart_all.sh --no-da360         # 只重启 YOPO + 主飞行 (DA360 保留)
#     ./restart_all.sh --no-yopo          # 只重启 DA360 + 主飞行 (YOPO 保留)
#
#   说明:
#     - 两个后端容器都以只读挂载各自的 server 脚本, 改完 Python 后
#       重启容器即生效, 无需重新 build 镜像。
#     - 主飞行容器以 --detach 后台运行, 前端 JS 以只读挂载, 浏览器需
#       强刷(Ctrl+F5)以加载最新前端代码。
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── 参数 ──
DO_DA360=1
DO_YOPO=1
DO_MAIN=1
for arg in "$@"; do
    case "$arg" in
        --no-da360) DO_DA360=0 ;;
        --no-yopo)  DO_YOPO=0 ;;
        --no-main)  DO_MAIN=0 ;;
        -h|--help)
            grep '^#' "$0" | sed -n '2,/^ *用法/p' | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知参数: $arg (用 --help 查看)" >&2; exit 1 ;;
    esac
done

DA360_NAME="mindcloud-da360-api"
YOPO_NAME="mindcloud-yopo-api"
MAIN_NAME="google-tiles-flight"

echo "==============================================="
echo " MindCloud_World_Fly 一键重启"
echo " 重启: DA360=$DO_DA360 YOPO=$DO_YOPO 主飞行=$DO_MAIN"
echo "==============================================="

# ── 停止并等待容器释放端口 ──
stop_wait() {
    local name="$1"
    if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "$name"; then
        echo "  停止 $name ..."
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
            echo "  [OK] $label 就绪"
            return 0
        fi
        sleep 2
        waited=$((waited + 2))
    done
    echo "  [WARN] $label $timeout 秒未就绪" >&2
    return 1
}

# ── 1. DA360 ──
if [ "$DO_DA360" = "1" ]; then
    echo "[1/3] 重启 DA360 深度服务 ..."
    stop_wait "$DA360_NAME"
    DA360_DETACH=1 DA360_NO_WARMUP=1 ./scripts/start_da360_api.sh large >/tmp/restart_da360.log 2>&1 &
    DA360_PID=$!
    wait_health "http://127.0.0.1:5688/health" 120 DA360 || true
fi

# ── 2. YOPO ──
if [ "$DO_YOPO" = "1" ]; then
    echo "[2/3] 重启 YOPO 避障后端 ..."
    stop_wait "$YOPO_NAME"
    # 默认部署 YOPO_40 (wc=8, 学习式避障) + TensorRT 加速 (YOPO_USE_TRT=1)。
    # 引擎缺失时仍正常启动(回退 PyTorch eager), 并给出生成引擎的提示。
    # 如需切回其他模型, 将下方 YOPO_MODEL_PATH 改为对应 checkpoint 即可。
    # 巡航速度默认 YOPO_VELOCITY=15.0 (vel_max≈15 m/s, acc_max≈37.5 m/s²):
    # 实测对比 — 8.0 巡航仅 ~7 m/s(太慢, 用户反馈 0.5m/s 即源于此);
    # 16.0 轨迹端点被网络预测放大到 ~76 m/s(即"突然飞得很快"的爆速真凶);
    # 15.0 中段巡航 ~12-15 m/s、端点 ~16-19 m/s, 快而可控。客户端侧已把
    # yopoPosErrMaxV 提到 15(解锁位置环巡航上限) + yopoMaxSpd=15 钳制双保险防猛冲。
    # 想更快可设 YOPO_VELOCITY=16~18(注意爆速风险); 更稳可降到 12。
    # YOPO_CTRL_TIME_SCALE: 指令连续修复后的"快进"系数(ctrl_time 倍速推进)。
    # 设为 1 = 完全跟随网络规划速度(vel_max≈15 → 巡航≤15 m/s)。
    # 注意: SCALE>1 会把指令速度推到 vel_max*SCALE(如 2→≈30 m/s), 虽被
    # YOPO_SPEED_CAP=15 硬上限钳回 15, 但会造成规划位置超前、无人机持续追迹滞后,
    # 故默认保持 1。提速诉求应通过调 YOPO_VELOCITY 实现, 而非放大 SCALE。
    TRT_ENGINE="$SCRIPT_DIR/asset/yopo-trt/yopo_trt.pth"
    if [ ! -f "$TRT_ENGINE" ]; then
        echo "  [WARN] 未找到 TensorRT 引擎: $TRT_ENGINE" >&2
        echo "         YOPO 将以 PyTorch eager 慢速运行。生成引擎:" >&2
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

# ── 3. 主飞行 ──
if [ "$DO_MAIN" = "1" ]; then
    echo "[3/3] 重启主飞行进程 ..."
    stop_wait "$MAIN_NAME"
    DETACH=1 \
        nohup bash "$SCRIPT_DIR/launch.sh" --detach >/tmp/restart_main.log 2>&1 &
    sleep 4
    wait_health "http://127.0.0.1:8080/" 60 主飞行 || true
fi

# 清理后台日志进程(如果有前台跟随)
wait 2>/dev/null || true

echo "==============================================="
echo " 全部就绪:"
docker ps --format '  {{.Names}}\t{{.Status}}\t{{.Ports}}' \
    | grep -E "$MAIN_NAME|$DA360_NAME|$YOPO_NAME" || echo "  (无容器运行)"
echo " 模拟器: http://127.0.0.1:8080  (请 Ctrl+F5 强刷加载最新前端)"
echo "==============================================="
