#!/usr/bin/env bash
set -Eeuo pipefail

# start_all.sh — 一键启动 MindCloud_World_Fly 全部服务
#
#   1. DA360 深度估计服务   http://127.0.0.1:5688
#   2. YOPO 避障后端        http://127.0.0.1:5689
#   3. 主飞行进程           http://127.0.0.1:8080
#
# 用法:
#   ./start_all.sh             # 启动所有服务, 不自动开浏览器
#   OPEN_BROWSER=1 ./start_all.sh   # 启动后自动打开浏览器
#
# 已运行的服务会自动跳过, 幂等可重复执行。
# 停止: Ctrl+C 停主进程;  YOPO: docker rm -f mindcloud-yopo-api
#       DA360: docker rm -f mindcloud-da360-api

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==============================================="
echo " MindCloud_World_Fly 一键启动"
echo "==============================================="

# ── 1. DA360 深度服务 (5688) ─────────────────────────
if curl -s -m 2 http://127.0.0.1:5688/health >/dev/null 2>&1; then
    echo "[1/3] DA360 深度服务  已运行  (http://127.0.0.1:5688)"
else
    echo "[1/3] 启动 DA360 深度服务 (后台) ..."
    ./scripts/start_da360_api.sh >/tmp/da360_api.log 2>&1 &
    ok=0
    for i in $(seq 1 60); do
        if curl -s -m 2 http://127.0.0.1:5688/health >/dev/null 2>&1; then
            echo "      DA360 就绪 (http://127.0.0.1:5688)"
            ok=1
            break
        fi
        sleep 2
    done
    if [ "$ok" != "1" ]; then
        echo "      WARN: DA360 120s 未就绪, 日志见 /tmp/da360_api.log" >&2
    fi
fi

# ── 2. YOPO 避障后端 (5689) ─────────────────────────
if curl -s -m 2 http://127.0.0.1:5689/yopo/status >/dev/null 2>&1; then
    echo "[2/3] YOPO 避障后端  已运行  (http://127.0.0.1:5689)"
else
    echo "[2/3] 启动 YOPO 避障后端 (后台容器) ..."
    YOPO_DETACH=1 ./scripts/start_yopo_api.sh
    echo "      YOPO 就绪 (http://127.0.0.1:5689)"
fi

# ── 3. 主飞行进程 (8080) ─────────────────────────
echo "[3/3] 启动主飞行进程  http://127.0.0.1:8080"
echo "      提示: 启动耗时约 10~30s, 请稍后刷新浏览器"
if [ "${OPEN_BROWSER:-0}" = "1" ]; then
    exec ./launch.sh
else
    exec ./launch.sh --no-open
fi
