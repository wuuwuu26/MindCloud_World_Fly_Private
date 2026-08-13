#!/bin/bash
# 等待无头采集达到目标样本数, 然后在 yopo-train 容器里启动训练(微调合成预训练模型)。
set -u
TARGET=${1:-200}
SERVER=http://localhost:8003
TRAIN_LOG=/tmp/train.log
MAX_WAIT_MIN=${2:-120}

echo "[wrap $(date +%T)] waiting for collection >= $TARGET samples (max ${MAX_WAIT_MIN}m)"

elapsed=0
while [ $elapsed -lt $MAX_WAIT_MIN ]; do
    s=$(python3 - <<'PY' 2>/dev/null
import urllib.request, json
try:
    d = json.loads(urllib.request.urlopen("$SERVER/dataset/status", timeout=5).read())
    print(int(d.get("samples", 0)))
except Exception:
    print(0)
PY
)
    if [ "${s:-0}" -ge "$TARGET" ]; then
        echo "[wrap $(date +%T)] collection done: $s samples"
        break
    fi
    sleep 30
    elapsed=$((elapsed + 1))
done

final=$(python3 - <<'PY' 2>/dev/null
import urllib.request, json
try:
    d = json.loads(urllib.request.urlopen("$SERVER/dataset/status", timeout=5).read())
    print(int(d.get("samples", 0)))
except Exception:
    print(0)
PY
)
echo "[wrap $(date +%T)] final collected samples = $final"

if [ "${final:-0}" -lt "$TARGET" ]; then
    echo "[wrap] collection did not reach target; aborting training."
    exit 1
fi

echo "[wrap $(date +%T)] launching training (finetune synthetic YOPO_1/epoch50 on Cesium data)..."
docker exec yopo-train bash -c "cd /workspace/YOPO/YOPO && YOPO_EPOCH=50 python3 train_yopo.py --pretrained 1 --trial 1 --epoch 50" > "$TRAIN_LOG" 2>&1
echo "[wrap $(date +%T)] training finished (exit $?). see $TRAIN_LOG"
