#!/usr/bin/env python3
"""等待无头采集达到目标样本数, 然后在 yopo-train 容器启动训练(修正 $SERVER 展开 bug)。"""
import time, json, urllib.request, subprocess, sys

TARGET = 160
SERVER = "http://localhost:8003"
TRAIN_CMD = ("docker exec yopo-train bash -c "
             "'cd /workspace/YOPO/YOPO && YOPO_EPOCH=50 python3 train_yopo.py "
             "--pretrained 1 --trial 1 --epoch 50' > /tmp/train.log 2>&1")


def get_samples():
    try:
        d = json.loads(urllib.request.urlopen(SERVER + "/dataset/status", timeout=5).read())
        return int(d.get("samples", 0))
    except Exception as e:
        return -1


def main():
    for i in range(220):  # 最多 ~110 分钟
        s = get_samples()
        print(f"[wait {i}] samples={s}/{TARGET}", flush=True)
        if s >= TARGET:
            print("[wait] target reached, launching training", flush=True)
            subprocess.run(TRAIN_CMD, shell=True)
            print("[wait] training launch command finished", flush=True)
            return
        time.sleep(30)
    print("[wait] TIMEOUT waiting for collection; aborting", flush=True)
    sys.exit(1)


if __name__ == "__main__":
    main()
