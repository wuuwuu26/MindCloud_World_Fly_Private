#!/usr/bin/env python3
"""
cesium_dataset_server.py — 接收浏览器端 CesiumYOPODataset 采集的样本, 按
YOPO_360 训练所需的磁盘格式落盘:

    <DATASET_DIR>/<map_i>/img_<n>.png        uint16, 值 = depth / maxDepth * 65535
    <DATASET_DIR>/<map_i>/img_<n>_m.png      uint8 掩码, 255 = 有效
    <DATASET_DIR>/pose-<map_i>.csv           表头 yaw,pitch,roll,px,py,pz
    <DATASET_DIR>/pointcloud-<map_i>.ply     Cesium 障碍几何点云 (SafetyLoss 建 ESDF 用)
    <DATASET_DIR>/max_depth.txt              maxDepth (文本)

train_yopo.py 的 SafetyLoss 会自动从 pointcloud-*.ply 构建 ESDF, 因此无需改动
训练代码。默认 DATASET_DIR = <repo>/YOPO_360/dataset (与 traj_opt.yaml 的
dataset_path: ../dataset 对应), 可用环境变量 YOPO_DATASET_DIR 覆盖。

用法:
    python3 scripts/cesium_dataset_server.py [--port 8003] [--dir <DATASET_DIR>]
"""
import argparse
import base64
import io
import json
import os
import threading

import numpy as np
from flask import Flask, request, jsonify
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DATASET_DIR = os.path.join(SCRIPT_DIR, "..", "YOPO_360", "dataset")

app = Flask(__name__)
_lock = threading.Lock()
_stats = {"maps": 0, "samples": 0}


def _cors_headers():
    return {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "GET,POST,OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
    }


@app.after_request
def _after(resp):
    for k, v in _cors_headers().items():
        resp.headers[k] = v
    return resp


def _dataset_dir():
    return os.environ.get("YOPO_DATASET_DIR", DEFAULT_DATASET_DIR)


def _write_png_u16(path, depth, max_depth):
    arr = np.clip(depth / max_depth * 65535.0, 0, 65535).astype(np.uint16)
    Image.fromarray(arr, "I;16").save(path)


def _write_png_u8(path, mask):
    Image.fromarray(mask.astype(np.uint8), "L").save(path)


@app.route("/dataset/begin", methods=["POST", "OPTIONS"])
def begin():
    if request.method == "OPTIONS":
        return ("", 204)
    body = request.get_json(force=True) or {}
    max_depth = float(body.get("max_depth", 20.0))
    num_maps = int(body.get("num_maps", 1))
    clear = bool(body.get("clear", False))
    ddir = _dataset_dir()
    with _lock:
        if clear and os.path.isdir(ddir):
            import shutil
            shutil.rmtree(ddir)
        os.makedirs(ddir, exist_ok=True)
        with open(os.path.join(ddir, "max_depth.txt"), "w") as f:
            f.write(f"{max_depth}\n")
        for m in range(num_maps):
            mdir = os.path.join(ddir, str(m))
            os.makedirs(mdir, exist_ok=True)
            # 重新开始时清空该 map 的 pose 文件, 并写入表头
            # 训练 yopo_dataset.py 用 skiprows=1 跳过表头, 读取 7 列
            # px,py,pz,qw,qx,qy,qz (位置 + 四元数 wxyz)
            with open(os.path.join(ddir, f"pose-{m}.csv"), "w") as f:
                f.write("px,py,pz,qw,qx,qy,qz\n")
        _stats["maps"] = num_maps
        _stats["samples"] = 0
    return jsonify({"ok": True, "dataset_dir": os.path.abspath(ddir), "max_depth": max_depth})


@app.route("/dataset/sample", methods=["POST", "OPTIONS"])
def sample():
    if request.method == "OPTIONS":
        return ("", 204)
    body = request.get_json(force=True) or {}
    map_id = int(body["map_id"])
    index = int(body["index"])
    max_depth = float(body.get("max_depth", 20.0))
    width = int(body.get("width", 384))
    height = int(body.get("height", 192))
    depth = np.frombuffer(base64.b64decode(body["depth_b64"]), dtype=np.float32)
    if depth.size != width * height:
        # 形状不一致时按实际长度推断 (尽量保持 384x192)
        depth = depth.reshape(-1)[: width * height]
        if depth.size < width * height:
            depth = np.pad(depth, (0, width * height - depth.size), constant_values=max_depth)
    depth = depth.reshape(height, width)
    mask = np.frombuffer(base64.b64decode(body["mask_b64"]), dtype=np.uint8)
    if mask.size != width * height:
        mask = np.pad(mask.astype(np.float32), (0, width * height - mask.size), constant_values=0).astype(np.uint8)
    mask = mask.reshape(height, width)
    pose = body["pose"]

    ddir = _dataset_dir()
    mdir = os.path.join(ddir, str(map_id))
    os.makedirs(mdir, exist_ok=True)
    with _lock:
        _write_png_u16(os.path.join(mdir, f"img_{index}.png"), depth, max_depth)
        _write_png_u8(os.path.join(mdir, f"img_{index}_m.png"), mask)
        with open(os.path.join(ddir, f"pose-{map_id}.csv"), "a") as f:
            f.write(f"{pose['px']},{pose['py']},{pose['pz']},"
                    f"{pose['qw']},{pose['qx']},{pose['qy']},{pose['qz']}\n")
        _stats["samples"] += 1
    return jsonify({"ok": True, "map_id": map_id, "index": index})


@app.route("/dataset/pointcloud", methods=["POST", "OPTIONS"])
def pointcloud():
    if request.method == "OPTIONS":
        return ("", 204)
    body = request.get_json(force=True) or {}
    map_id = int(body["map_id"])
    points = body.get("points", [])
    ddir = _dataset_dir()
    os.makedirs(ddir, exist_ok=True)
    path = os.path.join(ddir, f"pointcloud-{map_id}.ply")
    lines = ["ply", "format ascii 1.0",
             f"element vertex {len(points)}",
             "property float x", "property float y", "property float z",
             "end_header"]
    for p in points:
        lines.append(f"{float(p[0])} {float(p[1])} {float(p[2])}")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return jsonify({"ok": True, "map_id": map_id, "num_points": len(points)})


@app.route("/dataset/status", methods=["GET"])
def status():
    return jsonify({"ok": True, **_stats, "dataset_dir": os.path.abspath(_dataset_dir())})


@app.route("/dataset/health", methods=["GET"])
def health():
    return jsonify({"ok": True})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8003)
    parser.add_argument("--dir", type=str, default=None, help="override dataset dir")
    parser.add_argument("--host", type=str, default="0.0.0.0")
    args = parser.parse_args()
    if args.dir:
        os.environ["YOPO_DATASET_DIR"] = os.path.abspath(args.dir)
    print(f"[cesium_dataset_server] dataset dir = {os.path.abspath(_dataset_dir())}")
    print(f"[cesium_dataset_server] listening on http://{args.host}:{args.port}")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
