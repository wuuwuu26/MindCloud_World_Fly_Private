#!/usr/bin/env python3
"""
headless_collect.py — 用 Playwright 无头驱动飞行页面 (google-tiles-flight @ :8080)
自动采集 Cesium 训练数据集, 落盘由 cesium_dataset_server.py (:8003) 完成。

页面需带 ?autocollect=1 (main.js 会自动进入 tiles 模式并调用 CesiumYOPODataset)。
本脚本会显式确保 world/yopoDepthFromPanorama 就绪并触发采集, 并打印每步状态。

用法:
    python3 scripts/headless_collect.py --num-maps 4 --samples 300 --radius 12 --max-depth 20 --mode da360 --timeout 7200
"""
import argparse
import json
import sys
import time
import urllib.request

from playwright.sync_api import sync_playwright

APP_URL = "http://localhost:8080"
SERVER_URL = "http://localhost:8003"


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--num-maps", type=int, default=4)
    p.add_argument("--samples", type=int, default=300)
    p.add_argument("--radius", type=float, default=12.0)
    p.add_argument("--max-depth", type=float, default=20.0)
    p.add_argument("--mode", type=str, default="da360", choices=["da360", "raycast"])
    p.add_argument("--timeout", type=int, default=7200, help="总超时(秒)")
    p.add_argument("--app", type=str, default=APP_URL)
    p.add_argument("--server", type=str, default=SERVER_URL)
    return p.parse_args()


def server_status(server):
    try:
        with urllib.request.urlopen(f"{server}/dataset/status", timeout=5) as r:
            return json.loads(r.read().decode())
    except Exception:
        return None


def main():
    a = parse_args()
    target = a.num_maps * a.samples
    url = (f"{a.app}/?autocollect=1"
           f"&numMaps={a.num_maps}&samples={a.samples}"
           f"&radius={a.radius}&maxDepth={a.max_depth}"
           f"&mode={a.mode}&server={a.server}")

    print(f"[headless_collect] target={target} samples")
    print(f"[headless_collect] url={url}")

    launch_args = [
        "--use-gl=angle",
        "--use-angle=swiftshader",
        "--enable-unsafe-swiftshader",
        "--ignore-gpu-blocklist",
        "--enable-webgl",
        "--no-sandbox",
        "--disable-dev-shm-usage",
    ]

    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=True, args=launch_args)
        page = browser.new_page(viewport={"width": 1280, "height": 800})
        page.on("console", lambda m: print(f"[page:{m.type}] {m.text}"))
        page.on("pageerror", lambda e: print(f"[pageerror] {e}"))

        page.goto(url, wait_until="domcontentloaded", timeout=60000)

        def st():
            try:
                return page.evaluate("""() => ({
                    world: typeof window.world,
                    yopo: typeof window.yopoDepthFromPanorama,
                    cds: window.__cdsStatus || null,
                    startFn: typeof window.startTilesMode,
                    runFn: typeof window.runAutoCollect,
                })""")
            except Exception as e:
                return {"err": str(e)}

        # 1) 等 startTilesMode 函数就绪
        t0 = time.time()
        while time.time() - t0 < 30:
            s = st()
            if s.get("startFn") == "function":
                break
            time.sleep(1)

        # 2) 若 world 未就绪, 显式触发进入 tiles 模式
        s = st()
        print(f"[step] startFn={s.get('startFn')} world={s.get('world')} yopo={s.get('yopo')}")
        if s.get("world") != "object":
            print("[step] calling window.startTilesMode() ...")
            try:
                page.evaluate("() => window.startTilesMode && window.startTilesMode()")
            except Exception as e:
                print(f"[step] startTilesMode call err: {e}")
            # 等 world + yopo 就绪
            t1 = time.time()
            while time.time() - t1 < 180:
                s = st()
                if s.get("world") == "object" and s.get("yopo") == "object":
                    break
                time.sleep(2)
            s = st()
            print(f"[step] after startTilesMode: world={s.get('world')} yopo={s.get('yopo')}")

        # 3) 若采集未自动开始, 显式触发
        s = st()
        if s.get("cds") is None:
            print("[step] calling window.runAutoCollect() ...")
            try:
                page.evaluate("() => window.runAutoCollect && window.runAutoCollect()")
            except Exception as e:
                print(f"[step] runAutoCollect call err: {e}")
            time.sleep(2)
            s = st()
            print(f"[step] after runAutoCollect: cds={json.dumps(s.get('cds'), default=str)[:300]}")

        # 4) 轮询服务端进度
        start = time.time()
        last = -1
        done = False
        while time.time() - start < a.timeout:
            srv = server_status(a.server)
            if srv:
                if srv.get("samples", 0) != last:
                    print(f"[progress] samples={srv['samples']}/{target} maps={srv.get('maps')}")
                    last = srv["samples"]
                if srv["samples"] >= target:
                    done = True
                    break
            s = st()
            cds = s.get("cds")
            if cds and cds.get("error"):
                print(f"[PAGE ERROR] {cds['error']}")
                break
            time.sleep(3)

        if done:
            time.sleep(5)
            print(f"[headless_collect] DONE. status={server_status(a.server)}")

        browser.close()

    if done:
        print("[headless_collect] collection complete.")
        return 0
    print("[headless_collect] collection did NOT complete.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
