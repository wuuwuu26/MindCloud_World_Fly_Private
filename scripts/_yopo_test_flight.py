#!/usr/bin/env python3
"""
_yopo_test_flight.py — 无头实测 YOPO 自主导航端到端链路.
打开飞行页面 (:8080) -> startTilesMode 进入飞行 -> /yopo/set_goal 设目标
-> 激活 drone.yopo_nav -> 监控无人机位置/推理计数/是否到达 -> 截图.
验证 YOPO_26 是否被真实调用并产生有效轨迹.
"""
import json
import time
import urllib.request
from playwright.sync_api import sync_playwright

APP = "http://localhost:8080"
YOPO = "http://localhost:5689"
GOAL = {"x": 12.0, "y": 0.0, "z": 3.0}   # ENU 目标 (m), 距原点 12m 东、3m 高

LAUNCH = [
    "--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
    "--ignore-gpu-blocklist", "--enable-webgl", "--no-sandbox",
    "--disable-dev-shm-usage",
]


def set_goal(g):
    req = urllib.request.Request(
        YOPO + "/yopo/set_goal",
        data=json.dumps(g).encode(), headers={"Content-Type": "application/json"},
        method="POST")
    return json.loads(urllib.request.urlopen(req, timeout=5).read())


def server_status():
    try:
        return json.loads(urllib.request.urlopen(YOPO + "/yopo/status", timeout=3).read())
    except Exception as e:
        return {"err": str(e)}


def main():
    with sync_playwright() as pw:
        b = pw.chromium.launch(headless=True, args=LAUNCH)
        p = b.new_page(viewport={"width": 1280, "height": 800})
        p.on("console", lambda m: print(f"[page:{m.type}] {m.text[:200]}", flush=True))
        p.on("pageerror", lambda e: print(f"[pageerror] {e}", flush=True))

        p.goto(APP, wait_until="domcontentloaded", timeout=60000)

        # 1) 等 startTilesMode 函数就绪
        for _ in range(30):
            try:
                if p.evaluate("() => typeof window.startTilesMode") == "function":
                    break
            except Exception:
                pass
            time.sleep(1)

        # 2) 进入 tiles 飞行
        p.evaluate("() => window.startTilesMode && window.startTilesMode()")

        # 3) 等 world / drone / yopoDepthFromPanorama 就绪
        s = {}
        for _ in range(180):
            try:
                s = p.evaluate("() => ({w: typeof window.world, d: typeof window.drone,"
                               "y: typeof window.yopoDepthFromPanorama})")
            except Exception:
                s = {}
            if s.get("w") == "object" and s.get("d") == "object" and s.get("y") == "object":
                break
            time.sleep(2)
        print("after startTilesMode:", s)

        init = p.evaluate("() => ({x: window.drone.x, y: window.drone.y, z: window.drone.z,"
                          "fm: window.drone.flightMode})")
        print("init drone:", init)

        # 4) 服务端设目标 (前端 navigate 不传 goal, 依赖服务端 set_goal)
        print("set_goal resp:", set_goal(GOAL))
        time.sleep(1)

        # 5) 走正式 UI 流程激活导航: 设目标 + 同步下拉框 + 点击开始按钮。
        #    不能只手动设 flightMode, 否则每帧 drone.readSettings() 会用
        #    下拉框(flight-mode-select)的值覆盖掉 yopo_nav, 导致导航循环退出。
        p.evaluate("""(g) => {
            window.drone.yopoNavTarget = g;
            var ms = document.getElementById('flight-mode-select');
            if (ms) ms.value = 'yopo_nav';
            var btn = document.getElementById('yopo-start-nav-btn');
            if (btn) btn.click();
            else { window.drone.flightMode='yopo_nav'; window.drone.yopoNavActive=true; }
        }""", GOAL)
        print("nav activated via UI, target =", GOAL)

        p.screenshot(path="/tmp/yopo_test_start.png")

        # 6) 监控 ~80s
        traj = []
        for i in range(40):
            time.sleep(2)
            st = p.evaluate("""() => ({
                x: window.drone.x, y: window.drone.y, z: window.drone.z,
                fm: window.drone.flightMode,
                ic: window.drone.yopoInferenceCount,
                dist: window.drone.yopoDistToGoal,
                arr: window.drone.yopoArrived,
                nav: window.drone.yopoNavActive,
                cmd: window.drone.yopoCmdPos,
                src: window.drone.yopoDepthSource,
                unavail: window.drone.yopoDepthUnavailable
            })""")
            sv = server_status()
            traj.append((i, st, sv))
            print(f"t={i*2}s pos=({st['x']:.1f},{st['y']:.1f},{st['z']:.1f}) "
                  f"fm={st['fm']} ic={st['ic']} dist={st['dist']:.1f} arr={st['arr']} "
                  f"src={st.get('src')} unavail={st.get('unavail')} "
                  f"svIc={sv.get('inference_count')}", flush=True)
            if st["arr"]:
                print(">>> ARRIVED at goal <<<")
                break

        p.screenshot(path="/tmp/yopo_test_final.png")
        b.close()

        print("=== SUMMARY ===")
        print("init:", init)
        print("inference happened:", any(t[1]["ic"] > 0 for t in traj))
        print("max inference count:", max((t[1]["ic"] for t in traj), default=0))
        print("arrived:", any(t[1]["arr"] for t in traj))
        # 位移
        if traj:
            f = traj[0][1]
            l = traj[-1][1]
            d = ((l["x"]-f["x"])**2 + (l["y"]-f["y"])**2 + (l["z"]-f["z"])**2)**0.5
            print(f"displacement from t0: {d:.1f} m")


if __name__ == "__main__":
    main()
