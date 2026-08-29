#!/usr/bin/env python3
"""
_yopo_test_flight.py — headless end-to-end test of the YOPO autonomous navigation chain.
Open the flight page (:8080) -> startTilesMode to enter flight -> /yopo/set_goal to set
the goal -> activate drone.yopo_nav -> monitor drone position / inference count /
arrival -> screenshot.
Verifies that YOPO_26 is really invoked and produces a valid trajectory.
"""
import json
import time
import urllib.request
from playwright.sync_api import sync_playwright

APP = "http://localhost:8080"
YOPO = "http://localhost:5689"
GOAL = {"x": 12.0, "y": 0.0, "z": 3.0}   # ENU goal (m): 12 m east of the origin, 3 m high

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

        # 1) Wait until startTilesMode is available
        for _ in range(30):
            try:
                if p.evaluate("() => typeof window.startTilesMode") == "function":
                    break
            except Exception:
                pass
            time.sleep(1)

        # 2) Enter tiles flight
        p.evaluate("() => window.startTilesMode && window.startTilesMode()")

        # 3) Wait until world / drone / yopoDepthFromPanorama are ready
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

        # 4) Set the goal server-side (the frontend navigate call sends no goal
        #    and relies on the server-side set_goal)
        print("set_goal resp:", set_goal(GOAL))
        time.sleep(1)

        # 5) Activate navigation through the real UI flow: set the goal, sync the
        #    dropdown and click the start button. Setting flightMode by hand is not
        #    enough — drone.readSettings() overwrites yopo_nav with the value of the
        #    flight-mode-select dropdown every frame, which would exit the nav loop.
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

        # 6) Monitor for ~80 s
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
        # Displacement
        if traj:
            f = traj[0][1]
            l = traj[-1][1]
            d = ((l["x"]-f["x"])**2 + (l["y"]-f["y"])**2 + (l["z"]-f["z"])**2)**0.5
            print(f"displacement from t0: {d:.1f} m")


if __name__ == "__main__":
    main()
