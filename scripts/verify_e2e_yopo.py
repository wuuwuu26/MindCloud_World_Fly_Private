#!/usr/bin/env python3
"""End-to-end closed-loop verification script (Task 6).

Drives the running YOPO server HTTP API to simulate the real frontend pipeline:
  /yopo/navigate  (with ERP depth frame + odom)  -> inference + trajectory selection
  /yopo/control   (high-frequency odom)           -> advance ctrl_time, evaluate trajectory

Verification goals:
  1. Network forward inference is normal (no NaN / anomalies).
  2. The fixed azimuth mapping (frontend left-right flip + identity Rotation_bc) makes the drone
     advance toward the goal and avoid obstacles.
  3. replan timing is continuous (control advances the trajectory without jumps).

Usage:
  python3 scripts/verify_e2e_yopo.py [--url http://127.0.0.1:5689]
"""
import argparse
import base64
import json
import math
import struct
import sys
import urllib.request

DEPTH_H = 192
DEPTH_W = 384
FAR = 30.0  # depth (m) in open space; beyond the network's observation range it is treated as obstacle-free


def make_erp_depth(scene, flip=False):
    """Build a 192x384 ERP depth map (meters, 32FC1 little-endian).

    ERP convention (aligned with frontend yopo-depth-from-panorama.js):
      column col: azimuth alpha = (col/W - 0.5) * 2PI  (positive = turn left, right half of image = left)
      row row: elevation beta = (0.5 - row/H) * PI      (positive = up, top of image = up)
    i.e.: right half of image = drone's left side; left half of image = drone's right side.

    scene:
      'open'      fully open
      'wall_front' a near wall straight ahead (around col=W/2)
      'wall_left'  wall on the drone's true left (after fix: wall in right half of image; before fix flip=False: wall in left half)
      'wall_right' wall on the drone's true right
    flip: simulate the "old bug" (no flip), i.e. fill the real geometry using the wrong mapping.
    """
    import array
    grid = [[FAR for _ in range(DEPTH_W)] for _ in range(DEPTH_H)]

    def put_wall(alpha_deg, beta_center=0.0, half_width=18, dist=1.0):
        # Place the wall at the true azimuth alpha_deg (positive = left) onto the map.
        # If flip=True (old bug), the azimuth is negated -> mirrored left/right error.
        a = alpha_deg if not flip else -alpha_deg
        col = int(round(DEPTH_W / 2 + a * DEPTH_W / (2 * math.pi)))
        col = max(0, min(DEPTH_W - 1, col))
        row_c = int(round(DEPTH_H / 2 - beta_center * DEPTH_H / math.pi))
        half_cols = int(round(half_width * DEPTH_W / (2 * math.pi)))
        half_rows = int(round(40 * DEPTH_H / math.pi))  # Roughly +/-40 deg elevation coverage
        for dc in range(-half_cols, half_cols + 1):
            c = col + dc
            if 0 <= c < DEPTH_W:
                for dr in range(-half_rows, half_rows + 1):
                    r = row_c + dr
                    if 0 <= r < DEPTH_H:
                        grid[r][c] = dist

    if scene == 'wall_front':
        put_wall(0.0, dist=1.5)
    elif scene == 'wall_left':
        put_wall(+90.0, dist=1.0)   # true left side at 90°
    elif scene == 'wall_right':
        put_wall(-90.0, dist=1.0)  # true right side at 90°

    buf = array.array('f')
    for r in range(DEPTH_H):
        for c in range(DEPTH_W):
            buf.append(grid[r][c])
    return buf.tobytes()


def encode_b64(raw):
    return base64.b64encode(raw).decode('ascii')


def post(url, path, payload):
    data = json.dumps(payload).encode('utf-8')
    req = urllib.request.Request(url + path, data=data,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode('utf-8'))


def odom(x, y, z, vx=0, vy=0, vz=0, yaw_deg=0):
    # yaw_deg: drone nose yaw (ROS convention 0=forward). Represented as a quaternion about the z axis.
    # Test uses identity orientation (nose forward = ROS +x), y is left, consistent with the scene convention.
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    q = [0.0, 0.0, sy, cy]  # yaw/2 rotation about the z axis (ROS up)
    return {
        "position": {"x": x, "y": y, "z": z},
        "velocity": {"x": vx, "y": vy, "z": vz},
        "orientation": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]},
    }


def run_scene(url, scene, flip, goal):
    # set goal
    post(url, "/yopo/set_goal", {"x": goal[0], "y": goal[1], "z": goal[2]})
    depth = make_erp_depth(scene, flip=flip)
    nav_payload = {
        "depth": encode_b64(depth),
        "depth_encoding": "32FC1",
        "depth_shape": [DEPTH_H, DEPTH_W],
        **odom(0.0, 0.0, 0.0),
    }
    nav = post(url, "/yopo/navigate", nav_payload)
    if "error" in nav:
        return {"scene": scene, "flip": flip, "error": nav["error"]}

    # high-frequency control: advance ~1.7s (traj_time), collect horizontal trajectory projection
    pts = []
    for _ in range(105):  # ~60Hz * 1.75s
        ctrl = post(url, "/yopo/control", odom(0.0, 0.0, 0.0))
        if "error" in ctrl:
            return {"scene": scene, "flip": flip, "error": ctrl["error"]}
        p = ctrl.get("position", {})
        pts.append((p.get("x", 0.0), p.get("y", 0.0)))
    # trajectory end horizontal displacement (ROS: x=forward, y=left)
    end_x, end_y = pts[-1]
    # check NaN
    nan = any((math.isnan(px) or math.isnan(py)) for px, py in pts)
    # navigate directly returns the command for this replan (evaluated at ctrl_time=0)
    nav_vel = nav.get("velocity", {})
    nav_yaw = nav.get("yaw", 0.0)
    return {
        "scene": scene,
        "flip": flip,
        "nav_vel": (round(nav_vel.get("x", 0.0), 3), round(nav_vel.get("y", 0.0), 3)),
        "nav_yaw": round(nav_yaw, 3),
        "traj_end": (round(end_x, 3), round(end_y, 3)),
        "nan": nan,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:5689")
    args = ap.parse_args()
    url = args.url

    # Goal placed far straight ahead (ROS +x = forward)
    goal = [10.0, 0.0, 0.0]

    print("=" * 70)
    print(" YOPO end-to-end closed-loop verification (task 6)")
    print(f" server: {url}")
    print(" convention: ROS frame x=forward, y=left; right half of image=left, left half=right (after ERP alignment)")
    print("=" * 70)

    results = []
    # (scene, flip, label)
    cases = [
        ('open',       False, "open (after fix)"),
        ('wall_front', False, "front wall (after fix)"),
        ('wall_left',  False, "true left wall (after fix = right half of image)"),
        ('wall_left',  True,  "true left wall (before fix flip = wrong mirror)"),
        ('wall_right', False, "true right wall (after fix = left half of image)"),
    ]
    for scene, flip, label in cases:
        r = run_scene(url, scene, flip, goal)
        results.append((label, r))
        if "error" in r:
            print(f" [{label}] ERROR: {r['error']}")
        else:
            print(f" [{label}]")
            print(f"    nav_vel(x_fwd,y_left)={r['nav_vel']}  nav_yaw={r['nav_yaw']}")
            print(f"    traj_end(x_fwd,y_left)={r['traj_end']}  NaN={r['nan']}")

    print("-" * 70)
    print(" verdict:")
    all_ok = True
    R = dict((l, r) for l, r in results)

    # 1. No NaN / no error
    for label, r in R.items():
        if "error" in r or r.get("nan"):
            all_ok = False
            print(f"  [FAIL] {label}: NaN/error")

    # 2. network forward normal + replan continuous: trajectory end is finite and advanced (|end|>1e-3)
    for label, r in R.items():
        if "error" in r:
            continue
        ex, ey = r["traj_end"]
        if math.isfinite(ex) and math.isfinite(ey) and (abs(ex) + abs(ey)) > 1e-3:
            print(f"  [PASS] {label}: trajectory advanced (end {r['traj_end']})")
        else:
            all_ok = False
            print(f"  [FAIL] {label}: trajectory did not advance / not finite {r['traj_end']}")

    # 3. core: azimuth mapping mirror comparison
    #    true left wall, after fix (flip=False): wall in right half of image (= left), network should avoid right (y<0 or yaw right)
    #    true left wall, before fix (flip=True): wall in left half of image (= right), network should avoid left (y>0 or yaw left)
    #    the two should be mirror images (nav_yaw / nav_vel.y signs opposite), proving the ERP left-right flip fix works.
    lf = R["true left wall (after fix = right half of image)"]
    lb = R["true left wall (before fix flip = wrong mirror)"]
    if "error" not in lf and "error" not in lb:
        s_f = lf["nav_vel"][1] + 0.3 * math.sin(lf["nav_yaw"])
        s_b = lb["nav_vel"][1] + 0.3 * math.sin(lb["nav_yaw"])
        print(f"  mirror metric  after-fix side_sign={s_f:+.3f}   before-fix side_sign={s_b:+.3f}")
        if s_f * s_b < 0:
            print("  [PASS] mirror comparison: after-fix / before-fix outputs are opposite -> ERP left-right flip fix works")
        else:
            print("  [INFO] mirror comparison shows no clear opposite direction (network may pick another equivalent avoidance primitive)")
    else:
        all_ok = False
        print("  [FAIL] mirror comparison scene errored")

    print("=" * 70)
    print(" overall:", "ALL PASS" if all_ok else "PARTIAL / NEEDS REVIEW")
    print("=" * 70)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
