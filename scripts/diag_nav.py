#!/usr/bin/env python3
"""Diagnosis: inject "realistic-style" ERP depth into a running YOPO server and observe whether it flies toward the goal.

The YOPO network degrades to a hover primitive on out-of-distribution trivial inputs like
"all-flat-20m / single wall strip". This script builds an ERP depth distribution closer to
DA360's real output:
  - lower half of image (ground): depth ramps near->far (0.5~8m)
  - upper half of image (sky): far (20m)
  - an optional near wall straight ahead (column center) to test avoidance
It verifies the server outputs a "toward-goal (+x)" trajectory under such input, and whether yaw converges.

Usage: python3 scripts/diag_nav.py [--url http://127.0.0.1:5689]
"""
import argparse
import base64
import json
import math
import struct
import urllib.request

H, W = 192, 384
MAX_D = 20.0


def build_erp(wall_front=False, wall_left=False):
    """Build a depth (meters) close to a real ERP:
      depth d = 3.0 + 0.08*(row - H/2) ... simplified: ground (lower half) near, sky (upper half) far.
      Then overlay a near wall in front / on the left.
    """
    depth = [[0.0] * W for _ in range(H)]
    for r in range(H):
        # Upper half (sky): far 20m; lower half (ground): nearer toward the bottom (closer to the ground below)
        v = r / (H - 1)            # 0=top 1=bottom
        if v < 0.5:
            # sky: far distance
            d = 20.0
        else:
            # ground: from ~1m near the horizon (0.5) down to ~0.3m directly below
            d = 1.2 - 0.7 * (v - 0.5) * 2.0
            d = max(0.3, d)
        for c in range(W):
            depth[r][c] = d

    def put_wall(alpha_deg, dist=2.0, half=16):
        col = int(round(W / 2 + alpha_deg * W / (2 * math.pi)))
        col = max(0, min(W - 1, col))
        hc = int(round(half * W / (2 * math.pi)))
        # Cover the mid pitch (-30°~+10°) region
        r0, r1 = int(H * 0.35), int(H * 0.65)
        for dr in range(r0, r1):
            for dc in range(-hc, hc + 1):
                cc = col + dc
                if 0 <= cc < W:
                    depth[dr][cc] = dist

    if wall_front:
        put_wall(0.0)
    if wall_left:
        put_wall(+60.0)
    if wall_right if False else False:
        pass
    buf = bytearray()
    for r in range(H):
        for c in range(W):
            buf += struct.pack('<f', depth[r][c])
    return bytes(buf)


def call(url, path, payload):
    data = json.dumps(payload).encode()
    req = urllib.request.Request(url + path, data=data,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='http://127.0.0.1:5689')
    args = ap.parse_args()
    url = args.url
    print("=" * 66)
    print(" Diagnosis: realistic-style ERP depth -> does YOPO fly toward goal (+x)?")
    print("=" * 66)
    cases = [
        ('open(flat)',      build_erp()),
        ('wall_front(front wall)', build_erp(wall_front=True)),
        ('wall_left(left wall)',  build_erp(wall_left=True)),
    ]
    for label, depth in cases:
        call(url, '/yopo/set_goal', {'x': 10, 'y': 0, 'z': 0})
        nav = call(url, '/yopo/navigate', {
            'depth': base64.b64encode(depth).decode(),
            'depth_encoding': '32FC1',
            'depth_shape': [H, W],
            'position': {'x': 0, 'y': 2, 'z': 0},
            'velocity': {'x': 0, 'y': 0, 'z': 0},
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
        })
        if 'error' in nav:
            print(f"[{label}] ERROR: {nav['error']}")
            continue
        # advance control for traj_time
        pts = []
        for _ in range(110):
            c = call(url, '/yopo/control', {
                'position': {'x': 0, 'y': 2, 'z': 0},
                'velocity': {'x': 0, 'y': 0, 'z': 0},
                'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
            })
            if 'error' not in c:
                pts.append(c.get('position', {}))
        if pts:
            end = pts[-1]
            # ROS frame: x=forward, y=left (host output is already MC frame? server returns MC frame)
            print(f"[{label}] navigate_yaw={nav.get('yaw',0):.3f}  "
                  f"control_end=({end.get('x',0):.2f},{end.get('y',0):.2f},{end.get('z',0):.2f})")
        else:
            print(f"[{label}] no control output")
    print("=" * 66)


if __name__ == '__main__':
    main()
