#!/usr/bin/env python3
"""诊断: 注入"真实风格"ERP 深度到运行中的 YOPO server, 观察是否朝目标飞行.

YOPO 网络对"全 20m 平坦 / 单墙带"这类分布外极简输入会退化选悬停原语。
本脚本构造更接近 DA360 真实输出的 ERP 深度分布:
  - 图像下半 (地面) 深度由近到远渐变 (0.5~8m)
  - 图像上半 (天空) 远 (20m)
  - 目标正前方 (列中心) 可选一面近墙 (测避障)
验证 server 在此类输入下是否输出"朝目标(+x)"的轨迹, 以及 yaw 是否收敛。

用法: python3 scripts/diag_nav.py [--url http://127.0.0.1:5689]
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
    """构造接近真实 ERP 的深度 (米):
      深度值 d = 3.0 + 0.08*(row - H/2) ... 简化: 地面(下半)近, 天空(上半)远.
      再叠加前方/左侧近墙.
    """
    depth = [[0.0] * W for _ in range(H)]
    for r in range(H):
        # 上半(天空): 远 20m; 下半(地面): 越靠下越近 (越接近下方地面)
        v = r / (H - 1)            # 0=顶 1=底
        if v < 0.5:
            # 天空: 距离远
            d = 20.0
        else:
            # 地面: 从水平线(0.5)近距 1m 到正下方渐近 0.3m
            d = 1.2 - 0.7 * (v - 0.5) * 2.0
            d = max(0.3, d)
        for c in range(W):
            depth[r][c] = d

    def put_wall(alpha_deg, dist=2.0, half=16):
        col = int(round(W / 2 + alpha_deg * W / (2 * math.pi)))
        col = max(0, min(W - 1, col))
        hc = int(round(half * W / (2 * math.pi)))
        # 覆盖中段俯仰 (-30°~+10°) 区域
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
    print(" 诊断: 真实风格 ERP 深度 -> YOPO 是否朝目标(+x)飞行")
    print("=" * 66)
    cases = [
        ('open(空旷)',      build_erp()),
        ('wall_front(前墙)', build_erp(wall_front=True)),
        ('wall_left(左墙)',  build_erp(wall_left=True)),
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
        # control 推进 traj_time
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
            # ROS 系: x=前, y=左 (host 直接输出就是 MC 系? server 返回 MC 系)
            print(f"[{label}] navigate_yaw={nav.get('yaw',0):.3f}  "
                  f"control末点=({end.get('x',0):.2f},{end.get('y',0):.2f},{end.get('z',0):.2f})")
        else:
            print(f"[{label}] 无 control 输出")
    print("=" * 66)


if __name__ == '__main__':
    main()
