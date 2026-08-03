#!/usr/bin/env python3
"""自动闭环导航调试: 用可控场景验证 YOPO 能否"朝目标飞且避开障碍"。

分两级诊断:
  Level A: 用"理想 ERP 深度"(场景真实深度, 无 DA360/scale 误差) 直接喂 YOPO。
           验证 server+网络 在理想输入下能否朝目标 + 避障。
  Level B: 用 RGB 全景 -> DA360 -> 相对深度 -> 用场景真值标定 scale 转米,
           喂 YOPO。验证完整 DA360 链路 (不含前端 Cesium 标定的误差)。

场景(世界坐标, MC 系 x=东 y=上 z=北):
  起点 (0,2,0), 目标 (20,2,0)
  障碍墙: x=10 的平面, y∈[0,12], z∈[-4,4]  (无人机必须绕开)

用法: python3 scripts/auto_nav_test.py --level A [--da360 http://127.0.0.1:5688] [--yopo http://127.0.0.1:5689]
"""
import argparse
import base64
import json
import math
import struct
import time
import urllib.request

# ── 场景定义 ──
GOAL = (20.0, 2.0, 0.0)
WALL_X = 10.0      # 墙平面 x=10
WALL_YMAX = 12.0
WALL_ZMIN, WALL_ZMAX = -4.0, 4.0
ARRIVE = 2.0

H, W = 192, 384
MAX_D = 20.0
PANO_W, PANO_H = 672, 336   # DA360 输入全景分辨率


def mc_dir_to_pano_uv(dirx, diry, dirz):
    """MC 系方向 -> ERP 像素 (与前端 panorama shader 一致):
    forward=-z, right=+x, up=+y; yaw=PI-u*2PI, pitch=(v-0.5)*PI."""
    yaw = math.atan2(dirx, -dirz)
    pitch = math.asin(max(-1.0, min(1.0, diry)))
    u = 0.5 - yaw / (2 * math.pi)
    v = 0.5 - pitch / math.pi
    u = u % 1.0
    v = max(0.0, min(1.0, v))
    return u, v


def ray_scene(origin, d):
    """射线与场景求交, 返回最近距离或 None."""
    # 墙平面 x=WALL_X (垂直于 x)
    tx = None
    if abs(d[0]) > 1e-9:
        t = (WALL_X - origin[0]) / d[0]
        if t > 0:
            y = origin[1] + t * d[1]
            z = origin[2] + t * d[2]
            if 0.0 <= y <= WALL_YMAX and WALL_ZMIN <= z <= WALL_ZMAX:
                tx = t
    # 地面 y=0
    ty = None
    if abs(d[1]) > 1e-9:
        t = (0.0 - origin[1]) / d[1]
        if t > 0:
            ty = t
    cand = [t for t in (tx, ty) if t is not None]
    return min(cand) if cand else None


def render_scene(pos):
    """从无人机位置渲染 ERP RGB + 真实深度 (PANO_W x PANO_H)."""
    rgb = [[(0, 0, 0) for _ in range(PANO_W)] for _ in range(PANO_H)]
    depth = [[0.0] * PANO_W for _ in range(PANO_H)]
    for r in range(PANO_H):
        v = r / (PANO_H - 1)
        pitch = (v - 0.5) * math.pi
        for c in range(PANO_W):
            u = c / (PANO_W - 1)
            yaw = math.pi - u * 2 * math.pi
            # 世界方向 (MC: forward=-z, right=+x, up=+y)
            cy, sy = math.cos(yaw), math.sin(yaw)
            cp, sp = math.cos(pitch), math.sin(pitch)
            d = (sy * cp, sp, -cy * cp)  # (x, y, z)
            dist = ray_scene(pos, d)
            if dist is not None:
                depth[r][c] = min(dist, MAX_D)
                # 着色: 墙=灰, 地面=棕
                yh = pos[1] + dist * d[1]
                if yh <= 0.05:
                    rgb[r][c] = (90, 70, 50)     # 地面
                else:
                    rgb[r][c] = (110, 110, 130)  # 墙
            else:
                depth[r][c] = MAX_D
                rgb[r][c] = (130, 160, 210)      # 天空
    return rgb, depth


def rgb_to_jpeg(rgb, w, h):
    from PIL import Image
    img = Image.new('RGB', (w, h))
    img.putdata([rgb[r][c] for r in range(h) for c in range(w)])
    import io
    buf = io.BytesIO()
    img.save(buf, 'JPEG', quality=85)
    return buf.getvalue()


def decode_npy(b64, shape):
    raw = base64.b64decode(b64)
    off = 6
    ver = raw[off]; off += 1; off += 1  # ver_major, ver_minor
    if ver == 1:
        hlen = struct.unpack('<H', raw[off:off + 2])[0]; off += 2
    else:
        hlen = struct.unpack('<I', raw[off:off + 4])[0]; off += 4
    off += hlen
    total = 1
    for s in shape:
        total *= s
    return struct.unpack('<%df' % total, raw[off:off + total * 4])


def call(url, path, payload, timeout=60):
    data = json.dumps(payload).encode()
    req = urllib.request.Request(url + path, data=data,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def resize_nearest(src, sw, sh, dw, dh):
    """最近邻 resize 1D 深度数组."""
    out = []
    for y in range(dh):
        sy = min(sh - 1, int(y * sh / dh))
        for x in range(dw):
            sx = min(sw - 1, int(x * sw / dw))
            out.append(src[sy * sw + sx])
    return out


def da360_depth(url, rgb, w, h):
    jpg = rgb_to_jpeg(rgb, w, h)
    req = urllib.request.Request(url + "/depth", data=jpg,
                                 headers={'Content-Type': 'image/jpeg', 'X-DA360-Raw-Depth': '1'})
    with urllib.request.urlopen(req, timeout=60) as r:
        p = json.loads(r.read().decode())
    shape = p['raw_depth_shape']
    rel = decode_npy(p['raw_depth'], shape)
    return rel, shape, p['depth_scale']


def estimate_scale_gt(pos, rel, rw, rh):
    """用场景真值标定 scale: 对几个方向, 真实距离 / DA360 相对深度."""
    ratios = []
    for yaw in (-1.2, -0.6, 0.0, 0.6, 1.2):
        for pitch in (0.0, 0.3, -0.3):
            d = (math.sin(yaw) * math.cos(pitch), math.sin(pitch), -math.cos(yaw) * math.cos(pitch))
            gt = ray_scene(pos, d)
            if gt is None or gt < 0.3:
                continue
            u, v = mc_dir_to_pano_uv(*d)
            px = min(rw - 1, int(u * rw)); py = min(rh - 1, int(v * rh))
            relv = rel[py * rw + px]
            if relv > 1e-3:
                ratios.append(gt / relv)
    if len(ratios) < 3:
        return None
    ratios.sort()
    return ratios[len(ratios) // 2]


def yopo_depth_erp(depth_m, rw, rh):
    """resize 到 384x192, 翻转, 构建 mask, 返回 (depth_f32bytes, mask_bytes)."""
    dep = resize_nearest(depth_m, rw, rh, W, H)
    # 翻转列 (与前端一致)
    fl = []
    for r in range(H):
        row = dep[r * W:(r + 1) * W]
        fl.extend(reversed(row))
    buf = struct.pack('<%df' % (H * W), *fl)
    mask = bytes([255] * (H * W))
    return buf, mask


def run_loop(args, level):
    yopo = args.yopo
    da360 = args.da360
    pos = [0.0, 2.0, 0.0]
    goal = list(GOAL)
    call(yopo, '/yopo/set_goal', {'x': goal[0], 'y': goal[1], 'z': goal[2]})
    print("=" * 72)
    print(f" 闭环导航测试 Level={level}  起点={pos} 目标={goal}")
    print(f" 障碍墙 x={WALL_X}, y∈[0,{WALL_YMAX}], z∈[{WALL_ZMIN},{WALL_ZMAX}]")
    print("=" * 72)

    steps = 0
    max_steps = 40
    hit_wall = False
    traj = [tuple(pos)]
    while steps < max_steps:
        steps += 1
        # 渲染当前视角
        rgb, gt_depth = render_scene(tuple(pos))
        if level == 'A':
            # 理想深度: 直接用场景真值 (resize+flip)
            rw, rh = PANO_W, PANO_H
            flat = [gt_depth[r][c] for r in range(rh) for c in range(rw)]
            depth_bytes, mask = yopo_depth_erp(flat, rw, rh)
            scale_note = "ideal"
        else:
            # Level B: DA360 相对深度 + 场景真值标定 scale
            rel, shape, dscale = da360_depth(da360, rgb, PANO_W, PANO_H)
            rw, rh = shape[1], shape[0]
            scale = estimate_scale_gt(tuple(pos), rel, rw, rh)
            if scale is None:
                print(f" [step{steps}] scale 标定失败, 停止")
                break
            depth_m = [min(max(rel[i], 0.0) * scale, MAX_D) for i in range(rh * rw)]
            depth_bytes, mask = yopo_depth_erp(depth_m, rw, rh)
            scale_note = f"scale={scale:.3f}"

        # navigate
        nav = call(yopo, '/yopo/navigate', {
            'depth': base64.b64encode(depth_bytes).decode(),
            'mask': base64.b64encode(mask).decode(),
            'depth_encoding': '32FC1',
            'depth_shape': [H, W],
            'position': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
            'velocity': {'x': 0, 'y': 0, 'z': 0},
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
        })
        if 'error' in nav:
            print(f" [step{steps}] navigate error: {nav['error']}")
            break
        # control 推进, 取轨迹末端位移
        end = None
        for _ in range(110):  # 推进 traj_time (~1.67s)
            ctrl = call(yopo, '/yopo/control', {
                'position': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
                'velocity': {'x': 0, 'y': 0, 'z': 0},
                'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
            })
            if 'error' not in ctrl:
                end = ctrl.get('position', {})
        if end is None:
            print(f" [step{steps}] 无 control 轨迹")
            break
        new_pos = [end['x'], end['y'], end['z']]
        dist_goal = math.hypot(new_pos[0] - goal[0], new_pos[1] - goal[1], new_pos[2] - goal[2])
        # 碰撞检测
        if WALL_ZMIN - 0.3 < new_pos[2] < WALL_ZMAX + 0.3 and abs(new_pos[0] - WALL_X) < 0.3 and new_pos[1] <= WALL_YMAX:
            hit_wall = True
        move = math.hypot(new_pos[0] - pos[0], new_pos[1] - pos[1], new_pos[2] - pos[2])
        print(f" step{steps:2d} [{scale_note}] pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
              f"-> ({new_pos[0]:.2f},{new_pos[1]:.2f},{new_pos[2]:.2f}) move={move:.2f} "
              f"dist_goal={dist_goal:.2f} yaw={nav.get('yaw',0):.2f}")
        traj.append(tuple(new_pos))
        if dist_goal < ARRIVE:
            print(f"\n ✅ 到达目标! dist_goal={dist_goal:.2f} (steps={steps})")
            break
        if hit_wall:
            print(f"\n ❌ 撞墙! pos={new_pos}")
            break
        if move < 0.05 and steps > 3:
            print(f"\n ⚠️ 停滞(移动过小), 停止 (pos={new_pos})")
            break
        pos = new_pos
        time.sleep(0.05)

    ok = (not hit_wall) and math.hypot(pos[0]-goal[0], pos[1]-goal[1], pos[2]-goal[2]) < ARRIVE
    print("=" * 72)
    print(f" 结果: {'成功到达且未撞墙' if ok else '未成功'}"
          f" | steps={steps} hit_wall={hit_wall} 终位={tuple(round(x,1) for x in pos)}")
    print("=" * 72)
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--level', choices=['A', 'B', 'both'], default='both')
    ap.add_argument('--da360', default='http://127.0.0.1:5688')
    ap.add_argument('--yopo', default='http://127.0.0.1:5689')
    args = ap.parse_args()
    results = {}
    for lv in (['A'] if args.level == 'A' else ['B'] if args.level == 'B' else ['A', 'B']):
        results[lv] = run_loop(args, lv)
    print("\n汇总:", results)
    return 0 if all(results.values()) else 1


if __name__ == '__main__':
    import sys
    sys.exit(main())
