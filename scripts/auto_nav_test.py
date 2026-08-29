#!/usr/bin/env python3
"""Automatic closed-loop navigation debug: verify YOPO can "fly toward goal and avoid obstacles" in a controlled scene.

Two diagnostic levels:
  Level A: feed YOPO with "ideal ERP depth" (scene ground-truth depth, no DA360/scale error) directly.
           Validates that server+network can reach the goal + avoid obstacles under ideal input.
  Level B: RGB panorama -> DA360 -> relative depth -> calibrate scale to meters using scene ground truth,
           then feed YOPO. Validates the full DA360 chain (excluding front-end Cesium calibration error).

Scene (world coordinates, MC frame x=east y=up z=north):
  start (0,2,0), goal (20,2,0)
  obstacle wall: plane at x=10, y in [0,12], z in [-4,4]  (drone must detour around it)

Usage: python3 scripts/auto_nav_test.py --level A [--da360 http://127.0.0.1:5688] [--yopo http://127.0.0.1:5689]
"""
import argparse
import base64
import json
import math
import struct
import time
import urllib.request

# ── Scene definition ──
GOAL = (20.0, 2.0, 0.0)
WALL_X = 10.0      # wall plane at x=10
WALL_YMAX = 12.0
WALL_ZMIN, WALL_ZMAX = -4.0, 4.0
ARRIVE = 2.0

H, W = 192, 384
MAX_D = 20.0
PANO_W, PANO_H = 672, 336   # DA360 input panorama resolution


def mc_dir_to_pano_uv(dirx, diry, dirz):
    """MC frame direction -> ERP pixel (matches the front-end panorama shader):
    forward=-z, right=+x, up=+y; yaw=PI-u*2PI, pitch=(v-0.5)*PI."""
    yaw = math.atan2(dirx, -dirz)
    pitch = math.asin(max(-1.0, min(1.0, diry)))
    u = 0.5 - yaw / (2 * math.pi)
    v = 0.5 - pitch / math.pi
    u = u % 1.0
    v = max(0.0, min(1.0, v))
    return u, v


def ray_scene(origin, d):
    """Ray-scene intersection; return nearest distance or None."""
    # wall plane x=WALL_X (perpendicular to x)
    tx = None
    if abs(d[0]) > 1e-9:
        t = (WALL_X - origin[0]) / d[0]
        if t > 0:
            y = origin[1] + t * d[1]
            z = origin[2] + t * d[2]
            if 0.0 <= y <= WALL_YMAX and WALL_ZMIN <= z <= WALL_ZMAX:
                tx = t
    # ground plane y=0
    ty = None
    if abs(d[1]) > 1e-9:
        t = (0.0 - origin[1]) / d[1]
        if t > 0:
            ty = t
    cand = [t for t in (tx, ty) if t is not None]
    return min(cand) if cand else None


def render_scene(pos):
    """Render ERP RGB + ground-truth depth from the drone position (PANO_W x PANO_H)."""
    rgb = [[(0, 0, 0) for _ in range(PANO_W)] for _ in range(PANO_H)]
    depth = [[0.0] * PANO_W for _ in range(PANO_H)]
    for r in range(PANO_H):
        v = r / (PANO_H - 1)
        pitch = (v - 0.5) * math.pi
        for c in range(PANO_W):
            u = c / (PANO_W - 1)
            yaw = math.pi - u * 2 * math.pi
            # world direction (MC: forward=-z, right=+x, up=+y)
            cy, sy = math.cos(yaw), math.sin(yaw)
            cp, sp = math.cos(pitch), math.sin(pitch)
            d = (sy * cp, sp, -cy * cp)  # (x, y, z)
            dist = ray_scene(pos, d)
            if dist is not None:
                depth[r][c] = min(dist, MAX_D)
                # shading: wall=gray, ground=brown
                yh = pos[1] + dist * d[1]
                if yh <= 0.05:
                    rgb[r][c] = (90, 70, 50)     # ground
                else:
                    rgb[r][c] = (110, 110, 130)  # wall
            else:
                depth[r][c] = MAX_D
                rgb[r][c] = (130, 160, 210)      # sky
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
    """Nearest-neighbor resize for a 1D depth array."""
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
    """Calibrate scale against scene ground truth: true distance / DA360 relative depth over several directions."""
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
    """Resize to 384x192, flip, build mask; return (depth_f32bytes, mask_bytes)."""
    dep = resize_nearest(depth_m, rw, rh, W, H)
    # Flip columns (matches the frontend)
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
    print(f" Closed-loop nav test Level={level}  start={pos} goal={goal}")
    print(f" Obstacle wall x={WALL_X}, y in [0,{WALL_YMAX}], z in [{WALL_ZMIN},{WALL_ZMAX}]")
    print("=" * 72)

    steps = 0
    max_steps = 40
    hit_wall = False
    traj = [tuple(pos)]
    while steps < max_steps:
        steps += 1
        # render current view
        rgb, gt_depth = render_scene(tuple(pos))
        if level == 'A':
            # ideal depth: use scene ground truth directly (resize+flip)
            rw, rh = PANO_W, PANO_H
            flat = [gt_depth[r][c] for r in range(rh) for c in range(rw)]
            depth_bytes, mask = yopo_depth_erp(flat, rw, rh)
            scale_note = "ideal"
        else:
            # Level B: DA360 relative depth + scene-ground-truth scale calibration
            rel, shape, dscale = da360_depth(da360, rgb, PANO_W, PANO_H)
            rw, rh = shape[1], shape[0]
            scale = estimate_scale_gt(tuple(pos), rel, rw, rh)
            if scale is None:
                print(f" [step{steps}] scale calibration failed, stopping")
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
        # advance control, take trajectory end displacement
        end = None
        for _ in range(110):  # advance traj_time (~1.67s)
            ctrl = call(yopo, '/yopo/control', {
                'position': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
                'velocity': {'x': 0, 'y': 0, 'z': 0},
                'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
            })
            if 'error' not in ctrl:
                end = ctrl.get('position', {})
        if end is None:
            print(f" [step{steps}] no control trajectory")
            break
        new_pos = [end['x'], end['y'], end['z']]
        dist_goal = math.hypot(new_pos[0] - goal[0], new_pos[1] - goal[1], new_pos[2] - goal[2])
        # Collision check
        if WALL_ZMIN - 0.3 < new_pos[2] < WALL_ZMAX + 0.3 and abs(new_pos[0] - WALL_X) < 0.3 and new_pos[1] <= WALL_YMAX:
            hit_wall = True
        move = math.hypot(new_pos[0] - pos[0], new_pos[1] - pos[1], new_pos[2] - pos[2])
        print(f" step{steps:2d} [{scale_note}] pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
              f"-> ({new_pos[0]:.2f},{new_pos[1]:.2f},{new_pos[2]:.2f}) move={move:.2f} "
              f"dist_goal={dist_goal:.2f} yaw={nav.get('yaw',0):.2f}")
        traj.append(tuple(new_pos))
        if dist_goal < ARRIVE:
            print(f"\n [OK] reached goal! dist_goal={dist_goal:.2f} (steps={steps})")
            break
        if hit_wall:
            print(f"\n [FAIL] hit wall! pos={new_pos}")
            break
        if move < 0.05 and steps > 3:
            print(f"\n [WARN] stalled (movement too small), stopping (pos={new_pos})")
            break
        pos = new_pos
        time.sleep(0.05)

    ok = (not hit_wall) and math.hypot(pos[0]-goal[0], pos[1]-goal[1], pos[2]-goal[2]) < ARRIVE
    print("=" * 72)
    print(f" Result: {'reached goal without hitting wall' if ok else 'not successful'}"
          f" | steps={steps} hit_wall={hit_wall} final_pos={tuple(round(x,1) for x in pos)}")
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
    print("\nSummary:", results)
    return 0 if all(results.values()) else 1


if __name__ == '__main__':
    import sys
    sys.exit(main())
