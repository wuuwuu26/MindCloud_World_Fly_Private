#!/usr/bin/env python3
"""端到端诊断: 真实 DA360 深度 -> YOPO, 验证是否朝目标(+x)飞行.

链路: 生成 ERP 全景图 -> DA360 /depth 得相对深度(.npy) -> resize 到 384x192
       -> 估米(乘 scale) -> 注入 YOPO navigate + control -> 看轨迹方向.

注意: 纯合成全景图经 DA360 得到的深度仍是模型推理的真实分布输出,
      但 scale 是相对值, 需估米。本脚本重在验证"方向"是否朝目标。
"""
import argparse
import base64
import io
import json
import math
import struct
import urllib.request

from PIL import Image


def make_pano(tex_wall_front=True):
    """生成 ERP 全景: 天空/地面 + 前方建筑, 有清晰左右不对称(左远右近)."""
    W, H = 384, 192
    img = Image.new('RGB', (W, H))
    px = img.load()
    for y in range(H):
        for x in range(W):
            if y < H // 3:
                c = (110, 150, 215)   # 天空
            elif y < H // 3 * 2:
                c = (165, 140, 115)   # 中带
            else:
                c = (85, 70, 50)      # 地面
            # 前方建筑(中心)
            if H // 3 < y < H // 3 * 2 and W // 2 - 40 < x < W // 2 + 40:
                c = (60, 60, 90)
            # 左侧(图像左半)加一堵近墙 -> 右侧空 -> 网络应朝右/前
            if H // 3 < y < H // 3 * 2 and x < 60:
                c = (40, 45, 70)
            px[x, y] = c
    return img


def decode_npy_base64(b64, shape):
    raw = base64.b64decode(b64)
    npy = raw[8:]  # 跳过 magic+version+header_len 前的 magic6+ver2, 简化(header_len在前)
    # 正确解析 .npy header
    import struct as st
    off = 6          # magic
    ver_major = raw[off]; ver_minor = raw[off + 1]; off += 2
    if ver_major == 1:
        hlen = st.unpack('<H', raw[off:off + 2])[0]; off += 2
    else:
        hlen = st.unpack('<I', raw[off:off + 4])[0]; off += 4
    off += hlen  # skip header text
    data = raw[off:]
    total = 1
    for d in shape:
        total *= d
    vals = struct.unpack('<%df' % total, data[:total * 4])
    return list(vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--da360', default='http://127.0.0.1:5688')
    ap.add_argument('--yopo', default='http://127.0.0.1:5689')
    args = ap.parse_args()

    print("=" * 70)
    print(" 端到端诊断: DA360 真实深度 -> YOPO 是否朝目标(+x)")
    print("=" * 70)

    # 1. DA360 出深度
    img = make_pano()
    jpg = io.BytesIO(); img.save(jpg, 'JPEG', quality=80); jpg = jpg.getvalue()
    req = urllib.request.Request(args.da360 + "/depth", data=jpg,
                                 headers={'Content-Type': 'image/jpeg', 'X-DA360-Raw-Depth': '1'})
    with urllib.request.urlopen(req, timeout=60) as r:
        payload = json.loads(r.read().decode())
    shape = payload['raw_depth_shape']
    rel = decode_npy_base64(payload['raw_depth'], shape)
    print(f" DA360 深度 {shape[0]}x{shape[1]} 推理 {payload['timings_ms']['infer_ms']:.0f}ms")

    # 2. resize 到 384x192 (PIL)
    # DA360 depth: relative_to_nearest, min≈1.0。用 Image resize (needle 用 numpy, host无)
    from PIL import Image as I
    # 深度转 PIL 图像(相对值, 0..), 用相对值缩放到 0..255 便于 resize, 再还原
    minv = min(rel); maxv = max(rel); span = max(maxv - minv, 1e-6)
    g = [[int((rel[r * shape[1] + c] - minv) / span * 255) for c in range(shape[1])] for r in range(shape[0])]
    dmg = I.frombytes('L', (shape[1], shape[0]), bytes(v for row in g for v in row))
    dmg_r = dmg.resize((384, 192), I.BILINEAR)
    dvals = [dmg_r.getpixel((x, y)) for y in range(192) for x in range(384)]
    # 还原为相对深度
    def unmap(v): return minv + (v / 255.0) * span
    depth_m = [unmap(v) for v in dvals]
    # 估米: relative_to_nearest 即最近处=1.0, 其他按比例。假设最近≈1m, 线性映射 scale
    scale = 8.0  # 估算: 相对值 8.0 ~ 8m
    depth_met = [min(d * scale, 20.0) for d in depth_m]
    # 地面(下半)相对近 -> 确保地面在合理范围
    # 3. flip (与前端一致: ERP 列翻转)
    depth_flip = []
    for r in range(192):
        row = depth_met[r * 384:(r + 1) * 384]
        depth_flip.extend(reversed(row))
    # 4. mask
    mask = [255] * (192 * 384)

    def call(url, path, payload):
        data = json.dumps(payload).encode()
        req2 = urllib.request.Request(url + path, data=data,
                                      headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req2, timeout=30) as r2:
            return json.loads(r2.read().decode())

    def f32_bytes(vals):
        return struct.pack('<%df' % len(vals), *vals)

    call(args.yopo, '/yopo/set_goal', {'x': 10, 'y': 0, 'z': 0})
    nav = call(args.yopo, '/yopo/navigate', {
        'depth': base64.b64encode(f32_bytes(depth_flip)).decode(),
        'mask': base64.b64encode(bytes(mask)).decode(),
        'depth_encoding': '32FC1',
        'depth_shape': [192, 384],
        'position': {'x': 0, 'y': 2, 'z': 0},
        'velocity': {'x': 0, 'y': 0, 'z': 0},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    })
    if 'error' in nav:
        print("NAV ERROR:", nav['error'])
        return 1
    pts = []
    for _ in range(110):
        c = call(args.yopo, '/yopo/control', {
            'position': {'x': 0, 'y': 2, 'z': 0},
            'velocity': {'x': 0, 'y': 0, 'z': 0},
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
        })
        if 'error' not in c:
            pts.append(c.get('position', {}))
    print(f" navigate_yaw={nav.get('yaw',0):.3f}")
    if pts:
        e = pts[-1]
        print(f" control末点(MC系 x东y上z北)=({e.get('x',0):.2f},{e.get('y',0):.2f},{e.get('z',0):.2f})")
        print(f" 朝目标(x增大)判断: {'是,朝x前进' if e.get('x',0) > 0.3 else '否,未朝x前进'}")
    print("=" * 70)
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
