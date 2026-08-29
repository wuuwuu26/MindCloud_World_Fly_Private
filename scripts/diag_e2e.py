#!/usr/bin/env python3
"""End-to-end diagnosis: real DA360 depth -> YOPO, verify it flies toward the goal (+x).

Pipeline: generate ERP panorama -> DA360 /depth yields relative depth (.npy) -> resize to 384x192
       -> estimate meters (x scale) -> inject into YOPO navigate + control -> inspect trajectory direction.

Note: a purely synthetic panorama fed through DA360 still yields the model's true inferred depth distribution,
       but the scale is relative and must be converted to meters. This script focuses on verifying the
       "direction" points toward the goal.
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
    """Generate an ERP panorama: sky/ground + a front building, with clear left/right asymmetry (left far, right near)."""
    W, H = 384, 192
    img = Image.new('RGB', (W, H))
    px = img.load()
    for y in range(H):
        for x in range(W):
            if y < H // 3:
                c = (110, 150, 215)   # sky
            elif y < H // 3 * 2:
                c = (165, 140, 115)   # mid band
            else:
                c = (85, 70, 50)      # ground
            # Building ahead (center)
            if H // 3 < y < H // 3 * 2 and W // 2 - 40 < x < W // 2 + 40:
                c = (60, 60, 90)
            # Add a near wall on the left half of the image -> right side empty -> network should go right/forward
            if H // 3 < y < H // 3 * 2 and x < 60:
                c = (40, 45, 70)
            px[x, y] = c
    return img


def decode_npy_base64(b64, shape):
    raw = base64.b64decode(b64)
    npy = raw[8:]  # skip magic6+ver2 before magic+version+header_len, simplified (header_len first)
    # Properly parse the .npy header
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
    print(" End-to-end diagnosis: DA360 real depth -> does YOPO fly toward goal (+x)?")
    print("=" * 70)

    # 1. DA360 produces depth
    img = make_pano()
    jpg = io.BytesIO(); img.save(jpg, 'JPEG', quality=80); jpg = jpg.getvalue()
    req = urllib.request.Request(args.da360 + "/depth", data=jpg,
                                 headers={'Content-Type': 'image/jpeg', 'X-DA360-Raw-Depth': '1'})
    with urllib.request.urlopen(req, timeout=60) as r:
        payload = json.loads(r.read().decode())
    shape = payload['raw_depth_shape']
    rel = decode_npy_base64(payload['raw_depth'], shape)
    print(f" DA360 depth {shape[0]}x{shape[1]} inference {payload['timings_ms']['infer_ms']:.0f}ms")

    # 2. Resize to 384x192 (PIL)
    # DA360 depth: relative_to_nearest, min~1.0. Use Image resize (needle uses numpy, not available on host)
    from PIL import Image as I
    # Convert depth to a PIL image (relative values, 0..), scale to 0..255 for resize, then restore
    minv = min(rel); maxv = max(rel); span = max(maxv - minv, 1e-6)
    g = [[int((rel[r * shape[1] + c] - minv) / span * 255) for c in range(shape[1])] for r in range(shape[0])]
    dmg = I.frombytes('L', (shape[1], shape[0]), bytes(v for row in g for v in row))
    dmg_r = dmg.resize((384, 192), I.BILINEAR)
    dvals = [dmg_r.getpixel((x, y)) for y in range(192) for x in range(384)]
    # Restore to relative depth
    def unmap(v): return minv + (v / 255.0) * span
    depth_m = [unmap(v) for v in dvals]
    # Estimate meters: relative_to_nearest means nearest=1.0, others scaled proportionally. Assume nearest~1m, linear map via scale
    scale = 8.0  # Estimate: relative value 8.0 maps to roughly 8 m
    depth_met = [min(d * scale, 20.0) for d in depth_m]
    # Ground (lower half) is relatively near -> keep ground within a reasonable range
    # 3. flip (consistent with frontend: ERP column flip)
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
        print(f" control end point (MC frame x=east,y=up,z=north)=({e.get('x',0):.2f},{e.get('y',0):.2f},{e.get('z',0):.2f})")
        print(f" toward-goal (x increasing) check: {'yes, moving +x' if e.get('x',0) > 0.3 else 'no, not moving +x'}")
    print("=" * 70)
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
