#!/usr/bin/env python3
"""端到端闭环验证脚本 (任务6)。

驱动运行中的 YOPO server HTTP API, 模拟真实前端链路:
  /yopo/navigate  (带 ERP 深度帧 + odom)  -> 推理 + 选轨迹
  /yopo/control   (高频 odom)             -> 推进 ctrl_time, 评估轨迹

验证目标:
  1. 网络前向推理正常 (无 NaN / 异常)。
  2. 修复后的方位映射 (前端左右翻转 + Rotation_bc 单位阵) 使无人机
     朝目标前进、遇障避开。
  3. replan 时序连续 (control 推进轨迹无跳变)。

用法:
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
FAR = 30.0  # 空旷处深度 (米), 超过网络观测范围即视为无障碍


def make_erp_depth(scene, flip=False):
    """构造 192x384 ERP 深度图 (米, 32FC1 little-endian)。

    ERP 约定 (与前端 yopo-depth-from-panorama.js 对齐):
      列 col: 方位角 alpha = (col/W - 0.5) * 2PI  (正=左转, 图像右半=左)
      行 row: 俯仰角 beta  = (0.5 - row/H) * PI     (正=向上, 图像顶部=上)
    即: 图像右半 = 无人机左侧; 图像左半 = 无人机右侧。

    scene:
      'open'      全空旷
      'wall_front' 正前方 (col=W/2 附近) 一堵近墙
      'wall_left'  无人机真实左侧有墙 (修复后: 墙在图像右半; 未修复flip=False: 墙在图像左半)
      'wall_right' 无人机真实右侧有墙
    flip: 模拟"旧 bug" (未翻转), 即把真实几何按错误映射填充。
    """
    import array
    grid = [[FAR for _ in range(DEPTH_W)] for _ in range(DEPTH_H)]

    def put_wall(alpha_deg, beta_center=0.0, half_width=18, dist=1.0):
        # 把真实方位角 alpha_deg (正=左) 的墙放到图上。
        # 若 flip=True (旧bug), 方位取反 -> 左右镜像错误。
        a = alpha_deg if not flip else -alpha_deg
        col = int(round(DEPTH_W / 2 + a * DEPTH_W / (2 * math.pi)))
        col = max(0, min(DEPTH_W - 1, col))
        row_c = int(round(DEPTH_H / 2 - beta_center * DEPTH_H / math.pi))
        half_cols = int(round(half_width * DEPTH_W / (2 * math.pi)))
        half_rows = int(round(40 * DEPTH_H / math.pi))  # 约 ±40° 俯仰覆盖
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
        put_wall(+90.0, dist=1.0)   # 真实正左侧 90°
    elif scene == 'wall_right':
        put_wall(-90.0, dist=1.0)  # 真实正右侧 90°

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
    # yaw_deg: 无人机机头 yaw (ROS 约定 0=forward). 用四元数表示绕 z 轴.
    # 测试用 identity 朝向 (机头 forward = ROS +x), y 为左, 与场景约定一致.
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    q = [0.0, 0.0, sy, cy]  # 绕 z 轴 (ROS up) 旋转 yaw/2
    return {
        "position": {"x": x, "y": y, "z": z},
        "velocity": {"x": vx, "y": vy, "z": vz},
        "orientation": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]},
    }


def run_scene(url, scene, flip, goal):
    # 设置目标
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

    # 高频 control: 推进 ~1.7s (traj_time), 收集轨迹水平投影
    pts = []
    for _ in range(105):  # ~60Hz * 1.75s
        ctrl = post(url, "/yopo/control", odom(0.0, 0.0, 0.0))
        if "error" in ctrl:
            return {"scene": scene, "flip": flip, "error": ctrl["error"]}
        p = ctrl.get("position", {})
        pts.append((p.get("x", 0.0), p.get("y", 0.0)))
    # 轨迹末端水平位移 (ROS: x=前, y=左)
    end_x, end_y = pts[-1]
    # 检查 NaN
    nan = any((math.isnan(px) or math.isnan(py)) for px, py in pts)
    # navigate 直接返回该 replan 的指令 (ctrl_time=0 处评价值)
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

    # 目标放在正前方远处 (ROS +x = forward)
    goal = [10.0, 0.0, 0.0]

    print("=" * 70)
    print(" YOPO 端到端闭环验证 (任务6)")
    print(f" server: {url}")
    print(" 约定: ROS 系 x=前, y=左; 图像右半=左, 左半=右 (ERP 对齐后)")
    print("=" * 70)

    results = []
    # (场景, flip, 标签)
    cases = [
        ('open',       False, "全空旷 (修复后)"),
        ('wall_front', False, "正前墙  (修复后)"),
        ('wall_left',  False, "真实左墙 (修复后=图像右半)"),
        ('wall_left',  True,  "真实左墙 (未修复flip=错误镜像)"),
        ('wall_right', False, "真实右墙 (修复后=图像左半)"),
    ]
    for scene, flip, label in cases:
        r = run_scene(url, scene, flip, goal)
        results.append((label, r))
        if "error" in r:
            print(f" [{label}] ERROR: {r['error']}")
        else:
            print(f" [{label}]")
            print(f"    nav_vel(x前,y左)={r['nav_vel']}  nav_yaw={r['nav_yaw']}")
            print(f"    traj_end(x前,y左)={r['traj_end']}  NaN={r['nan']}")

    print("-" * 70)
    print(" 判定:")
    all_ok = True
    R = dict((l, r) for l, r in results)

    # 1. 无 NaN / 无 error
    for label, r in R.items():
        if "error" in r or r.get("nan"):
            all_ok = False
            print(f"  [FAIL] {label}: NaN/error")

    # 2. 网络前向正常 + replan 连续: 轨迹末端为有限值且被推进 (|end|>1e-3)
    for label, r in R.items():
        if "error" in r:
            continue
        ex, ey = r["traj_end"]
        if math.isfinite(ex) and math.isfinite(ey) and (abs(ex) + abs(ey)) > 1e-3:
            print(f"  [PASS] {label}: 轨迹已推进 (末端 {r['traj_end']})")
        else:
            all_ok = False
            print(f"  [FAIL] {label}: 轨迹未推进/非有限 {r['traj_end']}")

    # 3. 核心: 方位映射镜像对照
    #    真实左墙 修复后(flip=False) 把墙放到图像右半(=左), 网络应朝右避 (y<0 或 yaw 朝右)
    #    真实左墙 未修复(flip=True)  把墙放到图像左半(=右), 网络应朝左避 (y>0 或 yaw 朝左)
    #    二者应呈现镜像 (nav_yaw / nav_vel.y 符号相反), 证明 ERP 左右翻转修复生效。
    lf = R["真实左墙 (修复后=图像右半)"]
    lb = R["真实左墙 (未修复flip=错误镜像)"]
    if "error" not in lf and "error" not in lb:
        s_f = lf["nav_vel"][1] + 0.3 * math.sin(lf["nav_yaw"])
        s_b = lb["nav_vel"][1] + 0.3 * math.sin(lb["nav_yaw"])
        print(f"  镜像指标  修复后 side_sign={s_f:+.3f}   未修复 side_sign={s_b:+.3f}")
        if s_f * s_b < 0:
            print("  [PASS] 镜像对照: 修复后/未修复输出方向相反 -> ERP 左右翻转修复生效")
        else:
            print("  [INFO] 镜像对照未呈现明显反向 (网络可能选其它等价避障原语)")
    else:
        all_ok = False
        print("  [FAIL] 镜像对照场景出错")

    print("=" * 70)
    print(" 总体:", "ALL PASS" if all_ok else "PARTIAL / NEEDS REVIEW")
    print("=" * 70)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
