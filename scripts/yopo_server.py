#!/usr/bin/env python3
"""Flask API for YOPO autonomous drone navigation inference.

Provides a stateful HTTP endpoint that accepts a depth image + odometry
+ goal position and returns a PositionCommand (position, velocity,
acceleration, yaw).  The YOPO neural network runs inside this process;
no ROS dependency.

Key design decisions (aligned with original test_yopo_ros.py):
    - plan_from_reference=True: 新轨迹从上次指令(desire_pos/vel/acc)出发,
      与旧轨迹在衔接点重合 → 轨迹连续、无往复运动。这是原版级联控制的语义
      (轨迹规划 + SO3 位置控制器): 每次深度到达重新推理, 新多项式从当前
      desire 状态开始, 控制端逐帧评估多项式并更新 desire。
    - Replan on every request (each carries a new depth frame), matching
      YOPO_360's 30Hz depth-callback replan rate.
    - ctrl_time advanced by real dt (capped at CTRL_DT=0.02s, then scaled by
      CTRL_TIME_SCALE) in the high-freq /yopo/control endpoint; navigate
      resets it to 0 on replan. Time-scaling makes the drone traverse the same
      planned (collision-avoiding) path faster without altering its shape.
    - Yaw uses calculate_yaw() blending velocity + goal direction.
    - Camera pitch angle is configurable (original default: 0).

Usage:
    python scripts/yopo_server.py --port 5689
    # or via start_yopo_api.sh
"""

import argparse
import base64
import io
import math
import os
import sys
import time
import json
import asyncio
import threading
import concurrent.futures
import numpy as np

# ── YOPO module paths ─────────────────────────────────────────────
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YOPO_DIR = os.path.join(PROJECT_ROOT, 'third_party', 'yopo')
if os.path.isdir(YOPO_DIR):
    sys.path.insert(0, YOPO_DIR)

# ── Dependencies ──────────────────────────────────────────────────
try:
    import torch
    import cv2
    from PIL import Image
except ImportError as exc:
    raise SystemExit(
        "Missing YOPO server dependencies. Install:\n"
        "  pip install torch torchvision numpy pillow opencv-python scipy flask flask-cors"
    ) from exc

try:
    from flask import Flask, jsonify, request
except ImportError as exc:
    raise SystemExit("pip install flask flask-cors") from exc

try:
    from flask_cors import CORS
except ImportError:
    CORS = None

# WebSocket transport (efficient persistent alternative to per-call HTTP).
# Optional: if `websockets` is not installed the WS server is simply skipped
# and the client transparently falls back to plain HTTP.
try:
    import websockets
    HAVE_WEBSOCKETS = True
except Exception:
    HAVE_WEBSOCKETS = False

# ── YOPO imports (after sys.path manipulation) ────────────────────
YOPO_AVAILABLE = False
try:
    from config.config import cfg as yopo_cfg
    from policy.yopo_network import YopoNetwork
    from policy.poly_solver import Poly5Solver, calculate_yaw, wrap_to_pi
    from policy.state_transform import StateTransform
    from policy.primitive import LatticePrimitive
    YOPO_AVAILABLE = True
except Exception as _yopo_import_err:
    _yopo_import_err_msg = str(_yopo_import_err)

# ── Constants ─────────────────────────────────────────────────────
DEFAULT_PORT = 5689


class _TrtYopoModel:
    """TensorRT 推理封装: 从 ONNX 导出的引擎加载, 暴露与 YopoNetwork 同签名
    (depth, obs) -> (endstate, score) 的 __call__, 使推理调用处 (self.policy(...))
    无需改动。用 tensorrt Python API 直接执行引擎 (无需 torch2trt / nvcc)。"""

    def __init__(self, engine_path, device):
        import tensorrt as trt
        self.device = device
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            engine_data = f.read()
        runtime = trt.Runtime(self.logger)
        self.engine = runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()
        self.stream = torch.cuda.Stream()  # 独立 CUDA 流, 避免默认流的性能告警
        self.input_names = []
        self.output_names = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)

    def __call__(self, depth, obs):
        # torch.from_numpy/state_transform 的输出通常已连续; 仅在确实非连续时才拷贝,
        # 避免高频调用下无谓的整张张量复制。
        if not depth.is_contiguous():
            depth = depth.contiguous()
        if not obs.is_contiguous():
            obs = obs.contiguous()
        dev = depth.device
        self.context.set_tensor_address(self.input_names[0], depth.data_ptr())
        self.context.set_tensor_address(self.input_names[1], obs.data_ptr())
        out_bufs = {}
        for name in self.output_names:
            shape = tuple(self.engine.get_tensor_shape(name))
            t = torch.empty(shape, dtype=torch.float32, device=dev)
            out_bufs[name] = t
            self.context.set_tensor_address(name, t.data_ptr())
        # 输入张量由调用方在默认流上写入; 先同步默认流确保数据就绪, 再用独立流执行
        # (避免 TensorRT 在默认流上额外 cudaStreamSynchronize 的性能告警), 最后同步本流。
        torch.cuda.default_stream(dev).synchronize()
        self.context.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()
        return out_bufs[self.output_names[0]], out_bufs[self.output_names[1]]
DEFAULT_WS_PORT = 5690  # WebSocket transport port (0 = disabled)
DEFAULT_MODEL = os.path.join(
    YOPO_DIR, 'saved', 'YOPO_40', 'epoch20.pth'
)

#: MindCloud:  x=east,  y=up,    z=north   (body forward = -z at identity)
#: YOPO/ROS:   x=forward, y=left, z=up     (body forward = +x at identity)
#: At MC identity, drone faces south (-Z), so ROS forward = -MC_Z = south
R_MC_TO_ROS = np.array([
    [0.0, 0.0, -1.0],  # ros_x = -mc_z (forward = south at identity)
    [-1.0, 0.0, 0.0],  # ros_y = -mc_x (left = west at identity)
    [0.0, 1.0, 0.0],   # ros_z =  mc_y (up = up)
], dtype=np.float64)
R_ROS_TO_MC = R_MC_TO_ROS.T

# ERP panorama resolution (YOPO_360). The actual image_height/width and
# image_channels are read from the YOPO config below; these constants are
# only used to validate incoming depth payloads.
DEPTH_HEIGHT = 192
DEPTH_WIDTH = 384
MAX_DIS = 20.0
MIN_DIS = 0.04
CTRL_DT = 0.02  # 50 Hz control loop (matches original YOPO)
# 时间缩放(可选微调): 让 ctrl_time 以 >1 倍速推进, 无人机沿同一条(网络规划的、
# 已避障的)空间轨迹更快走完 -> 巡航速度 ≈ vel_max * CTRL_TIME_SCALE。默认 1.0。
# 真正决定规划速度的是 cfg["velocity"] (见 YOPO_VELOCITY), 本项仅作叠加微调。
# Guard against an empty env string (shell exports "") which would crash float("").
_env_tscale = os.environ.get("YOPO_CTRL_TIME_SCALE")
CTRL_TIME_SCALE = float(_env_tscale) if _env_tscale else 1.0
# 指令速度硬上限(m/s): 反应预算限速器已移除, 此绝对钳制保证任何配置下
# (含 YOPO_VELOCITY / YOPO_CTRL_TIME_SCALE 调大) 指令速度都不超过该值。默认 15。
_env_scap = os.environ.get("YOPO_SPEED_CAP")
YOPO_SPEED_CAP = float(_env_scap) if _env_scap else 15.0
# 轨迹末端延伸(秒): 修"提速→减速→再加速"锯齿的关键。
# 网络轨迹时长远短于深度环重规划间隔 —— 实测 traj_time ≈ 0.67s
#   (radio_range=5 → sgm_time=2*5/6=1.667s; YOPO_VELOCITY=15 → ratio=2.5 →
#    segment_time=1.667/2.5=0.667s), 叠加 CTRL_TIME_SCALE=2 后仅 0.33s 真实时间
#   就把整条轨迹走完。此后 ctrl_time 被 poly_duration 封顶 → 指令冻结在轨迹末端。
# 客户端控制器为 velTarget = clamp(1.0*posErr, ±15) + ffVel (drone.js),
#   冻结后 cmdPos 停止前进、cmdVel 仍是末端速度 → 无人机冲过冻结点 → posErr 转负
#   → 被位置环拉回减速 → 下次重规划再加速 = 锯齿的真因。
# 故轨迹走完后按【末端速度】继续线性外推: 指令位置以末端速度继续前进、速度保持
#   末端速度, 使指令始终位于无人机前方且同速移动, posErr 稳定 ≈ 0, 消除拉回与锯齿。
#   同时指令速度来自网络规划的末端速度而非"当前真实速度", 打破 main.js 注释所述
#   "多项式起点速度=真实速度 → 指令速度≈真实速度 → 趋近 0" 的循环依赖。
# 超出 TRAJ_EXTEND_S 仍无重规划(深度通道异常)则退回冻结行为, 避免无限盲飞。
_env_extend = os.environ.get("YOPO_TRAJ_EXTEND_S")
TRAJ_EXTEND_S = float(_env_extend) if _env_extend else 2.0

# 反应预算限速器已移除: 不再按重规划间隔动态限速。无人机按网络规划速度飞行,
# ctrl_time 推进倍率直接取 CTRL_TIME_SCALE(默认 1.0, 可用 YOPO_CTRL_TIME_SCALE 快进,
# 见 restart_all.sh 当前设为 2 → 实际会飞 2x 网络速度)。避障改由 ① 网络 argmin(score)
# 选出的轨迹 ② 客户端几何反应式势场 ③ DA360 深度安全壳 三层共同保证; 指令 PVA
# 仍按同一 rate 缩放以保持自洽(见 _compute_command)。
def _env_float(name, default):
    """读取环境变量浮点值; 空串/非法值回退 default (避免 float("") 崩溃)。"""
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


# (反应预算限速器常量 REACT_*/GOVERNOR_MIN_V/ALLOW_TIMESCALE_BOOST 已移除:
#  无人机直接按 CTRL_TIME_SCALE 飞行, 避障交由网络 + 客户端几何/DA360 安全壳。)

ARRIVE_THRESHOLD = 2.0  # metres (matches test_yopo_ros.py L132: norm(pos-goal)<2.0)
# 终点接管距离: 网络在 goal_length (2*radio_range=10m) 内目标观测被按 goal_length
# 归一化缩小(state_transform.normalize_obs), lattice 又全是巡航型轨迹(端点速度可
# 达 vel_max≈6m/s), 接近目标时 argmin(score) 反复选出过冲/回头轨迹; 叠加
# plan_from_reference 下参考点越过目标会使目标方向观测翻转 → 目标附近速度/位置
# 来回波动、到不了目标。距目标 FINAL_APPROACH_DIST 内不再用网络推理, 直接规划
# 一条"终端速度/加速度=0、平滑减速停到目标点"的五次多项式。
FINAL_APPROACH_DIST = 12.0  # metres (与客户端 yopoFinalApproachDist 一致)

# ── YOPO 轨迹选择 ──
# 严格遵循 YOPO_360 test_yopo_ros.py: 直接 argmin(score) 选最优轨迹, 不做任何
# 额外的几何避障干预。避障完全由网络在训练期 safety_loss 中学到的 score 提供
# ("学习式避障"), 而非部署端叠加的几何碰撞代价, 与官方部署实现完全一致。
# plan_from_reference=True: 新轨迹从上次指令(desire_pos/vel/acc)出发, 在衔接点
#   与旧轨迹重合 -> 轨迹连续。原版 test settings L440 即 True。
PLAN_FROM_REFERENCE = True
# 启用 calculate_yaw(): 机身平滑转向目标方向, 使目标落入 lattice 扇区覆盖范围。
DEFAULT_LOCK_YAW = False

app = Flask(__name__)
if CORS is not None:
    CORS(app)


class YOPOServer:
    """Wraps YOPO inference in a Flask-friendly singleton.

    Architecture mirrors the original test_yopo_ros.py:
      - callback_depth: runs network inference when depth arrives → builds polynomial
      - control_pub: fixed-rate timer that advances ctrl_time and evaluates polynomial
    In our Flask version, /yopo/navigate combines both: it always evaluates
    the polynomial at the current ctrl_time, and replans when needed.
    """

    def __init__(self, model_path, device="cuda" if torch.cuda.is_available() else "cpu",
                 verbose=False, visualize=False,
                 camera_pitch_deg=0.0, lock_yaw=DEFAULT_LOCK_YAW):
        self.device = device
        self.verbose = verbose
        self.visualize = visualize
        self.camera_pitch_deg = camera_pitch_deg

        # ── YOPO config ──
        if not YOPO_AVAILABLE:
            raise RuntimeError(f"YOPO imports failed: {_yopo_import_err_msg}")
        yopo_cfg["train"] = False
        # Test-time planned speed (modifiable). NOTE: vel_max_train MUST stay
        # fixed so the network's input normalisation matches training — raising
        # `velocity` only scales the *physical* trajectory faster (see
        # policy/primitive.py: vel_max = velocity, normalisation uses vel_max_train).
        # This is the real knob for "fly faster"; override via YOPO_VELOCITY.
        # NOTE: when the env var is unset the shell may export an empty string;
        # guard against float("") by falling back to the config value in that case.
        _env_vel = os.environ.get("YOPO_VELOCITY")
        if _env_vel:
            yopo_cfg["velocity"] = float(_env_vel)
        self.height = yopo_cfg['image_height']
        self.width = yopo_cfg['image_width']
        self.in_channels = int(yopo_cfg['image_channels'])

        self.lock_yaw = bool(lock_yaw)
        self.min_dis = MIN_DIS
        self.max_dis = MAX_DIS

        # ── State ──
        self.goal = np.array([10.0, 0.0, 2.0])
        self.arrive = False

        # Current odometry (updated per navigate call)
        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.quat = np.array([0.0, 0.0, 0.0, 1.0])
        # 最近一次里程计的采集时刻(见 _current_state): 用于把 pos 外推到"当前",
        # 使重规划的多项式起点=无人机真实当前状态, 而非"发请求时的状态"。
        self.odom_time = None

        # Desired trajectory state (updated by polynomial evaluation)
        self.desire_pos = None
        self.desire_vel = None
        self.desire_acc = None
        self.desire_init = False
        self.last_yaw = 0.0

        # Trajectory tracking
        self.ctrl_time = None  # None means no trajectory yet
        self.optimal_poly_x = None
        self.optimal_poly_y = None
        self.optimal_poly_z = None
        self.last_position_cmd = None
        self.poly_duration = None  # 当前多项式的时长(s): 网络轨迹=traj_time, 终点接管=规划时长T
        # 轨迹走完后是否按末端速度外推(见 TRAJ_EXTEND_S): 网络轨迹=是; 终点接管=否
        # (终点接管必须停在目标点, 外推会冲过目标)。
        self.poly_extend = False
        # ctrl_time 的有效推进倍率(纯时间重参数化的 rate): 反应预算限速器已移除,
        # 直接取 CTRL_TIME_SCALE(默认 1.0, 可用 YOPO_CTRL_TIME_SCALE 开启"快进";
        # restart_all.sh 当前设为 2 → 飞 2x 网络速度)。
        self._time_rate = CTRL_TIME_SCALE
        self.last_control_time = None
        self.last_fwd_obstacle_dist = None
        self._lock = threading.Lock()
        self._last_depth_input = None
        self._last_depth_map = None
        self._depth_anomaly = False
        self._last_obs_input = None


        # ── Simple avoidance state ──
        # last_cmd_dir: 3D unit vector in MC world (horizontal) from last avoid() call
        # last_cruise_speed: scalar m/s
        # last_climb_rate: scalar m/s (vertical velocity target)
        # last_target_yaw: target yaw (rad) for smooth yaw tracking

        # ── Transforms & model ──
        if not YOPO_AVAILABLE:
            raise RuntimeError(f"YOPO imports failed: {_yopo_import_err_msg}")

        self.state_transform = StateTransform()
        self.lattice_primitive = LatticePrimitive.get_instance()
        self.traj_time = self.lattice_primitive.segment_time
        self._angles_np = self.lattice_primitive.lattice_angle_node.cpu().numpy()

        # 全向 ERP 全景无相机俯仰偏置: body 系与 camera 系重合。
        # (原版针对前视 pinhole 才需要 camera_pitch; 360 全景下叠加俯仰会
        #  使锚点方向与深度图行位置错配。)
        self.Rotation_bc = np.eye(3)

        # ── Load model ──
        print(f"Loading YOPO model from: {model_path}")
        print(f"Using device: {self.device}")
        print(f"Camera pitch: {self.camera_pitch_deg}°, plan_from_reference: {PLAN_FROM_REFERENCE}, "
              f"lock_yaw: {self.lock_yaw}, in_channels: {self.in_channels}")
        state_dict = torch.load(model_path, map_location=self.device, weights_only=True)
        self.policy = YopoNetwork()
        self.policy.load_state_dict(state_dict)
        self.policy = self.policy.to(self.device)
        self.policy.eval()

        # ── 可选 TensorRT 加速 (README: TensorRT Deployment) ──
        # 仅当 YOPO_USE_TRT=1 且转换好的引擎 yopo_trt.pth 存在时启用; 否则回退
        # PyTorch eager (原生路径)。转换脚本见 scripts/yopo_trt_transfer.py: 把本
        # eager 模型经 ONNX 导出固化为 TensorRT 引擎 (fp16)。Orin NX 上推理仅 1~5ms,
        # 相比 eager 的 100~350ms 大幅提速, 重规划频率随之提升、盲飞段缩短、避障改善。
        # 用 tensorrt 直接加载引擎并包成与 YopoNetwork 同签名 (depth, obs)->(endstate, score)
        # 的推理封装 (_TrtYopoModel), 故推理调用处 (self.policy(...)) 无需改动。
        self.use_trt = False
        trt_path = os.environ.get('YOPO_TRT_PATH') or os.path.join(
            os.path.dirname(model_path), 'yopo_trt.pth')
        if os.environ.get('YOPO_USE_TRT', '0').lower() in ('1', 'true', 'yes'):
            if not os.path.isfile(trt_path):
                print(f"[TensorRT] 未找到引擎 {trt_path}; 请先运行 scripts/yopo_trt_transfer.py 生成, 回退 eager")
            else:
                try:
                    self.policy = _TrtYopoModel(trt_path, self.device)
                    self.use_trt = True
                    print(f"[TensorRT] 已加载 {trt_path} — 推理加速启用")
                except Exception as e:
                    print(f"[TensorRT] 加载失败 ({e}); 回退 PyTorch eager")
        # Warmup does one dummy forward pass so the first real inference isn't
        # penalised by lazy CUDA/JIT init. Skippable via YOPO_NO_WARMUP=1 to
        # make restarts faster (first navigate will pay the small init cost).
        if os.environ.get("YOPO_NO_WARMUP", "0") == "1":
            print("Skipping YOPO warmup (YOPO_NO_WARMUP=1).")
        else:
            self._warm_up()
        print(f"YOPO model loaded. Traj time: {self.traj_time:.2f}s, "
              f"Traj num: {self.lattice_primitive.traj_num}, "
              f"vel_max: {self.lattice_primitive.vel_max:.1f}, "
              f"acc_max: {self.lattice_primitive.acc_max:.1f}")
        # Timing stats
        self.time_forward = 0.0
        self.time_prepare = 0.0
        self.time_process = 0.0
        self.count = 0

    def _warm_up(self):
        depth = torch.zeros((1, self.in_channels, self.height, self.width),
                            dtype=torch.float32, device=self.device)
        obs = torch.zeros((1, 9), dtype=torch.float32, device=self.device)
        obs = self.state_transform.prepare_input(obs)
        with torch.inference_mode():
            endstate_pred, score_pred = self.policy(depth, obs)
        _ = self.state_transform.pred_to_endstate(endstate_pred)

    def set_goal(self, x, y, z):
        self.goal = np.array([float(x), float(y), float(z)])
        self.arrive = False
        self.ctrl_time = None
        self.desire_init = False
        self.optimal_poly_x = None
        self.optimal_poly_y = None
        self.optimal_poly_z = None
        print(f"New goal: ({x:.1f}, {y:.1f}, {z:.1f})")
        # Reset simple avoidance state on new goal
        # Reset 终点平滑, 避免上一目标的方向惯性
        self._last_end_xy = None

    # ═══════════════════════════════════════════════════════════════
    #  简化避障算法 (Simple Reactive Obstacle Avoidance)
    #
    #  原理:
    #    1. ERP全景深度图 (384x192), 列W/2=body前方, 列W/4=body左侧, 列3W/4=body右侧
    #    2. 在水平条带 (rows H/2±15) 扫描 -90°→+90° 方向, 每个方向取patch最小深度
    #    3. 前方畅通(>SAFE_DIST): 朝目标方向飞
    #    4. 前方受阻: 选最优方向 = argmax(clear × gauss(angle-goal_angle, σ=60°))
    #    5. 全方向受阻(<EMERGENCY_DIST): 紧急爬升
    #    6. P速度控制器: target_pos = pos + cmd_dir*lookahead, target_vel = cmd_dir*cruise_speed
    #
    #  坐标系:
    #    MC: x=east, y=up, z=north. body forward at identity = -z (south)
    #    ROS: x=forward, y=left, z=up
    #    body forward in MC world = R_ROS_TO_MC @ R_wc_ros @ [1,0,0]
    #    body right  in MC world = R_ROS_TO_MC @ R_wc_ros @ [0,-1,0]
    #    Panorama column k → azimuth delta = (k - W/2) * 2π/W (positive = right/CW)
    # ═══════════════════════════════════════════════════════════════

    # ── Simple avoidance tunables ──
    # 深度更新~2.6Hz → 380ms/帧。在3m/s下无人机每帧移动1.1m。
    # SAFE_DIST必须 >> 1.1m + 刹车距离(~2m) + 余量 → 用15m。
    _SA_SAFE_DIST = 15.0       # m: 前方畅通阈值
    _SA_EMERGENCY_DIST = 5.0   # m: 紧急爬升阈值
    _SA_CRUISE_SPEED = 3.0     # m/s: 巡航速度(降低以增加反应时间)
    _SA_LOOKAHEAD = 4.0        # m: target_pos = pos + cmd_dir * lookahead
    _SA_BAND_HALF_H = 25       # rows: 水平条带半高 (±25 rows ≈ ±23° around horizon)
    _SA_PATCH_HALF_W = 14      # cols: 每个方向patch半宽 (~13° wide)
    _SA_SCAN_ANGLES = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]
    _SA_GOAL_SIGMA_DEG = 50.0  # 高斯权重σ: 越小越偏向目标方向
    _SA_YAW_SMOOTH = 0.25      # yaw低通滤波(降低以减少抖动)
    _SA_DIR_SMOOTH = 0.4       # cmd_dir低通滤波(降低以减少抖动)
    _SA_CLIMB_KP = 0.5         # 高度P增益
    _SA_CLIMB_MAX = 3.0        # m/s: 最大爬升/下降率
    _SA_EMERGENCY_CLIMB = 3.0  # m/s: 紧急爬升率
    _SA_MIN_SPEED = 0.5        # m/s: 障碍附近最低速度(保持机动性)
    _SA_DEPTH_AGE_WARN = 0.2   # s: 深度过期警告阈值, 开始减速
    _SA_DEPTH_AGE_STOP = 1.0   # s: 深度过期停车阈值, 完全停车
    _SA_MAX_ALT_ABOVE_GOAL = 100.0  # m: 最大超过目标高度, 超过则强制下降
    _SA_MAX_EMERGENCY_FRAMES = 8    # 连续紧急帧数上限, 超过则盲飞

    @staticmethod
    def _quat_mc_to_ros(quat):
        """Convert MindCloud quaternion [x,y,z,w] to ROS/YOPO quaternion."""
        from scipy.spatial.transform import Rotation as R
        R_mc = R.from_quat(quat).as_matrix()
        R_ros = R_MC_TO_ROS @ R_mc @ R_ROS_TO_MC
        return R.from_matrix(R_ros).as_quat()

    @staticmethod
    def _vec_mc_to_ros(v):
        return R_MC_TO_ROS @ np.asarray(v, dtype=np.float64)

    @staticmethod
    def _vec_ros_to_mc(v):
        return R_ROS_TO_MC @ np.asarray(v, dtype=np.float64)

    def _process_odom(self):
        """Build normalised observation vector from odometry + goal.

        条件状态必须与多项式起点一致(都用真实里程计), 否则网络预测的末端
        位移(相对量)会从"它以为的起点(desire_*)"算起, 而多项式却从真实起点
        起步 → 衔接错位。故 vel/goal 均用 self.vel/self.pos (真实), acc 用
        self.desire_acc(与多项式起点加速度一致)。
        """
        from scipy.spatial.transform import Rotation as R

        # Convert MindCloud body orientation to ROS body orientation
        quat_ros = self._quat_mc_to_ros(self.quat)
        Rotation_wb = R.from_quat(quat_ros).as_matrix()
        Rotation_wc = np.dot(Rotation_wb, self.Rotation_bc)
        Rotation_cw = Rotation_wc.T

        # Velocity: 真实当前速度(与多项式起点一致)
        vel_w = self._vec_mc_to_ros(self.vel)

        # Acceleration: 最近指令加速度(=多项式起点加速度估计)
        if self.desire_acc is not None:
            acc_w = self._vec_mc_to_ros(self.desire_acc)
        else:
            acc_w = np.zeros(3)

        # Goal: 相对真实当前位置
        goal_w = self._vec_mc_to_ros(self.goal - self.pos)

        vel_c = np.dot(Rotation_cw, vel_w)
        acc_c = np.dot(Rotation_cw, acc_w)
        goal_c = np.dot(Rotation_cw, goal_w)

        obs = np.concatenate((vel_c, acc_c, goal_c), axis=0).astype(np.float32)
        obs_norm = self.state_transform.normalize_obs(torch.from_numpy(obs[None, :]))
        return obs_norm, Rotation_wc

    def _run_inference(self, Rotation_wc):
        """Run YOPO network inference and transform output to world frame.

        SLOW (1-2s on DA360). Does NOT touch shared polynomial/ctrl_time
        state, so it runs OUTSIDE the lock — control requests stay
        responsive during inference.

        Returns:
            endstate_w_ros: [1, 3, 3] array in ROS world frame
        """
        # Network inference
        endstate_pred, score_pred = self.policy(self._last_depth_input, self._last_obs_input)
        endstate_pred = endstate_pred.cpu().numpy()
        score_pred = score_pred.cpu().numpy()

        endstate, score = self._process_output(endstate_pred, score_pred)

        # endstate shape [1, 9] in body(camera) frame: [px,py,pz,vx,vy,vz,ax,ay,az]
        endstate_c = endstate.reshape(-1, 3, 3).transpose(0, 2, 1)  # [1, 3, 3]
        endstate_w_ros = np.matmul(Rotation_wc, endstate_c)  # [1, 3, 3]
        return endstate_w_ros

    def _current_state(self, max_dt=1.0):
        """返回 (pos, vel): 无人机【当前时刻】的位置与速度(一阶外推)。

        为什么不能直接用 self.pos: navigate() 是阻塞长任务(深度清洗 + 网络推理),
        而 WS transport 在单线程 asyncio 事件循环里【同步】调用它(_ws_handler →
        _ws_handle_message → srv.navigate), 期间同一条 WS 连接上的 control 消息
        全部排队饿死 → self.pos 冻结在 navigate() 开头(客户端发请求那一刻)采集的
        值。直接拿它当多项式起点, 轨迹是从"发请求时的位置"起步, 而无人机此时已
        前进 v*dt —— 起点落在机体后方, 指令位置瞬间回跳 → 位置环把无人机往回拉
        → 减速 → 下次重规划再加速(速度锯齿)。

        故以最近一次里程计的时间戳为基准, 用速度做一阶外推得到真正的当前位置,
        保证重规划的多项式起点 C0=当前位置、C1=当前速度。

        注: 网络观测 obs 仍用"深度采集时刻"的状态(_process_odom 在推理前调用),
        这是正确的 —— 深度图与当时的位姿对应; 只有【轨迹起点】需要外推到当前。

        max_dt 上限防止里程计长期不更新(暂停/断流)时外推失控。
        """
        pos = self.pos.copy()
        vel = self.vel.copy()
        if self.odom_time is not None:
            dt = min(max(time.time() - self.odom_time, 0.0), max_dt)
            pos = pos + vel * dt
        return pos, vel

    # 反应预算限速器(_safe_speed)已移除: 不再按重规划间隔计算 v_safe 来限
    # 速。ctrl_time 推进倍率由 _build_polynomial 直接取 CTRL_TIME_SCALE。

    def _build_polynomial(self, endstate_w_ros):
        """Build polynomial trajectory from inference output. FAST (~1ms).

        严格对齐 YOPO_360 test_yopo_ros.py _run_inference (L238-L250):
          - start_pos/vel 由 plan_from_reference 决定 (True=上次指令 desire)
          - 3D 导航: 不做 z 轴水平面投影, 完全信任网络预测的 z 终端状态
            (垂直锚点 β 与 z 轴 PVA 由网络根据深度场景选择, 支持上/下避障)
          - 三轴 Poly5Solver, 无任何缩放/平滑/速度干预
        """
        # 连续性(关键): 新轨迹必须从上一条轨迹"当前真实状态"平滑接出。
        # 深度推理慢(1-2s)时, 两次规划之间 ctrl_time 触顶、无人机沿旧轨迹末端
        # 直线滑行, 参考状态 desire_* 会冻结在旧轨迹末端并滞后于真实位置。
        # 若仍从 desire_* 起步, 新指令会猛拉回滞后点 → 衔接处跳动/不连续。
        # 故起点用无人机【真实里程计状态】外推到当前时刻, 保证规划前后
        # 位置 C0、速度 C1 连续; 加速度取最近指令加速度(self.desire_acc,
        # 飞行/滑行期间≈真实加速度)作为 C2 估计。
        # 注意: 不能直接 self.pos —— 它在 navigate() 阻塞期间会冻结成陈旧值
        # (详见 _current_state 注释), 必须按里程计时间戳外推到"当前"。
        start_pos_mc, start_vel_mc = self._current_state()
        start_pos_ros = self._vec_mc_to_ros(start_pos_mc)
        start_vel_ros = self._vec_mc_to_ros(start_vel_mc)
        start_acc_ros = self._vec_mc_to_ros(self.desire_acc) if self.desire_acc is not None else np.zeros(3)

        # 3D 导航: 完全信任网络预测的 z(垂直)终端状态 —— 不强制终点高度=目标高度,
        # 不强制垂直速度/加速度=0。网络输入 obs 已含"目标相对位置(含垂直分量)",
        # 网络会自行权衡水平推进与垂直机动(上/下避障, 垂直锚点 β), 这正是 YOPO 的
        # 3D 能力。早期曾强制 fixed_height=goal[1] 并清零垂直 PVA, 效果是"把 3D
        # 导航降级为 2D": 目标高度与当前高度差距大时, 整条轨迹被纵向拖拽, 且网络
        # 预测的垂直位移/速度被丢弃 → 无人机"一开始就往目标高度冲/不动"。删除后
        # 恢复网络原生的 3D 轨迹(与 docstring 及官方 test_yopo_ros.py 完全一致)。

        # ctrl_time 推进倍率 = CTRL_TIME_SCALE。反应预算限速器已移除: 不再按
        # v_safe/vel_max 把 rate 压到 <1 来动态限速, 无人机按网络规划速度飞行
        # (默认 1.0, 可用 YOPO_CTRL_TIME_SCALE 开启"快进")。
        # 指令 PVA 仍按同一 rate 缩放(见 _compute_command), 保证位置/速度/加速度自洽。
        self._time_rate = CTRL_TIME_SCALE

        self.optimal_poly_x = Poly5Solver(
            start_pos_ros[0], start_vel_ros[0], start_acc_ros[0],
            endstate_w_ros[0, 0, 0] + start_pos_ros[0],
            endstate_w_ros[0, 0, 1],
            endstate_w_ros[0, 0, 2],
            self.traj_time
        )
        self.optimal_poly_y = Poly5Solver(
            start_pos_ros[1], start_vel_ros[1], start_acc_ros[1],
            endstate_w_ros[0, 1, 0] + start_pos_ros[1],
            endstate_w_ros[0, 1, 1],
            endstate_w_ros[0, 1, 2],
            self.traj_time
        )
        self.optimal_poly_z = Poly5Solver(
            start_pos_ros[2], start_vel_ros[2], start_acc_ros[2],
            endstate_w_ros[0, 2, 0] + start_pos_ros[2],
            endstate_w_ros[0, 2, 1],
            endstate_w_ros[0, 2, 2],
            self.traj_time
        )
        # ctrl_time 从 0 起步(不保留进度): 保证指令位置/速度/加速度 C0/C1/C2 连续。
        # 原因(实测): 若把 ctrl_time 保留到上一条轨迹的进度, navigate 重建新轨迹后
        # _compute_command 会直接从【新轨迹中段】采样 → 指令位置在重规划瞬间大幅跳变
        # (实测重规划间最大跳变 10.4m), 无人机运动不连续。改为从 0 起步后, 新轨迹
        # 起点=当前真实位置/速度/加速度(line 上面的 start_*), 指令从当前位置连续
        # 接出, 重规划间跳变降到 ~2m。巡航速度由 CTRL_TIME_SCALE 与 YOPO_VELOCITY
        # 提供, 控制环(50Hz)从 0 快速推进到中段, 不影响连续性。
        # 终点接管 _plan_final_approach 独立设置 ctrl_time=0, 与此一致。
        self.ctrl_time = 0.0
        self.poly_duration = self.traj_time
        self.poly_extend = True

    def _make_final_approach_polys(self):
        """构建终点接管(到目标点的五次多项式), 返回 (px, py, pz, T, start_pos_ros)。"""
        # 与 _build_polynomial 一致: 起点用【真实里程计状态】, 保证与上一阶段
        # (网络规划)轨迹在衔接处 C0/C1/C2 连续, 不出现拉回式跳动。
        # 与 _build_polynomial 一致: 起点取【当前时刻】状态(见 _current_state)。
        start_pos_mc, start_vel_mc = self._current_state()
        start_acc_mc = self.desire_acc.copy() if self.desire_acc is not None else np.zeros(3)

        # 时长: 平均速度 ≤ vel_max 且留出减速时间, 夹在 [0.8, 3.0]s
        dist = float(np.linalg.norm(self.goal - start_pos_mc))
        vel_max = float(self.lattice_primitive.vel_max)
        T = float(np.clip(1.2 * dist / max(vel_max, 0.1), 0.8, 3.0))
        self._fa_T = T

        start_pos_ros = self._vec_mc_to_ros(start_pos_mc)
        start_vel_ros = self._vec_mc_to_ros(start_vel_mc)
        start_acc_ros = self._vec_mc_to_ros(start_acc_mc)
        end_pos_ros = self._vec_mc_to_ros(self.goal)

        px = Poly5Solver(start_pos_ros[0], start_vel_ros[0], start_acc_ros[0], end_pos_ros[0], 0.0, 0.0, T)
        py = Poly5Solver(start_pos_ros[1], start_vel_ros[1], start_acc_ros[1], end_pos_ros[1], 0.0, 0.0, T)
        pz = Poly5Solver(start_pos_ros[2], start_vel_ros[2], start_acc_ros[2], end_pos_ros[2], 0.0, 0.0, T)
        return px, py, pz, T, start_pos_ros

    def _plan_final_approach(self):
        """终点接管: 距目标较近时, 不用网络推理, 直接规划一条从当前参考状态
        (desire_pos/vel/acc) 到目标点、终端速度/加速度为 0 的五次多项式。

        目的: 消除网络在 goal_length=2*radio_range=10m 内的近目标失稳(目标观测
        被归一化缩小 + 巡航型 lattice 无停车轨迹 → 过冲/回头振荡)。终点多项式
        保证平滑减速并精确停在目标点, 无人机进入 ARRIVE_THRESHOLD(2m) 后由
        arrive 判定接管。

        与 _build_polynomial 保持一致: 多项式建立在 ROS 系, 由 _compute_command
        统一转回 MindCloud 系; 时长 T 由剩余距离和 vel_max 决定, 并写入
        poly_duration 供 ctrl_time 封顶使用(否则 60Hz 控制环会在 traj_time
        处截断, 终点多项式到不了目标)。
        """
        px, py, pz, T, _ = self._make_final_approach_polys()
        self.optimal_poly_x, self.optimal_poly_y, self.optimal_poly_z = px, py, pz
        self.ctrl_time = 0.0
        self.poly_duration = T
        # 终点接管禁止外推: 必须精确停在目标点(外推会冲过目标)。
        self.poly_extend = False

    def _preprocess_depth(self, depth_raw, mask_raw=None):
        """Normalize depth to [0,1], build validity mask, return (1, C, H, W) array.

        Mirrors test_yopo_ros.py _preprocess_depth:
          - invalid pixels (NaN, <min, or mask==0) are replaced with the
            panorama-mean of valid pixels (cheap, sim2real-safe; replaces the
            old cv2.inpaint which is slow and not ERP-aware).
          - 2-channel stack [depth, valid] when in_channels >= 2.
        """
        if depth_raw.shape[0] != self.height or depth_raw.shape[1] != self.width:
            depth_raw = cv2.resize(depth_raw, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
            if mask_raw is not None:
                mask_raw = cv2.resize(mask_raw, (self.width, self.height), interpolation=cv2.INTER_NEAREST)

        # 深度可信度检测已移除: 原版的深度异常悬停启发式在城市楼群等真实环境
        # 会误判"近距像素多"为深度失败而频繁悬停, 直接阻止正常导航 (用户要求
        # 严格对齐原版, 原版无此逻辑)。深度有效性交由 mask 通道与网络自行判断。

        # ── 临时诊断: 低频打印前端传来的深度分布 (确认 DA360 scale) ──
        self._diag_frames = getattr(self, '_diag_frames', 0) + 1
        if self._diag_frames % 30 == 1:
            fv = depth_raw[np.isfinite(depth_raw) & (depth_raw > 0.01)]
            if fv.size:
                print(f"[深度诊断] min={float(fv.min()):.2f} med={float(np.median(fv)):.2f} "
                      f"max={float(fv.max()):.2f} <2m={(fv < 2.0).mean():.2f} <5m={(fv < 5.0).mean():.2f}")

        depth = np.minimum(depth_raw, self.max_dis) / self.max_dis
        nan_mask = np.isnan(depth) | (depth < self.min_dis / self.max_dis)

        if mask_raw is not None:
            valid = (mask_raw > 127).astype(np.uint8)
        else:
            valid = (~nan_mask).astype(np.uint8)

        # ── 无效深度填充: 严格对齐原版 test_yopo_ros.py ──
        # 无效像素统一填"有效像素均值", 有效性信息通过 mask 通道(channel 1)
        # 交给网络自行判断 —— 这是训练时的输入约定。
        # 不可把无效区填成近距障碍: 那会与 mask 语义冲突(网络已从 mask 得知
        # 该处无效), 等于人为制造虚假近障, 使代价场失真。
        invalid = nan_mask | (valid == 0)
        if invalid.any():
            fill = float(depth[~invalid].mean()) if (~invalid).any() else 1.0
            depth = np.where(invalid, np.float32(fill), depth)
        depth = depth.astype(np.float32)

        if self.in_channels >= 2:
            stacked = np.stack([depth, valid.astype(np.float32)], axis=0)  # (2, H, W)
        else:
            stacked = depth[np.newaxis, ...]  # (1, H, W)
        return stacked.reshape(1, self.in_channels, self.height, self.width)

    @torch.inference_mode()
    def navigate(self, depth_bytes, depth_encoding, pos, vel, quat, mask_bytes=None):
        """Stateful YOPO inference.

        Mirrors the original two-thread architecture:
          - Inference (callback_depth): runs when depth arrives, builds polynomial
          - Control (control_pub): fixed 50Hz, advances ctrl_time, evaluates polynomial

        Here both happen in one call, but we advance ctrl_time by CTRL_DT
        (fixed step) instead of real elapsed time, matching the original.

        Args:
            mask_bytes: optional raw bytes of a uint8 (mono8) validity mask,
                        same HxW as depth. 255 = valid, 0 = invalid. When
                        provided and in_channels >= 2, the mask is fed to the
                        network as the second channel.
        """
        time0 = time.time()
        now = time0

        # ── 0. Update odometry (先于深度处理, 深度清洗需要高度信息) ──
        self.pos = np.array(pos, dtype=np.float64)
        self.vel = np.array(vel, dtype=np.float64)
        self.quat = np.array(quat, dtype=np.float64)
        self.odom_time = time.time()

        # ── 1. Depth processing (ERP mean-fill + optional mask) ──
        if depth_encoding == "32FC1":
            depth_raw = np.frombuffer(depth_bytes, dtype=np.float32).reshape(self.height, self.width)
        elif depth_encoding == "16UC1":
            depth_raw = np.frombuffer(depth_bytes, dtype=np.uint16).reshape(self.height, self.width).astype(np.float32) / 1000.0
        else:
            depth_raw = np.frombuffer(depth_bytes, dtype=np.float32).reshape(self.height, self.width)

        mask_raw = None
        if mask_bytes is not None and self.in_channels >= 2:
            mask_raw = np.frombuffer(mask_bytes, dtype=np.uint8).reshape(self.height, self.width)

        depth = self._preprocess_depth(depth_raw, mask_raw=mask_raw)
        # 缓存 CPU 侧归一化深度(channel 0), 供 _process_output 计算前方障碍距离,
        # 避免每次推理都把 GPU 上的整幅深度图 .cpu().numpy() 回拷。
        self._last_depth_map_np = depth[0, 0]
        time1 = time.time()

        if not self.desire_init:
            self.desire_pos = self.pos.copy()
            self.desire_vel = self.vel.copy()
            self.desire_acc = np.zeros(3)
            # Initialize last_yaw from the drone's ACTUAL heading (ROS yaw) so
            # that lock_yaw holds the current heading. Hard-coding 0 would
            # force the drone to yaw to south (0°) on entering yopo_nav.
            from scipy.spatial.transform import Rotation as R
            quat_ros = self._quat_mc_to_ros(self.quat)
            self.last_yaw = float(R.from_quat(quat_ros).as_euler('ZYX', degrees=False)[0])
            self.desire_init = True

        # ── 参考状态同步 (plan_from_reference) ──
        # 原版语义: 新轨迹从上次指令 desire_pos 出发, 保证轨迹连续。
        # 但当无人机被外力/手动大幅移动(如直接拖到高空), desire_pos 仍是旧
        # 轨迹参考, 与实际位置严重脱节, 会导致:
        #   - goal 相对量失真 (目标方向算错, 如明明该下降却看到"目标在头顶")
        #   - cmd 位置与实际位置相差数百米, 无人机被往错误方向猛拉
        # 检测到参考偏差超过一个轨迹跨度时, 把参考状态重置为实际 odometry,
        # 使网络基于"无人机真实位置到目标"的相对量规划, 高度/方向均正确。
        if self.desire_pos is not None:
            drift = np.linalg.norm(self.pos - self.desire_pos)
            if drift > self.traj_time * self.lattice_primitive.vel_max + 5.0:
                self.desire_pos = self.pos.copy()
                self.desire_vel = self.vel.copy()
                self.desire_acc = np.zeros(3)
                print(f"[参考同步] desire_pos 与实位偏差 {drift:.0f}m, 重置到实际位置")

        # ── 2b. Arrival check ──
        dist_to_goal = float(np.linalg.norm(self.pos - self.goal))

        if dist_to_goal < ARRIVE_THRESHOLD:
            self.arrive = True

        if self.arrive:
            # Arrived: return goal with zero velocity, client PD hold takes over
            return {
                "position": {"x": float(self.goal[0]), "y": float(self.goal[1]), "z": float(self.goal[2])},
                "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
                "yaw": float(self.last_yaw),
                "yaw_dot": 0.0,
                "arrived": True,
                "dist_to_goal": dist_to_goal,
                "ctrl_time": 0.0,
            }

        # ── 2c. Final approach: 距目标较近时跳过慢推理, 直接多项式减速到目标 ──
        # 原理见 FINAL_APPROACH_DIST 注释。多项式终点=目标且终端速度/加速度=0,
        # 由 60Hz control_update 持续推进, 进入 ARRIVE_THRESHOLD 后置 arrive。
        if dist_to_goal < FINAL_APPROACH_DIST:
            # 终点接管: 距目标较近时跳过网络推理, 直接对目标点规划一条终端速度/
            # 加速度为 0 的五次多项式, 平滑减速停到目标点(解决近目标 argmin 过冲
            # 造成的 40m 波动)。不做任何几何避障干预 —— 严格遵循 YOPO 的学习式避障。
            with self._lock:
                self._plan_final_approach()
                self.last_control_time = now
                cmd = self._compute_command()
            cmd["arrived"] = bool(self.arrive)
            cmd["dist_to_goal"] = dist_to_goal
            self.count += 1
            if self.count < 5 or self.count % 30 == 0:
                print(f"[YOPO 终点接管 #{self.count}] dist={dist_to_goal:.1f}m → 直接减速到目标点")
            return cmd

        # ── 3. Prepare network input ──
        depth_input = torch.from_numpy(depth).to(self.device, non_blocking=True)
        obs_norm, Rotation_wc = self._process_odom()
        obs_input = obs_norm.to(self.device, non_blocking=True)
        obs_input = self.state_transform.prepare_input(obs_input)
        self._last_depth_input = depth_input
        self._last_obs_input = obs_input
        time2 = time.time()

        # ── 4. Trajectory planning ──
        # 反应预算限速器已移除: 不再测量/平滑重规划间隔来动态限速。

        # Replan on every navigate call (each carries a new depth frame).
        # ctrl_time is reset to 0 inside _build_polynomial (matches original
        # callback_depth). The high-freq /yopo/control endpoint advances
        # ctrl_time each frame, so no lookahead hack is needed here.
        need_replan = True

        if need_replan:
            # Inference OUTSIDE lock (slow 1-2s, doesn't touch shared state).
            # This lets /yopo/control keep running during inference.
            endstate_w_ros = self._run_inference(Rotation_wc)
            # Polynomial construction INSIDE lock (fast ~1ms, writes shared state)
            with self._lock:
                self._build_polynomial(endstate_w_ros)
                self.last_control_time = now

        time3 = time.time()

        # ── 5. Compute command from polynomial ──
        with self._lock:
            cmd = self._compute_command()
        time4 = time.time()

        # ── 7. Arrival check ──
        dist_to_goal = np.linalg.norm(self.pos - self.goal)
        if dist_to_goal < ARRIVE_THRESHOLD and not self.arrive:
            self.arrive = True
            print(f"Arrived at goal! dist={dist_to_goal:.2f}m")

        # ── Timing ──
        self.time_prepare += (time2 - time1)
        self.time_forward += (time3 - time2)
        self.time_process += (time4 - time3)
        self.count += 1
        if self.verbose and self.count % 30 == 0:
            total = (time4 - time0) * 1000
            print(f"YOPO: prep={1000*(time2-time1):.1f}ms "
                  f"fwd={1000*(time3-time2):.1f}ms "
                  f"post={1000*(time4-time3):.1f}ms total={total:.1f}ms "
                  f"ctrl_t={self.ctrl_time:.3f}s replan={need_replan}")


        if self.count < 5 or self.count % 60 == 0:
            cmd_pos = cmd["position"]
            print(f"[YOPO #{self.count}] pos=({self.pos[0]:.1f},{self.pos[1]:.1f},{self.pos[2]:.1f}) "
                  f"cmd=({cmd_pos['x']:.1f},{cmd_pos['y']:.1f},{cmd_pos['z']:.1f}) "
                  f"ctrl_t={self.ctrl_time:.2f}s replan={need_replan} "
                  f"dist_goal={dist_to_goal:.1f}m")

        # 对齐原版 _run_inference/control_pub: 无障碍速度缩放/接近目标限速等额外干预
        cmd["arrived"] = bool(self.arrive)
        cmd["dist_to_goal"] = float(dist_to_goal)
        return cmd

    def control_update(self, pos, vel, quat):
        """High-frequency control update without depth/inference.

        Mirrors original control_pub() in test_yopo_ros.py: advances
        ctrl_time by real dt (capped at CTRL_DT = 0.02s, matching the
        original 50Hz fixed-step) and evaluates the last polynomial.

        Called at ~60Hz by the client render loop; navigate() replans
        at ~0.4Hz (depth arrival rate). This separation prevents blind
        flight between depth frames: the control command is always fresh.

        Args:
            pos/vel/quat: current drone odometry (MindCloud frame)
        Returns:
            PositionCommand dict (same shape as navigate())
        """
        now = time.time()

        # Update odometry (no lock: minor race with navigate acceptable,
        # matches original YOPO which doesn't lock odom either)
        self.pos = np.array(pos, dtype=np.float64)
        self.vel = np.array(vel, dtype=np.float64)
        self.quat = np.array(quat, dtype=np.float64)
        self.odom_time = time.time()

        # Arrival check
        dist_to_goal = float(np.linalg.norm(self.pos - self.goal))
        if dist_to_goal < ARRIVE_THRESHOLD:
            self.arrive = True
        if self.arrive:
            return {
                "position": {"x": float(self.goal[0]), "y": float(self.goal[1]), "z": float(self.goal[2])},
                "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
                "yaw": float(self.last_yaw),
                "yaw_dot": 0.0,
                "arrived": True,
                "dist_to_goal": dist_to_goal,
                "ctrl_time": 0.0,
            }

        with self._lock:
            # No trajectory yet (before first navigate completes)
            if (self.optimal_poly_x is None or self.ctrl_time is None):
                return {
                    "position": {"x": float(self.pos[0]), "y": float(self.pos[1]), "z": float(self.pos[2])},
                    "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "yaw": float(self.last_yaw),
                    "yaw_dot": 0.0,
                    "arrived": False,
                    "dist_to_goal": dist_to_goal,
                    "ctrl_time": 0.0,
                }

            # Advance ctrl_time by real dt (capped at CTRL_DT), then scale by
            # the time-scale rate (CTRL_TIME_SCALE) so the planned trajectory is
            # traversed at the configured speed (反应预算限速器已移除)。
            # 真实巡航速度 ≈ vel_max * rate; 指令 PVA 同样按 rate 缩放(见 _compute_command)。
            # 封顶用 poly_duration: 终点接管多项式时长可能大于 lattice traj_time,
            # 若按 traj_time 封顶会中途截断。
            dt = now - self.last_control_time if self.last_control_time else 0.0
            dt = min(max(dt, 0.0), CTRL_DT) * self._time_rate
            cap = self.poly_duration if self.poly_duration else self.traj_time
            # 网络轨迹超出 poly_duration 后继续外推 TRAJ_EXTEND_S(见 TRAJ_EXTEND_S 注释),
            # 使指令位置继续前进而非冻结; 终点接管(poly_extend=False)严格封顶在 T, 保证停在目标点。
            if self.poly_extend:
                cap += TRAJ_EXTEND_S
            self.ctrl_time = min(self.ctrl_time + dt, cap)
            self.last_control_time = now

            cmd = self._compute_command()

        # 对齐原版 control_pub: 无障碍速度缩放/接近目标限速等额外干预
        cmd["arrived"] = bool(self.arrive)
        cmd["dist_to_goal"] = float(dist_to_goal)
        return cmd

    def _process_output(self, endstate_pred, score_pred):
        """选择最优轨迹: 严格遵循 YOPO_360 test_yopo_ros.py, 直接 argmin(score)。

        不做任何几何避障干预 —— 避障完全由网络在训练期 safety_loss 中学到的
        score 提供(学习式避障), 与官方部署实现完全一致。
        """
        N = self.lattice_primitive.traj_num
        raw = endstate_pred.reshape(9, N).T          # [N, 9] 原始网络输出
        score = score_pred.reshape(N)

        # ── 前方障碍距离 (仅诊断, 不参与轨迹选择) ──
        # 复用 navigate 预处理阶段缓存的 CPU 侧归一化深度, 避免每次推理把整幅
        # 深度图从 GPU .cpu().numpy() 回拷(192×384×4B≈288KB/次) —— 省一次 D2H
        # 拷贝与一次大数组分配, 使重规划高频化(TensorRT 1.3ms 下该开销不可忽略)。
        fwd_dist = self.max_dis
        depth_map = getattr(self, '_last_depth_map_np', None)
        if depth_map is not None:
            self._last_depth_map = depth_map  # 诊断端点用
            H, W = depth_map.shape
            fwd_patch = depth_map[H//2-14:H//2+15, W//2-30:W//2+31]
            fwd_dist = float(fwd_patch.min()) * self.max_dis if fwd_patch.size else self.max_dis
        self.last_fwd_obstacle_dist = fwd_dist

        action_id = int(np.argmin(score))

        lattice_id = N - 1 - action_id
        endstate = self.state_transform.pred_to_endstate_cpu(
            raw[action_id:action_id+1, :], lattice_id
        )

        # ── Logging ──
        if self.verbose or self.count < 8 or self.count % 20 == 0:
            angles = self._angles_np
            chosen_alpha = float(angles[lattice_id, 0]) * 180.0 / np.pi
            chosen_beta = float(angles[lattice_id, 1]) * 180.0 / np.pi
            print(f"[YOPO轨迹] action_id=#{action_id} α={chosen_alpha:+5.0f}° "
                  f"β={chosen_beta:+5.0f}° score={float(score[action_id]):.3f} "
                  f"前方障碍={fwd_dist:5.1f}m")

        return endstate, float(score[action_id])

    def _compute_command(self):
        """Evaluate polynomial at ctrl_time and compute yaw.

        Mirrors original control_pub() in test_yopo_ros.py:
          - Evaluate position/velocity/acceleration from polynomial
          - Update desire_pos/vel/acc for next plan_from_reference cycle
          - Use calculate_yaw() to blend velocity direction + goal direction
          - Convert from ROS frame back to MindCloud
        """
        if (self.optimal_poly_x is None or self.optimal_poly_y is None
                or self.optimal_poly_z is None or self.ctrl_time is None):
            return {
                "position": {"x": float(self.pos[0]), "y": float(self.pos[1]), "z": float(self.pos[2])},
                "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
                "yaw": float(self.last_yaw),
                "yaw_dot": 0.0,
            }

        T = self.poly_duration if self.poly_duration else self.traj_time
        extend_s = TRAJ_EXTEND_S if self.poly_extend else 0.0
        t = min(self.ctrl_time, T + extend_s)

        # Evaluate polynomial in ROS frame
        if t > T and extend_s > 0.0:
            # 轨迹已走完: 按末端速度线性外推, 指令位置继续以 v(T) 前进、速度保持 v(T)。
            # 避免指令冻结在末端导致无人机冲过冻结点被位置环拉回(锯齿根因, 见 TRAJ_EXTEND_S)。
            # C0/C1 在 t=T 处连续(位置/速度都取多项式的终端值), 加速度置 0(匀速直线)。
            dt_ext = t - T
            pos_ros = np.array([
                self.optimal_poly_x.get_position(T) + self.optimal_poly_x.get_velocity(T) * dt_ext,
                self.optimal_poly_y.get_position(T) + self.optimal_poly_y.get_velocity(T) * dt_ext,
                self.optimal_poly_z.get_position(T) + self.optimal_poly_z.get_velocity(T) * dt_ext,
            ])
            vel_ros = np.array([
                self.optimal_poly_x.get_velocity(T),
                self.optimal_poly_y.get_velocity(T),
                self.optimal_poly_z.get_velocity(T),
            ])
            acc_ros = np.zeros(3)
        else:
            pos_ros = np.array([
                self.optimal_poly_x.get_position(t),
                self.optimal_poly_y.get_position(t),
                self.optimal_poly_z.get_position(t),
            ])
            vel_ros = np.array([
                self.optimal_poly_x.get_velocity(t),
                self.optimal_poly_y.get_velocity(t),
                self.optimal_poly_z.get_velocity(t),
            ])
            acc_ros = np.array([
                self.optimal_poly_x.get_acceleration(t),
                self.optimal_poly_y.get_acceleration(t),
                self.optimal_poly_z.get_acceleration(t),
            ])

        # ── 纯时间重参数化: 指令 PVA 三者必须自洽 ──
        # cmdPos = p(ctrl_time), 而 ctrl_time 以 rate 倍真实时间推进, 故指令位置的
        # 真实前进速率是 p'(t)*rate。若指令速度仍输出 p'(t)(不缩放), 则"位置暗含的
        # 速度"与"指令速度"相差 rate 倍 —— 二者打架: 无人机只能靠客户端位置环
        # P 项(clamp ±15)硬追, 并稳定滞后于规划位置(实测可达十几米), 等于
        # "不在规划的位置上飞行", 避障形同虚设。
        # 故输出速度乘 rate、加速度乘 rate², 使位置/速度/加速度描述同一实际运动。
        rate = float(getattr(self, "_time_rate", 1.0))
        pos_mc = self._vec_ros_to_mc(pos_ros)
        vel_mc = self._vec_ros_to_mc(vel_ros) * rate
        acc_mc = self._vec_ros_to_mc(acc_ros) * (rate * rate)

        # 绝对速度上限(反应预算限速器已移除后的硬兜底): 指令速度向量模长
        # 不得超过 YOPO_SPEED_CAP(默认 15 m/s), 保证"所有限速最高到 15 m/s"。
        # 仅截断超过上限的部分、方向保持不变; 正常巡航(rate=1, vel_max≤15)下不触发。
        speed_cap = float(YOPO_SPEED_CAP)
        _sp = float(np.linalg.norm(vel_mc))
        if _sp > speed_cap and _sp > 1e-6:
            vel_mc = vel_mc * (speed_cap / _sp)

        # Update desire state for plan_from_reference (critical for stable tracking)
        self.desire_pos = pos_mc
        self.desire_vel = vel_mc
        self.desire_acc = acc_mc
        self.desire_init = True

        # ── Yaw calculation ──
        # ERP/360° (lock_yaw=True): yaw is decoupled from obstacle avoidance;
        # hold the last yaw so the panorama orientation stays consistent with
        # the network's training distribution.  This mirrors test_yopo_ros.py
        # control_pub() lock_yaw branch.
        #
        # Otherwise (lock_yaw=False): compute yaw in ROS frame (x=forward,
        # y=left, z=up).  ROS yaw is identical to drone.js this.yaw convention:
        #   0° = forward at identity (drone faces south in MindCloud),
        #   positive = counter-clockwise (left turn) when viewed from above.
        # So the returned yaw can be used directly by drone.js without any
        # conversion.  Previously this was computed as a geographic bearing
        # (atan2(north, east)) which is offset by 90° AND mirrored, causing
        # the drone to spin and drift away from the goal.
        if self.lock_yaw:
            yaw = float(self.last_yaw)
            yaw_dot = 0.0
        else:
            goal_dir_ros = self._vec_mc_to_ros(self.goal - self.desire_pos)
            # ROS horizontal plane: [x=forward, y=left]
            vel_dir_h_ros = np.array([vel_ros[0], vel_ros[1]])
            goal_dir_h_ros = np.array([goal_dir_ros[0], goal_dir_ros[1]])

            yaw, yaw_dot = calculate_yaw(
                vel_dir_h_ros, goal_dir_h_ros,
                self.last_yaw, CTRL_DT, max_yaw_rate=0.5,
            )
            self.last_yaw = yaw

        px, py, pz = pos_mc
        vx, vy, vz = vel_mc
        ax, ay, az = acc_mc

        # 严格对齐原版: 不做任何碰撞减速/速度缩放。
        # 避障完全由网络 argmin(score) 选出的轨迹保证。

        self.last_position_cmd = {
            "position": {"x": float(px), "y": float(py), "z": float(pz)},
            "velocity": {"x": float(vx), "y": float(vy), "z": float(vz)},
            "acceleration": {"x": float(ax), "y": float(ay), "z": float(az)},
            "yaw": float(yaw),
            "yaw_dot": float(yaw_dot),
        }
        return self.last_position_cmd


# ── Flask routes ──────────────────────────────────────────────────

yopo_server = None  # global singleton


def _get_server():
    global yopo_server
    if yopo_server is None:
        raise RuntimeError("YOPO server not initialised")
    return yopo_server


@app.route("/yopo/status", methods=["GET"])
def status():
    srv = _get_server()
    resp = {
        "status": "ok",
        "goal": srv.goal.tolist(),
        "arrived": srv.arrive,
        "pos": srv.pos.tolist(),
        "device": srv.device,
        "traj_time": srv.traj_time,
        "inference_count": srv.count,
        "mode": "yopo",
    }
    if srv.lattice_primitive is not None:
        resp["traj_num"] = srv.lattice_primitive.traj_num
        resp["vel_max"] = srv.lattice_primitive.vel_max
        resp["acc_max"] = srv.lattice_primitive.acc_max
    # 时间重参数化倍率(反应预算限速器已移除, 仅剩 CTRL_TIME_SCALE 快进因子):
    resp["time_rate"] = round(float(getattr(srv, "_time_rate", 1.0)), 3)
    return jsonify(resp)


@app.route("/yopo/depth_diag", methods=["GET"])
def depth_diag():
    """深度方向诊断: 用与碰撞过滤器相同的采样公式, 对照无人机实际朝向验证方向映射."""
    srv = _get_server()
    if srv._last_depth_map is None:
        return jsonify({"error": "no depth frame yet", "inference_count": srv.count})
    dm = srv._last_depth_map  # (H,W) normalized [0,1], 0=近 1=远
    H, W = dm.shape
    max_d = srv.max_dis

    def sample(alpha_deg, beta_deg):
        a = np.radians(alpha_deg)
        b = np.radians(beta_deg)
        col = int(round(W / 2 + a * W / (2 * np.pi)))   # 正α=左转=图像右半
        row = int(round(H / 2 - b * H / np.pi))          # 正β=向上=图像顶部
        col = max(0, min(W - 1, col))
        row = max(0, min(H - 1, row))
        r0, r1 = max(0, row - 4), min(H, row + 5)
        c0, c1 = max(0, col - 8), min(W, col + 9)
        return round(float(dm[r0:r1, c0:c1].min()) * max_d, 2)

    # 无人机机头 world 朝向 (MC)
    from scipy.spatial.transform import Rotation as R
    qr = srv._quat_mc_to_ros(srv.quat)
    fwd_mc = srv._vec_ros_to_mc(R.from_quat(qr).as_matrix() @ np.array([1.0, 0.0, 0.0]))
    return jsonify({
        "pos_mc": [round(v, 2) for v in srv.pos.tolist()],
        "fwd_mc": [round(v, 3) for v in fwd_mc.tolist()],
        "fwd_dist_m": round(srv.last_fwd_obstacle_dist or max_d, 2),
        "top_band_m": round(float(dm[0:H // 6, :].mean()) * max_d, 2),
        "bottom_band_m": round(float(dm[5 * H // 6:, :].mean()) * max_d, 2),
        "dir_sample_m": {
            "前方α0": sample(0, 0),
            "左α+90": sample(90, 0),
            "右α-90": sample(-90, 0),
            "后方α180": sample(180, 0),
            "上β+75": sample(0, 75),
            "下β-75": sample(0, -75),
            "左前α+45": sample(45, 0),
            "右前α-45": sample(-45, 0),
        },
    })


@app.route("/yopo/set_goal", methods=["POST"])
def set_goal():
    data = request.get_json(silent=True) or {}
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    z = float(data.get("z", 2))
    srv = _get_server()
    srv.set_goal(x, y, z)
    return jsonify({"status": "ok", "goal": [x, y, z]})


@app.route("/yopo/navigate", methods=["POST"])
def navigate():
    """Main inference endpoint."""
    data = request.get_json(silent=True)
    if not data:
        return jsonify({"error": "empty body"}), 400

    srv = _get_server()

    # Decode depth
    depth_b64 = data.get("depth", "")
    if not depth_b64:
        return jsonify({"error": "missing depth"}), 400
    try:
        depth_bytes = base64.b64decode(depth_b64)
    except Exception as e:
        return jsonify({"error": f"depth base64 decode failed: {e}"}), 400

    depth_encoding = data.get("depth_encoding", "32FC1")
    shape = data.get("depth_shape", [DEPTH_HEIGHT, DEPTH_WIDTH])
    expected_size = shape[0] * shape[1] * (4 if depth_encoding == "32FC1" else 2)
    if len(depth_bytes) != expected_size:
        return jsonify({"error": f"depth size mismatch: got {len(depth_bytes)} expected {expected_size}"}), 400

    # Optional validity mask (uint8/mono8, 255=valid, 0=invalid). Same HxW as
    # depth. Only used when the server is running with in_channels >= 2.
    mask_bytes = None
    mask_b64 = data.get("mask", "")
    if mask_b64:
        try:
            mask_bytes = base64.b64decode(mask_b64)
        except Exception as e:
            return jsonify({"error": f"mask base64 decode failed: {e}"}), 400
        expected_mask_size = shape[0] * shape[1]
        if len(mask_bytes) != expected_mask_size:
            return jsonify({"error": f"mask size mismatch: got {len(mask_bytes)} expected {expected_mask_size}"}), 400

    pos = data.get("position", {})
    vel = data.get("velocity", {})
    orient = data.get("orientation", {})

    position = np.array([pos.get("x", 0), pos.get("y", 2), pos.get("z", 0)], dtype=np.float64)
    velocity = np.array([vel.get("x", 0), vel.get("y", 0), vel.get("z", 0)], dtype=np.float64)
    quat = np.array([
        orient.get("x", 0), orient.get("y", 0),
        orient.get("z", 0), orient.get("w", 1)
    ], dtype=np.float64)

    try:
        cmd = srv.navigate(depth_bytes, depth_encoding, position, velocity, quat,
                           mask_bytes=mask_bytes)
        return jsonify(cmd)
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/yopo/control", methods=["POST"])
def control():
    """High-frequency control endpoint (no depth/inference).

    Advances ctrl_time and evaluates the last polynomial. Called at ~60Hz
    by the client render loop; /yopo/navigate replans at ~0.4Hz.
    """
    data = request.get_json(silent=True)
    if not data:
        return jsonify({"error": "empty body"}), 400

    srv = _get_server()

    pos = data.get("position", {})
    vel = data.get("velocity", {})
    orient = data.get("orientation", {})

    position = np.array([pos.get("x", 0), pos.get("y", 2), pos.get("z", 0)], dtype=np.float64)
    velocity = np.array([vel.get("x", 0), vel.get("y", 0), vel.get("z", 0)], dtype=np.float64)
    quat = np.array([
        orient.get("x", 0), orient.get("y", 0),
        orient.get("z", 0), orient.get("w", 1)
    ], dtype=np.float64)

    try:
        cmd = srv.control_update(position, velocity, quat)
        return jsonify(cmd)
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# ── WebSocket transport ───────────────────────────────────────────
# A persistent WS connection removes per-call HTTP handshake/header/base64
# overhead. The browser client sends control/navigate messages and the server
# replies with the same PositionCommand JSON. Binary navigate frames carry raw
# depth bytes (no base64) for maximum efficiency.
def _ws_odom_from(obj):
    pos = obj.get("position", {})
    vel = obj.get("velocity", {})
    orient = obj.get("orientation", {})
    position = np.array([pos.get("x", 0), pos.get("y", 2), pos.get("z", 0)], dtype=np.float64)
    velocity = np.array([vel.get("x", 0), vel.get("y", 0), vel.get("z", 0)], dtype=np.float64)
    quat = np.array([
        orient.get("x", 0), orient.get("y", 0),
        orient.get("z", 0), orient.get("w", 1)
    ], dtype=np.float64)
    return position, velocity, quat


def _ws_handle_message(raw):
    """Process one WS message. Returns a JSON-serialisable dict."""
    srv = yopo_server
    if srv is None:
        return {"error": "server not initialised"}

    # Binary navigate frame: [uint32 BE header_len][utf8 json][depth bytes][mask bytes]
    if isinstance(raw, (bytes, bytearray)):
        try:
            import struct
            if len(raw) < 4:
                return {"error": "binary frame too short"}
            hdr_len = struct.unpack(">I", raw[:4])[0]
            hdr = json.loads(raw[4:4 + hdr_len].decode("utf-8"))
            off = 4 + hdr_len
            H, W = hdr["depth_shape"]
            enc = hdr.get("depth_encoding", "32FC1")
            dsize = H * W * (4 if enc == "32FC1" else 2)
            depth_bytes = bytes(raw[off:off + dsize])
            off += dsize
            mask_bytes = None
            if hdr.get("mask"):
                msize = H * W  # mono8 (uint8)
                if len(raw) >= off + msize:
                    mask_bytes = bytes(raw[off:off + msize])
            position, velocity, quat = _ws_odom_from(hdr)
            cmd = srv.navigate(depth_bytes, enc, position, velocity, quat,
                               mask_bytes=mask_bytes)
            cmd["id"] = hdr.get("id")
            return cmd
        except Exception as e:
            return {"error": f"ws navigate failed: {e}"}

    # Text JSON
    try:
        msg = json.loads(raw)
    except Exception as e:
        return {"error": f"json parse failed: {e}"}
    t = msg.get("type")
    if t == "control":
        position, velocity, quat = _ws_odom_from(msg)
        cmd = srv.control_update(position, velocity, quat)
        cmd["id"] = msg.get("id")
        return cmd
    elif t == "navigate":
        # JSON navigate (depth as base64) — fallback path
        depth_b64 = msg.get("depth", "")
        if not depth_b64:
            return {"error": "missing depth"}
        try:
            depth_bytes = base64.b64decode(depth_b64)
        except Exception as e:
            return {"error": f"depth base64 decode failed: {e}"}
        enc = msg.get("depth_encoding", "32FC1")
        shape = msg.get("depth_shape", [DEPTH_HEIGHT, DEPTH_WIDTH])
        expected = shape[0] * shape[1] * (4 if enc == "32FC1" else 2)
        if len(depth_bytes) != expected:
            return {"error": f"depth size mismatch: got {len(depth_bytes)} expected {expected}"}
        mask_bytes = None
        mask_b64 = msg.get("mask", "")
        if mask_b64:
            try:
                mask_bytes = base64.b64decode(mask_b64)
            except Exception:
                mask_bytes = None
        position, velocity, quat = _ws_odom_from(msg)
        try:
            cmd = srv.navigate(depth_bytes, enc, position, velocity, quat,
                               mask_bytes=mask_bytes)
            cmd["id"] = msg.get("id")
            return cmd
        except Exception as e:
            return {"error": str(e)}
    else:
        return {"error": f"unknown ws message type: {t}"}


def _is_ws_navigate_text(raw):
    """text JSON 帧是否为 navigate(base64 回退路径)。

    control 必须留在事件循环内立即执行, 绝不能被线程池排队 —— 否则 50Hz 控制环
    会被 navigate 的推理耗时卡住(self.pos 冻结 → 见 _current_state)。
    """
    try:
        return json.loads(raw).get("type") == "navigate"
    except Exception:
        return False


# navigate() 是阻塞长任务(深度清洗 + 网络推理)。若在 WS 事件循环里同步执行,
# 会在其整个耗时内饿死同一条连接上的 control 消息, 使 self.pos/self.vel 冻结在
# navigate() 开头的采集值 → 重规划的多项式起点陈旧、落在机体后方 → 指令回跳拉拽
# (详见 _current_state 注释)。放进线程池后可让 control 继续被即时处理。
# max_workers=1 是必须的: 两次 navigate 共享 self._last_depth_input /
# self._last_obs_input, 并发执行会让推理读到另一帧的深度或观测。
_ws_nav_executor = concurrent.futures.ThreadPoolExecutor(
    max_workers=1, thread_name_prefix="ws-navigate")


async def _ws_handler(websocket):
    remote = getattr(websocket, "remote_address", "unknown")
    print(f"[WS] client connected {remote}")
    loop = asyncio.get_running_loop()
    try:
        async for raw in websocket:
            try:
                # 二进制帧恒为 navigate(见 _ws_handle_message 的 bytes 分支);
                # text 帧按 type 判定。仅 navigate 走线程池, control 保持同步即时。
                if isinstance(raw, (bytes, bytearray)) or _is_ws_navigate_text(raw):
                    resp = await loop.run_in_executor(
                        _ws_nav_executor, _ws_handle_message, raw)
                else:
                    resp = _ws_handle_message(raw)
            except Exception as e:
                resp = {"error": str(e)}
            try:
                await websocket.send(json.dumps(resp))
            except Exception:
                break
    except Exception:
        pass
    finally:
        print(f"[WS] client disconnected {remote}")


def _run_ws_server(port):
    if not HAVE_WEBSOCKETS:
        print(f"[WS] 'websockets' not installed; skipping ws://0.0.0.0:{port} "
              f"(client will fall back to HTTP)")
        return
    import asyncio
    async def _serve():
        async with websockets.serve(_ws_handler, "0.0.0.0", port, max_size=None):
            await asyncio.get_event_loop().create_future()
    try:
        asyncio.run(_serve())
    except Exception as e:
        print(f"[WS] failed to start on {port}: {e}")


def main():
    parser = argparse.ArgumentParser(description="YOPO navigation server")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Server port")
    parser.add_argument("--model-path", type=str, default=DEFAULT_MODEL,
                        help="Path to YOPO model checkpoint")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Bind address")
    parser.add_argument("--verbose", action="store_true", help="Print timing logs")
    parser.add_argument("--camera-pitch", type=float, default=0.0,
                        help="Camera pitch angle in degrees (0=level, -30=30° down)")
    parser.add_argument("--lock-yaw", type=lambda v: str(v).lower() in ("1", "true", "yes"),
                        default=DEFAULT_LOCK_YAW,
                        help="Lock yaw to initial heading (ERP/360° default true). "
                             "Pass 'false' to let yaw follow the goal direction.")
    parser.add_argument("--ws-port", type=int, default=DEFAULT_WS_PORT,
                        help="WebSocket transport port (0 = disabled). More efficient "
                             "than per-call HTTP for the 60Hz control / depth stream.")
    args = parser.parse_args()

    global yopo_server
    yopo_server = YOPOServer(
        model_path=args.model_path,
        verbose=args.verbose,
        camera_pitch_deg=args.camera_pitch,
        lock_yaw=args.lock_yaw,
    )

    # Start the WebSocket transport in a background thread (shares yopo_server).
    if args.ws_port and args.ws_port > 0:
        tws = threading.Thread(target=_run_ws_server, args=(args.ws_port,), daemon=True)
        tws.start()
        print(f"YOPO websocket transport starting on ws://0.0.0.0:{args.ws_port}")

    print(f"YOPO server starting on {args.host}:{args.port}")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
