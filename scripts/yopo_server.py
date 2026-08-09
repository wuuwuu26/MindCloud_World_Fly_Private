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
    - ctrl_time advanced by real dt (capped at CTRL_DT=0.02s) in the
      high-freq /yopo/control endpoint; navigate resets it to 0 on replan.
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
import threading
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
DEFAULT_MODEL = os.path.join(
    YOPO_DIR, 'saved', 'YOPO_18', 'epoch20.pth'
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
ARRIVE_THRESHOLD = 2.0  # metres (matches test_yopo_ros.py L132: norm(pos-goal)<2.0)
# 终点接管距离: 网络在 goal_length (2*radio_range=10m) 内目标观测被按 goal_length
# 归一化缩小(state_transform.normalize_obs), lattice 又全是巡航型轨迹(端点速度可
# 达 vel_max≈6m/s), 接近目标时 argmin(score) 反复选出过冲/回头轨迹; 叠加
# plan_from_reference 下参考点越过目标会使目标方向观测翻转 → 目标附近速度/位置
# 来回波动、到不了目标。距目标 FINAL_APPROACH_DIST 内不再用网络推理, 直接规划
# 一条"终端速度/加速度=0、平滑减速停到目标点"的五次多项式。
FINAL_APPROACH_DIST = 12.0  # metres (与客户端 yopoFinalApproachDist 一致)
# YOPO 轨迹选择: 严格对齐 YOPO_360 test_yopo_ros.py (process_output L297-L310)
#   直接 argmin(score) 选最优轨迹, 不做任何额外干预。
#   目标代价与安全代价均已包含在网络输出的 score 中(训练时的 score_label),
#   任何外部启发式(目标引导/碰撞减速/紧急脱离)都会破坏这一代价场的平衡,
#   曾导致互相打架的抖动, 已全部移除。
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
        self.last_nav_time = None
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
        self.last_nav_time = None
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

        Mirrors original process_odom() in test_yopo_ros.py:
          - vel_w uses desire_vel when plan_from_reference=True
          - acc_w uses desire_acc
          - goal is relative to desire_pos when plan_from_reference=True
        """
        from scipy.spatial.transform import Rotation as R

        # Convert MindCloud body orientation to ROS body orientation
        quat_ros = self._quat_mc_to_ros(self.quat)
        Rotation_wb = R.from_quat(quat_ros).as_matrix()
        Rotation_wc = np.dot(Rotation_wb, self.Rotation_bc)
        Rotation_cw = Rotation_wc.T

        # Velocity: use desire_vel when plan_from_reference (original YOPO behavior)
        if PLAN_FROM_REFERENCE and self.desire_vel is not None:
            vel_w = self._vec_mc_to_ros(self.desire_vel)
        else:
            vel_w = self._vec_mc_to_ros(self.vel)

        # Acceleration: always from desired (polynomial output)
        if self.desire_acc is not None:
            acc_w = self._vec_mc_to_ros(self.desire_acc)
        else:
            acc_w = np.zeros(3)

        # Goal: relative to desire_pos when plan_from_reference
        if PLAN_FROM_REFERENCE and self.desire_pos is not None:
            goal_w = self._vec_mc_to_ros(self.goal - self.desire_pos)
        else:
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

    def _build_polynomial(self, endstate_w_ros):
        """Build polynomial trajectory from inference output. FAST (~1ms).

        严格对齐 YOPO_360 test_yopo_ros.py _run_inference (L238-L250):
          - start_pos/vel 由 plan_from_reference 决定 (True=上次指令 desire)
          - 3D 导航: 不做 z 轴水平面投影, 完全信任网络预测的 z 终端状态
            (垂直锚点 β 与 z 轴 PVA 由网络根据深度场景选择, 支持上/下避障)
          - 三轴 Poly5Solver, 无任何缩放/平滑/速度干预
        """
        # Start from desire_pos/vel when plan_from_reference, else actual odom
        if PLAN_FROM_REFERENCE and self.desire_pos is not None:
            start_pos_ros = self._vec_mc_to_ros(self.desire_pos)
            start_vel_ros = self._vec_mc_to_ros(self.desire_vel)
        else:
            start_pos_ros = self._vec_mc_to_ros(self.pos)
            start_vel_ros = self._vec_mc_to_ros(self.vel)

        start_acc_ros = self._vec_mc_to_ros(self.desire_acc) if self.desire_acc is not None else np.zeros(3)

        # 强制 z 轴终点到固定巡航高度 (对齐原版 test_yopo_ros.py L240:
        #   endstate_w[:, 2, 0] = fixed_height - start_pos[2])
        # 原版为水平导航: 网络只负责水平(x,y)方向规划, 高度方向强制保持
        # fixed_height(即目标高度), 不被网络不可靠的垂直输出(β锚点)带飞。
        # MindCloud 之前"完全信任网络 z 终端状态"会导致无人机被网络垂直输出
        # 猛拉(如飞到 160m 高空、β=±75° 乱跳), 无法保持目标高度水平推进。
        fixed_height = float(self.goal[1])  # MC y=up = 目标高度(ROS z 一致)
        endstate_w_ros[0, 2, 0] = fixed_height - start_pos_ros[2]

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
        self.ctrl_time = 0.0
        self.poly_duration = self.traj_time

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
        if self.desire_pos is None:
            start_pos_mc = self.pos.copy()
            start_vel_mc = self.vel.copy()
            start_acc_mc = np.zeros(3)
        else:
            start_pos_mc = self.desire_pos.copy()
            start_vel_mc = self.desire_vel.copy() if self.desire_vel is not None else np.zeros(3)
            start_acc_mc = self.desire_acc.copy() if self.desire_acc is not None else np.zeros(3)

        # 时长: 平均速度 ≤ vel_max 且留出减速时间, 夹在 [0.8, 3.0]s
        dist = float(np.linalg.norm(self.goal - start_pos_mc))
        vel_max = float(self.lattice_primitive.vel_max)
        T = float(np.clip(1.2 * dist / max(vel_max, 0.1), 0.8, 3.0))

        start_pos_ros = self._vec_mc_to_ros(start_pos_mc)
        start_vel_ros = self._vec_mc_to_ros(start_vel_mc)
        start_acc_ros = self._vec_mc_to_ros(start_acc_mc)
        end_pos_ros = self._vec_mc_to_ros(self.goal)

        self.optimal_poly_x = Poly5Solver(
            start_pos_ros[0], start_vel_ros[0], start_acc_ros[0],
            end_pos_ros[0], 0.0, 0.0, T)
        self.optimal_poly_y = Poly5Solver(
            start_pos_ros[1], start_vel_ros[1], start_acc_ros[1],
            end_pos_ros[1], 0.0, 0.0, T)
        self.optimal_poly_z = Poly5Solver(
            start_pos_ros[2], start_vel_ros[2], start_acc_ros[2],
            end_pos_ros[2], 0.0, 0.0, T)
        self.ctrl_time = 0.0
        self.poly_duration = T

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
        # Compute real elapsed time since last call (depth capture is slow)
        dt_real = 0.0
        if self.last_nav_time is not None:
            dt_real = min(now - self.last_nav_time, self.traj_time * 2)
        self.last_nav_time = now

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

            # Advance ctrl_time by real dt, capped at CTRL_DT (matches
            # original 50Hz fixed-step; prevents dangerous jumps on stalls).
            # 封顶用 poly_duration: 终点接管多项式的时长可能大于 lattice
            # traj_time, 若仍按 traj_time 封顶会在中途截断, 到不了目标点。
            dt = now - self.last_control_time if self.last_control_time else 0.0
            dt = min(max(dt, 0.0), CTRL_DT)
            cap = self.poly_duration if self.poly_duration else self.traj_time
            self.ctrl_time = min(self.ctrl_time + dt, cap)
            self.last_control_time = now

            cmd = self._compute_command()

        # 对齐原版 control_pub: 无障碍速度缩放/接近目标限速等额外干预
        cmd["arrived"] = bool(self.arrive)
        cmd["dist_to_goal"] = float(dist_to_goal)
        return cmd

    def _process_output(self, endstate_pred, score_pred):
        """Select best trajectory: 严格对齐 YOPO_360 test_yopo_ros.py process_output (L297-L310).

        直接 argmin(score) 选最优轨迹, 不做任何额外干预
        (无碰撞过滤器/3D方向连续性/高度引导——这些都是本项目后来加的,
         会干扰网络原生的避障决策, 现按用户要求移除)。
        仅保留前方障碍距离计算用于诊断日志, 不参与选择。
        """
        endstate_pred = endstate_pred.reshape(9, self.lattice_primitive.traj_num).T
        score = score_pred.reshape(self.lattice_primitive.traj_num)
        N = self.lattice_primitive.traj_num

        # ── 前方障碍距离 (仅诊断, 不参与选择) ──
        fwd_dist = self.max_dis
        if self._last_depth_input is not None:
            depth_map = self._last_depth_input[0, 0].cpu().numpy()  # (H,W) norm[0,1]
            self._last_depth_map = depth_map  # 诊断端点用
            H, W = depth_map.shape
            # 前向检测 patch 加宽: 原 ±12列/±6行 只覆盖很窄的正前方, 斜前方/
            # 侧向近障不触发减速 → 全速撞。扩展到 ±30列/±14行(前向扇形),
            # 让更多方向的近障都能进入减速判定。
            fwd_patch = depth_map[H//2-14:H//2+15, W//2-30:W//2+31]
            fwd_dist = float(fwd_patch.min()) * self.max_dis if fwd_patch.size else self.max_dis
        self.last_fwd_obstacle_dist = fwd_dist

        # 原版 L302: action_id = argmin(score); lattice_id = traj_num-1-action_id
        # 目标代价已由网络 score 内含(训练时 wg=0.15), 不再叠加任何外部引导项。
        action_id = int(np.argmin(score))
        lattice_id = N - 1 - action_id
        endstate = self.state_transform.pred_to_endstate_cpu(
            endstate_pred[action_id:action_id+1, :], lattice_id
        )

        # ── Logging ──
        if self.count < 8 or self.count % 20 == 0:
            angles = self._angles_np
            chosen_alpha = float(angles[lattice_id, 0]) * 180.0 / np.pi
            chosen_beta = float(angles[lattice_id, 1]) * 180.0 / np.pi
            print(f"[YOPO避障] argmin(score)=#{action_id} α={chosen_alpha:+5.0f}° "
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

        t = min(self.ctrl_time, self.poly_duration if self.poly_duration else self.traj_time)

        # Evaluate polynomial in ROS frame
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

        # Convert to MindCloud frame
        pos_mc = self._vec_ros_to_mc(pos_ros)
        vel_mc = self._vec_ros_to_mc(vel_ros)
        acc_mc = self._vec_ros_to_mc(acc_ros)

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
    args = parser.parse_args()

    global yopo_server
    yopo_server = YOPOServer(
        model_path=args.model_path,
        verbose=args.verbose,
        camera_pitch_deg=args.camera_pitch,
        lock_yaw=args.lock_yaw,
    )

    print(f"YOPO server starting on {args.host}:{args.port}")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
