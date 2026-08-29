#!/usr/bin/env python3
"""Flask API for YOPO autonomous drone navigation inference.

Provides a stateful HTTP endpoint that accepts a depth image + odometry
+ goal position and returns a PositionCommand (position, velocity,
acceleration, yaw).  The YOPO neural network runs inside this process;
no ROS dependency.

Key design decisions (aligned with original test_yopo_ros.py):
    - plan_from_reference=True: the new trajectory starts from the last command
      (desire_pos/vel/acc), coinciding with the previous trajectory at the join
      point -> continuous, no back-and-forth motion. This preserves the original
      cascaded-control semantics (trajectory planning + SO3 position controller):
      on each new depth frame it replans; the new polynomial starts from the
      current desire state, and the controller evaluates the polynomial per frame
      and updates desire.
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
    """TensorRT inference wrapper: loads an engine exported from ONNX and exposes
    a __call__ with the same signature as YopoNetwork, (depth, obs) -> (endstate, score),
    so call sites (self.policy(...)) need no changes. Executes the engine directly
    via the tensorrt Python API (no torch2trt / nvcc needed)."""

    def __init__(self, engine_path, device):
        import tensorrt as trt
        self.device = device
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            engine_data = f.read()
        runtime = trt.Runtime(self.logger)
        self.engine = runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()
        self.stream = torch.cuda.Stream()  # dedicated CUDA stream to avoid default-stream perf warnings
        self.input_names = []
        self.output_names = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)

    def __call__(self, depth, obs):
        # torch.from_numpy / state_transform outputs are usually already contiguous;
        # only copy when truly non-contiguous to avoid needless full-tensor copies
        # under high-frequency calls.
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
        # Input tensors are written by the caller on the default stream; sync the
        # default stream first to ensure data is ready, run on the dedicated stream
        # (avoids TensorRT's extra cudaStreamSynchronize on the default stream), then
        # sync this stream.
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
# Time scaling (optional fine-tune): advance ctrl_time faster than 1x so the drone
# covers the same (network-planned, collision-avoiding) spatial trajectory quicker ->
# cruise speed ~= vel_max * CTRL_TIME_SCALE. Default 1.0. The real planned speed is
# decided by cfg["velocity"] (see YOPO_VELOCITY); this is only an additive fine-tune.
# Guard against an empty env string (shell exports "") which would crash float("").
_env_tscale = os.environ.get("YOPO_CTRL_TIME_SCALE")
CTRL_TIME_SCALE = float(_env_tscale) if _env_tscale else 1.0
# Hard command-speed cap (m/s): the reactive-budget speed governor was removed; this
# absolute clamp guarantees the command speed never exceeds this value under any
# config (including raised YOPO_VELOCITY / YOPO_CTRL_TIME_SCALE). Default 15.
_env_scap = os.environ.get("YOPO_SPEED_CAP")
YOPO_SPEED_CAP = float(_env_scap) if _env_scap else 15.0
# Trajectory-end extension (seconds): the key fix for the "speed-up -> slow-down ->
# speed-up again" sawtooth. The network trajectory duration is far shorter than the
# depth-loop replan interval -- measured traj_time ~= 0.67s
#   (radio_range=5 -> sgm_time=2*5/6=1.667s; YOPO_VELOCITY=15 -> ratio=2.5 ->
#    segment_time=1.667/2.5=0.667s), and at CTRL_TIME_SCALE=1 it takes just ~0.67s of
#   real time to finish the whole trajectory (even shorter at SCALE=2, only 0.33s).
#   After that ctrl_time is capped by poly_duration -> the command freezes at the end.
# The client controller is velTarget = clamp(1.0*posErr, +/-15) + ffVel (drone.js);
#   once frozen, cmdPos stops advancing while cmdVel is still the end velocity -> the
#   drone overshoots the freeze point -> posErr goes negative -> the position loop pulls
#   it back and slows down -> next replan speeds up again = the root cause of the sawtooth.
# So after the trajectory ends, linearly extrapolate at the [end velocity]: the command
#   position keeps advancing at the end velocity and the velocity stays at the end
#   velocity, keeping the command always ahead of and co-moving with the drone, posErr
#   stabilizes ~= 0, eliminating the pull-back and sawtooth. Meanwhile the command
#   velocity comes from the network-planned end velocity rather than the "current true
#   velocity", breaking the circular dependency described in main.js's comment:
#   "poly start velocity = true velocity -> command velocity ~= true velocity -> ~0".
# If no replan occurs beyond TRAJ_EXTEND_S (depth channel anomaly), fall back to
# freezing to avoid unbounded blind flight.
_env_extend = os.environ.get("YOPO_TRAJ_EXTEND_S")
TRAJ_EXTEND_S = float(_env_extend) if _env_extend else 2.0

# Reactive-budget speed governor removed: no longer dynamically limits speed by replan
# interval. The drone flies at the network-planned speed; the ctrl_time advance rate is
# taken directly as CTRL_TIME_SCALE (default 1.0; restart_all.sh currently sets 1, i.e.
# fully follows the network-planned speed vel_max~=15 -> cruise <=15 m/s. Setting >1 can
# "fast-forward" beyond vel_max*SCALE, but gets hard-clamped back by YOPO_SPEED_CAP=15
# and the planned position leads while the drone keeps lagging, so keep it at 1).
# Avoidance is now jointly guaranteed by (1) the trajectory selected via argmin(score) and
# (2) the client-side geometric reactive potential field; command PVA is still scaled by
# the same rate to stay self-consistent (see _compute_command).
def _env_float(name, default):
    """Read an env var as float; fall back to default on empty/invalid value (avoids float("") crash)."""
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


# (Reactive-budget governor constants REACT_*/GOVERNOR_MIN_V/ALLOW_TIMESCALE_BOOST removed:
#  the drone now flies directly by CTRL_TIME_SCALE; avoidance is delegated to the network
#  plus the client-side geometric reactive potential field.)

ARRIVE_THRESHOLD = 2.0  # metres (matches test_yopo_ros.py L132: norm(pos-goal)<2.0)
# Final-approach distance: within goal_length (2*radio_range=10m) the goal observation is
# normalized/squeezed by goal_length (state_transform.normalize_obs), and the lattice is all
# cruise-type trajectories (endpoint speed up to vel_max~=6m/s). Near the goal, argmin(score)
# repeatedly picks overshoot/turn-back trajectories; combined with plan_from_reference where
# the reference point passes the goal, the goal-direction observation flips -> velocity/position
# oscillate near the goal and it never arrives. Within FINAL_APPROACH_DIST of the goal, stop
# using network inference and directly plan a quintic polynomial that "decelerates smoothly to
# the goal with zero terminal velocity/acceleration".
FINAL_APPROACH_DIST = 12.0  # metres (matches the client-side yopoFinalApproachDist)

# ── YOPO trajectory selection ──
# Strictly follows YOPO_360 test_yopo_ros.py: pick the best trajectory directly via
# argmin(score), without any extra geometric avoidance intervention. Avoidance is provided
# entirely by the score the network learned during training (safety_loss) -- "learning-based
# avoidance" -- rather than a deployment-side geometric collision cost, exactly matching the
# official deployment implementation.
# plan_from_reference=True: the new trajectory starts from the last command (desire_pos/vel/acc)
# and coincides with the old trajectory at the join point -> continuous. The original test
# settings L440 already set this to True.
PLAN_FROM_REFERENCE = True
# Enable calculate_yaw(): the body yaws smoothly toward the goal so the goal falls
# inside the lattice sector coverage.
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
        # Timestamp of the most recent odometry sample (see _current_state): used to extrapolate
        # pos to "now" so the replanned polynomial starts from the drone's true current state,
        # not the state "when the request was sent".
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
        self.poly_duration = None  # current polynomial duration (s): network traj=traj_time, final approach=planned T
        # Whether to extrapolate at end velocity after the trajectory ends (see TRAJ_EXTEND_S):
        # network trajectory=yes; final approach=no (final approach must stop at the goal, extrapolation would overshoot).
        self.poly_extend = False
        # Effective ctrl_time advance rate (pure time-reparameterization rate): the reactive-budget
        # speed governor was removed, so take CTRL_TIME_SCALE directly (default 1.0; restart_all.sh
        # currently sets 1 -> fully follows the network-planned speed; >1 would be hard-clamped by
        # YOPO_SPEED_CAP=15 and lead the plan, so keep it at 1).
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

        # Omnidirectional ERP panorama has no camera pitch offset: body frame coincides with camera frame.
        # (The original used camera_pitch only for a forward pinhole camera; adding pitch under a 360
        #  panorama would mismatch the anchor direction with the depth-map row positions.)
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

        # ── Optional TensorRT acceleration (README: TensorRT Deployment) ──
        # Enabled only when YOPO_USE_TRT=1 and the converted engine yopo_trt.pth exists; otherwise
        # falls back to PyTorch eager (native path). Conversion script: scripts/yopo_trt_transfer.py,
        # which exports this eager model via ONNX and solidifies it into a TensorRT engine (fp16).
        # On Orin NX inference is only 1~5ms, vs 100~350ms eager -- a big speedup that raises replan
        # frequency, shortens blind-flight segments, and improves avoidance. tensorrt loads the engine
        # directly and wraps it into an inference wrapper (_TrtYopoModel) with the same signature as
        # YopoNetwork, (depth, obs)->(endstate, score), so call sites (self.policy(...)) need no changes.
        self.use_trt = False
        trt_path = os.environ.get('YOPO_TRT_PATH') or os.path.join(
            os.path.dirname(model_path), 'yopo_trt.pth')
        if os.environ.get('YOPO_USE_TRT', '0').lower() in ('1', 'true', 'yes'):
            if not os.path.isfile(trt_path):
                print(f"[TensorRT] engine not found at {trt_path}; run scripts/yopo_trt_transfer.py to generate it first, falling back to eager")
            else:
                try:
                    self.policy = _TrtYopoModel(trt_path, self.device)
                    self.use_trt = True
                    print(f"[TensorRT] loaded {trt_path} -- inference acceleration enabled")
                except Exception as e:
                    print(f"[TensorRT] load failed ({e}); falling back to PyTorch eager")
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
        # Also reset end-point smoothing to avoid direction inertia from the previous goal.
        self._last_end_xy = None

    # ═══════════════════════════════════════════════════════════════
    #  Simplified avoidance algorithm (Simple Reactive Obstacle Avoidance)
    #
    #  Principle:
    #    1. ERP panorama depth map (384x192): column W/2 = body front, W/4 = body left, 3W/4 = body right
    #    2. Scan directions -90° -> +90° over the horizontal band (rows H/2±15); for each direction take
    #       the patch-min depth
    #    3. Front clear (>SAFE_DIST): fly toward the goal direction
    #    4. Front blocked: pick best direction = argmax(clear × gauss(angle-goal_angle, σ=60°))
    #    5. All directions blocked (<EMERGENCY_DIST): emergency climb
    #    6. P velocity controller: target_pos = pos + cmd_dir*lookahead, target_vel = cmd_dir*cruise_speed
    #
    #  Coordinate frames:
    #    MC: x=east, y=up, z=north. body forward at identity = -z (south)
    #    ROS: x=forward, y=left, z=up
    #    body forward in MC world = R_ROS_TO_MC @ R_wc_ros @ [1,0,0]
    #    body right  in MC world = R_ROS_TO_MC @ R_wc_ros @ [0,-1,0]
    #    Panorama column k -> azimuth delta = (k - W/2) * 2π/W (positive = right/CW)
    # ═══════════════════════════════════════════════════════════════

    # ── Simple avoidance tunables ──
    # Depth updates ~2.6Hz -> 380ms/frame. At 3m/s the drone moves 1.1m per frame.
    # SAFE_DIST must be >> 1.1m + braking distance (~2m) + margin -> use 15m.
    _SA_SAFE_DIST = 15.0       # m: front-clear threshold
    _SA_EMERGENCY_DIST = 5.0   # m: emergency-climb threshold
    _SA_CRUISE_SPEED = 3.0     # m/s: cruise speed (lowered to increase reaction time)
    _SA_LOOKAHEAD = 4.0        # m: target_pos = pos + cmd_dir * lookahead
    _SA_BAND_HALF_H = 25       # rows: horizontal band half-height (±25 rows ≈ ±23° around horizon)
    _SA_PATCH_HALF_W = 14      # cols: per-direction patch half-width (~13° wide)
    _SA_SCAN_ANGLES = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]
    _SA_GOAL_SIGMA_DEG = 50.0  # gaussian weight σ: smaller = more biased toward goal direction
    _SA_YAW_SMOOTH = 0.25      # yaw low-pass filter (lower = less jitter)
    _SA_DIR_SMOOTH = 0.4       # cmd_dir low-pass filter (lower = less jitter)
    _SA_CLIMB_KP = 0.5         # altitude P gain
    _SA_CLIMB_MAX = 3.0        # m/s: max climb/descend rate
    _SA_EMERGENCY_CLIMB = 3.0  # m/s: emergency climb rate
    _SA_MIN_SPEED = 0.5        # m/s: min speed near obstacles (keep maneuverability)
    _SA_DEPTH_AGE_WARN = 0.2   # s: depth-stale warning threshold, start slowing down
    _SA_DEPTH_AGE_STOP = 1.0   # s: depth-stale stop threshold, full stop
    _SA_MAX_ALT_ABOVE_GOAL = 100.0  # m: max altitude above goal; beyond this force descent
    _SA_MAX_EMERGENCY_FRAMES = 8    # max consecutive emergency frames before blind flight

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

        The conditional state must match the polynomial start (both use true odometry); otherwise the
        network's predicted end displacement (a relative quantity) would be measured from "the start it
        thinks it has" (desire_*) while the polynomial actually starts from the true start -> join
        mismatch. Hence vel/goal use self.vel/self.pos (true) and acc uses self.desire_acc (matching the
        polynomial start acceleration).
        """
        from scipy.spatial.transform import Rotation as R

        # Convert MindCloud body orientation to ROS body orientation
        quat_ros = self._quat_mc_to_ros(self.quat)
        Rotation_wb = R.from_quat(quat_ros).as_matrix()
        Rotation_wc = np.dot(Rotation_wb, self.Rotation_bc)
        Rotation_cw = Rotation_wc.T

        # Velocity: true current velocity (matches the polynomial start point)
        vel_w = self._vec_mc_to_ros(self.vel)

        # Acceleration: most recent commanded acceleration (= polynomial start acceleration estimate)
        if self.desire_acc is not None:
            acc_w = self._vec_mc_to_ros(self.desire_acc)
        else:
            acc_w = np.zeros(3)

        # Goal: relative to the true current position
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
        """Return (pos, vel): the drone's position and velocity at "now" (1st-order extrapolation).

        Why self.pos cannot be used directly: navigate() is a long blocking task
        (depth cleanup + network inference), and the WS transport calls it
        synchronously inside the single-threaded asyncio event loop (_ws_handler ->
        _ws_handle_message -> srv.navigate). During that time every control message on
        the same WS connection queues up and starves, so self.pos stays frozen at the
        value captured at the start of navigate() (the moment the client sent the
        request). Using that as the polynomial start point makes the trajectory start
        from "the position when the request was sent", while the drone has already
        advanced by v*dt -- the start point lands behind the body, the commanded
        position jumps backwards, the position loop drags the drone back -> it slows
        down and re-accelerates on the next replan (velocity sawtooth).

        So we extrapolate forward from the most recent odometry timestamp using the
        velocity, which gives the true current position and keeps the replanned
        polynomial start at C0 = current position, C1 = current velocity.

        Note: the network observation obs still uses the state "at depth capture time"
        (_process_odom runs before inference) -- that is correct, since the depth image
        corresponds to the pose of that moment; only the trajectory start point needs
        to be extrapolated to "now".

        max_dt caps the extrapolation so it cannot run away when odometry stops
        updating (paused / stream interrupted).
        """
        pos = self.pos.copy()
        vel = self.vel.copy()
        if self.odom_time is not None:
            dt = min(max(time.time() - self.odom_time, 0.0), max_dt)
            pos = pos + vel * dt
        return pos, vel

    # The reactive-budget speed governor (_safe_speed) was removed: v_safe is no longer
    # derived from the replan interval to cap speed. The ctrl_time advance rate is taken
    # directly from CTRL_TIME_SCALE inside _build_polynomial.

    def _build_polynomial(self, endstate_w_ros):
        """Build polynomial trajectory from inference output. FAST (~1ms).

        Strictly aligned with YOPO_360 test_yopo_ros.py _run_inference (L238-L250):
          - start_pos/vel are decided by plan_from_reference (True = last commanded desire)
          - 3D navigation: no z-axis projection onto the horizontal plane; the
            network-predicted z terminal state is trusted completely (the vertical
            anchor beta and the z-axis PVA are chosen by the network from the depth
            scene, enabling avoidance up/down)
          - three-axis Poly5Solver, with no scaling / smoothing / speed intervention
        """
        # Continuity (critical): the new trajectory must grow smoothly out of the
        # previous trajectory's "true current state".
        # When inference is slow (1-2 s), ctrl_time saturates between two plans and the
        # drone coasts straight along the old trajectory's tail, so the reference state
        # desire_* freezes at that tail and lags behind the real position.
        # Starting from desire_* anyway would yank the new command back to the lagging
        # point -> jumps / discontinuity at the join.
        # So the start point comes from the drone's true odometry state extrapolated to
        # "now", keeping position C0 and velocity C1 continuous across plans; the
        # acceleration uses the latest commanded acceleration (self.desire_acc, which is
        # close to the real acceleration while flying/coasting) as the C2 estimate.
        # Note: self.pos cannot be used directly -- it freezes to a stale value while
        # navigate() blocks (see _current_state), it must be extrapolated from the
        # odometry timestamp to "now".
        start_pos_mc, start_vel_mc = self._current_state()
        start_pos_ros = self._vec_mc_to_ros(start_pos_mc)
        start_vel_ros = self._vec_mc_to_ros(start_vel_mc)
        start_acc_ros = self._vec_mc_to_ros(self.desire_acc) if self.desire_acc is not None else np.zeros(3)

        # 3D navigation: trust the network-predicted z (vertical) terminal state
        # completely -- do not force the end height to equal the goal height, and do not
        # force vertical velocity/acceleration to zero. The network input obs already
        # carries "goal relative position (including the vertical component)", so the
        # network weighs horizontal progress against vertical manoeuvres on its own
        # (avoiding up/down, vertical anchor beta) -- that is YOPO's 3D capability.
        # An early version forced fixed_height = goal[1] and zeroed the vertical PVA,
        # which effectively "downgraded 3D navigation to 2D": with a large height gap
        # between the goal and the current position the whole trajectory got dragged
        # vertically and the network's predicted vertical displacement/velocity was
        # discarded -> the drone "rushed toward the goal altitude at once / did not move".
        # Removing that restored the network's native 3D trajectory (identical to the
        # docstring and the official test_yopo_ros.py).

        # ctrl_time advance rate = CTRL_TIME_SCALE. The reactive-budget speed governor was
        # removed: rate is no longer pushed below 1 via v_safe/vel_max to cap speed
        # dynamically, the drone flies at the network-planned speed (default 1.0; enable
        # "fast forward" with YOPO_CTRL_TIME_SCALE).
        # The commanded PVA is still scaled by the same rate (see _compute_command) so
        # position/velocity/acceleration stay mutually consistent.
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
        # ctrl_time starts at 0 (progress is not carried over) so the commanded
        # position/velocity/acceleration C0/C1/C2 stay continuous.
        # Reason (measured): if ctrl_time kept the previous trajectory's progress, then
        # after navigate() rebuilt the trajectory _compute_command would sample straight
        # from the new trajectory's middle -> the commanded position would jump sharply
        # at the replan instant (measured peak jump of 10.4 m between replans) and the
        # drone's motion would be discontinuous. Starting from 0 instead puts the new
        # trajectory's start at the current true position/velocity/acceleration (the
        # start_* values above), so commands grow continuously out of the current
        # position and the jump between replans drops to ~2 m. Cruise speed comes from
        # CTRL_TIME_SCALE and YOPO_VELOCITY, and the 50 Hz control loop advances from 0
        # into the middle quickly, so continuity is not affected.
        # The final-approach takeover in _plan_final_approach sets ctrl_time = 0 on its
        # own, which is consistent with this.
        self.ctrl_time = 0.0
        self.poly_duration = self.traj_time
        self.poly_extend = True

    def _make_final_approach_polys(self):
        """Build the final-approach quintic polynomial to the goal.

        Returns (px, py, pz, T, start_pos_ros).
        """
        # Consistent with _build_polynomial: the start point is the true odometry state,
        # which keeps C0/C1/C2 continuous where this joins the previous (network-planned)
        # trajectory, with no pull-back jumps.
        # Consistent with _build_polynomial: the start point is the state "at now"
        # (see _current_state).
        start_pos_mc, start_vel_mc = self._current_state()
        start_acc_mc = self.desire_acc.copy() if self.desire_acc is not None else np.zeros(3)

        # Duration: average speed <= vel_max with room to decelerate, clamped to [0.8, 3.0] s
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
        """Final-approach takeover: when the goal is close, skip network inference and
        directly plan a quintic polynomial from the current reference state
        (desire_pos/vel/acc) to the goal with zero terminal velocity/acceleration.

        Purpose: remove the network's near-goal instability inside
        goal_length = 2*radio_range = 10 m (the goal observation gets squeezed by
        normalisation and the lattice only has cruise-type trajectories, no stopping
        ones -> overshoot / turn-back oscillation). The final polynomial guarantees a
        smooth deceleration and an accurate stop at the goal; once the drone is within
        ARRIVE_THRESHOLD (2 m) the arrive check takes over.

        Consistent with _build_polynomial: the polynomial is built in the ROS frame and
        converted back to the MindCloud frame by _compute_command; the duration T is
        derived from the remaining distance and vel_max, and written to poly_duration so
        ctrl_time can be capped with it (otherwise the 60 Hz control loop would cut off
        at traj_time and the final polynomial would never reach the goal).
        """
        px, py, pz, T, _ = self._make_final_approach_polys()
        self.optimal_poly_x, self.optimal_poly_y, self.optimal_poly_z = px, py, pz
        self.ctrl_time = 0.0
        self.poly_duration = T
        # Extrapolation is forbidden during final approach: the drone must stop exactly
        # at the goal (extrapolation would overshoot it).
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

        # Depth-credibility detection was removed: the upstream heuristic that hovers on
        # "abnormal depth" mistakes "many near pixels" for a depth failure in real urban
        # building clusters and hovers constantly, blocking normal navigation (the user
        # asked for strict alignment with upstream, which has no such logic). Depth
        # validity is left to the mask channel and the network itself.

        # ── Temporary diagnostics: print the incoming depth distribution at low rate ──
        # (to confirm the DA360 scale)
        self._diag_frames = getattr(self, '_diag_frames', 0) + 1
        if self._diag_frames % 30 == 1:
            fv = depth_raw[np.isfinite(depth_raw) & (depth_raw > 0.01)]
            if fv.size:
                print(f"[depth diag] min={float(fv.min()):.2f} med={float(np.median(fv)):.2f} "
                      f"max={float(fv.max()):.2f} <2m={(fv < 2.0).mean():.2f} <5m={(fv < 5.0).mean():.2f}")

        depth = np.minimum(depth_raw, self.max_dis) / self.max_dis
        nan_mask = np.isnan(depth) | (depth < self.min_dis / self.max_dis)

        if mask_raw is not None:
            valid = (mask_raw > 127).astype(np.uint8)
        else:
            valid = (~nan_mask).astype(np.uint8)

        # ── Invalid-depth fill: strictly aligned with upstream test_yopo_ros.py ──
        # Invalid pixels are uniformly filled with "the mean of valid pixels", and the
        # validity information is handed to the network through the mask channel
        # (channel 1) for it to judge -- that is the training-time input convention.
        # Never fill invalid regions as near obstacles: that contradicts the mask
        # semantics (the network already knows from the mask that those pixels are
        # invalid) and equals fabricating fake near obstacles, distorting the cost field.
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

        # ── 0. Update odometry (before depth processing, which needs the altitude) ──
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
        # Cache the CPU-side normalized depth (channel 0) so _process_output can compute
        # the forward obstacle distance, avoiding a .cpu().numpy() copy-back of the whole
        # depth image from the GPU on every inference.
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

        # ── Reference state sync (plan_from_reference) ──
        # Upstream semantics: the new trajectory starts from the last commanded
        # desire_pos, which keeps trajectories continuous.
        # But when the drone is moved a long way by external forces or manually (e.g.
        # dragged straight up to high altitude), desire_pos is still the old trajectory
        # reference and is badly out of sync with the real position, which causes:
        #   - distorted goal relative values (wrong goal direction, e.g. showing "the
        #     goal is overhead" when the drone should actually descend)
        #   - cmd position hundreds of metres away from the real position, yanking the
        #     drone hard in the wrong direction
        # When the reference drift exceeds one trajectory span, reset the reference state
        # to the actual odometry so the network plans from "the drone's true position to
        # the goal" relative values, with both altitude and direction correct.
        if self.desire_pos is not None:
            drift = np.linalg.norm(self.pos - self.desire_pos)
            if drift > self.traj_time * self.lattice_primitive.vel_max + 5.0:
                self.desire_pos = self.pos.copy()
                self.desire_vel = self.vel.copy()
                self.desire_acc = np.zeros(3)
                print(f"[ref sync] desire_pos drifted {drift:.0f}m from the real position, resetting to actual position")

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

        # ── 2c. Final approach: when the goal is close, skip slow inference and
        # decelerate straight to the goal with a polynomial ──
        # See the FINAL_APPROACH_DIST comment for the rationale. The polynomial ends at
        # the goal with zero terminal velocity/acceleration, is advanced continuously by
        # the 60 Hz control_update, and sets arrive once ARRIVE_THRESHOLD is crossed.
        if dist_to_goal < FINAL_APPROACH_DIST:
            # Final-approach takeover: when the goal is close, skip network inference and
            # plan a quintic polynomial straight to the goal with zero terminal
            # velocity/acceleration, decelerating smoothly to a stop at the goal (fixes
            # the 40 m oscillation caused by near-goal argmin overshoot).
            # No geometric avoidance intervention at all -- strictly follows YOPO's
            # learning-based avoidance.
            with self._lock:
                self._plan_final_approach()
                self.last_control_time = now
                cmd = self._compute_command()
            cmd["arrived"] = bool(self.arrive)
            cmd["dist_to_goal"] = dist_to_goal
            self.count += 1
            if self.count < 5 or self.count % 30 == 0:
                print(f"[YOPO final approach #{self.count}] dist={dist_to_goal:.1f}m -> decelerating straight to the goal")
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
        # The reactive-budget speed governor was removed: the replan interval is no longer
        # measured/smoothed to cap speed dynamically.

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

        # Aligned with upstream _run_inference/control_pub: no extra intervention such as
        # obstacle-free speed scaling or near-goal speed limiting.
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
            # traversed at the configured speed (the reactive-budget speed governor was
            # removed).
            # True cruise speed ~= vel_max * rate; the commanded PVA is scaled by the same
            # rate (see _compute_command).
            # The cap uses poly_duration: the final-approach polynomial can be longer than
            # the lattice traj_time, and capping at traj_time would cut it off midway.
            dt = now - self.last_control_time if self.last_control_time else 0.0
            dt = min(max(dt, 0.0), CTRL_DT) * self._time_rate
            cap = self.poly_duration if self.poly_duration else self.traj_time
            # Network trajectories keep extrapolating for TRAJ_EXTEND_S past
            # poly_duration (see the TRAJ_EXTEND_S comment) so the commanded position
            # keeps advancing instead of freezing; the final approach
            # (poly_extend=False) is capped strictly at T to stop at the goal.
            if self.poly_extend:
                cap += TRAJ_EXTEND_S
            self.ctrl_time = min(self.ctrl_time + dt, cap)
            self.last_control_time = now

            cmd = self._compute_command()

        # Aligned with upstream control_pub: no extra intervention such as obstacle-free
        # speed scaling or near-goal speed limiting.
        cmd["arrived"] = bool(self.arrive)
        cmd["dist_to_goal"] = float(dist_to_goal)
        return cmd

    def _process_output(self, endstate_pred, score_pred):
        """Select the best trajectory: strictly follows YOPO_360 test_yopo_ros.py and
        takes a plain argmin(score).

        No geometric avoidance intervention at all -- avoidance comes entirely from the
        score the network learned via safety_loss during training (learning-based
        avoidance), exactly matching the official deployment implementation.
        """
        N = self.lattice_primitive.traj_num
        raw = endstate_pred.reshape(9, N).T          # [N, 9] raw network output
        score = score_pred.reshape(N)

        # ── Forward obstacle distance (diagnostics only, not used for selection) ──
        # Reuses the CPU-side normalized depth cached during navigate's preprocessing,
        # avoiding a .cpu().numpy() copy-back of the whole depth image from the GPU on
        # every inference (192x384x4B ~= 288 KB per call) -- saves one D2H copy and one
        # large allocation, which matters for high-rate replanning (with TensorRT at
        # 1.3 ms that overhead is no longer negligible).
        fwd_dist = self.max_dis
        depth_map = getattr(self, '_last_depth_map_np', None)
        if depth_map is not None:
            self._last_depth_map = depth_map  # for the diagnostics endpoint
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
            print(f"[YOPO traj] action_id=#{action_id} alpha={chosen_alpha:+5.0f}deg "
                  f"beta={chosen_beta:+5.0f}deg score={float(score[action_id]):.3f} "
                  f"fwd obstacle={fwd_dist:5.1f}m")

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
            # Trajectory finished: extrapolate linearly at the terminal velocity, so the
            # commanded position keeps advancing at v(T) while the velocity stays v(T).
            # This avoids freezing the command at the endpoint, which would let the drone
            # fly past the frozen point and then be dragged back by the position loop
            # (root cause of the sawtooth, see TRAJ_EXTEND_S).
            # C0/C1 stay continuous at t=T (both position and velocity take the
            # polynomial's terminal values); acceleration is set to 0 (uniform straight
            # motion).
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

        # ── Pure time reparameterization: the commanded PVA must be self-consistent ──
        # cmdPos = p(ctrl_time), and ctrl_time advances at `rate` times real time, so the
        # true advance rate of the commanded position is p'(t)*rate. If the commanded
        # velocity still reported p'(t) (unscaled), the "velocity implied by the position"
        # and the "commanded velocity" would differ by a factor of rate -- they fight each
        # other: the drone can only chase hard through the client position loop's P term
        # (clamped to +/-15) and settles well behind the planned position (measured up to
        # a dozen metres), i.e. "flying at a position that was never planned", which makes
        # avoidance useless.
        # Hence the output velocity is multiplied by rate and the acceleration by rate^2,
        # so position/velocity/acceleration describe the same physical motion.
        rate = float(getattr(self, "_time_rate", 1.0))
        pos_mc = self._vec_ros_to_mc(pos_ros)
        vel_mc = self._vec_ros_to_mc(vel_ros) * rate
        acc_mc = self._vec_ros_to_mc(acc_ros) * (rate * rate)

        # Absolute speed ceiling (the hard backstop left after the reactive-budget speed
        # governor was removed): the norm of the commanded velocity vector must not exceed
        # YOPO_SPEED_CAP (default 15 m/s), guaranteeing "no limit ever goes above 15 m/s".
        # Only the excess is clipped and the direction is preserved; this never triggers
        # during normal cruise (rate=1, vel_max<=15).
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

        # Strictly aligned with upstream: no collision deceleration or speed scaling at
        # all. Avoidance is guaranteed entirely by the trajectory chosen by the network's
        # argmin(score).

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
    # Time-reparameterization rate (the reactive-budget speed governor was removed, only
    # the CTRL_TIME_SCALE fast-forward factor remains):
    resp["time_rate"] = round(float(getattr(srv, "_time_rate", 1.0)), 3)
    return jsonify(resp)


@app.route("/yopo/depth_diag", methods=["GET"])
def depth_diag():
    """Depth direction diagnostics: sample with the same formula as the collision filter
    and check the direction mapping against the drone's actual heading."""
    srv = _get_server()
    if srv._last_depth_map is None:
        return jsonify({"error": "no depth frame yet", "inference_count": srv.count})
    dm = srv._last_depth_map  # (H,W) normalized [0,1], 0=near 1=far
    H, W = dm.shape
    max_d = srv.max_dis

    def sample(alpha_deg, beta_deg):
        a = np.radians(alpha_deg)
        b = np.radians(beta_deg)
        col = int(round(W / 2 + a * W / (2 * np.pi)))   # positive alpha = turn left = right half of image
        row = int(round(H / 2 - b * H / np.pi))          # positive beta = up = top of image
        col = max(0, min(W - 1, col))
        row = max(0, min(H - 1, row))
        r0, r1 = max(0, row - 4), min(H, row + 5)
        c0, c1 = max(0, col - 8), min(W, col + 9)
        return round(float(dm[r0:r1, c0:c1].min()) * max_d, 2)

    # Drone nose heading in world frame (MC)
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
            "front alpha 0": sample(0, 0),
            "left alpha +90": sample(90, 0),
            "right alpha -90": sample(-90, 0),
            "rear alpha 180": sample(180, 0),
            "up beta +75": sample(0, 75),
            "down beta -75": sample(0, -75),
            "front-left alpha +45": sample(45, 0),
            "front-right alpha -45": sample(-45, 0),
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
    """Whether a text JSON frame is a navigate message (base64 fallback path).

    control must stay inside the event loop and run immediately; it must never be queued
    onto the thread pool -- otherwise the 50 Hz control loop gets blocked by navigate's
    inference time (self.pos freezes -> see _current_state).
    """
    try:
        return json.loads(raw).get("type") == "navigate"
    except Exception:
        return False


# navigate() is a long blocking task (depth cleanup + network inference). Running it
# synchronously inside the WS event loop starves every control message on the same
# connection for its whole duration, freezing self.pos/self.vel at the values captured at
# the start of navigate() -> the replanned polynomial starts from a stale point behind the
# body -> commands jump back and yank the drone (see the _current_state comment).
# Pushing it onto the thread pool lets control keep being handled immediately.
# max_workers=1 is mandatory: two navigate calls share self._last_depth_input /
# self._last_obs_input, so concurrent execution would let inference read another frame's
# depth or observation.
_ws_nav_executor = concurrent.futures.ThreadPoolExecutor(
    max_workers=1, thread_name_prefix="ws-navigate")


async def _ws_handler(websocket):
    remote = getattr(websocket, "remote_address", "unknown")
    print(f"[WS] client connected {remote}")
    loop = asyncio.get_running_loop()
    try:
        async for raw in websocket:
            try:
                # Binary frames are always navigate (see the bytes branch of
                # _ws_handle_message); text frames are classified by their type.
                # Only navigate goes to the thread pool, control stays synchronous and
                # immediate.
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
