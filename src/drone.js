/*
 * Copyright 2026 Manifold Tech Ltd.
 * Author: MENG Guotao <mengguotao@manifoldtech.cn>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Drone physics v3 — quaternion-based orientation.
 *
 * All rotations are applied in the drone's BODY frame via quaternion multiplication.
 * This eliminates Euler-angle cross-coupling: roll is always around the drone's
 * nose-to-tail axis regardless of heading.
 *
 * Geometry (top view = square):
 *   - droneSize: width = depth (configurable)
 *   - CG at center
 *   - Camera at front edge (CG + local forward * droneSize/2)
 *   - Thrust along local +Y through CG
 *   - Forward = local -Z at identity orientation
 *
 * FPV:   sticks → body-frame angular rates,  throttle → thrust,  no self-leveling
 * Drone: sticks → velocity command → position setpoint,  cascaded PI position/velocity/tilt hold
 */

import { reportUserError } from './error-report.js';

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;
const G = 9.81;              // gravitational acceleration (m/s²)
const AIR_DENSITY = 1.225;   // kg/m³ at sea level
const DRONE_BOOST_MULTIPLIER = 2.0;
const FPV_BOOST_MULTIPLIER = 1.7;
const DRONE_MAX_SUPPORTED_SPEED = 300 / 3.6; // 300 km/h in m/s
const DRONE_MAX_SUPPORTED_VSPEED = 25;

// Reusable PlayCanvas math objects (avoid per-frame allocation)
const _quat  = new pc.Quat();
const _quat2 = new pc.Quat();
const _mat4  = new pc.Mat4();
const _v3    = new pc.Vec3();

export class Drone {
    constructor() {
        // ---- Geometry ----
        this.droneSize = 0.5;

        // ---- State ----
        this.x = 0; this.y = 2; this.z = 0;
        this.vx = 0; this.vy = 0; this.vz = 0;

        // Quaternion orientation (single source of truth)
        this.orientation = new pc.Quat();

        // Angular velocity in body frame (deg/s)
        this.pitchRate = 0;
        this.rollRate  = 0;
        this.yawRate   = 0;

        // Euler angles (derived from orientation each frame, for HUD/readout)
        this.pitch = 0;
        this.roll  = 0;
        this.yaw   = 0;

        // ---- Tunable parameters ----
        this.flightMode  = 'drone';
        // Previous-frame flight mode: used by update() to detect mode
        // transitions and re-anchor position / integrator state so the new
        // mode starts cleanly from the drone's current pose.
        this._prevFlightMode = this.flightMode;
        this.mass        = 500;    // grams
        this.maxThrust   = 1000;   // grams-force
        this.dragCd      = 1.0;    // drag coefficient (dimensionless)
        this.dragArea     = 0.0015; // frontal area (m²), tuned for high-speed quad flight

        this.maxPitchRate = 220;
        this.maxRollRate  = 220;
        this.maxYawRate   = 120;
        this.droneMaxYawRate = 80;  // Drone mode yaw rate limit (deg/s)

        this.droneMaxAngle   = 58;
        this.droneAngleRate  = 280;
        this.droneMaxVSpeed  = 12.0;
        this.droneMaxSpeed   = DRONE_MAX_SUPPORTED_SPEED;

        // Cascaded PID gains
        this.dronePosKp  = 2.0;
        this.dronePosKi  = 0.3;
        this.dronePosKd  = 0.1;
        this.droneVelKp  = 3.0;
        this.droneVelKi  = 1.0;
        this.droneVelKd  = 0.05;
        this.droneAltKp  = 4.0;
        this.droneAltKi  = 2.0;
        this.droneAltKd  = 0.1;

        // Position-hold setpoints (horizontal XY + altitude Y). Drone mode
        // yaw is pure rate control and does not use a target heading.
        this._targetX = 0; this._targetY = 2; this._targetZ = 0;

        // Smoothed attitude targets (prevent limit-cycle at angle clamp)
        this._smoothTargetPitch = 0;
        this._smoothTargetRoll  = 0;

        // Integral accumulators (position loop)
        this._posIntX = 0; this._posIntY = 0; this._posIntZ = 0;
        // Integral accumulators (velocity loop)
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;
        // Previous errors for derivative term
        this._prevPosErrX = 0; this._prevPosErrY = 0; this._prevPosErrZ = 0;
        this._prevVelErrX = 0; this._prevVelErrY = 0; this._prevVelErrZ = 0;
        // Filtered derivative values (low-pass to suppress jitter)
        this._filtPosDerrX = 0; this._filtPosDerrY = 0; this._filtPosDerrZ = 0;
        this._filtVelDerrX = 0; this._filtVelDerrY = 0; this._filtVelDerrZ = 0;
        // Anti-windup limits
        this._posIntMax = 5.0;
        this._velIntMax = 15.0;

        this.angularDrag = 8.0;

        // ---- SimpleFlight 状态（级联 PID 积分/微分记忆）----
        this._sfVelIntX = 0; this._sfVelIntY = 0; this._sfVelIntZ = 0;
        this._sfPrevVelErrX = 0; this._sfPrevVelErrY = 0; this._sfPrevVelErrZ = 0;
        this._sfFiltVelDerrX = 0; this._sfFiltVelDerrY = 0; this._sfFiltVelDerrZ = 0;
        this._sfRateIntPitch = 0; this._sfRateIntRoll = 0; this._sfRateIntYaw = 0;
        this._sfPrevRateErrPitch = 0; this._sfPrevRateErrRoll = 0; this._sfPrevRateErrYaw = 0;
        this._sfPrevAngleErrPitch = 0; this._sfPrevAngleErrRoll = 0;
        this._sfFiltAngleDerrPitch = 0; this._sfFiltAngleDerrRoll = 0;
        this._sfPrevAltErr = 0;
        this._sfFiltAltDerr = 0;
        // SimpleFlight 增益（AirSim Params.hpp 默认值）
        this.sfPosKp = 1.0;
        this.sfVelKp = 5.0; this.sfVelKi = 0.0; this.sfVelKd = 1.0;
        this.sfAngleKp = 4.5; this.sfAngleKd = 0.1;
        this.sfRateKp = 0.8; this.sfRateKi = 0.0; this.sfRateKd = 0.0;
        this.sfAltKp = 2.0; this.sfAltKd = 0.5;
        this.sfYawRateKp = 1.0;
        this._sfVelIntMax = 15.0;
        this._sfRateIntMax = 50.0;

        // ---- YOPO 导航状态 ----
        this.yopoNavTarget = null;         // {x, y, z} 目标点
        this.yopoNavActive = false;       // 导航是否激活
        this.yopoArrived = false;         // 是否到达目标
        this.yopoDistToGoal = 0;          // 到目标距离
        this.arriveThreshold = 2.0;       // 到达判定半径 (米), matches test_yopo_ros.py L132
        // 终点接管距离: 网络在 goal_length (2*radio_range=10m) 内目标观测被归一化
        // 缩小, lattice 又全是巡航型轨迹, 接近目标时 argmin(score) 反复选出过冲/
        // 回头轨迹 → 速度/位置来回波动、到不了目标。距目标 12m 内不再跟随 YOPO
        // 轨迹, 改为直接对目标点做 PD 收敛(位置P+速度阻尼D+按距离限速)。
        this.yopoFinalApproachDist = 12.0; // 距目标 12m 内终点接管 (m)
        this.yopoArriveHoldM = 3.5;        // 客户端到达锁定的距离阈值 (m)
        this.yopoArriveHoldV = 1.0;        // 客户端到达锁定的速度阈值 (m/s)
        this.yopoCmdPos = null;           // {x, y, z} 当前指令位置
        this.yopoCmdVel = null;           // {x, y, z} 当前指令速度
        this.yopoCmdAcc = null;           // {x, y, z} 当前指令加速度
        this.yopoCmdTime = 0;            // performance.now() 时间戳，追踪 cmd 新鲜度
        this.yopoCmdYaw = 0;              // 当前指令偏航 (rad, ROS/drone yaw 约定)
        this.yopoCmdYawDot = 0;           // 当前指令偏航角速率 (rad/s)
        this.yopoDepthUnavailable = false; // DA360 深度不可用 → 悬停等待(不回退射线检测)
        this.yopoInferenceCount = 0;      // 推理计数
        this.yopoServerUrl = 'http://localhost:5689'; // YOPO 服务器地址

        // 仅依赖 YOPO_360 网络自身的学习式避障 (训练期 safety_loss, wc=8 学到),
        // 不叠加任何几何反应式避障/势场法。以下仅保留 YOPO 命令解析所需的安全与缩放参数,
        // 以及一个纯地形采样的被动地面安全网 (见 _controlYOPO 内硬性地面下限)。
        this.yopoCrashFloor = 1.0;    // 硬性地面安全下限 (m): 净空低于此值强制上爬, 防盲降撞地
        // 期望加速度安全上限 (m/s²): 远低于物理极角上限 (~15.7, maxAngle=58°).
        // 重规划(深度环)较慢时, 过大的加速度会让无人机在下一指令到达前冲入障碍. 限幅留余量.
        this.yopoAccMax = 8.0;
        // ── 几何反应式避障 (势场法, 基于 Cesium 真值射线) —— 参考 git 3b92a03 ──
        // 与 DA360 深度无关: 直接用 world.pickLocalRay 探测水平 360° 环形障碍距离 +
        // 地面/屋顶间隙 + 三层高度(当前/上/下), 生成排斥(rep)/切向绕行(tan)/
        // 近障刹车(brake)/竖直越障(vRep)。仅当障碍进入探测半径才生效, 路径畅通时
        // 输出为零 → 不影响正常导航。
        this.yopoAvoidEnabled = true;
        // 360° 均匀环形射线: 取代原 8 向粗采样(45° 间隔会在侧向/斜向留下大空隙, 漏检墙角/柱/凹槽)。
        // 生成 yopoAvoidRayCount 条等角分布的水平射线, 任意方向障碍都能被探测到。
        this.yopoAvoidRayCount = 36;       // 360° 射线数(10° 间隔); 越大越密、射线开销越高, 卡顿时可下调
        this.yopoAvoidRays = (() => {
            const arr = [], N = this.yopoAvoidRayCount;
            for (let i = 0; i < N; i++) {
                const a = (i * 2 * Math.PI) / N;   // 水平面等角分布, 覆盖完整 360°
                arr.push({ x: Math.cos(a), y: 0, z: Math.sin(a) });
            }
            return arr;
        })();
        this.yopoAvoidRange = 35.0;   // 障碍探测半径 (m) — 18m/s 巡航需更长前瞻: 刹车距离 v²/2a≈13.5m,
                                      // 35m 给足探测滞后+响应余量, 障碍更早进入感知。
        this.yopoAvoidRepRange = 20.0; // 排斥/切向/刹车作用距离 (m): 18m/s 下 20m 提供 ~1.1s 响应窗口,
                                      // 刹车距离 v²/2a≈10.8m < 20m, 更早介入且留足余量; goalClear 畅通
                                      // 阈值=本值, 故同步避免"走廊畅通判定"被过早解除。
        this.yopoAvoidGain = 10.0;    // 通用避障增益基准: 现主要用于竖直安全(upPush/vRep=
                                      // gain*系数), 水平 rep/tan 已拆为下方独立增益以便分别调强弱。
        this.yopoAvoidRepGain = 6.0;  // 排斥(径向推离)最大速度 (m/s): 6 比上版 4 略强, 遇障更易
                                      // 及时推离/绕开(配合 repRange 增大更及时); 仍远低于初版 10 避免过推。
        this.yopoAvoidTanGain = 14.0; // 切向(绕行)速度增益 (m/s): 调大→遇障时更果断朝目标侧绕行、
                                      // 更快绕过, 解决"排斥大、切向弱→总被推回而非绕开"。
        this.yopoAvoidDecel = 15.0;   // 安全刹车减速度 (m/s²): 已调大(原 12)→ 运动学 v_safe=√(2ad)
                                      // 更短, 触发距离减小后仍能刹停且留余量。物理上限 ~15.7(58° 倾角),
                                      // 取 15 逼近上限使减速更果断; 实际减速受 yopoAccMax 钳制。
        this.yopoAvoidQueryMs = 35;   // 射线探测节流 (ms): 18m/s 时每 0.63m 更新一次,
                                      // 探测更密 → 障碍信息更实时, 避障响应更快。
        this.yopoMinAlt = 2.5;        // 地面/屋顶最小净空 (m) — 软避障(上推)触发阈值
        this.yopoAvoidFanHalf = 0.8;  // 扇形射线束半宽 (m): 覆盖机体半宽+余量, 防凹槽漏检
        this.yopoAvoidFanRays = 3;    // 前向关键方向的扇形射线数(中心±两侧): 防凹窗/门洞/
                                      // 凹墙面漏检(单中心射线会穿过凹槽)。侧向/后向用 1 条
                                      // 中心射线(origin=机体 → 命中缓存), 帧率与安全平衡。
        this.yopoAvoidCeilRay = true; // 额外向上探测屋顶/悬挑下沿, 防钻入矮檐
        this.yopoAvoidCeilLook = 1.2; // 上仰探测高度 (m)
        this.yopoAvoidVertRay = true;     // 正上/正下竖直射线(防撞顶/撞正下方障碍)
        this.yopoAvoidVertRange = 12.0;   // 竖直射线探测范围 (m)
        // —— 竖直越障 (A+B 方案) ——
        this.yopoAvoidVStep = 8.0;        // 竖直探测抬升/下探步长 (m), *2 高层可越更高楼
        this.yopoAvoidVClimbScale = 0.9;  // 竖直越障速度 = gain*scale; 原 6*0.5=3m/s, 现 10*0.9=9m/s,
                                          // 高速下越障爬升更快; < droneMaxVSpeed=12 安全
        this.yopoAvoidVBlock = 12.0;      // 前进净空 < 此值即触发竖直越障 (m): 18m/s 下 stop+12≈13.1m≈0.73s,
                                          // 比原 8m(0.44s)留更多越障提前量, 免得临到障碍才爬
        this.yopoAvoidVClear = 0.45;      // 上层视为"畅通"的距离占比 (> R*该值 即畅通)
        this.yopoAvoidStop = this.yopoAvoidFanHalf + 0.3; // ≈1.1m 安全净空 (贴合机体)
        this._avoidProbe = null;      // 势场射线探测缓存
        this._avoidAccScale = 1.0;    // 势场刹车缩放 (加速度前馈衰减用)
        // YOPO 多项式加速度前馈缩放: 网络 cmdAcc 过大直接叠加会猛推, 削到 0.4 更柔和.
        this.yopoFFAccScale = 0.4;
        // 垂直速度前馈: 3D 导航下网络垂直轨迹(爬升/下降)是有效机动, 不再大幅
        // 缩放削弱; 仅保留温和限幅防猛冲(上限与位置环垂直误差限幅协同)。
        this.yopoFFVelYScale = 1.0;
        this.yopoFFVelYMax = 8.0;
        this.collisionRadius = 0.25;
        this.bounceDamping   = 0.3;
        this._collisionProvider = null; // 供碰撞解析/地形采样访问 (update 注入)

        // ---- Output state ----
        this.isColliding      = false;
        this.collisionIntensity = 0;
        this.speed            = 0;
        this.groundSpeed      = 0;
        this.airSpeed         = 0;
        this.verticalSpeed    = 0;
        this.thrustOutput     = 0;
        this.throttlePercent  = 0;
        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this.effectiveMaxSpeed = this.droneMaxSpeed;
        this.boostActive      = false;
        this.boostMultiplier  = 1.0;

        // Camera mount angle (degrees, positive = tilted up)
        // FPV mode: fixed during flight, set via settings (0..60)
        // Drone mode: live tilt via input (-90..0)
        this.cameraMountAngle = 30; // FPV default
        this.cameraTiltAngle  = 0;  // Drone mode live tilt

        // Spawn
        this._spawnX = 0; this._spawnY = 2; this._spawnZ = 0;
    }

    // ---- Public API ----

    setSpawnPoint(x, y, z) {
        this._spawnX = x; this._spawnY = y; this._spawnZ = z;
        this.reset();
    }

    reset() {
        this.x = this._spawnX; this.y = this._spawnY; this.z = this._spawnZ;
        this.vx = 0; this.vy = 0; this.vz = 0;
        this.orientation.set(0, 0, 0, 1); // identity
        this.pitchRate = 0; this.rollRate = 0; this.yawRate = 0;
        this.pitch = 0; this.roll = 0; this.yaw = 0;
        this.isColliding = false;
        this.collisionIntensity = 0;
        this.thrustOutput = 0;
        this.throttlePercent = 0;
        this.speed = 0;
        this.groundSpeed = 0;
        this.airSpeed = 0;
        this.verticalSpeed = 0;
        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this.effectiveMaxSpeed = this.droneMaxSpeed;
        this.boostActive = false;
        this.boostMultiplier = 1.0;
        this._targetX = this._spawnX; this._targetY = this._spawnY; this._targetZ = this._spawnZ;
        this._posIntX = 0; this._posIntY = 0; this._posIntZ = 0;
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;
        this._prevPosErrX = 0; this._prevPosErrY = 0; this._prevPosErrZ = 0;
        this._prevVelErrX = 0; this._prevVelErrY = 0; this._prevVelErrZ = 0;
        this._filtPosDerrX = 0; this._filtPosDerrY = 0; this._filtPosDerrZ = 0;
        this._filtVelDerrX = 0; this._filtVelDerrY = 0; this._filtVelDerrZ = 0;
        this._smoothTargetPitch = 0;
        this._smoothTargetRoll  = 0;
        // SimpleFlight 状态清零
        this._sfVelIntX = 0; this._sfVelIntY = 0; this._sfVelIntZ = 0;
        this._sfPrevVelErrX = 0; this._sfPrevVelErrY = 0; this._sfPrevVelErrZ = 0;
        this._sfFiltVelDerrX = 0; this._sfFiltVelDerrY = 0; this._sfFiltVelDerrZ = 0;
        this._sfRateIntPitch = 0; this._sfRateIntRoll = 0; this._sfRateIntYaw = 0;
        this._sfPrevRateErrPitch = 0; this._sfPrevRateErrRoll = 0; this._sfPrevRateErrYaw = 0;
        this._sfPrevAngleErrPitch = 0; this._sfPrevAngleErrRoll = 0;
        this._sfFiltAngleDerrPitch = 0; this._sfFiltAngleDerrRoll = 0;
        this._sfPrevAltErr = 0;
        this._sfFiltAltDerr = 0;
        // YOPO 状态清零
        this.yopoNavTarget = null;
        this.yopoNavActive = false;
        this.yopoArrived = false;
        this.yopoDistToGoal = 0;
        this.yopoCmdPos = null;
        this.yopoCmdVel = null;
        this.yopoCmdAcc = null;
        this.yopoCmdTime = 0;
        this.yopoCmdYaw = 0;
        this.yopoCmdYawDot = 0;
        this.yopoInferenceCount = 0;
    }

    readSettings() {
        const el = (id) => document.getElementById(id);
        const v  = (id) => { const e = el(id); return e ? parseFloat(e.value) : null; };
        const massVal   = v('phys-mass');
        const thrustVal = v('phys-thrust');
        const cdVal     = v('phys-drag-cd');
        const areaVal   = v('phys-drag-area');
        const radiusVal = v('phys-collision-radius');
        const sizeVal   = v('phys-drone-size');
        const droneMaxSpeedVal  = v('drone-max-speed');
        const droneMaxVSpeedVal = v('drone-max-vspeed');
        const modeEl    = el('flight-mode-select');
        const posKp = v('ctrl-pos-kp');
        const posKi = v('ctrl-pos-ki');
        const velKp = v('ctrl-vel-kp');
        const velKi = v('ctrl-vel-ki');
        const altKp = v('ctrl-alt-kp');
        const altKi = v('ctrl-alt-ki');
        if (massVal !== null)   this.mass = massVal;
        if (thrustVal !== null) this.maxThrust = thrustVal;
        if (cdVal !== null)     this.dragCd = cdVal;
        if (areaVal !== null)   this.dragArea = areaVal;
        if (radiusVal !== null) this.collisionRadius = radiusVal;
        if (sizeVal !== null)   this.droneSize = sizeVal;
        if (droneMaxSpeedVal !== null) {
            this.droneMaxSpeed = Math.max(1, Math.min(DRONE_MAX_SUPPORTED_SPEED, droneMaxSpeedVal));
        }
        if (droneMaxVSpeedVal !== null) {
            this.droneMaxVSpeed = Math.max(1, Math.min(DRONE_MAX_SUPPORTED_VSPEED, droneMaxVSpeedVal));
        }
        if (modeEl) this.flightMode = modeEl.value;
        const mountAngle = v('cam-mount-angle');
        if (mountAngle !== null) this.cameraMountAngle = mountAngle;
        const posKd = v('ctrl-pos-kd');
        const velKd = v('ctrl-vel-kd');
        const altKd = v('ctrl-alt-kd');
        if (posKp !== null) this.dronePosKp = posKp;
        if (posKi !== null) this.dronePosKi = posKi;
        if (posKd !== null) this.dronePosKd = posKd;
        if (velKp !== null) this.droneVelKp = velKp;
        if (velKi !== null) this.droneVelKi = velKi;
        if (velKd !== null) this.droneVelKd = velKd;
        if (altKp !== null) this.droneAltKp = altKp;
        if (altKi !== null) this.droneAltKi = altKi;
        if (altKd !== null) this.droneAltKd = altKd;

        // SimpleFlight 增益
        const sfPosKp = v('sf-pos-kp');
        const sfVelKp = v('sf-vel-kp');
        const sfVelKi = v('sf-vel-ki');
        const sfVelKd = v('sf-vel-kd');
        const sfAngleKp = v('sf-angle-kp');
        const sfAngleKd = v('sf-angle-kd');
        const sfRateKp = v('sf-rate-kp');
        const sfRateKi = v('sf-rate-ki');
        const sfAltKp = v('sf-alt-kp');
        const sfAltKd = v('sf-alt-kd');
        const sfYawRateKp = v('sf-yaw-rate-kp');
        if (sfPosKp !== null) this.sfPosKp = sfPosKp;
        if (sfVelKp !== null) this.sfVelKp = sfVelKp;
        if (sfVelKi !== null) this.sfVelKi = sfVelKi;
        if (sfVelKd !== null) this.sfVelKd = sfVelKd;
        if (sfAngleKp !== null) this.sfAngleKp = sfAngleKp;
        if (sfAngleKd !== null) this.sfAngleKd = sfAngleKd;
        if (sfRateKp !== null) this.sfRateKp = sfRateKp;
        if (sfRateKi !== null) this.sfRateKi = sfRateKi;
        if (sfAltKp !== null) this.sfAltKp = sfAltKp;
        if (sfAltKd !== null) this.sfAltKd = sfAltKd;
        if (sfYawRateKp !== null) this.sfYawRateKp = sfYawRateKp;
    }

    update(dt, input, collisionProvider) {
        dt = Math.min(dt, 0.05);

        // 记录碰撞提供者: 几何避障通过它访问 world.pickLocalRay / sampleHeightAtLocal
        if (collisionProvider) this._collisionProvider = collisionProvider;

        // 0. Handle flight-mode transitions (M key, RC channel, or dropdown).
        // readSettings() has already copied the latest dropdown value into
        // this.flightMode for this frame, so comparing against the cached
        // previous value detects a change on the first frame it becomes
        // effective.
        if (this.flightMode !== this._prevFlightMode) {
            this._onFlightModeChanged(this._prevFlightMode, this.flightMode);
            this._prevFlightMode = this.flightMode;
        }

        // 1. Control law → updates orientation quaternion and thrustOutput
        if (!input.armed) {
            this._updateDisarmed(dt);
        } else if (this.flightMode === 'drone') {
            this._controlDrone(dt, input);
        } else if (this.flightMode === 'simpleflight') {
            this._controlSimpleFlight(dt, input);
        } else if (this.flightMode === 'yopo_nav') {
            this._controlYOPO(dt, input);
        } else {
            this._controlFPV(dt, input);
        }

        // 2. Extract rotation matrix from orientation
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);

        // Local up = Y column of rotation matrix
        _mat4.getY(_v3);
        const upX = _v3.x, upY = _v3.y, upZ = _v3.z;

        // 3. Forces: thrust along local up + gravity + quadratic drag
        const massG = Math.max(this.mass, 1); // guard against zero mass
        const massKg = massG / 1000;
        // thrustOutput is in grams-force; convert to acceleration: (gf / g_mass) * G
        const thrustAccel = (this.thrustOutput / massG) * G;
        let ax = upX * thrustAccel;
        let ay = upY * thrustAccel - G;
        let az = upZ * thrustAccel;

        // Quadratic drag: F = 0.5 * Cd * A * rho * v^2, a = F / m
        const spd = Math.sqrt(this.vx * this.vx + this.vy * this.vy + this.vz * this.vz);
        if (spd > 0.001) {
            const dragForce = 0.5 * this.dragCd * this.dragArea * AIR_DENSITY * spd * spd;
            const dragAccel = dragForce / massKg;
            ax -= (this.vx / spd) * dragAccel;
            ay -= (this.vy / spd) * dragAccel;
            az -= (this.vz / spd) * dragAccel;
        }

        const previousPosition = { x: this.x, y: this.y, z: this.z };

        // 4. Integrate velocity & position
        this.vx += ax * dt;
        this.vy += ay * dt;
        this.vz += az * dt;
        this.x += this.vx * dt;
        this.y += this.vy * dt;
        this.z += this.vz * dt;

        // NaN guard — reset if physics blew up
        if (!Number.isFinite(this.x) || !Number.isFinite(this.y) || !Number.isFinite(this.z) ||
            !Number.isFinite(this.vx) || !Number.isFinite(this.vy) || !Number.isFinite(this.vz)) {
            reportUserError(
                'Drone physics produced invalid state; resetting',
                new Error(`pos=${this.x},${this.y},${this.z}, vel=${this.vx},${this.vy},${this.vz}, mass=${this.mass}, thrust=${this.thrustOutput}, dragCd=${this.dragCd}, dragArea=${this.dragArea}`),
                { key: 'drone-physics-nan', intervalMs: 10000 }
            );
            this.reset();
            return;
        }

        // 5. Collisions
        this._handleCollisions(collisionProvider, previousPosition, dt);

        // 6. Derive euler angles for HUD
        this._updateEulerFromQuat();
        this.groundSpeed = Math.sqrt(this.vx * this.vx + this.vz * this.vz);
        this.airSpeed = Math.sqrt(this.vx * this.vx + this.vy * this.vy + this.vz * this.vz);
        this.speed = this.groundSpeed;
        this.verticalSpeed = this.vy;
    }

    getCameraTransform() {
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);

        // Local forward = -Z column
        _mat4.getZ(_v3);
        _v3.mulScalar(-1);
        const halfSize = this.droneSize * 0.5;

        // Camera mount pitch offset (body-frame X rotation)
        const mountDeg = this.flightMode === 'fpv' ? this.cameraMountAngle : this.cameraTiltAngle;
        const mountRad = mountDeg * DEG2RAD * 0.5;
        _quat.set(Math.sin(mountRad), 0, 0, Math.cos(mountRad));
        _quat2.copy(this.orientation).mul(_quat);

        // Extract euler angles from camera orientation (with mount offset)
        const euler = this._quatToEuler(_quat2);

        return {
            position: {
                x: this.x + _v3.x * halfSize,
                y: this.y + _v3.y * halfSize,
                z: this.z + _v3.z * halfSize
            },
            rotation: { x: euler.x, y: euler.y, z: euler.z },
            orientation: { x: _quat2.x, y: _quat2.y, z: _quat2.z, w: _quat2.w }
        };
    }

    getPanoramaTransform() {
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);
        _v3.mulScalar(-1);
        const noseOffset = this.droneSize * 0.5;

        return {
            position: {
                x: this.x + _v3.x * noseOffset,
                y: this.y + _v3.y * noseOffset,
                z: this.z + _v3.z * noseOffset
            },
            rotation: { x: this.pitch, y: this.yaw, z: this.roll },
            orientation: {
                x: this.orientation.x,
                y: this.orientation.y,
                z: this.orientation.z,
                w: this.orientation.w
            }
        };
    }

    getBodyTransform() {
        return {
            position: { x: this.x, y: this.y, z: this.z },
            rotation: { x: this.pitch, y: this.yaw, z: this.roll },
            orientation: {
                x: this.orientation.x,
                y: this.orientation.y,
                z: this.orientation.z,
                w: this.orientation.w
            }
        };
    }

    adjustCameraTilt(delta) {
        this.cameraTiltAngle = Math.max(-90, Math.min(0, this.cameraTiltAngle + delta));
    }

    // ---- Orientation helpers ----

    /**
     * Apply an incremental body-frame rotation.
     * bodyAxis: 'x' (pitch), 'y' (yaw), or 'z' (roll)
     * angleDeg: rotation in degrees
     *
     * Body-frame: orientation = orientation * deltaQuat
     * World-frame (yaw): orientation = deltaQuat * orientation
     */
    _applyBodyRotation(axisX, axisY, axisZ, angleDeg) {
        if (Math.abs(angleDeg) < 1e-8) return;
        const halfRad = (angleDeg * DEG2RAD) * 0.5;
        const s = Math.sin(halfRad);
        _quat.set(axisX * s, axisY * s, axisZ * s, Math.cos(halfRad));
        // Body frame: q_new = q_current * q_delta
        _quat2.copy(this.orientation).mul(_quat);
        this.orientation.copy(_quat2).normalize();
    }


    /**
     * Decompose orientation into yaw (world Y rotation) and body tilt.
     * Returns { yawDeg, bodyPitchDeg, bodyRollDeg }
     */
    _decomposeOrientation() {
        // Extract yaw from the local +Z column projected onto the XZ plane.
        // R_Y(yaw) maps (0,0,1) → (sinYaw, 0, cosYaw), so:
        //   sinYaw = localZ.x,  cosYaw = localZ.z
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3); // local +Z direction in world
        const yawRad = Math.atan2(_v3.x, _v3.z);
        const yawDeg = yawRad * RAD2DEG;

        // Build yaw-only quaternion
        const halfYaw = yawRad * 0.5;
        _quat.set(0, Math.sin(halfYaw), 0, Math.cos(halfYaw));

        // Body tilt = inverse(yawQuat) * orientation
        _quat2.copy(_quat).invert().mul(this.orientation);

        // Extract pitch and roll from the tilt quaternion
        // tiltQuat represents R_X(pitch) * R_Z(roll) approximately
        const tiltEuler = new pc.Vec3();
        _quat2.getEulerAngles(tiltEuler);

        return {
            yawDeg: yawDeg,
            bodyPitchDeg: tiltEuler.x,
            bodyRollDeg: tiltEuler.z
        };
    }

    _updateEulerFromQuat() {
        const e = new pc.Vec3();
        this.orientation.getEulerAngles(e);
        this.pitch = e.x;
        this.yaw   = e.y;
        this.roll  = e.z;

        // Yaw-independent body tilt for OSD artificial horizon
        const dec = this._decomposeOrientation();
        this.bodyPitch = dec.bodyPitchDeg;
        this.bodyRoll  = dec.bodyRollDeg;
    }

    _quatToEuler(q) {
        const e = new pc.Vec3();
        q.getEulerAngles(e);
        return { x: e.x, y: e.y, z: e.z };
    }

    // ---- Control laws ----

    /**
     * Called once on the frame a flight-mode transition is detected.
     * Re-anchors position-hold + altitude-hold setpoints to the drone's
     * current state and clears PID integrator / derivative memory so the
     * new mode does not fly toward stale targets or apply leftover control
     * effort accumulated during the previous mode.
     *
     * Note on orientation: we deliberately do NOT reset pitch/roll here.
     * Drone mode's tilt controller will naturally level the craft over a
     * few hundred ms from whatever attitude FPV left behind, which matches
     * the user-visible "roll and pitch switch to level" expectation. Yaw
     * is pure rate control and needs no reset.
     */
    _onFlightModeChanged(oldMode, newMode) {
        this._targetX = this.x;
        this._targetY = this.y;
        this._targetZ = this.z;
        this._posIntX = 0; this._posIntY = 0; this._posIntZ = 0;
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;
        this._prevPosErrX = 0; this._prevPosErrY = 0; this._prevPosErrZ = 0;
        this._prevVelErrX = 0; this._prevVelErrY = 0; this._prevVelErrZ = 0;
        this._filtPosDerrX = 0; this._filtPosDerrY = 0; this._filtPosDerrZ = 0;
        this._filtVelDerrX = 0; this._filtVelDerrY = 0; this._filtVelDerrZ = 0;
        this._smoothTargetPitch = 0;
        this._smoothTargetRoll  = 0;
        // SimpleFlight 状态清零
        this._sfVelIntX = 0; this._sfVelIntY = 0; this._sfVelIntZ = 0;
        this._sfPrevVelErrX = 0; this._sfPrevVelErrY = 0; this._sfPrevVelErrZ = 0;
        this._sfFiltVelDerrX = 0; this._sfFiltVelDerrY = 0; this._sfFiltVelDerrZ = 0;
        this._sfRateIntPitch = 0; this._sfRateIntRoll = 0; this._sfRateIntYaw = 0;
        this._sfPrevRateErrPitch = 0; this._sfPrevRateErrRoll = 0; this._sfPrevRateErrYaw = 0;
        this._sfPrevAngleErrPitch = 0; this._sfPrevAngleErrRoll = 0;
        this._sfFiltAngleDerrPitch = 0; this._sfFiltAngleDerrRoll = 0;
        this._sfPrevAltErr = 0;
        this._sfFiltAltDerr = 0;
        // Reset YOPO velocity smoothing
        this._yopoVelSmoothX = undefined;
        this._yopoVelSmoothY = undefined;
        this._yopoVelSmoothZ = undefined;
        // YOPO 状态清零 — only when LEAVING yopo_nav mode.
        // When entering yopo_nav, preserve the target and active flag set by
        // the UI handler, otherwise navigation never starts.
        if (newMode !== 'yopo_nav') {
            this.yopoNavTarget = null;
            this.yopoNavActive = false;
            this.yopoArrived = false;
            this.yopoDistToGoal = 0;
            this.yopoCmdPos = null;
            this.yopoCmdVel = null;
            this.yopoCmdAcc = null;
            this.yopoCmdTime = 0;
            this.yopoCmdYaw = 0;
            this.yopoCmdYawDot = 0;
            this.yopoDepthUnavailable = false;
            this.yopoInferenceCount = 0;
        }
    }

    _updateDisarmed(dt) {
        this.thrustOutput = 0;
        this.throttlePercent = 0;
        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this.effectiveMaxSpeed = (this.flightMode === 'drone' || this.flightMode === 'simpleflight') ? this.droneMaxSpeed : null;
        this.boostActive = false;
        this.boostMultiplier = 1.0;
        // Damp angular rates
        const damp = Math.exp(-this.angularDrag * dt);
        this.pitchRate *= damp;
        this.rollRate  *= damp;
        this.yawRate   *= damp;

        // Auto-level toward identity tilt (keep current yaw)
        const dec = this._decomposeOrientation();
        const levelSpeed = 60; // deg/s
        const pitchStep = Math.min(levelSpeed * dt, Math.abs(dec.bodyPitchDeg));
        const rollStep  = Math.min(levelSpeed * dt, Math.abs(dec.bodyRollDeg));

        if (pitchStep > 0.01) {
            this._applyBodyRotation(1, 0, 0, -Math.sign(dec.bodyPitchDeg) * pitchStep);
        }
        if (rollStep > 0.01) {
            this._applyBodyRotation(0, 0, 1, -Math.sign(dec.bodyRollDeg) * rollStep);
        }
    }

    _controlFPV(dt, input) {
        const boost = input.boost ? FPV_BOOST_MULTIPLIER : 1.0;
        const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };
        this.boostActive = !!input.boost;
        this.boostMultiplier = boost;
        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this.effectiveMaxSpeed = null;

        // Sticks → target angular rates (body frame), scaled by rate
        const tPR = input.pitch * this.maxPitchRate * rates.pitch * boost;
        const tRR = -input.roll * this.maxRollRate * rates.roll * boost;
        const tYR = input.yaw  * this.maxYawRate  * rates.yaw  * boost;

        // Smooth rate tracking
        const s = 1 - Math.exp(-15 * dt);
        this.pitchRate += (tPR - this.pitchRate) * s;
        this.rollRate  += (tRR - this.rollRate)  * s;
        this.yawRate   += (tYR - this.yawRate)   * s;

        // Damp when centered
        const ad = Math.exp(-this.angularDrag * dt);
        if (Math.abs(input.pitch) < 0.05) this.pitchRate *= ad;
        if (Math.abs(input.roll)  < 0.05) this.rollRate  *= ad;
        if (Math.abs(input.yaw)   < 0.05) this.yawRate   *= ad;

        // Apply body-frame rotations
        this._applyBodyRotation(1, 0, 0, this.pitchRate * dt); // pitch around body X
        this._applyBodyRotation(0, 0, 1, this.rollRate * dt);  // roll around body Z
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);      // yaw around body Y

        // Throttle → thrust (in grams-force)
        this.thrustOutput = ((input.throttle + 1) * 0.5) * this.maxThrust * boost;
        this.throttlePercent = this.maxThrust > 0
            ? Math.max(0, Math.min(1, this.thrustOutput / (this.maxThrust * boost)))
            : 0;
    }

    _controlDrone(dt, input) {
        const boost = input.boost ? DRONE_BOOST_MULTIPLIER : 1.0;
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        this.boostActive = !!input.boost;
        this.boostMultiplier = boost;

        // ---- 1. Determine stick state and compute desired velocity ----
        // Get body-frame forward (-Z) and right (+X) in world XZ plane
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);
        let fwdX = -_v3.x, fwdZ = -_v3.z;
        _mat4.getX(_v3);
        let rightX = _v3.x, rightZ = _v3.z;

        const fwdLen = Math.sqrt(fwdX * fwdX + fwdZ * fwdZ);
        if (fwdLen > 1e-4) {
            fwdX /= fwdLen; fwdZ /= fwdLen;
        }
        const rightLen = Math.sqrt(rightX * rightX + rightZ * rightZ);
        if (rightLen > 1e-4) {
            rightX /= rightLen; rightZ /= rightLen;
        }

        const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };
        const maxSpd = Math.min(DRONE_MAX_SUPPORTED_SPEED, this.droneMaxSpeed * boost);
        this.effectiveMaxSpeed = maxSpd;

        const horizActive = Math.abs(input.pitch) > 0.05 || Math.abs(input.roll) > 0.05;
        const vertActive  = Math.abs(input.throttle) > 0.05;

        const yawActive = Math.abs(input.yaw) > 0.05;

        let vDesX, vDesY, vDesZ;
        let pilotCmdX = 0;
        let pilotCmdZ = 0;

        // ---- Horizontal: stick = target velocity, centered = position hold ----
        if (horizActive) {
            // Stick directly commands target velocity (body-frame → world-frame)
            const cmdFwd   = -input.pitch * maxSpd * rates.pitch;
            const cmdRight =  input.roll  * maxSpd * rates.roll;
            pilotCmdX = cmdFwd * fwdX + cmdRight * rightX;
            pilotCmdZ = cmdFwd * fwdZ + cmdRight * rightZ;
            const pilotCmdH = Math.sqrt(pilotCmdX * pilotCmdX + pilotCmdZ * pilotCmdZ);
            if (pilotCmdH > maxSpd) {
                const s = maxSpd / pilotCmdH;
                pilotCmdX *= s; pilotCmdZ *= s;
            }
            vDesX = pilotCmdX;
            vDesZ = pilotCmdZ;

            // Latch current position as hold target for when stick is released
            this._targetX = this.x;
            this._targetZ = this.z;
            // Clear position-loop state (not needed while stick is active)
            this._posIntX = 0; this._posIntZ = 0;
            this._filtPosDerrX = 0; this._filtPosDerrZ = 0;
            this._prevPosErrX = 0; this._prevPosErrZ = 0;
        } else {
            // Sticks centered → position hold via PID
            const posErrX = this._targetX - this.x;
            const posErrZ = this._targetZ - this.z;

            const piMax = this._posIntMax;
            this._posIntX = clamp(this._posIntX + posErrX * dt, -piMax, piMax);
            this._posIntZ = clamp(this._posIntZ + posErrZ * dt, -piMax, piMax);

            const dAlpha = 1 - Math.exp(-20 * dt);
            const rawPosDerrX = dt > 0 ? (posErrX - this._prevPosErrX) / dt : 0;
            const rawPosDerrZ = dt > 0 ? (posErrZ - this._prevPosErrZ) / dt : 0;
            this._filtPosDerrX += (rawPosDerrX - this._filtPosDerrX) * dAlpha;
            this._filtPosDerrZ += (rawPosDerrZ - this._filtPosDerrZ) * dAlpha;
            this._prevPosErrX = posErrX;
            this._prevPosErrZ = posErrZ;

            vDesX = this.dronePosKp * posErrX + this.dronePosKi * this._posIntX + this.dronePosKd * this._filtPosDerrX;
            vDesZ = this.dronePosKp * posErrZ + this.dronePosKi * this._posIntZ + this.dronePosKd * this._filtPosDerrZ;
        }

        // ---- Vertical: stick = target vertical speed, centered = altitude hold ----
        if (vertActive) {
            vDesY = input.throttle * this.droneMaxVSpeed * boost;

            // Latch current altitude as hold target
            this._targetY = this.y;
            this._posIntY = 0;
            this._filtPosDerrY = 0;
            this._prevPosErrY = 0;
        } else {
            const posErrY = this._targetY - this.y;

            const piMax = this._posIntMax;
            this._posIntY = clamp(this._posIntY + posErrY * dt, -piMax, piMax);

            const dAlpha = 1 - Math.exp(-20 * dt);
            const rawPosDerrY = dt > 0 ? (posErrY - this._prevPosErrY) / dt : 0;
            this._filtPosDerrY += (rawPosDerrY - this._filtPosDerrY) * dAlpha;
            this._prevPosErrY = posErrY;

            vDesY = this.droneAltKp * posErrY + this.droneAltKi * this._posIntY + this.droneAltKd * this._filtPosDerrY;
        }

        // Clamp desired velocity
        const vDesH = Math.sqrt(vDesX * vDesX + vDesZ * vDesZ);
        if (vDesH > maxSpd) {
            const s = maxSpd / vDesH;
            vDesX *= s; vDesZ *= s;
        }
        vDesY = clamp(vDesY, -this.droneMaxVSpeed * boost, this.droneMaxVSpeed * boost);
        this.targetGroundSpeed = Math.sqrt(vDesX * vDesX + vDesZ * vDesZ);
        this.pilotGroundSpeedCommand = Math.sqrt(pilotCmdX * pilotCmdX + pilotCmdZ * pilotCmdZ);
        this.commandedGroundSpeed = this.targetGroundSpeed;

        // ---- 2. Inner loop: Velocity PID → desired tilt angles ----
        const maxAngle = this.droneMaxAngle;
        let velErrX = vDesX - this.vx;
        const velErrY = vDesY - this.vy;
        let velErrZ = vDesZ - this.vz;

        // Clamp velocity error so acceleration demand stays within angle limit
        const aMaxHoriz = G * Math.tan(maxAngle * DEG2RAD);
        const velErrClamp = aMaxHoriz / this.droneVelKp;
        velErrX = clamp(velErrX, -velErrClamp, velErrClamp);
        velErrZ = clamp(velErrZ, -velErrClamp, velErrClamp);

        // Accumulate velocity integral (with anti-windup)
        const viMax = this._velIntMax;
        this._velIntX = clamp(this._velIntX + velErrX * dt, -viMax, viMax);
        this._velIntY = clamp(this._velIntY + velErrY * dt, -viMax, viMax);
        this._velIntZ = clamp(this._velIntZ + velErrZ * dt, -viMax, viMax);

        // Derivative of velocity error (low-pass filtered to suppress jitter)
        const vdAlpha = 1 - Math.exp(-15 * dt);
        const rawVelDerrX = dt > 0 ? (velErrX - this._prevVelErrX) / dt : 0;
        const rawVelDerrY = dt > 0 ? (velErrY - this._prevVelErrY) / dt : 0;
        const rawVelDerrZ = dt > 0 ? (velErrZ - this._prevVelErrZ) / dt : 0;
        this._filtVelDerrX += (rawVelDerrX - this._filtVelDerrX) * vdAlpha;
        this._filtVelDerrY += (rawVelDerrY - this._filtVelDerrY) * vdAlpha;
        this._filtVelDerrZ += (rawVelDerrZ - this._filtVelDerrZ) * vdAlpha;
        this._prevVelErrX = velErrX;
        this._prevVelErrY = velErrY;
        this._prevVelErrZ = velErrZ;

        // Desired world-frame horizontal acceleration
        const aDesX = this.droneVelKp * velErrX + this.droneVelKi * this._velIntX + this.droneVelKd * this._filtVelDerrX;
        const aDesZ = this.droneVelKp * velErrZ + this.droneVelKi * this._velIntZ + this.droneVelKd * this._filtVelDerrZ;

        // Project desired acceleration onto body forward/right to get tilt angles
        const aFwd   = aDesX * fwdX + aDesZ * fwdZ;
        const aRight = aDesX * rightX + aDesZ * rightZ;

        // Forward accel → negative pitch (nose down), right accel → positive roll
        const targetPitch = clamp(-aFwd / G * RAD2DEG, -maxAngle, maxAngle);
        const targetRoll  = clamp(-aRight / G * RAD2DEG, -maxAngle, maxAngle);

        // Smooth target angles to prevent residual oscillation at saturation boundary
        const smoothFactor = 1 - Math.exp(-10 * dt);
        this._smoothTargetPitch += (targetPitch - this._smoothTargetPitch) * smoothFactor;
        this._smoothTargetRoll  += (targetRoll  - this._smoothTargetRoll)  * smoothFactor;

        // ---- 3. Attitude P-controller: tilt error → body rotation ----
        const dec = this._decomposeOrientation();
        const pitchErr = this._smoothTargetPitch - dec.bodyPitchDeg;
        const rollErr  = this._smoothTargetRoll  - dec.bodyRollDeg;

        const maxStep = this.droneAngleRate * dt;
        const dpitch = clamp(pitchErr, -maxStep, maxStep);
        const droll  = clamp(rollErr,  -maxStep, maxStep);

        this._applyBodyRotation(1, 0, 0, dpitch);
        this._applyBodyRotation(0, 0, 1, droll);

        this.pitchRate = pitchErr * 5;
        this.rollRate  = rollErr  * 5;

        // ---- 4. Yaw: pure rate control, no target heading ----
        // Stick commands yaw rate directly; a centered stick damps the rate
        // toward zero (same pattern as FPV). This preserves whatever heading
        // the drone has at that moment — in particular, a FPV→drone switch
        // keeps the current heading instead of snapping to a stale setpoint.
        const droneYawMax = this.droneMaxYawRate * rates.yaw * boost;
        const tYR = input.yaw * droneYawMax;
        const ys = 1 - Math.exp(-15 * dt);
        this.yawRate += (tYR - this.yawRate) * ys;
        if (!yawActive) {
            // Stick centered → angular drag damps residual yaw rate to zero.
            this.yawRate *= Math.exp(-this.angularDrag * dt);
        }
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);

        // ---- 5. Altitude PID → thrust (in grams-force) ----
        const aDesY = this.droneVelKp * velErrY + this.droneVelKi * this._velIntY + this.droneVelKd * this._filtVelDerrY;
        let cmdGf = this.mass * (G + aDesY) / G;

        // Tilt compensation
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getY(_v3);
        const cosT = Math.max(0.1, _v3.y);
        cmdGf /= cosT;

        this.thrustOutput = clamp(cmdGf, 0, this.maxThrust * boost);
        this.throttlePercent = this.maxThrust > 0
            ? Math.max(0, Math.min(1, this.thrustOutput / (this.maxThrust * boost)))
            : 0;
    }

    /**
     * SimpleFlight 控制律 — AirSim simpleflight 级联 PID 端口。
     *
     * 4 层级联：位置环 (P) → 速度环 (PID) → 姿态环 (PD) → 角速率环 (PID)
     * 输入映射复用 drone 模式：俯仰/横滚=速度指令、油门=爬升率、
     * 偏航=偏航角速率，松杆=位置/高度锁定。
     * 输出契约与 _controlDrone 一致：thrustOutput (克力) +
     * _applyBodyRotation 累积姿态，由 update() 统一积分。
     */
    _controlSimpleFlight(dt, input) {
        const boost = input.boost ? DRONE_BOOST_MULTIPLIER : 1.0;
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        this.boostActive = !!input.boost;
        this.boostMultiplier = boost;

        // ---- 1. Body-frame forward/right (同 _controlDrone) ----
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);
        let fwdX = -_v3.x, fwdZ = -_v3.z;
        _mat4.getX(_v3);
        let rightX = _v3.x, rightZ = _v3.z;
        const fwdLen = Math.sqrt(fwdX * fwdX + fwdZ * fwdZ);
        if (fwdLen > 1e-4) { fwdX /= fwdLen; fwdZ /= fwdLen; }
        const rightLen = Math.sqrt(rightX * rightX + rightZ * rightZ);
        if (rightLen > 1e-4) { rightX /= rightLen; rightZ /= rightLen; }

        const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };
        const maxSpd = Math.min(DRONE_MAX_SUPPORTED_SPEED, this.droneMaxSpeed * boost);
        this.effectiveMaxSpeed = maxSpd;

        const horizActive = Math.abs(input.pitch) > 0.05 || Math.abs(input.roll) > 0.05;
        const vertActive  = Math.abs(input.throttle) > 0.05;

        // ---- 2. 位置环 (P) → 速度目标 ----
        let velTargetX, velTargetY, velTargetZ;
        let pilotCmdX = 0, pilotCmdZ = 0;
        if (horizActive) {
            const cmdFwd   = -input.pitch * maxSpd * rates.pitch;
            const cmdRight =  input.roll  * maxSpd * rates.roll;
            pilotCmdX = cmdFwd * fwdX + cmdRight * rightX;
            pilotCmdZ = cmdFwd * fwdZ + cmdRight * rightZ;
            const pilotCmdH = Math.sqrt(pilotCmdX * pilotCmdX + pilotCmdZ * pilotCmdZ);
            if (pilotCmdH > maxSpd) {
                const s = maxSpd / pilotCmdH;
                pilotCmdX *= s; pilotCmdZ *= s;
            }
            velTargetX = pilotCmdX;
            velTargetZ = pilotCmdZ;
            this._targetX = this.x;
            this._targetZ = this.z;
            this._sfVelIntX = 0; this._sfVelIntZ = 0;
            this._sfFiltVelDerrX = 0; this._sfFiltVelDerrZ = 0;
            this._sfPrevVelErrX = 0; this._sfPrevVelErrZ = 0;
        } else {
            const posErrX = this._targetX - this.x;
            const posErrZ = this._targetZ - this.z;
            velTargetX = this.sfPosKp * posErrX;
            velTargetZ = this.sfPosKp * posErrZ;
        }

        // 垂直：摇杆=爬升率，松杆=高度锁定 (PD)
        if (vertActive) {
            velTargetY = input.throttle * this.droneMaxVSpeed * boost;
            this._targetY = this.y;
            this._sfVelIntY = 0;
            this._sfFiltVelDerrY = 0;
            this._sfPrevVelErrY = 0;
            this._sfPrevAltErr = 0;
            this._sfFiltAltDerr = 0;
        } else {
            const altErr = this._targetY - this.y;
            const dAlpha = 1 - Math.exp(-20 * dt);
            const rawAltDerr = dt > 0 ? (altErr - this._sfPrevAltErr) / dt : 0;
            this._sfFiltAltDerr += (rawAltDerr - this._sfFiltAltDerr) * dAlpha;
            this._sfPrevAltErr = altErr;
            velTargetY = this.sfAltKp * altErr + this.sfAltKd * this._sfFiltAltDerr;
        }

        // 速度目标限幅
        const velTargetH = Math.sqrt(velTargetX * velTargetX + velTargetZ * velTargetZ);
        if (velTargetH > maxSpd) {
            const s = maxSpd / velTargetH;
            velTargetX *= s; velTargetZ *= s;
        }
        velTargetY = clamp(velTargetY, -this.droneMaxVSpeed * boost, this.droneMaxVSpeed * boost);
        this.targetGroundSpeed = Math.sqrt(velTargetX * velTargetX + velTargetZ * velTargetZ);
        this.pilotGroundSpeedCommand = Math.sqrt(pilotCmdX * pilotCmdX + pilotCmdZ * pilotCmdZ);
        this.commandedGroundSpeed = this.targetGroundSpeed;

        // ---- 3. 速度环 (PID) → 期望加速度 ----
        const velErrX = velTargetX - this.vx;
        const velErrY = velTargetY - this.vy;
        const velErrZ = velTargetZ - this.vz;

        // 限幅水平速度误差，使加速度需求不超过倾斜角上限
        const maxAngle = this.droneMaxAngle;
        const aMaxHoriz = G * Math.tan(maxAngle * DEG2RAD);
        const velErrClamp = aMaxHoriz / Math.max(0.01, this.sfVelKp);
        const velErrXc = clamp(velErrX, -velErrClamp, velErrClamp);
        const velErrZc = clamp(velErrZ, -velErrClamp, velErrClamp);

        // 积分 + 抗饱和
        const viMax = this._sfVelIntMax;
        this._sfVelIntX = clamp(this._sfVelIntX + velErrXc * dt, -viMax, viMax);
        this._sfVelIntY = clamp(this._sfVelIntY + velErrY  * dt, -viMax, viMax);
        this._sfVelIntZ = clamp(this._sfVelIntZ + velErrZc * dt, -viMax, viMax);

        // 微分 (低通滤波)
        const vdAlpha = 1 - Math.exp(-15 * dt);
        const rawVelDerrX = dt > 0 ? (velErrXc - this._sfPrevVelErrX) / dt : 0;
        const rawVelDerrY = dt > 0 ? (velErrY  - this._sfPrevVelErrY) / dt : 0;
        const rawVelDerrZ = dt > 0 ? (velErrZc - this._sfPrevVelErrZ) / dt : 0;
        this._sfFiltVelDerrX += (rawVelDerrX - this._sfFiltVelDerrX) * vdAlpha;
        this._sfFiltVelDerrY += (rawVelDerrY - this._sfFiltVelDerrY) * vdAlpha;
        this._sfFiltVelDerrZ += (rawVelDerrZ - this._sfFiltVelDerrZ) * vdAlpha;
        this._sfPrevVelErrX = velErrXc;
        this._sfPrevVelErrY = velErrY;
        this._sfPrevVelErrZ = velErrZc;

        const aDesX = this.sfVelKp * velErrXc + this.sfVelKi * this._sfVelIntX + this.sfVelKd * this._sfFiltVelDerrX;
        const aDesY = this.sfVelKp * velErrY  + this.sfVelKi * this._sfVelIntY + this.sfVelKd * this._sfFiltVelDerrY;
        const aDesZ = this.sfVelKp * velErrZc + this.sfVelKi * this._sfVelIntZ + this.sfVelKd * this._sfFiltVelDerrZ;

        // ---- 4. 投影到 body frame → 期望倾斜角 ----
        const aFwd   = aDesX * fwdX + aDesZ * fwdZ;
        const aRight = aDesX * rightX + aDesZ * rightZ;
        const targetPitch = clamp(-aFwd / G * RAD2DEG, -maxAngle, maxAngle);
        const targetRoll  = clamp(-aRight / G * RAD2DEG, -maxAngle, maxAngle);

        // ---- 5. 姿态环 (PD) → 期望角速率 ----
        const dec = this._decomposeOrientation();
        const angleErrPitch = targetPitch - dec.bodyPitchDeg;
        const angleErrRoll  = targetRoll  - dec.bodyRollDeg;
        // 微分低通滤波，抑制高频噪声
        const adAlpha = 1 - Math.exp(-15 * dt);
        const rawAngleDerrPitch = dt > 0 ? (angleErrPitch - this._sfPrevAngleErrPitch) / dt : 0;
        const rawAngleDerrRoll  = dt > 0 ? (angleErrRoll  - this._sfPrevAngleErrRoll)  / dt : 0;
        this._sfFiltAngleDerrPitch += (rawAngleDerrPitch - this._sfFiltAngleDerrPitch) * adAlpha;
        this._sfFiltAngleDerrRoll  += (rawAngleDerrRoll  - this._sfFiltAngleDerrRoll)  * adAlpha;
        this._sfPrevAngleErrPitch = angleErrPitch;
        this._sfPrevAngleErrRoll  = angleErrRoll;

        const rateTargetPitch = this.sfAngleKp * angleErrPitch + this.sfAngleKd * this._sfFiltAngleDerrPitch;
        const rateTargetRoll  = this.sfAngleKp * angleErrRoll  + this.sfAngleKd * this._sfFiltAngleDerrRoll;

        // ---- 6. 角速率环 (PID) → 期望角速度 → 平滑后应用 ----
        const rateErrPitch = rateTargetPitch - this.pitchRate;
        const rateErrRoll  = rateTargetRoll  - this.rollRate;
        const rateIntMax = this._sfRateIntMax;
        this._sfRateIntPitch = clamp(this._sfRateIntPitch + rateErrPitch * dt, -rateIntMax, rateIntMax);
        this._sfRateIntRoll  = clamp(this._sfRateIntRoll  + rateErrRoll  * dt, -rateIntMax, rateIntMax);
        const rateDerrPitch = dt > 0 ? (rateErrPitch - this._sfPrevRateErrPitch) / dt : 0;
        const rateDerrRoll  = dt > 0 ? (rateErrRoll  - this._sfPrevRateErrRoll)  / dt : 0;
        this._sfPrevRateErrPitch = rateErrPitch;
        this._sfPrevRateErrRoll  = rateErrRoll;

        const angVelPitch = this.sfRateKp * rateErrPitch + this.sfRateKi * this._sfRateIntPitch + this.sfRateKd * rateDerrPitch;
        const angVelRoll  = this.sfRateKp * rateErrRoll  + this.sfRateKi * this._sfRateIntRoll  + this.sfRateKd * rateDerrRoll;

        // 平滑角速度（模拟转动惯量，防止帧间瞬变导致抖动）
        const rateSmooth = 1 - Math.exp(-25 * dt);
        this.pitchRate += (angVelPitch - this.pitchRate) * rateSmooth;
        this.rollRate  += (angVelRoll  - this.rollRate)  * rateSmooth;
        this._applyBodyRotation(1, 0, 0, this.pitchRate * dt);
        this._applyBodyRotation(0, 0, 1, this.rollRate * dt);

        // ---- 7. 偏航：角速率 P 跟踪（同样平滑） ----
        const droneYawMax = this.droneMaxYawRate * rates.yaw * boost;
        const rateTargetYaw = input.yaw * droneYawMax;
        const rateErrYaw = rateTargetYaw - this.yawRate;
        this._sfRateIntYaw = clamp(this._sfRateIntYaw + rateErrYaw * dt, -rateIntMax, rateIntMax);
        const angVelYaw = this.sfYawRateKp * rateErrYaw;
        this.yawRate += (angVelYaw - this.yawRate) * rateSmooth;
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);

        // ---- 8. 高度 → 推力 (倾斜补偿) ----
        let cmdGf = this.mass * (G + aDesY) / G;
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getY(_v3);
        const cosT = Math.max(0.1, _v3.y);
        cmdGf /= cosT;

        this.thrustOutput = clamp(cmdGf, 0, this.maxThrust * boost);
        this.throttlePercent = this.maxThrust > 0
            ? Math.max(0, Math.min(1, this.thrustOutput / (this.maxThrust * boost)))
            : 0;
    }

    /**
     * YOPO 导航控制律 — 仅使用 YOPO 模型输出 (世界系位置/速度/加速度/偏航指令)。
     *
     * 控制器与 YOPO 适配: 轨迹跟踪 = 位置环 P + 速度前馈 + 加速度前馈, 偏航 P + yaw_dot
     * 前馈; 增益对齐 YOPO_360 SO3 控制器 (Hummingbird: kx=2, kv=1.8, kz=3.5)。
     * 速度环用 SO3 风格纯 P (无 I/D, 避免 replan 跳变导致积分绕偏/震荡); 姿态/角速率/
     * 推力级联沿用 SimpleFlight 同一套 PID, 推力做倾斜补偿。
     * 不叠加任何额外导航算法/修饰: 仅依赖 YOPO_360 网络自身的学习式避障, 几何反应式避障(势场法)已按需求删除。
     *
     * 终点 12m 内切换为对 yopoNavTarget 的 PD 收敛 + 距离限幅减速斜坡, 保证进入到达圈。
     */
    _controlYOPO(dt, input) {
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        this.boostActive = false;
        this.boostMultiplier = 1.0;
        this.effectiveMaxSpeed = this.droneMaxSpeed;

        // ---- 0. 检测摇杆活动 ----
        const horizActive = Math.abs(input.pitch) > 0.05 || Math.abs(input.roll) > 0.05;
        const vertActive  = Math.abs(input.throttle) > 0.05;
        const yawActive   = Math.abs(input.yaw) > 0.05;
        const stickActive = horizActive || vertActive || yawActive;

        // ---- 0b. 目标点距离 + 终点接管判定 ----
        // 网络在 goal_length (2*radio_range=10m) 内目标观测被按 10m 归一化缩小,
        // lattice 又全是巡航型轨迹(端点速度可达 vel_max≈6m/s), 因此接近目标时
        // argmin(score) 会反复选出过冲/回头的轨迹; 叠加 plan_from_reference 下
        // 参考点一旦越过目标, 目标方向观测翻转 → 速度/位置来回波动、到不了目标。
        // 距目标 yopoFinalApproachDist 内不再跟随 YOPO 轨迹, 改为直接对目标点做 PD 收敛,
        // 保证最终进入到达圈。
        let distGoal = Number.POSITIVE_INFINITY;
        if (this.yopoNavTarget) {
            const gdx = this.yopoNavTarget.x - this.x;
            const gdy = this.yopoNavTarget.y - this.y;
            const gdz = this.yopoNavTarget.z - this.z;
            distGoal = Math.sqrt(gdx * gdx + gdy * gdy + gdz * gdz);
        }
        this.yopoDistToGoal = distGoal;

        // 客户端到达锁定 (兜底): 目标 yopoArriveHoldM 内且速度 < yopoArriveHoldV → 视为到达。
        // server 的 2m 到达判定是异步返回的, 若轨迹在 2m 圈外轻微滞留,
        // 该兜底确保客户端切换到终点悬停, 避免"永远差一步"。
        if (this.yopoNavTarget && distGoal < this.yopoArriveHoldM) {
            const spdNow = Math.sqrt(this.vx*this.vx + this.vy*this.vy + this.vz*this.vz);
            if (spdNow < this.yopoArriveHoldV) this.yopoArrived = true;
        }

        const yopoNearGoalHold =
            this.yopoNavTarget && !stickActive &&
            (this.yopoArrived || distGoal < this.yopoFinalApproachDist);

        // ---- 1. 诊断日志 ----
        if (this.yopoInferenceCount < 5 || this.yopoInferenceCount % 120 === 0) {
            const hasCmd = this.yopoCmdPos ? 'YES' : 'NO';
            const cmdStr = this.yopoCmdPos
                ? `cmd=(${this.yopoCmdPos.x.toFixed(1)},${this.yopoCmdPos.y.toFixed(1)},${this.yopoCmdPos.z.toFixed(1)})`
                : '';
            console.log(`_controlYOPO #${this.yopoInferenceCount}: armed=${input.armed} hasCmd=${hasCmd} ${cmdStr} ` +
                `pos=(${this.x.toFixed(1)},${this.y.toFixed(1)},${this.z.toFixed(1)})`);
        }

        // ---- 2. 机体前向/右向 (与 _controlSimpleFlight 同义) ----
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);
        let fwdX = -_v3.x, fwdZ = -_v3.z;
        _mat4.getX(_v3);
        let rightX = _v3.x, rightZ = _v3.z;
        const fwdLen = Math.sqrt(fwdX * fwdX + fwdZ * fwdZ);
        if (fwdLen > 1e-4) { fwdX /= fwdLen; fwdZ /= fwdLen; }
        const rightLen = Math.sqrt(rightX * rightX + rightZ * rightZ);
        if (rightLen > 1e-4) { rightX /= rightLen; rightZ /= rightLen; }

        // YOPO 导航最大水平速度。服务端默认 YOPO_VELOCITY=8.0(巡航 vel_max≈8),
        // 位置环误差贡献限幅 4 m/s + 速度前馈 12 m/s → 峰值≈16 m/s, 钳制到 13
        // 保留跟踪余量, 避免网络切换高速轨迹时位置误差把速度目标顶到上限造成"突然猛冲"。
        // 提速到 20.0 m/s: 配合服务端 YOPO_VELOCITY=15 + 客户端 yopoPosErrMaxV=15 解锁位置环巡航,
        // 实际巡航 ~12-15 m/s、端点 ~16-19; 硬上限 20 防爆速猛冲。避障已配套加大探测半径/刹车/增益。
        const yopoMaxSpd = 20.0;
        const maxSpd = stickActive ? this.droneMaxSpeed : yopoMaxSpd;
        const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };

        // ---- 3. 确定速度目标 ----
        let velTargetX, velTargetZ, velTargetY;
        let pilotCmdX = 0, pilotCmdZ = 0;
        let useAccFeedforward = false;

        if (yopoNearGoalHold) {
            // 终点接管: 位置 P + 速度阻尼 D 直接收敛到目标点。
            // 最大速度随距离递减 → 自然减速斜坡: 12m→~3m/s, 5m→~1.75m/s,
            // 1m→~0.8m/s(下限), 配合 -holdKd*v 阻尼, 平滑停在目标点。
            // 撞墙/贴地时压到低速, 避免顶着障碍来回顶撞。
            const gErrX = this.yopoNavTarget.x - this.x;
            const gErrZ = this.yopoNavTarget.z - this.z;
            const gErrY = this.yopoNavTarget.y - this.y;
            const holdKp = 1.5, holdAltKp = 2.5, holdKd = 1.5;
            const collideStall = this.isColliding ? 0.35 : 1.0;
            const holdMaxV = Math.max(0.3, Math.min(4.0, distGoal * 0.4)) * collideStall;
            velTargetX = holdKp * gErrX - holdKd * this.vx;
            velTargetZ = holdKp * gErrZ - holdKd * this.vz;
            velTargetY = holdAltKp * gErrY - holdKd * this.vy;
            const vh = Math.sqrt(velTargetX*velTargetX + velTargetZ*velTargetZ);
            if (vh > holdMaxV) { const s = holdMaxV / vh; velTargetX *= s; velTargetZ *= s; }
        } else if (stickActive) {
            // 摇杆抢占：使用人工控制
            if (horizActive) {
                const cmdFwd   = -input.pitch * maxSpd * rates.pitch;
                const cmdRight =  input.roll  * maxSpd * rates.roll;
                pilotCmdX = cmdFwd * fwdX + cmdRight * rightX;
                pilotCmdZ = cmdFwd * fwdZ + cmdRight * rightZ;
                const pilotCmdH = Math.sqrt(pilotCmdX * pilotCmdX + pilotCmdZ * pilotCmdZ);
                if (pilotCmdH > maxSpd) {
                    const s = maxSpd / pilotCmdH;
                    pilotCmdX *= s; pilotCmdZ *= s;
                }
                velTargetX = pilotCmdX;
                velTargetZ = pilotCmdZ;
                this._targetX = this.x;
                this._targetZ = this.z;
            } else {
                velTargetX = 0;
                velTargetZ = 0;
            }
            velTargetY = vertActive ? input.throttle * this.droneMaxVSpeed : 0;
            if (vertActive) this._targetY = this.y;
        } else if (this.yopoCmdPos) {
            // YOPO 轨迹指令：位置环 P + 速度前馈 + 加速度前馈
            // yopoCmdPos/Vel/Acc 是多项式评估的期望状态（plan_from_reference）。
            // 位置环只补偿跟踪偏差，速度+加速度前馈主导跟踪，确保高效精准。
            //
            // 增益对齐 YOPO_360 SO3 控制器 (Hummingbird: kx=2, kv=1.8, kz=3.5)。
            // 级联结构等效: kx_eff = velKp*yopoPosKp, kv_eff = velKp。
            // 不加 Ki/Kd：YOPO 每次 replan 时 ffVel 跳变，Ki 会积分绕偏导致
            // "一前一后"震荡，Kd 会在跳变处产生加速度尖峰。SO3 本身无 I/D。
            const posErrX = this.yopoCmdPos.x - this.x;
            const posErrZ = this.yopoCmdPos.z - this.z;
            const posErrY = this.yopoCmdPos.y - this.y;

            // 命令过期保护: 60Hz 控制环下命令始终新鲜。仅在控制环崩溃(>3s)时衰减。
            const cmdAgeS = (performance.now() - this.yopoCmdTime) / 1000;
            const ffDecay = cmdAgeS < 3.0 ? 1.0 : Math.max(0, 1.0 - (cmdAgeS - 3.0) / 1.0);
            const ffX = (this.yopoCmdVel ? this.yopoCmdVel.x : 0) * ffDecay;
            const ffZ = (this.yopoCmdVel ? this.yopoCmdVel.z : 0) * ffDecay;
            // 垂直速度前馈缩放+限幅: 网络垂直轨迹常过冲(本应爬升却给向下巨速),
            // 若直接叠加会盖过位置环导致猛扎。限幅后位置环(知道真实高度误差)主导。
            let ffY = (this.yopoCmdVel ? (this.yopoCmdVel.y || 0) : 0) * ffDecay * this.yopoFFVelYScale;
            if (ffY > this.yopoFFVelYMax) ffY = this.yopoFFVelYMax;
            else if (ffY < -this.yopoFFVelYMax) ffY = -this.yopoFFVelYMax;

            const yopoPosKp = 1.0;   // 位置环增益: 兼顾"拉回旧指令位置"趋势与巡航速度。配合服务端时间缩放, 让无人机更紧地追上更快的指令。
            const yopoAltKp = 1.2;   // 高度环增益: 3D 导航下垂直误差由网络轨迹主导, 位置环仅纠偏
            // 位置误差项限幅: 原为 4.0 把巡航死死卡在 ±4 m/s(用户实测 0~4m/s 的根因——
            // 当服务端速度前馈 ffX≈0 时, 无人机只能靠位置环追 cmdPos, 被 4 上限限成龟速)。
            // 提到 15 后, 位置环可输出与 YOPO 规划速度(~15)匹配的水平/垂直速度, 真正解锁巡航;
            // 仍由 yopoMaxSpd 硬钳制防 replan 跳变猛冲, 且避障已加强足以托住高速。
            const yopoPosErrMaxV = 15.0;  // 水平位置误差贡献上限 (m/s): 匹配 yopo 速度上限 ~15
            const yopoAltErrMaxV = 15.0;  // 垂直位置误差贡献上限 (m/s): 3D 导航允许垂直机动
            velTargetX = clamp(yopoPosKp * posErrX, -yopoPosErrMaxV, yopoPosErrMaxV) + ffX;
            velTargetZ = clamp(yopoPosKp * posErrZ, -yopoPosErrMaxV, yopoPosErrMaxV) + ffZ;
            velTargetY = clamp(yopoAltKp * posErrY, -yopoAltErrMaxV, yopoAltErrMaxV) + ffY;
            useAccFeedforward = true;
        } else if (this.yopoCmdVel && (Math.abs(this.yopoCmdVel.x) > 0.01 || Math.abs(this.yopoCmdVel.z) > 0.01)) {
            // 仅有 YOPO 速度指令（无位置指令）→ 纯速度跟踪
            velTargetX = this.yopoCmdVel.x;
            velTargetZ = this.yopoCmdVel.z;
            velTargetY = this.yopoCmdVel.y || 0;
        } else {
            // 无 YOPO 指令 → 悬停（不直线飞向目标，避免绕过避障）
            velTargetX = 0; velTargetZ = 0;
            velTargetY = 0;
        }

        // ── 几何反应式避障 (势场法, 参考 git 3b92a03) ──
        // 基于 Cesium 真值射线: 探测水平 360° 环形障碍距离 + 地面/屋顶间隙 + 三层高度,
        // 生成排斥(rep)/切向绕行(tan)/近障刹车(brake)/竖直越障(vRep)。这是**主动**
        // 避障层: 中距(4~25m)就开始连续绕行+刹车。路径通畅时输出为零 → 不影响导航。
        this._avoidAccScale = 1.0;
        // 终点接管阶段(yopoNearGoalHold: 距目标 <12m 或已到达)PD 已直接收敛到目标点。
        // 此阶段势场仍保留"绕行(tan)+减速(brake)+竖直防撞", 但**不叠加法向 rep** ——
        // 否则 rep 把无人机推离目标, 与 PD 在同一直线来回拉扯 → "前后摆动/徘徊", 且被
        // 推离后 yopoArrived 永远置不上、势场永久生效(死循环)。只取 tan+brake 既能避开
        // 接管范围内的障碍(绕行+减速), 又不与 PD 对抗。碰撞另有 _handleCollisions 兜底。
        if (this.yopoAvoidEnabled && this.yopoNavTarget &&
            !stickActive && !this.yopoArrived) {
            this._updateAvoidProbe();
            const avoid = this._avoidanceVelocity(velTargetX, velTargetZ);
            if (avoid) {
                if (yopoNearGoalHold) {
                    // 终点接管: 仅叠加绕行(tan)与减速(brake), 不加法向 rep(避免推离目标摆动)
                    velTargetX = velTargetX * avoid.brake + avoid.tanX;
                    velTargetZ = velTargetZ * avoid.brake + avoid.tanZ;
                } else {
                    velTargetX = velTargetX * avoid.brake + avoid.repX + avoid.tanX;
                    velTargetZ = velTargetZ * avoid.brake + avoid.repZ + avoid.tanZ;
                }
                // 竖直障碍(正下/正上方有障)水平绕行: 叠加 vGo 平滑离开障碍足迹(不升不降)
                velTargetX += avoid.vGoX;
                velTargetZ += avoid.vGoZ;
                // 竖直: 地面净空上推 + 竖直越障 + 下降运动学刹车。终点段同样保留(防撞地/顶/下障)。
                velTargetY = velTargetY * avoid.brake;
                if (avoid.vRep) velTargetY = velTargetY * 0.3 + avoid.vRep;
                velTargetY += avoid.upPush;
                // 垂直下降运动学刹车: 允许的最大下降速度 = vSafeDown (>=0)。
                // 若网络轨迹要求更快的下降(velTargetY 很负), 钳制到 -vSafeDown,
                // 保证在正下方/前下方净空内物理上刹得住 → 不撞下方障碍。
                if (avoid.vSafeDown !== null && Number.isFinite(avoid.vSafeDown)) {
                    if (velTargetY < -avoid.vSafeDown) {
                        velTargetY = -avoid.vSafeDown;
                        this._yopoGroundFloorActive = true; // 触发上爬/悬停姿态
                    }
                }
                // 正下/正上方有障碍: 保持高度、不升不降, 完全交给水平绕行 vGo 平滑飞过,
                // 避免"想下降→被射线/碰撞推开→又想下降"的来回抖动。
                if (Math.hypot(avoid.vGoX, avoid.vGoZ) > 1e-6) {
                    velTargetY = 0;
                }
                this._avoidAccScale = avoid.brake;
            }
        }

        // ── 被动地面安全网 (非几何避障) ──
        // 几何反应式避障(势场法)已按需求删除, 仅保留基于地形高度采样的被动安全网:
        // 当净空低于 yopoCrashFloor 时强制上爬, 防止快速飞行 + 重规划间隙盲降撞地。
        this._yopoGroundFloorActive = false;
        const cp = this._collisionProvider;
        const w = cp ? cp.world : null;
        let groundGap = Number.POSITIVE_INFINITY;
        if (w && w.ready && typeof w.sampleHeightAtLocal === 'function') {
            const gy = w.sampleHeightAtLocal(this.x, this.z, 0.6);
            if (Number.isFinite(gy)) groundGap = this.y - gy;
        }
        if (Number.isFinite(groundGap) && groundGap < this.yopoCrashFloor) {
            // 越接近地面, 上爬速率越大; 至少 +1 m/s 保证脱离。
            const climb = (this.yopoCrashFloor - groundGap) * 4.0 + 1.0;
            if (velTargetY < climb) velTargetY = climb;
            this._yopoGroundFloorActive = true;
        }

        // 速度目标限幅
        const velTargetH = Math.sqrt(velTargetX * velTargetX + velTargetZ * velTargetZ);
        if (velTargetH > maxSpd) {
            const s = maxSpd / velTargetH;
            velTargetX *= s; velTargetZ *= s;
        }
        velTargetY = clamp(velTargetY, -this.droneMaxVSpeed, this.droneMaxVSpeed);
        this.targetGroundSpeed = Math.sqrt(velTargetX * velTargetX + velTargetZ * velTargetZ);

        // 诊断: 记录速度目标
        if (this.yopoInferenceCount < 5 || this.yopoInferenceCount % 120 === 0) {
            console.log(`_controlYOPO velTarget=(${velTargetX.toFixed(2)},${velTargetY.toFixed(2)},${velTargetZ.toFixed(2)}) ` +
                `stickActive=${stickActive} thrust=${this.thrustOutput.toFixed(0)}`);
        }
        this.pilotGroundSpeedCommand = Math.sqrt(pilotCmdX * pilotCmdX + pilotCmdZ * pilotCmdZ);
        this.commandedGroundSpeed = this.targetGroundSpeed;

        // ---- 4. 速度环 (PID) → 期望加速度 ----
        // YOPO 轨迹跟踪使用 SO3 风格纯 P 速度环（无 I/D）：
        //   - 无 Ki：避免 replan 时 ffVel 跳变造成的积分绕偏与"一前一后"震荡
        //   - 无 Kd：避免 replan 跳变处 d(velErr)/dt 产生加速度/倾斜尖峰
        // 增益取 1.5(低于 SO3 hummingbird kv≈1.8, 用户要求补偿不要太高):
        //   由 ffVel/ffAcc 前馈主导跟踪, P 环只做柔和纠偏, 运动更平滑、少拉扯。
        // 摇杆/悬停模式仍用 SimpleFlight 默认 PID 增益。
        const velErrX = velTargetX - this.vx;
        const velErrY = velTargetY - this.vy;
        const velErrZ = velTargetZ - this.vz;

        const maxAngle = this.droneMaxAngle;
        const aMaxHoriz = G * Math.tan(maxAngle * DEG2RAD);
        // YOPO 专用速度环参数(已调低, 补偿柔和)
        const velKp = useAccFeedforward ? 2.2 : this.sfVelKp;
        const velKi = useAccFeedforward ? 0.0 : this.sfVelKi;
        const velKd = useAccFeedforward ? 0.0 : this.sfVelKd;
        const velErrClamp = aMaxHoriz / Math.max(0.01, velKp);
        const velErrXc = clamp(velErrX, -velErrClamp, velErrClamp);
        const velErrZc = clamp(velErrZ, -velErrClamp, velErrClamp);

        const viMax = this._sfVelIntMax;
        this._sfVelIntX = clamp(this._sfVelIntX + velErrXc * dt, -viMax, viMax);
        this._sfVelIntY = clamp(this._sfVelIntY + velErrY  * dt, -viMax, viMax);
        this._sfVelIntZ = clamp(this._sfVelIntZ + velErrZc * dt, -viMax, viMax);

        const vdAlpha = 1 - Math.exp(-15 * dt);
        const rawVelDerrX = dt > 0 ? (velErrXc - this._sfPrevVelErrX) / dt : 0;
        const rawVelDerrY = dt > 0 ? (velErrY  - this._sfPrevVelErrY) / dt : 0;
        const rawVelDerrZ = dt > 0 ? (velErrZc - this._sfPrevVelErrZ) / dt : 0;
        this._sfFiltVelDerrX += (rawVelDerrX - this._sfFiltVelDerrX) * vdAlpha;
        this._sfFiltVelDerrY += (rawVelDerrY - this._sfFiltVelDerrY) * vdAlpha;
        this._sfFiltVelDerrZ += (rawVelDerrZ - this._sfFiltVelDerrZ) * vdAlpha;
        this._sfPrevVelErrX = velErrXc;
        this._sfPrevVelErrY = velErrY;
        this._sfPrevVelErrZ = velErrZc;

        // 速度环 PID → 期望加速度
        let aDesX = velKp * velErrXc + velKi * this._sfVelIntX + velKd * this._sfFiltVelDerrX;
        let aDesY = velKp * velErrY  + velKi * this._sfVelIntY + velKd * this._sfFiltVelDerrY;
        let aDesZ = velKp * velErrZc + velKi * this._sfVelIntZ + velKd * this._sfFiltVelDerrZ;

        // 加速度前馈：YOPO 多项式加速度直接叠加，提高轨迹跟踪精度和效率。
        // SO3-style P 控制器对 ffAcc 依赖更强（无 Ki/Kd 掩盖），但 cmd 在
        // 两次 server 响应间会变陈旧（深度捕获 ~100-300ms）。陈旧的 ffAcc
        // 来自旧 ctrl_time 的多项式，方向与大小都可能错。按 cmd 年龄线性
        // 衰减：<80ms 全量，80-200ms 线性降至 0，>200ms 关闭。
        if (useAccFeedforward && this.yopoCmdAcc) {
            const cmdAgeMs = this.yopoCmdTime > 0 ? (performance.now() - this.yopoCmdTime) : 999;
            let ffScale = this.yopoFFAccScale;
            if (cmdAgeMs > 200) {
                ffScale = 0.0;
            } else if (cmdAgeMs > 80) {
                ffScale = 1.0 - (cmdAgeMs - 80) / 120;
            }
            // 势场避障刹车时衰减加速度前馈: 否则多项式 ffAcc 仍会把无人机"顶向"障碍
            ffScale *= this._avoidAccScale || 1.0;
            aDesX += this.yopoCmdAcc.x * ffScale;
            aDesY += (this.yopoCmdAcc.y || 0) * ffScale;
            aDesZ += this.yopoCmdAcc.z * ffScale;
            // 硬地面下限触发时, 禁止向下加速度前馈(否则会抵消上爬), 并强制 aDesY>=0
            if (this._yopoGroundFloorActive) {
                if (this.yopoCmdAcc && this.yopoCmdAcc.y < 0) aDesY -= this.yopoCmdAcc.y * ffScale;
                if (aDesY < 0) aDesY = 0;
            }
        }

        // ── 期望加速度安全上限 (防止"加速度过大→下一个指令前撞障碍") ──
        // 重规划(深度环)较慢时, 过大的合成加速度会让无人机在下一避障指令到达前
        // 就冲入实体。对水平合成加速度与垂直加速度分别限幅到 yopoAccMax, 留出刹车
        // 与反应余量(等效最大倾角由 58° 降到 ~atan(8/9.81)≈39°)。
        const aMaxCmd = this.yopoAccMax;
        const aH = Math.hypot(aDesX, aDesZ);
        if (aH > aMaxCmd) {
            const s = aMaxCmd / aH;
            aDesX *= s;
            aDesZ *= s;
        }
        if (aDesY > aMaxCmd) aDesY = aMaxCmd;
        else if (aDesY < -aMaxCmd) aDesY = -aMaxCmd;

        // ---- 5. 投影到 body frame → 期望倾斜角 ----
        const aFwd   = aDesX * fwdX + aDesZ * fwdZ;
        const aRight = aDesX * rightX + aDesZ * rightZ;
        const targetPitch = clamp(-aFwd / G * RAD2DEG, -maxAngle, maxAngle);
        const targetRoll  = clamp(-aRight / G * RAD2DEG, -maxAngle, maxAngle);

        // ---- 6. 姿态环 (PD) → 期望角速率 ----
        const dec = this._decomposeOrientation();
        const angleErrPitch = targetPitch - dec.bodyPitchDeg;
        const angleErrRoll  = targetRoll  - dec.bodyRollDeg;
        const adAlpha = 1 - Math.exp(-15 * dt);
        const rawAngleDerrPitch = dt > 0 ? (angleErrPitch - this._sfPrevAngleErrPitch) / dt : 0;
        const rawAngleDerrRoll  = dt > 0 ? (angleErrRoll  - this._sfPrevAngleErrRoll)  / dt : 0;
        this._sfFiltAngleDerrPitch += (rawAngleDerrPitch - this._sfFiltAngleDerrPitch) * adAlpha;
        this._sfFiltAngleDerrRoll  += (rawAngleDerrRoll  - this._sfFiltAngleDerrRoll)  * adAlpha;
        this._sfPrevAngleErrPitch = angleErrPitch;
        this._sfPrevAngleErrRoll  = angleErrRoll;

        const rateTargetPitch = this.sfAngleKp * angleErrPitch + this.sfAngleKd * this._sfFiltAngleDerrPitch;
        const rateTargetRoll  = this.sfAngleKp * angleErrRoll  + this.sfAngleKd * this._sfFiltAngleDerrRoll;

        // ---- 7. 角速率环 (PID) ----
        const rateErrPitch = rateTargetPitch - this.pitchRate;
        const rateErrRoll  = rateTargetRoll  - this.rollRate;
        const rateIntMax = this._sfRateIntMax;
        this._sfRateIntPitch = clamp(this._sfRateIntPitch + rateErrPitch * dt, -rateIntMax, rateIntMax);
        this._sfRateIntRoll  = clamp(this._sfRateIntRoll  + rateErrRoll  * dt, -rateIntMax, rateIntMax);
        const rateDerrPitch = dt > 0 ? (rateErrPitch - this._sfPrevRateErrPitch) / dt : 0;
        const rateDerrRoll  = dt > 0 ? (rateErrRoll  - this._sfPrevRateErrRoll)  / dt : 0;
        this._sfPrevRateErrPitch = rateErrPitch;
        this._sfPrevRateErrRoll  = rateErrRoll;

        const angVelPitch = this.sfRateKp * rateErrPitch + this.sfRateKi * this._sfRateIntPitch + this.sfRateKd * rateDerrPitch;
        const angVelRoll  = this.sfRateKp * rateErrRoll  + this.sfRateKi * this._sfRateIntRoll  + this.sfRateKd * rateDerrRoll;

        const rateSmooth = 1 - Math.exp(-25 * dt);
        this.pitchRate += (angVelPitch - this.pitchRate) * rateSmooth;
        this.rollRate  += (angVelRoll  - this.rollRate)  * rateSmooth;
        this._applyBodyRotation(1, 0, 0, this.pitchRate * dt);
        this._applyBodyRotation(0, 0, 1, this.rollRate * dt);

        // ---- 8. 偏航：跟踪 YOPO 偏航指令 ----
        let targetYawRate = 0;
        if (yawActive) {
            // 摇杆控制偏航
            const droneYawMax = this.droneMaxYawRate;
            targetYawRate = input.yaw * droneYawMax;
        } else if (yopoNearGoalHold) {
            // 终点接管: 保持当前偏航，不旋转
            targetYawRate = 0;
        } else if (this.yopoCmdYaw !== null) {
            // 跟踪 YOPO 偏航指令（P 控制 + yaw_dot 前馈）
            // yopoCmdYaw 已由 calculate_yaw() 做速率限制（max 0.5π rad/s），
            // 且坐标系与 this.yaw 一致（ROS yaw = drone yaw），可直接相减。
            let cmdYawDeg = this.yopoCmdYaw * RAD2DEG;
            let yawErr = cmdYawDeg - this.yaw;
            while (yawErr > 180) yawErr -= 360;
            while (yawErr < -180) yawErr += 360;
            // yaw_dot 前馈（yopoCmdYawDot 由 server 返回，已转 deg/s）
            const yawDotFeed = (this.yopoCmdYawDot || 0) * RAD2DEG;
            targetYawRate = clamp(yawErr * 3.0 + yawDotFeed,
                                  -this.droneMaxYawRate, this.droneMaxYawRate);
        }
        const rateErrYaw = targetYawRate - this.yawRate;
        const angVelYaw = this.sfYawRateKp * rateErrYaw;
        this.yawRate += (angVelYaw - this.yawRate) * rateSmooth;
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);

        // ---- 9. 高度 → 推力 (倾斜补偿) ----
        let cmdGf = this.mass * (G + aDesY) / G;
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getY(_v3);
        const cosT = Math.max(0.1, _v3.y);
        cmdGf /= cosT;

        this.thrustOutput = clamp(cmdGf, 0, this.maxThrust);
        this.throttlePercent = this.maxThrust > 0
            ? Math.max(0, Math.min(1, this.thrustOutput / this.maxThrust))
            : 0;
    }

    // ---- 几何反应式避障 (势场法) 辅助 — 参考 git 3b92a03 ----

    /** 节流更新射线探测缓存: 控制环下每 yopoAvoidQueryMs 探测一次。 */
    _updateAvoidProbe() {
        const now = performance.now();
        const p = this._avoidProbe;
        // 探测节流速度自适应: 高速时保持密(60ms), 低速/静止时放宽(最长 400ms),
        // 平衡"飞得快时避障及时"与"帧率"(forceFresh 全量探测 GPU 开销随频率线性)。
        const spdHNow = Math.hypot(this.vx, this.vz);
        const queryMs = Math.max(this.yopoAvoidQueryMs,
            Math.min(400, 400 - spdHNow * 25));
        if (p && now - p.time < queryMs) return;
        // 位置变化很小也可复用, 减少 Cesium pickFromRay 开销
        if (p) {
            const moved = Math.hypot(this.x - p.x, this.z - p.z);
            const dy = Math.abs(this.y - p.y);
            if (moved < 0.4 && dy < 2.0 && now - p.time < 900) return;
        }
        this._avoidProbe = this._computeAvoidProbe();
    }

    /**
     * 探测水平 360° 环形障碍距离 + 地面/屋顶间隙 (世界系, 单位 m)。
     * 采用"扇形射线束": 每个主方向从机体左右偏移点发出多条平行射线, 取最近距离。
     * 单条中心射线在遇建筑物内凹部分(凹窗/门洞/凹墙面)时, 会穿过凹槽而漏检,
     * 两侧偏移射线能命中凹槽两边的墙沿, 从而把"凹进去的墙"当作障碍感知,
     * 避免无人机误以为前方畅通而撞上凹槽侧壁/后壁。
     */
    _computeAvoidProbe() {
        const provider = this._collisionProvider;
        const w = provider ? provider.world : null;
        if (!w || !w.ready || typeof w.pickLocalRay !== 'function') return null;

        const dirs = this.yopoAvoidRays;
        const R = this.yopoAvoidRange;
        const nFan = this.yopoAvoidFanRays;
        const half = this.yopoAvoidFanHalf;

        // 地面间隙先算: 用于 clamp 下探层起点, 避免贴地误检
        let groundGap = Number.POSITIVE_INFINITY;
        if (typeof w.sampleHeightAtLocal === 'function') {
            const gy = w.sampleHeightAtLocal(this.x, this.z, 0.6);
            if (Number.isFinite(gy)) groundGap = this.y - gy;
        }
        const groundY = Number.isFinite(groundGap) ? (this.y - groundGap) : -1e9;

        // 扇形射线束: 在某高度层沿主方向发 nFan 条平行射线, 取最近障碍距离。
        // 单条中心射线在遇建筑物内凹部分(凹窗/门洞/凹墙面)时, 会穿过凹槽而漏检,
        // 两侧偏移射线能命中凹槽两边的墙沿, 从而把"凹进去的墙"当作障碍感知。
        // 前向(关键避障方向)用完整扇形防凹槽漏检; 侧向/后向用单中心射线省开销。
        // nFan=1 时中心射线 origin=机体位置 → 命中 pickLocalRay 缓存桶 → 零 GPU 开销。
        // forceFresh=true: 跳过 pickLocalRay 缓存, 每次真实拾取。高速(12~15m/s)时
        // 缓存 150ms TTL 内无人机已移动 1.8~2.25m, 返回陈旧距离 → 刹车距离算错
        // (偏大) → "飞得快时避障来不及响应"撞墙。避障安全网必须用当前真实距离。
        // 探测本身已 80ms 节流 + 位置复用, forceFresh 只影响本函数内拾取, 不污染缓存。
        const pickF = (o, d, dist) => w.pickLocalRay(o, d, dist, true);
        const fanDist = (dir, yLevel, fanCount) => {
            const cnt = fanCount || 1;
            const dlen = Math.hypot(dir.x, dir.z) || 1e-6;
            const nx = dir.z / dlen, nz = -dir.x / dlen; // 主方向法向(水平面内垂直向量)
            let near = R;
            for (let k = 0; k < cnt; k++) {
                const off = (cnt === 1) ? 0 : (2 * k / (cnt - 1) - 1) * half;
                const o = { x: this.x + nx * off, y: yLevel, z: this.z + nz * off };
                const hit = pickF(o, dir, R);
                const hd = (hit && Number.isFinite(hit.distance) && hit.distance > 0.04)
                    ? hit.distance : R;
                if (hd < near) near = hd;
            }
            return near;
        };
        // 前进方向(水平): 速度优先, 否则机体前向 -Z
        let fwdHx = 0, fwdHz = -1;
        const spdHv = Math.hypot(this.vx, this.vz);
        if (spdHv > 0.3) { fwdHx = this.vx / spdHv; fwdHz = this.vz / spdHv; }
        // 前向方向的索引集合: 与前进方向点积 > 0.35 (即夹角 <70°) 的方向用扇形, 其余单射线
        const fwdFanIdx = [];
        for (let i = 0; i < dirs.length; i++) {
            const dot = dirs[i].x * fwdHx + dirs[i].z * fwdHz;
            if (dot > 0.35) fwdFanIdx.push(i);
        }
        const fanCountOf = (i) => (fwdFanIdx.indexOf(i) >= 0 ? this.yopoAvoidFanRays : 1);

        // 屋顶/悬挑下沿探测: 向上看是否存在遮挡, 防止钻入低矮檐口/凹槽顶。
        // 屋顶/悬挑下沿探测: 向上看遮挡, 防钻入低矮檐口/凹槽顶。环形布局无固定"前/后",
        // 故按与前进方向最对齐的 2 条射线做上探(静止/无前向时回退 dirs[0]/[1])。
        let ceilHit = false;
        if (this.yopoAvoidCeilRay) {
            const oUp = { x: this.x, y: this.y + this.yopoAvoidCeilLook, z: this.z };
            const cand = [];
            for (let i = 0; i < dirs.length; i++) {
                const dot = dirs[i].x * fwdHx + dirs[i].z * fwdHz;
                if (dot > 0.3) cand.push({ i, dot });
            }
            cand.sort((p, q) => q.dot - p.dot);
            const probeIdx = cand.length >= 2 ? [cand[0].i, cand[1].i] : [0, 1];
            for (const i of probeIdx) {
                const h = pickF(oUp, dirs[i], Math.min(R, 6));
                if (h && Number.isFinite(h.distance) && h.distance > 0.04) { ceilHit = true; break; }
            }
        }

        // 高度探测: mid(当前高度)全部环形方向; high/high2/low 只对"最对齐前进方向"
        // 的 2 条射线探测 — 竖直越障只关心前进方向能否上越/下钻, 减少射线数提帧率。
        const dists = new Array(dirs.length);
        const distsHigh = new Array(dirs.length);
        const distsHigh2 = new Array(dirs.length);
        const distsLow = new Array(dirs.length);
        const yHigh = this.y + this.yopoAvoidVStep;
        const yHigh2 = this.y + this.yopoAvoidVStep * 2;
        const yLow = Math.max(this.y - this.yopoAvoidVStep, groundY + 1.0);
        const lowOk = (yLow - groundY) > 1.5; // 下探层明显高于地面才算有效可钻
        // 选最对齐前进方向的 2 条射线做高层探测
        const vProbeIdx = [];
        for (let pass = 0; pass < 2; pass++) {
            let bi = -1, bd = -1;
            for (let i = 0; i < dirs.length; i++) {
                if (vProbeIdx.indexOf(i) >= 0) continue;
                const dot = dirs[i].x * fwdHx + dirs[i].z * fwdHz;
                if (dot > bd) { bd = dot; bi = i; }
            }
            if (bi >= 0) vProbeIdx.push(bi);
        }

        for (let i = 0; i < dirs.length; i++) {
            dists[i] = fanDist(dirs[i], this.y, fanCountOf(i));
            if (vProbeIdx.indexOf(i) >= 0) {
                distsHigh[i] = fanDist(dirs[i], yHigh, 1);
                distsHigh2[i] = fanDist(dirs[i], yHigh2, 1);
                distsLow[i] = lowOk ? fanDist(dirs[i], yLow, 1) : dists[i];
            } else {
                distsHigh[i] = dists[i];
                distsHigh2[i] = dists[i];
                distsLow[i] = dists[i];
            }
        }

        // 前下方探测: 沿前进方向向下俯视 -20°, 检测前方下方的建筑/地形抬升。
        // YOPO 3D 导航下垂直下降轨迹(爬升后回落/到目标点下降)会撞上前方下方障碍,
        // 水平射线(同高度)测不到; 此处专测前进方向 ±18° 两条下俯射线, 取最近值
        // 参与刹车 + 上推, 防止"下降撞地/撞楼下沿"。
        let fwdDownDist = R;
        if (spdHv > 0.3 || true) {
            const downRad = -20 * DEG2RAD;
            const cd = Math.cos(downRad), sd = Math.sin(downRad);
            const baseX = fwdHx * cd, baseY = sd, baseZ = fwdHz * cd;
            const nlen = Math.hypot(fwdHx, fwdHz) || 1e-6;
            const npx = fwdHz / nlen, npz = -fwdHx / nlen;
            const probeDn = (off) => {
                const o = { x: this.x + npx * off, y: this.y, z: this.z + npz * off };
                const h = pickF(o, { x: baseX, y: baseY, z: baseZ }, R);
                return (h && Number.isFinite(h.distance) && h.distance > 0.04) ? h.distance : R;
            };
            const d1 = probeDn(0);
            const d2 = probeDn(0.5);
            fwdDownDist = Math.min(d1, d2);
        }

        // 正上方/正下方竖直射线: 水平环@高层与前下俯视都测不到"同 x,z 正上/正下"的障碍
        // (如头顶天花板、脚下方形建筑顶)。防止上爬撞顶、垂直下降撞正下方障碍。
        let vUpDist = R, vDownDist = R;
        if (this.yopoAvoidVertRay) {
            const hUp = pickF({ x: this.x, y: this.y + 0.5, z: this.z }, { x: 0, y: 1, z: 0 }, this.yopoAvoidVertRange);
            vUpDist = (hUp && Number.isFinite(hUp.distance) && hUp.distance > 0.04) ? hUp.distance : R;
            const hDn = pickF({ x: this.x, y: this.y - 0.5, z: this.z }, { x: 0, y: -1, z: 0 }, this.yopoAvoidVertRange);
            vDownDist = (hDn && Number.isFinite(hDn.distance) && hDn.distance > 0.04) ? hDn.distance : R;
        }

        return {
            dists,
            distsHigh,
            distsHigh2,
            distsLow,
            lowOk,
            groundGap,
            ceilHit,
            fwdDownDist,
            vUpDist,
            vDownDist,
            highProbeIdx: vProbeIdx, // 已做高层探测的方向索引(竖直越障只能在这些方向上判断)
            x: this.x, y: this.y, z: this.z,
            time: performance.now(),
        };
    }

    /**
     * 势场避障速度: 返回 {repX, repZ, tanX, tanZ, brake, upPush, vRep}。
     *   - 排斥(rep): 障碍越近越大, 方向远离障碍簇;
     *   - 切向绕行(tan): 垂直于排斥方向, 取更靠近目标/期望方向的一侧,
     *     让无人机贴着障碍滑向目标, 避免势场局部极小;
     *   - 刹车(brake): 前进方向威胁越近越慢 (0..1);
     *   - upPush: 地面/屋顶净空不足时上推;
     *   - vRep: 竖直越障速度 (水平被强挡且某侧竖直空间畅通时爬升/下钻)。
     * 仅当障碍进入 yopoAvoidRange 内有非零输出; 路径通畅时 brake=1 且
     * rep/tan=0, 完全不影响导航到目标点的最终目标。
     */
    _avoidanceVelocity(velTargetX, velTargetZ) {
        const p = this._avoidProbe;
        if (!p) return null;
        const R = this.yopoAvoidRange;
        const dirs = this.yopoAvoidRays;
        const dists = p.dists;
        if (!dists || dists.length !== dirs.length) return null;
        // 局部 clamp: 文件内多个方法各自定义局部 clamp(不同作用域),
        // 此处也需定义, 否则下方刹车软逻辑的 clamp 调用会 ReferenceError。
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

        let repX = 0, repZ = 0;
        let dMin = R;        // 整体最近障碍 (排斥/切向强度)
        let dAhead = R;      // 前进方向威胁(含竖直威胁, 用于刹车/上推)
        let dAheadH = R;     // 仅水平环射线的最前进距离(不含 fwdDownDist/groundGap/ceilHit),
                             // 专用于竖直越障判定, 避免竖直威胁把 dAhead 拉小误触发上越/下钻。
        let openDirX = 0, openDirZ = 0, openMax = -1;

        const des = Math.hypot(velTargetX, velTargetZ);
        let udx = 0, udz = 0;
        if (des > 0.3) { udx = velTargetX / des; udz = velTargetZ / des; }

        for (let i = 0; i < dirs.length; i++) {
            const d = dists[i];
            if (!Number.isFinite(d) || d <= 0) continue;
            if (d < dMin) dMin = d;
            if (d > openMax) { openMax = d; openDirX = dirs[i].x; openDirZ = dirs[i].z; }
            if (d < this.yopoAvoidRepRange) {
                const w = 1 - d / this.yopoAvoidRepRange;
                repX -= dirs[i].x * w;
                repZ -= dirs[i].z * w;
            }
            // 前进方向威胁: 期望速度方向附近的障碍计入刹车
            const dot = dirs[i].x * udx + dirs[i].z * udz;
            if (des > 0.3 ? (dot > 0.5 && d < dAhead) : (d < dAhead)) dAhead = d;
            if (des > 0.3 ? (dot > 0.5 && d < dAheadH) : (d < dAheadH)) dAheadH = d;
        }

        // 地面/屋顶间隙不足 → 上推 + 参与刹车
        let upPush = 0;
        if (Number.isFinite(p.groundGap) && p.groundGap < this.yopoMinAlt) {
            upPush = (this.yopoMinAlt - p.groundGap) * this.yopoAvoidGain * 0.5;
            if (p.groundGap < dAhead) dAhead = p.groundGap;
        }
        // 前下方障碍(下降撞地/撞楼下沿): 前下方净空 < minAlt → 上推 + 刹车。
        // 与正下方 groundGap 不同, 这里专门挡"前进方向往下看"的威胁(如前方低矮建筑/
        // 地形抬升/目标点下降路径上的遮挡), 防止下降轨迹撞上去。
        if (Number.isFinite(p.fwdDownDist) && p.fwdDownDist < this.yopoMinAlt) {
            const push = (this.yopoMinAlt - p.fwdDownDist) * this.yopoAvoidGain * 0.45;
            if (push > upPush) upPush = push;
            if (p.fwdDownDist < dAhead) dAhead = p.fwdDownDist;
        }

        // ---- 垂直下降运动学刹车 ----
        // 水平方向已有 v_safe=√(2ad) 刹车, 但垂直方向只有 upPush(净空<minAlt 才触发),
        // 高速下降(网络轨迹下降/终点下降)时 2.5m 净空根本刹不住 → 撞下方障碍(用户反馈)。
        // 此处按正下方与前下方的**最小**净空, 限制最大下降速度:
        //   vSafeDown = √(2·a·(gap - standoff))
        // 使无人机在任意净空都能物理上刹停, 不再"全速俯冲撞地"。
        // upPush 较弱时由 vSafeDown 直接钳制 velTargetY(见 _controlYOPO 调用处)。
        let vSafeDown = null;
        const downGap = Math.min(
            Number.isFinite(p.groundGap) ? p.groundGap : R,
            Number.isFinite(p.fwdDownDist) ? p.fwdDownDist : R,
            Number.isFinite(p.vDownDist) ? p.vDownDist : R
        );
        if (Number.isFinite(downGap) && downGap < this.yopoAvoidRange) {
            const aD = this.yopoAvoidDecel;
            const sd = this.yopoAvoidStop;
            if (downGap <= sd) {
                vSafeDown = 0;          // 净空已不足 → 完全禁止下降
            } else {
                vSafeDown = Math.sqrt(2 * aD * (downGap - sd));
            }
            // 净空不足时同时加强上推(与 upPush 取较大)
            if (downGap < this.yopoMinAlt) {
                const push = (this.yopoMinAlt - downGap) * this.yopoAvoidGain * 0.6;
                if (push > upPush) upPush = push;
            }
        }

        // 正上方净空不足时限制上推速度(对称于 vSafeDown 的下降刹车): 上推速度不超过
        // 能在正上方障碍前刹停的值, 防止上爬/上推撞顶(用户反馈正上方易撞)。
        if (Number.isFinite(p.vUpDist)) {
            const aU = this.yopoAvoidDecel, su = this.yopoAvoidStop;
            if (p.vUpDist <= su) { if (upPush > 0) upPush = 0; }
            else {
                const vSafeUp = Math.sqrt(2 * aU * (p.vUpDist - su));
                if (upPush > vSafeUp) upPush = vSafeUp;
            }
        }

        // 屋顶/悬挑下沿遮挡: 表明头顶上方存在内凹顶/檐口, 先别急着前进钻入。
        // 通过降低前进速度留出反应时间, 同时加一点上/后退趋势, 避免撞上凹槽顶。
        if (p.ceilHit) {
            const eff = Math.max(2.0, R * 0.35); // 视为中近距离威胁
            if (eff < dAhead) dAhead = eff;
        }

        // 近障刹车: 双层渐进减速, 保证灵敏度 + 物理刹停。
        //   1) 运动学硬刹车: v_safe = sqrt(2*a*(d - standoff)), 保证在净空内一定刹停
        //      (无论多快, 物理上不可能撞上墙)。
        //   2) 渐进软刹车: 在 repRange 内按距离平滑降速(线性缩放), 让无人机"越近越慢",
        //      提前减速而非到 8m 才突然刹。两者取更保守(更小)的 brake。
        let brake = 1;
        const aDecel = this.yopoAvoidDecel;
        const standoff = this.yopoAvoidStop;
        const spdFwd = Math.hypot(velTargetX, velTargetZ);
        if (dAhead <= standoff) {
            brake = 0;  // 已进入安全净空 → 完全停止前进
        } else if (dAhead < R) {
            const vSafe = Math.sqrt(2 * aDecel * (dAhead - standoff));
            const kinBrake = spdFwd > 1e-3 ? Math.min(1, vSafe / spdFwd) : 0;
            // 渐进软刹车: repRange→满速, standoff*2→0。让 12m/s 在 18m 处就开始减速,
            // 越近越慢, 大幅提前避障响应(灵敏度), 同时软刹车与运动学刹车取小者。
            const soft = clamp(
                (dAhead - standoff * 2) / (this.yopoAvoidRepRange - standoff * 2),
                0, 1
            );
            brake = Math.min(kinBrake, soft);
        }

        const repMag = Math.hypot(repX, repZ);
        // 排斥强度限幅
        if (repMag > 1e-6) {
            const s = Math.min(1, this.yopoAvoidRepGain / repMag);
            repX *= s; repZ *= s;
        }

        // 切向绕行: 用"最近障碍方向 dMin"算确定性切向(绕开最近障碍、朝目标侧), 不挑开口 /
        // 不 fallback 最空方向 —— 避免绕到侧面、朝目标被挡时误选"最空=来路"方向折返
        // ("绕开又回去")。叠加方向滞后记忆: 与上一帧 tan 夹角 >120° 且上一帧方向此刻仍通畅
        // 时保持上一帧, 防止过障碍正中时合力翻转导致来回绕。
        let tanX = 0, tanZ = 0;
        // 仅当正前方较近处确有障碍(dAhead < repRange*0.5 ≈9m)才施加切向绕行。
        // 通道/狭窄空间里障碍多在两侧, 正前(目标方向=通道纵深)畅通, 若仍施加切向会把
        // 无人机推向侧壁、卡在通道口进不去; 此条件下正前畅通→不绕行, 由 rep(非终点段)
        // 推离侧墙保持居中 / 终点段由 PD 收敛中线, 无人机得以直行入通道。正前确有近障
        // 时才正常绕行。
        if (dMin < R && dAhead < this.yopoAvoidRepRange * 0.5) {
            // 找最近障碍方向(dMin 对应的射线方向)
            let mi = -1;
            for (let i = 0; i < dirs.length; i++) {
                const d = dists[i];
                if (!Number.isFinite(d) || d <= 0) continue;
                if (mi < 0 || d < dists[mi]) mi = i;
            }
            if (mi >= 0) {
                const ox = dirs[mi].x, oz = dirs[mi].z;   // 指向最近障碍
                // 两个切向候选(垂直于障碍方向): 选朝目标(期望速度)投影更大的一侧绕行;
                // 目标在障碍正后方时两候选都≈0, 任取一侧继续绕行(不再折返来路)。
                const tx1 = -oz, tz1 = ox;
                const tx2 = oz, tz2 = -ox;
                const c1 = tx1 * udx + tz1 * udz;
                const c2 = tx2 * udx + tz2 * udz;
                let fx, fz;
                if (c1 >= c2) { fx = tx1; fz = tz1; } else { fx = tx2; fz = tz2; }
                const t = this.yopoAvoidTanGain * (1 - dMin / this.yopoAvoidRepRange);
                fx *= t; fz *= t;
                // 方向滞后记忆: 与上一帧 tan 夹角 >120° 且上一帧方向此刻仍通畅时, 保持上一帧
                const lt = this._avoidLastTan || null;
                if (lt) {
                    const lm = Math.hypot(lt.x, lt.z), nm = Math.hypot(fx, fz);
                    if (lm > 1e-3 && nm > 1e-3) {
                        const cos = (fx * lt.x + fz * lt.z) / (nm * lm);
                        let lastOk = false;
                        for (let i = 0; i < dirs.length; i++) {
                            const lnx = lt.x / lm, lnz = lt.z / lm; // 上一帧方向(归一化)
                            if (Math.abs(dirs[i].x - lnx) < 0.01 && Math.abs(dirs[i].z - lnz) < 0.01) {
                                if (dists[i] > this.yopoAvoidStop + 2.0) lastOk = true;
                                break;
                            }
                        }
                        if (cos < -0.5 && lastOk) { fx = lt.x * nm / lm; fz = lt.z * nm / lm; }
                    }
                }
                tanX = fx; tanZ = fz;
                this._avoidLastTan = { x: tanX, z: tanZ };
            }
        }

        // ---- 出口识别: 朝目标点的直线走廊畅通 → 直飞, 不被绕行/避障扰动 ----
        // 用"飞行走廊"判定通畅度: 走廊 = 以"机体→导航目标"方位为中心线、半宽 pathHalfWidth 的
        // 带状区域; 仅落在走廊内(实测到该中心线的垂直偏移 < 半宽)的障碍才算"挡在路上"。
        // 走廊外的侧旁障碍(即便很近)直接忽略——直飞即可安全通过。竖直威胁
        // (fwdDownDist/groundGap/ceilHit)一律不参与, 由 upPush/vSafeDown 单独处理垂直安全。
        // 判定轴: 目标方位(机体→导航目标)为主, 但 goalClear 同时满足"命令速度方向走廊也畅通"
        // (见下双走廊判定)。单用目标方位会漏掉"网络把无人机指向建筑、但目标线暂清"→仍全速直冲
        // (即"规划往建筑物上撞"); 仅用命令速度方向(此前版本)则转弯/绕行瞬间 velTarget 偏入侧旁
        // 建筑、且其横向偏移其实很大→会被误判"被挡"→ goalClear 失效→持续切向绕行("到目标畅通却
        // 还绕行")。故两者都用"走廊(横向偏移<半宽)"度量而非锥形最小距离: 真正在路径上的近障才
        // 算挡路, 侧旁/转弯偏指的大偏移障碍忽略 → 既不盲冲建筑, 也不在畅通时乱绕。无导航目标时
        // 命令速度方向回退到目标方位。
        let gx, gz;
        if (this.yopoNavTarget) {
            const tdx = this.yopoNavTarget.x - this.x;
            const tdz = this.yopoNavTarget.z - this.z;
            const tl = Math.hypot(tdx, tdz);
            if (tl > 0.5) { gx = tdx / tl; gz = tdz / tl; }
            else { gx = udx; gz = udz; }
        } else {
            gx = udx; gz = udz;
        }
        let goalClear = false;
        if (des > 0.3 || this.yopoNavTarget) {
            const pathHalfWidth = 2.5;                      // m, 飞行走廊半宽(机体半径+余量)
            const clearThresh = this.yopoAvoidRepRange;     // 走廊内无近障(>作用距离)才算畅通
            // 双走廊判定:
            //   dPath — 沿"机体→目标"方位的走廊净空(到目标的路畅通否)
            //   dCmd  — 沿"命令速度方向(实际前进方向)"的走廊净空(网络/轨迹正指向的路面畅通否)
            // 两者都畅通才解除避障直飞。仅用目标走廊会漏掉"网络把无人机指向建筑、但目标线暂清"
            // 的情况→仍全速直冲(即'规划往建筑物上撞'); 两者皆查可杜绝。用走廊(横向偏移<半宽)
            // 而非 dAhead 锥形最小, 避免转弯瞬间速度偏入侧旁建筑(横向偏移大)被误判挡路→持续
            // 绕行(此前'到目标畅通却还绕行'的根因)。
            let dPath = R, dCmd = R;
            const cx = udx, cz = udz, cMag = Math.hypot(cx, cz);
            const cxn = cMag > 0.3 ? cx / cMag : gx;        // 无有效前进速度时回退目标方位
            const czn = cMag > 0.3 ? cz / cMag : gz;
            for (let i = 0; i < dirs.length; i++) {
                const dd = dists[i];
                if (!Number.isFinite(dd) || dd <= 0) continue;
                const dotT = dirs[i].x * gx + dirs[i].z * gz;
                if (dotT > 0) {
                    const latT = dd * Math.sqrt(Math.max(0, 1 - dotT * dotT));
                    if (latT < pathHalfWidth && dd < dPath) dPath = dd;
                }
                const dotC = dirs[i].x * cxn + dirs[i].z * czn;
                if (dotC > 0) {
                    const latC = dd * Math.sqrt(Math.max(0, 1 - dotC * dotC));
                    if (latC < pathHalfWidth && dd < dCmd) dCmd = dd;
                }
            }
            if (dPath > clearThresh && dCmd > clearThresh) goalClear = true;
        }

        // ---- 竖直越障 (A) ----
        // 水平前进被强挡(dAhead 近)且某侧竖直空间畅通时, 主动爬升/下降越过中间障碍。
        // 沿最对齐前进方向的射线, 比较其 mid/high/high2/low 多层距离: 任一层上方畅通→上越,
        // 否则下方畅通→下钻。探测随高度动态刷新, 无人机持续爬升直到越过障碍顶。
        let vRep = 0;
        const blockDist = this.yopoAvoidStop + this.yopoAvoidVBlock;
        // 竖直越障只在"去往目标的水平走廊确实被挡"时才触发: 用 !goalClear(前进走廊内有障碍)且
        // 水平前进近距(dAheadH<blockDist)双重判定。dAhead 含 fwdDownDist/groundGap/ceilHit 等竖直
        // 威胁(用于刹车/上推), 若直接用会把 dAhead 拉小 → "往目标方向畅通却误上越/下钻"。改用
        // 纯水平的 dAheadH + 走廊判定后, 仅在正前方确有水平障碍(而非下方净空不足)时才爬升/下钻。
        // 终点接管区(距目标 <yopoFinalApproachDist 或已到达)内彻底禁用竖直越障: 此时应直接 PD
        // 收敛到目标点, 爬升会偏离目标、且"明明畅通却飞起来"(双走廊/goalClear 在终点微调时可能因
        // 速度方向偏指侧旁建筑而失效)。竖直安全(upPush/vSafeDown)仍保留防撞地/顶。
        const nt = this.yopoNavTarget;
        const nearGoal = nt && (this.yopoArrived ||
            Math.hypot(nt.x - this.x, nt.z - this.z) < this.yopoFinalApproachDist);
        if (!nearGoal && !goalClear && dAheadH < blockDist && des > 0.3 &&
            p.distsHigh && p.distsHigh2 && p.distsLow) {
            // 只从已做高层探测的 2 条方向(vProbeIdx 记录在 probe 的 highProbeIdx 中)里
            // 选最对齐前进方向的射线: 保证 dH/dH2/dL 是真实高层距离而非 mid 值。
            let bi = -1, bdot = 0.5;
            const hiIdx = p.highProbeIdx || null;
            for (let i = 0; i < dirs.length; i++) {
                if (hiIdx && hiIdx.indexOf(i) < 0) continue; // 仅限已测高层方向
                const dot = dirs[i].x * udx + dirs[i].z * udz;
                if (dot > bdot) { bdot = dot; bi = i; }
            }
            if (bi >= 0) {
                const dH = p.distsHigh[bi];
                const dH2 = p.distsHigh2[bi];
                const dL = p.distsLow[bi];
                const clearD = R * this.yopoAvoidVClear; // 该层距离 > 此值视为畅通, 可飞越
                const upClear = ((dH > clearD) || (dH2 > clearD)) && (p.vUpDist > clearD); // 上方水平层与正上方都畅通
                // 下钻更保守: 仅 low 层畅通还不够(只测 2 条方向), 必须正下方与前下方
                // 净空都充足才允许下降, 否则下钻会撞到未探测到的下方障碍(用户反馈)。
                const downClear = (p.lowOk === true) && (dL > clearD) &&
                    Number.isFinite(p.groundGap) && p.groundGap > this.yopoMinAlt &&
                    Number.isFinite(p.fwdDownDist) && p.fwdDownDist > this.yopoMinAlt &&
                    Number.isFinite(p.vDownDist) && p.vDownDist > this.yopoMinAlt;
                const e = this.yopoAvoidGain * this.yopoAvoidVClimbScale;
                if (upClear && downClear) vRep = e;       // 两侧皆可 → 优先爬升(更安全)
                else if (upClear) vRep = e;               // 上越
                else if (downClear) vRep = -e;            // 下钻(净空确认充足)
            }
        }

        // 势场只负责"停住不撞", 不持续推离: 用"最近障碍距离 dMin"(任意方向)调制, 而非朝
        // 目标方向的 dAhead——绕到障碍侧面时 dAhead 虽小(目标在障碍后), 但 dMin 仍近, 故
        // 推离/绕行保持全量, 坚定把无人机带过障碍尽头, 不会中途放手被目标引力拉回(解决
        // "绕开又回去"); 仅真正贴死障碍(dMin≤standoff)时归零(停住后不反向推), 再配合 rep
        // 随距离 w 衰减, 远离后自然减弱, 不一路推过头。
        const repHold = clamp(dMin / standoff, 0, 1);
        repX *= repHold; repZ *= repHold;
        tanX *= repHold; tanZ *= repHold;

        // 出口畅通时彻底解除水平排斥/切向/刹车, 直飞目标:
        // 这是"明明没障碍却总被推开"的根治点——只要朝目标方向的水平通道净空充足(dg>clearThresh
        // 且邻近射线/前锥形无障碍), 就全速直飞, 不叠加任何 rep/tan/brake。
        // 注意: 仅当确有前进意图(des>0.3)才解除——悬停时仍保留 rep/tan 维持与障碍的安全间距,
        // 不被误判为"畅通"而漂向墙面。竖直安全(upPush/vSafeDown)始终生效, 与水平直飞互不干扰。
        if (goalClear && des > 0.3) {
            repX = 0; repZ = 0;          // 水平排斥彻底归零(不再留 15% 残余推力)
            tanX = 0; tanZ = 0;          // 切向完全去掉(避免绕回起点)
            brake = 1.0;                 // 出口畅通即全速, 不被竖直威胁误减速
            vRep = 0;                    // 竖直越障也解除: 走廊畅通不爬升/下钻
        }

        // ---- 竖直障碍水平绕行 (B) ----
        // 正下方有"建筑/结构"(vDownDist 小且明显高于地面, 非贴地地形)或正上方有障碍(vUpDist 小)时,
        // 不再施加下/上运动去"钻过", 改为保持高度、用水平绕行(vGo)平滑离开障碍正下/正上方足迹,
        // 避免"想下降→被射线/碰撞推开→又想下降"的来回抖动。竖直越障(vRep)针对"正前方水平被挡、
        // 上下有缝"; 此处针对"正下/正上方挡路"——唯一安全路径是水平绕开。nearGoal 内不启用(交 PD 收敛)。
        let vGoX = 0, vGoZ = 0;
        const vGoThresh = this.yopoAvoidStop + 3.0;   // ~4.1m: 脚下/头顶近障视为挡路
        const gg = Number.isFinite(p.groundGap) ? p.groundGap : R;
        // 正下方是"结构而非地形": 直下命中远高于地面 → 是建筑/悬挑而非贴地。贴地地形(无建筑)
        // 仍走 upPush/vSafeDown 正常处理, 不会被此处拦截而禁止下降(否则低空飞行无法降落)。
        const structBelow = Number.isFinite(p.vDownDist) && p.vDownDist < vGoThresh &&
            (gg - p.vDownDist > 1.5);
        const aboveBlocked = Number.isFinite(p.vUpDist) && p.vUpDist < vGoThresh;
        if ((structBelow || aboveBlocked) && !nearGoal) {
            // 选水平最空方向离开障碍足迹: 优先"前向半球最空", 否则用全局最空(openDir),
            // 保证绕行同时尽量向目标前进, 不折返来路。
            let ox = openDirX, oz = openDirZ;
            if (ox * gx + oz * gz < 0.3) {
                let best = -1, bestD = 0.3;
                for (let i = 0; i < dirs.length; i++) {
                    const d = dists[i];
                    if (!Number.isFinite(d) || d <= 0) continue;
                    if (dirs[i].x * gx + dirs[i].z * gz <= 0.3) continue;
                    if (d > bestD) { bestD = d; best = i; }
                }
                if (best >= 0) { ox = dirs[best].x; oz = dirs[best].z; }
            }
            const om = Math.hypot(ox, oz) || 1;
            ox /= om; oz /= om;
            const closeness = structBelow
                ? clamp(p.vDownDist / vGoThresh, 0, 1)
                : clamp(p.vUpDist / vGoThresh, 0, 1);
            const strength = this.yopoAvoidTanGain * (0.3 + 0.2 * (1 - closeness));
            vGoX = ox * strength;
            vGoZ = oz * strength;
        }

        return { repX, repZ, tanX, tanZ, brake, upPush, vRep, vSafeDown, vGoX, vGoZ };
    }

    // ---- Collision ----

    _handleCollisions(collisionProvider, previousPosition = null, dt = 0.016) {
        this.isColliding = false;
        this.collisionIntensity = 0;

        if (collisionProvider && typeof collisionProvider.queryCollisionResponse === 'function') {
            let anyCollision = false;
            let strongest = 0;

            for (let i = 0; i < 3; i++) {
                const collision = collisionProvider.queryCollisionResponse(this.x, this.y, this.z, this.collisionRadius, {
                    previous: i === 0 ? previousPosition : null,
                    velocity: { x: this.vx, y: this.vy, z: this.vz },
                    dt,
                });

                if (!collision || collision.penetration <= 0) break;

                anyCollision = true;
                strongest = Math.max(strongest, collision.penetration);

                const pushDist = collision.penetration + 0.04;
                this.x += collision.normal.x * pushDist;
                this.y += collision.normal.y * pushDist;
                this.z += collision.normal.z * pushDist;

                const vDotN = this.vx * collision.normal.x +
                              this.vy * collision.normal.y +
                              this.vz * collision.normal.z;
                if (vDotN < 0) {
                    const bounce = collision.source === 'swept' || collision.source === 'ray'
                        ? Math.max(this.bounceDamping, 0.55)
                        : this.bounceDamping;
                    this.vx -= collision.normal.x * vDotN * (1 + bounce);
                    this.vy -= collision.normal.y * vDotN * (1 + bounce);
                    this.vz -= collision.normal.z * vDotN * (1 + bounce);
                }

                const separationSpeed = Math.min(8, collision.penetration * 24);
                this.vx += collision.normal.x * separationSpeed;
                this.vy += collision.normal.y * separationSpeed;
                this.vz += collision.normal.z * separationSpeed;

                this.vx *= 0.65;
                this.vy *= 0.65;
                this.vz *= 0.65;
            }

            if (anyCollision) {
                this.isColliding = true;
                this.collisionIntensity = Math.min(1, strongest / Math.max(this.collisionRadius, 0.05));
                if (this.flightMode === 'drone') {
                    this._targetX = this.x;
                    this._targetY = this.y;
                    this._targetZ = this.z;
                }
            }
        }

    }
}
