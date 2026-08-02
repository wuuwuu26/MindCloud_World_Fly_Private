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
        this.droneSize = 0.3;

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
        this.sfRateKp = 0.5; this.sfRateKi = 0.0; this.sfRateKd = 0.0;
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
        this.yopoCmdPos = null;           // {x, y, z} 当前指令位置
        this.yopoCmdVel = null;           // {x, y, z} 当前指令速度
        this.yopoCmdAcc = null;           // {x, y, z} 当前指令加速度
        this.yopoCmdTime = 0;            // performance.now() 时间戳，追踪 cmd 新鲜度
        this.yopoCmdYaw = 0;              // 当前指令偏航 (rad, ROS/drone yaw 约定)
        this.yopoCmdYawDot = 0;           // 当前指令偏航角速率 (rad/s)
        this.yopoInferenceCount = 0;      // 推理计数
        this.yopoServerUrl = 'http://localhost:5689'; // YOPO 服务器地址

        this.collisionRadius = 0.3;
        this.bounceDamping   = 0.3;

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
        let velTargetX, velTargetZ;
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
        let velTargetY;
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
     * YOPO 导航控制律 — 跟踪 YOPO 后端返回的位置/速度指令。
     *
     * 复用 SimpleFlight 的级联 PID 框架（位置→速度→姿态→角速率），
     * 将 YOPO 神经网络输出的位置/速度指令作为前馈输入到底层控制器。
     * 支持摇杆抢占：推动摇杆时临时切换到人工控制，松杆后恢复导航。
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

        // ---- 0b. 到达目标 → 在目标点位置悬停（高增益位置环）----
        // 到达后不再跟随YOPO轨迹（网络在目标附近规划不稳定），
        // 改为直接位置悬停到目标点。
        const yopoArrivedHold = this.yopoArrived && this.yopoNavTarget && !stickActive;

        // ---- 1. Diagnostic logging ----
        if (this.yopoInferenceCount < 5 || this.yopoInferenceCount % 120 === 0) {
            const hasCmd = this.yopoCmdPos ? 'YES' : 'NO';
            const cmdStr = this.yopoCmdPos
                ? `cmd=(${this.yopoCmdPos.x.toFixed(1)},${this.yopoCmdPos.y.toFixed(1)},${this.yopoCmdPos.z.toFixed(1)})`
                : '';
            console.log(`_controlYOPO #${this.yopoInferenceCount}: armed=${input.armed} hasCmd=${hasCmd} ${cmdStr} ` +
                `pos=(${this.x.toFixed(1)},${this.y.toFixed(1)},${this.z.toFixed(1)})`);
        }

        // ---- 2. Body-frame forward/right ----
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);
        let fwdX = -_v3.x, fwdZ = -_v3.z;
        _mat4.getX(_v3);
        let rightX = _v3.x, rightZ = _v3.z;
        const fwdLen = Math.sqrt(fwdX * fwdX + fwdZ * fwdZ);
        if (fwdLen > 1e-4) { fwdX /= fwdLen; fwdZ /= fwdLen; }
        const rightLen = Math.sqrt(rightX * rightX + rightZ * rightZ);
        if (rightLen > 1e-4) { rightX /= rightLen; rightZ /= rightLen; }

        // YOPO 导航最大水平速度。多项式 vel_max=6, 但终点放大+垂直飞越需要更高速度。
        const yopoMaxSpd = 15.0;
        const maxSpd = stickActive ? this.droneMaxSpeed : yopoMaxSpd;
        const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };

        // ---- 3. 确定速度目标 ----
        let velTargetX, velTargetZ, velTargetY;
        let pilotCmdX = 0, pilotCmdZ = 0;
        let useAccFeedforward = false;

        if (yopoArrivedHold) {
            // 到达目标 → PD 悬停收敛到目标点（位置 P + 速度阻尼 D）
            // 对齐 SO3 的 kx*posErr + kv*(0-vel) 行为（des_vel=0 时自然减速）
            // 纯 P 会在到达 goal 时速度不为 0 → 过冲 → 拉回 → 晃动
            const gErrX = this.yopoNavTarget.x - this.x;
            const gErrZ = this.yopoNavTarget.z - this.z;
            const gErrY = this.yopoNavTarget.y - this.y;
            const holdKp = 1.5, holdAltKp = 2.5, holdKd = 1.5, holdMaxV = 2.0;
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
            const ffY = (this.yopoCmdVel ? (this.yopoCmdVel.y || 0) : 0) * ffDecay;

            const yopoPosKp = 0.7;   // 位置环降低(用户要求): 减弱"拉回旧指令位置"的趋势, 由速度/加速度前馈主导跟踪
            const yopoAltKp = 1.2;   // 高度环同步降低, 垂直运动更平滑
            velTargetX = yopoPosKp * posErrX + ffX;
            velTargetZ = yopoPosKp * posErrZ + ffZ;
            velTargetY = yopoAltKp * posErrY + ffY;
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

        // 速度目标限幅
        const velTargetH = Math.sqrt(velTargetX * velTargetX + velTargetZ * velTargetZ);
        if (velTargetH > maxSpd) {
            const s = maxSpd / velTargetH;
            velTargetX *= s; velTargetZ *= s;
        }
        velTargetY = clamp(velTargetY, -this.droneMaxVSpeed, this.droneMaxVSpeed);
        this.targetGroundSpeed = Math.sqrt(velTargetX * velTargetX + velTargetZ * velTargetZ);

        // Diagnostic: log velocity targets when navigating
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
        const velKp = useAccFeedforward ? 1.5 : this.sfVelKp;
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
            let ffScale = 1.0;
            if (cmdAgeMs > 200) {
                ffScale = 0.0;
            } else if (cmdAgeMs > 80) {
                ffScale = 1.0 - (cmdAgeMs - 80) / 120;
            }
            aDesX += this.yopoCmdAcc.x * ffScale;
            aDesY += (this.yopoCmdAcc.y || 0) * ffScale;
            aDesZ += this.yopoCmdAcc.z * ffScale;
        }

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
        } else if (yopoArrivedHold) {
            // 到达后保持当前偏航，不旋转
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
