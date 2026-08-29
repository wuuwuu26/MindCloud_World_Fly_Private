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
        this.droneMaxVSpeed  = 14.0;          // Hard vertical speed cap (m/s): 12 -> 14, so the vertical
                                              // obstacle-clearing climb (vRep=19) really reaches ~14
                                              // (the old 12 clamped the climb and made clearing sluggish);
                                              // also improves altitude tracking response. Supported by yopoAccMax=11.
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

        // ---- SimpleFlight state (cascaded PID integral/derivative memory) ----
        this._sfVelIntX = 0; this._sfVelIntY = 0; this._sfVelIntZ = 0;
        this._sfPrevVelErrX = 0; this._sfPrevVelErrY = 0; this._sfPrevVelErrZ = 0;
        this._sfFiltVelDerrX = 0; this._sfFiltVelDerrY = 0; this._sfFiltVelDerrZ = 0;
        this._sfRateIntPitch = 0; this._sfRateIntRoll = 0; this._sfRateIntYaw = 0;
        this._sfPrevRateErrPitch = 0; this._sfPrevRateErrRoll = 0; this._sfPrevRateErrYaw = 0;
        this._sfPrevAngleErrPitch = 0; this._sfPrevAngleErrRoll = 0;
        this._sfFiltAngleDerrPitch = 0; this._sfFiltAngleDerrRoll = 0;
        this._sfPrevAltErr = 0;
        this._sfFiltAltDerr = 0;
        // SimpleFlight gains (AirSim Params.hpp defaults)
        this.sfPosKp = 1.0;
        this.sfVelKp = 5.0; this.sfVelKi = 0.0; this.sfVelKd = 1.0;
        this.sfAngleKp = 4.5; this.sfAngleKd = 0.1;
        this.sfRateKp = 0.8; this.sfRateKi = 0.0; this.sfRateKd = 0.0;
        this.sfAltKp = 2.0; this.sfAltKd = 0.5;
        this.sfYawRateKp = 1.0;
        this._sfVelIntMax = 15.0;
        this._sfRateIntMax = 50.0;

        // ---- YOPO navigation state ----
        this.yopoNavTarget = null;         // {x, y, z} goal point
        this.yopoNavActive = false;       // whether navigation is active
        this.yopoArrived = false;         // whether the goal has been reached
        this.yopoDistToGoal = 0;          // distance to the goal
        this.arriveThreshold = 2.0;       // arrival radius (m), matches test_yopo_ros.py L132
        // Final-approach takeover distance: inside goal_length (2*radio_range = 10 m) the
        // network's goal observation is squeezed by normalisation, and the lattice only holds
        // cruise-type trajectories, so near the goal argmin(score) keeps picking overshooting /
        // turn-back trajectories -> velocity/position oscillate and the goal is never reached.
        // Within 12 m of the goal we stop following the YOPO trajectory and instead run a PD
        // convergence straight onto the goal point (position P + velocity damping D +
        // distance-based speed limiting).
        this.yopoFinalApproachDist = 12.0; // Final-approach takeover within 12 m of the goal (m)
        this.yopoArriveHoldM = 3.5;        // Client-side arrival lock distance threshold (m)
        this.yopoArriveHoldV = 1.0;        // Client-side arrival lock speed threshold (m/s)
        this.yopoCmdPos = null;           // {x, y, z} current commanded position
        this.yopoCmdVel = null;           // {x, y, z} current commanded velocity
        this.yopoCmdAcc = null;           // {x, y, z} current commanded acceleration
        this.yopoCmdTime = 0;             // performance.now() timestamp, tracks command freshness
        this.yopoCmdYaw = 0;              // current commanded yaw (rad, ROS/drone yaw convention)
        this.yopoCmdYawDot = 0;           // current commanded yaw rate (rad/s)
        this.yopoDepthUnavailable = false; // DA360 depth unavailable -> hover and wait (no fallback to raycasting)
        this.yopoInferenceCount = 0;      // inference counter
        this.yopoServerUrl = 'http://localhost:5689'; // YOPO server address

        // Rely only on the YOPO_360 network's own learning-based avoidance (learned during
        // training via safety_loss, wc=8); do not stack any geometric reactive avoidance /
        // potential-field method on top. The values below only keep the safety and scaling
        // parameters needed to interpret YOPO commands, plus a passive ground safety net based
        // on pure terrain sampling (see the hard ground floor inside _controlYOPO).
        this.yopoCrashFloor = 1.0;    // Hard ground safety floor (m): below this clearance a climb is forced, preventing blind descent into the ground
        // Desired acceleration safety ceiling (m/s^2): clamps the actually reachable combined
        // horizontal + vertical acceleration (including avoidance braking / detour /
        // obstacle clearing).
        // The old 8.0 (~39 deg tilt) was too conservative and made ray-based avoidance
        // "decelerate too late / detour and clear obstacles too hesitantly". Raised to 11.0
        // (~48 deg, still far below the physical tilt ceiling of ~15.7 / 58 deg): real
        // deceleration / manoeuvring acceleration goes from 8 to 11 -> shorter stopping
        // distance, more responsive handling, and the vertical acceleration ceiling for
        // obstacle-clearing climbs rises with it (see the aDesY clamp in this file).
        // Risk: with slow replanning (depth loop) a large acceleration could carry the drone
        // into an obstacle before the next command arrives -- but the avoidance brake factor
        // (brake = min(1, v_safe/spdFwd)) already holds the speed target down so it never
        // charges blindly forward; and the conservative v_safe threshold of yopoAvoidDecel
        // (7.2) is far below this value (11), leaving ~2 m+ of stopping margin, so safety does
        // not regress.
        this.yopoAccMax = 11.0;
        // ── Geometric reactive avoidance (potential field, based on Cesium ground-truth rays)
        //    -- ported from git 3b92a03 ──
        // Independent of DA360 depth: it uses world.pickLocalRay directly to probe horizontal
        // 360 deg ring obstacle distances + ground/roof clearance + three altitude layers
        // (current / above / below), producing repulsion (rep) / tangential detour (tan) /
        // near-obstacle braking (brake) / vertical obstacle clearing (vRep). It only kicks in
        // once an obstacle enters the detection radius; when the path is clear its output is
        // zero -> normal navigation is unaffected.
        this.yopoAvoidEnabled = true;
        // Uniform 360 deg ring of rays: replaces the old coarse 8-direction sampling (45 deg
        // spacing left big gaps on the sides/diagonals, missing corners / pillars / recesses).
        // Generates yopoAvoidRayCount equiangular horizontal rays so an obstacle in any
        // direction is detected.
        this.yopoAvoidRayCount = 36;       // Number of 360 deg rays (10 deg spacing); denser = more ray cost, lower this if the frame rate stutters
        this.yopoAvoidRays = (() => {
            const arr = [], N = this.yopoAvoidRayCount;
            for (let i = 0; i < N; i++) {
                const a = (i * 2 * Math.PI) / N;   // Equiangular on the horizontal plane, covering the full 360 deg
                arr.push({ x: Math.cos(a), y: 0, z: Math.sin(a) });
            }
            return arr;
        })();
        this.yopoAvoidRange = 55.0;   // Obstacle detection radius (m) -- a high-speed cruise needs a
                                      // longer look-ahead: the braking distance v^2/2a is ~13.5 m at
                                      // 15 m/s, and 42 m gives enough margin for detection lag plus
                                      // response, so obstacles are sensed earlier. Ray length is free
                                      // (it does not add GPU cost), so extending it only helps.
        this.yopoAvoidRepRange = 20.0; // Repulsion / tangential / braking range (m): at 18 m/s,
                                      // 20 m provides a ~1.1 s response window, and the braking
                                      // distance v^2/2a ~= 10.8 m < 20 m means it engages earlier
                                      // with margin; the goalClear threshold equals this value, so
                                      // it also keeps the "corridor is clear" verdict from being
                                      // released too early.
                                      // Note: this value is also goalClear's clearThresh, so raising
                                      // it detours earlier but makes "corridor clear" too strict ->
                                      // triggering clearing/detours even when the path really is
                                      // clear. To strengthen the detour tune RepGain/TanGain
                                      // (strength), not this (trigger threshold).
        // At 15 m/s a 20 m action range is only 1.33 s of reaction time, and the braking distance
        // (v^2/2a = 225/14.4 ~= 15.6 m) eats almost all of it. The base value cannot simply be
        // raised because it doubles as goalClear's clearThresh, so a separate high-speed ceiling
        // widens only the repulsion / tangential / brake reach while clearThresh stays at 20 m.
        this.yopoAvoidRepRangeHi = 38.0;   // Repulsion / tangential / brake range at yopoAvoidRefSpeed (m)
        this.yopoAvoidGain = 10.0;    // Generic avoidance gain base: now used mainly for vertical
                                      // safety (upPush/vRep = gain * factor); the horizontal
                                      // rep/tan have been split into separate gains below so they
                                      // can be tuned independently.
        this.yopoAvoidRepGain = 15.0; // Repulsion (radial push-away) max speed (m/s): raised to 15
                                      // for a more decisive push/detour on contact
                                      // (together with the larger repRange it reacts sooner), instead
                                      // of just being "pushed back rather than steered around"
        this.yopoAvoidTanGain = 34.0; // Tangential (detour) speed gain (m/s): 34 (a notch above the
                                      // previous 30) for a more decisive steer-around at speed while
                                      // still well below the 36 that measured as detouring too fast.
                                      // Together with the "lateral speed budget reservation" inside
                                      // _controlYOPO (capped at 55%), the detour component is not
                                      // drowned by the forward component and the motion stays smooth.
        this.yopoAvoidDecel = 8.0;    // The "assumed deceleration" (m/s^2) used by the
                                      // v_safe = sqrt(2ad) brake threshold, deliberately lower than
                                      // yopoAccMax (11): it makes the kinematic brake trigger
                                      // EARLIER and leaves margin (when cruising at 10 m/s it starts
                                      // slowing at ~8 m instead of later). The real stopping
                                      // capability comes from yopoAccMax = 11
                                      // (11 > 8 -> physically it can always stop). The two are
                                      // decoupled: this value conservatively governs "promptness",
                                      // while yopoAccMax governs "strength" (the real deceleration /
                                      // manoeuvring acceleration).
        this.yopoAvoidQueryMs = 20;   // Ray probe throttle (ms): at 18 m/s this refreshes every
                                      // 0.36 m, so probing is denser -> obstacle info is fresher and
                                      // avoidance reacts faster.
        // ── Speed-adaptive probe budget (high-speed responsiveness) ──
        // A full ring probe used to cast 36 + 9 + 2 = 47 forceFresh scene.pickFromRay calls. Every
        // one of them is a complete GPU render plus a read-back pipeline stall, and they run
        // synchronously inside the render frame loop (drone.update -> _controlYOPO), so a single
        // probe can easily cost 60-150 ms. That has two consequences at speed: the avoidance data
        // is already stale by the time it is consumed, and the frame rate collapses -- which in
        // turn slows panorama capture, DA360 depth and the command loop, i.e. all three of the
        // "too slow when flying fast" symptoms at once.
        // Instead of throttling the whole probe (which stales the forward direction, the one that
        // actually matters for braking), the budget below bounds how many GPU picks a single cycle
        // may issue: the forward cone is re-probed every cycle, while the periphery rotates through
        // round-robin slices so the whole ring still refreshes within a few cycles.
        this.yopoAvoidFastSpeed = 6.0;   // m/s: below this the full-resolution profile is used
        this.yopoAvoidRefSpeed = 15.0;   // m/s: the speed at which the high-speed profile is fully applied
        this.yopoAvoidStrideHi = 2;      // Use every 2nd ray (20 deg spacing) at high speed: 18 rays instead of 36
        this.yopoAvoidCoreDeg = 25;      // Half-angle (deg) of the core cone: keeps the full 10 deg
                                         // resolution at every speed, because it is the sector that
                                         // decides the braking distance. Halving the resolution
                                         // there would open 20 deg gaps (a ~10 m hole at 30 m) right
                                         // where a miss is least affordable.
        this.yopoAvoidConeDeg = 55;      // Half-angle (deg) of the outer forward cone re-probed every cycle
        this.yopoAvoidConeDegHi = 45;    // Narrower cone at high speed (fewer rays, still refreshed every cycle)
        this.yopoAvoidSliceMax = 6;      // Max peripheral rays re-probed per cycle (round-robin)
        this.yopoAvoidVertEvery = 1;     // Probe the straight up/down rays every N cycles (1 = every cycle, safety-critical)
        this._avoidRing = null;          // Persistent ring distances carried across cycles (m)
        this._avoidRingAge = null;       // ms since each ring direction was last really probed
        this._avoidSliceCursor = 0;      // Round-robin cursor over the peripheral directions
        this._avoidCycle = 0;            // Probe cycle counter
        this._avoidPrevBlocked = false;  // Whether the previous cycle saw the forward corridor blocked (gates the extra vertical layers)
        this._avoidPerf = { probeMs: 0, rays: 0, rayTotal: 0, cycles: 0, ringAgeMax: 0 };
        this.yopoMinAlt = 2.5;        // Minimum ground/roof clearance (m) -- threshold that triggers soft avoidance (upward push)

        this.yopoAvoidVertRay = true;     // Straight up/down vertical rays (prevents hitting the ceiling / an obstacle straight below)
        this.yopoAvoidVertRange = 12.0;   // Vertical ray detection range (m)
        // ── Vertical obstacle clearing (plan A+B) ──
        this.yopoAvoidVStep = 9.0;        // Vertical probe step up/down (m); *2 high layers can clear taller buildings (8 -> 9: probes slightly higher, clearing taller obstacles)
        this.yopoAvoidVClimbScale = 1.9;  // Vertical clearing speed = gain*scale = 10*1.9 = 19 m/s, a
                                          // fiercer, faster climb over obstacle tops; clamped to 14 by
                                          // droneMaxVSpeed (raised to 14) but full climb is commanded
                                          // earlier, and with yopoAccMax raised the vertical
                                          // acceleration ceiling grows too, so the climb builds faster.
        this.yopoAvoidVBlock = 12.0;      // Forward clearance below this triggers vertical clearing (m):
                                          // at 18 m/s stop+12 ~= 13.1 m ~= 0.73 s, leaving more clearing
                                          // lead time than the old 8 m (0.44 s), so it does not climb
                                          // only when already at the obstacle.
                                          // Do not blindly raise it: the larger the release distance,
                                          // the easier it is to climb spuriously when "the way ahead is
                                          // actually clear". To strengthen clearing tune VClimbScale
                                          // (climb strength), not this (trigger threshold).
        this.yopoAvoidVClear = 0.38;      // Fraction of the range above which a layer counts as
                                          // "clear" (> R*value ~= 13.3 m means clear): raising it makes
                                          // the drone more willing to judge the upper layer flyable,
                                          // increasing clearing willingness (0.32 -> 0.38; vUpDist
                                          // still prevents ceiling hits)
        this.yopoAvoidStop = 1.1;     // Safety clearance (m), matching the body half-width plus margin
        // Range (m) and floor of the "progressive soft brake": the soft brake only provides the
        // comfortable "slower as you get closer" deceleration; the physical stop is handled by the
        // kinematic brake (v_safe = sqrt(2ad)). Previously the soft brake reused repRange (20 m)
        // with no floor, so it started scaling speed down from 20 m out, leaving only 0.44 at 10 m
        // and 0.16 at 5 m -- in urban scenes dAhead is often 8-20 m, so the cruise speed was
        // permanently squeezed to 30%-70% of the commanded speed (the main cause of the 1-4 m/s the
        // user measured). After decoupling, it only engages within 12 m and never drops below 0.55;
        // close range is handled by the kinematic brake, which guarantees a real stop.
        this.yopoAvoidBrakeRange = 12.0;  // Soft-brake deceleration zone (m): no slowdown beyond this distance
        this.yopoAvoidBrakeRangeHi = 22.0; // Soft-brake zone at yopoAvoidRefSpeed (m): at 15 m/s the drone
                                          // needs to start easing off much earlier. Safe to widen because
                                          // the floor (0.55) caps how much it can ever slow down on its
                                          // own -- the physical stop is always the kinematic brake.
        this.yopoAvoidBrakeFloor = 0.55;  // Soft-brake floor: the soft brake alone can only slow to 55% of speed
        this.yopoAvoidBrakeReaction = 0.35; // Brake reaction time (s): the delay between issuing the
                                          // deceleration command and it physically taking effect
                                          // (probe read-back + command link + attitude build-up). The
                                          // drone keeps advancing on the old command during this window,
                                          // so this distance (spd * reaction) is subtracted from the
                                          // available stopping room; 0.35 s is the measured end-to-end lag.
        // Planning deceleration used by the *horizontal* kinematic brake. This is deliberately far below
        // the physically reachable aDecel (7.2) -- the previous brake planned with aDecel and therefore
        // assumed the drone can always decelerate at the physical maximum, but the real velocity
        // controller lags, so the achieved deceleration is much lower and the plan overshot into walls
        // ("deceleration not fast / not timely enough"). Planning with ~3 m/s^2 leaves a ~2x margin over
        // a realistic controller decel, so the commanded target speed is always low enough that the
        // (slower) real deceleration still stops inside the standoff. The physical stop is guaranteed by
        // construction; this only trades a bit of early slowing for never crashing.
        this.yopoAvoidBrakeDecel = 3.0;
        this._avoidProbe = null;      // Potential-field ray probe cache
        this._avoidAccScale = 1.0;    // Potential-field brake scale (used to attenuate the acceleration feed-forward)
        // YOPO polynomial acceleration feed-forward scale: adding the network's cmdAcc raw would
        // shove the drone around, so it is softened to 0.4.
        this.yopoFFAccScale = 0.4;
        // Vertical velocity feed-forward: with 3D navigation the network's vertical trajectory
        // (climb/descent) is a valid manoeuvre, so it is no longer heavily scaled down; only a
        // gentle clamp remains to prevent sudden dives (its ceiling works together with the
        // position loop's vertical error clamp).
        this.yopoFFVelYScale = 1.0;
        this.yopoFFVelYMax = 8.0;
        this.collisionRadius = 0.25;
        this.bounceDamping   = 0.3;
        this._collisionProvider = null; // Gives collision resolution / terrain sampling access (injected in update)

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
        // Reset the SimpleFlight state
        this._sfVelIntX = 0; this._sfVelIntY = 0; this._sfVelIntZ = 0;
        this._sfPrevVelErrX = 0; this._sfPrevVelErrY = 0; this._sfPrevVelErrZ = 0;
        this._sfFiltVelDerrX = 0; this._sfFiltVelDerrY = 0; this._sfFiltVelDerrZ = 0;
        this._sfRateIntPitch = 0; this._sfRateIntRoll = 0; this._sfRateIntYaw = 0;
        this._sfPrevRateErrPitch = 0; this._sfPrevRateErrRoll = 0; this._sfPrevRateErrYaw = 0;
        this._sfPrevAngleErrPitch = 0; this._sfPrevAngleErrRoll = 0;
        this._sfFiltAngleDerrPitch = 0; this._sfFiltAngleDerrRoll = 0;
        this._sfPrevAltErr = 0;
        this._sfFiltAltDerr = 0;
        // Reset the YOPO state
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

        // SimpleFlight gains
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

        // Store the collision provider: geometric avoidance uses it to reach
        // world.pickLocalRay / sampleHeightAtLocal
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
        // Reset the SimpleFlight state
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
        // Reset the YOPO state — only when LEAVING yopo_nav mode.
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
     * SimpleFlight control law — a port of AirSim's simpleflight cascaded PID.
     *
     * 4 cascaded layers: position loop (P) -> velocity loop (PID) -> attitude loop (PD) ->
     * angular-rate loop (PID)
     * The input mapping reuses drone mode: pitch/roll = velocity command, throttle = climb
     * rate, yaw = yaw rate, stick released = position/altitude hold.
     * The output contract matches _controlDrone: thrustOutput (grams of force) plus the
     * attitude accumulated by _applyBodyRotation, integrated by update().
     */
    _controlSimpleFlight(dt, input) {
        const boost = input.boost ? DRONE_BOOST_MULTIPLIER : 1.0;
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        this.boostActive = !!input.boost;
        this.boostMultiplier = boost;

        // ---- 1. Body-frame forward/right (same as _controlDrone) ----
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

        // ---- 2. Position loop (P) -> velocity target ----
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

        // Vertical: stick = climb rate, stick released = altitude hold (PD)
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

        // Clamp the velocity target
        const velTargetH = Math.sqrt(velTargetX * velTargetX + velTargetZ * velTargetZ);
        if (velTargetH > maxSpd) {
            const s = maxSpd / velTargetH;
            velTargetX *= s; velTargetZ *= s;
        }
        velTargetY = clamp(velTargetY, -this.droneMaxVSpeed * boost, this.droneMaxVSpeed * boost);
        this.targetGroundSpeed = Math.sqrt(velTargetX * velTargetX + velTargetZ * velTargetZ);
        this.pilotGroundSpeedCommand = Math.sqrt(pilotCmdX * pilotCmdX + pilotCmdZ * pilotCmdZ);
        this.commandedGroundSpeed = this.targetGroundSpeed;

        // ---- 3. Velocity loop (PID) -> desired acceleration ----
        const velErrX = velTargetX - this.vx;
        const velErrY = velTargetY - this.vy;
        const velErrZ = velTargetZ - this.vz;

        // Clamp the horizontal velocity error so the acceleration demand stays below the
        // tilt-angle ceiling
        const maxAngle = this.droneMaxAngle;
        const aMaxHoriz = G * Math.tan(maxAngle * DEG2RAD);
        const velErrClamp = aMaxHoriz / Math.max(0.01, this.sfVelKp);
        const velErrXc = clamp(velErrX, -velErrClamp, velErrClamp);
        const velErrZc = clamp(velErrZ, -velErrClamp, velErrClamp);

        // Integral with anti-windup
        const viMax = this._sfVelIntMax;
        this._sfVelIntX = clamp(this._sfVelIntX + velErrXc * dt, -viMax, viMax);
        this._sfVelIntY = clamp(this._sfVelIntY + velErrY  * dt, -viMax, viMax);
        this._sfVelIntZ = clamp(this._sfVelIntZ + velErrZc * dt, -viMax, viMax);

        // Derivative (low-pass filtered)
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

        // ---- 4. Project into the body frame -> desired tilt angle ----
        const aFwd   = aDesX * fwdX + aDesZ * fwdZ;
        const aRight = aDesX * rightX + aDesZ * rightZ;
        const targetPitch = clamp(-aFwd / G * RAD2DEG, -maxAngle, maxAngle);
        const targetRoll  = clamp(-aRight / G * RAD2DEG, -maxAngle, maxAngle);

        // ---- 5. Attitude loop (PD) -> desired angular rate ----
        const dec = this._decomposeOrientation();
        const angleErrPitch = targetPitch - dec.bodyPitchDeg;
        const angleErrRoll  = targetRoll  - dec.bodyRollDeg;
        // Derivative low-pass filter, suppressing high-frequency noise
        const adAlpha = 1 - Math.exp(-15 * dt);
        const rawAngleDerrPitch = dt > 0 ? (angleErrPitch - this._sfPrevAngleErrPitch) / dt : 0;
        const rawAngleDerrRoll  = dt > 0 ? (angleErrRoll  - this._sfPrevAngleErrRoll)  / dt : 0;
        this._sfFiltAngleDerrPitch += (rawAngleDerrPitch - this._sfFiltAngleDerrPitch) * adAlpha;
        this._sfFiltAngleDerrRoll  += (rawAngleDerrRoll  - this._sfFiltAngleDerrRoll)  * adAlpha;
        this._sfPrevAngleErrPitch = angleErrPitch;
        this._sfPrevAngleErrRoll  = angleErrRoll;

        const rateTargetPitch = this.sfAngleKp * angleErrPitch + this.sfAngleKd * this._sfFiltAngleDerrPitch;
        const rateTargetRoll  = this.sfAngleKp * angleErrRoll  + this.sfAngleKd * this._sfFiltAngleDerrRoll;

        // ---- 6. Angular-rate loop (PID) -> desired angular velocity -> smoothed and applied ----
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

        // Smooth the angular velocity (simulating rotational inertia, preventing jitter from
        // frame-to-frame jumps)
        const rateSmooth = 1 - Math.exp(-25 * dt);
        this.pitchRate += (angVelPitch - this.pitchRate) * rateSmooth;
        this.rollRate  += (angVelRoll  - this.rollRate)  * rateSmooth;
        this._applyBodyRotation(1, 0, 0, this.pitchRate * dt);
        this._applyBodyRotation(0, 0, 1, this.rollRate * dt);

        // ---- 7. Yaw: angular-rate P tracking (equally smoothed) ----
        const droneYawMax = this.droneMaxYawRate * rates.yaw * boost;
        const rateTargetYaw = input.yaw * droneYawMax;
        const rateErrYaw = rateTargetYaw - this.yawRate;
        this._sfRateIntYaw = clamp(this._sfRateIntYaw + rateErrYaw * dt, -rateIntMax, rateIntMax);
        const angVelYaw = this.sfYawRateKp * rateErrYaw;
        this.yawRate += (angVelYaw - this.yawRate) * rateSmooth;
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);

        // ---- 8. Altitude -> thrust (tilt compensation) ----
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
     * YOPO navigation control law — uses only the YOPO model output (world-frame
     * position/velocity/acceleration/yaw commands).
     *
     * Controller adapted to YOPO: trajectory tracking = position loop P + velocity
     * feed-forward + acceleration feed-forward, yaw P + yaw_dot feed-forward; gains aligned
     * with the YOPO_360 SO3 controller (Hummingbird: kx=2, kv=1.8, kz=3.5).
     * The velocity loop is an SO3-style pure P (no I/D, avoiding integral wind-up /
     * oscillation from replan jumps); the attitude / angular-rate / thrust cascade reuses the
     * same PID set as SimpleFlight, with tilt compensation on the thrust.
     * No extra navigation algorithm / embellishment is stacked on top: it relies purely on the
     * YOPO_360 network's own learning-based avoidance; the geometric reactive avoidance
     * (potential field) was removed as requested.
     *
     * Within 12 m of the goal it switches to a PD convergence onto yopoNavTarget plus a
     * distance-limited deceleration ramp, guaranteeing entry into the arrival circle.
     */
    _controlYOPO(dt, input) {
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        this.boostActive = false;
        this.boostMultiplier = 1.0;
        this.effectiveMaxSpeed = this.droneMaxSpeed;

        // ---- 0. Detect stick activity ----
        const horizActive = Math.abs(input.pitch) > 0.05 || Math.abs(input.roll) > 0.05;
        const vertActive  = Math.abs(input.throttle) > 0.05;
        const yawActive   = Math.abs(input.yaw) > 0.05;
        const stickActive = horizActive || vertActive || yawActive;

        // ---- 0b. Goal distance + final-approach takeover decision ----
        // Inside goal_length (2*radio_range = 10 m) the network's goal observation is squeezed
        // by the 10 m normalisation, and the lattice only holds cruise-type trajectories
        // (endpoint speeds up to vel_max ~= 6 m/s), so near the goal argmin(score) keeps picking
        // overshooting / turn-back trajectories; on top of that, under plan_from_reference the
        // goal-direction observation flips as soon as the reference point passes the goal ->
        // velocity/position oscillate and the goal is never reached.
        // Within yopoFinalApproachDist of the goal we stop following the YOPO trajectory and run
        // a PD convergence straight onto the goal point, guaranteeing entry into the arrival
        // circle.
        let distGoal = Number.POSITIVE_INFINITY;
        if (this.yopoNavTarget) {
            const gdx = this.yopoNavTarget.x - this.x;
            const gdy = this.yopoNavTarget.y - this.y;
            const gdz = this.yopoNavTarget.z - this.z;
            distGoal = Math.sqrt(gdx * gdx + gdy * gdy + gdz * gdz);
        }
        this.yopoDistToGoal = distGoal;

        // Client-side arrival lock (backstop): within yopoArriveHoldM of the goal and with a
        // speed below yopoArriveHoldV -> treat as arrived.
        // The server's 2 m arrival verdict comes back asynchronously, and if the trajectory
        // lingers slightly outside the 2 m circle this backstop makes the client switch to
        // holding at the goal, avoiding "always one step short".
        if (this.yopoNavTarget && distGoal < this.yopoArriveHoldM) {
            const spdNow = Math.sqrt(this.vx*this.vx + this.vy*this.vy + this.vz*this.vz);
            if (spdNow < this.yopoArriveHoldV) this.yopoArrived = true;
        }

        const yopoNearGoalHold =
            this.yopoNavTarget && !stickActive &&
            (this.yopoArrived || distGoal < this.yopoFinalApproachDist);

        // ---- 1. Diagnostics log ----
        if (this.yopoInferenceCount < 5 || this.yopoInferenceCount % 120 === 0) {
            const hasCmd = this.yopoCmdPos ? 'YES' : 'NO';
            const cmdStr = this.yopoCmdPos
                ? `cmd=(${this.yopoCmdPos.x.toFixed(1)},${this.yopoCmdPos.y.toFixed(1)},${this.yopoCmdPos.z.toFixed(1)})`
                : '';
            console.log(`_controlYOPO #${this.yopoInferenceCount}: armed=${input.armed} hasCmd=${hasCmd} ${cmdStr} ` +
                `pos=(${this.x.toFixed(1)},${this.y.toFixed(1)},${this.z.toFixed(1)})`);
        }

        // ---- 2. Body forward/right (same meaning as in _controlSimpleFlight) ----
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);
        let fwdX = -_v3.x, fwdZ = -_v3.z;
        _mat4.getX(_v3);
        let rightX = _v3.x, rightZ = _v3.z;
        const fwdLen = Math.sqrt(fwdX * fwdX + fwdZ * fwdZ);
        if (fwdLen > 1e-4) { fwdX /= fwdLen; fwdZ /= fwdLen; }
        const rightLen = Math.sqrt(rightX * rightX + rightZ * rightZ);
        if (rightLen > 1e-4) { rightX /= rightLen; rightZ /= rightLen; }

        // YOPO navigation maximum horizontal speed. The server default is YOPO_VELOCITY=8.0
        // (cruise vel_max ~= 8); the position-loop error contribution clamped to 4 m/s plus a
        // 12 m/s velocity feed-forward gives a peak of ~16 m/s, so it is clamped to 13 to keep
        // tracking margin and prevent the position error from driving the velocity target to the
        // ceiling when the network switches to a fast trajectory, which would cause a "sudden
        // lurch".
        // Hard ceiling 15 m/s: aligned with the server's YOPO_VELOCITY=15 plus the server's
        // YOPO_SPEED_CAP=15, guaranteeing "no speed limit ever goes above 15 m/s". The real
        // cruise is ~12-15 m/s, with the endpoints clamped by the server / position loop;
        // avoidance was beefed up to match with a larger detection radius, stronger braking and
        // higher gains.
        const yopoMaxSpd = 15.0;
        const maxSpd = stickActive ? this.droneMaxSpeed : yopoMaxSpd;
        const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };

        // ---- 3. Determine the velocity target ----
        let velTargetX, velTargetZ, velTargetY;
        let pilotCmdX = 0, pilotCmdZ = 0;
        let useAccFeedforward = false;

        if (yopoNearGoalHold) {
            // Final-approach takeover: position P + velocity damping D converges straight onto
            // the goal point.
            // The maximum speed shrinks with distance -> a natural deceleration ramp:
            // 12 m -> ~3 m/s, 5 m -> ~1.75 m/s, 1 m -> ~0.8 m/s (floor), combined with the
            // -holdKd*v damping, so it stops smoothly at the goal.
            // When hitting a wall / hugging the ground it is squeezed to a low speed, avoiding
            // pushing back and forth against the obstacle.
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
            // Stick override: use manual control
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
            // YOPO trajectory command: position loop P + velocity feed-forward +
            // acceleration feed-forward
            // yopoCmdPos/Vel/Acc are the desired states from the polynomial evaluation
            // (plan_from_reference).
            // The position loop only compensates the tracking error, while the velocity plus
            // acceleration feed-forward dominate tracking, keeping it efficient and precise.
            //
            // Gains aligned with the YOPO_360 SO3 controller (Hummingbird: kx=2, kv=1.8,
            // kz=3.5).
            // The cascade is equivalent to: kx_eff = velKp*yopoPosKp, kv_eff = velKp.
            // No Ki/Kd: ffVel jumps on every YOPO replan, so Ki would wind up and cause
            // "back and forth" oscillation, and Kd would generate acceleration spikes at the
            // jumps. SO3 itself has no I/D.
            const posErrX = this.yopoCmdPos.x - this.x;
            const posErrZ = this.yopoCmdPos.z - this.z;
            const posErrY = this.yopoCmdPos.y - this.y;

            // Command expiry protection: under the 60 Hz control loop commands are always
            // fresh. Only decays if the control loop dies (> 3 s).
            const cmdAgeS = (performance.now() - this.yopoCmdTime) / 1000;
            const ffDecay = cmdAgeS < 3.0 ? 1.0 : Math.max(0, 1.0 - (cmdAgeS - 3.0) / 1.0);
            const ffX = (this.yopoCmdVel ? this.yopoCmdVel.x : 0) * ffDecay;
            const ffZ = (this.yopoCmdVel ? this.yopoCmdVel.z : 0) * ffDecay;
            // Vertical velocity feed-forward scaling + clamping: the network's vertical
            // trajectory often overshoots (it should climb but commands a huge downward speed);
            // adding that raw would drown the position loop and cause a sudden dive. After
            // clamping, the position loop (which knows the true altitude error) dominates.
            let ffY = (this.yopoCmdVel ? (this.yopoCmdVel.y || 0) : 0) * ffDecay * this.yopoFFVelYScale;
            if (ffY > this.yopoFFVelYMax) ffY = this.yopoFFVelYMax;
            else if (ffY < -this.yopoFFVelYMax) ffY = -this.yopoFFVelYMax;

            const yopoPosKp = 1.0;   // Position loop gain: balances the "pull back to the old commanded position" tendency against cruise speed. Together with the server-side time scaling it makes the drone track faster commands more tightly.
            const yopoAltKp = 1.2;   // Altitude loop gain: with 3D navigation the vertical error is dominated by the network trajectory, the position loop only corrects
            // Position error term clamp: the old 4.0 pinned the cruise speed to +/-4 m/s (the root
            // cause of the 0-4 m/s the user measured -- when the server velocity feed-forward ffX
            // is ~0, the drone can only chase cmdPos through the position loop and is throttled to
            // a crawl by the 4 ceiling).
            // Raised to 15, the position loop can output horizontal/vertical speeds matching the
            // YOPO planned speed (~15), truly unlocking cruise; yopoMaxSpd still clamps hard
            // against lurching on replan jumps, and avoidance is strong enough to hold high
            // speeds.
            const yopoPosErrMaxV = 15.0;  // Horizontal position error contribution ceiling (m/s): matches the YOPO speed ceiling of ~15
            const yopoAltErrMaxV = 15.0;  // Vertical position error contribution ceiling (m/s): 3D navigation allows vertical manoeuvres
            velTargetX = clamp(yopoPosKp * posErrX, -yopoPosErrMaxV, yopoPosErrMaxV) + ffX;
            velTargetZ = clamp(yopoPosKp * posErrZ, -yopoPosErrMaxV, yopoPosErrMaxV) + ffZ;
            velTargetY = clamp(yopoAltKp * posErrY, -yopoAltErrMaxV, yopoAltErrMaxV) + ffY;
            useAccFeedforward = true;
        } else if (this.yopoCmdVel && (Math.abs(this.yopoCmdVel.x) > 0.01 || Math.abs(this.yopoCmdVel.z) > 0.01)) {
            // Only a YOPO velocity command (no position command) -> pure velocity tracking
            velTargetX = this.yopoCmdVel.x;
            velTargetZ = this.yopoCmdVel.z;
            velTargetY = this.yopoCmdVel.y || 0;
        } else {
            // No YOPO command -> hover (do not fly straight at the goal, which would bypass
            // avoidance)
            velTargetX = 0; velTargetZ = 0;
            velTargetY = 0;
        }

        // ── Geometric reactive avoidance (potential field, ported from git 3b92a03) ──
        // Based on Cesium ground-truth rays: probe the horizontal 360 deg ring obstacle
        // distances + ground/roof clearance + three altitude layers, producing repulsion (rep) /
        // tangential detour (tan) / near-obstacle braking (brake) / vertical obstacle clearing
        // (vRep). This is the **active** avoidance layer: it keeps detouring and braking from
        // mid range (4-25 m) onwards. When the path is clear its output is zero -> navigation is
        // unaffected.
        this._avoidAccScale = 1.0;
        // During the final-approach takeover (yopoNearGoalHold: within 12 m of the goal or
        // already arrived) the PD already converges straight onto the goal point.
        // In this phase the potential field still keeps "detour (tan) + slowdown (brake) +
        // vertical collision protection", but does NOT add the normal-direction rep --
        // otherwise rep would push the drone away from the goal and fight the PD back and forth
        // along the same line -> "swinging / wandering", and once pushed away yopoArrived would
        // never be set while the field stays active forever (a dead loop). Taking only
        // tan+brake avoids obstacles inside the takeover range (detour + slowdown) without
        // fighting the PD. Collisions are additionally covered by _handleCollisions.
        if (this.yopoAvoidEnabled && this.yopoNavTarget &&
            !stickActive && !this.yopoArrived) {
            this._updateAvoidProbe();
            const avoid = this._avoidanceVelocity(velTargetX, velTargetZ);
            if (avoid) {
                if (yopoNearGoalHold) {
                    // Final-approach takeover: add only the detour (tan) and slowdown (brake),
                    // not the normal-direction rep (which would push it off the goal and swing)
                    velTargetX = velTargetX * avoid.brake + avoid.tanX;
                    velTargetZ = velTargetZ * avoid.brake + avoid.tanZ;
                } else {
                    // Core of horizontal obstacle avoidance: budget "forward progress" and
                    // "lateral detour (rep+tan)" separately.
                    // Previously velTarget = forward*brake + rep + tan was then clamped to maxSpd
                    // as a whole, so the forward component took the lion's share and the
                    // tangential one got scaled away -> "charging at full speed while grazing",
                    // never getting around the obstacle.
                    // Now the lateral detour vector is preserved on its own: the closer the
                    // obstacle / the stronger the detour, the more the forward component is
                    // suppressed, so the velocity vector really tilts tangentially (sliding past
                    // the obstacle). Lateral takes at most 55% of the speed budget (the previous
                    // 70% measured as detouring too fast), forward keeps at least 30% to avoid
                    // stalling completely; once past the obstacle dAhead grows, steer returns to
                    // zero and full speed resumes automatically.
                    const fwdX = velTargetX * avoid.brake;
                    const fwdZ = velTargetZ * avoid.brake;
                    const steerX = avoid.repX + avoid.tanX;
                    const steerZ = avoid.repZ + avoid.tanZ;
                    const steerMag = Math.hypot(steerX, steerZ);
                    if (steerMag > 1e-3) {
                        const lateralBudget = Math.min(maxSpd * 0.55, steerMag);
                        const fwdAllow = Math.max(maxSpd * 0.30, maxSpd - lateralBudget);
                        const fwdMag = Math.hypot(fwdX, fwdZ);
                        if (fwdMag > fwdAllow) {
                            const s = fwdAllow / fwdMag;
                            velTargetX = fwdX * s + steerX;
                            velTargetZ = fwdZ * s + steerZ;
                        } else {
                            velTargetX = fwdX + steerX;
                            velTargetZ = fwdZ + steerZ;
                        }
                    } else {
                        velTargetX = fwdX + steerX;
                        velTargetZ = fwdZ + steerZ;
                    }
                }
                // Horizontal detour around vertical obstacles (something straight below / above):
                // add vGo to leave the obstacle footprint smoothly (neither climbing nor
                // descending)
                velTargetX += avoid.vGoX;
                velTargetZ += avoid.vGoZ;
                // Vertical: ground clearance push-up + vertical obstacle clearing + descent
                // kinematic brake. Kept during the final phase too (protects against ground /
                // ceiling / obstacles below).
                velTargetY = velTargetY * avoid.brake;
                if (avoid.vRep) velTargetY = velTargetY * 0.3 + avoid.vRep;
                velTargetY += avoid.upPush;
                // Vertical descent kinematic brake: the maximum allowed descent speed is
                // vSafeDown (>= 0).
                // If the network trajectory demands a faster descent (very negative velTargetY),
                // clamp it to -vSafeDown, guaranteeing it can physically stop within the
                // clearance below / ahead-below -> no crash into obstacles below.
                if (avoid.vSafeDown !== null && Number.isFinite(avoid.vSafeDown)) {
                    if (velTargetY < -avoid.vSafeDown) {
                        velTargetY = -avoid.vSafeDown;
                        this._yopoGroundFloorActive = true; // Trigger climb / hold attitude
                    }
                }
                // Obstacle straight below / above: hold altitude, neither climb nor descend, and
                // let the horizontal detour vGo fly past smoothly, avoiding the "wants to
                // descend -> pushed away by rays/collision -> wants to descend again" oscillation.
                if (Math.hypot(avoid.vGoX, avoid.vGoZ) > 1e-6) {
                    velTargetY = 0;
                }
                this._avoidAccScale = avoid.brake;
            }
        }

        // ── Passive ground safety net (not geometric avoidance) ──
        // The geometric reactive avoidance (potential field) was removed as requested; only the
        // passive safety net based on terrain height sampling remains:
        // when the clearance drops below yopoCrashFloor a climb is forced, preventing a blind
        // descent into the ground during fast flight + replanning gaps.
        this._yopoGroundFloorActive = false;
        const cp = this._collisionProvider;
        const w = cp ? cp.world : null;
        let groundGap = Number.POSITIVE_INFINITY;
        if (w && w.ready && typeof w.sampleHeightAtLocal === 'function') {
            const gy = w.sampleHeightAtLocal(this.x, this.z, 0.6);
            if (Number.isFinite(gy)) groundGap = this.y - gy;
        }
        if (Number.isFinite(groundGap) && groundGap < this.yopoCrashFloor) {
            // The closer to the ground, the higher the climb rate; at least +1 m/s to break away.
            const climb = (this.yopoCrashFloor - groundGap) * 4.0 + 1.0;
            if (velTargetY < climb) velTargetY = climb;
            this._yopoGroundFloorActive = true;
        }

        // Clamp the velocity target
        const velTargetH = Math.sqrt(velTargetX * velTargetX + velTargetZ * velTargetZ);
        if (velTargetH > maxSpd) {
            const s = maxSpd / velTargetH;
            velTargetX *= s; velTargetZ *= s;
        }
        velTargetY = clamp(velTargetY, -this.droneMaxVSpeed, this.droneMaxVSpeed);
        this.targetGroundSpeed = Math.sqrt(velTargetX * velTargetX + velTargetZ * velTargetZ);

        // Diagnostics: record the velocity target
        if (this.yopoInferenceCount < 5 || this.yopoInferenceCount % 120 === 0) {
            console.log(`_controlYOPO velTarget=(${velTargetX.toFixed(2)},${velTargetY.toFixed(2)},${velTargetZ.toFixed(2)}) ` +
                `stickActive=${stickActive} thrust=${this.thrustOutput.toFixed(0)}`);
        }
        this.pilotGroundSpeedCommand = Math.sqrt(pilotCmdX * pilotCmdX + pilotCmdZ * pilotCmdZ);
        this.commandedGroundSpeed = this.targetGroundSpeed;

        // ---- 4. Velocity loop (PID) -> desired acceleration ----
        // YOPO trajectory tracking uses an SO3-style pure-P velocity loop (no I/D):
        //   - no Ki: avoids the integral wind-up and "back and forth" oscillation caused by the
        //     ffVel jump on replan
        //   - no Kd: avoids acceleration/tilt spikes from d(velErr)/dt at the replan jump
        // The gain is 1.5 (below the SO3 hummingbird kv ~= 1.8, since the user asked for modest
        // compensation):
        //   tracking is dominated by the ffVel/ffAcc feed-forward and the P loop only corrects
        //   gently, so motion is smoother and less jerky.
        // Stick / hover mode still uses the SimpleFlight default PID gains.
        const velErrX = velTargetX - this.vx;
        const velErrY = velTargetY - this.vy;
        const velErrZ = velTargetZ - this.vz;

        const maxAngle = this.droneMaxAngle;
        const aMaxHoriz = G * Math.tan(maxAngle * DEG2RAD);
        // YOPO-specific velocity loop parameters (tuned down, gentle compensation)
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

        // Velocity loop PID -> desired acceleration
        let aDesX = velKp * velErrXc + velKi * this._sfVelIntX + velKd * this._sfFiltVelDerrX;
        let aDesY = velKp * velErrY  + velKi * this._sfVelIntY + velKd * this._sfFiltVelDerrY;
        let aDesZ = velKp * velErrZc + velKi * this._sfVelIntZ + velKd * this._sfFiltVelDerrZ;

        // Acceleration feed-forward: the YOPO polynomial acceleration is added directly to
        // improve trajectory tracking accuracy and efficiency.
        // The SO3-style P controller leans harder on ffAcc (no Ki/Kd to mask it), but the command
        // goes stale between two server responses (depth capture takes ~100-300 ms). A stale
        // ffAcc comes from the polynomial at the old ctrl_time, so both its direction and
        // magnitude can be wrong. It decays linearly with the command age: full below 80 ms,
        // ramping to 0 over 80-200 ms, off above 200 ms.
        if (useAccFeedforward && this.yopoCmdAcc) {
            const cmdAgeMs = this.yopoCmdTime > 0 ? (performance.now() - this.yopoCmdTime) : 999;
            let ffScale = this.yopoFFAccScale;
            if (cmdAgeMs > 200) {
                ffScale = 0.0;
            } else if (cmdAgeMs > 80) {
                ffScale = 1.0 - (cmdAgeMs - 80) / 120;
            }
            // Attenuate the acceleration feed-forward while the potential field brakes:
            // otherwise the polynomial ffAcc would still "push" the drone toward the obstacle
            ffScale *= this._avoidAccScale || 1.0;
            aDesX += this.yopoCmdAcc.x * ffScale;
            aDesY += (this.yopoCmdAcc.y || 0) * ffScale;
            aDesZ += this.yopoCmdAcc.z * ffScale;
            // When the hard ground floor triggers, forbid a downward acceleration feed-forward
            // (it would cancel the climb) and force aDesY >= 0
            if (this._yopoGroundFloorActive) {
                if (this.yopoCmdAcc && this.yopoCmdAcc.y < 0) aDesY -= this.yopoCmdAcc.y * ffScale;
                if (aDesY < 0) aDesY = 0;
            }
        }

        // ── Desired acceleration safety ceiling (prevents "acceleration too large -> hits an
        // obstacle before the next command") ──
        // When replanning (the depth loop) is slow, an excessively large combined acceleration
        // can carry the drone into a solid before the next avoidance command arrives. The
        // combined horizontal acceleration and the vertical acceleration are each clamped to
        // yopoAccMax, leaving braking and reaction margin (the equivalent max tilt drops from
        // 58 deg to ~atan(8/9.81) ~= 39 deg).
        const aMaxCmd = this.yopoAccMax;
        const aH = Math.hypot(aDesX, aDesZ);
        if (aH > aMaxCmd) {
            const s = aMaxCmd / aH;
            aDesX *= s;
            aDesZ *= s;
        }
        if (aDesY > aMaxCmd) aDesY = aMaxCmd;
        else if (aDesY < -aMaxCmd) aDesY = -aMaxCmd;

        // ---- 5. Project into the body frame -> desired tilt angle ----
        const aFwd   = aDesX * fwdX + aDesZ * fwdZ;
        const aRight = aDesX * rightX + aDesZ * rightZ;
        const targetPitch = clamp(-aFwd / G * RAD2DEG, -maxAngle, maxAngle);
        const targetRoll  = clamp(-aRight / G * RAD2DEG, -maxAngle, maxAngle);

        // ---- 6. Attitude loop (PD) -> desired angular rate ----
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

        // ---- 7. Angular-rate loop (PID) ----
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

        // ---- 8. Yaw: track the YOPO yaw command ----
        let targetYawRate = 0;
        if (yawActive) {
            // Stick-controlled yaw
            const droneYawMax = this.droneMaxYawRate;
            targetYawRate = input.yaw * droneYawMax;
        } else if (yopoNearGoalHold) {
            // Final-approach takeover: hold the current yaw, do not rotate
            targetYawRate = 0;
        } else if (this.yopoCmdYaw !== null) {
            // Track the YOPO yaw command (P control + yaw_dot feed-forward)
            // yopoCmdYaw is already rate-limited by calculate_yaw() (max 0.5*pi rad/s),
            // and its coordinate system matches this.yaw (ROS yaw = drone yaw), so it can be
            // subtracted directly.
            let cmdYawDeg = this.yopoCmdYaw * RAD2DEG;
            let yawErr = cmdYawDeg - this.yaw;
            while (yawErr > 180) yawErr -= 360;
            while (yawErr < -180) yawErr += 360;
            // yaw_dot feed-forward (yopoCmdYawDot comes from the server, already in deg/s)
            const yawDotFeed = (this.yopoCmdYawDot || 0) * RAD2DEG;
            targetYawRate = clamp(yawErr * 3.0 + yawDotFeed,
                                  -this.droneMaxYawRate, this.droneMaxYawRate);
        }
        const rateErrYaw = targetYawRate - this.yawRate;
        const angVelYaw = this.sfYawRateKp * rateErrYaw;
        this.yawRate += (angVelYaw - this.yawRate) * rateSmooth;
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);

        // ---- 9. Altitude -> thrust (tilt compensation) ----
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

    // ---- Geometric reactive avoidance (potential field) helpers — ported from git 3b92a03 ----

    /** Throttled update of the ray probe cache: probes once every yopoAvoidQueryMs under the
     * control loop. */
    _updateAvoidProbe() {
        const now = performance.now();
        const p = this._avoidProbe;
        // Probe throttle adapts to speed: stays dense at high speed (60 ms) and relaxes at low
        // speed / when stationary (up to 400 ms), balancing "avoidance stays prompt when flying
        // fast" against "frame rate" (the GPU cost of a forceFresh full probe scales linearly
        // with frequency).
        const spdHNow = Math.hypot(this.vx, this.vz);
        // Above yopoAvoidFastSpeed the probe runs at the full rate (yopoAvoidQueryMs) so the
        // forward cone is refreshed every cycle; below it the interval relaxes up to 400 ms,
        // since a slow-moving drone needs far less look-ahead and the saved time goes to the
        // frame rate.
        const queryMs = spdHNow > this.yopoAvoidFastSpeed
            ? this.yopoAvoidQueryMs
            : Math.max(this.yopoAvoidQueryMs, Math.min(400, 400 - spdHNow * 25));
        if (p && now - p.time < queryMs) return;
        // Reusable when the position barely changed, reducing Cesium pickFromRay cost.
        // The 900 ms reuse window is only safe when the drone is nearly stationary: at speed the
        // carried-over ring data goes stale quickly, so it is capped to ~2 probe intervals.
        if (p) {
            const moved = Math.hypot(this.x - p.x, this.z - p.z);
            const dy = Math.abs(this.y - p.y);
            const reuseMs = spdHNow > this.yopoAvoidFastSpeed
                ? Math.min(120, queryMs * 2)
                : 900;
            if (moved < 0.4 && dy < 2.0 && now - p.time < reuseMs) return;
        }
        this._avoidProbe = this._computeAvoidProbe();
    }

    /**
     * Probe the horizontal 360 deg ring obstacle distances + ground/roof clearance
     * (world frame, metres).
     * Uses a "fan ray bundle": for every main direction several parallel rays are cast from
     * points offset left/right of the body, and the nearest distance is taken.
     * A single centre ray misses recessed parts of buildings (recessed windows, doorways,
     * concave walls) by passing right through them; the offset rays on both sides hit the wall
     * edges flanking the recess, so the "recessed wall" is perceived as an obstacle and the
     * drone does not wrongly assume the way ahead is clear and crash into the recess's
     * side/back walls.
     */
    _computeAvoidProbe() {
        const tStart = performance.now();
        const provider = this._collisionProvider;
        const w = provider ? provider.world : null;
        if (!w || !w.ready || typeof w.pickLocalRay !== 'function') return null;

        const dirs = this.yopoAvoidRays;
        const R = this.yopoAvoidRange;
        const N = dirs.length;
        const prevProbe = this._avoidProbe;
        const dtProbe = prevProbe ? Math.max(0, tStart - prevProbe.time) : 0;

        // Compute the ground clearance first: used to clamp the start of the down-probe layer
        // and avoid false detections when hugging the ground
        let groundGap = Number.POSITIVE_INFINITY;
        if (typeof w.sampleHeightAtLocal === 'function') {
            const gy = w.sampleHeightAtLocal(this.x, this.z, 0.6);
            if (Number.isFinite(gy)) groundGap = this.y - gy;
        }
        const groundY = Number.isFinite(groundGap) ? (this.y - groundGap) : -1e9;

        // A single horizontal ray (cast from the body position) takes the nearest obstacle
        // distance. The old "fan bundle against recess misses" was removed:
        // only 1 centre ray per direction, simplifying the probe (at the cost of missing
        // recessed windows / recess inner walls).
        // forceFresh=true: skips the pickLocalRay cache and does a real pick every time. At high
        // speed the drone moves several metres within the cache TTL, so a cached hit returns a
        // stale distance -> the braking distance is computed wrong -> wall impact; avoidance
        // must use the current true distance. Throttling happens in _updateAvoidProbe.
        let rayCount = 0;
        const pickF = (o, d, dist) => w.pickLocalRay(o, d, dist, true);
        const rayDist = (dir, yLevel) => {
            rayCount++;
            const hit = pickF({ x: this.x, y: yLevel, z: this.z }, dir, R);
            return (hit && Number.isFinite(hit.distance) && hit.distance > 0.04) ? hit.distance : R;
        };
        // Forward direction (horizontal): velocity takes priority, otherwise the body forward -Z
        let fwdHx = 0, fwdHz = -1;
        const spdHv = Math.hypot(this.vx, this.vz);
        if (spdHv > 0.3) { fwdHx = this.vx / spdHv; fwdHz = this.vz / spdHv; }

        // ── Speed profile ──
        // Every GPU pick below is a full scene render plus a read-back stall, executed
        // synchronously in the render frame loop, so the number of picks per cycle -- not the
        // throttle interval -- is what decides whether avoidance keeps up at speed.
        const tFast = Math.max(0, Math.min(1,
            (spdHv - this.yopoAvoidFastSpeed) /
            Math.max(1e-3, this.yopoAvoidRefSpeed - this.yopoAvoidFastSpeed)));
        const stride = tFast >= 0.5 ? Math.max(1, Math.round(this.yopoAvoidStrideHi)) : 1;
        const coneDeg = this.yopoAvoidConeDeg +
            (this.yopoAvoidConeDegHi - this.yopoAvoidConeDeg) * tFast;
        const coneCos = Math.cos(coneDeg * Math.PI / 180);
        const coreCos = Math.cos(this.yopoAvoidCoreDeg * Math.PI / 180);

        // Persistent ring state: a direction that is not re-probed this cycle keeps its last
        // measured distance, so the repulsion / detour sums always see a complete 360 deg ring
        // instead of a partially filled one.
        if (!this._avoidRing || this._avoidRing.length !== N) {
            this._avoidRing = new Float64Array(N).fill(R);
            this._avoidRingAge = new Float64Array(N).fill(1e9);
            this._avoidSliceCursor = 0;
        }
        const ring = this._avoidRing;
        const ringAge = this._avoidRingAge;
        for (let i = 0; i < N; i += stride) ringAge[i] = Math.min(1e9, ringAge[i] + dtProbe);

        // Split the directions into three tiers:
        //   core  — the braking-critical sector straight ahead, always full resolution and always
        //           re-probed (a miss here is what actually causes a crash);
        //   cone  — the wider forward sector, re-probed every cycle but downsampled at speed;
        //   periphery — everything else, rotated through round-robin slices across cycles.
        const coreIdx = [];
        const coneIdx = [];
        const periIdx = [];
        for (let i = 0; i < N; i++) {
            const dot = dirs[i].x * fwdHx + dirs[i].z * fwdHz;
            if (dot >= coreCos) {
                coreIdx.push(i);
            } else if ((i % stride) !== 0) {
                continue;               // dropped by the high-speed stride, mirrored from a neighbour
            } else if (dot >= coneCos) {
                coneIdx.push(i);
            } else {
                periIdx.push(i);
            }
        }
        const coreSet = new Set(coreIdx);
        const coneAll = coreIdx.concat(coneIdx);
        const sliceMax = Math.max(1, Math.round(this.yopoAvoidSliceMax));
        const slice = [];
        for (let k = 0; k < sliceMax && periIdx.length > 0; k++) {
            this._avoidSliceCursor = (this._avoidSliceCursor + 1) % periIdx.length;
            slice.push(periIdx[this._avoidSliceCursor]);
        }

        // Altitude probing: mid (current altitude) for the probed directions; high/high2/low are
        // only probed along the 3 rays best aligned with the forward direction, and only while the
        // forward corridor is actually blocked -- vertical obstacle clearing only cares whether the
        // forward direction can be flown over / dived under, so fewer rays means a better frame
        // rate. The blocking verdict comes from the previous cycle (one cycle of lag is irrelevant
        // for a manoeuvre that takes seconds).
        const dists = new Array(N);
        const distsHigh = new Array(N);
        const distsHigh2 = new Array(N);
        const distsLow = new Array(N);
        const yHigh = this.y + this.yopoAvoidVStep;
        const yHigh2 = this.y + this.yopoAvoidVStep * 2;
        const yLow = Math.max(this.y - this.yopoAvoidVStep, groundY + 1.0);
        const lowOk = (yLow - groundY) > 1.5; // The down-probe layer counts as a valid dive only if clearly above the ground
        const probeAux = this._avoidPrevBlocked;

        // Pick the 3 rays best aligned with the forward direction for the high-layer probe
        // (more coverage -> more directions available for vertical clearing)
        const vProbeIdx = (coneAll.length > 0 ? coneAll : periIdx)
            .slice()
            .sort((a, b) => (dirs[b].x * fwdHx + dirs[b].z * fwdHz) -
                            (dirs[a].x * fwdHx + dirs[a].z * fwdHz))
            .slice(0, 3);

        const probeSet = new Set(coneAll);
        for (const i of slice) probeSet.add(i);
        if (probeAux) for (const i of vProbeIdx) probeSet.add(i);

        for (let i = 0; i < N; i++) {
            if (!coreSet.has(i) && (i % stride) !== 0) {
                // Direction skipped by the high-speed stride: mirror the neighbouring probed
                // direction so the ring stays gap-free (angular resolution drops, coverage does not).
                const nb = i - (i % stride);
                dists[i] = dists[nb];
                distsHigh[i] = distsHigh[nb];
                distsHigh2[i] = distsHigh2[nb];
                distsLow[i] = distsLow[nb];
                continue;
            }
            if (probeSet.has(i)) {
                ring[i] = dists[i] = rayDist(dirs[i], this.y);
                ringAge[i] = 0;
                if (probeAux && vProbeIdx.indexOf(i) >= 0) {
                    distsHigh[i] = rayDist(dirs[i], yHigh);
                    distsHigh2[i] = rayDist(dirs[i], yHigh2);
                    distsLow[i] = lowOk ? rayDist(dirs[i], yLow) : dists[i];
                } else {
                    distsHigh[i] = dists[i];
                    distsHigh2[i] = dists[i];
                    distsLow[i] = dists[i];
                }
            } else {
                dists[i] = ring[i];
                distsHigh[i] = dists[i];
                distsHigh2[i] = dists[i];
                distsLow[i] = dists[i];
            }
        }



        // Straight up / straight down vertical rays: the horizontal ring at any layer cannot
        // detect an obstacle "directly above / below at the same x,z" (such as a ceiling
        // overhead or a square rooftop underfoot). Prevents climbing into the ceiling and
        // descending into an obstacle straight below.
        let vUpDist = R, vDownDist = R;
        const vertEvery = Math.max(1, Math.round(this.yopoAvoidVertEvery));
        if (this.yopoAvoidVertRay && (this._avoidCycle % vertEvery === 0)) {
            rayCount += 2;
            const hUp = pickF({ x: this.x, y: this.y + 0.5, z: this.z }, { x: 0, y: 1, z: 0 }, this.yopoAvoidVertRange);
            vUpDist = (hUp && Number.isFinite(hUp.distance) && hUp.distance > 0.04) ? hUp.distance : R;
            const hDn = pickF({ x: this.x, y: this.y - 0.5, z: this.z }, { x: 0, y: -1, z: 0 }, this.yopoAvoidVertRange);
            vDownDist = (hDn && Number.isFinite(hDn.distance) && hDn.distance > 0.04) ? hDn.distance : R;
        } else if (prevProbe && Number.isFinite(prevProbe.vUpDist)) {
            vUpDist = prevProbe.vUpDist;
            vDownDist = prevProbe.vDownDist;
        }

        // Cheap (CPU-only) forward-corridor estimate used to decide whether the next cycle should
        // spend rays on the vertical layers. It mirrors the corridor measure in
        // _avoidanceVelocity so the extra rays are only issued when clearing is actually reachable.
        let gx = fwdHx, gz = fwdHz;
        if (this.yopoNavTarget) {
            const tdx = this.yopoNavTarget.x - this.x;
            const tdz = this.yopoNavTarget.z - this.z;
            const tl = Math.hypot(tdx, tdz);
            if (tl > 0.5) { gx = tdx / tl; gz = tdz / tl; }
        }
        let fwdCorridor = R;
        for (const i of coneAll) {
            const d = dists[i];
            if (!Number.isFinite(d) || d <= 0) continue;
            const dotG = dirs[i].x * gx + dirs[i].z * gz;
            if (dotG <= 0) continue;
            const latG = d * Math.sqrt(Math.max(0, 1 - dotG * dotG));
            if (latG < 3.0 && d < fwdCorridor) fwdCorridor = d;
        }
        this._avoidPrevBlocked = fwdCorridor < (this.yopoAvoidStop + this.yopoAvoidVBlock);
        this._avoidCycle++;

        let ringAgeMax = 0;
        for (let i = 0; i < N; i += stride) {
            if (ringAge[i] > ringAgeMax) ringAgeMax = ringAge[i];
        }
        const probeMs = performance.now() - tStart;
        this._avoidPerf.probeMs = probeMs;
        this._avoidPerf.rays = rayCount;
        this._avoidPerf.rayTotal += rayCount;
        this._avoidPerf.cycles++;
        this._avoidPerf.ringAgeMax = ringAgeMax;

        return {
            dists,
            distsHigh,
            distsHigh2,
            distsLow,
            lowOk,
            groundGap,
            vUpDist,
            vDownDist,
            highProbeIdx: vProbeIdx, // Indices of the directions that got a high-layer probe (vertical clearing can only judge on those)
            probeMs,
            ringAgeMax,
            x: this.x, y: this.y, z: this.z,
            time: performance.now(),
        };
    }

    /**
     * Potential-field avoidance velocity: returns {repX, repZ, tanX, tanZ, brake, upPush,
     * vRep}.
     *   - repulsion (rep): grows as the obstacle gets closer, pointing away from the obstacle
     *     cluster;
     *   - tangential detour (tan): perpendicular to the repulsion direction, choosing the side
     *     closer to the goal / desired direction, so the drone slides along the obstacle toward
     *     the goal and escapes potential-field local minima;
     *   - brake: the closer the threat ahead, the slower (0..1);
     *   - upPush: pushes up when the ground/roof clearance is insufficient;
     *   - vRep: vertical obstacle-clearing speed (climb / dive when the horizontal way is
     *     strongly blocked while one vertical side is clear).
     * Returns non-zero only once an obstacle is inside yopoAvoidRange; when the path is clear
     * brake = 1 and rep/tan = 0, so the ultimate goal of navigating to the target point is
     * never affected.
     */
    _avoidanceVelocity(velTargetX, velTargetZ) {
        const p = this._avoidProbe;
        if (!p) return null;
        const R = this.yopoAvoidRange;
        const dirs = this.yopoAvoidRays;
        const dists = p.dists;
        if (!dists || dists.length !== dirs.length) return null;
        // Local clamp: several methods in this file define their own local clamp (different
        // scopes), so it is needed here too -- otherwise the clamp call in the soft-brake logic
        // below would raise a ReferenceError.
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        // Safe braking deceleration: it must use the "actually reachable" value, not the
        // theoretical value for the physical tilt limit.
        // _controlYOPO clamps both the combined horizontal acceleration and the vertical
        // acceleration to yopoAccMax (8 m/s^2), while yopoAvoidDecel (15) is only the theoretical
        // ceiling for the 58 deg tilt limit. Computing v_safe = sqrt(2*a*d) with 15 overestimates
        // the braking capability by nearly a factor of two -> the speed the kinematic brake lets
        // through cannot be stopped physically.
        // Take the smaller of the two and multiply by 0.9 to compensate for the response lag
        // (ray throttle 35 ms + control loop 20 ms + attitude build-up), during which the drone
        // still advances on the old command.
        const aDecel = Math.min(this.yopoAvoidDecel, this.yopoAccMax) * 0.9;

        // Speed-adaptive action ranges. At 15 m/s the fixed 20 m range leaves only ~1.3 s, while
        // the braking distance alone is ~15.6 m, so repulsion / detour / braking all engage far
        // too late. These scale with speed up to yopoAvoidRefSpeed; goalClear's clearThresh
        // deliberately keeps using the fixed yopoAvoidRepRange so widening the action range does
        // not make the "corridor is clear" verdict stricter (see the goalClear block below).
        const spdNow = Math.hypot(this.vx, this.vz);
        const tFast = Math.max(0, Math.min(1,
            (spdNow - this.yopoAvoidFastSpeed) /
            Math.max(1e-3, this.yopoAvoidRefSpeed - this.yopoAvoidFastSpeed)));
        const repRange = this.yopoAvoidRepRange +
            (this.yopoAvoidRepRangeHi - this.yopoAvoidRepRange) * tFast;
        const brakeRange = this.yopoAvoidBrakeRange +
            (this.yopoAvoidBrakeRangeHi - this.yopoAvoidBrakeRange) * tFast;

        let repX = 0, repZ = 0;
        let dMin = R;        // Nearest obstacle overall (drives repulsion / tangential strength)
        let dAhead = R;      // Threat ahead (including vertical threats, used for braking / push-up)
        let dAheadH = R;     // Nearest-ahead distance from the horizontal ring rays only
                             // (excluding vertical threats), used only for the vertical clearing
                             // decision, so vertical threats cannot shrink dAhead and wrongly
                             // trigger flying over / diving under.
        let openDirX = 0, openDirZ = 0, openMax = -1;

        const des = Math.hypot(velTargetX, velTargetZ);
        let udx = 0, udz = 0;
        if (des > 0.3) { udx = velTargetX / des; udz = velTargetZ / des; }

        // Goal bearing (body -> navigation goal): computed up front, used by the "corridor"
        // check for dAheadH below.
        // Falls back to the commanded velocity direction when there is no navigation goal.
        let gx = udx, gz = udz;
        if (this.yopoNavTarget) {
            const tdx = this.yopoNavTarget.x - this.x;
            const tdz = this.yopoNavTarget.z - this.z;
            const tl = Math.hypot(tdx, tdz);
            if (tl > 0.5) { gx = tdx / tl; gz = tdz / tl; }
        }
        // Corridor half-width (m) used to trigger obstacle clearing: slightly wider than
        // goalClear's pathHalfWidth (2.5) to leave a safety margin.
        const surmountHalfW = 3.0;

        for (let i = 0; i < dirs.length; i++) {
            const d = dists[i];
            if (!Number.isFinite(d) || d <= 0) continue;
            if (d < dMin) dMin = d;
            if (d > openMax) { openMax = d; openDirX = dirs[i].x; openDirZ = dirs[i].z; }
            if (d < repRange) {
                const w = 1 - d / repRange;
                repX -= dirs[i].x * w;
                repZ -= dirs[i].z * w;
            }
            // Threat ahead: obstacles near the desired velocity direction count toward braking
            const dot = dirs[i].x * udx + dirs[i].z * udz;
            if (des > 0.3 ? (dot > 0.5 && d < dAhead) : (d < dAhead)) dAhead = d;
            // dAheadH (clearing only): counts only horizontal obstacles that really sit inside the
            // goal corridor -- measured by lateral offset, not by the cone's minimum distance.
            // The old dot > 0.5 (a wide +/-60 deg cone) let a near obstacle 55 deg to the side
            // shrink it too; that is inconsistent with goalClear's corridor criterion, so "the way
            // to the goal is actually clear" cases were jointly misfired into flying over / diving
            // under by (a distant corridor obstacle + a near side obstacle).
            const dotG = dirs[i].x * gx + dirs[i].z * gz;
            if (dotG > 0) {
                const latG = d * Math.sqrt(Math.max(0, 1 - dotG * dotG));
                if (latG < surmountHalfW && d < dAheadH) dAheadH = d;
            }
        }

        // Insufficient ground/roof clearance -> push up and take part in braking
        let upPush = 0;
        if (Number.isFinite(p.groundGap) && p.groundGap < this.yopoMinAlt) {
            upPush = (this.yopoMinAlt - p.groundGap) * this.yopoAvoidGain * 0.5;
            if (p.groundGap < dAhead) dAhead = p.groundGap;
        }

        // ---- Vertical descent kinematic brake ----
        // The horizontal direction already has the v_safe = sqrt(2ad) brake, but vertically there
        // is only upPush (triggered when the clearance drops below minAlt), so a fast descent
        // (network trajectory descent / final descent) simply cannot stop within 2.5 m of
        // clearance -> it crashes into obstacles below (user feedback).
        // Here the maximum descent speed is limited by the clearance straight below:
        //   vSafeDown = sqrt(2*a*(gap - standoff))
        // so the drone can physically stop at any clearance instead of "diving at full speed into
        // the ground".
        // When upPush is weak, vSafeDown clamps velTargetY directly (see the _controlYOPO call
        // site).
        let vSafeDown = null;
        const downGap = Math.min(
            Number.isFinite(p.groundGap) ? p.groundGap : R,
            Number.isFinite(p.vDownDist) ? p.vDownDist : R
        );
        if (Number.isFinite(downGap) && downGap < this.yopoAvoidRange) {
            const aD = aDecel;
            const sd = this.yopoAvoidStop;
            if (downGap <= sd) {
                vSafeDown = 0;          // Clearance already insufficient -> forbid descending entirely
            } else {
                vSafeDown = Math.sqrt(2 * aD * (downGap - sd));
            }
            // Strengthen the push-up when clearance is insufficient (take the larger of the two)
            if (downGap < this.yopoMinAlt) {
                const push = (this.yopoMinAlt - downGap) * this.yopoAvoidGain * 0.6;
                if (push > upPush) upPush = push;
            }
        }

        // Limit the push-up speed when the clearance overhead is insufficient (symmetric to
        // vSafeDown's descent brake): the push-up speed must not exceed the value that can still
        // stop before the obstacle overhead, preventing climbing / pushing into the ceiling (the
        // user reported easy ceiling hits).
        if (Number.isFinite(p.vUpDist)) {
            const aU = aDecel, su = this.yopoAvoidStop;
            if (p.vUpDist <= su) { if (upPush > 0) upPush = 0; }
            else {
                const vSafeUp = Math.sqrt(2 * aU * (p.vUpDist - su));
                if (upPush > vSafeUp) upPush = vSafeUp;
            }
        }



        // Near-obstacle braking: two-layer progressive deceleration for both sensitivity and a
        // physical stop.
        //   1) Hard kinematic brake: v_safe = sqrt(2*a*(d - standoff)), guaranteeing a stop within
        //      the clearance no matter what (however fast it flies, hitting the wall is physically
        //      impossible). a uses the conservative yopoAvoidBrakeDecel, deliberately well below the
        //      physically reachable aDecel -- the real velocity controller cannot decelerate at the
        //      physical maximum, so planning with it overshoots; the lower value leaves margin for the
        //      slower real deceleration.
        //   2) Progressive soft brake: within yopoAvoidBrakeRange it scales the speed down
        //      smoothly with distance (with a floor) so the drone gets "slower as it gets closer",
        //      decelerating early instead of braking abruptly at the last moment. The more
        //      conservative (smaller) of the two brakes wins.
        //      Division of labour: the soft brake only handles comfortable deceleration, the
        //      physical stop belongs to (1) -- the soft brake must no longer be squeezed all the
        //      way to 0 and dominate the speed limit (that was exactly what caused 1-4 m/s).
        let brake = 1;
        const standoff = this.yopoAvoidStop;
        // Use the LARGER of the commanded and the actual velocity for the reaction distance: if the
        // drone is actually moving faster than the network commands (glide / overshoot), braking must
        // cover the real closing speed, not the optimistic commanded one.
        const spdFwd = Math.max(
            Math.hypot(velTargetX, velTargetZ),
            Math.hypot(this.vx, this.vz)
        );
        // Reaction buffer: the deceleration command does not physically bite until the drone has
        // tilted to the new attitude, so during the reaction window it keeps moving at the old
        // speed over reactionDist = spdFwd * reactionSec. That distance is subtracted from the
        // available stopping room, so the brake begins early enough to still stop inside standoff.
        // Without this, at 15 m/s the brake only engaged ~1.7 m too late and the drone grazed the
        // wall. The buffer scales with speed automatically (the faster it goes, the more lead it
        // needs).
        const reactionDist = spdFwd * this.yopoAvoidBrakeReaction;
        if (dAhead <= standoff + reactionDist) {
            brake = 0;  // Already inside the safety clearance + reaction distance -> stop advancing entirely
        } else if (dAhead < R) {
            const dEff = dAhead - standoff - reactionDist;  // effective stopping distance
            // Plan the safe speed with the conservative yopoAvoidBrakeDecel (NOT the physical-max
            // aDecel): the real velocity controller cannot decelerate at the physical maximum, so
            // planning with it overshoots. The lower value guarantees the (slower) real stop fits.
            const vSafe = Math.sqrt(2 * this.yopoAvoidBrakeDecel * dEff);
            const kinBrake = spdFwd > 1e-3 ? Math.min(1, vSafe / spdFwd) : 0;
            // Progressive soft brake: within yopoAvoidBrakeRange it scales the speed down
            // smoothly with distance and applies the yopoAvoidBrakeFloor floor -- it only
            // handles the comfortable "slower as you get closer" deceleration, while the physical
            // stop is handled by the kinematic brake kinBrake above (computed from the actually
            // reachable deceleration, so it can always stop).
            // Previously the soft brake reused repRange (20 m) with no floor: it started scaling
            // speed down from 20 m out, leaving only 0.44 at 10 m and 0.16 at 5 m, while in urban
            // scenes dAhead is often 8-20 m -> the cruise speed was permanently squeezed to
            // 30%-70% of the commanded speed (the main cause of the measured 1-4 m/s), and at
            // close range it degenerated to 0, stalling the drone in place. After decoupling,
            // close range is handled by the kinematic brake, so safety improves rather than
            // regressing.
            const softT = clamp(
                (dAhead - standoff * 2) / (brakeRange - standoff * 2),
                0, 1
            );
            const soft = this.yopoAvoidBrakeFloor +
                (1 - this.yopoAvoidBrakeFloor) * softT;
            brake = Math.min(kinBrake, soft);
        }

        const repMag = Math.hypot(repX, repZ);
        // Clamp the repulsion strength
        if (repMag > 1e-6) {
            const s = Math.min(1, this.yopoAvoidRepGain / repMag);
            repX *= s; repZ *= s;
        }

        // Tangential detour: compute a deterministic tangent from the "nearest obstacle direction
        // dMin" (steering around the nearest obstacle toward the goal side), without picking an
        // opening / falling back to the emptiest direction -- that would detour to the side and,
        // when the goal is blocked, wrongly choose "emptiest = the way it came" and turn back
        // ("steers around then goes back"). On top of that a direction hysteresis memory: if the
        // angle against the previous frame's tan exceeds 120 deg while that direction is still
        // clear, keep the previous frame, preventing the resultant from flipping when passing the
        // obstacle centre and causing back-and-forth detours.
        let tanX = 0, tanZ = 0;
        // Apply the tangential detour only when there really is an obstacle fairly close ahead.
        // The threshold is relaxed to repRange*0.8 ~= 16 m:
        // the previous 0.5 (~9 m) was too late -- it waited until the drone was right on top of
        // it, leaving too little lateral manoeuvring distance, so it often "tried to detour and
        // hit it anyway".
        // Starting at 16 m, together with the lateral speed budget below, the drone has ample
        // distance to complete the detour arc.
        // In corridors / tight spaces obstacles are mostly on the two sides, while straight ahead
        // (the goal direction = the corridor depth) is clear (large dAhead) -> no detour; rep
        // (outside the final phase) pushes it off the side walls to keep it centred, while in the
        // final phase the PD converges onto the centre line, so the drone flies straight into the
        // corridor.
        if (dMin < R && dAhead < repRange * 0.8) {
            // Find the nearest obstacle direction (the ray direction matching dMin)
            let mi = -1;
            for (let i = 0; i < dirs.length; i++) {
                const d = dists[i];
                if (!Number.isFinite(d) || d <= 0) continue;
                if (mi < 0 || d < dists[mi]) mi = i;
            }
            if (mi >= 0) {
                const ox = dirs[mi].x, oz = dirs[mi].z;   // Pointing at the nearest obstacle
                // Two tangential candidates (perpendicular to the obstacle direction): pick the
                // side with the larger projection toward the goal (desired velocity);
                // when the goal is directly behind the obstacle both candidates are ~0, so either
                // side keeps the detour going (no turning back down the way it came).
                const tx1 = -oz, tz1 = ox;
                const tx2 = oz, tz2 = -ox;
                const c1 = tx1 * udx + tz1 * udz;
                const c2 = tx2 * udx + tz2 * udz;
                let fx, fz;
                if (c1 >= c2) { fx = tx1; fz = tz1; } else { fx = tx2; fz = tz2; }
                const t = this.yopoAvoidTanGain * (1 - dMin / repRange);
                fx *= t; fz *= t;
                // Direction hysteresis memory: if the angle against the previous frame's tan
                // exceeds 120 deg while that direction is still clear, keep the previous frame
                const lt = this._avoidLastTan || null;
                if (lt) {
                    const lm = Math.hypot(lt.x, lt.z), nm = Math.hypot(fx, fz);
                    if (lm > 1e-3 && nm > 1e-3) {
                        const cos = (fx * lt.x + fz * lt.z) / (nm * lm);
                        let lastOk = false;
                        for (let i = 0; i < dirs.length; i++) {
                            const lnx = lt.x / lm, lnz = lt.z / lm; // Previous-frame direction (normalised)
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

        // ---- Exit recognition: the straight corridor toward the goal is clear -> fly straight,
        // without being disturbed by detours / avoidance ----
        // Clearance is judged with a "flight corridor": the corridor is a band whose centre line
        // is the "body -> navigation goal" bearing and whose half-width is pathHalfWidth; only
        // obstacles inside that band (the measured perpendicular offset from the centre line is
        // below the half-width) count as "blocking the way".
        // Side obstacles outside the corridor (however close) are ignored -- a straight flight
        // passes them safely. Vertical threats (groundGap etc.) never take part in the corridor
        // decision; vertical safety is handled separately by upPush/vSafeDown.
        // Decision axis: the goal bearing (body -> navigation goal) is primary, but goalClear
        // additionally requires "the corridor along the commanded velocity direction is clear too"
        // (see the dual-corridor check below). Using only the goal bearing would miss "the network
        // points the drone at a building while the goal line is momentarily clear" -> it would
        // still charge at full speed (i.e. "planning straight into a building"); using only the
        // commanded velocity direction (the previous version) means that at the instant of a turn
        // / detour velTarget points into a side building whose lateral offset is actually large ->
        // it gets misjudged as "blocked" -> goalClear breaks -> continuous tangential detour
        // ("still detouring although the way to the goal is clear"). So both use the corridor
        // measure (lateral offset < half-width) instead of the cone's minimum distance: only near
        // obstacles that are really on the path count as blocking, while side / turn-mispointed
        // large-offset obstacles are ignored -> it neither charges blindly into buildings nor
        // wanders when the way is clear. With no navigation goal, the commanded velocity direction
        // falls back to the goal bearing.
        // gx/gz (goal bearing) were computed at the top of the function and are shared by dAheadH
        // and this corridor check.
        let goalClear = false;
        if (des > 0.3 || this.yopoNavTarget) {
            const pathHalfWidth = 2.5;                      // m, flight corridor half-width (body radius + margin)
            // Deliberately the fixed base value, NOT the speed-adaptive repRange: widening the
            // action range at speed must not also make the "corridor is clear" verdict stricter,
            // otherwise the drone would start detouring on an actually clear path.
            const clearThresh = this.yopoAvoidRepRange;     // The corridor counts as clear only when there is no near obstacle in it (> the action range)
            // Dual-corridor check:
            //   dPath — corridor clearance along the "body -> goal" bearing (is the way to the goal clear)
            //   dCmd  — corridor clearance along the "commanded velocity direction" (the actual
            //           heading) (is the surface the network/trajectory points at clear)
            // Only when both are clear does it release avoidance and fly straight. Using only the
            // goal corridor would miss "the network points the drone at a building while the goal
            // line is momentarily clear" -> it would still charge at full speed (i.e. "planning
            // straight into a building"); checking both rules that out. Use the corridor (lateral
            // offset < half-width) rather than the dAhead cone minimum, so a turn that briefly
            // points the velocity into a side building (large lateral offset) is not misjudged as
            // blocked -> continuous detour (the root cause of the earlier "still detouring
            // although the way to the goal is clear").
            let dPath = R, dCmd = R;
            const cx = udx, cz = udz, cMag = Math.hypot(cx, cz);
            const cxn = cMag > 0.3 ? cx / cMag : gx;        // Fall back to the goal bearing when there is no valid forward speed
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

        // ---- Vertical obstacle clearing (A) ----
        // When forward progress is strongly blocked (dAhead small) and one vertical side is
        // clear, actively climb / descend over the intervening obstacle.
        // Along the rays best aligned with the forward direction, compare the mid/high/high2/low
        // layer distances: if any upper layer is clear -> fly over, otherwise if the lower one is
        // clear -> dive under. The probe refreshes dynamically with altitude, so the drone keeps
        // climbing until it clears the obstacle top.
        let vRep = 0;
        const blockDist = this.yopoAvoidStop + this.yopoAvoidVBlock;
        // Vertical clearing only triggers when "the horizontal corridor toward the goal really is
        // blocked": a dual decision using !goalClear (there is an obstacle in the forward corridor)
        // and a short horizontal forward distance (dAheadH < blockDist). dAhead includes vertical
        // threats (groundGap etc., used for braking / push-up), so using it directly would shrink
        // dAhead -> "flying over / diving under by mistake although the way to the goal is clear".
        // Switching to the purely horizontal dAheadH plus the corridor check means it only climbs /
        // dives when there really is a horizontal obstacle ahead (rather than insufficient
        // clearance below).
        // Vertical clearing is fully disabled inside the final-approach zone (within
        // yopoFinalApproachDist of the goal or already arrived): there it should converge straight
        // onto the goal with the PD, and climbing would deviate from the goal and produce "taking
        // off although the way is clear" (the dual corridor / goalClear can break during final
        // trimming when the velocity direction points at a side building). Vertical safety
        // (upPush/vSafeDown) is still kept to prevent ground / ceiling hits.
        const nt = this.yopoNavTarget;
        const nearGoal = nt && (this.yopoArrived ||
            Math.hypot(nt.x - this.x, nt.z - this.z) < this.yopoFinalApproachDist);
        if (!nearGoal && !goalClear && dAheadH < blockDist && des > 0.3 &&
            p.distsHigh && p.distsHigh2 && p.distsLow) {
            // Among the directions that got a high-layer probe (highProbeIdx), if the upper layer
            // is clear in ANY direction inside the "forward hemisphere" it counts as flyable --
            // not just the best-aligned direction, so "straight ahead is blocked but a gap
            // slightly to the side can be flown over" is not missed (raises clearing willingness /
            // success rate).
            const clearD = R * this.yopoAvoidVClear; // A layer distance above this value counts as clear, i.e. flyable
            const hiIdx = p.highProbeIdx || null;
            let upClear = false, downClear = false;
            for (let i = 0; i < dirs.length; i++) {
                if (hiIdx && hiIdx.indexOf(i) < 0) continue; // Only directions probed at the high layer
                if (dirs[i].x * udx + dirs[i].z * udz < 0.3) continue; // Forward hemisphere only
                const dH = p.distsHigh[i], dH2 = p.distsHigh2[i], dL = p.distsLow[i];
                // Either upper horizontal layer is clear and straight up is clear too -> can fly over
                if (((dH > clearD) || (dH2 > clearD)) && (p.vUpDist > clearD)) upClear = true;
                // Diving under is more conservative: the low layer is clear and the clearance
                // straight below is sufficient, preventing a dive into an unprobed obstacle below
                if ((p.lowOk === true) && (dL > clearD) &&
                    Number.isFinite(p.groundGap) && p.groundGap > this.yopoMinAlt &&
                    Number.isFinite(p.vDownDist) && p.vDownDist > this.yopoMinAlt) downClear = true;
            }
            const e = this.yopoAvoidGain * this.yopoAvoidVClimbScale;
            if (upClear && downClear) vRep = e;       // Both available -> prefer climbing (safer)
            else if (upClear) vRep = e;               // Fly over
            else if (downClear) vRep = -e;            // Dive under (clearance confirmed sufficient)
        }

        // The potential field only handles "stop without hitting", it does not keep pushing away:
        // it is modulated by the "nearest obstacle distance dMin" (any direction) rather than by
        // dAhead, which points at the goal -- when detouring around the side of an obstacle dAhead
        // is small (the goal is behind the obstacle) but dMin is still near, so the push-away /
        // detour stays at full strength and firmly carries the drone past the end of the obstacle
        // without letting go halfway and being pulled back by the goal attraction (fixes "steers
        // around then goes back"); it only goes to zero when really glued to the obstacle
        // (dMin <= standoff) (once stopped it does not push back), and combined with rep decaying
        // with distance w it naturally weakens once away, never pushing too far.
        const repHold = clamp(dMin / standoff, 0, 1);
        repX *= repHold; repZ *= repHold;
        tanX *= repHold; tanZ *= repHold;

        // Completely release the horizontal repulsion / tangential / braking when the exit is
        // clear, flying straight at the goal:
        // this is the cure for "always pushed away although there is no obstacle" -- as long as
        // the horizontal passage toward the goal has ample clearance (dg > clearThresh and the
        // neighbouring rays / forward cone are free), fly straight at full speed without stacking
        // any rep/tan/brake.
        // Note: it is only released when there really is an intent to advance (des > 0.3) -- while
        // hovering, rep/tan are kept to maintain a safe distance from obstacles, so it is not
        // misjudged as "clear" and drifts into a wall. Vertical safety (upPush/vSafeDown) always
        // applies and does not interfere with straight horizontal flight.
        if (goalClear && des > 0.3) {
            repX = 0; repZ = 0;          // Horizontal repulsion fully zeroed (no 15% residual push left)
            tanX = 0; tanZ = 0;          // Tangential removed entirely (avoids detouring back to the start)
            brake = 1.0;                 // Clear exit means full speed, not slowed by vertical threats
            vRep = 0;                    // Vertical clearing released too: a clear corridor means no climbing / diving
        }

        // ---- Horizontal detour around vertical obstacles (B) ----
        // When there is a "building / structure" straight below (vDownDist small and clearly above
        // the ground, i.e. not hugging terrain) or an obstacle straight above (vUpDist small), do
        // not apply down/up motion to "squeeze through"; instead hold altitude and use the
        // horizontal detour (vGo) to leave the obstacle footprint above/below smoothly, avoiding
        // the "wants to descend -> pushed away by rays/collision -> wants to descend again"
        // oscillation. Vertical clearing (vRep) targets "blocked horizontally straight ahead with
        // a gap above/below"; this targets "blocked straight below / above" -- the only safe path
        // is a horizontal detour. Not enabled inside nearGoal (handed to the PD convergence).
        let vGoX = 0, vGoZ = 0;
        const vGoThresh = this.yopoAvoidStop + 3.0;   // ~4.1 m: a near obstacle underfoot / overhead counts as blocking
                                                      // Do not raise it: when flying over a rooftop at altitude the
                                                      // "straight below" ray necessarily hits the building, so a
                                                      // larger threshold causes pointless lateral detours while "the
                                                      // way to the goal is clear".
                                                      // vGo is now gated by goalClear (see the condition below): no
                                                      // lateral push while the corridor is clear.
        const gg = Number.isFinite(p.groundGap) ? p.groundGap : R;
        // Straight below is a "structure, not terrain": the straight-down hit is far above the
        // ground -> it is a building / overhang rather than hugging terrain. Terrain hugging the
        // ground (no building) still goes through upPush/vSafeDown normally and is not intercepted
        // here to forbid descending (otherwise low-altitude flight could never land).
        const structBelow = Number.isFinite(p.vDownDist) && p.vDownDist < vGoThresh &&
            (gg - p.vDownDist > 1.5);
        const aboveBlocked = Number.isFinite(p.vUpDist) && p.vUpDist < vGoThresh;
        // Key fix: vGo is gated by goalClear -- the horizontal detour around a vertical obstacle
        // footprint only happens when "the horizontal passage toward the goal is not clear".
        // Previously vGo was not constrained by goalClear, so when the drone was above a rooftop
        // (straight-down hit the building -> structBelow) while the path to the goal was actually
        // clear, it was still forcefully pushed sideways, showing up as "inexplicable detour
        // during a clear straight flight". When the corridor is clear the flight altitude is high
        // enough and there is no need to leave the rooftop footprint, so flying straight is fine;
        // vertical safety (upPush/vSafeDown) always applies to prevent ground / ceiling hits.
        if ((structBelow || aboveBlocked) && !nearGoal && !goalClear) {
            // Pick the emptiest horizontal direction to leave the obstacle footprint: prefer "the
            // emptiest in the forward hemisphere", otherwise use the globally emptiest (openDir),
            // so the detour still advances toward the goal and does not turn back.
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
