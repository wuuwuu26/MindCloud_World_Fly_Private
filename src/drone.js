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

        // 58 -> 60 deg. THIS is the real ceiling on braking: the achievable horizontal
        // deceleration is G*tan(maxAngle), i.e. 15.7 m/s^2 at 58 deg and 17.0 at 60 deg, and
        // yopoAvoidBrakeAccel already used ~99% of the 58 deg figure -- so however high that
        // parameter is set, braking could never exceed 15.7 until this was raised.
        // 60 deg is the thrust-to-weight limit of this airframe (mass 500 g / maxThrust 1000 gf
        // -> acos(mass/maxThrust) = 60 deg), so it is the last of the available authority.
        // Normal flight is unaffected: it is capped by yopoAccMax (11 m/s^2 ~= 48 deg of tilt).
        // If the drone visibly sinks while braking, drop back to 59 or 58.
        this.droneMaxAngle   = 60;
        this.droneAngleRate  = 280;
        this.droneMaxVSpeed  = 15.0;          // Hard vertical speed cap (m/s): kept at 15 per request. The
                                              // obstacle-clearing climb/descent rate is instead raised via the
                                              // vSafeUp / vSafeDown caps (see yopoAvoidVDecel), which stay subordinate
                                              // to this hard clamp so absolute vertical speed never exceeds 15.
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
        // Final-approach takeover distance (m): how far out the client PD takes navigation over
        // from the YOPO network. This zone MUST stay enabled -- it is NOT about slowing down.
        // Inside goal_length (2*radio_range = 10 m) the network's goal observation is squeezed by
        // normalisation and the lattice only holds cruise-type trajectories, so near the goal
        // argmin(score) keeps picking overshoot / turn-back trajectories -> position and velocity
        // oscillate and the goal is never reached. The hold PD taking over is what fixes that;
        // setting this to 0 hands the final metres back to the network and the navigation goes
        // inaccurate again (measured).
        // SET BACK TO 12 (it had been raised to 20). 12 m already covers the region where the
        // network degenerates (goal_length = 2*radio_range = 10 m). Inside the zone the takeover
        // runs on RAY BRAKING ALONE (tan / vGo / vRep / upPush are all off), so a larger zone
        // means a longer stretch flown with no detour and no clearing at all -- which measured as
        // "navigation inaccurate near the goal" whenever the goal sits against a building.
        // Shrinking it costs no speed: holdMaxV = sqrt(2*holdDecel*d) sits above the 15 m/s
        // ceiling beyond ~12.5 m, so the cruise still reaches the boundary at full speed and the
        // ramp only bites inside the last ~12 m.
        // (removed) yopoFinalApproachDist was the radius of the old 12 m final-approach takeover
        // zone. There is no takeover zone any more (see _controlYOPO).
        // Minimum cruise-speed floor toward the goal (m/s): the YOPO network frequently parks its
        // local waypoint (cmdPos ~= drone, cmdVel ~= 0) even when the global goal is tens of metres
        // away, which collapses velTarget to a crawl (~1-2 m/s) and the drone barely progresses.
        // When the ray-avoidance layer is NOT actively braking/gating (i.e. the path is clear) and a
        // live, distant nav target exists, this guarantees at least yopoCruiseMinSpd of forward
        // progress along the goal bearing, so the drone keeps cruising instead of crawling. Avoidance
        // always wins: if braking/gated is true this floor is skipped entirely, so an obstacle still
        // stops the drone. Only the projection of velTarget onto the goal direction is topped up, so a
        // healthy network command (already fast) is never throttled down to this value.
        this.yopoCruiseMinSpd = 12.0;
        // Detour speed floor (m/s): while a horizontal avoidance detour (repulsion + tangential) is
        // actually in play, the lateral escape budget is raised to this floor instead of the plain
        // cruise floor. This is what makes going AROUND an obstacle as decisive as going OVER/UNDER
        // it (vRep ~ 22 m/s): previously the detour was capped at 0.72 * yopoCruiseMinSpd ~= 8.6 m/s,
        // ~2.5x weaker than the vertical escape, so the drone "slowed down but did not get around".
        // Raising it to ~22 m/s lateral gives the detour the same get-out-of-the-way authority. The
        // toward-obstacle component is STILL capped by the proximity governor, and the forward brake /
        // closing gate stay on, so the drone cannot charge into the wall -- only the slide-around
        // speed is freed up. Lower this (e.g. 16) if fast detours clip wall corners; raise toward 28
        // to match vRep exactly.
        this.yopoDetourSpeedFloor = 28.0;
        // Fraction of the budget the lateral detour (rep + tan) may consume. Kept at the measured-safe
        // 0.72 (0.75 measured as "detouring too fast"); the higher get-around speed comes from the
        // raised budget floor above, not from loosening this share.
        this.yopoSteerCapFrac = 0.72;
        // Minimum share of the (unraised) forward budget kept as a forward floor while a lateral detour
        // is in play. With an obstacle ahead the forward component is squeezed to this fraction, so the
        // drone slides around at the detour speed instead of driving at the obstacle -- this is the
        // "obstacle ahead -> do not command a big forward speed, keep the distance" knob. Lower it
        // (e.g. 0.05) to hug the obstacle less / brake harder; raise it (e.g. 0.2) only if the drone
        // stalls in front of obstacles instead of going around.
        this.yopoFwdFloorFrac = 0.10;
        // Below this distance to the nav target the cruise floor is disabled, so the final-approach
        // PD convergence and the network's own slow-down near the goal are respected.
        this.yopoCruiseMinDist = 5.0;
        // Distance within which the potential field drops its normal-direction REPULSION (m).
        // Pinned at 12 m: within the last 12 m to the goal the normal repulsion stays off so the
        // drone can converge onto a goal that sits against a building. Obstacle protection inside
        // the zone comes from the ray brake + tangential detour + vertical safety (vSafeDown /
        // vSafeUp / upPush) + collision handling. The tangential detour is intentionally kept on
        // right up to the goal so the drone can still slide around the side of the building.
        // Trade-off: without normal repulsion there is no head-on push-away. If the run-in starts
        // grazing obstacles head-on, reduce this value (e.g. 8 m) so repulsion re-engages sooner.
        // (removed) yopoGoalRepSuppressDist / yopoFinalApproachVMax belonged to the old 12 m
        // final-approach takeover zone, which no longer exists (see _controlYOPO).
        // Slew-rate ceiling for the velocity target near the goal (m/s^2): how fast the commanded
        // velocity may CHANGE. holdKp is stiff, so any high-frequency wobble in its inputs (ray
        // brake toggling, probe noise, replan jumps) becomes a step in the target -> an
        // acceleration step -> attitude overshoot -> sway. This caps the CHANGE while leaving the
        // steady-state speed untouched. Keep it above the airframe's real acceleration (~11-17
        // m/s^2) so legitimate braking is never slowed down; it only bites on frame-to-frame steps.
        // LOWERED 20 -> 14 (settle-faster request): 14 m/s^2 is still above the airframe's real
        // acceleration ceiling (yopoAccMax = 11), so no genuine manoeuvre is ever throttled --
        // the cap only bites on frame-to-frame STEPS, which are exactly what makes the airframe
        // overshoot in attitude and sway at the goal. A tighter cap = fewer steps get through =
        // a calmer, quicker settle.
        // (removed) yopoTakeoverSlew was the velocity-target slew cap of the old takeover zone.
        this.yopoArriveHoldM = 4.0;        // Client-side arrival lock distance threshold (m). DISTANCE-ONLY
                                          // per request: the speed gate (yopoArriveHoldV) was removed, because
                                          // inside the region where the network degenerates (goal_length =
                                          // 2*radio_range = 10 m) the drone can dither a few metres short and
                                          // never slow below it -- so it never handed over to the hold PD and
                                          // never arrived. Latching on distance alone makes the handover
                                          // unconditional: < 4 m -> arrived -> the hold PD converges the rest.
        // (removed) yopoArriveHoldV was the speed gate of the arrival lock.
        // (removed) yopoArriveTakeoverM / yopoArriveStallSec / yopoArriveProgressEps / _arriveStallT /
        // _arriveBestD were the stall-based takeover backstop, dropped per request -- the
        // distance-only arrival lock above covers the same case unconditionally.
        // (removed) yopoArriveDeadbandM / yopoArriveVertH / yopoArriveAltKp / yopoArriveAltVMax
        // were all part of the old final-approach takeover (deadband, straight-vertical mode and
        // the in-deadband altitude trim). The reference-style arrival hold is a single PD on all
        // three axes and needs none of them (see _controlYOPO).
        // ── Cruise-phase "vertical first" (straight ascend / descend) ──
        // Outside the 12 m takeover zone the height change is entirely the network's job, and the
        // lattice only holds cruise-shaped arcs with a small vertical component -- so a goal that
        // differs mostly in HEIGHT is traded for a wide spiral. The cruise speed floor
        // (yopoCruiseMinSpd) makes it worse: it tops the horizontal speed up to 12 m/s ALONG THE
        // HORIZONTAL GOAL BEARING, so while the drone is still 30 m below the goal it is pushed
        // straight past the goal column and has to come back around -- the big circling seen during
        // a climb / descent.
        // When the height error clearly dominates the horizontal offset, the vertical channel is
        // taken over by a direct P climb / descent and the horizontal command is dialled down, so
        // the drone goes straight up / down onto the goal altitude.
        this.yopoVertFirstEnabled = true;
        // ENGAGEMENT GATE -- widened, because the gate was why the descent stayed at the network's own
        // ~8 m/s: while it is closed the client never overrides the vertical channel at all, so
        // yopoVertFirstVMax / yopoVertFirstDecel / the descent standoff have NO effect (observed: the
        // descent speed did not move across all three). The old gate (vfH < 20 AND |dy| > 1.2*vfH)
        // only opened almost directly above the goal column, so most descents never took over.
        this.yopoVertFirstHDist = 35.0;   // Max horizontal distance to the goal for this mode (m):
                                          // 20 -> 35 so a descent starts being client-driven earlier.
                                          // Wider = straight altitude changes start earlier (and the
                                          // cruise floor stands down over a wider area), narrower =
                                          // the network keeps cruising longer first
        this.yopoVertFirstMinDY = 4.0;    // Min |height error| to engage (m); released below 0.6x of it
        this.yopoVertFirstRatio = 0.9;    // The height error must also exceed this multiple of the
                                          // horizontal offset, so a far-away goal keeps cruising.
                                          // 1.2 -> 0.9: the old value required the drone to be nearly
                                          // overhead before the descent override opened at all.
        this.yopoVertFirstKp = 1.5;       // P gain on the height error -> climb / descent speed
        this.yopoVertFirstVMax = 10.0;    // Hard cap on that speed (m/s): 6.0 -> 10.0. This is the climb /
                                          // descent speed held while going straight up / down onto the goal
                                          // altitude (kept well under droneMaxVSpeed = 15). Raise it for a
                                          // faster altitude change, lower it if the vertical motion feels
                                          // abrupt. The actual rate is still min()'d with the sqrt(2*a*d)
                                          // arrival ramp and with the ray layer's vSafeUp / vSafeDown, so a
                                          // faster value cannot overshoot the goal or dive through an obstacle.
        this.yopoVertFirstDecel = 5.0;    // Assumed deceleration for the sqrt(2*a*d) arrival ramp (m/s^2).
                                          // RAISED 3.0 -> 5.0: at 3.0 the ramp only allowed sqrt(2*3*10) ~= 7.7 m/s
                                          // at a 10 m height error, so the descent was stuck at ~8 m/s and the
                                          // yopoVertFirstVMax = 10 ceiling was never reached (it needed ~16.7 m of
                                          // height error). At 5.0 the ramp permits the full 10 m/s from ~10 m of
                                          // height error up, easing off near the goal (7.1 at 5 m, 4.5 at 2 m).
                                          // The real vertical deceleration is ~10 m/s^2 (yopoAccMax = 11 with lag),
                                          // so planning with 5.0 keeps a large stopping margin -- the sqrt(2ad)
                                          // ramp still guarantees no overshoot of the goal altitude.
        // While a direct climb / descent is running, the horizontal command is normally scaled down so
        // the drone creeps onto the goal column. If the ray layer is actively repelling / detouring by
        // more than this (m/s of repulsion + tangential), the scaling is dropped entirely and the full
        // horizontal command is kept -- otherwise the lateral escape is crippled to 30% and the drone
        // climbs / descends straight into the obstacle it is supposed to be avoiding.
        this.yopoVertFirstRayThreshold = 2.0;
        // Weight of the VERTICAL LOOK-AHEAD repulsion: the horizontal ring only probes at the drone's
        // current altitude, so an obstacle it is about to descend / climb into (below / above / to the
        // lower side) is invisible to the ring. This projects the probe layer the drone is moving into
        // (low layer while descending, high layer while climbing) into the lateral repulsion, so it
        // shifts sideways before arriving. 1.0 = as strong as a ring obstacle; lower it if the drone
        // drifts sideways too eagerly while changing altitude.
        this.yopoVertLookWeight = 0.9;
        this.yopoVertFirstHScale = 0.3;   // Fraction of the horizontal command kept: it still creeps
                                          // onto the goal column instead of hovering in place
        this.yopoVertFirstMinV = 1.0;     // If the clearance only allows less than this (m/s), stand
                                          // down: something is straight above / below, so a straight
                                          // climb / descent is not possible and freezing in place would
                                          // be worse than letting the network spiral
        this._vertFirstOn = false;        // Engage/release hysteresis latch
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
        this.yopoCrashFloor = 1.5;    // Hard ground safety floor (m): below this clearance a climb is forced, preventing blind descent into the ground.
                                      // RAISED 1.0 -> 1.5 (keep-further-from-obstacles-below request): the last-resort push-away now
                                      // triggers 0.5 m higher, so the drone never skims the ground / rooftops.
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
        // Velocity-loop gain while tracking the YOPO trajectory (acceleration feed-forward on).
        // The original 2.2 needed a ~5 m/s velocity error before it commanded the full
        // acceleration budget, so the drone trailed its own target and never reached the 15 m/s
        // ceiling. 4.0 measured fastest. 6.0 was tried and is SLOWER in practice: it saturates at
        // only a ~1.8 m/s error but starts fighting the acceleration feed-forward and twitches
        // (vertically too, since both share this gain) -- that costs more speed than the earlier
        // saturation gains. Kept at 4.0.
        this.yopoVelKp = 4.0;
        // Trajectory-tracking position-loop gains (cruise, outside the final-approach zone).
        // NOTE: these work AGAINST cruise speed, not for it. The position loop pulls the drone
        // back onto the commanded position, so whenever the drone runs AHEAD of cmdPos -- which
        // the velocity feed-forward constantly makes it do -- a larger gain brakes harder.
        // Raising these 1.0 -> 1.5 was measured as slower; back at 1.0.
        // To cruise faster, LOWER them instead (e.g. 0.7): tracking then leans on the feed-forward
        // and the residual pull-back stops eating the cruise speed. Collision safety is unaffected
        // because the ray-avoidance layer is completely independent of cmdPos.
        this.yopoPosKp = 0.8;   // lowered 1.0 -> 0.8: weaker pull-back braking -> higher cruise speed
        this.yopoAltKp = 2.0;   // RAISED 1.2 -> 2.0: the vertical error now contributes nearly twice
                                // the speed, so residual height errors are closed much faster. The
                                // vertical position error is dominated by the network trajectory, so
                                // this mainly speeds up correcting whatever the feed-forward leaves.
                                // Lower it if the altitude starts oscillating.
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
        this.yopoAvoidRayCount = 12;       // Number of 360 deg rays (30 deg spacing); denser = more ray cost, lower this if the frame rate stutters
        this.yopoAvoidRays = (() => {
            const arr = [], N = this.yopoAvoidRayCount;
            for (let i = 0; i < N; i++) {
                const a = (i * 2 * Math.PI) / N;   // Equiangular on the horizontal plane, covering the full 360 deg
                arr.push({ x: Math.cos(a), y: 0, z: Math.sin(a) });
            }
            return arr;
        })();
        this.yopoAvoidRange = 65.0;   // Obstacle detection radius (m) -- RAISED 55 -> 65: a high-speed cruise needs a
                                      // longer look-ahead: the braking distance v^2/2a is ~13.5 m at
                                      // 15 m/s, and 42 m gives enough margin for detection lag plus
                                      // response, so obstacles are sensed earlier. Ray length is free
                                      // (it does not add GPU cost), so extending it only helps.
        this.yopoAvoidRepRange = 28.0; // Repulsion / tangential / braking range (m): widened so it pushes off earlier
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
        this.yopoAvoidRepRangeHi = 60.0;   // Repulsion / tangential / brake range at yopoAvoidRefSpeed (m): RAISED 50 -> 60 for more high-speed look-ahead
        // Side-push range + desired side standoff: DECOUPLED from yopoAvoidRepRange (which also
        // gates goalClear, so it must stay ~20 m to avoid declaring "corridor blocked" on clear
        // paths). The side repulsion below uses this wider range with a keep-out-shaped weight so
        // the drone holds ~yopoAvoidSideStandoff off building faces (keep-larger-side-distance
        // request) and is pushed off from further out (push-early request), without making the
        // "way ahead is clear" verdict stricter.
        this.yopoAvoidSideStandoff = 10.0;  // Desired side clearance from walls / building faces (m). REVERTED 13.0 -> 10.0: widening the full-strength keep-out band to 13 m made the lateral repulsion strong enough to push a detour BACK once it came abreast of the obstacle ("there is an exit but it turns around and goes back"), and it did nothing for the remaining collisions -- those are missed detections, where no distance is measured at all and a standoff cannot help.
        this.yopoAvoidPushRange = 36.0;     // Side-push detection range (m) at low speed
        this.yopoAvoidPushRangeHi = 70.0;   // Side-push detection range (m) at yopoAvoidRefSpeed: RAISED 56 -> 70
        this.yopoAvoidGain = 13.0;    // RAISED 10 -> 13. Generic avoidance gain base: now used mainly for vertical
                                      // safety (upPush/vRep = gain * factor); the horizontal
                                      // rep/tan have been split into separate gains below so they
                                      // can be tuned independently.
        this.yopoAvoidRepGain = 26.0; // RAISED 18 -> 20 -> 24 -> 26. Repulsion (radial push-away) max speed (m/s): raised for a more decisive push-off (highest-priority avoidance)
                                      // for a more decisive push/detour on contact
                                      // (together with the wider side pushRange + keep-out weight it reacts sooner
                                      // and holds further off building faces), instead
                                      // of just being "pushed back rather than steered around"
        this.yopoAvoidTanGain = 88.0; // RAISED 54 -> 68 -> 78 -> 88. Stronger lateral steer-around so the drone
                                      // commits to sliding past the obstacle instead of grazing it.
                                      // Still pairs with steerCap inside _controlYOPO (kept at/below
                                      // 0.72 of maxSpd) so the detour stays physically smooth; drop
                                      // back toward 54 if it feels wild.
                                      // Together with the "lateral speed budget reservation" inside
                                      // _controlYOPO (capped at 55%), the detour component is not
                                      // drowned by the forward component and the motion stays smooth.
        // Guards against the detour carrying the drone AROUND THE FAR SIDE of an obstacle instead of
        // past it (user report: "the way to the goal is clear, yet it goes around the other side of
        // the building"). Both act on the angle between the tangential direction and the goal
        // bearing:
        this.yopoTanConeCos = 0.17;   // Cosine of the cone (+-80 deg) around the goal bearing in
                                      // which an obstacle counts as "in the way" for the tangential
                                      // detour. Widened from 0.34 so obstacles slightly to the side
                                      // still trigger a steer-around instead of only a push-away.
                                      // Obstacles further to the side / behind no longer supply the
                                      // detour direction (they are handled by the repulsion instead).
                                      // -1 restores the old behaviour of taking the globally nearest
                                      // obstacle.
        this.yopoTanAwayCos = -0.2;   // The remembered tangent from the previous frame is only kept
                                      // while it still leads roughly toward the goal; below this
                                      // cosine (> ~100 deg off the bearing) the direction memory is
                                      // dropped and the turn-back toward the goal is allowed.
                                      // -1 restores the old unconditional memory.
        this.yopoTanAwayScale = 0.78; // Scale applied to a tangent that points more than 90 deg away
                                      // from the goal: it is no longer steering around the obstacle,
                                      // it is carrying the drone away, so the goal-directed terms
                                      // (trajectory / cruise floor) get the upper hand. 1.0 disables
                                      // the guard; raised from 0.5 so the detour keeps more authority
                                      // while rounding the obstacle before turning back.
        this.yopoAvoidDecel = 8.5;    // Retained for config compatibility. The active vertical kinematic
                                      // deceleration for vSafeUp / vSafeDown is now yopoAvoidVDecel (below); the
                                      // forward brake uses yopoAvoidBrakeDecel, so this value no longer gates any cap.
        // Vertical deceleration used ONLY for the vSafeUp / vSafeDown (and upPush) caps. The vertical thrust
        // axis brakes far harder than the forward tilt axis, so it can be set well above the forward
        // yopoAvoidDecel / yopoAccMax. Raising it makes the stoppable climb / dive rate -- i.e. vSafeUp and
        // vSafeDown -- larger, so the drone gains / loses altitude faster while still able to stop within the
        // measured overhead / under-foot clearance. Still subordinate to droneMaxVSpeed (15), so absolute
        // vertical speed is capped regardless. Lower this (e.g. 10) if fast climbs feel like they overshoot
        // the ceiling / dive into the ground; raise toward 16 for even snappier vertical clearing.
        this.yopoAvoidVDecel = 13.0;
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
        // A full ring probe used to cast 24 + 9 + 2 = 35 forceFresh scene.pickFromRay calls. Every
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
        // Obstacles lying BEYOND the navigation goal must not brake the final approach (see the
        // `beyondGoal` logic in _avoidanceVelocity): the drone only ever travels as far as the
        // goal, so the wall a goal sits against is not something to stop for. When the ray hits
        // something farther away than the goal, the clearance used by the brake is capped at
        // (distance to the goal + this margin) instead of the raw ray distance.
        this.yopoAvoidGoalMargin = 2.5;        // m of extra room past the goal
        // ...and in that case the brake may never go below this, so a beyond-goal obstacle can
        // only slow the run-in, never pin the drone metres short of the goal.
        this.yopoAvoidGoalBrakeFloor = 0.40;   // 0..1 floor applied for a beyond-goal obstacle
        // Tolerance (m) for the "threat is the goal itself" verdict. When the goal sits against
        // a wall the forward ray hits that wall at (almost exactly) the goal's horizontal
        // distance, so dAhead ~= distGoalH and a strict `dAhead > distGoalH` test misses it:
        // brakeClear then falls to ~distGoalH, which is inside standoff(7.5) + reactionDist ->
        // brake = 0 AND the closing-speed gate clips the goalward component to 0 -> the drone is
        // pinned metres short of the goal and takes forever to arrive ("takes ages to stop after takeover").
        // A threat within this margin of the goal is treated as beyond-goal: the brake keeps its
        // floor (never 0) and the gate stands down, leaving the stop to the final-approach PD,
        // whose holdMaxV = sqrt(2*a*d) already guarantees a physically stoppable run-in.
        this.yopoAvoidGoalGateMargin = 1.0;
        // (removed) yopoTakeoverSteerEndDist belonged to the old 12 m final-approach takeover:
        // the distance under which the ray steering stood down near the goal. There is no
        // takeover zone any more (see _controlYOPO), so the parameter is gone.
        // Probe TIERING REMOVED: every one of the yopoAvoidRayCount ring directions is now
        // re-probed EVERY cycle with forceFresh=true -- no pickLocalRay cache, no downsampling,
        // no mirrored neighbours and no round-robin slices. The parameters below used to drive
        // that tiering and are therefore no longer read by _computeAvoidProbe; they are kept so
        // any external / UI override of them stays harmless.
        this.yopoAvoidStrideHi = 2;      // (unused) high-speed ray stride: was every 2nd ray at speed
        this.yopoAvoidCoreDeg = 25;      // (unused) half-angle (deg) of the always-fresh core cone
        this.yopoAvoidConeDeg = 55;      // (unused) half-angle (deg) of the forward cone re-probed every cycle
        this.yopoAvoidConeDegHi = 55;    // (unused) forward cone half-angle at high speed
        this.yopoAvoidSliceMax = 12;     // (unused) max peripheral rays re-probed per cycle (round-robin)
        this.yopoAvoidVertEvery = 1;     // (unused) straight up/down rays are now probed EVERY cycle
        this._avoidRing = null;          // Ring distances from the last cycle (fully overwritten every cycle now)
        this._avoidRingAge = null;       // ms since each ring direction was probed (always 0 now)
        this._avoidSliceCursor = 0;      // (unused) round-robin cursor over the peripheral directions
        this._avoidCycle = 0;            // Probe cycle counter
        this._avoidPrevBlocked = false;  // (diagnostic only) previous cycle saw the forward corridor blocked
        this._avoidPerf = { probeMs: 0, rays: 0, rayTotal: 0, cycles: 0, ringAgeMax: 0 };
        this.yopoMinAlt = 8.0;        // Minimum ground/roof clearance (m) -- threshold that triggers soft avoidance (upward push)
                                      // RAISED 2.5 -> 3.0 -> 4.0 -> 8.0: the push-up now engages 5.5 m earlier. The binding
                                      // clearance when flying OVER a rooftop is vDownDist (the straight-down ray): once the gap
                                      // to the rooftop below drops under 8.0 m the drone is pushed up, so it keeps ~8 m of
                                      // vertical margin above rooftops instead of skimming them. (yopoAvoidStopDown = 8.0 then
                                      // forbids descending back into that same band.)

        this.yopoAvoidVertRay = true;     // Straight up/down vertical rays (prevents hitting the ceiling / an obstacle straight below)
        // Vertical ray detection range (m). RAISED 12 -> 30: this is the timeliness of BELOW avoidance.
        // An obstacle straight below (a rooftop / a lower building) was invisible beyond 12 m --
        // groundGap only covers the TERRAIN, not structures -- so at the now-faster 10 m/s descent the
        // drone got ~1.2 s of warning minus command lag and reacted too late. At 30 m it gets ~3 s, and
        // vSafeDown ramps the descent down from far out instead of a late hard stop. The cost is
        // negligible (two vertical rays per frame), and vSafeUp / vSafeDown are sqrt(2ad) caps, so a
        // longer detection range only makes them ease off EARLIER -- it cannot slow clear-air climbs.
        this.yopoAvoidVertRange = 30.0;
        // ── Vertical obstacle clearing (plan A+B) ──
        this.yopoAvoidVStep = 9.0;        // Vertical probe step up/down (m); *2 high layers can clear taller buildings (8 -> 9: probes slightly higher, clearing taller obstacles)
        this.yopoAvoidVClimbScale = 2.2;  // Vertical clearing speed = gain*scale = 13*2.2 = 28.6 m/s, a
                                          // fiercer, faster climb / dive over obstacle tops; clamped to 15 by
                                          // droneMaxVSpeed but full climb is commanded earlier, and with
                                          // yopoAccMax raised the vertical acceleration ceiling grows too, so the
                                          // climb builds faster. (Kept at 2.2 per request; the faster vertical
                                          // clear comes from the raised vSafeUp/vSafeDown caps below.)
        // ── vGo (plan B): lateral speed used to leave the footprint of an obstacle straight
        // below / above ──
        //   strength = yopoAvoidTanGain * (VGoBase + VGoSpan * (1 - closeness))
        // VGoBase is the speed when the obstacle has only just entered the threshold,
        // VGoBase+VGoSpan when it is right on top of the drone.
        // Raised from 0.30 / 0.20: with the 34 m/s tangential gain that was only 10.2-17 m/s,
        // which left the drone loitering over the rooftop it was trying to get off.
        // vGo is not limited by steerCap and is gated by goalClear, and the composed target is
        // still clamped to maxSpd -- so raising it makes the detour more decisive (vGo dominates
        // the direction) rather than faster than the 15 m/s ceiling.
        this.yopoAvoidVGoBase = 0.85;     // RAISED 0.60 -> 0.85. Higher base lateral speed when an
                                          // obstacle first enters the underfoot / overhead threshold.
        this.yopoAvoidVGoSpan = 0.60;     // RAISED 0.42 -> 0.60. Stronger peak lateral push when the
                                          // obstacle is right on top of the drone.
        // Dedicated, LARGER stopping deceleration used ONLY for the vGo lateral-escape cap (vGoSafe).
        // Lateral roll maneuvering can brake far harder than the forward dive brake
        // (yopoAvoidBrakeDecel, kept conservative at 7.5 for collision safety). Reusing that forward
        // value here was over-cautious and throttled vGo to ~2-3 m/s even in perfectly open air below
        // a rooftop -- the previous VGoBase/VGoSpan bumps were in fact no-ops because vGoMag is
        // min(strength, vGoSafe) and vGoSafe was the binding constraint. With this, vGo reaches a
        // usable escape speed (≈5 m/s at 3 m clearance, ≈9 m/s at 4 m, ≈11 m/s at 5 m) while still
        // never driving into the side obstacle measured along the escape direction.
        this.yopoAvoidVGoDecel = 34.0;
        this.yopoAvoidVBlock = 20.0;      // RAISED 16 -> 20. Forward clearance below this triggers vertical clearing (m):
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
        this.yopoAvoidStop = 6.0;     // Safety clearance (m): the distance the drone keeps off obstacles.
                                      // RAISED 2.5 -> 4.0 -> 6.0: holds further off walls / buildings per
                                      // request (still clipping obstacles with too little margin).
                                      // Drives the down/up clearance (2844/2862) and the vertical-
                                      // clearing block distance (3137).
                                      // Trade-off: a larger standoff shrinks dGate (= dAhead - standoff -
                                      // reactionDist), so vCloseMax drops and the drone is vetted to a
                                      // lower speed near obstacles. That is the intended direction here
                                      // (safety over cruise speed). Do NOT widen yopoAvoidRepRange to
                                      // compensate: it doubles as goalClear's clearThresh, so raising it
                                      // makes "corridor is clear" too strict and detours on open paths.
        // HORIZONTAL brake standoff (m), split from yopoAvoidStop. RAISED 6.0 -> 7.5 -> 9.0
        // (keep-further-away request): the horizontal kinematic brake now plans its stop 9.0 m off
        // the obstacle instead of 7.5 m, so the drone holds a wider margin from walls / buildings.
        // The VERTICAL UP standoff (vSafeUp above) deliberately KEEPS yopoAvoidStop = 6.0: raising it
        // would over-restrict climbing / over-head clearance. The DESCENT (below) standoff is the
        // separate yopoAvoidStopDown just below, which IS raised so "below" avoidance holds further off.
        // NOTE: the soft-brake zone (yopoAvoidBrakeRange) must stay comfortably above 2x this
        // value, otherwise the (brakeClear - standoff*2) normalisation degenerates and the
        // progressive soft brake stops shaping the approach at all -- BrakeRange was raised to
        // 30.0 in the same step for exactly this reason (see BrakeRange below).
        this.yopoAvoidStopH = 9.0;
        // VERTICAL DOWN standoff (m), SEPARATE from the shared yopoAvoidStop so that "keep further from
        // obstacles below" raises only the DESCENT safety margin and does NOT also forbid climbing /
        // over-head clearance (vSafeUp / vGo still use yopoAvoidStop = 6.0).
        // LOWERED 8.0 -> 5.0: this standoff is the binding term of vSafeDown =
        // sqrt(2*aDecel*(downGap - standoff)), i.e. it sets the descent speed for a given clearance
        // below. At 8.0 m the descent was pinned to ~8 m/s whenever the clearance below was ~11 m
        // (goals close to the ground), and no amount of tuning yopoVertFirstVMax / yopoVertFirstDecel
        // could change it -- the observed speed did not move when those were raised, which is what
        // identified this term as the binder. At 5.0 m the same clearance allows the full
        // yopoVertFirstVMax (10 m/s). The drone still holds a real 5 m floor above whatever is below
        // (vSafeDown = 0 below that), and the vertical look-ahead repulsion covers side obstacles, so
        // a fast descent is still stoppable within the clearance. Raise it back toward 8 if fast
        // descents feel too close to the ground / rooftops.
        // RAISED 5.0 -> 7.0 (crash fix): the vertical safety envelope had been eroded by stacking --
        // planned vertical decel 7.65 -> 9.9, descent speed 6 -> 10 AND this standoff 8 -> 5 -- which
        // left the descent at 10 m/s with only ~5 m of margin and a plan that assumes the velocity
        // loop delivers 9.9 m/s^2 of vertical braking on demand. Any lag there and the drone descends
        // into the obstacle below. 7.0 restores a real margin while still allowing the full 10 m/s
        // whenever the clearance below is >= ~13 m (sqrt(2*9.9*(13-7)) = 10.9), easing to ~9 m/s at
        // 11 m of clearance. This standoff IS the crash margin for the fast descent -- do not lower
        // it again without lowering the descent speed to match.
        this.yopoAvoidStopDown = 7.0;
        // Proximity speed governor standoff (m): a SECOND, independent speed limiter that caps the
        // drone's TOTAL horizontal speed by the nearest obstacle in ANY direction, not just the
        // forward one the brake watches. The brake above only throttles the dAhead (forward)
        // component, so when the planner commands a high speed while an obstacle sits close on the
        // side / at an angle, the overall speed is never constrained and the drone charges in too
        // fast to react ("close to an obstacle yet the plan commands high speed -> cannot avoid in
        // time"). This governor forces the speed down to a value proportional to the clearance past
        // this standoff, reaching the cruise ceiling only beyond ~24 m, so open flight is untouched
        // but a high planner speed next to a near building is always reined in.
        this.yopoAvoidNearStop = 6.0;     // RAISED 4.0 -> 6.0: this is the standoff the proximity governor
                                          // keeps from the nearest obstacle -- the speed toward it ramps from 0
                                          // at this distance, so the drone now holds ~6 m instead of ~4 m. It was
                                          // the loosest of the three clearances (side 10 / forward 7.5 / this 4),
                                          // which is why the drone still closed in on obstacles: at 10 m it was
                                          // allowed 0.75*(10-4) = 4.5 m/s toward them. Now 0.75*(10-6) = 3.0 m/s.
                                          // Pair this with the fwdAllow / yopoFwdFloorFrac squeeze above.
        this.yopoAvoidNearK = 0.75;   // Proximity speed ramp (m/s per m of clearance past yopoAvoidNearStop)
        this.yopoAvoidVGoThresh = 7.0; // Underfoot / overhead blocking threshold for the horizontal vGo
                                      // detour (m). Deliberately DECOUPLED from yopoAvoidStop (it used to
                                      // be yopoAvoidStop + 3.0, i.e. 7.0 at stop = 4.0) and pinned to that
                                      // value while standoff grows: when flying over a rooftop the
                                      // "straight below" ray necessarily hits the building, so a threshold
                                      // that rises with the standoff triggers pointless lateral detours
                                      // even though the path overhead is perfectly clear.
        // Corridor guard (m): goalClear can be released via the "commanded heading" corridor (dCmd)
        // even when the "bearing to the goal" corridor (dPath) contains an obstacle. That escape hatch
        // is needed (it stops a detour from self-locking), but it must never release when the goal
        // corridor is blocked CLOSE IN -- with brake = 1 and rep = 0 the drone then charges straight
        // into an obstacle only a couple of metres away on its own goal bearing. Inside this distance
        // the dCmd escape hatch is refused and goalClear stays false, so the ray brake / repulsion stay
        // armed. Sized so the drone can always stop: at the 12 m/s cruise floor with
        // yopoAvoidBrakeDecel 7.5 the stopping distance is v^2/(2a) ~= 9.6 m, so 12 m leaves margin.
        // Obstacles beyond the goal are excluded upstream (dd < reach <= distGoalH), so a goal sitting
        // against a wall still releases normally and the drone can still arrive.
        this.yopoCorridorGuardDist = 12.0;
        // Range (m) and floor of the "progressive soft brake": the soft brake only provides the
        // comfortable "slower as you get closer" deceleration; the physical stop is handled by the
        // kinematic brake (v_safe = sqrt(2ad)). Previously the soft brake reused repRange (20 m)
        // with no floor, so it started scaling speed down from 20 m out, leaving only 0.44 at 10 m
        // and 0.16 at 5 m -- in urban scenes dAhead is often 8-20 m, so the cruise speed was
        // permanently squeezed to 30%-70% of the commanded speed (the main cause of the 1-4 m/s the
        // user measured). After decoupling, it only engages within 12 m and never drops below 0.55;
        // close range is handled by the kinematic brake, which guarantees a real stop.
        this.yopoAvoidBrakeRange = 30.0;  // Soft-brake deceleration zone (m). RAISED 18 -> 24 -> 30 together
                                          // with yopoAvoidStopH 6 -> 7.5 -> 9.0: the soft zone is normalised as
                                          // (brakeClear - standoff*2) / (range - standoff*2), so with the
                                          // wider standoff the old 18 m, and then 24 m, left too narrow a
                                          // band and the "ease off as you get closer" shaping nearly vanished.
        this.yopoAvoidBrakeRangeHi = 54.0; // Soft-brake zone at yopoAvoidRefSpeed (m): RAISED 40 -> 54 so the drone starts easing off earlier at speed
                                          // needs to start easing off much earlier. Safe to widen because
                                          // the floor (0.85) caps how much it can ever slow down on its
                                          // own -- the physical stop is always the kinematic brake.
        this.yopoAvoidBrakeFloor = 0.85;  // Soft-brake floor: RAISED 0.55 -> 0.80 -> 0.85. The soft brake only
                                          // shapes the "ease off as you get closer" deceleration -- the
                                          // physical stop is the kinematic brake below, which is computed
                                          // from the actually reachable deceleration and can always stop.
                                          // At 0.55 the soft brake alone pinned the cruise to ~55% of the
                                          // commanded speed (the "only manages 3 m/s" symptom); 0.80 keeps
                                          // a gentle ease-off without throttling the whole flight.
        // Brake reaction time (s): the delay between issuing the deceleration command and it
        // physically taking effect (probe read-back + command link + attitude build-up). The drone
        // keeps advancing at the old speed during this window, so (spd * reaction) is subtracted from
        // the available stopping room.
        // 0.35 s was the measured link lag, but it is no longer the dominant term at speed: while
        // cruising fast the drone flies at a FORWARD tilt (~45 deg at 15 m/s), so before any
        // deceleration exists the attitude must first slew from +45 deg to the braking tilt (~-55 deg)
        // -- about 100 deg of rotation, which at the stock attitude gains costs ~0.28 s by itself.
        // Together with the probe lag the real dead time at 15 m/s is ~0.6 s, i.e. ~9 m of completely
        // un-braked glide -- which is precisely the "ray avoidance is too late when moving fast"
        // symptom. The base is therefore raised to 0.5 s and grows with speed (see
        // yopoAvoidBrakeReactionHi) so the brake is commanded early enough to absorb the slew.
        this.yopoAvoidBrakeReaction = 0.46;   // 0.50 -> 0.42 -> 0.32 -> 0.36 -> 0.46: RAISED for timely braking.
                                              // This is the dead-time reserve subtracted from the stopping room, so
                                              // a larger value makes the brake COMMANDED EARLIER -- the direct fix for
                                              // "braking is not timely enough". Every 0.1 s here is ~1.5 m of extra
                                              // lead at 15 m/s, so 0.36 -> 0.46 adds ~1.5 m of early braking.
                                              // loop at 60 Hz, so the transport lag is far below 0.3 s. The
                                              // angle-gain boost (2.2) also cut the tilt slew to ~0.12 s, so the
                                              // old buffer was double-counting the dead time. Every 0.1 s here
                                              // is ~1.5 m of stopping room at 15 m/s.
        // Reaction time at yopoAvoidRefSpeed (s): the faster it flies, the larger the forward tilt it
        // must shed before the deceleration bites, so the lead grows with speed instead of staying
        // constant. Interpolated with the same tFast used by the other high-speed profiles.
        this.yopoAvoidBrakeReactionHi = 0.80;   // 0.75 -> 0.60 -> 0.48 -> 0.60 -> 0.80. RAISED: the "sometimes
                                                // braking is not timely enough" cases are almost all at speed, where
                                                // the drone must first slew out of its forward tilt (~100 deg of
                                                // rotation) before any deceleration exists. Reserving 0.8 s of dead
                                                // time at cruise speed gives ~12 m of lead at 15 m/s (vs ~9 m at
                                                // 0.60), so the brake is commanded early enough to absorb the slew.
        // Planning deceleration used by the *horizontal* kinematic brake. This is deliberately far below
        // the physically reachable aDecel (7.2) -- the previous brake planned with aDecel and therefore
        // assumed the drone can always decelerate at the physical maximum, but the real velocity
        // controller lags, so the achieved deceleration is much lower and the plan overshot into walls
        // ("deceleration not fast / not timely enough"). Planning with ~3 m/s^2 leaves a ~2x margin over
        // a realistic controller decel, so the commanded target speed is always low enough that the
        // (slower) real deceleration still stops inside the standoff. The physical stop is guaranteed by
        // construction; this only trades a bit of early slowing for never crashing.
        this.yopoAvoidBrakeDecel = 6.5;   // 3.0 -> 4.5 -> 3.5 -> 5.0 -> 7.0 -> 7.5 -> 6.5: LOWERED back for timely
                                          // braking. Drives BOTH vSafe and the
                                          // closing-speed gate vCloseMax -- the single biggest speed lever,
                                          // and what pinned the cruise at ~11 m/s.
                                          // This is the deceleration the brake may PLAN with, and it must stay
                                          // at or below what the velocity loop can actually DELIVER -- which
                                          // lags well behind the 17.0 m/s^2 the airframe holds at max tilt.
                                          // At 9.5 the plan was optimistic: v_safe = sqrt(2*a*dEff) then cleared
                                          // the drone to keep a speed it could not actually shed in the distance
                                          // available, so it flew into obstacles it had already "decided" it
                                          // could stop for -- the repeated impacts.
                                          // LOWERED 7.5 -> 6.5 for "braking is still not timely enough
                                          // sometimes": planning with a smaller deceleration makes v_safe and
                                          // vCloseMax smaller, so the brake both engages earlier and demands
                                          // more, leaving extra margin for the cases where the real loop
                                          // delivers less than assumed (slew already in progress, replan gap).
                                          // Do NOT raise this for speed -- it buys speed by lying about the
                                          // stopping distance, and is paid for in collisions.
                                          // Still well under the real braking authority (17.0 m/s^2).
        // Maximum deceleration (m/s^2) the ray-avoidance layer is allowed to COMMAND when braking
        // hard. This is the authoritative, strong brake: while the kinematic *plan* uses the
        // conservative yopoAvoidBrakeDecel (3) so it can always stop with margin, the *actual*
        // deceleration is pushed up to near the physical tilt limit so the drone really does slam the
        // brakes instead of drifting into the wall ("deceleration not strong enough / still hitting
        // obstacles").
        // Raised 14 -> 15.5: the airframe here is mass=500 g / maxThrust=1000 gf, so the thrust a
        // 2:1 thrust-to-weight ratio can sustain caps the usable tilt at acos(mass/maxThrust) = 60 deg,
        // i.e. G*tan(60) = 17.0 m/s^2 of deceleration, while droneMaxAngle (58 deg) allows 15.7.
        // 15.5 therefore uses ~99% of what the airframe can actually hold without sinking, versus 89%
        // before -- it is the last bit of braking authority available without changing the airframe.
        // The passive collision handler remains the final backstop.
        this.yopoAvoidBrakeAccel = 17.0;   // 14 -> 15.5 -> 17.0, now matching G*tan(60 deg) = 17.0 --
                                          // the deceleration the airframe can actually hold at the
                                          // raised droneMaxAngle. Previously 15.5 was clipped to 15.7
                                          // by the 58 deg tilt limit, so raising it alone achieved
                                          // nothing; with 60 deg it is fully usable.
        // Minimum fraction of yopoAvoidBrakeAccel commanded as soon as the ray layer is braking.
        // The feed-forward used to be purely proportional (aB = brakeAccel * (1 - brake)), so a brake
        // that had only just begun to bite (brake ~0.9) commanded a token 1.5 m/s^2 and the drone kept
        // coasting at nearly full speed -- the "brakes, but far too gently" case. From now on entering
        // the braking state hands over at least this fraction of the authority immediately; the
        // proportional term still takes over as the obstacle closes.
        this.yopoAvoidBrakeMinFrac = 0.85;   // 0.55 -> 0.85: entering the braking state now hands over
                                             // 0.85 * 17.0 = 14.5 m/s^2 immediately instead of 8.5, so
                                             // the brake bites at once rather than ramping up gently
                                             // while the drone coasts on at nearly full speed.
        // Attitude-loop gain multiplier applied while braking. This attacks the dominant part of the
        // dead time: the ~0.28 s needed to slew from the cruise tilt to the braking tilt. With the
        // stock sfAngleKp=4.5 the rate target for a 100 deg error is ~450 deg/s and the rate loop
        // (sfRateKp=0.8, 40 ms smoothing) needs ~0.28 s end to end, during which the drone flies on
        // un-braked. Scaling the angle error by 2.2 shortens the slew further (~0.12 s), so the
        // 17.0 m/s^2 above is actually reached while there is still room to stop -- a higher commanded
        // deceleration is worthless if the tilt never gets there in time.
        this.yopoAvoidBrakeAngleGain = 2.2;
        // Brake value below which the attitude-gain boost above is allowed to engage. It must not be
        // keyed off `braking` alone: `braking` is true whenever brake < 0.95 and the soft brake now
        // covers 22 m, so in an urban scene it is true nearly all the time -- boosting the gain on
        // that would make normal flight twitchy and hurt the damping ratio (sfAngleKd is not scaled
        // with it). 0.7 means the boost is reserved for a real emergency stop, or for the
        // closing-speed gate firing, which is exactly when the extra ~0.12 s decides crash or not.
        this.yopoAvoidBrakeUrgent = 0.7;
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
        this.yopoFFVelYMax = 14.0;   // RAISED 8.0 -> 14.0: this clamp capped the vertical
                                     // feed-forward at 8 m/s, so climbs/descents could never exceed
                                     // it no matter what the trajectory asked for. Descent is still
                                     // limited by vSafeDown (from the measured clearance below) and
                                     // the total by droneMaxVSpeed (15), so this cannot cause a dive
                                     // into the ground. Drop to 10-12 if vertical motion gets jerky.
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

        // ---- 0b. Arrived -> position hold on the goal ----
        // Ported from MindCloud_World_Fly_With_Yopo (src/drone.js): that reference implementation
        // has NO distance-based takeover zone. It follows the YOPO network all the way in and
        // only switches to a position hold on the goal once arrival is latched. The previous
        // 12 m "final-approach takeover" PD here (distance-scheduled gains, a sqrt(2*a*d) speed
        // ceiling, slew limiting, boundary blending, a vertical deadband and an arrival lock)
        // was the source of the persistent end-game sway: it fought the network, the ray layer
        // and the velocity-measurement noise all at the same time.
        let distGoal = Number.POSITIVE_INFINITY;
        if (this.yopoNavTarget) {
            const gdx = this.yopoNavTarget.x - this.x;
            const gdy = this.yopoNavTarget.y - this.y;
            const gdz = this.yopoNavTarget.z - this.z;
            distGoal = Math.sqrt(gdx * gdx + gdy * gdy + gdz * gdz);
        }
        this.yopoDistToGoal = distGoal;

        // Client-side arrival lock (backstop): within yopoArriveHoldM of the goal -> treat as arrived.
        // DISTANCE-ONLY per request (the speed < yopoArriveHoldV condition was removed): inside the
        // region where the YOPO network degenerates (goal_length = 2*radio_range = 10 m, its goal
        // observation is squeezed by the normalisation) the drone can dither a few metres short and
        // never slow below the old speed gate, so it never handed over to the hold PD. Latching on
        // distance alone makes the handover unconditional.
        // The server's 2 m arrival verdict comes back asynchronously, and if the trajectory
        // lingers slightly outside the 2 m circle this backstop makes the client switch to
        // holding at the goal, avoiding "always one step short".
        if (this.yopoNavTarget && distGoal < this.yopoArriveHoldM) {
            this.yopoArrived = true;
        }

        const yopoArrivedHold = this.yopoArrived && this.yopoNavTarget && !stickActive;

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
        // Hand-over ramp state: reset whenever we are NOT in stick override, so the next takeover
        // captures a fresh "speed at takeover" instead of reusing a stale one.
        if (!stickActive) this._handoverActive = false;

        if (yopoArrivedHold) {
            // Arrived at the goal -> PD hover converging onto the goal point
            // (position P + velocity damping D).
            // Mirrors the SO3 behaviour kx*posErr + kv*(0 - vel): with des_vel = 0 the drone
            // decelerates on its own. A pure P term would still be carrying speed at the goal
            // -> overshoot -> pull back -> sway, which is why the D term is kept.
            // Ported from MindCloud_World_Fly_With_Yopo (src/drone.js).
            const gErrX = this.yopoNavTarget.x - this.x;
            const gErrZ = this.yopoNavTarget.z - this.z;
            const gErrY = this.yopoNavTarget.y - this.y;
            const holdKp = 1.5, holdAltKp = 2.5, holdKd = 1.5, holdMaxV = 2.0;
            velTargetX = holdKp * gErrX - holdKd * this.vx;
            velTargetZ = holdKp * gErrZ - holdKd * this.vz;
            velTargetY = holdAltKp * gErrY - holdKd * this.vy;
            const vh = Math.sqrt(velTargetX*velTargetX + velTargetZ*velTargetZ);
            if (vh > holdMaxV) { const s = holdMaxV / vh; velTargetX *= s; velTargetZ *= s; }
            // Vertical (descent) speed cap: prevents a fast dive that overshoots the goal or passes
            // through a side building while descending onto the goal. The ray layer's vSafeDown below
            // tightens this further to the measured clearance below.
            const av = Math.abs(velTargetY);
            if (av > holdMaxV) velTargetY *= holdMaxV / av;
        } else if (stickActive) {
            // Stick override: use manual control.
            // Smooth hand-over on release: ramp the velocity target from the speed AT THE MOMENT
            // OF TAKEOVER linearly down to zero over a fixed window. Making velTarget an
            // INDEPENDENT time function (not a follower of the current vx) is what fixes both
            // earlier failures at once:
            //   - a "snap to 0" jumped the velocity error ~11 m/s -> P term commanded a step of
            //     full acceleration (= aMax) -> attitude loop overshot -> lurch + sway;
            //   - an "exponential follow of vx" kept velTarget ~= vx every frame, so the velocity
            //     error stayed ~0 and the drone barely decelerated -> never stopped.
            // The linear window starts with velTarget ~= vx (error ~0, no step) and then keeps
            // widening the error until it reaches 0 at the end -> smooth AND decisive.
            // Cancel any running ramp as soon as the sticks are actively flown again.
            if (horizActive) this._handoverActive = false;
            if (!this._handoverActive) {
                this._handoverActive = true;
                this._handoverVx0 = this.vx;
                this._handoverVz0 = this.vz;
                this._handoverVy0 = this.vy;
                this._handoverT = 0;
            }
            this._handoverT += dt;
            const handoverMaxA = 7.5;  // m/s^2 cap on the stick-release brake: keeps the tilt small
                                       // (~37 deg) so the attitude loop does not overshoot while
                                       // levelling back to a hover -> no extended sway.
            const handoverSpeed0 = Math.hypot(this._handoverVx0, this._handoverVz0, this._handoverVy0);
            const handoverDur = Math.max(1.2, handoverSpeed0 / handoverMaxA);
            const handoverF = 1 - Math.min(1, this._handoverT / handoverDur);
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
                velTargetX = this._handoverVx0 * handoverF;
                velTargetZ = this._handoverVz0 * handoverF;
            }
            if (vertActive) {
                velTargetY = input.throttle * this.droneMaxVSpeed;
                this._targetY = this.y;
            } else if (!horizActive) {
                velTargetY = this._handoverVy0 * handoverF;
            } else {
                velTargetY = 0;
            }
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

            const yopoPosKp = this.yopoPosKp;   // Position loop gain: balances the "pull back to the old commanded position" tendency against cruise speed. Together with the server-side time scaling it makes the drone track faster commands more tightly.
            const yopoAltKp = this.yopoAltKp;   // Altitude loop gain: with 3D navigation the vertical error is dominated by the network trajectory, the position loop only corrects
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
        // True when the ray-avoidance layer is actively braking. While true the network's
        // acceleration feed-forward is suppressed (the ray layer takes priority over the YOPO
        // navigation plan) and a hard braking deceleration is commanded -- so the drone actually
        // slows instead of the network's forward acceleration cancelling the brake.
        let braking = false;
        // True when the ray layer had to override the plan via the closing-speed gate (see below).
        // Like `braking`, it suppresses the network's acceleration feed-forward and commands a
        // hard deceleration -- i.e. the ray layer wins over the YOPO plan.
        let gated = false;
        // Cached ray-avoidance result. `avoid` is declared inside the potential-field block below,
        // but the authoritative braking feed-forward further down needs its `brake` value too --
        // so we hoist a reference and the brake value to this outer scope.
        let avoid = null;
        let avoidBrake = 1.0;
        // ── Geometric reactive avoidance (potential field) ──
        // Probes the horizontal 360 deg ring obstacle distances + ground/roof clearance + three
        // altitude layers, producing repulsion (rep) / tangential detour (tan) / near-obstacle
        // braking (brake) / vertical obstacle clearing (vRep). This is the active avoidance
        // layer and it now runs UNCHANGED for the whole flight: there is no "takeover zone"
        // carve-out any more (the old 12 m final-approach PD that needed protecting is gone).
        // Once arrival is latched (yopoArrived) only the safety floor is kept -- brake plus the
        // vertical clearance limits -- because any directional thrust would fight the position
        // hold on the goal and turn into sway. Collisions are additionally covered by
        // _handleCollisions.
        if (this.yopoAvoidEnabled && this.yopoNavTarget &&
            !stickActive) {
            this._updateAvoidProbe();
            avoid = this._avoidanceVelocity(velTargetX, velTargetZ, velTargetY);
            if (avoid) {
                // Smooth the ray brake: asymmetric low-pass, TIGHTEN AT ONCE / RELEASE SLOWLY.
                // The probe is noisy near a goal that sits against a building -- measured: brake
                // flipping 0.89 <-> 1.00 frame to frame as goalClear toggles N/Y. Now that the
                // takeover zone relies on the brake ALONE (tan / vGo / vRep removed), that
                // toggling rescales the entire velocity target by ~11% every other frame, which is
                // a major part of the sway.
                // Safety: a TIGHTENING brake is applied immediately; only the release is slowed,
                // so this can never delay an actual stop.
                const brakeReleaseTau = 0.30;
                const brakeAlpha = 1 - Math.exp(-dt / brakeReleaseTau);
                if (this._avoidBrakeFilt === undefined || !Number.isFinite(this._avoidBrakeFilt)) {
                    this._avoidBrakeFilt = avoid.brake;
                } else if (avoid.brake < this._avoidBrakeFilt) {
                    this._avoidBrakeFilt = avoid.brake;              // tighten: immediate
                } else {
                    this._avoidBrakeFilt += (avoid.brake - this._avoidBrakeFilt) * brakeAlpha;
                }
                avoid.brake = this._avoidBrakeFilt;
                braking = avoid.brake < 0.95;
                avoidBrake = avoid.brake;
                // The ray layer keeps FULL priority right up to (and including) arrival.
                // There is no longer a "takeover zone" carve-out: while the drone is still
                // navigating it gets exactly the same avoidance as during cruise (repulsion,
                // tangential detour, braking, vertical clearing). The previous zone-specific
                // handling (repulsion suppressed, tangential steer gated by a hysteresis, an
                // end-game stand-down) existed only to protect the old 12 m takeover PD from
                // being fought by the ray layer, and that PD is gone now.
                // Arrived: still apply the FULL horizontal avoidance (rep + tan + vGo + brake), not
                // just the brake scaling. A takeover that relies on the brake alone lets the PD slide
                // the drone straight through a side building while converging / holding on the goal
                // (the "passes through a side building on descent" report). The directional thrust now
                // keeps it off the wall. Use the already speed-capped PD velocity as the forward
                // budget so the detour stays gentle and the drone does not overshoot the goal.
                const arrivedGentle = this.yopoArrived;
                {
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
                    let steerX = avoid.repX + avoid.tanX;
                    let steerZ = avoid.repZ + avoid.tanZ;
                    // Arrived: hold position on the goal -- do NOT apply the directional detour
                    // (rep/tan) push, otherwise the drone keeps sliding / veering around obstacles
                    // near the goal instead of settling. Only the passive brake (keeping velTarget
                    // scaled by avoid.brake) and the vertical vSafe clamps still apply.
                    if (this.yopoArrived) { steerX = 0; steerZ = 0; }
                    // Cap the detour vector ITSELF. Previously lateralBudget only shrank fwdAllow
                    // and the steer vector was added unclamped -- with the tangential gain at
                    // 34 m/s and the repulsion at 15, the detour could alone command maxSpd even
                    // while the brake demanded a full stop. That is the direct cause of "it still
                    // plans full speed with an obstacle ahead / the brake does not hold".
                    // Raised 0.68 -> 0.72: allow an even larger share of the speed budget into the
                    // lateral detour, so the drone slides past obstacles faster instead of crawling
                    // around them. 0.75 was measured as "detouring too fast", so this stays just
                    // under it; drop back to 0.68 if the detour feels wild.
                    // Budget base: the cruise floor (yopoCruiseMinSpd) or the actual commanded
                    // speed, whichever is LARGER. Keying it to the commanded speed alone was a
                    // regression: the network itself slows its commands when the depth shows
                    // obstacles, so during a real detour the budget collapsed exactly when it was
                    // needed (commanded 8 m/s -> only ~5.4 m/s of steering authority, i.e. the
                    // "detour is not decisive" report). The cruise floor keeps the detour strong.
                    // WHILE A DETOUR IS ACTUALLY IN PLAY we raise the floor to yopoDetourSpeedFloor
                    // (~22 m/s, comparable to the vertical vRep escape) so the lateral get-around
                    // speed is as decisive as flying over / under. The toward-obstacle component is
                    // still capped by the proximity governor and the forward brake / closing gate stay
                    // on, so this only frees the SLIDE-AROUND speed, never the charge-in.
                    const detourActive = Math.hypot(steerX, steerZ) > 1.5;
                    // Budget for the LATERAL DETOUR cap: raised to yopoDetourSpeedFloor while a detour is
                    // actually in play, so the slide-around escape is as decisive as flying over / under.
                    const budgetBase = arrivedGentle
                        ? Math.max(0.1, Math.hypot(velTargetX, velTargetZ))
                        : Math.max(this.yopoCruiseMinSpd, Math.hypot(velTargetX, velTargetZ),
                                   detourActive ? this.yopoDetourSpeedFloor : 0);
                    // Budget for the FORWARD allowance: deliberately NOT raised by the detour floor.
                    // Using the raised budget here was wrong -- fwdAllow = budget - lateralBudget then
                    // RELAXED from ~3.4 to ~6.2 m/s exactly when an obstacle is ahead, i.e. it let the
                    // drone push harder toward the obstacle. With an obstacle in front the forward
                    // component must stay tightly braked so the drone keeps its distance; only the
                    // lateral slide-around gets the extra authority.
                    const fwdBudget = arrivedGentle
                        ? Math.max(0.1, Math.hypot(velTargetX, velTargetZ))
                        : Math.max(this.yopoCruiseMinSpd, Math.hypot(velTargetX, velTargetZ));
                    const steerCap = budgetBase * this.yopoSteerCapFrac;
                    let steerMag = Math.hypot(steerX, steerZ);
                    if (steerMag > steerCap) {
                        const s = steerCap / steerMag;
                        steerX *= s; steerZ *= s; steerMag = steerCap;
                    }
                    if (steerMag > 1e-3) {
                        const lateralBudget = steerMag;
                        // Keep only a small forward floor (0.10) so that once a detour is in play
                        // the repulsion becomes the dominant velocity component -- the ray
                        // avoidance has the highest priority and the drone slides past the
                        // obstacle instead of pushing into it.
                        // Based on fwdBudget (NOT the detour-raised budget): with an obstacle ahead the
                        // forward component is squeezed down to this floor while the lateral detour
                        // carries the motion, so the drone slides around and keeps its distance
                        // instead of driving at the obstacle.
                        const fwdAllow = Math.max(fwdBudget * this.yopoFwdFloorFrac,
                                                  fwdBudget - lateralBudget);
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
                // descending).
                if (!this.yopoArrived) {
                    velTargetX += avoid.vGoX;
                    velTargetZ += avoid.vGoZ;
                }
                // Vertical: DIRECTION-AWARE instead of the old blanket `velTargetY *= brake`.
                // The old line throttled the climb by the HORIZONTAL obstacle brake, so near a
                // goal against a building (brake ~0.3) even the PD's straight climb crawled --
                // the drone could only gain/lose altitude with slow network circles and grazed
                // obstacles while doing it. Now:
                //   - CLIMB is NOT scaled by the horizontal brake (climbing away from a
                //     horizontal obstacle is the correct escape) and is capped only by the
                //     measured overhead clearance vSafeUp -- fast, yet unable to punch into a
                //     ceiling / overhang;
                //   - DESCENT keeps the horizontal brake scaling (an obstacle ahead at this
                //     altitude IS a threat while descending into it) plus the vSafeDown clamp
                //     below for the clearance under the drone.
                if (!this.yopoArrived) {
                    if (avoid.vRep) velTargetY = velTargetY * 0.3 + avoid.vRep;
                    velTargetY += avoid.upPush;
                }
                if (velTargetY < 0) {
                    velTargetY = velTargetY * avoid.brake;
                } else if (avoid.vSafeUp !== null && Number.isFinite(avoid.vSafeUp) &&
                           velTargetY > avoid.vSafeUp) {
                    velTargetY = avoid.vSafeUp;
                }
                // Vertical descent kinematic brake: the maximum allowed descent speed is
                // vSafeDown (>= 0).
                // If the network trajectory demands a faster descent (very negative velTargetY),
                // clamp it to -vSafeDown, guaranteeing it can physically stop within the
                // clearance below / ahead-below -> no crash into obstacles below.
                // KEPT inside the takeover zone: this one is not a detour or a climb-over, it is
                // the ray layer's descent SPEED LIMIT derived from the measured clearance below,
                // i.e. part of the ray avoidance itself. Without it the drone would descend into
                // the ground / an obstacle below while the PD pulls it toward the goal.
                if (avoid.vSafeDown !== null && Number.isFinite(avoid.vSafeDown)) {
                    if (velTargetY < -avoid.vSafeDown) {
                        velTargetY = -avoid.vSafeDown;
                        this._yopoGroundFloorActive = true; // Trigger climb / hold attitude
                    }
                }
                // Obstacle straight below / above: hold altitude, neither climb nor descend, and
                // let the horizontal detour vGo fly past smoothly, avoiding the "wants to
                // descend -> pushed away by rays/collision -> wants to descend again" oscillation.
                // EXCEPTION -- while OVERFLYING an obstacle, holding altitude is exactly wrong: if the
                // clearance straight below is already insufficient (vSafeDown pinned to 0, i.e. the drone
                // is skimming the rooftop it is passing over) the altitude hold keeps it glued at that
                // height and it clips the obstacle below. In that case the vertical escape must win, so
                // the climb (upPush / vSafeUp) is let through and the drone gains separation instead.
                const vGoHoldAlt = Math.hypot(avoid.vGoX, avoid.vGoZ) > 1e-6;
                const belowTooClose = avoid.vSafeDown !== null &&
                                      Number.isFinite(avoid.vSafeDown) &&
                                      avoid.vSafeDown <= 0.05;
                if (vGoHoldAlt && !belowTooClose) {
                    velTargetY = 0;
                }

                // ── Authoritative closing-speed gate (velocity-obstacle clipping) ──
                // `brake` only ever multiplied the FORWARD component. The detour terms
                // (repulsion + tangential, plus vGo) are added on top of it unbraked, and the
                // only remaining limit was the global maxSpd clamp. With the tangential gain at
                // 34 m/s and vGo at ~17 m/s, an obstacle dead ahead (brake ~ 0) could still yield
                // a ~15 m/s target -- exactly "it still plans full speed toward an obstacle" and
                // "the brake does not hold".
                // So the brake is now enforced on the FINAL composed vector: project it onto the
                // threat direction and hard-cap the closing speed at vCloseMax, the speed from
                // which the conservative brake can still stop inside the standoff. Only the
                // component ALONG the threat direction is removed, so the tangential part
                // survives untouched -- "charging at the wall" becomes "sliding along the wall",
                // the detour we actually want, rather than stalling in place.
                if (avoid.threatHasDir) {
                    const vClose = velTargetX * avoid.threatDirX +
                                   velTargetZ * avoid.threatDirZ;
                    if (vClose > avoid.vCloseMax) {
                        const excess = vClose - avoid.vCloseMax;
                        velTargetX -= excess * avoid.threatDirX;
                        velTargetZ -= excess * avoid.threatDirZ;
                        gated = true;
                    }
                }
                // ── Proximity speed governor ──
                // Cap ONLY the component of motion TOWARD the nearest obstacle (dMin direction),
                // leaving the tangential / slide-around component free so the drone can still detour.
                // The brake / closing gate above only constrain the forward (dAhead) component, so a
                // high planner speed next to a side/angled obstacle would otherwise slip through and
                // arrive too fast to react; this reins in only the "charging into the wall" part.
                // Motion that is NOT toward the obstacle (sliding along it, going around) stays
                // unlimited -- otherwise the drone could not get around at all.
                if (avoid.nearHasDir && avoid.nearSpeedCap !== undefined &&
                    Number.isFinite(avoid.nearSpeedCap)) {
                    const vNear = velTargetX * avoid.nearDirX +
                                  velTargetZ * avoid.nearDirZ;   // >0 = moving toward the obstacle
                    if (vNear > avoid.nearSpeedCap) {
                        const excess = vNear - avoid.nearSpeedCap;
                        velTargetX -= excess * avoid.nearDirX;
                        velTargetZ -= excess * avoid.nearDirZ;
                        gated = true;
                    }
                }
                // The ray layer is authoritative whenever it actually had to override the plan:
                // either the progressive brake engaged, or the closing-speed gate clipped the
                // target. In both cases the network's acceleration feed-forward is suppressed
                // and a hard deceleration is commanded.
                if (gated) braking = true;

                this._avoidAccScale = avoid.brake;
            }
        }

        // ── Cruise-phase direct ascend / descend ("vertical first") ──
        // See yopoVertFirst* in the constructor. Only on the CRUISE branch: inside the takeover
        // zone the final-approach PD already converges straight onto the goal (and carries its own
        // straight-vertical deadband), and under manual control the throttle is the pilot's.
        const inCruise = !stickActive && !this.yopoArrived;
        let vertFirst = false;
        // Vertical decision diagnostics: filled by the vertFirst branch below, printed in the periodic
        // log. They exist to answer "why is the descent not at yopoVertFirstVMax" without guessing --
        // the observed descent speed did not move across changes to VMax / the ramp decel / the
        // descent standoff, which means the vertFirst override was never applying at all.
        let vfD_h = 0, vfD_dy = 0, vfD_gate = 'off', vfD_go = false, vfD_rep = false,
            vfD_ramp = 0, vfD_vsd = null, vfD_vcmd = 0, vfD_pre = 0;
        vfD_pre = velTargetY;   // vertical target as composed by the network + avoidance (pre-override)
        if (this.yopoVertFirstEnabled && inCruise && this.yopoNavTarget && !this.yopoArrived) {
            const vfDx = this.yopoNavTarget.x - this.x;
            const vfDz = this.yopoNavTarget.z - this.z;
            const vfDy = this.yopoNavTarget.y - this.y;
            const vfH = Math.hypot(vfDx, vfDz);
            const vfAbsY = Math.abs(vfDy);
            vfD_h = vfH; vfD_dy = vfAbsY;
            // Hysteresis: engage on the full thresholds, release only once clearly outside them, so
            // the mode does not chatter on/off at the boundary (that chatter would show up as the
            // drone alternating between climbing and circling).
            const engage = vfH < this.yopoVertFirstHDist &&
                           vfAbsY > this.yopoVertFirstMinDY &&
                           vfAbsY > this.yopoVertFirstRatio * vfH;
            const keep = vfH < this.yopoVertFirstHDist * 1.25 &&
                         vfAbsY > this.yopoVertFirstMinDY * 0.6;
            this._vertFirstOn = this._vertFirstOn ? keep : engage;
            vfD_gate = this._vertFirstOn ? 'open' : 'closed';

            // Vertical clearance gate. The straight climb / descent may only run at a speed the
            // airframe can still stop within the clearance measured straight above / below (the
            // same kinematic limits the ray layer itself uses), and it stands down entirely when
            // the ray layer is actively pushing sideways (vGo = an obstacle straight overhead /
            // underfoot, where the only way out really is lateral) or when it wants to clear an
            // obstacle vertically in the opposite direction (vRep).
            const vGoActive = !!(avoid && Math.hypot(avoid.vGoX, avoid.vGoZ) > 1e-6);
            const vRepAgainst = !!(avoid && avoid.vRep &&
                                   Math.sign(avoid.vRep) !== Math.sign(vfDy));
            vfD_go = vGoActive; vfD_rep = vRepAgainst;
            if (this._vertFirstOn && !vGoActive && !vRepAgainst) {
                // Arrival ramp sqrt(2*a*d): the climb eases off as the goal altitude is approached
                // (no overshoot) and never starts with a velocity step.
                vfD_ramp = Math.sqrt(2 * this.yopoVertFirstDecel * vfAbsY);
                let vAllow = Math.min(this.yopoVertFirstVMax, vfD_ramp);
                if (vfDy > 0) {
                    if (avoid && avoid.vSafeUp !== null && Number.isFinite(avoid.vSafeUp)) {
                        vfD_vsd = avoid.vSafeUp;
                        vAllow = Math.min(vAllow, avoid.vSafeUp);
                    }
                } else if (avoid && avoid.vSafeDown !== null && Number.isFinite(avoid.vSafeDown)) {
                    vfD_vsd = avoid.vSafeDown;
                    vAllow = Math.min(vAllow, avoid.vSafeDown);
                }
                if (vAllow >= this.yopoVertFirstMinV) {
                    vertFirst = true;
                    const vCmd = Math.min(vAllow, this.yopoVertFirstKp * vfAbsY);
                    vfD_vcmd = vfDy > 0 ? vCmd : -vCmd;
                    velTargetY = vfDy > 0 ? vCmd : -vCmd;
                    // Horizontal: normally only a fraction is kept, so the drone creeps onto the goal
                    // column instead of holding a wide circle (the network trajectory plus the ray
                    // detour are what made the circle; the cruise floor below is switched off as well).
                    // EXCEPTION -- while the ray layer is actively pushing it away from an obstacle
                    // (significant repulsion / tangential detour, or the brake / closing gate fired)
                    // that scaling cripples the very avoidance which has to keep it off an obstacle it
                    // is climbing or descending past: it was left with only 30% of the lateral escape
                    // and simply descended / climbed into it. Keep the FULL horizontal command in that
                    // case so the lateral escape still works; only dial it down on a clear approach.
                    const rayPushing = !!avoid && (
                        Math.hypot(avoid.repX + avoid.tanX, avoid.repZ + avoid.tanZ) >
                            this.yopoVertFirstRayThreshold ||
                        avoid.brake < 0.95
                    );
                    const hScale = rayPushing ? 1.0 : this.yopoVertFirstHScale;
                    velTargetX *= hScale;
                    velTargetZ *= hScale;
                }
            }
        } else {
            this._vertFirstOn = false;
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

        // ── Minimum cruise-speed floor toward the goal (network under-drive guard) ──
        // The YOPO network often parks its local waypoint (cmdPos ~= drone, cmdVel ~= 0) while the
        // global goal is still far away, collapsing velTarget to a crawl (~1-2 m/s). When the
        // ray-avoidance layer is NOT actively braking/gating (path is clear) and a live, distant nav
        // target exists, top up the projection of velTarget onto the goal bearing so the drone keeps
        // at least yopoCruiseMinSpd of forward progress. Avoidance keeps priority: any time
        // braking/gated is true this is skipped, so obstacles still stop the drone.
        //
        // IMPORTANT: `braking` reads the ASYMMETRIC-FILTERED brake, which RATCHETS -- a tightening
        // brake is applied instantly while a release only ramps back with tau = 0.30 s. So on a
        // goalClear verdict that flips Y/N, the filtered brake can sit far below 0.95 for seconds
        // (`braking` stays true) even while the corridor is genuinely open. Measured: goalClear=Y,
        // brake=0.26, |vel| falling 2.0 -> 1.5 with the goal 58 m away -- this floor was disabled
        // the whole time and the drone crawled.
        // So the floor ALSO trusts `goalClear`, the RAW verdict computed before that filter: it is
        // exactly the authority the avoidance layer itself uses (on goalClear it already zeroes
        // rep/tan and releases the closing-speed gate to vCloseMax = Infinity). Obstacles still win
        // because a blocked corridor makes goalClear false and the floor stands down.
        const goalOpen = !!(avoid && avoid.goalClear);
        // Only ever active on the CRUISE branch (following the network trajectory). It must stay out
        // of: (a) stick override -- it would shove 12 m/s toward the goal while the pilot is flying;
        // (b) the final-approach hold -- that PD is deliberately decelerating onto the goal, and
        // forcing a 12 m/s floor would overshoot straight past it (exactly the arrival sway that was
        // just fixed with the deadband).
        // (inCruise is defined above, next to the vertical-first block.)
        // vertFirst is excluded on purpose: while the drone is doing a straight climb / descent the
        // floor would shove 12 m/s back along the horizontal goal bearing, which is exactly the
        // overshoot-and-come-back that produced the wide circling.
        if (inCruise && !vertFirst && (goalOpen || (!braking && !gated)) &&
            this.yopoNavTarget && !this.yopoArrived) {
            const ngx = this.yopoNavTarget.x - this.x;
            const ngz = this.yopoNavTarget.z - this.z;
            const ngd = Math.hypot(ngx, ngz);
            if (ngd > this.yopoCruiseMinDist) {
                const vNowH = Math.hypot(velTargetX, velTargetZ);
                if (vNowH < this.yopoCruiseMinSpd) {
                    const gdx = ngx / ngd, gdz = ngz / ngd;
                    const proj = velTargetX * gdx + velTargetZ * gdz;
                    const need = this.yopoCruiseMinSpd - proj;
                    if (need > 0) {
                        velTargetX += need * gdx;
                        velTargetZ += need * gdz;
                    }
                }
            }
        }

        // Diagnostics: record the velocity target, broken down so the speed limit can be located.
        //   cmdVel = horizontal speed the YOPO network itself commanded (is the limit upstream?)
        //   brake  = ray-avoidance brake factor (1 = not slowing at all)
        //   |vel|  = the speed actually achieved (compare with velTarget: if the target is much
        //            higher, the limit is the velocity loop / acceleration, not the planner)
        if (this.yopoInferenceCount < 5 || this.yopoInferenceCount % 120 === 0) {
            const spdH = Math.hypot(this.vx, this.vz);
            const cmdVelH = this.yopoCmdVel
                ? Math.hypot(this.yopoCmdVel.x || 0, this.yopoCmdVel.z || 0)
                : 0;
            // towardGoal: cosine between the actual velocity and the bearing to the goal.
            //   +1 = flying straight at the goal, 0 = perpendicular, -1 = flying directly away.
            // Together with rep/tan it separates "the avoidance field is pushing me off the
            // goal" from "the backend commanded a heading away from the goal".
            let towardGoal = 0;
            if (this.yopoNavTarget) {
                const gdx = this.yopoNavTarget.x - this.x;
                const gdz = this.yopoNavTarget.z - this.z;
                const gl = Math.hypot(gdx, gdz);
                if (gl > 0.5 && spdH > 0.2) {
                    towardGoal = (this.vx * gdx + this.vz * gdz) / (gl * spdH);
                }
            }
            const repMag = avoid ? Math.hypot(avoid.repX, avoid.repZ) : 0;
            const tanMag = avoid ? Math.hypot(avoid.tanX, avoid.tanZ) : 0;
            console.log(
                `_controlYOPO velTarget=(${velTargetX.toFixed(2)},${velTargetY.toFixed(2)},${velTargetZ.toFixed(2)}) ` +
                `|vel|=${spdH.toFixed(2)} cmdVel=${cmdVelH.toFixed(2)} ` +
                `towardGoal=${towardGoal.toFixed(2)} rep=${repMag.toFixed(1)} tan=${tanMag.toFixed(1)} ` +
                `brake=${avoid ? avoid.brake.toFixed(2) : 'n/a'} ` +
                `vCloseMax=${avoid ? (Number.isFinite(avoid.vCloseMax) ? avoid.vCloseMax.toFixed(2) : 'inf') : 'n/a'} ` +
                `goalClear=${avoid ? (avoid.goalClear ? 'Y' : 'N') : 'n/a'} ` +
                `distGoal=${distGoal.toFixed(1)} arrivedHold=${yopoArrivedHold} ` +
                `vertFirst=${vertFirst ? 'Y' : 'N'} ` +
                `vf(h=${vfD_h.toFixed(1)} dy=${vfD_dy.toFixed(1)} gate=${vfD_gate} ` +
                `go=${vfD_go ? 'Y' : 'N'} rep=${vfD_rep ? 'Y' : 'N'} ` +
                `ramp=${vfD_ramp.toFixed(1)} vsd=${vfD_vsd === null ? 'n/a' : vfD_vsd.toFixed(1)} ` +
                `vcmd=${vfD_vcmd.toFixed(1)} pre=${vfD_pre.toFixed(1)}) ` +
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
        // YOPO-specific velocity loop parameters. The gain now comes from this.yopoVelKp (4.0)
        // instead of the old 2.2 -- see that parameter's comment for why 2.2 cost most of the
        // cruise speed. The velocity error is clamped to aMaxHoriz/velKp just below, so the
        // acceleration demand stays inside what the tilt limit can produce.
        const velKp = useAccFeedforward ? this.yopoVelKp : this.sfVelKp;
        const velKi = useAccFeedforward ? 0.0 : (stickActive ? 0.0 : this.sfVelKi);
        // Drop the velocity-loop D term once latched as arrived: the remaining error is tiny and
        // the D term there only amplifies velocity measurement noise, which shows up as a
        // permanent fine jitter while hovering on the goal.
        // Also drop the D term during a stick override (taking over from YOPO navigation).
        // Handing over snaps the velocity target straight from the YOPO command (say 11 m/s) to
        // the stick command (0 when the sticks are released), so the velocity ERROR jumps by that
        // whole amount in one frame. The D term differentiates that jump into a large
        // acceleration spike, which is what makes the drone lurch and then sway for a moment
        // instead of just braking smoothly to a stop.
        // During a stick override we still want the velocity-loop D term EXCEPT when the pilot is
        // actively moving the sticks (a stick step would differentiate into a tilt spike). On the
        // released hand-over ramp the velocity target is a smooth linear function, so D only adds
        // useful damping and kills the residual sway after the stop.
        // The takeover zone must keep the velocity-loop D term (damping) -- this is the motion-loop
        // root cause of "jitter inside the takeover range":
        // the takeover zone inherits useAccFeedforward = true from cruise and the old test zeroed
        // velKd along with it, degrading the velocity loop to pure P; combined with the inner-loop
        // (attitude) lag and the 60-150 ms ray-probe latency, the phase lag makes the closed loop
        // under-damped. Numerical simulation (entering at d=12 m, v=10 m/s, velKp=4.0): at tau=0.20 s
        // pure P overshoots 1.21 m, tail speed 1.16 m/s, 4 speed reversals; at tau=0.30 s it
        // overshoots 2.32 m with a 3.88 m/s tail and never settles on the goal.
        // Restoring velKd = 1.0 under the same conditions drops the overshoot to 0.14 m, the tail to
        // 0.05 m/s and the reversals to 0, and stays robust over the whole tau = 0.08~0.40 range
        // (overshoot <= 0.89 m, tail <= 0.21 m/s, parks exactly on the goal) with the settle time
        // unchanged (1.60 s -> 1.53 s). Inside the takeover zone the velocity target comes from the
        // local PD + slew limit and the network feed-forward is stopped and stale there (no ffVel
        // jumps), so the D term does not amplify noise -- unlike cruise (which has feed-forward jumps
        // and still keeps D off).
        // After arrival the D term is switched off as well: with the error near zero it only
        // differentiates and amplifies the velocity-measurement noise into a permanent fine jitter
        // while parked on the goal (which is why arrival switched D off in the first place). Damping
        // after arrival comes from the lock logic (velTarget forced to zero plus the inner P loop's
        // velocity feedback toward zero).
        const velKd = (useAccFeedforward || (stickActive && horizActive)) ? 0.0 : this.sfVelKd;
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
            // While the ray-avoidance layer is actively braking, the network's acceleration
            // feed-forward is SUPPRESSED entirely (it may point straight at the obstacle and would
            // otherwise cancel the brake). This is what makes the ray layer take priority over the
            // YOPO navigation plan. When not braking we only attenuate it by the brake scale.
            ffScale *= braking ? 0 : (this._avoidAccScale || 1.0);
            aDesX += this.yopoCmdAcc.x * ffScale;
            // During a vertical-first climb / descent the network's vertical acceleration belongs
            // to the spiral we just opted out of, so it is dropped: the straight altitude change is
            // then driven by the velocity loop alone and stays predictable.
            aDesY += vertFirst ? 0 : (this.yopoCmdAcc.y || 0) * ffScale;
            aDesZ += this.yopoCmdAcc.z * ffScale;
            // When the hard ground floor triggers, forbid a downward acceleration feed-forward
            // (it would cancel the climb) and force aDesY >= 0
            if (this._yopoGroundFloorActive) {
                if (this.yopoCmdAcc && this.yopoCmdAcc.y < 0) aDesY -= this.yopoCmdAcc.y * ffScale;
                if (aDesY < 0) aDesY = 0;
            }
        }

        // ── Authoritative braking feed-forward (ray layer overrides the network) ──
        // When the ray avoidance is braking, actively command deceleration along the CURRENT velocity
        // direction at up to yopoAvoidBrakeAccel. This makes the real deceleration strong and
        // immediate instead of waiting for the P velocity loop to chase a lower setpoint, directly
        // fixing "deceleration not strong enough / still hitting obstacles". It is added on top of the
        // velocity loop and is the reason the ray layer wins over the YOPO plan.
        // ── Emergency-brake state (shared by this feed-forward and the attitude gain below) ──
        // `braking` alone is too weak a condition for handing over full authority: it is true as soon
        // as brake < 0.95 and the soft brake now reaches 22 m, so in an urban scene it is true almost
        // continuously. Floors/boosts applied unconditionally on it would brake hard 40+ m from the
        // obstacle and drag the cruise speed down (the old "only manages 1-4 m/s" complaint).
        // Everything aggressive is therefore gated on `brakeUrgent`: a genuine emergency stop
        // (brake below yopoAvoidBrakeUrgent) or the closing-speed gate having fired.
        const brakeUrgent = braking && (gated || avoidBrake < this.yopoAvoidBrakeUrgent);
        if (braking) {
            const spd = Math.hypot(this.vx, this.vz);
            if (spd > 0.2) {
                // Deceleration commanded by the ray layer.
                // The progressive brake contributes proportionally to (1 - brake). When the
                // closing-speed gate fired, the plan was closing faster than stoppable, so the
                // brake gets most of its authority regardless of the progressive value -- without
                // this, a gate firing while brake was still ~1 (e.g. the corridor was declared
                // clear but a near side obstacle was closing) would command no deceleration at all.
                const aFromBrake = this.yopoAvoidBrakeAccel * (1 - avoidBrake);
                // Floor the commanded deceleration, but ONLY once the stop is urgent. A brake that
                // has just begun to bite (brake ~0.9) only deserves the proportional ~(1-brake)
                // nudge; flooring there would slam on ~8.5 m/s^2 forty metres from the obstacle and
                // destroy the cruise speed. Once urgent (brake < yopoAvoidBrakeUrgent, or the gate
                // fired) at least yopoAvoidBrakeMinFrac of the authority is handed over immediately,
                // so the drone no longer coasts at nearly full speed while the brake is merely
                // "technically on". The gate keeps a larger floor (0.85) and the result stays capped
                // at the full brakeAccel.
                const minFrac = gated
                    ? Math.max(this.yopoAvoidBrakeMinFrac, 0.85)
                    : this.yopoAvoidBrakeMinFrac;
                const aB = brakeUrgent
                    ? Math.min(this.yopoAvoidBrakeAccel,
                               Math.max(aFromBrake, this.yopoAvoidBrakeAccel * minFrac))
                    : aFromBrake;
                aDesX += -(this.vx / spd) * aB;
                aDesZ += -(this.vz / spd) * aB;
            }
        }

        // ── Desired acceleration safety ceiling (prevents "acceleration too large -> hits an
        // obstacle before the next command") ──
        // When replanning (the depth loop) is slow, an excessively large combined acceleration
        // can carry the drone into a solid before the next avoidance command arrives. The
        // combined horizontal acceleration and the vertical acceleration are each clamped to
        // yopoAccMax, leaving braking and reaction margin (the equivalent max tilt drops from
        // 58 deg to ~atan(8/9.81) ~= 39 deg).
        // While the ray layer is braking, allow the deceleration to use a higher ceiling
        // (yopoAvoidBrakeAccel, near the physical tilt limit) so the drone can actually slam the
        // brakes. During normal flight the ceiling stays at yopoAccMax.
        // While the sticks are released and the hand-over ramp is braking, cap the commanded
        // acceleration to handoverMaxA (6 m/s^2 ~= 31 deg tilt) instead of the full yopoAccMax
        // (8 m/s^2 ~= 39 deg). The smaller tilt means far less overshoot when the attitude loop
        // levels back to a hover, so the drone settles quickly instead of swaying for seconds.
        const aMaxCmd = (stickActive && !horizActive)
            ? Math.min(this.yopoAccMax, 7.5)
            : (braking ? Math.max(this.yopoAccMax, this.yopoAvoidBrakeAccel) : this.yopoAccMax);
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

        // While the ray layer is braking HARD, the attitude slew IS the reaction time: the drone must
        // rotate from its cruise tilt to the braking tilt before any deceleration exists at all, and
        // at the stock gains that costs ~0.28 s of completely un-braked flight at speed. Scaling the
        // angle error shortens the slew to ~0.16 s so the higher yopoAvoidBrakeAccel is actually
        // delivered while there is still room to stop.
        // `brakeUrgent` (computed above, next to the braking feed-forward) is the gate: the boost is
        // reserved for a real emergency stop or a fired closing-speed gate, which is exactly when the
        // extra ~0.12 s decides whether it crashes.
        const angleKp = this.sfAngleKp * (brakeUrgent ? this.yopoAvoidBrakeAngleGain : 1.0);
        const rateTargetPitch = angleKp * angleErrPitch + this.sfAngleKd * this._sfFiltAngleDerrPitch;
        const rateTargetRoll  = angleKp * angleErrRoll  + this.sfAngleKd * this._sfFiltAngleDerrRoll;

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
        } else if (yopoArrivedHold) {
            // Arrived: hold the current yaw, do not rotate
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
        // NO CACHING, NO TIERING (explicit requirement): every direction of the 360 deg ring is
        // re-probed EVERY cycle with forceFresh=true, so no distance is ever served from the
        // pickLocalRay cache and no direction is downsampled, mirrored or rotated out.
        // Rationale: the pick cache (0.5 m origin quantisation + direction bucket + 150 ms TTL)
        // and the tiering (core / cone / periphery + high-speed stride + round-robin slices) both
        // let a direction keep a distance measured from a different position or an older cycle.
        // At cruise speed the drone covers several metres per cycle, so a stale / interpolated
        // distance computes the braking distance wrong and the ray layer under-reacts -- which is
        // the "still plans a big speed toward the obstacle" symptom.
        // Cost: N fresh GPU picks per cycle (N = yopoAvoidRayCount = 12) instead of ~20. The ring
        // is small enough that this stays affordable, and correctness beats frame rate here.
        // Throttling (how often a probe cycle runs) still happens in _updateAvoidProbe.
        let rayCount = 0;
        const rayDist = (dir, yLevel) => {
            rayCount++;
            const hit = w.pickLocalRay({ x: this.x, y: yLevel, z: this.z }, dir, R, true);
            return (hit && Number.isFinite(hit.distance) && hit.distance > 0.04) ? hit.distance : R;
        };
        // Forward direction (horizontal): velocity takes priority, otherwise the body forward -Z
        let fwdHx = 0, fwdHz = -1;
        const spdHv = Math.hypot(this.vx, this.vz);
        if (spdHv > 0.3) { fwdHx = this.vx / spdHv; fwdHz = this.vz / spdHv; }

        // Ring state: kept so the returned arrays always describe a complete 360 deg ring, but
        // with the tiering gone every direction is overwritten by a fresh measurement every
        // cycle, so a carried-over distance or ringAge staleness can no longer reach the field.
        if (!this._avoidRing || this._avoidRing.length !== N) {
            this._avoidRing = new Float64Array(N).fill(R);
            this._avoidRingAge = new Float64Array(N).fill(1e9);
            this._avoidSliceCursor = 0;
        }
        const ring = this._avoidRing;
        const ringAge = this._avoidRingAge;

        // Altitude probing: mid (current altitude) for every ring direction; the extra
        // high/high2/low layers go along the 3 rays best aligned with the forward direction,
        // because vertical obstacle clearing only cares whether the surface straight ahead can be
        // flown over / dived under. Those 3 rays are now probed EVERY cycle (no gating on the
        // previous cycle's blocked verdict) and always fresh.
        const dists = new Array(N);
        const distsHigh = new Array(N);
        const distsHigh2 = new Array(N);
        const distsLow = new Array(N);
        const yHigh = this.y + this.yopoAvoidVStep;
        const yHigh2 = this.y + this.yopoAvoidVStep * 2;
        const yLow = Math.max(this.y - this.yopoAvoidVStep, groundY + 1.0);
        const lowOk = (yLow - groundY) > 1.5; // The down-probe layer counts as a valid dive only if clearly above the ground

        // The 3 forward-most rays carry the vertical over/under layers. Chosen from the whole ring
        // now that the core/cone/periphery tiering is gone.
        const vProbeIdx = Array.from({ length: N }, (_, i) => i)
            .sort((a, b) => (dirs[b].x * fwdHx + dirs[b].z * fwdHz) -
                            (dirs[a].x * fwdHx + dirs[a].z * fwdHz))
            .slice(0, 3);

        // Main ring: EVERY direction, EVERY cycle, forceFresh -- no cache, no stride downsampling,
        // no round-robin slices and no mirrored neighbours, so the potential field never sees a
        // distance measured from another position or from an older cycle.
        for (let i = 0; i < N; i++) {
            ring[i] = dists[i] = rayDist(dirs[i], this.y);
            ringAge[i] = 0;
            distsHigh[i] = dists[i];
            distsHigh2[i] = dists[i];
            distsLow[i] = dists[i];
        }
        // Vertical over/under layers along the forward-most rays: every cycle, fresh.
        // Gating these on the PREVIOUS cycle's "forward corridor blocked" verdict made vertical
        // clearing depend on a one-cycle-old decision, so it could miss the gap and never trigger.
        for (const i of vProbeIdx) {
            distsHigh[i] = rayDist(dirs[i], yHigh);
            distsHigh2[i] = rayDist(dirs[i], yHigh2);
            distsLow[i] = lowOk ? rayDist(dirs[i], yLow) : dists[i];
        }



        // Straight up / straight down vertical rays: the horizontal ring at any layer cannot
        // detect an obstacle "directly above / below at the same x,z" (such as a ceiling
        // overhead or a square rooftop underfoot). Prevents climbing into the ceiling and
        // descending into an obstacle straight below.
        let vUpDist = R, vDownDist = R;
        // Straight up / down: probed EVERY cycle (no "every N cycles" tiering) and always fresh --
        // a stale ceiling / floor distance is a real hazard.
        if (this.yopoAvoidVertRay) {
            rayCount += 2;
            // Ceiling / floor safety rays stay fresh (a stale ceiling distance is a real hazard).
            const hUp = w.pickLocalRay({ x: this.x, y: this.y + 0.5, z: this.z }, { x: 0, y: 1, z: 0 }, this.yopoAvoidVertRange, true);
            vUpDist = (hUp && Number.isFinite(hUp.distance) && hUp.distance > 0.04) ? hUp.distance : R;
            const hDn = w.pickLocalRay({ x: this.x, y: this.y - 0.5, z: this.z }, { x: 0, y: -1, z: 0 }, this.yopoAvoidVertRange, true);
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
        // State record / diagnostic only now. With the tiering removed the vertical (high/high2/
        // low) layers are probed EVERY cycle along the forward-most rays, so this verdict no
        // longer gates them. It used to decide whether those rays were spent at all, which made
        // vertical clearing depend on a one-cycle-old decision -- a side obstacle inside the old
        // narrowed / halved cone kept it false and "vertical clearing stopped triggering".
        // Scanning the whole ring still keeps it identical in coverage to dAheadH.
        let fwdCorridor = R;
        for (let i = 0; i < dirs.length; i++) {
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
        for (let i = 0; i < N; i++) {
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
    _avoidanceVelocity(velTargetX, velTargetZ, velTargetY = 0) {
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
        // Vertical-only kinematic deceleration for vSafeUp / vSafeDown / upPush, using yopoAvoidVDecel
        // (the thrust axis brakes harder than the tilt-limited forward axis). It is STILL bounded by
        // yopoAccMax and derated by 0.9 for the response lag (ray throttle + control loop + attitude
        // build-up), because vSafeUp / vSafeDown are STOPPABLE-speed caps: the `a` used to compute them
        // must not exceed what the airframe can really arrest. Feeding in a raw value above that limit
        // lets the drone descend / climb faster than it can stop, so it punches into the obstacle above
        // or below -- exactly the "flying over the obstacle yet hitting the one below" symptom.
        const aDecel = Math.min(this.yopoAvoidVDecel, this.yopoAccMax) * 0.9;

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
        // Side-push range: DECOUPLED from repRange (which also gates goalClear). Wider so the
        // side repulsion holds the drone further off building faces and engages earlier, without
        // making the "corridor is clear" verdict stricter.
        const pushRange = this.yopoAvoidPushRange +
            (this.yopoAvoidPushRangeHi - this.yopoAvoidPushRange) * tFast;

        let repX = 0, repZ = 0;
        let dMin = R;        // Nearest obstacle overall (drives repulsion / tangential strength)
        let dMinDirX = 0, dMinDirZ = 0;  // Unit direction (outward from drone) toward the nearest obstacle
        let dAhead = R;      // Threat ahead (including vertical threats, used for braking / push-up)
        let dAheadH = R;     // Nearest-ahead distance from the horizontal ring rays only
                             // (excluding vertical threats), used only for the vertical clearing
                             // decision, so vertical threats cannot shrink dAhead and wrongly
                             // trigger flying over / diving under.
        let openDirX = 0, openDirZ = 0, openMax = -1;
        // Nearest obstacle that actually lies IN THE WAY, i.e. inside a wide cone around the goal
        // bearing, plus its ray index. The tangential detour is computed from THIS direction.
        // It used to be computed from dMin, the globally nearest obstacle -- which may sit behind
        // or beside the drone. A building just passed / alongside then set the tangent, and the
        // detour happily steered around to the far side of THAT building even though the way ahead
        // was wide open ("the way to the goal is clear, yet it goes around the other side").
        let dFront = R, miFront = -1;
        // Unit direction of the threat that produced dAhead, plus a validity flag. The call site
        // uses it to hard-clip the closing speed of the FINAL composed velocity target.
        // dAheadHasDir is false when dAhead came from a vertical threat (groundGap): there is no
        // horizontal direction to clip against, so the horizontal gate must not fire.
        let dAheadDirX = 0, dAheadDirZ = 0, dAheadHasDir = false;

        const des = Math.hypot(velTargetX, velTargetZ);
        let udx = 0, udz = 0;
        if (des > 0.3) { udx = velTargetX / des; udz = velTargetZ / des; }

        // Goal bearing (body -> navigation goal): computed up front, used by the "corridor"
        // check for dAheadH below.
        // Falls back to the commanded velocity direction when there is no navigation goal.
        let gx = udx, gz = udz;
        // Horizontal distance to the navigation goal (0 = no goal). Bounds the corridor and
        // threat checks: the drone only ever travels AS FAR AS THE GOAL (the final-approach PD
        // stops it there), so an obstacle lying beyond the goal -- typically the wall / tree the
        // goal sits against -- can never be hit and must not count as "blocking the way".
        let distGoalH = 0;
        if (this.yopoNavTarget) {
            const tdx = this.yopoNavTarget.x - this.x;
            const tdz = this.yopoNavTarget.z - this.z;
            const tl = Math.hypot(tdx, tdz);
            distGoalH = tl;
            if (tl > 0.5) { gx = tdx / tl; gz = tdz / tl; }
        }
        // Corridor half-width (m) used to trigger obstacle clearing: slightly wider than
        // goalClear's pathHalfWidth (2.5) to leave a safety margin.
        const surmountHalfW = 3.0;

        for (let i = 0; i < dirs.length; i++) {
            const d = dists[i];
            if (!Number.isFinite(d) || d <= 0) continue;
            if (d < dMin) { dMin = d; dMinDirX = dirs[i].x; dMinDirZ = dirs[i].z; }
            if (d > openMax) { openMax = d; openDirX = dirs[i].x; openDirZ = dirs[i].z; }
            if (d < pushRange) {
                // Keep-out shaped repulsion: full strength inside the side standoff
                // (yopoAvoidSideStandoff), tapering linearly to 0 at pushRange. This holds the
                // drone ~sideStandoff off building faces and engages from further out than the old
                // 1 - d/repRange falloff (which was ~0 beyond ~28 m and far too weak at 10-20 m,
                // letting the side graze / clip through during a descent).
                const sd = this.yopoAvoidSideStandoff;
                const w = Math.min(1, Math.max(0, (pushRange - d) / Math.max(1e-3, pushRange - sd)));
                repX -= dirs[i].x * w;
                repZ -= dirs[i].z * w;
            }
            // Threat ahead: obstacles near the desired velocity direction count toward braking
            const dot = dirs[i].x * udx + dirs[i].z * udz;
            if (des > 0.3 ? (dot > 0.5 && d < dAhead) : (d < dAhead)) {
                dAhead = d;
                dAheadDirX = dirs[i].x; dAheadDirZ = dirs[i].z; dAheadHasDir = true;
            }
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
            // Detour reference: nearest obstacle in the way (wide cone around the goal bearing).
            if (dotG > this.yopoTanConeCos && d < dFront) { dFront = d; miFront = i; }
        }

        // ── Vertical look-ahead repulsion ──
        // The horizontal ring only probes at the DRONE'S CURRENT altitude, so an obstacle sitting BELOW
        // (or above) it -- i.e. one the drone is about to descend / climb into -- is invisible to the
        // ring and produces no lateral repulsion until the drone reaches its altitude. At a fast
        // vertical speed that is far too late: the drone arrives at the obstacle already inside it (the
        // "does not avoid an obstacle below / to the lower side while descending or climbing" symptom).
        // Anticipate it by projecting the probe layer the drone is moving INTO (low layer while
        // descending, high layer while climbing) into the lateral repulsion, so it shifts sideways
        // before it gets there. vSafeDown / vSafeUp only guard the clearance STRAIGHT below / above,
        // so a lower-SIDE obstacle was previously not covered by anything at all.
        {
            const lookDown = velTargetY < -0.5;
            const lookUp = velTargetY > 0.5;
            // distsLow is only a real low-layer ray when lowOk holds (otherwise it is a copy of the
            // current layer, which would double-count the ring repulsion); distsHigh is only real on
            // the indices that actually got a high probe.
            const layer = lookDown && p.lowOk === true ? p.distsLow
                        : lookUp ? p.distsHigh : null;
            const hiIdx = p.highProbeIdx || null;
            if (layer) {
                const sd = this.yopoAvoidSideStandoff;
                for (let i = 0; i < dirs.length; i++) {
                    if (lookUp && hiIdx && hiIdx.indexOf(i) < 0) continue;
                    const d = layer[i];
                    if (!Number.isFinite(d) || d <= 0 || d >= pushRange) continue;
                    const w = Math.min(1, Math.max(0, (pushRange - d) / Math.max(1e-3, pushRange - sd)));
                    repX -= dirs[i].x * w * this.yopoVertLookWeight;
                    repZ -= dirs[i].z * w * this.yopoVertLookWeight;
                }
            }
        }

        // Authoritative threat: also brake on where the drone is ACTUALLY moving (inertia / drift /
        // the network commanding a curve while the body still carries velocity into the wall), not just
        // on where the network commands. Otherwise the network can point the command around the
        // obstacle while the airframe keeps closing on it and the ray brake never fires. If the actual
        // velocity heading sees a nearer obstacle than the commanded heading, adopt that nearer
        // distance as the threat.
        const spdAct = Math.hypot(this.vx, this.vz);
        if (spdAct > 0.5) {
            const adx = this.vx / spdAct, adz = this.vz / spdAct;
            let dAheadAct = R, dAheadActDirX = 0, dAheadActDirZ = 0;
            for (let i = 0; i < dirs.length; i++) {
                const d = dists[i];
                if (!Number.isFinite(d) || d <= 0) continue;
                const dotA = dirs[i].x * adx + dirs[i].z * adz;
                if (dotA > 0.5 && d < dAheadAct) {
                    dAheadAct = d;
                    dAheadActDirX = dirs[i].x; dAheadActDirZ = dirs[i].z;
                }
            }
            if (dAheadAct < dAhead) {
                dAhead = dAheadAct;
                dAheadDirX = dAheadActDirX; dAheadDirZ = dAheadActDirZ; dAheadHasDir = true;
            }
        }

        // Insufficient ground/roof clearance -> push up and take part in braking
        let upPush = 0;
        if (Number.isFinite(p.groundGap) && p.groundGap < this.yopoMinAlt) {
            upPush = (this.yopoMinAlt - p.groundGap) * this.yopoAvoidGain * 0.5;
            // Vertical threat: no horizontal direction, so the closing-speed gate must stay off.
            // Do NOT overwrite dAhead with groundGap: dAhead drives the FORWARD brake, and a small
            // clearance straight below must not throttle horizontal cruise. The vertical channel
            // (upPush / vSafeDown) already handles ground/ceiling safety independently, so braking
            // the forward velocity for an obstacle that is directly below would just make the drone
            // crawl in perfectly clear horizontal air whenever it flies low -- the "slows down under
            // an obstacle" symptom.
            if (p.groundGap < dAhead) { dAheadHasDir = false; }
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
            const sd = this.yopoAvoidStopDown;  // independent DOWN standoff (raised so descent holds further off below)
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

        // Standalone climb speed cap (symmetric to vSafeDown): returned so _controlYOPO can cap
        // ANY upward velocity target (the final-approach PD climb, vRep fly-over, upPush) by the
        // measured overhead clearance. This is what lets the climb ignore the HORIZONTAL brake
        // (climbing away from a wall is the correct escape) while staying physically unable to
        // hit a ceiling / overhang straight above.
        let vSafeUp = null;
        if (Number.isFinite(p.vUpDist)) {
            const su = this.yopoAvoidStop;
            vSafeUp = p.vUpDist <= su ? 0 : Math.sqrt(2 * aDecel * (p.vUpDist - su));
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
        // Horizontal standoff: the WIDER yopoAvoidStopH (7.5 m) so the drone keeps further off
        // walls / buildings (keep-further-away request). The vertical clearances (vSafeUp /
        // vSafeDown above) still use the narrower yopoAvoidStop on purpose -- see the note there.
        const standoff = this.yopoAvoidStopH;
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
        // The lead now grows with speed on top of that (yopoAvoidBrakeReactionHi): a constant
        // reaction time under-estimates the dead time exactly when it matters most, because the
        // faster the cruise the bigger the forward tilt that has to be shed first.
        const reactionSec = this.yopoAvoidBrakeReaction +
            (this.yopoAvoidBrakeReactionHi - this.yopoAvoidBrakeReaction) * tFast;
        const reactionDist = spdFwd * reactionSec;
        // Obstacles BEYOND the goal must not brake the final approach. The drone only ever
        // travels AS FAR AS THE GOAL -- stopping there is the final-approach PD's job, and it has
        // its own sqrt(2*a*d) ceiling -- so a wall BEHIND the goal is not something to stop for.
        // Treating it as one drove brake to 0 while the drone was still metres short and pinned it
        // there: the "navigation is inaccurate near the goal / never quite reaches it" symptom.
        // This got much worse once the tangential detour (tan) was removed from the takeover zone,
        // because braking is then the only obstacle reaction left.
        // A threat NEARER than the goal (dAhead <= distGoalH - yopoAvoidGoalGateMargin) still uses
        // the real ray distance, so genuine blocking of the remaining path stays fully covered.
        // Threats within the margin of the goal ARE the goal's wall (see yopoAvoidGoalGateMargin):
        // they must take the beyond-goal path, otherwise brake lands in the `brake = 0` branch
        // below (brakeClear ~= distGoalH <= standoff + reactionDist) and the drone is pinned.
        const beyondGoal = this.yopoNavTarget && distGoalH > 0.5 &&
            dAhead > distGoalH - this.yopoAvoidGoalGateMargin;
        const brakeClear = beyondGoal ? distGoalH + this.yopoAvoidGoalMargin : dAhead;
        if (brakeClear <= standoff + reactionDist) {
            brake = 0;  // Already inside the safety clearance + reaction distance -> stop advancing entirely
        } else if (brakeClear < R) {
            const dEff = brakeClear - standoff - reactionDist;  // effective stopping distance
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
                (brakeClear - standoff * 2) / (brakeRange - standoff * 2),
                0, 1
            );
            const soft = this.yopoAvoidBrakeFloor +
                (1 - this.yopoAvoidBrakeFloor) * softT;
            brake = Math.min(kinBrake, soft);
        }
        // A beyond-goal obstacle may only slow the run-in, never pin the drone metres short of the
        // goal: keep enough authority to keep closing on it. This is deliberately applied AFTER
        // the branch above so it also lifts the `brake = 0` case -- that branch is exactly where a
        // goal-adjacent wall used to pin the drone (brakeClear ~= distGoalH <= standoff +
        // reactionDist -> brake = 0 with no floor).
        if (beyondGoal && brake < this.yopoAvoidGoalBrakeFloor) {
            brake = this.yopoAvoidGoalBrakeFloor;
        }

        const repMag = Math.hypot(repX, repZ);
        // Clamp the repulsion strength
        if (repMag > 1e-6) {
            const s = Math.min(1, this.yopoAvoidRepGain / repMag);
            repX *= s; repZ *= s;
        }

        // Tangential detour: compute a deterministic tangent from the direction of the obstacle
        // that is actually IN THE WAY (dFront / miFront, a wide cone around the goal bearing),
        // steering around it toward the goal side, without picking an opening / falling back to the
        // emptiest direction -- that would detour to the side and, when the goal is blocked, wrongly
        // choose "emptiest = the way it came" and turn back ("steers around then goes back").
        // The reference used to be dMin, the GLOBALLY nearest obstacle, which may sit behind or
        // beside the drone: a building just passed / alongside then supplied the tangent and the
        // detour steered around to the far side of that building although the way ahead was open.
        // On top of that a direction hysteresis memory: if the angle against the previous frame's
        // tan exceeds 120 deg while that direction is still clear, keep the previous frame,
        // preventing the resultant from flipping when passing the obstacle centre and causing
        // back-and-forth detours.
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
        // Reference direction: the obstacle in the goal cone when there is one, otherwise the
        // commanded-velocity threat direction (dAheadDir) -- and no detour at all when neither
        // exists, since there is then nothing to steer around.
        const useFrontRef = miFront >= 0;
        const tanHasRef = useFrontRef || dAheadHasDir;
        if (tanHasRef && dAhead < repRange) {
            {   // Scope kept so the tangent locals stay together; the detour runs whenever a
                // reference direction was found above.
                const ox = useFrontRef ? dirs[miFront].x : dAheadDirX;   // Pointing at the obstacle to steer around
                const oz = useFrontRef ? dirs[miFront].z : dAheadDirZ;
                const tanRefD = useFrontRef ? dFront : dAhead;
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
                const t = this.yopoAvoidTanGain * Math.max(0, 1 - tanRefD / repRange);
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
                        // The memory must not lock a FINISHED detour into a full circle. Passing the
                        // obstacle centre the correct tangent flips (> 120 deg) and the memory holds
                        // the old one -- that is what it is for. But exactly the same flip happens
                        // when the corner has been rounded and the tangent should turn BACK toward
                        // the goal: the old direction is by then wide open (it points along the far
                        // side of the building, so lastOk is true), so it is kept and the drone
                        // keeps sailing around to the other side of the building -- the "the way to
                        // the goal is wide open yet it goes around the far side" report.
                        // So the memory is only honoured while it still leads roughly toward the
                        // goal; once the remembered tangent points more than ~100 deg away from the
                        // goal bearing it is dropped and the flip back is allowed.
                        const ltToGoal = (lt.x / lm) * gx + (lt.z / lm) * gz;
                        if (cos < -0.5 && lastOk && ltToGoal > this.yopoTanAwayCos) {
                            fx = lt.x * nm / lm; fz = lt.z * nm / lm;
                        }
                    }
                }
                // Second guard: a tangent that points more than ~90 deg off the goal bearing is no
                // longer steering AROUND an obstacle, it is carrying the drone away from the goal.
                // Scale it down so the goal-directed terms (network trajectory, cruise floor) win
                // back the upper hand instead of the drone being pushed around the far side.
                const nmA = Math.hypot(fx, fz);
                if (nmA > 1e-3) {
                    const fToGoal = (fx * gx + fz * gz) / nmA;
                    if (fToGoal < 0) {
                        fx *= this.yopoTanAwayScale; fz *= this.yopoTanAwayScale;
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
        // Corridor measurements, hoisted so the periodic diagnostics log at the end of this method
        // can report them (they are what decides "is the way to the goal clear").
        let dPath = R, dCmd = R, reach = R;
        if (des > 0.3 || this.yopoNavTarget) {
            const pathHalfWidth = 2.5;                      // m, flight corridor half-width (body radius + margin)
            // Deliberately the fixed base value, NOT the speed-adaptive repRange: widening the
            // action range at speed must not also make the "corridor is clear" verdict stricter,
            // otherwise the drone would start detouring on an actually clear path.
            const clearThresh = this.yopoAvoidRepRange;     // The corridor counts as clear only when there is no near obstacle in it (> the action range)
            // Only the stretch that actually has to be flown is inspected -- never past the goal.
            // Without this, a final waypoint set against a wall / tree keeps the corridor
            // "blocked" forever (the obstacle behind the goal is inside the fixed 20 m window),
            // so goalClear never fires and the side obstacles go on tilting the heading: the
            // drone then refuses to fly straight at a goal that is in fact wide open.
            // Obstacles NEARER than the goal are still checked at full strength, so genuine
            // blocking of the remaining path is detected exactly as before.
            reach = distGoalH > 0.5 ? Math.min(clearThresh, distGoalH) : clearThresh;
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
            dPath = R; dCmd = R;
            const cx = udx, cz = udz, cMag = Math.hypot(cx, cz);
            const cxn = cMag > 0.3 ? cx / cMag : gx;        // Fall back to the goal bearing when there is no valid forward speed
            const czn = cMag > 0.3 ? cz / cMag : gz;
            for (let i = 0; i < dirs.length; i++) {
                const dd = dists[i];
                if (!Number.isFinite(dd) || dd <= 0) continue;
                const dotT = dirs[i].x * gx + dirs[i].z * gz;
                if (dotT > 0) {
                    const latT = dd * Math.sqrt(Math.max(0, 1 - dotT * dotT));
                    if (latT < pathHalfWidth && dd < reach && dd < dPath) dPath = dd;
                }
                const dotC = dirs[i].x * cxn + dirs[i].z * czn;
                if (dotC > 0) {
                    const latC = dd * Math.sqrt(Math.max(0, 1 - dotC * dotC));
                    if (latC < pathHalfWidth && dd < reach && dd < dCmd) dCmd = dd;
                }
            }
            // Escape hatch for the dual-corridor check: when the commanded direction already
            // points at the goal (dot > 0.7, i.e. within ~45 deg), the two corridors sweep the
            // same volume, so demanding both is merely double-counting -- and worse, it self-locks.
            // The commanded heading is itself the PREVIOUS frame's velocity target, which already
            // had avoidance added to it: a momentary detour nudge turns it slightly off the goal,
            // that shrinks dCmd, goalClear breaks, and the very detour / vertical clearing that
            // caused the nudge stays armed even though the way to the goal is wide open. That is
            // the "path is clearly open yet it still detours / climbs" symptom.
            // When the heading really is aimed somewhere else (the network wants to go around),
            // dCmd still has to be clear on its own, so nothing is weakened there.
            // Release avoidance when EITHER corridor is open (the original rule demanded BOTH).
            // Requiring both self-locks: the commanded direction is the previous frame's velocity
            // target, which already had avoidance folded into it. Once a detour has swung the
            // heading away from the goal, dCmd looks along that detoured heading -- which by
            // definition still has the obstacle alongside it -- so dCmd stays blocked, goalClear
            // stays false, and the detour / vertical clearing can never be released even though
            // the way to the goal is wide open.
            // The earlier "command already points at the goal" escape hatch only covered
            // deviations under ~45 deg, so any detour that had swung further than that stayed
            // locked in -- that is the remaining "goal is clearly open but it still detours /
            // climbs" case.
            // Honouring either corridor keeps the useful half of the original rule: when the goal
            // line itself is blocked, the commanded corridor still has to be open before
            // avoidance stands down, so a genuine detour is not weakened.
            // EXCEPT close in: the dCmd escape hatch must not release when the bearing-to-goal
            // corridor (dPath) holds an obstacle within yopoCorridorGuardDist. Releasing there sets
            // brake = 1 and zeroes rep/tan, and since the drone is in fact driven ALONG the goal
            // bearing (the network, and the cruise floor below), it charges straight into an obstacle
            // that is only a few metres away -- the "an obstacle is right there but it still flies
            // into it" symptom. Refusing the escape hatch keeps the ray brake and the repulsion armed
            // so the drone slows and goes around instead.
            // Obstacles beyond the goal were already excluded (dd < reach <= distGoalH), so this only
            // ever bites on something genuinely between the drone and the goal.
            const pathBlockedClose = dPath < this.yopoCorridorGuardDist;
            if (dPath > reach || (dCmd > reach && !pathBlockedClose)) goalClear = true;
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
        // Vertical clearing is disabled once arrival is latched: from then on the drone is
        // holding position on the goal, and climbing would deviate from the goal and produce
        // "taking off although the way is clear" (the dual corridor / goalClear can break
        // during final trimming when the velocity direction points at a side building).
        // Vertical safety (upPush/vSafeDown) is still kept to prevent ground / ceiling hits.
        if (!this.yopoArrived && !goalClear && dAheadH < blockDist && des > 0.3 &&
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
        // Vertical-clearing (fly-over / dive-under) flag: the drone has committed to passing the
        // obstacle ABOVE / BELOW, so there is no horizontal-collision risk to pace the horizontal
        // speed for. While clearing, the proximity governor / closing gate / forward brake must all
        // stay off -- otherwise the drone crawls into the obstacle footprint instead of clearing it.
        const clearing = vRep !== 0;

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
        // The tangential detour is NOT decayed by repHold: the closer the obstacle, the MORE
        // steering authority the detour needs, and decaying it here (repHold < 1 whenever
        // dMin < standoff) is exactly the "it slows down but does not go around" symptom.
        // Raising the standoff to 7.5 m made it worse: the fade now starts 1.5 m earlier, so
        // grazing an obstacle at 5 m lateral distance used to cut the detour to ~67% right
        // where it matters most. The 0.85 floor keeps the detour decisive while still fading
        // it slightly once truly glued to the obstacle.
        const tanHold = Math.max(repHold, 0.85);
        tanX *= tanHold; tanZ *= tanHold;

        // Completely release the horizontal repulsion / tangential / braking when the exit is
        // clear, flying straight at the goal:
        // this is the cure for "always pushed away although there is no obstacle" -- as long as
        // the horizontal passage toward the goal has ample clearance (dg > clearThresh and the
        // neighbouring rays / forward cone are free), fly straight at full speed without stacking
        // any rep/tan/brake.
        // Released when the corridor is clear AND there is intent to advance. Intent to advance is
        // either an actual forward speed command (des > 0.3) OR a live navigation goal (yopoNavTarget):
        // the network often parks the local waypoint (cmdPos ~= drone pos, cmdVel ~= 0) while the
        // global goal is still tens of metres away, so des collapses below 0.3 even though the drone
        // is supposed to be progressing toward the goal. In that case the des > 0.3 test alone would
        // suppress the release, leaving brake=0 / tan huge / vCloseMax=0 with the corridor OPEN --
        // a hard dead-stop (the "movement stuck although the way is clear" symptom). A live goal makes
        // the release fire so the drone advances down the clear corridor.
        // Genuine hovering (no nav target and des <= 0.3) still keeps rep/tan to hold a safe distance
        // from obstacles. Vertical safety (upPush/vSafeDown) always applies and does not interfere
        // with straight horizontal flight.
        // Forward-cone guard: only release to full speed (brake = 1, rep/tan cleared) when the
        // forward cone dAhead is ALSO genuinely clear. The 2.5 m corridor test above can miss a
        // wide wall that no ray happened to hit inside that narrow band -- common with the sparse
        // 12-ray ring -- leaving goalClear true while an obstacle still sits in the path. dAhead
        // (cone ~+-80 deg) captures it, so when dAhead is short we KEEP the distance-based brake /
        // rep / tan instead of charging in at full speed. (Fixes "obstacle ahead but it still plans
        // that direction's speed".)
        if (goalClear && (des > 0.3 || this.yopoNavTarget) &&
            dAhead > standoff + reactionDist + 2.0) {
            repX = 0; repZ = 0;          // Horizontal repulsion fully zeroed (no 15% residual push left)
            tanX = 0; tanZ = 0;          // Tangential removed entirely (avoids detouring back to the start)
            brake = 1.0;                 // Clear exit means full speed, not slowed by vertical threats
            vRep = 0;                    // Vertical clearing released too: a clear corridor means no climbing / diving
            // Drop the tangential direction memory as soon as the corridor is open. While it is
            // kept, the NEXT obstacle re-uses the OLD tangent -- which pointed around the far side
            // of the obstacle that has just been cleared -- so the drone turned back into the detour
            // it had just escaped ("there is an exit but it goes back around"). With the corridor
            // open there is nothing to steer around, so the memory has served its purpose and the
            // next detour has to be chosen from the current geometry.
            this._avoidLastTan = null;
        }

        // ---- Horizontal detour around vertical obstacles (B) ----
        // When there is a "building / structure" straight below (vDownDist small and clearly above
        // the ground, i.e. not hugging terrain) or an obstacle straight above (vUpDist small), do
        // not apply down/up motion to "squeeze through"; instead hold altitude and use the
        // horizontal detour (vGo) to leave the obstacle footprint above/below smoothly, avoiding
        // the "wants to descend -> pushed away by rays/collision -> wants to descend again"
        // oscillation. Vertical clearing (vRep) targets "blocked horizontally straight ahead with
        // a gap above/below"; this targets "blocked straight below / above" -- the only safe path
        // is a horizontal detour. Not enabled after arrival (handed to the PD convergence).
        let vGoX = 0, vGoZ = 0;
        const vGoThresh = this.yopoAvoidVGoThresh;    // A near obstacle underfoot / overhead counts as blocking.
                                                      // PINNED, no longer yopoAvoidStop + 3.0: when flying over a
                                                      // rooftop at altitude the "straight below" ray necessarily hits
                                                      // the building, so a threshold that grows with the standoff
                                                      // causes pointless lateral detours while "the way to the goal
                                                      // is clear".
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
        if ((structBelow || aboveBlocked) && !this.yopoArrived && !goalClear) {
            // Pick the emptiest horizontal direction to leave the obstacle footprint: prefer "the
            // emptiest in the forward hemisphere", otherwise use the globally emptiest (openDir),
            // so the detour still advances toward the goal and does not turn back.
            let ox = openDirX, oz = openDirZ;
            // Clearance actually measured along the chosen escape direction -- needed for the
            // safety cap below.
            let vGoClear = openMax;
            if (ox * gx + oz * gz < 0.3) {
                let best = -1, bestD = 0.3;
                for (let i = 0; i < dirs.length; i++) {
                    const d = dists[i];
                    if (!Number.isFinite(d) || d <= 0) continue;
                    if (dirs[i].x * gx + dirs[i].z * gz <= 0.3) continue;
                    if (d > bestD) { bestD = d; best = i; }
                }
                if (best >= 0) { ox = dirs[best].x; oz = dirs[best].z; vGoClear = bestD; }
            }
            const om = Math.hypot(ox, oz) || 1;
            ox /= om; oz /= om;
            const closeness = structBelow
                ? clamp(p.vDownDist / vGoThresh, 0, 1)
                : clamp(p.vUpDist / vGoThresh, 0, 1);
            const strength = this.yopoAvoidTanGain *
                (this.yopoAvoidVGoBase + this.yopoAvoidVGoSpan * (1 - closeness));
            // Safety cap ('ray avoidance wins'): vGo is added straight onto the velocity target
            // and bypasses the forward brake, so it must carry its own limit -- never escape
            // faster than the drone can stop inside the clearance that was actually measured
            // along that direction. Without this, a strong vGo can drive the drone into a side
            // obstacle even while the forward brake is fully applied.
            // vGoSafe uses the dedicated lateral-maneuver deceleration (yopoAvoidVGoDecel), which is
            // far larger than the forward brake decel -- lateral roll can stop much harder, so the
            // escape speed is no longer strangled to ~3 m/s just because the drone flies low.
            const vGoSafe = Math.sqrt(Math.max(0,
                2 * this.yopoAvoidVGoDecel * Math.max(0, vGoClear - this.yopoAvoidStop)));
            const vGoMag = Math.min(strength, vGoSafe);
            vGoX = ox * vGoMag;
            vGoZ = oz * vGoMag;
        }

        // ---- Closing-speed budget for the authoritative gate ----
        // vCloseMax is the largest speed ALONG THE THREAT DIRECTION from which the drone can still
        // stop inside the standoff, using the same conservative deceleration as the kinematic
        // brake (identical to vSafe, including the reaction-distance buffer).
        // The call site applies it to the FINAL composed velocity target, so the brake stays
        // authoritative over the detour terms (repulsion + tangential + vGo) which are added on
        // top of the braked forward component and would otherwise bypass it entirely.
        // The gate must not throttle the drone for a threat that lies BEYOND the goal: the drone
        // stops at the goal, so that obstacle is unreachable. Without this, a goal set against a
        // wall makes dAhead small for the whole final approach, and because the reaction distance
        // grows with speed, dGate can go <= 0 -> vCloseMax becomes 0 -> the drone is forbidden
        // from moving toward a goal that is completely open (the "does not fly straight / crawls
        // or stalls at the last waypoint" symptom). Stopping at the goal is the PD's job.
        // The SAME dead-stop happens whenever goalClear is true (the corridor to the goal is open)
        // but the probe still reported a small dAhead for an obstacle OFF the goal corridor: that
        // off-corridor threat keeps dAhead <= distGoalH, so the "beyond goal" test above stays
        // false and vCloseMax pins at 0 despite the path being clear. So the gate also releases on
        // goalClear itself -- an open corridor means nothing blocks the run to the goal, so the
        // closing-speed clip is pointless and must stand down. distGoalH > 0.5 guards both releases
        // so the gate is never disabled right at the goal. A genuinely blocking threat (dAhead <=
        // distGoalH AND inside the corridor) makes goalClear false, so it still throttles normally.
        // Third release: the threat IS the goal's wall (dAhead within yopoAvoidGoalGateMargin of
        // distGoalH). goalClear is false there (the goal corridor ends at the wall), so without
        // this arm the gate clipped the goalward component to 0 (dGate = distGoalH - standoff -
        // reactionDist <= 0) and the drone could never close the last metres -- the stop belongs
        // to the final-approach PD, whose holdMaxV already guarantees a stoppable run-in.
        const gateBeyondGoal = (distGoalH > 0.5 &&
                                dAhead > distGoalH - this.yopoAvoidGoalGateMargin) ||
                               (goalClear && distGoalH > 0.5);
        const dGate = dAhead - standoff - reactionDist;
        const vCloseMax = (!dAheadHasDir || gateBeyondGoal) ? Infinity
            : (dGate > 0 ? Math.sqrt(2 * this.yopoAvoidBrakeDecel * dGate) : 0);

        // ── Proximity speed governor ──
        // Caps only the COMPONENT OF MOTION TOWARD the nearest obstacle (dMin direction), leaving
        // the tangential / slide-around component free so the drone can still detour past it. The
        // forward brake above can be bypassed when the obstacle is to the side / at an angle and the
        // plan still commands full speed; this reins the *charging-in* component in so the drone can
        // always react, but motion that is NOT toward the obstacle (sliding along the wall, going
        // around) is left unlimited. Open flight (dMin beyond ~24 m) is left at full speed.
        let nearSpeedCap = Infinity;
        let nearHasDir = false;
        // While vertically clearing an obstacle, the nearest horizontal obstacle is the one being
        // flown over / dived under -- do NOT pace the horizontal speed against it (the vertical
        // channel owns the escape). Leave the governor off so the drone can clear at full speed.
        if (Number.isFinite(dMin) && dMin < R && !clearing) {
            const dEffN = dMin - this.yopoAvoidNearStop;
            nearSpeedCap = dEffN <= 0 ? 0 : this.yopoAvoidNearK * dEffN;
            nearHasDir = true;
        }

        // ── Periodic diagnostics: everything needed to tell "the way really is blocked" from
        // "it detours around the far side although the corridor is open" ──
        // dPath / reach  = corridor clearance along the goal bearing vs. the distance inspected;
        //                  dPath > reach is the "way to the goal is clear" verdict.
        // dCmd           = the same along the commanded heading (the second corridor).
        // tanToGoal      = cosine between the detour direction and the goal bearing
        //                  (negative = the detour is carrying it AWAY from the goal).
        if ((this._avoidLogN = (this._avoidLogN || 0) + 1) % 60 === 0) {
            const tm = Math.hypot(tanX, tanZ);
            const tanToGoal = tm > 1e-3 ? (tanX * gx + tanZ * gz) / tm : 0;
            const repM = Math.hypot(repX, repZ);
            console.log(
                `_avoid goalClear=${goalClear ? 'Y' : 'N'} dPath=${dPath.toFixed(1)} ` +
                `dCmd=${dCmd.toFixed(1)} reach=${reach.toFixed(1)} distGoalH=${distGoalH.toFixed(1)} ` +
                `dAhead=${dAhead.toFixed(1)} dAheadH=${dAheadH.toFixed(1)} dMin=${dMin.toFixed(1)} ` +
                `rep=${repM.toFixed(1)} tan=${tm.toFixed(1)} tanToGoal=${tanToGoal.toFixed(2)} ` +
                `brake=${brake.toFixed(2)} vRep=${vRep.toFixed(1)} ` +
                `vGo=${Math.hypot(vGoX, vGoZ).toFixed(1)} vUp=${Number.isFinite(p.vUpDist) ? p.vUpDist.toFixed(1) : 'n/a'} ` +
                `vDown=${Number.isFinite(p.vDownDist) ? p.vDownDist.toFixed(1) : 'n/a'}`);
        }

        return { repX, repZ, tanX, tanZ, brake, upPush, vRep, vSafeDown, vSafeUp, vGoX, vGoZ,
                 threatDirX: dAheadDirX, threatDirZ: dAheadDirZ,
                 threatHasDir: dAheadHasDir, vCloseMax, nearSpeedCap,
                 nearDirX: dMinDirX, nearDirZ: dMinDirZ, nearHasDir,
                 // Diagnostic: true when the way to the goal was judged open and the avoidance
                 // terms above were released. N here while the path looks clear is the direct
                 // cause of "it still detours / climbs although the goal direction is open".
                 goalClear };
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
