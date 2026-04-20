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
 * Race-course subsystem — user-drawn, closed-loop gate track with
 * centripetal Catmull-Rom spline smoothing.
 *
 * Design summary:
 *
 *   - Deterministic, non-random: the course is EXACTLY the ordered list
 *     of points the user placed in the path editor. No retries, no seed,
 *     no RNG. Per-map JSON persistence is handled outside this module
 *     (see `src/path-store.js`).
 *   - Tangent-oriented: each gate's plane normal equals the Catmull-Rom
 *     tangent at that control point, so the drone naturally flies through
 *     each gate along the smooth curve direction.
 *   - Closed loop: the first gate doubles as start/finish line for lap
 *     timing (Phase B). Segments connect N-1 → 0 naturally.
 *   - Sensor only: gates don't collide with the drone. They exist purely
 *     as visual landmarks and pass-through triggers.
 *   - G-key gated: entities exist as soon as `rebuild()` runs but are
 *     hidden until `setVisible(true)` is called (typically by main.js
 *     on a `G` keypress).
 *
 * Lap timing (Phase B):
 *
 *   - A lap starts the first time the drone crosses gate 0 in the
 *     positive tangent direction.
 *   - Gates 1..N-1 must be crossed in order. Crossing the wrong gate, or
 *     missing one, flags the current lap as "missed" — the drone can
 *     keep flying but no lap time will be recorded this loop.
 *   - Crossing gate 0 again ends the lap. If not missed, we record
 *     `lapMs = now - lapStart`, update `bestLapMs`, and fire the
 *     `onBestLap(newBestMs)` callback so main.js can persist it.
 *   - `R` reset (via `resetLap()`): clears current-lap progress and
 *     timer, but keeps bestLap and the path itself.
 *
 * Lifecycle (from main.js):
 *
 *   gateCourse = new GateCourse();
 *   gateCourse.configure({ gateSize, clearance });
 *   gateCourse.rebuild({ app, octree, points });   // on path-editor Accept / scene load
 *   gateCourse.setVisible(true);                   // on G key
 *   gateCourse.update(dt, drone, nowMs);           // every flight frame
 *   gateCourse.resetLap();                         // on R reset
 *   gateCourse.destroy();                          // on ESC → loading
 */

import { evaluateClosed, tangentAtPoint, sampleClosed } from './catmull-rom.js';

const DEFAULT_OPTS = {
    gateSize:  1.2,    // metres — square gate, edge length
    clearance: 0.8,    // metres — used only by the editor's red/green preview
};

// Gate colors — emissive-only (no lighting contribution), chosen for
// max contrast against bright 3DGS backgrounds.
//   start      → gate 0, the launch / finish line (chartreuse, pulses
//                when it is also the next gate)
//   next       → the gate you must fly next (yellow, pulses)
//   upcoming   → any other gate within the horizon window (cyan)
// The finish gate (index N-1) carries a checker-flag texture instead
// of a solid colour — see `getCheckerTexture` below.
const COL_START    = { r: 0.40, g: 1.00, b: 0.10 }; // chartreuse
const COL_NEXT     = { r: 1.00, g: 0.84, b: 0.00 }; // yellow
const COL_UPCOMING = { r: 0.00, g: 0.75, b: 1.00 }; // cyan

// How many gates ahead of `nextGateIdx` remain visible (inclusive of
// the next gate itself). Gates behind the drone naturally fall outside
// this window via the wrap-around distance, and so do gates far ahead,
// keeping the cockpit view clean while still previewing the first few
// gates of the next lap as the current lap winds down.
const HORIZON = 5;

// Emissive strength used by every gate material. 3.5 was chosen
// empirically against bright 3DGS scenes where anything below ~3 gets
// washed out by the cloud.
const EMISSIVE_INTENSITY = 3.5;

// ------------------------------------------------------------------
// Procedural canvas textures (finish-gate checker & gate-0 START sign)
// ------------------------------------------------------------------
//
// Both textures are built from HTMLCanvasElements on first use so no
// binary assets ship with the module. Each is cached at module scope
// so the same pc.Texture is reused across rebuilds within a session.

/**
 * 2×2 black/white checker. One pixel per square so the pattern stays
 * razor-sharp under NEAREST filtering; per-edge materials control how
 * many times the texture repeats along each bar so the squares end up
 * roughly equal-size regardless of the bar's aspect ratio.
 */
let _checkerTex = null;
function getCheckerTexture(app) {
    if (_checkerTex) return _checkerTex;
    if (!app || !app.graphicsDevice) return null;
    /* global pc */
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = 2;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#ffffff'; ctx.fillRect(0, 0, 1, 1); ctx.fillRect(1, 1, 1, 1);
    ctx.fillStyle = '#000000'; ctx.fillRect(1, 0, 1, 1); ctx.fillRect(0, 1, 1, 1);
    const tex = new pc.Texture(app.graphicsDevice, {
        format: pc.PIXELFORMAT_RGBA8,
        width: 2, height: 2,
        mipmaps: false,
        addressU: pc.ADDRESS_REPEAT,
        addressV: pc.ADDRESS_REPEAT,
        magFilter: pc.FILTER_NEAREST,
        minFilter: pc.FILTER_NEAREST,
    });
    tex.setSource(canvas);
    _checkerTex = tex;
    return tex;
}

/**
 * "START" sign panel texture — bold black letters on a white plate with
 * a thin black border so the label reads at a distance when glowing
 * under the high emissive intensity.
 */
let _startLabelTex = null;
function getStartLabelTexture(app) {
    if (_startLabelTex) return _startLabelTex;
    if (!app || !app.graphicsDevice) return null;
    /* global pc */
    const W = 512, H = 128;
    const canvas = document.createElement('canvas');
    canvas.width = W; canvas.height = H;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#ffffff'; ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = '#000000'; ctx.lineWidth = 6; ctx.strokeRect(3, 3, W - 6, H - 6);
    ctx.fillStyle = '#000000';
    ctx.font = 'bold 96px "Arial Black", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('START', W / 2, H / 2 + 4);
    const tex = new pc.Texture(app.graphicsDevice, {
        format: pc.PIXELFORMAT_RGBA8,
        width: W, height: H,
        mipmaps: true,
        addressU: pc.ADDRESS_CLAMP_TO_EDGE,
        addressV: pc.ADDRESS_CLAMP_TO_EDGE,
        magFilter: pc.FILTER_LINEAR,
        minFilter: pc.FILTER_LINEAR_MIPMAP_LINEAR,
    });
    tex.setSource(canvas);
    _startLabelTex = tex;
    return tex;
}

// Pulse animation (next gate only): +/- 8 % scale @ 2 Hz.
const PULSE_AMP = 0.08;
const PULSE_HZ  = 2.0;

export class GateCourse {
    constructor() {
        this.opts = { ...DEFAULT_OPTS };

        /**
         * @type {Array<{
         *   pos: {x:number,y:number,z:number},
         *   travelDir: {x:number,y:number,z:number},
         *   passed: boolean,
         *   entity: any,
         *   material: any,
         *   invWorld: any,
         * }>}
         */
        this.gates = [];
        this.nextGateIdx = 0;

        // Private state
        this._rootEntity = null;
        this._pulseTime = 0;
        this._prevDronePos = null;
        this._visible = false;

        // Lap-timing state (Phase B). Strict-in-order rule (see
        // `_handleCrossing`) means an out-of-order cross is silently
        // ignored, so there is no "missed lap" concept — either you
        // flew the gates in sequence, or the lap simply hasn't
        // completed yet.
        this.lapStart      = null;   // ms timestamp when gate 0 was last crossed cleanly
        this.lapCount      = 0;      // completed laps this session
        this.bestLapMs     = null;   // best lap this session OR loaded from disk
        this.currentLapMs  = 0;      // live for HUD

        // Callbacks wired by main.js.
        this.onGatePassed = null;
        this.onLapComplete = null;   // (lapMs, isBest) → void; main.js persists best
        this.onBestLap    = null;    // (newBestMs) → void; subset of onLapComplete when isBest
    }

    /** Merge new options. Does not rebuild by itself. */
    configure(opts) {
        this.opts = { ...this.opts, ...(opts || {}) };
    }

    /** Restore a previously-persisted best lap (before the first rebuild). */
    setBestLapMs(ms) {
        this.bestLapMs = (Number.isFinite(Number(ms)) && Number(ms) > 0) ? Number(ms) : null;
    }

    /**
     * Rebuild the gate entities from an ordered list of user-placed
     * control points. Destroys any existing gates first.
     *
     * @param {{
     *   app:     any,                           // pc.Application
     *   octree?: any,                           // present, unused — kept for symmetry with old API
     *   points:  Array<{x:number,y:number,z:number}>,   // >= 3 for a valid closed loop
     * }} ctx
     */
    rebuild(ctx) {
        this.destroy();

        const points = Array.isArray(ctx.points) ? ctx.points : [];
        if (points.length < 3) {
            // Not enough points for a closed loop — leave the course empty
            // and let the UI status messaging explain.
            return;
        }

        // Materialise each control point as a gate with orientation taken
        // from the Catmull-Rom tangent at that point.
        const gates = new Array(points.length);
        for (let i = 0; i < points.length; i++) {
            const td = tangentAtPoint(points, i);
            gates[i] = {
                pos:       { x: points[i].x, y: points[i].y, z: points[i].z },
                travelDir: td,
                passed:    false,
                entity:    null,
                material:  null,
                invWorld:  null,
                // `fixedTint` gates (currently just the finish gate) carry
                // a texture that must not be overwritten by the per-frame
                // color-state repaint in `_updateGateAppearance`.
                fixedTint: false,
            };
        }
        this.gates = gates;
        this.nextGateIdx = 0;

        // Build PlayCanvas entities (optional when unit-testing without an app).
        this._app = ctx.app || null;
        if (ctx.app) {
            /* global pc */
            this._rootEntity = new pc.Entity('race-course');
            ctx.app.root.addChild(this._rootEntity);
            for (let i = 0; i < gates.length; i++) {
                const ent = this._buildGateEntity(gates[i], i);
                this._rootEntity.addChild(ent);
                gates[i].entity = ent;
                // Cache world-to-local BEFORE any pulse scaling so pass
                // detection uses a stable gate plane even while the visual
                // pulse throbs.
                const inv = new pc.Mat4();
                inv.copy(ent.getWorldTransform()).invert();
                gates[i].invWorld = inv;
            }
            // Paint everything once the whole gates[] array is populated
            // so the horizon-based visibility logic sees the final N.
            this._updateAllAppearances();
            // Apply current visibility (entities default to enabled; if the
            // user hasn't pressed G yet we want them hidden).
            this._applyVisibility();
        }

        this._prevDronePos = null;
        this._pulseTime = 0;
        this.resetLap();
    }

    /**
     * Show / hide the whole gate course in the scene. Pass detection still
     * runs only when visible — we treat the course as "not engaged" when
     * the user has toggled it off via G.
     */
    setVisible(on) {
        this._visible = !!on;
        this._applyVisibility();
        if (!on) {
            // Hiding mid-flight: clear current-lap state so a later re-show
            // doesn't resurrect a stale timer.
            this.resetLap();
        }
    }

    isVisible() { return this._visible; }

    _applyVisibility() {
        if (!this._rootEntity) return;
        try { this._rootEntity.enabled = this._visible; } catch (_) { /* no-op */ }
    }

    /**
     * Sampled points along the smooth Catmull-Rom curve, for editor / debug
     * rendering. Returns [] when < 3 gates. Output is a flat list of
     * `{pos, tangent}` entries (see catmull-rom.js#sampleClosed).
     */
    sampleCurve(samplesPerSegment = 24) {
        const points = this.gates.map(g => g.pos);
        return sampleClosed(points, samplesPerSegment);
    }

    /**
     * Build a PlayCanvas entity for one gate — four thin boxes forming a
     * square outline in the gate's local XY plane; local +Z = travel
     * direction. Orientation is derived from the Catmull-Rom tangent so
     * the gate plane is naturally perpendicular to the curve.
     *
     * A per-gate StandardMaterial is created here so colors can change
     * independently without leaking between gates.
     */
    _buildGateEntity(gate, index) {
        /* global pc */
        const size = this.opts.gateSize;
        const half = size * 0.5;
        const thickness = Math.max(0.06, size * 0.10); // 10 % of size, min 6 cm

        const e = new pc.Entity(`gate-${index}`);
        e.setPosition(gate.pos.x, gate.pos.y, gate.pos.z);

        // Build an orthonormal basis with local +Z = travelDir. Runtime is
        // always Y-up (see ply-parser.js — all source coords transformed
        // to Y-up at parse time), so worldUp is world +Y.
        const fwd = new pc.Vec3(gate.travelDir.x, gate.travelDir.y, gate.travelDir.z);
        const worldUp = new pc.Vec3(0, 1, 0);
        const right = new pc.Vec3();
        right.cross(worldUp, fwd);
        if (right.lengthSq() < 1e-6) {
            // travelDir parallel to worldUp (drone flying straight up/down
            // between two identically-placed gates). Pick any perpendicular.
            if (Math.abs(fwd.x) < 0.9) right.set(1, 0, 0);
            else right.set(0, 1, 0);
            const dot = right.dot(fwd);
            right.sub(new pc.Vec3(fwd.x * dot, fwd.y * dot, fwd.z * dot));
        }
        right.normalize();
        const up = new pc.Vec3();
        up.cross(fwd, right).normalize();

        // Column-major rotation matrix with basis (right, up, fwd).
        const m = new pc.Mat4();
        const d = m.data;
        d[0] = right.x; d[1] = right.y; d[2]  = right.z; d[3]  = 0;
        d[4] = up.x;    d[5] = up.y;    d[6]  = up.z;    d[7]  = 0;
        d[8] = fwd.x;   d[9] = fwd.y;   d[10] = fwd.z;   d[11] = 0;
        d[12] = 0;      d[13] = 0;      d[14] = 0;       d[15] = 1;

        const q = new pc.Quat().setFromMat4(m);
        e.setRotation(q);

        // Four edges of the square outline, positioned in the gate's
        // local XY plane. `tileU/tileV` only matter for the finish gate
        // — they make the checker squares come out roughly square on
        // each bar despite the bars' different aspect ratios.
        const edges = [
            { pos: [0,  half, 0], scale: [size + thickness, thickness, thickness], tileU: 6, tileV: 1 }, // top
            { pos: [0, -half, 0], scale: [size + thickness, thickness, thickness], tileU: 6, tileV: 1 }, // bottom
            { pos: [-half, 0, 0], scale: [thickness, size - thickness, thickness], tileU: 1, tileV: 5 }, // left
            { pos: [ half, 0, 0], scale: [thickness, size - thickness, thickness], tileU: 1, tileV: 5 }, // right
        ];

        // Finish gate (last in the closed loop) wears a checker-flag
        // texture so it is instantly recognisable as the lap terminator,
        // just like real motorsport circuits.
        const isFinish = (index === this.gates.length - 1);
        const makeMat = (tileU, tileV) => {
            const m = new pc.StandardMaterial();
            if (isFinish) {
                const tex = getCheckerTexture(this._app);
                if (tex) {
                    m.diffuseMap  = tex;
                    m.emissiveMap = tex;
                    m.diffuseMapTiling  = new pc.Vec2(tileU, tileV);
                    m.emissiveMapTiling = new pc.Vec2(tileU, tileV);
                }
                m.diffuse.set(1, 1, 1);
                m.emissive.set(1, 1, 1);
            } else {
                m.diffuse.set(COL_UPCOMING.r, COL_UPCOMING.g, COL_UPCOMING.b);
                m.emissive.set(COL_UPCOMING.r, COL_UPCOMING.g, COL_UPCOMING.b);
            }
            m.emissiveIntensity = EMISSIVE_INTENSITY;
            m.useLighting       = false;
            m.useFog            = false;
            m.opacity           = 1.0;
            m.update();
            return m;
        };

        // Non-finish gates share a single material across all four bars
        // (so `_updateGateAppearance` can repaint with one call). The
        // finish gate needs one material per bar so each bar can carry
        // its own checker tiling.
        let edgeMats;
        if (isFinish) {
            edgeMats = edges.map(ed => makeMat(ed.tileU, ed.tileV));
            gate.fixedTint = true;
        } else {
            const shared = makeMat(1, 1);
            edgeMats = [shared, shared, shared, shared];
        }
        gate.material = edgeMats[0];

        for (let ei = 0; ei < edges.length; ei++) {
            const edge = edges[ei];
            const seg = new pc.Entity();
            seg.addComponent('render', { type: 'box' });
            const mis = seg.render.meshInstances;
            for (let mi = 0; mi < mis.length; mi++) mis[mi].material = edgeMats[ei];
            seg.setLocalPosition(edge.pos[0], edge.pos[1], edge.pos[2]);
            seg.setLocalScale(edge.scale[0], edge.scale[1], edge.scale[2]);
            e.addChild(seg);
        }

        // Gate 0 gets a floating "START" sign above the top bar so it
        // reads unambiguously as the launch position even when the frame
        // itself is partially obscured.
        if (index === 0) {
            const tex = getStartLabelTexture(this._app);
            if (tex) {
                const sign = new pc.Entity('gate-0-sign');
                sign.addComponent('render', { type: 'box' });
                const signMat = new pc.StandardMaterial();
                signMat.diffuseMap        = tex;
                signMat.emissiveMap       = tex;
                signMat.diffuse.set(1, 1, 1);
                signMat.emissive.set(1, 1, 1);
                signMat.emissiveIntensity = EMISSIVE_INTENSITY;
                signMat.useLighting       = false;
                signMat.useFog            = false;
                signMat.opacity           = 1.0;
                signMat.update();
                const signMis = sign.render.meshInstances;
                for (let mi = 0; mi < signMis.length; mi++) signMis[mi].material = signMat;
                // Plate size: full gate width, ~1/4 of gate height, thin.
                sign.setLocalPosition(0, half + size * 0.22, 0);
                sign.setLocalScale(size * 0.9, size * 0.28, thickness * 0.6);
                e.addChild(sign);
            }
        }

        // Force world-transform recompute so invWorld (captured by the
        // caller) reflects the final rotation.
        e.getWorldTransform();
        return e;
    }

    /**
     * Repaint and show/hide one gate based on the current state.
     *
     * Visibility: circular-distance horizon only. A gate is shown iff
     * `(index - nextGateIdx + N) % N < HORIZON`. This one rule covers
     * three cases at once:
     *   - the current `next` gate (distance 0) is always visible;
     *   - gates just flown through wrap to very large distances and
     *     vanish behind the drone;
     *   - gates at the start of the NEXT lap slide into view as
     *     `nextGateIdx` nears the end of the current lap, giving a
     *     seamless rollover between laps.
     */
    _updateGateAppearance(gate, index) {
        if (!gate || !gate.material) return;
        const N = this.gates.length;

        // Horizon-only visibility. Circular distance already handles the
        // two natural hide cases: gates we just flew through end up with
        // very large distance and vanish, while gates at the start of
        // the next lap slide into view as `nextGateIdx` approaches the
        // end of the current lap — giving a seamless rollover.
        const dist = (index - this.nextGateIdx + N) % N;
        const visible = dist < HORIZON;
        if (gate.entity) {
            try { gate.entity.enabled = visible; } catch (_) { /* no-op */ }
        }
        if (!visible) return;

        // Gates with a fixed texture (currently just the finish gate's
        // chequered flag) must not have their material diffuse/emissive
        // overwritten — doing so would tint the checker pattern.
        if (gate.fixedTint) return;

        // Gate 0 is the launch / finish line and always shows as
        // chartreuse, even when it is the next gate — the pulse scale
        // animation (applied elsewhere) plus the extra horizontal
        // crossbar built into its geometry already signal "this is the
        // one to fly through", so we don't override with yellow.
        const isNext = (index === this.nextGateIdx);
        let c;
        if (index === 0)             c = COL_START;
        else if (isNext)             c = COL_NEXT;
        else                          c = COL_UPCOMING;
        gate.material.diffuse.set(c.r, c.g, c.b);
        gate.material.emissive.set(c.r, c.g, c.b);
        gate.material.update();
    }

    /**
     * Repaint every gate. Cheap (<= 20 gates typically) and needed on
     * any state change that may flip visibility for multiple gates at
     * once (next-index advance, lap reset, path rebuild).
     */
    _updateAllAppearances() {
        for (let i = 0; i < this.gates.length; i++) {
            this._updateGateAppearance(this.gates[i], i);
        }
    }

    /**
     * Per-frame update — pulses the next gate, runs pass detection, and
     * advances the lap timer. Safe to call even when the course is
     * empty, hidden, or the drone is missing.
     *
     * @param {number} dt       seconds since last frame
     * @param {Drone}  drone    drone with .x/.y/.z
     * @param {number} [nowMs]  monotonic timestamp for lap timing; defaults
     *                          to performance.now() so callers rarely need
     *                          to supply it.
     */
    update(dt, drone, nowMs) {
        if (!this.gates.length || !this._visible) {
            // Keep HUD silent when hidden: clear currentLapMs so the lap
            // readout doesn't display stale milliseconds.
            this.currentLapMs = 0;
            return;
        }

        // Pulse the current "next" gate.
        this._pulseTime += dt;
        const s = 1 + PULSE_AMP * Math.sin(this._pulseTime * PULSE_HZ * Math.PI * 2);
        const nextG = this.gates[this.nextGateIdx];
        if (nextG && nextG.entity) nextG.entity.setLocalScale(s, s, s);

        if (!drone) return;
        const now = Number.isFinite(nowMs) ? nowMs : (typeof performance !== 'undefined' ? performance.now() : Date.now());
        const curr = { x: drone.x, y: drone.y, z: drone.z };

        // Live lap clock.
        if (this.lapStart != null) this.currentLapMs = now - this.lapStart;

        if (this._prevDronePos) {
            for (let i = 0; i < this.gates.length; i++) {
                const g = this.gates[i];
                if (!g.entity) continue;
                if (!this._checkCross(g, this._prevDronePos, curr)) continue;

                this._handleCrossing(i, now);
            }
        }
        this._prevDronePos = curr;
    }

    /**
     * React to a confirmed drone-through-gate crossing.
     *
     * Strict in-order rule: anything other than the currently-expected
     * `nextGateIdx` is silently ignored — no mark-passed, no colour
     * change, no HUD update. The player must fly the yellow (or
     * chartreuse-pulsing, for gate 0) gate before any other counts.
     *
     * @param {number} i      gate index just crossed
     * @param {number} now    ms timestamp of the cross
     */
    _handleCrossing(i, now) {
        if (i !== this.nextGateIdx) {
            console.log(`[Race] ignored out-of-order cross: expected gate ${this.nextGateIdx + 1}, got ${i + 1}`);
            return;
        }

        if (i === 0) {
            // Gate 0 is the lap boundary. `lapStart != null` means we
            // have been around the loop at least once — record that lap
            // before resetting. First-ever cross just starts the timer.
            if (this.lapStart != null) {
                const lapMs = now - this.lapStart;
                this.lapCount += 1;
                let isBest = false;
                if (this.bestLapMs == null || lapMs < this.bestLapMs) {
                    this.bestLapMs = lapMs;
                    isBest = true;
                }
                this._notifyLap(lapMs, isBest);
            }
            // Fresh lap in both cases: clear pass flags, restart timer.
            this._beginLap(now);
        }

        this._markPassed(i);
    }

    _markPassed(i) {
        const g = this.gates[i];
        if (!g || g.passed) return;
        g.passed = true;
        // Advance nextGateIdx to the first still-unpassed gate (circular).
        // We scan forward from (i + 1) % N, stopping at the first gap.
        const N = this.gates.length;
        let k = (i + 1) % N;
        for (let step = 0; step < N; step++) {
            if (!this.gates[k].passed) break;
            k = (k + 1) % N;
        }
        this.nextGateIdx = k;

        // Reset the pulse scale of the just-passed gate.
        if (g.entity) g.entity.setLocalScale(1, 1, 1);

        // Visibility depends on `nextGateIdx`, so every advance may
        // reveal the newly-in-horizon gate at the far end and hide the
        // old `nextGateIdx - 0` slot — full refresh is cheapest.
        this._updateAllAppearances();

        console.log(`[Race] passed gate ${i + 1} / ${N}`);
        if (typeof this.onGatePassed === 'function') {
            try { this.onGatePassed(i, N); }
            catch (e) { console.warn('[Race] onGatePassed callback error:', e); }
        }
    }

    /**
     * Begin a fresh lap — clear pass flags on all gates and restart
     * the timer. Does NOT mark gate 0 as passed; caller must do that
     * after calling _beginLap (see _handleCrossing).
     */
    _beginLap(now) {
        for (let i = 0; i < this.gates.length; i++) {
            this.gates[i].passed = false;
            if (this.gates[i].entity) this.gates[i].entity.setLocalScale(1, 1, 1);
        }
        this.nextGateIdx  = 0;
        this.lapStart     = now;
        this.currentLapMs = 0;
        this._updateAllAppearances();
    }

    _notifyLap(lapMs, isBest) {
        console.log(`[Race] lap ${this.lapCount}: ${(lapMs / 1000).toFixed(3)} s${isBest ? ' — NEW BEST' : ''}`);
        if (typeof this.onLapComplete === 'function') {
            try { this.onLapComplete(lapMs, isBest); }
            catch (e) { console.warn('[Race] onLapComplete callback error:', e); }
        }
        if (isBest && typeof this.onBestLap === 'function') {
            try { this.onBestLap(lapMs); }
            catch (e) { console.warn('[Race] onBestLap callback error:', e); }
        }
    }

    /**
     * Line-vs-plane crossing test in gate-local space. Returns true iff
     * the segment (prev→curr) crosses the gate's XY plane AND the
     * intersection point lies within the gate's rectangle (±half, ±half).
     */
    _checkCross(gate, prev, curr) {
        if (!gate.invWorld) return false;
        /* global pc */
        const p0 = new pc.Vec3(prev.x, prev.y, prev.z);
        const p1 = new pc.Vec3(curr.x, curr.y, curr.z);
        gate.invWorld.transformPoint(p0, p0);
        gate.invWorld.transformPoint(p1, p1);

        if (p0.z * p1.z > 0) return false;
        const denom = p0.z - p1.z;
        if (Math.abs(denom) < 1e-9) return false;

        const t = p0.z / denom;
        const hx = p0.x + (p1.x - p0.x) * t;
        const hy = p0.y + (p1.y - p0.y) * t;
        const half = this.opts.gateSize * 0.5;
        return Math.abs(hx) <= half && Math.abs(hy) <= half;
    }

    /**
     * Clear the current lap's progress without touching the layout or
     * best-lap record. Called on `R` reset and whenever the course is
     * toggled off via G.
     */
    resetLap() {
        for (let i = 0; i < this.gates.length; i++) {
            this.gates[i].passed = false;
            if (this.gates[i].entity) this.gates[i].entity.setLocalScale(1, 1, 1);
        }
        this.nextGateIdx    = 0;
        this.lapStart       = null;
        this.currentLapMs   = 0;
        this._prevDronePos  = null;
        this._updateAllAppearances();
    }

    /**
     * Back-compat shim: older code calls `resetProgress()` on the drone
     * reset event. Forward to the Phase B implementation.
     */
    resetProgress() { this.resetLap(); }

    /** Tear down all PlayCanvas entities and clear state. */
    destroy() {
        if (this._rootEntity) {
            try { this._rootEntity.destroy(); } catch (_) { /* already dead */ }
            this._rootEntity = null;
        }
        this.gates = [];
        this.nextGateIdx   = 0;
        this._prevDronePos = null;
        this._pulseTime    = 0;
        this.lapStart      = null;
        this.currentLapMs  = 0;
        this.lapCount      = 0;
        // bestLapMs intentionally preserved — survives scene-exit so
        // the persisted record isn't clobbered by a transient destroy().
    }

    /** UI status — e.g. "5 gates · best 00:39.1" or "no path drawn". */
    statusText() {
        if (!this.gates.length) return 'no path drawn';
        const gates = `${this.gates.length} gates`;
        if (this.bestLapMs != null) return `${gates} · best ${formatLap(this.bestLapMs)}`;
        return gates;
    }

    /** Count of passed gates this lap — used by HUD. */
    passedCount() {
        let n = 0;
        for (const g of this.gates) if (g.passed) n++;
        return n;
    }
}

// ---- Exports for HUD formatting ------------------------------------
/**
 * Format a millisecond duration as `MM:SS.mmm`. Exported so hud.js can
 * render lap times in the same style the Race Course module uses
 * internally for logging.
 */
export function formatLap(ms) {
    if (!Number.isFinite(ms) || ms < 0) return '--:--.---';
    const totalMs = Math.round(ms);
    const minutes = Math.floor(totalMs / 60000);
    const seconds = Math.floor((totalMs % 60000) / 1000);
    const millis  = totalMs % 1000;
    return `${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}.${String(millis).padStart(3, '0')}`;
}

/**
 * Helper: sample the Catmull-Rom-derived gate tangent at control point i.
 * Exposed for the path editor, which shows tangent-direction arrows in
 * the top-down canvas so the user knows which way each gate will face.
 */
export { evaluateClosed, tangentAtPoint, sampleClosed };
