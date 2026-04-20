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
 * Centripetal Catmull-Rom curve evaluator, closed-loop only.
 *
 * A centripetal Catmull-Rom spline (alpha = 0.5) passes through every control
 * point and, unlike uniform CR, is guaranteed loop-free and cusp-free even
 * when the spacing between control points varies wildly. That property is
 * what we rely on for the gate editor: the spline must visit every gate the
 * user places, in order, with smooth curvature — regardless of whether the
 * user drops them 1 m or 100 m apart.
 *
 * Closed-loop wrap: for N control points P[0..N-1], segment i connects
 * P[i] and P[(i+1) mod N] using P[(i-1+N) mod N] and P[(i+2) mod N] as
 * the outer guide points. Requires N >= 3 for a non-degenerate loop;
 * callers (path editor) gate save-to-disk on that threshold.
 *
 * All points are {x, y, z} POJOs. Tangents are returned as unit vectors
 * pointing along the travel direction (from P[i] toward P[i+1]).
 */

const ALPHA = 0.5;           // centripetal parameterization
const EPS   = 1e-6;          // min segment length to avoid /0 in parameterization
const DELTA = 1e-4;          // finite-difference step for tangent sampling

/** Euclidean distance between two {x,y,z} points. */
function dist(a, b) {
    const dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/** Linear interpolate two points by scalar s; s=0 → a, s=1 → b. */
function lerpPt(a, b, s) {
    return {
        x: a.x + (b.x - a.x) * s,
        y: a.y + (b.y - a.y) * s,
        z: a.z + (b.z - a.z) * s,
    };
}

/**
 * Barycentric evaluation of a single centripetal Catmull-Rom segment at
 * internal parameter u (u in [t1, t2]). Returns just the position.
 *
 * Classic Barry-Goldman 3-level lerp formulation: more verbose than the
 * standard cubic polynomial form but numerically stable for extreme
 * spacing ratios and works with any alpha.
 */
function _posAt(P0, P1, P2, P3, t0, t1, t2, t3, u) {
    // Guard parameterization — if two consecutive points coincide (d = 0),
    // the knot deltas collapse; clamping avoids NaN.
    const d01 = Math.max(t1 - t0, EPS);
    const d12 = Math.max(t2 - t1, EPS);
    const d23 = Math.max(t3 - t2, EPS);
    const d02 = Math.max(t2 - t0, EPS);
    const d13 = Math.max(t3 - t1, EPS);

    const A1 = lerpPt(P0, P1, (u - t0) / d01);
    const A2 = lerpPt(P1, P2, (u - t1) / d12);
    const A3 = lerpPt(P2, P3, (u - t2) / d23);
    const B1 = lerpPt(A1, A2, (u - t0) / d02);
    const B2 = lerpPt(A2, A3, (u - t1) / d13);
    return lerpPt(B1, B2, (u - t1) / d12);
}

/**
 * Evaluate the closed-loop Catmull-Rom curve on segment `i` (connecting
 * points[i] → points[(i+1) % N]) at normalized parameter t in [0, 1].
 *
 * @param {Array<{x,y,z}>} points   control points, must have length >= 3
 * @param {number} i                segment index, 0..N-1
 * @param {number} t                segment-local parameter, 0..1
 * @returns {{ pos: {x,y,z}, tangent: {x,y,z} }}
 *   `tangent` is unit-length in the direction of increasing t (i.e.
 *   pointing from points[i] toward points[i+1] along the spline).
 */
export function evaluateClosed(points, i, t) {
    const N = points.length;
    if (N < 3) throw new Error('closed Catmull-Rom requires >= 3 points');

    const P0 = points[(i - 1 + N) % N];
    const P1 = points[i % N];
    const P2 = points[(i + 1) % N];
    const P3 = points[(i + 2) % N];

    // Centripetal knots: ti = t_{i-1} + dist(P_{i-1}, Pi)^alpha.
    // Guard against degenerate (coincident) points by clamping d >= EPS.
    const t0 = 0;
    const t1 = t0 + Math.max(Math.pow(dist(P0, P1), ALPHA), EPS);
    const t2 = t1 + Math.max(Math.pow(dist(P1, P2), ALPHA), EPS);
    const t3 = t2 + Math.max(Math.pow(dist(P2, P3), ALPHA), EPS);

    const u = t1 + Math.max(0, Math.min(1, t)) * (t2 - t1);

    const pos = _posAt(P0, P1, P2, P3, t0, t1, t2, t3, u);

    // Tangent via central difference. Cheaper to derive than the closed
    // form and numerically robust for the curvature ratios we care about.
    // Clamp near the segment ends so we don't step outside [t1, t2];
    // direction is what matters, not absolute magnitude.
    const du   = DELTA * (t2 - t1);
    const uLo  = Math.max(u - du, t1);
    const uHi  = Math.min(u + du, t2);
    const pLo  = _posAt(P0, P1, P2, P3, t0, t1, t2, t3, uLo);
    const pHi  = _posAt(P0, P1, P2, P3, t0, t1, t2, t3, uHi);
    let tx = pHi.x - pLo.x, ty = pHi.y - pLo.y, tz = pHi.z - pLo.z;
    const len = Math.sqrt(tx * tx + ty * ty + tz * tz);
    if (len > 1e-9) { tx /= len; ty /= len; tz /= len; }
    else {
        // Degenerate (likely coincident neighbours). Fall back to the chord
        // direction so the caller still has a usable orientation.
        tx = P2.x - P1.x; ty = P2.y - P1.y; tz = P2.z - P1.z;
        const clen = Math.sqrt(tx * tx + ty * ty + tz * tz) || 1;
        tx /= clen; ty /= clen; tz /= clen;
    }

    return { pos, tangent: { x: tx, y: ty, z: tz } };
}

/**
 * Sample the entire closed-loop curve into a flat array of
 * `samplesPerSegment` points per segment (evenly spaced in segment t, not
 * in arc length — fine for visual preview; if callers want arc-length
 * uniform sampling they can reparameterize downstream).
 *
 * Each entry is `{ pos, tangent }`. The last sample of segment i is not
 * repeated as the first of segment i+1 — the returned array is exactly
 * `samplesPerSegment * N` entries, tracing the full loop once and closing
 * naturally (consumer can draw line strip with `closePath()` or repeat
 * the first point).
 *
 * @param {Array<{x,y,z}>} points
 * @param {number} samplesPerSegment   typical: 16..32 for editor preview
 * @returns {Array<{ pos: {x,y,z}, tangent: {x,y,z} }>}
 */
export function sampleClosed(points, samplesPerSegment = 24) {
    const N = points.length;
    if (N < 3) return [];
    const out = [];
    for (let i = 0; i < N; i++) {
        for (let s = 0; s < samplesPerSegment; s++) {
            const t = s / samplesPerSegment;
            out.push(evaluateClosed(points, i, t));
        }
    }
    return out;
}

/**
 * Compute the unit tangent at a control point (not mid-segment) as the
 * average of the outgoing tangent of the previous segment and the
 * incoming tangent of the next segment. This is the orientation used
 * for the gate sitting AT the control point.
 *
 * Returns a unit vector. For pathological inputs (all coincident) falls
 * back to +X.
 */
export function tangentAtPoint(points, i) {
    const N = points.length;
    if (N < 3) return { x: 1, y: 0, z: 0 };
    // Incoming tangent = direction at t=1 of segment (i-1), outgoing = t=0 of segment i.
    const inc = evaluateClosed(points, (i - 1 + N) % N, 1).tangent;
    const out = evaluateClosed(points, i % N, 0).tangent;
    let tx = inc.x + out.x;
    let ty = inc.y + out.y;
    let tz = inc.z + out.z;
    const len = Math.sqrt(tx * tx + ty * ty + tz * tz);
    if (len > 1e-9) return { x: tx / len, y: ty / len, z: tz / len };
    return { x: 1, y: 0, z: 0 };
}
