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
 * Collision adapter for Cesium / Google Photorealistic 3D Tiles.
 *
 * Cesium does not expose Google tiles as a CPU physics mesh. This provider
 * therefore builds a conservative local proxy from currently loaded render data:
 *
 *   - `scene.sampleHeight()` handles vertical roof / terrain overlap.
 *   - `scene.pickFromRay()` probes the current sphere neighborhood.
 *   - A swept ray from previous position to current position catches direct
 *     wall impacts that would otherwise tunnel through between frames.
 */

function normalize(v, fallback = { x: 0, y: 1, z: 0 }) {
    const len = Math.hypot(v.x, v.y, v.z);
    if (len < 1e-8) return fallback;
    return { x: v.x / len, y: v.y / len, z: v.z / len };
}

function hitNormalFromPoint(cx, cy, cz, p, fallback) {
    return normalize({ x: cx - p.x, y: cy - p.y, z: cz - p.z }, fallback);
}

function horizontalDistance(a, b) {
    return Math.hypot(a.x - b.x, a.z - b.z);
}

export class TilesCollisionProvider {
    constructor(world, options = {}) {
        this.world = world;
        this.enabled = true;
        this.verticalSkin = Number.isFinite(options.verticalSkin) ? options.verticalSkin : 0.03;
        this.horizontalSkin = Number.isFinite(options.horizontalSkin) ? options.horizontalSkin : 0.07;
        this.rayExtra = Number.isFinite(options.rayExtra) ? options.rayExtra : 0.45;
        this.sweptExtra = Number.isFinite(options.sweptExtra) ? options.sweptExtra : 0.35;
        this.heightProbeWidth = Number.isFinite(options.heightProbeWidth) ? options.heightProbeWidth : 0.35;
        this.minPenetration = Number.isFinite(options.minPenetration) ? options.minPenetration : 0.015;
        this.minRayDistance = Number.isFinite(options.minRayDistance) ? options.minRayDistance : 0.04;
        this.lastDebug = null;

        this._rayDirs = [
            { x: 1, y: 0, z: 0 },
            { x: -1, y: 0, z: 0 },
            { x: 0, y: 0, z: 1 },
            { x: 0, y: 0, z: -1 },
            { x: 0.7071, y: 0, z: 0.7071 },
            { x: -0.7071, y: 0, z: 0.7071 },
            { x: 0.7071, y: 0, z: -0.7071 },
            { x: -0.7071, y: 0, z: -0.7071 },
        ];
    }

    queryCollisionResponse(x, y, z, radius, state = {}) {
        if (!this.enabled || !this.world || !this.world.ready) return null;

        const hits = [];
        const r = Math.max(0.05, radius || 0.3);
        const center = { x, y, z };

        this._queryHeight(center, r, hits);
        this._querySweptMotion(center, r, state, hits);
        this._queryNeighborhood(center, r, hits);

        if (!hits.length) {
            this.lastDebug = { colliding: false };
            return null;
        }

        hits.sort((a, b) => b.penetration - a.penetration);
        const hit = hits[0];
        this.lastDebug = {
            colliding: true,
            source: hit.source,
            penetration: hit.penetration,
            normal: hit.normal,
        };
        return hit;
    }

    _queryHeight(center, radius, hits) {
        const surfaceY = this.world.sampleHeightAtLocal(center.x, center.z, this.heightProbeWidth);
        if (!Number.isFinite(surfaceY)) return;

        const penetration = surfaceY + radius + this.verticalSkin - center.y;
        if (penetration <= this.minPenetration) return;

        hits.push({
            normal: { x: 0, y: 1, z: 0 },
            penetration,
            source: 'height',
            pointCount: 1,
        });
    }

    _querySweptMotion(center, radius, state, hits) {
        const prev = state && state.previous;
        if (!prev) return;

        const motion = {
            x: center.x - prev.x,
            y: center.y - prev.y,
            z: center.z - prev.z,
        };
        const motionDistance = Math.hypot(motion.x, motion.y, motion.z);
        if (!Number.isFinite(motionDistance) || motionDistance < 0.03) return;

        const dir = normalize(motion, { x: 0, y: 0, z: -1 });
        const maxDistance = motionDistance + radius + this.sweptExtra;
        const hit = this.world.pickLocalRay(prev, dir, maxDistance);
        if (!this._validRayHit(prev, hit, maxDistance)) return;

        const penetration = motionDistance + radius + this.horizontalSkin - hit.distance;
        if (penetration <= this.minPenetration) return;

        const fallback = Math.abs(dir.y) < 0.7
            ? normalize({ x: -dir.x, y: 0, z: -dir.z }, { x: 0, y: 0, z: 1 })
            : normalize({ x: -dir.x, y: -dir.y, z: -dir.z }, { x: 0, y: 1, z: 0 });
        hits.push({
            normal: Math.abs(dir.y) < 0.7 ? fallback : hitNormalFromPoint(center.x, center.y, center.z, hit.position, fallback),
            penetration,
            source: 'swept',
            pointCount: 1,
        });
    }

    _queryNeighborhood(center, radius, hits) {
        const maxDistance = radius + this.rayExtra;
        const verticalOffsets = [0, radius * 0.45, -radius * 0.45];

        for (const dy of verticalOffsets) {
            const origin = { x: center.x, y: center.y + dy, z: center.z };
            for (const dir of this._rayDirs) {
                const hit = this.world.pickLocalRay(origin, dir, maxDistance);
                if (!this._validRayHit(origin, hit, maxDistance)) continue;

                const penetration = radius + this.horizontalSkin - hit.distance;
                if (penetration <= this.minPenetration) continue;

                const fallback = { x: -dir.x, y: 0, z: -dir.z };
                let normal = hitNormalFromPoint(center.x, center.y, center.z, hit.position, fallback);
                if (Math.abs(normal.y) > 0.65 && horizontalDistance(center, hit.position) > radius * 0.35) {
                    normal = normalize({ x: normal.x, y: 0, z: normal.z }, fallback);
                }

                hits.push({
                    normal,
                    penetration,
                    source: 'ray',
                    pointCount: 1,
                });
            }
        }
    }

    _validRayHit(origin, hit, maxDistance) {
        if (!hit || !hit.position || !Number.isFinite(hit.distance)) return false;
        if (hit.distance < this.minRayDistance || hit.distance > maxDistance) return false;
        const dx = hit.position.x - origin.x;
        const dy = hit.position.y - origin.y;
        const dz = hit.position.z - origin.z;
        return Number.isFinite(dx) && Number.isFinite(dy) && Number.isFinite(dz);
    }
}
