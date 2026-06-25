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
 * Octree-based collision detection for point cloud data.
 * Builds a spatial index from Gaussian center positions and provides
 * sphere-vs-pointcloud queries for drone collision.
 */

const MAX_POINTS_PER_NODE = 1024;
const MAX_DEPTH = 32;
const MIN_SPLIT_EXTENT = 1e-6;

class OctreeNode {
    constructor(minX, minY, minZ, maxX, maxY, maxZ, depth) {
        this.minX = minX; this.minY = minY; this.minZ = minZ;
        this.maxX = maxX; this.maxY = maxY; this.maxZ = maxZ;
        this.depth = depth;
        this.left = null;
        this.right = null;
        this.first = 0;
        this.count = 0;
    }

    get isLeaf() { return this.left === null && this.right === null; }

    intersectsSphere(cx, cy, cz, r) {
        // Closest point on AABB to sphere center
        const dx = Math.max(this.minX - cx, 0, cx - this.maxX);
        const dy = Math.max(this.minY - cy, 0, cy - this.maxY);
        const dz = Math.max(this.minZ - cz, 0, cz - this.maxZ);
        return (dx * dx + dy * dy + dz * dz) <= (r * r);
    }
}

export class Octree {
    constructor() {
        this.root = null;
        this.positions = null;
        this.indices = null;
        this.pointCount = 0;
        this._leafBoundsCache = null;
    }

    /**
     * Build the octree from a Float32Array of positions [x0,y0,z0, x1,y1,z1, ...]
     */
    build(positions, bounds = null) {
        this.positions = positions;
        this.pointCount = positions.length / 3;
        this._leafBoundsCache = null;

        this.indices = new Uint32Array(this.pointCount);
        for (let i = 0; i < this.pointCount; i++) {
            this.indices[i] = i;
        }

        if (this.pointCount === 0) {
            const b = bounds || { min: [0, 0, 0], max: [0, 0, 0] };
            this.root = new OctreeNode(b.min[0], b.min[1], b.min[2], b.max[0], b.max[1], b.max[2], 0);
            return this;
        }

        this.root = this._buildRange(0, this.pointCount, 0);

        return this;
    }

    _buildRange(first, count, depth) {
        const bounds = this._computeBounds(first, first + count);
        const node = new OctreeNode(bounds.minX, bounds.minY, bounds.minZ, bounds.maxX, bounds.maxY, bounds.maxZ, depth);

        const spanX = bounds.maxX - bounds.minX;
        const spanY = bounds.maxY - bounds.minY;
        const spanZ = bounds.maxZ - bounds.minZ;
        const maxSpan = Math.max(spanX, spanY, spanZ);

        if (count <= MAX_POINTS_PER_NODE || depth >= MAX_DEPTH || maxSpan <= MIN_SPLIT_EXTENT) {
            node.first = first;
            node.count = count;
            return node;
        }

        const axis = spanX >= spanY && spanX >= spanZ ? 0 : (spanY >= spanZ ? 1 : 2);
        const end = first + count;
        const splitValue = axis === 0
            ? (bounds.minX + bounds.maxX) * 0.5
            : axis === 1
                ? (bounds.minY + bounds.maxY) * 0.5
                : (bounds.minZ + bounds.maxZ) * 0.5;

        let mid = this._partitionByValue(first, end, axis, splitValue);
        const leftCount = mid - first;
        const rightCount = end - mid;

        // Large outliers make midpoint splits collapse into one huge leaf.
        // Fall back to a median split whenever the midpoint split is empty
        // or severely imbalanced; exact per-node bounds still make queries
        // prune aggressively.
        if (leftCount === 0 || rightCount === 0 || leftCount < count * 0.1 || rightCount < count * 0.1) {
            mid = first + (count >> 1);
            this._selectKth(first, end - 1, mid, axis);
        }

        node.left = this._buildRange(first, mid - first, depth + 1);
        node.right = this._buildRange(mid, end - mid, depth + 1);
        return node;
    }

    _computeBounds(first, end) {
        let minX = Infinity, minY = Infinity, minZ = Infinity;
        let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
        const positions = this.positions;
        const indices = this.indices;
        for (let i = first; i < end; i++) {
            const off = indices[i] * 3;
            const x = positions[off];
            const y = positions[off + 1];
            const z = positions[off + 2];
            if (x < minX) minX = x; if (x > maxX) maxX = x;
            if (y < minY) minY = y; if (y > maxY) maxY = y;
            if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
        }
        return { minX, minY, minZ, maxX, maxY, maxZ };
    }

    _valueAtIndexSlot(slot, axis) {
        return this.positions[this.indices[slot] * 3 + axis];
    }

    _swapIndexSlots(a, b) {
        const tmp = this.indices[a];
        this.indices[a] = this.indices[b];
        this.indices[b] = tmp;
    }

    _partitionByValue(first, end, axis, splitValue) {
        let left = first;
        let right = end - 1;
        while (left <= right) {
            while (left <= right && this._valueAtIndexSlot(left, axis) < splitValue) left++;
            while (left <= right && this._valueAtIndexSlot(right, axis) >= splitValue) right--;
            if (left < right) {
                this._swapIndexSlots(left, right);
                left++;
                right--;
            }
        }
        return left;
    }

    _partitionAroundPivot(left, right, pivotIndex, axis) {
        const pivotValue = this._valueAtIndexSlot(pivotIndex, axis);
        this._swapIndexSlots(pivotIndex, right);
        let store = left;
        for (let i = left; i < right; i++) {
            if (this._valueAtIndexSlot(i, axis) < pivotValue) {
                this._swapIndexSlots(store, i);
                store++;
            }
        }
        this._swapIndexSlots(right, store);
        return store;
    }

    _selectKth(left, right, kth, axis) {
        while (left < right) {
            const pivotIndex = (left + right) >> 1;
            const pivotNew = this._partitionAroundPivot(left, right, pivotIndex, axis);
            if (kth === pivotNew) return;
            if (kth < pivotNew) right = pivotNew - 1;
            else left = pivotNew + 1;
        }
    }

    /**
     * Collect AABBs of all non-empty leaf nodes.
     * Returns a Float32Array: [minX,minY,minZ, maxX,maxY,maxZ, ...] (6 floats per leaf).
     * Result is cached after first call.
     */
    getLeafBounds() {
        if (this._leafBoundsCache) return this._leafBoundsCache;
        const list = [];
        if (this.root) this._collectLeaves(this.root, list);
        this._leafBoundsCache = new Float32Array(list);
        return this._leafBoundsCache;
    }

    _collectLeaves(node, list) {
        if (!node) return;
        if (node.isLeaf) {
            if (node.count > 0) {
                list.push(node.minX, node.minY, node.minZ, node.maxX, node.maxY, node.maxZ);
            }
        } else {
            this._collectLeaves(node.left, list);
            this._collectLeaves(node.right, list);
        }
    }

    _querySphereNode(node, cx, cy, cz, r, rSq, results) {
        if (!node || !node.intersectsSphere(cx, cy, cz, r)) return;

        if (node.isLeaf) {
            const positions = this.positions;
            const indices = this.indices;
            const end = node.first + node.count;
            for (let p = node.first; p < end; p++) {
                const idx = indices[p];
                const off = idx * 3;
                const px = positions[off];
                const py = positions[off + 1];
                const pz = positions[off + 2];
                const dx = px - cx, dy = py - cy, dz = pz - cz;
                const distSq = dx * dx + dy * dy + dz * dz;
                if (distSq <= rSq) {
                    results.push({ index: idx, x: px, y: py, z: pz, distSq });
                }
            }
        } else {
            this._querySphereNode(node.left, cx, cy, cz, r, rSq, results);
            this._querySphereNode(node.right, cx, cy, cz, r, rSq, results);
        }
    }

    /**
     * Query all points within a sphere. Returns array of { index, x, y, z, distSq }.
     */
    querySphere(cx, cy, cz, radius) {
        const results = [];
        const rSq = radius * radius;
        if (this.root) {
            this._querySphereNode(this.root, cx, cy, cz, radius, rSq, results);
        }
        return results;
    }

    querySphereCount(cx, cy, cz, radius) {
        const rSq = radius * radius;
        return this.root ? this._querySphereCountNode(this.root, cx, cy, cz, radius, rSq) : 0;
    }

    _querySphereCountNode(node, cx, cy, cz, r, rSq) {
        if (!node || !node.intersectsSphere(cx, cy, cz, r)) return 0;
        if (!node.isLeaf) {
            return this._querySphereCountNode(node.left, cx, cy, cz, r, rSq) +
                   this._querySphereCountNode(node.right, cx, cy, cz, r, rSq);
        }

        let count = 0;
        const positions = this.positions;
        const indices = this.indices;
        const end = node.first + node.count;
        for (let p = node.first; p < end; p++) {
            const off = indices[p] * 3;
            const dx = positions[off] - cx;
            const dy = positions[off + 1] - cy;
            const dz = positions[off + 2] - cz;
            if (dx * dx + dy * dy + dz * dz <= rSq) count++;
        }
        return count;
    }

    queryCollisionResponse(cx, cy, cz, radius) {
        if (!this.root) return null;
        const state = { nx: 0, ny: 0, nz: 0, minDist: Infinity, count: 0 };
        this._queryCollisionNode(this.root, cx, cy, cz, radius, radius * radius, state);
        if (state.count === 0) return null;

        const len = Math.sqrt(state.nx * state.nx + state.ny * state.ny + state.nz * state.nz);
        if (len < 0.0001) {
            return { normal: { x: 0, y: 1, z: 0 }, penetration: radius - state.minDist, pointCount: state.count };
        }

        const invLen = 1 / len;
        return {
            normal: { x: state.nx * invLen, y: state.ny * invLen, z: state.nz * invLen },
            penetration: Math.max(0, radius - state.minDist),
            pointCount: state.count,
        };
    }

    _queryCollisionNode(node, cx, cy, cz, r, rSq, state) {
        if (!node || !node.intersectsSphere(cx, cy, cz, r)) return;
        if (!node.isLeaf) {
            this._queryCollisionNode(node.left, cx, cy, cz, r, rSq, state);
            this._queryCollisionNode(node.right, cx, cy, cz, r, rSq, state);
            return;
        }

        const positions = this.positions;
        const indices = this.indices;
        const end = node.first + node.count;
        for (let p = node.first; p < end; p++) {
            const off = indices[p] * 3;
            const dx = cx - positions[off];
            const dy = cy - positions[off + 1];
            const dz = cz - positions[off + 2];
            const distSq = dx * dx + dy * dy + dz * dz;
            if (distSq > rSq) continue;

            const dist = Math.sqrt(distSq);
            state.count++;
            if (dist < state.minDist) state.minDist = dist;
            if (dist < 0.0001) continue;

            const w = 1.0 / (dist + 0.001);
            state.nx += dx * w;
            state.ny += dy * w;
            state.nz += dz * w;
        }
    }
}

/**
 * Collision detection result processor.
 * Given a sphere query result, computes collision normal and penetration.
 */
export function computeCollisionResponse(dronePos, radius, queryResults) {
    if (queryResults.length === 0) {
        return null;
    }

    // Compute average direction from nearby points to drone (surface normal estimate)
    let nx = 0, ny = 0, nz = 0;
    let minDist = Infinity;

    for (const pt of queryResults) {
        const dx = dronePos.x - pt.x;
        const dy = dronePos.y - pt.y;
        const dz = dronePos.z - pt.z;
        const dist = Math.sqrt(pt.distSq);
        if (dist < 0.0001) continue;

        // Weight by inverse distance (closer points contribute more to normal)
        const w = 1.0 / (dist + 0.001);
        nx += dx * w;
        ny += dy * w;
        nz += dz * w;

        if (dist < minDist) minDist = dist;
    }

    // Normalize
    const len = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 0.0001) {
        // Degenerate case: push straight up
        return { normal: { x: 0, y: 1, z: 0 }, penetration: radius - minDist, pointCount: queryResults.length };
    }

    nx /= len; ny /= len; nz /= len;
    const penetration = radius - minDist;

    return {
        normal: { x: nx, y: ny, z: nz },
        penetration: Math.max(0, penetration),
        pointCount: queryResults.length
    };
}
