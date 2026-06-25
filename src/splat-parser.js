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
 * .splat Parser — antimatter15 format (32 bytes per gaussian, no header).
 * Layout per gaussian:
 *   float32×3  position (x, y, z)     offset 0   (12 bytes)
 *   float32×3  scale    (sx, sy, sz)  offset 12  (12 bytes)
 *   uint8×4    color    (r, g, b, a)  offset 24  (4 bytes, a = opacity 0-255)
 *   uint8×4    quaternion (compressed) offset 28  (4 bytes)
 *                                     Total: 32 bytes
 */

const SPLAT_RECORD_SIZE = 32;

/**
 * Parse .splat file for gaussian center positions (x, y, z).
 * Returns { positions: Float32Array, vertexCount, bounds }.
 */
export function parseSplatForPositions(arrayBuffer, options = {}) {
    const zUp = options.zUp !== undefined ? options.zUp : true;

    if (arrayBuffer.byteLength % SPLAT_RECORD_SIZE !== 0) {
        throw new Error(`Invalid .splat file: size ${arrayBuffer.byteLength} is not a multiple of ${SPLAT_RECORD_SIZE}`);
    }

    const vertexCount = arrayBuffer.byteLength / SPLAT_RECORD_SIZE;
    const dataView = new DataView(arrayBuffer);
    const positions = new Float32Array(vertexCount * 3);
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;

    for (let i = 0; i < vertexCount; i++) {
        const base = i * SPLAT_RECORD_SIZE;
        let rawX = dataView.getFloat32(base, true);
        let rawY = dataView.getFloat32(base + 4, true);
        let rawZ = dataView.getFloat32(base + 8, true);
        if (!isFinite(rawX)) rawX = 0;
        if (!isFinite(rawY)) rawY = 0;
        if (!isFinite(rawZ)) rawZ = 0;

        const off = i * 3;
        if (zUp) {
            // Z-up to Y-up: x' = x, y' = z, z' = -y
            positions[off]     = rawX;
            positions[off + 1] = rawZ;
            positions[off + 2] = -rawY;
        } else {
            // Y-up (COLMAP/OpenCV convention: Y-down) → flip Y
            positions[off]     = rawX;
            positions[off + 1] = -rawY;
            positions[off + 2] = -rawZ;
        }

        const x = positions[off], y = positions[off + 1], z = positions[off + 2];
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }

    return {
        positions,
        vertexCount,
        bounds: { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] }
    };
}

/**
 * Parse opacity values from a .splat file.
 * Color alpha is at offset 27 per record, already 0-255 linear.
 * Returns Float32Array of length vertexCount with values in [0,1].
 */
export function parseSplatOpacities(arrayBuffer) {
    if (arrayBuffer.byteLength % SPLAT_RECORD_SIZE !== 0) return null;

    const vertexCount = arrayBuffer.byteLength / SPLAT_RECORD_SIZE;
    const bytes = new Uint8Array(arrayBuffer);
    const opacities = new Float32Array(vertexCount);

    for (let i = 0; i < vertexCount; i++) {
        opacities[i] = bytes[i * SPLAT_RECORD_SIZE + 27] / 255.0;
    }

    return opacities;
}

/**
 * Compute raw centroid from .splat positions (untransformed, file-native coords).
 * Returns { x, y, z }.
 */
export function parseSplatRawCentroid(arrayBuffer) {
    if (arrayBuffer.byteLength % SPLAT_RECORD_SIZE !== 0) return { x: 0, y: 0, z: 0 };

    const vertexCount = arrayBuffer.byteLength / SPLAT_RECORD_SIZE;
    const dataView = new DataView(arrayBuffer);
    let cx = 0, cy = 0, cz = 0;
    let validCount = 0;

    for (let i = 0; i < vertexCount; i++) {
        const base = i * SPLAT_RECORD_SIZE;
        const rx = dataView.getFloat32(base, true);
        const ry = dataView.getFloat32(base + 4, true);
        const rz = dataView.getFloat32(base + 8, true);
        if (!isFinite(rx) || !isFinite(ry) || !isFinite(rz)) continue;
        cx += rx; cy += ry; cz += rz;
        validCount++;
    }

    if (validCount === 0) return { x: 0, y: 0, z: 0 };
    return { x: cx / validCount, y: cy / validCount, z: cz / validCount };
}

/**
 * Convert .splat ArrayBuffer to a minimal binary PLY ArrayBuffer
 * that PlayCanvas can load as a gsplat asset.
 *
 * PLY properties (matching 3DGS standard):
 *   float x, y, z, nx, ny, nz,
 *   float f_dc_0, f_dc_1, f_dc_2,
 *   float opacity,
 *   float scale_0, scale_1, scale_2,
 *   float rot_0, rot_1, rot_2, rot_3
 */
export function splatToPlyBuffer(arrayBuffer) {
    if (arrayBuffer.byteLength % SPLAT_RECORD_SIZE !== 0) {
        throw new Error('Invalid .splat file for conversion');
    }

    const vertexCount = arrayBuffer.byteLength / SPLAT_RECORD_SIZE;
    const srcView = new DataView(arrayBuffer);
    const srcBytes = new Uint8Array(arrayBuffer);

    // Build PLY header
    const header =
        'ply\n' +
        'format binary_little_endian 1.0\n' +
        `element vertex ${vertexCount}\n` +
        'property float x\n' +
        'property float y\n' +
        'property float z\n' +
        'property float nx\n' +
        'property float ny\n' +
        'property float nz\n' +
        'property float f_dc_0\n' +
        'property float f_dc_1\n' +
        'property float f_dc_2\n' +
        'property float opacity\n' +
        'property float scale_0\n' +
        'property float scale_1\n' +
        'property float scale_2\n' +
        'property float rot_0\n' +
        'property float rot_1\n' +
        'property float rot_2\n' +
        'property float rot_3\n' +
        'end_header\n';

    const headerBytes = new TextEncoder().encode(header);
    // 17 float32 properties = 68 bytes per vertex
    const vertexStride = 17 * 4;
    const totalSize = headerBytes.length + vertexCount * vertexStride;
    const outBuffer = new ArrayBuffer(totalSize);
    const outBytes = new Uint8Array(outBuffer);
    const outView = new DataView(outBuffer);

    // Copy header
    outBytes.set(headerBytes, 0);

    const SH_C0 = 0.28209479177387814; // 1 / (2 * sqrt(pi))

    for (let i = 0; i < vertexCount; i++) {
        const srcBase = i * SPLAT_RECORD_SIZE;
        const dstBase = headerBytes.length + i * vertexStride;

        // Position (x, y, z)
        outView.setFloat32(dstBase + 0, srcView.getFloat32(srcBase, true), true);
        outView.setFloat32(dstBase + 4, srcView.getFloat32(srcBase + 4, true), true);
        outView.setFloat32(dstBase + 8, srcView.getFloat32(srcBase + 8, true), true);

        // Normals (unused, set to 0)
        outView.setFloat32(dstBase + 12, 0, true);
        outView.setFloat32(dstBase + 16, 0, true);
        outView.setFloat32(dstBase + 20, 0, true);

        // Color: convert sRGB [0,255] → SH DC coefficients
        // f_dc = (color/255 - 0.5) / SH_C0
        const r = srcBytes[srcBase + 24];
        const g = srcBytes[srcBase + 25];
        const b = srcBytes[srcBase + 26];
        outView.setFloat32(dstBase + 24, (r / 255.0 - 0.5) / SH_C0, true);
        outView.setFloat32(dstBase + 28, (g / 255.0 - 0.5) / SH_C0, true);
        outView.setFloat32(dstBase + 32, (b / 255.0 - 0.5) / SH_C0, true);

        // Opacity: convert [0,255] → logit (inverse sigmoid)
        const a = srcBytes[srcBase + 27];
        const opNorm = Math.max(0.001, Math.min(0.999, a / 255.0));
        const logitOp = Math.log(opNorm / (1.0 - opNorm));
        outView.setFloat32(dstBase + 36, logitOp, true);

        // Scale (already log-space in .splat? No — .splat stores raw scale)
        // 3DGS PLY stores log(scale), so we take log
        const sx = srcView.getFloat32(srcBase + 12, true);
        const sy = srcView.getFloat32(srcBase + 16, true);
        const sz = srcView.getFloat32(srcBase + 20, true);
        outView.setFloat32(dstBase + 40, Math.log(Math.max(1e-7, sx)), true);
        outView.setFloat32(dstBase + 44, Math.log(Math.max(1e-7, sy)), true);
        outView.setFloat32(dstBase + 48, Math.log(Math.max(1e-7, sz)), true);

        // Quaternion: .splat stores as uint8 [0,255] → normalize to [-1,1]
        // Convention: (w, x, y, z) as rot_0..rot_3
        const q0 = (srcBytes[srcBase + 28] - 128) / 128.0;
        const q1 = (srcBytes[srcBase + 29] - 128) / 128.0;
        const q2 = (srcBytes[srcBase + 30] - 128) / 128.0;
        const q3 = (srcBytes[srcBase + 31] - 128) / 128.0;
        // Normalize quaternion
        const qLen = Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3) || 1;
        outView.setFloat32(dstBase + 52, q0 / qLen, true);
        outView.setFloat32(dstBase + 56, q1 / qLen, true);
        outView.setFloat32(dstBase + 60, q2 / qLen, true);
        outView.setFloat32(dstBase + 64, q3 / qLen, true);
    }

    return outBuffer;
}
