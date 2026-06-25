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
 * SOG Parser — PlayCanvas Spatially Ordered Gaussians format.
 * A .sog file is a ZIP archive containing meta.json + WebP images.
 * Positions are encoded across means_l.webp and means_u.webp with
 * log-space quantization described in meta.json.
 *
 * Requires JSZip to be loaded globally (window.JSZip).
 */

/**
 * Decode a WebP image blob into RGBA pixel data.
 * Returns { width, height, data: Uint8ClampedArray }.
 */
async function decodeWebpImage(blob) {
    const bitmap = await createImageBitmap(blob);
    const canvas = new OffscreenCanvas(bitmap.width, bitmap.height);
    const ctx = canvas.getContext('2d');
    ctx.drawImage(bitmap, 0, 0);
    const imageData = ctx.getImageData(0, 0, bitmap.width, bitmap.height);
    bitmap.close();
    return { width: imageData.width, height: imageData.height, data: imageData.data };
}

/**
 * Symmetric log inverse: unlog(n) = sign(n) * (exp(|n|) - 1)
 */
function unlog(n) {
    return Math.sign(n) * (Math.exp(Math.abs(n)) - 1);
}

/**
 * Linear interpolation.
 */
function lerp(a, b, t) {
    return a + (b - a) * t;
}

/**
 * Parse a .sog file for gaussian center positions.
 * Returns { positions: Float32Array, vertexCount, bounds }.
 */
export async function parseSogForPositions(arrayBuffer, options = {}) {
    if (typeof JSZip === 'undefined') {
        throw new Error('JSZip is required to parse .sog files');
    }

    const zip = await JSZip.loadAsync(arrayBuffer);

    // Parse meta.json
    const metaFile = zip.file('meta.json');
    if (!metaFile) throw new Error('SOG archive missing meta.json');
    const metaText = await metaFile.async('string');
    const meta = JSON.parse(metaText);

    const vertexCount = meta.count;
    if (!vertexCount || vertexCount <= 0) {
        throw new Error('SOG meta.json has invalid count');
    }

    const mins = meta.means.mins;
    const maxs = meta.means.maxs;
    const meansFiles = meta.means.files; // ["means_l.webp", "means_u.webp"]

    // Load means_l and means_u images
    const meansLFile = zip.file(meansFiles[0]);
    const meansUFile = zip.file(meansFiles[1]);
    if (!meansLFile || !meansUFile) {
        throw new Error('SOG archive missing means images');
    }

    const [meansLBlob, meansUBlob] = await Promise.all([
        meansLFile.async('blob'),
        meansUFile.async('blob'),
    ]);

    const [meansL, meansU] = await Promise.all([
        decodeWebpImage(meansLBlob),
        decodeWebpImage(meansUBlob),
    ]);

    // Reconstruct positions
    const zUp = options.zUp || false;
    const positions = new Float32Array(vertexCount * 3);
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;

    for (let i = 0; i < vertexCount; i++) {
        const pixelOffset = i * 4; // RGBA per pixel

        // 16-bit quantized values per axis
        const qx = (meansU.data[pixelOffset + 0] << 8) | meansL.data[pixelOffset + 0];
        const qy = (meansU.data[pixelOffset + 1] << 8) | meansL.data[pixelOffset + 1];
        const qz = (meansU.data[pixelOffset + 2] << 8) | meansL.data[pixelOffset + 2];

        // Dequantize to log-domain
        const nx = lerp(mins[0], maxs[0], qx / 65535);
        const ny = lerp(mins[1], maxs[1], qy / 65535);
        const nz = lerp(mins[2], maxs[2], qz / 65535);

        // Undo symmetric log transform
        let rawX = unlog(nx);
        let rawY = unlog(ny);
        let rawZ = unlog(nz);
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
            // Y-up — pass through directly
            positions[off]     = rawX;
            positions[off + 1] = rawY;
            positions[off + 2] = rawZ;
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
 * Parse opacity values from a .sog file.
 * Opacity is stored in sh0.webp alpha channel, already sigmoid-mapped [0,255].
 * Returns Float32Array of length vertexCount with values in [0,1], or null.
 */
export async function parseSogOpacities(arrayBuffer) {
    if (typeof JSZip === 'undefined') return null;

    const zip = await JSZip.loadAsync(arrayBuffer);

    const metaFile = zip.file('meta.json');
    if (!metaFile) return null;
    const meta = JSON.parse(await metaFile.async('string'));

    const vertexCount = meta.count;
    if (!vertexCount) return null;

    const sh0Files = meta.sh0 && meta.sh0.files;
    if (!sh0Files || sh0Files.length === 0) return null;

    const sh0File = zip.file(sh0Files[0]);
    if (!sh0File) return null;

    const sh0Blob = await sh0File.async('blob');
    const sh0 = await decodeWebpImage(sh0Blob);

    const opacities = new Float32Array(vertexCount);
    for (let i = 0; i < vertexCount; i++) {
        opacities[i] = sh0.data[i * 4 + 3] / 255.0;
    }

    return opacities;
}

/**
 * Compute raw centroid from .sog positions (in SOG's native Y-up frame).
 * Returns { x, y, z }.
 */
export async function parseSogRawCentroid(arrayBuffer) {
    const { positions, vertexCount } = await parseSogForPositions(arrayBuffer);
    let cx = 0, cy = 0, cz = 0;
    let validCount = 0;
    for (let i = 0; i < vertexCount; i++) {
        const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        cx += x; cy += y; cz += z;
        validCount++;
    }
    if (validCount === 0) return { x: 0, y: 0, z: 0 };
    return { x: cx / validCount, y: cy / validCount, z: cz / validCount };
}
