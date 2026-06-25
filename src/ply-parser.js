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
 * PLY Parser - extracts Gaussian center positions (x, y, z) from a .ply file
 * for use in the collision system. Supports both ASCII and binary (little-endian) PLY formats.
 * Applies Z-up to Y-up coordinate transform: x' = x, y' = z, z' = -y
 */

const _plyHeaderCache = new WeakMap();
const _textDecoder = new TextDecoder('utf-8');
const _textEncoder = new TextEncoder();

function findPlyHeaderEnd(bytes) {
    const limit = Math.min(bytes.length - 10, 65536);
    for (let i = 0; i <= limit; i++) {
        if (bytes[i] === 0x65 && bytes[i + 1] === 0x6e && bytes[i + 2] === 0x64 &&
            bytes[i + 3] === 0x5f && bytes[i + 4] === 0x68 && bytes[i + 5] === 0x65 &&
            bytes[i + 6] === 0x61 && bytes[i + 7] === 0x64 && bytes[i + 8] === 0x65 &&
            bytes[i + 9] === 0x72) {
            let j = i + 10;
            while (j < bytes.length && bytes[j] !== 0x0a) j++;
            return j + 1;
        }
    }
    return -1;
}

function parsePlyHeader(arrayBuffer) {
    const cached = _plyHeaderCache.get(arrayBuffer);
    if (cached) return cached;

    const bytes = new Uint8Array(arrayBuffer);
    const headerEnd = findPlyHeaderEnd(bytes);
    if (headerEnd === -1) {
        throw new Error('Invalid PLY file: could not find end_header');
    }

    const headerText = _textDecoder.decode(bytes.subarray(0, headerEnd));
    const headerLines = headerText.split('\n').map(l => l.trim());

    let vertexCount = 0;
    let format = 'ascii';
    const properties = [];
    let inVertexElement = false;
    let offset = 0;

    for (const line of headerLines) {
        if (line.startsWith('format')) {
            if (line.includes('binary_little_endian')) format = 'binary_little_endian';
            else if (line.includes('binary_big_endian')) format = 'binary_big_endian';
            else format = 'ascii';
        } else if (line.startsWith('element vertex')) {
            vertexCount = parseInt(line.split(/\s+/)[2], 10);
            inVertexElement = true;
        } else if (line.startsWith('element') && inVertexElement) {
            inVertexElement = false;
        } else if (line.startsWith('property') && inVertexElement) {
            const parts = line.split(/\s+/);
            // List properties are not part of standard 3DGS vertex payloads;
            // keep the old parser behavior by treating unknown/list lines as
            // one default-sized scalar instead of attempting variable strides.
            const type = parts[1] === 'list' ? parts[parts.length - 2] : parts[1];
            const name = parts[1] === 'list' ? parts[parts.length - 1] : parts[2];
            const size = getPropertySize(type);
            properties.push({ type, name, size, offset });
            offset += size;
        }
    }

    const meta = {
        bytes,
        headerEnd,
        headerText,
        headerLines,
        vertexCount,
        format,
        properties,
        vertexStride: offset,
        isLittle: format === 'binary_little_endian',
        xIdx: properties.findIndex(p => p.name === 'x'),
        yIdx: properties.findIndex(p => p.name === 'y'),
        zIdx: properties.findIndex(p => p.name === 'z'),
        opacityIdx: properties.findIndex(p => p.name === 'opacity'),
    };
    _plyHeaderCache.set(arrayBuffer, meta);
    return meta;
}

function _transformPosition(rawX, rawY, rawZ, zUp, positions, off) {
    if (!isFinite(rawX)) rawX = 0;
    if (!isFinite(rawY)) rawY = 0;
    if (!isFinite(rawZ)) rawZ = 0;
    if (zUp) {
        positions[off]     = rawX;
        positions[off + 1] = rawZ;
        positions[off + 2] = -rawY;
    } else {
        positions[off]     = rawX;
        positions[off + 1] = -rawY;
        positions[off + 2] = -rawZ;
    }
}

function _emptyBounds() {
    return {
        minX: Infinity, minY: Infinity, minZ: Infinity,
        maxX: -Infinity, maxY: -Infinity, maxZ: -Infinity
    };
}

function _includeBounds(bounds, x, y, z) {
    if (x < bounds.minX) bounds.minX = x; if (x > bounds.maxX) bounds.maxX = x;
    if (y < bounds.minY) bounds.minY = y; if (y > bounds.maxY) bounds.maxY = y;
    if (z < bounds.minZ) bounds.minZ = z; if (z > bounds.maxZ) bounds.maxZ = z;
}

function _boundsObject(bounds) {
    return { min: [bounds.minX, bounds.minY, bounds.minZ], max: [bounds.maxX, bounds.maxY, bounds.maxZ] };
}

export function parsePlyForPositions(arrayBuffer, options = {}) {
    const zUp = options.zUp !== undefined ? options.zUp : true;
    const meta = parsePlyHeader(arrayBuffer);
    const { vertexCount, format, properties, xIdx, yIdx, zIdx } = meta;

    if (vertexCount === 0) {
        throw new Error('PLY file has no vertices');
    }

    if (xIdx === -1 || yIdx === -1 || zIdx === -1) {
        throw new Error('PLY file missing x, y, or z properties');
    }

    const positions = new Float32Array(vertexCount * 3);
    const bounds = _emptyBounds();

    if (format === 'ascii') {
        const bodyText = _textDecoder.decode(meta.bytes.subarray(meta.headerEnd));
        const lines = bodyText.trim().split('\n');
        for (let i = 0; i < vertexCount && i < lines.length; i++) {
            const vals = lines[i].trim().split(/\s+/);
            const off = i * 3;
            _transformPosition(parseFloat(vals[xIdx]), parseFloat(vals[yIdx]), parseFloat(vals[zIdx]), zUp, positions, off);
            _includeBounds(bounds, positions[off], positions[off + 1], positions[off + 2]);
        }
    } else {
        const dataView = new DataView(arrayBuffer, meta.headerEnd);
        const isLittle = meta.isLittle;
        const xProp = properties[xIdx], yProp = properties[yIdx], zProp = properties[zIdx];

        for (let i = 0; i < vertexCount; i++) {
            const base = i * meta.vertexStride;
            const off = i * 3;
            _transformPosition(
                readFloat(dataView, base + xProp.offset, xProp.type, isLittle),
                readFloat(dataView, base + yProp.offset, yProp.type, isLittle),
                readFloat(dataView, base + zProp.offset, zProp.type, isLittle),
                zUp,
                positions,
                off
            );
            _includeBounds(bounds, positions[off], positions[off + 1], positions[off + 2]);
        }
    }

    return {
        positions,
        vertexCount,
        bounds: _boundsObject(bounds)
    };
}

/**
 * Parse all CPU-side scene data needed during initial PLY load in one pass.
 * This avoids separate full-vertex scans for opacity, raw centroid, position
 * extraction, and transformed centroid. Set includeOpacities=false when
 * reparsing only for a coordinate-system preview.
 */
export function parsePlySceneData(arrayBuffer, options = {}) {
    const zUp = options.zUp !== undefined ? options.zUp : true;
    const includeOpacities = options.includeOpacities !== false;
    const meta = parsePlyHeader(arrayBuffer);
    const { vertexCount, format, properties, xIdx, yIdx, zIdx, opacityIdx } = meta;

    if (vertexCount === 0) {
        throw new Error('PLY file has no vertices');
    }
    if (xIdx === -1 || yIdx === -1 || zIdx === -1) {
        throw new Error('PLY file missing x, y, or z properties');
    }

    const positions = new Float32Array(vertexCount * 3);
    const opacities = includeOpacities && opacityIdx !== -1 ? new Float32Array(vertexCount) : null;
    const bounds = _emptyBounds();
    const sigmoid = x => 1.0 / (1.0 + Math.exp(-x));

    let rawCx = 0, rawCy = 0, rawCz = 0, rawValidCount = 0;
    let cx = 0, cy = 0, cz = 0;

    const consumeVertex = (i, rawX, rawY, rawZ, rawOpacity) => {
        if (isFinite(rawX) && isFinite(rawY) && isFinite(rawZ)) {
            rawCx += rawX; rawCy += rawY; rawCz += rawZ;
            rawValidCount++;
        }

        const off = i * 3;
        _transformPosition(rawX, rawY, rawZ, zUp, positions, off);
        const x = positions[off], y = positions[off + 1], z = positions[off + 2];
        cx += x; cy += y; cz += z;
        _includeBounds(bounds, x, y, z);

        if (opacities) opacities[i] = sigmoid(rawOpacity);
    };

    if (format === 'ascii') {
        const bodyText = _textDecoder.decode(meta.bytes.subarray(meta.headerEnd));
        const lines = bodyText.trim().split('\n');
        for (let i = 0; i < vertexCount && i < lines.length; i++) {
            const vals = lines[i].trim().split(/\s+/);
            consumeVertex(
                i,
                parseFloat(vals[xIdx]),
                parseFloat(vals[yIdx]),
                parseFloat(vals[zIdx]),
                opacities && opacityIdx !== -1 ? parseFloat(vals[opacityIdx]) : 0
            );
        }
    } else {
        const dataView = new DataView(arrayBuffer, meta.headerEnd);
        const isLittle = meta.isLittle;
        const xProp = properties[xIdx], yProp = properties[yIdx], zProp = properties[zIdx];
        const opProp = opacities && opacityIdx !== -1 ? properties[opacityIdx] : null;

        for (let i = 0; i < vertexCount; i++) {
            const base = i * meta.vertexStride;
            consumeVertex(
                i,
                readFloat(dataView, base + xProp.offset, xProp.type, isLittle),
                readFloat(dataView, base + yProp.offset, yProp.type, isLittle),
                readFloat(dataView, base + zProp.offset, zProp.type, isLittle),
                opProp ? readFloat(dataView, base + opProp.offset, opProp.type, isLittle) : 0
            );
        }
    }

    if (vertexCount > 0) {
        cx /= vertexCount; cy /= vertexCount; cz /= vertexCount;
    }

    let maxDistSq = 0;
    for (let i = 0; i < vertexCount; i++) {
        const off = i * 3;
        const dx = positions[off] - cx;
        const dy = positions[off + 1] - cy;
        const dz = positions[off + 2] - cz;
        const distSq = dx * dx + dy * dy + dz * dz;
        if (distSq > maxDistSq) maxDistSq = distSq;
    }

    return {
        positions,
        vertexCount,
        bounds: _boundsObject(bounds),
        opacities,
        rawCentroid: rawValidCount > 0
            ? { x: rawCx / rawValidCount, y: rawCy / rawValidCount, z: rawCz / rawValidCount }
            : { x: 0, y: 0, z: 0 },
        analysis: {
            centroid: { x: cx, y: cy, z: cz },
            maxDistance: Math.sqrt(maxDistSq),
        },
    };
}

/**
 * Parse opacity values from a PLY binary.
 * 3DGS stores opacity as logit; we apply sigmoid to get [0,1].
 * Returns Float32Array of length vertexCount, or null if no opacity property.
 */
export function parsePlyOpacities(arrayBuffer) {
    let meta;
    try { meta = parsePlyHeader(arrayBuffer); }
    catch (_) { return null; }

    const { vertexCount, format, properties, opacityIdx } = meta;
    if (opacityIdx === -1 || vertexCount === 0) return null;

    const sigmoid = x => 1.0 / (1.0 + Math.exp(-x));
    const opacities = new Float32Array(vertexCount);

    if (format === 'ascii') {
        const bodyText = _textDecoder.decode(meta.bytes.subarray(meta.headerEnd));
        const lines = bodyText.trim().split('\n');
        for (let i = 0; i < vertexCount && i < lines.length; i++) {
            opacities[i] = sigmoid(parseFloat(lines[i].trim().split(/\s+/)[opacityIdx]));
        }
    } else {
        const dataView = new DataView(arrayBuffer, meta.headerEnd);
        const isLittle = meta.isLittle;
        const opProp = properties[opacityIdx];
        for (let i = 0; i < vertexCount; i++) {
            const raw = readFloat(dataView, i * meta.vertexStride + opProp.offset, opProp.type, isLittle);
            opacities[i] = sigmoid(raw);
        }
    }
    return opacities;
}

/**
 * Compute centroid from raw (untransformed) PLY positions.
 * Returns { x, y, z } in the PLY file's native coordinate space.
 */
export function parsePlyRawCentroid(arrayBuffer) {
    let meta;
    try { meta = parsePlyHeader(arrayBuffer); }
    catch (_) { return { x: 0, y: 0, z: 0 }; }

    const { vertexCount, format, properties, xIdx, yIdx, zIdx } = meta;
    if (xIdx === -1 || yIdx === -1 || zIdx === -1 || vertexCount === 0) return { x: 0, y: 0, z: 0 };

    let cx = 0, cy = 0, cz = 0;
    let validCount = 0;

    if (format === 'ascii') {
        const bodyText = _textDecoder.decode(meta.bytes.subarray(meta.headerEnd));
        const lines = bodyText.trim().split('\n');
        for (let i = 0; i < vertexCount && i < lines.length; i++) {
            const vals = lines[i].trim().split(/\s+/);
            const rx = parseFloat(vals[xIdx]), ry = parseFloat(vals[yIdx]), rz = parseFloat(vals[zIdx]);
            if (!isFinite(rx) || !isFinite(ry) || !isFinite(rz)) continue;
            cx += rx; cy += ry; cz += rz;
            validCount++;
        }
    } else {
        const dataView = new DataView(arrayBuffer, meta.headerEnd);
        const isLittle = meta.isLittle;
        const xProp = properties[xIdx], yProp = properties[yIdx], zProp = properties[zIdx];
        for (let i = 0; i < vertexCount; i++) {
            const base = i * meta.vertexStride;
            const rx = readFloat(dataView, base + xProp.offset, xProp.type, isLittle);
            const ry = readFloat(dataView, base + yProp.offset, yProp.type, isLittle);
            const rz = readFloat(dataView, base + zProp.offset, zProp.type, isLittle);
            if (!isFinite(rx) || !isFinite(ry) || !isFinite(rz)) continue;
            cx += rx; cy += ry; cz += rz;
            validCount++;
        }
    }

    if (validCount === 0) return { x: 0, y: 0, z: 0 };
    return { x: cx / validCount, y: cy / validCount, z: cz / validCount };
}

/**
 * Count points passing both distance AND opacity filters.
 * Uses transformed positions (same as collision system) and parsed opacities.
 */
export function countFilteredPoints(positions, opacities, vertexCount, cx, cy, cz, maxDist, minOpacity) {
    const maxDistSq = maxDist * maxDist;
    let count = 0;
    const checkOpacity = !!opacities && minOpacity > 0;
    for (let i = 0, off = 0; i < vertexCount; i++, off += 3) {
        if (checkOpacity && opacities[i] < minOpacity) continue;
        const dx = positions[off] - cx;
        const dy = positions[off + 1] - cy;
        const dz = positions[off + 2] - cz;
        if (dx * dx + dy * dy + dz * dz <= maxDistSq) count++;
    }
    return count;
}

/**
 * Filter PLY binary by both distance AND opacity.
 * Returns { filteredBuffer, keptCount }.
 */
export function filterPlyByCriteria(arrayBuffer, positions, opacities, vertexCount, cx, cy, cz, maxDist, minOpacity) {
    const maxDistSq = maxDist * maxDist;
    const keepIndices = [];
    for (let i = 0; i < vertexCount; i++) {
        if (opacities && opacities[i] < minOpacity) continue;
        const dx = positions[i * 3] - cx;
        const dy = positions[i * 3 + 1] - cy;
        const dz = positions[i * 3 + 2] - cz;
        if (dx * dx + dy * dy + dz * dz <= maxDistSq) {
            keepIndices.push(i);
        }
    }

    if (keepIndices.length === vertexCount) {
        return { filteredBuffer: arrayBuffer, keptCount: vertexCount };
    }

    const bytes = new Uint8Array(arrayBuffer);
    const decoder = new TextDecoder('utf-8');

    let headerEnd = -1;
    for (let i = 0; i < Math.min(bytes.length, 65536); i++) {
        if (bytes[i] === 0x65 && bytes[i + 1] === 0x6e && bytes[i + 2] === 0x64 &&
            bytes[i + 3] === 0x5f && bytes[i + 4] === 0x68 && bytes[i + 5] === 0x65 &&
            bytes[i + 6] === 0x61 && bytes[i + 7] === 0x64 && bytes[i + 8] === 0x65 &&
            bytes[i + 9] === 0x72) {
            let j = i + 10;
            while (j < bytes.length && bytes[j] !== 0x0a) j++;
            headerEnd = j + 1;
            break;
        }
    }

    const headerText = decoder.decode(bytes.slice(0, headerEnd));
    const newHeader = headerText.replace(/element vertex \d+/, `element vertex ${keepIndices.length}`);

    const headerLines = headerText.split('\n').map(l => l.trim());
    const properties = [];
    let inV = false;
    for (const line of headerLines) {
        if (line.startsWith('element vertex')) inV = true;
        else if (line.startsWith('element') && inV) inV = false;
        else if (line.startsWith('property') && inV) {
            properties.push({ type: line.split(/\s+/)[1] });
        }
    }
    const vertexStride = properties.map(p => getPropertySize(p.type)).reduce((a, b) => a + b, 0);

    const headerBytes = _textEncoder.encode(newHeader);
    const dataAfterVertices = bytes.slice(headerEnd + vertexCount * vertexStride);
    const newSize = headerBytes.length + keepIndices.length * vertexStride + dataAfterVertices.length;
    const newBuffer = new ArrayBuffer(newSize);
    const newBytes = new Uint8Array(newBuffer);

    newBytes.set(headerBytes, 0);
    let offset = headerBytes.length;
    for (const idx of keepIndices) {
        const srcStart = headerEnd + idx * vertexStride;
        newBytes.set(new Uint8Array(arrayBuffer, srcStart, vertexStride), offset);
        offset += vertexStride;
    }
    if (dataAfterVertices.length > 0) {
        newBytes.set(dataAfterVertices, offset);
    }

    return { filteredBuffer: newBuffer, keptCount: keepIndices.length };
}

/**
 * Analyze point cloud distances from centroid.
 * Returns { centroid: {x,y,z}, maxDistance: number }
 */
export function analyzePlyDistances(positions, vertexCount) {
    let cx = 0, cy = 0, cz = 0;
    let validCount = 0;
    for (let i = 0, off = 0; i < vertexCount; i++, off += 3) {
        const x = positions[off], y = positions[off + 1], z = positions[off + 2];
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        cx += x; cy += y; cz += z;
        validCount++;
    }
    if (validCount === 0) return { centroid: { x: 0, y: 0, z: 0 }, maxDistance: 1 };
    cx /= validCount; cy /= validCount; cz /= validCount;

    let maxDistSq = 0;
    for (let i = 0, off = 0; i < vertexCount; i++, off += 3) {
        const x = positions[off], y = positions[off + 1], z = positions[off + 2];
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        const dx = x - cx, dy = y - cy, dz = z - cz;
        const distSq = dx * dx + dy * dy + dz * dz;
        if (distSq > maxDistSq) maxDistSq = distSq;
    }

    return { centroid: { x: cx, y: cy, z: cz }, maxDistance: Math.sqrt(maxDistSq) };
}

/**
 * Count how many points are within maxDistance from center.
 */
export function countPointsWithinDistance(positions, vertexCount, cx, cy, cz, maxDistance) {
    const maxDistSq = maxDistance * maxDistance;
    let count = 0;
    for (let i = 0, off = 0; i < vertexCount; i++, off += 3) {
        const dx = positions[off] - cx;
        const dy = positions[off + 1] - cy;
        const dz = positions[off + 2] - cz;
        if (dx * dx + dy * dy + dz * dz <= maxDistSq) count++;
    }
    return count;
}

/**
 * Filter a PLY binary ArrayBuffer, keeping only vertices within maxDistance
 * from the given center (computed on the transformed positions).
 * Returns { filteredBuffer: ArrayBuffer, keptCount: number }
 */
export function filterPlyByDistance(arrayBuffer, positions, vertexCount, cx, cy, cz, maxDistance) {
    const maxDistSq = maxDistance * maxDistance;
    const keepIndices = [];
    for (let i = 0; i < vertexCount; i++) {
        const dx = positions[i * 3] - cx;
        const dy = positions[i * 3 + 1] - cy;
        const dz = positions[i * 3 + 2] - cz;
        if (dx * dx + dy * dy + dz * dz <= maxDistSq) {
            keepIndices.push(i);
        }
    }

    if (keepIndices.length === vertexCount) {
        return { filteredBuffer: arrayBuffer, keptCount: vertexCount };
    }

    const bytes = new Uint8Array(arrayBuffer);
    const decoder = new TextDecoder('utf-8');

    // Find header end
    let headerEnd = -1;
    for (let i = 0; i < Math.min(bytes.length, 65536); i++) {
        if (bytes[i] === 0x65 && bytes[i + 1] === 0x6e && bytes[i + 2] === 0x64 &&
            bytes[i + 3] === 0x5f && bytes[i + 4] === 0x68 && bytes[i + 5] === 0x65 &&
            bytes[i + 6] === 0x61 && bytes[i + 7] === 0x64 && bytes[i + 8] === 0x65 &&
            bytes[i + 9] === 0x72) {
            let j = i + 10;
            while (j < bytes.length && bytes[j] !== 0x0a) j++;
            headerEnd = j + 1;
            break;
        }
    }

    const headerText = decoder.decode(bytes.slice(0, headerEnd));

    // Replace vertex count in header
    const newHeader = headerText.replace(
        /element vertex \d+/,
        `element vertex ${keepIndices.length}`
    );

    // Compute vertex stride from properties
    const headerLines = headerText.split('\n').map(l => l.trim());
    const properties = [];
    let inVertex = false;
    for (const line of headerLines) {
        if (line.startsWith('element vertex')) inVertex = true;
        else if (line.startsWith('element') && inVertex) inVertex = false;
        else if (line.startsWith('property') && inVertex) {
            properties.push({ type: line.split(/\s+/)[1] });
        }
    }
    const vertexStride = properties.map(p => getPropertySize(p.type)).reduce((a, b) => a + b, 0);

    // Build new buffer
    const headerBytes = _textEncoder.encode(newHeader);
    const dataAfterVertices = bytes.slice(headerEnd + vertexCount * vertexStride);
    const newSize = headerBytes.length + keepIndices.length * vertexStride + dataAfterVertices.length;
    const newBuffer = new ArrayBuffer(newSize);
    const newBytes = new Uint8Array(newBuffer);

    // Copy header
    newBytes.set(headerBytes, 0);

    // Copy kept vertex rows
    let offset = headerBytes.length;
    for (const idx of keepIndices) {
        const srcStart = headerEnd + idx * vertexStride;
        newBytes.set(new Uint8Array(arrayBuffer, srcStart, vertexStride), offset);
        offset += vertexStride;
    }

    // Copy any data after vertices (other elements)
    if (dataAfterVertices.length > 0) {
        newBytes.set(dataAfterVertices, offset);
    }

    return { filteredBuffer: newBuffer, keptCount: keepIndices.length };
}

/**
 * Sanitize a binary PLY ArrayBuffer in-place: replace every NaN / ±Inf in
 * float32 and float64 vertex properties with 0.  Returns the same buffer
 * (mutated) so it can be used as a drop-in replacement before passing to
 * the renderer.  ASCII PLY files are returned unchanged.
 *
 * This prevents corrupt SH / scale / rotation values from producing
 * rainbow-coloured flickering artifacts during camera rotation.
 */
export function sanitizePlyBuffer(arrayBuffer) {
    let meta;
    try { meta = parsePlyHeader(arrayBuffer); }
    catch (_) { return arrayBuffer; }

    if (meta.format === 'ascii') return arrayBuffer;
    if (meta.vertexCount === 0 || meta.properties.length === 0) return arrayBuffer;

    let fixed = 0;
    const allFloat32 = meta.properties.every(p => (p.type === 'float' || p.type === 'float32') && p.size === 4);

    if (meta.isLittle && allFloat32 && (meta.headerEnd % 4) === 0) {
        const floatCount = (meta.vertexStride / 4) * meta.vertexCount;
        const values = new Float32Array(arrayBuffer, meta.headerEnd, floatCount);
        for (let i = 0; i < values.length; i++) {
            if (!Number.isFinite(values[i])) {
                values[i] = 0;
                fixed++;
            }
        }
    } else {
        const dv = new DataView(arrayBuffer, meta.headerEnd);
        for (let i = 0; i < meta.vertexCount; i++) {
            let off = i * meta.vertexStride;
            for (const prop of meta.properties) {
                if (prop.type === 'float' || prop.type === 'float32') {
                    const v = dv.getFloat32(off, meta.isLittle);
                    if (!isFinite(v)) { dv.setFloat32(off, 0, meta.isLittle); fixed++; }
                } else if (prop.type === 'double' || prop.type === 'float64') {
                    const v = dv.getFloat64(off, meta.isLittle);
                    if (!isFinite(v)) { dv.setFloat64(off, 0, meta.isLittle); fixed++; }
                }
                off += prop.size;
            }
        }
    }

    if (fixed > 0) {
        console.warn(`sanitizePlyBuffer: replaced ${fixed} NaN/Inf values in ${meta.vertexCount} vertices`);
    }
    return arrayBuffer;
}

function getPropertySize(type) {
    switch (type) {
        case 'char': case 'int8': case 'uchar': case 'uint8': return 1;
        case 'short': case 'int16': case 'ushort': case 'uint16': return 2;
        case 'int': case 'int32': case 'uint': case 'uint32': case 'float': case 'float32': return 4;
        case 'double': case 'float64': return 8;
        default: return 4;
    }
}

function readFloat(dataView, offset, type, isLittle) {
    switch (type) {
        case 'float': case 'float32': return dataView.getFloat32(offset, isLittle);
        case 'double': case 'float64': return dataView.getFloat64(offset, isLittle);
        case 'int': case 'int32': return dataView.getInt32(offset, isLittle);
        case 'uint': case 'uint32': return dataView.getUint32(offset, isLittle);
        case 'short': case 'int16': return dataView.getInt16(offset, isLittle);
        case 'ushort': case 'uint16': return dataView.getUint16(offset, isLittle);
        case 'char': case 'int8': return dataView.getInt8(offset);
        case 'uchar': case 'uint8': return dataView.getUint8(offset);
        default: return dataView.getFloat32(offset, isLittle);
    }
}
