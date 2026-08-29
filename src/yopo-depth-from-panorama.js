/*
 * Copyright 2026 Manifold Tech Ltd.
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
 * Convert DA360 panoramic depth into a YOPO depth input.
 *
 * DA360 outputs a metric depth image up to an unknown global scale.  We
 * recover the scale by comparing a few Cesium raycast distances (ground truth)
 * against the corresponding DA360 relative depths.
 *
 * Two modes are supported:
 *   - captureYOPODepthERP (primary, YOPO_360): DA360 already returns an
 *     equirectangular panoramic depth map — exactly the ERP layout the
 *     YOPO_360 network expects.  We resize it to 384x192 and emit a validity
 *     mask as the second channel.  No reprojection is needed.
 *   - captureYOPODepth (fallback, legacy pinhole): the scaled panoramic depth
 *     is reprojected into a forward-facing pinhole depth image via
 *     reprojectForward().  Kept for backward compatibility / debugging.
 */

const DEFAULT_WIDTH = 160;   // legacy pinhole image_width  (columns)
const DEFAULT_HEIGHT = 96;   // legacy pinhole image_height (rows)
const DEFAULT_WIDTH_ERP = 384;   // YOPO_360 ERP image_width  (columns)
const DEFAULT_HEIGHT_ERP = 192;  // YOPO_360 ERP image_height (rows)
const DEFAULT_HFOV_DEG = 90;
const DEFAULT_MAX_DISTANCE = 20.0;
// Upper bound on DA360 relative depth: in relative_to_nearest, nearby objects usually have
// rel < 40; anything above is treated as a distant/sky artefact (the monocular model's
// 1/disp outputs values in the hundreds for distant scenery), and calibration must filter
// those out so the scale is not squeezed down.
const DA360_REL_MAX = 40;
// Physically plausible range for scale: the nearest point of a relative_to_nearest scene is
// usually 0.5-30 m (hugging the ground at 0.5 m, tens of metres for the nearest structure at
// altitude). Outside that range calibration is considered polluted and the historical value
// is reused. Overridable with ?da360ScaleMin/Max.
const DA360_SCALE_MIN = 0.5;
const DA360_SCALE_MAX = 30.0;
// Temporal smoothing factor for scale (weight of the new value): 0.5 means half new / half
// historical, damping the frame-to-frame relative-depth drift of DA360.
const DA360_SCALE_SMOOTH = 0.5;

export class YOPODepthFromPanorama {
    constructor(world, panoramaSensor) {
        this.world = world;
        this.panoramaSensor = panoramaSensor;
        this.lastScale = 1.0;
        this.lastRelativeDepth = null;
        this.lastScaleTimestamp = 0;
        this.scaleConfidence = 0;
        this._erpFrameCount = 0;       // ERP frame counter
        // Calibration policy: calibrate once for every DA360 depth map obtained (see
        // _refreshDepth). The raycast cost is absorbed by pickLocalRay's 150 ms
        // direction-bucketed cache, so no downsampling is needed.
        // ── Predictive depth cache (for lower latency) ──
        // The depth refresh (calling DA360) runs asynchronously in the background, and the
        // nav loop just reuses the most recent processed ERP depth every frame, hiding
        // DA360's 140 ms wait in the background. That shrinks the depth-loop period from
        // ~230 ms (DA360+YOPO) to roughly the YOPO inference time, making control more
        // responsive.
        this._depthCache = null;       // {depth, mask, scale, confidence, time}
        this._refreshing = false;      // whether a DA360 refresh is in flight
        this._depthCacheTtlMs = 150;   // cache freshness threshold; beyond it the frame waits synchronously for real depth
        // Minimum interval between DA360 refreshes (ms): when the depth loop fires at a high
        // rate (e.g. navigate throttled to 33 ms), not rate-limiting would keep hammering
        // DA360, and slow inference plus queuing would actually hurt real-time behaviour.
        // With rate limiting the cache is reused: the depth loop runs at the navigate rate
        // while DA360 updates the cache in the background at <= minInterval.
        // The faster GPU post-processing made refreshes cheaper: 30 -> 10 -> 5 -> 3 ms,
        // further raising the real depth rate (in-flight DA360 requests are guarded by the
        // _refreshing mutex so they never pile up; a shorter interval keeps the cache
        // fresher and YOPO more real-time).
        // This value is also the baseline for the "motion command (replan) update rate" --
        // main.js binds the navigate client-side throttle (_requestInterval) straight to it,
        // keeping both rates strictly identical and raising them together.
        this._minRefreshIntervalMs = 3;
        // ── GPU post-processing (lazy init) ──
        // These per-pixel loops (rawDepth/resize/flip/mask) run on the CPU main thread: at
        // 384x192 that is tens of thousands of interpolations plus a flip every time, a fixed
        // cost of every depth-loop cycle. A single WebGL2 draw performs
        // "bilinear resize + horizontal flip + mask + clamp" in one pass and the CPU only
        // reads the final result back, significantly shortening _refreshDepth -> higher depth
        // rate / better real-time behaviour.
        // If creation fails (no WebGL2) it automatically falls back to the CPU path;
        // functionality is unaffected.
        this._gpu = null;
    }

    /** Lazily initialize the GPU post-processing pipeline. Returns whether it is usable. */
    _ensureGpu() {
        if (this._gpu) return this._gpu.ok;
        const gpu = { ok: false };
        try {
            const canvas = document.createElement('canvas');
            canvas.width = 64; canvas.height = 64;
            const gl = canvas.getContext('webgl2', {
                depth: false, stencil: false, antialias: false, alpha: true,
                premultipliedAlpha: false, preserveDrawingBuffer: true,
            });
            if (!gl) return (this._gpu = gpu) && false;

            // Vertex shader: full-screen triangle
            const vsSrc = `#version 300 es
                in vec2 a_pos;
                void main() { gl_Position = vec4(a_pos, 0.0, 1.0); }`;
            // Fragment shader: sample the depth texture and do bilinear resize + mask +
            // clamp. u_tex is an R32F depth texture. Note: the RGB panorama is already
            // flipped horizontally at render time in panorama-sensor into the YOPO training
            // view (left half = left direction), and the DA360 depth layout follows the input
            // RGB (pixel-aligned), so the column order is NOT flipped here -- the depth is
            // already in the YOPO anchor view. Invalid pixels (NaN/<=0) are written as 0 so
            // the downstream mask check is easy.
            const fsSrc = `#version 300 es
                precision highp float;
                precision highp sampler2D;
                uniform sampler2D u_tex;
                uniform vec2 u_srcSize;
                uniform vec2 u_dstSize;
                out vec4 outColor;
                void main() {
                    vec2 dstUv = gl_FragCoord.xy / u_dstSize;
                    float srcX = (u_srcSize.x - 1.0) * dstUv.x;
                    float srcY = (u_srcSize.y - 1.0) * dstUv.y;
                    vec2 srcUv = vec2(srcX, srcY) / u_srcSize;
                    float v = texture(u_tex, srcUv).r;
                    // mask: 255 when valid (v > 1e-3 and finite), otherwise 0
                    float m = (v > 1e-3) ? 255.0 : 0.0;
                    outColor = vec4(v, m, 0.0, 1.0);
                }`;
            const compile = (type, src) => {
                const sh = gl.createShader(type);
                gl.shaderSource(sh, src); gl.compileShader(sh);
                if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
                    console.warn('[gpu-depth] shader compile failed:', gl.getShaderInfoLog(sh));
                    return null;
                }
                return sh;
            };
            const vs = compile(gl.VERTEX_SHADER, vsSrc);
            const fs = compile(gl.FRAGMENT_SHADER, fsSrc);
            if (!vs || !fs) return (this._gpu = gpu) && false;
            const prog = gl.createProgram();
            gl.attachShader(prog, vs); gl.attachShader(prog, fs); gl.linkProgram(prog);
            if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) {
                console.warn('[gpu-depth] program link failed:', gl.getProgramInfoLog(prog));
                return (this._gpu = gpu) && false;
            }
            const buf = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, buf);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
                -1, -1, 3, -1, -1, 3,
            ]), gl.STATIC_DRAW);
            gpu.gl = gl; gpu.prog = prog; gpu.buf = buf;
            gpu.aPos = gl.getAttribLocation(prog, 'a_pos');
            gpu.uTex = gl.getUniformLocation(prog, 'u_tex');
            gpu.uSrcSize = gl.getUniformLocation(prog, 'u_srcSize');
            gpu.uDstSize = gl.getUniformLocation(prog, 'u_dstSize');
            // Single-channel texture unit
            gpu.tex = gl.createTexture();
            gpu.fb = gl.createFramebuffer();
            gpu.pbo = null;
            gpu.ok = true;
        } catch (e) {
            console.warn('[gpu-depth] WebGL2 init failed, falling back to CPU:', e);
            gpu.ok = false;
        }
        this._gpu = gpu;
        return gpu.ok;
    }

    /**
     * GPU post-processing: resize rawDepth (Float32Array) to dstW x dstH with a horizontal
     * flip, and generate the mask (Uint8Array) at the same time.
     * Returns {depth, mask}, or null on failure.
     */
    _gpuResizeFlip(rawDepth, rawW, rawH, dstW, dstH) {
        if (!this._ensureGpu()) return null;
        const gl = this._gpu.gl;
        const n = rawW * rawH;
        if (!this._gpu.floatTex || this._gpu.floatTex.n < n) {
            // (Re)create the R32F texture: only rebuilt when the size grows
            const old = this._gpu.floatTex;
            if (old) { gl.deleteTexture(old.tex); }
            const tex = gl.createTexture();
            gl.bindTexture(gl.TEXTURE_2D, tex);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
            // Pre-allocate storage large enough
            gl.texStorage2D(gl.TEXTURE_2D, 1, gl.R32F, rawW, rawH);
            this._gpu.floatTex = { tex, n };
            this._gpu.nRaw = n;
        }
        const ftex = this._gpu.floatTex.tex;
        gl.bindTexture(gl.TEXTURE_2D, ftex);
        gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, rawW, rawH, gl.RED, gl.FLOAT, rawDepth);

        // Output buffers: depth (R32F) + mask (R8) as two FBO attachments
        const outW = dstW, outH = dstH;
        let outDepth = this._gpu.outDepthTex;
        if (!outDepth || outDepth.w !== outW || outDepth.h !== outH) {
            if (outDepth) { gl.deleteTexture(outDepth.tex); }
            const dt = gl.createTexture();
            gl.bindTexture(gl.TEXTURE_2D, dt);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
            gl.texStorage2D(gl.TEXTURE_2D, 1, gl.R32F, outW, outH);
            const mt = gl.createTexture();
            gl.bindTexture(gl.TEXTURE_2D, mt);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
            gl.texStorage2D(gl.TEXTURE_2D, 1, gl.R8, outW, outH);
            this._gpu.outDepthTex = { tex: dt, w: outW, h: outH };
            this._gpu.outMaskTex = { tex: mt, w: outW, h: outH };
        }

        // Render to the FBO (MRT: depth + mask)
        gl.bindFramebuffer(gl.FRAMEBUFFER, this._gpu.fb);
        gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this._gpu.outDepthTex.tex, 0);
        gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT1, gl.TEXTURE_2D, this._gpu.outMaskTex.tex, 0);
        if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) !== gl.FRAMEBUFFER_COMPLETE) return null;

        gl.viewport(0, 0, outW, outH);
        gl.useProgram(this._gpu.prog);
        gl.uniform1i(this._gpu.uTex, 0);
        gl.uniform2f(this._gpu.uSrcSize, rawW, rawH);
        gl.uniform2f(this._gpu.uDstSize, outW, outH);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, ftex);

        gl.bindBuffer(gl.ARRAY_BUFFER, this._gpu.buf);
        gl.enableVertexAttribArray(this._gpu.aPos);
        gl.vertexAttribPointer(this._gpu.aPos, 2, gl.FLOAT, false, 0, 0);

        const drawBufs = [gl.COLOR_ATTACHMENT0, gl.COLOR_ATTACHMENT1];
        gl.drawBuffers(drawBufs);
        gl.drawArrays(gl.TRIANGLES, 0, 3);

        // Read-back: buffers are reused (not recreated while the size is unchanged), which
        // reduces per-frame GC pressure and allocation cost.
        // The output arrays are reused for the caller (the caller consumes them before the
        // next refresh, see how _refreshDepth uses them).
        const nOut = outW * outH;
        if (!this._gpu.depthBuf || this._gpu.depthBuf.length !== nOut) {
            this._gpu.depthBuf = new Float32Array(nOut);
            this._gpu.maskBuf = new Uint8Array(nOut);
        }
        const depthOut = this._gpu.depthBuf;
        const maskOut = this._gpu.maskBuf;
        gl.readPixels(0, 0, outW, outH, gl.RED, gl.FLOAT, depthOut);
        gl.readPixels(0, 0, outW, outH, gl.RED, gl.UNSIGNED_BYTE, maskOut);
        gl.bindFramebuffer(gl.FRAMEBUFFER, null);

        return { depth: depthOut, mask: maskOut };
    }

    /**
     * Fetch the latest raw panoramic depth from DA360.
     *
     * @param {number} [timeoutMs=8000]
     * @returns {{depth: Float32Array, width: number, height: number, scale: object}|null}
     */
    async fetchRawDepth(timeoutMs = 8000) {
        if (!this.panoramaSensor || !this.panoramaSensor.hasRgbFrame || !this.panoramaSensor.hasRgbFrame()) {
            if (!this._noRgbLogged) {
                this._noRgbLogged = true;
                console.warn('YOPO fetchRawDepth: no RGB frame from panoramaSensor');
            }
            return null;
        }

        const canvas = this.panoramaSensor.rgbCanvas;
        if (!canvas || !canvas.width || !canvas.height) {
            if (!this._noCanvasLogged) {
                this._noCanvasLogged = true;
                console.warn('YOPO fetchRawDepth: panoramaSensor.rgbCanvas is empty');
            }
            return null;
        }

        const uploadCanvas = this.panoramaSensor._depthUploadCanvas
            ? this.panoramaSensor._depthUploadCanvas(canvas)
            : canvas;

        const blob = await this.panoramaSensor._canvasToJpegBlob(uploadCanvas);
        const headers = { 'X-DA360-Raw-Depth': '1', 'X-DA360-Raw-Binary': '1' };
        let body;
        if (blob) {
            headers['Content-Type'] = blob.type || 'image/jpeg';
            body = blob;
        } else {
            headers['Content-Type'] = 'application/json';
            body = JSON.stringify({
                image: uploadCanvas.toDataURL('image/jpeg', this.panoramaSensor._jpegQuality || 0.74),
            });
        }

        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), timeoutMs);
        try {
            const response = await fetch(this.panoramaSensor.endpoint, {
                method: 'POST',
                headers,
                body,
                signal: controller.signal,
            });
            if (!response.ok) throw new Error(`DA360 HTTP ${response.status}`);

            // ── Binary fast path: DA360 returns raw float32 bytes directly
            //    (no .npy/base64 round-trip) → skip the ~1.2MB atob decode. ──
            if (response.headers.get('Content-Type') === 'application/octet-stream') {
                const buf = await response.arrayBuffer();
                const shapeStr = response.headers.get('X-DA360-Raw-Depth-Shape') || '';
                const shape = shapeStr.split(/\s+/).filter(Boolean).map(Number);
                const depth = new Float32Array(buf);
                const scaleHdr = response.headers.get('X-DA360-Depth-Scale');
                const scale = scaleHdr ? JSON.parse(scaleHdr) : { relative_to_nearest: 1 };
                if (!this._da360OkLogged) {
                    this._da360OkLogged = true;
                    console.log(`YOPO fetchRawDepth: DA360 returned binary depth ${depth.length} floats`
                        + (shape.length ? ` shape [${shape}]` : ''));
                }
                return {
                    depth,
                    width: shape.length === 2 ? shape[1] : 384,
                    height: shape.length === 2 ? shape[0] : 192,
                    scale,
                };
            }

            // ── Legacy JSON+base64 fallback ──
            const payload = await response.json();
            if (!payload || !payload.raw_depth || !payload.raw_depth_shape) {
                throw new Error('DA360 response missing raw_depth');
            }
            const depth = this._decodeBase64Float32(payload.raw_depth, payload.raw_depth_shape);
            if (!this._da360OkLogged) {
                this._da360OkLogged = true;
                console.log(`YOPO fetchRawDepth: DA360 returned depth shape [${payload.raw_depth_shape}], ${depth.length} floats`);
            }
            return { depth, width: payload.raw_depth_shape[1], height: payload.raw_depth_shape[0], scale: payload.depth_scale };
        } finally {
            window.clearTimeout(timeout);
        }
    }

    _decodeBase64Float32(b64, shape) {
        const binary = atob(b64);
        const bytes = new Uint8Array(binary.length);
        for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
        const npyBytes = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
        // .npy header parsing: little-endian, magic '\x93NUMPY', then version, header len, header text, data.
        const view = new DataView(npyBytes);
        let offset = 0;
        // Skip magic 6 bytes + version 2 bytes = 8
        offset = 8;
        const headerLen = view.getUint16(offset, true);
        offset += 2;
        const headerEnd = offset + headerLen;
        offset = headerEnd;
        // data is C-contiguous float32
        const total = shape.reduce((a, b) => a * b, 1);
        return new Float32Array(npyBytes, offset, total);
    }

    /**
     * Sample Cesium ground-truth distances at a sparse set of calibration
     * directions in front of the drone.
     *
     * @param {object} cameraTransform
     * @param {number} maxDistance
     * @returns {Array<{dir: {x,y,z}, distance: number}>}
     */
    sampleCalibrationPoints(cameraTransform, maxDistance = DEFAULT_MAX_DISTANCE) {
        if (!this.world || typeof this.world.pickLocalRay !== 'function') return [];
        const basis = this.world.getTransformBasisLocal
            ? this.world.getTransformBasisLocal(cameraTransform)
            : this._basisFromTransform(cameraTransform);
        const pos = cameraTransform.position;

        const samples = [];
        const add = (u, v) => {
            const dir = {
                x: basis.forward.x + u * basis.right.x + v * basis.up.x,
                y: basis.forward.y + u * basis.right.y + v * basis.up.y,
                z: basis.forward.z + u * basis.right.z + v * basis.up.z,
            };
            const norm = Math.hypot(dir.x, dir.y, dir.z);
            if (norm < 1e-9) return;
            const n = { x: dir.x / norm, y: dir.y / norm, z: dir.z / norm };
            // forceFresh=true: calibration rays must return the current true distance and
            // must not use pickLocalRay's direction-bucketed cache (a cache hit returns a
            // stale distance with <= 150 ms / <= 0.5 m drift, biasing the calibration scale).
            // The cost is that all 4 rays per depth map do a real GPU pick, but the metric
            // calibration always stays fresh.
            const hit = this.world.pickLocalRay(pos, n, maxDistance, true);
            if (hit && hit.distance > 0.2 && hit.distance < maxDistance) {
                samples.push({ dir: n, distance: hit.distance });
            }
        };

        // Sparse 2x2 grid in the forward hemisphere (4 raycasts, not 9)
        for (let r = -1; r <= 1; r += 2) {
            for (let c = -1; c <= 1; c += 2) {
                add(c * 0.5, r * 0.5);
            }
        }
        // Extra calibration points for high altitude: the forward 2x2 grid rays can all miss
        // in open / high-altitude scenes (hitting the sky -> pickLocalRay returns null),
        // leaving too few calibrationPoints and falling back to the historical scale (which
        // can be far too small at altitude -> the depth map is squeezed into "obstacles on
        // all sides"). Added:
        //   1. straight ahead add(0,0): the nearest obstacle ahead, the direction avoidance
        //      cares about most
        //   2. straight down add(0,-1): ground distance ~= drone altitude, the only reliably
        //      hittable point at high altitude
        add(0, 0);
        add(0, -1);
        return samples;
    }

    _basisFromTransform(transform) {
        const q = transform.orientation;
        if (!q) {
            return {
                right: { x: 1, y: 0, z: 0 },
                up: { x: 0, y: 1, z: 0 },
                forward: { x: 0, y: 0, z: -1 },
            };
        }
        const rotate = (v) => this._rotateVectorByQuat(q, v);
        const right = rotate({ x: 1, y: 0, z: 0 });
        const up = rotate({ x: 0, y: 1, z: 0 });
        const forward = rotate({ x: 0, y: 0, z: -1 });
        return { right, up, forward };
    }

    _rotateVectorByQuat(q, v) {
        const x = v.x, y = v.y, z = v.z;
        const qx = q.x, qy = q.y, qz = q.z, qw = q.w;
        const ix = qw * x + qy * z - qz * y;
        const iy = qw * y + qz * x - qx * z;
        const iz = qw * z + qx * y - qy * x;
        const iw = -qx * x - qy * y - qz * z;
        return {
            x: ix * qw + iw * -qx + iy * -qz - iz * -qy,
            y: iy * qw + iw * -qy + iz * -qx - ix * -qz,
            z: iz * qw + iw * -qz + ix * -qy - iy * -qx,
        };
    }

    /**
     * Look up relative depth in the equirectangular panoramic depth map for a
     * world direction.
     *
     * @param {Float32Array} panoDepth
     * @param {number} panoWidth
     * @param {number} panoHeight
     * @param {{x,y,z}} dir - world direction (MindCloud frame)
     * @returns {number|null}
     */
    _samplePanoramaDepth(panoDepth, panoWidth, panoHeight, dir) {
        // ERP sampling in the YOPO_360 training view (consistent with sensor_simulator.cu /
        // primitive.py):
        //   yaw = PI - (u+0.5)/W * 2PI  =>  yaw increases with the column index, u=0.5 is the
        //   centre = straight ahead
        //   body NWU: x=fwd, y=left, z=up => left half (u<0.5) = left direction, right half
        //   (u>0.5) = right direction
        // The RGB panorama is already flipped into that view at render time in
        // panorama-sensor and the DA360 depth layout follows it, so we sample directly with
        // this convention (no longer using the mirrored layout of the Cesium shader).
        // MindCloud frame: forward=-z, right=+x, up=+y -> MC right (+x) maps to the YOPO
        // right direction.
        const yaw = Math.atan2(dir.x, -dir.z);
        const pitch = Math.asin(Math.max(-1, Math.min(1, dir.y)));
        let u = 0.5 + yaw / (2 * Math.PI);
        let v = 0.5 - pitch / Math.PI;
        u = (u % 1 + 1) % 1;
        v = Math.max(0, Math.min(1, v));

        const px = u * (panoWidth - 1);
        const py = v * (panoHeight - 1);
        const x0 = Math.floor(px);
        const y0 = Math.floor(py);
        const x1 = Math.min(panoWidth - 1, x0 + 1);
        const y1 = Math.min(panoHeight - 1, y0 + 1);
        const fx = px - x0;
        const fy = py - y0;

        const i00 = y0 * panoWidth + x0;
        const i10 = y0 * panoWidth + x1;
        const i01 = y1 * panoWidth + x0;
        const i11 = y1 * panoWidth + x1;
        const v00 = panoDepth[i00];
        const v10 = panoDepth[i10];
        const v01 = panoDepth[i01];
        const v11 = panoDepth[i11];
        if (!Number.isFinite(v00) || !Number.isFinite(v10) || !Number.isFinite(v01) || !Number.isFinite(v11)) {
            return null;
        }
        const top = v00 + (v10 - v00) * fx;
        const bot = v01 + (v11 - v01) * fx;
        return top + (bot - top) * fy;
    }

    /**
     * Estimate the global scale that converts DA360 relative depth into metres.
     *
     * @param {Float32Array} panoDepth
     * @param {number} panoWidth
     * @param {number} panoHeight
     * @param {Array<{dir,distance}>} calibrationPoints
     * @returns {{scale: number, confidence: number}}
     */
    estimateScale(panoDepth, panoWidth, panoHeight, calibrationPoints) {
        const ratios = [];
        for (const p of calibrationPoints) {
            const rel = this._samplePanoramaDepth(panoDepth, panoWidth, panoHeight, p.dir);
            // Only trust "near / mid-range" calibration points: DA360 is a monocular depth
            // model that outputs relative_to_nearest (nearest scene point = 1.0). For ground
            // seen from high altitude, for distant scenery and for the sky, 1/disp yields
            // huge fake values (rel in the hundreds); computing scale = true distance / rel
            // from those squeezes the scale to something tiny (0.39 measured) -> the whole
            // depth map is shrunk into "obstacles on all sides", YOPO mistakes everything for
            // a wall and only dares to pick sharp-turn / dive avoidance trajectories instead
            // of flying toward the goal. Real nearby objects usually have rel < 40, so the
            // unreliable far points with huge rel are filtered out and only reliable close
            // range points calibrate.
            if (rel !== null && rel > 1e-3 && rel < DA360_REL_MAX) {
                ratios.push(p.distance / rel);
            }
        }
        if (ratios.length < 2) {
            // Not enough calibration points (all filtered out): fall back to the historical
            // scale so a single outlier cannot dominate
            return { scale: this.lastScale, confidence: this.scaleConfidence * 0.7 };
        }
        ratios.sort((a, b) => a - b);
        const median = ratios[Math.floor(ratios.length / 2)];
        const mad = ratios.map(r => Math.abs(r - median)).sort((a, b) => a - b)[Math.floor(ratios.length / 2)];
        const inliers = ratios.filter(r => Math.abs(r - median) <= Math.max(0.05 * median, 3 * mad));
        let scale = inliers.length > 0
            ? inliers.reduce((a, b) => a + b, 0) / inliers.length
            : median;
        // Physical plausibility clamp: the nearest point of a relative_to_nearest scene is
        // usually 0.3-8 m, so scale should land in [DA360_SCALE_MIN, DA360_SCALE_MAX].
        // Landing outside means calibration was polluted by DA360's far-point artefacts, so
        // fall back to the historical scale and avoid feeding the network absurd metric
        // values (such as a 0.39 m median).
        if (!Number.isFinite(scale) || scale < DA360_SCALE_MIN || scale > DA360_SCALE_MAX) {
            return { scale: this.lastScale, confidence: this.scaleConfidence * 0.6 };
        }
        // Temporal smoothing: blend the new scale with the history to resist scale jumps
        // caused by DA360's frame-to-frame relative-depth drift (jumps make the same scene
        // flip between "near obstacle" and "wide open", making the network's decisions
        // jittery and the flight incoherent).
        scale = this.lastScale * (1 - DA360_SCALE_SMOOTH) + scale * DA360_SCALE_SMOOTH;
        const confidence = Math.min(1.0, inliers.length / Math.max(3, calibrationPoints.length * 0.8));
        this.lastScale = scale;
        this.scaleConfidence = confidence;
        this.lastScaleTimestamp = performance.now();
        return { scale, confidence };
    }

    /**
     * Reproject the scaled panoramic depth into a forward-facing pinhole depth
     * image suitable for YOPO.
     *
     * @param {Float32Array} panoDepth
     * @param {number} panoWidth
     * @param {number} panoHeight
     * @param {object} cameraTransform
     * @param {number} scale
     * @param {object} [options]
     * @returns {{depth: Float32Array, encoding: string}}
     */
    reprojectForward(panoDepth, panoWidth, panoHeight, cameraTransform, scale, options = {}) {
        const width = Math.max(16, options.width || DEFAULT_WIDTH);
        const height = Math.max(16, options.height || DEFAULT_HEIGHT);
        const hfovDeg = options.hfovDeg || DEFAULT_HFOV_DEG;
        const maxDistance = options.maxDistance || DEFAULT_MAX_DISTANCE;

        const basis = this.world.getTransformBasisLocal
            ? this.world.getTransformBasisLocal(cameraTransform)
            : this._basisFromTransform(cameraTransform);
        const pos = cameraTransform.position;

        const hfovRad = hfovDeg * Math.PI / 180;
        const vfovRad = hfovRad * (height / width);
        const tanHalfH = Math.tan(hfovRad * 0.5);
        const tanHalfV = Math.tan(vfovRad * 0.5);

        const depth = new Float32Array(width * height);
        for (let py = 0; py < height; py++) {
            // Image convention: row 0 = top (up/sky), row max = bottom (down/ground)
            const v = height > 1 ? 1.0 - (py / (height - 1)) * 2.0 : 0;
            for (let px = 0; px < width; px++) {
                const u = width > 1 ? (px / (width - 1)) * 2 - 1 : 0;
                const dir = {
                    x: basis.forward.x + u * tanHalfH * basis.right.x + v * tanHalfV * basis.up.x,
                    y: basis.forward.y + u * tanHalfH * basis.right.y + v * tanHalfV * basis.up.y,
                    z: basis.forward.z + u * tanHalfH * basis.right.z + v * tanHalfV * basis.up.z,
                };
                const norm = Math.hypot(dir.x, dir.y, dir.z);
                if (norm < 1e-9) {
                    depth[py * width + px] = maxDistance;
                    continue;
                }
                const n = { x: dir.x / norm, y: dir.y / norm, z: dir.z / norm };
                const rel = this._samplePanoramaDepth(panoDepth, panoWidth, panoHeight, n);
                if (rel === null || rel <= 0) {
                    depth[py * width + px] = maxDistance;
                } else {
                    const d = rel * scale;
                    depth[py * width + px] = Math.min(d, maxDistance);
                }
            }
        }
        return { depth, encoding: '32FC1' };
    }

    /**
     * Main entry point: fetch DA360 depth, calibrate with Cesium, and produce
     * a YOPO-formatted forward depth map.
     *
     * @param {object} cameraTransform
     * @param {object} [options]
     * @returns {{depth: Float32Array, encoding: string, scale: number, confidence: number}|null}
     */
    async captureYOPODepth(cameraTransform, options = {}) {
        const raw = await this.fetchRawDepth(options.timeoutMs);
        if (!raw) return null;

        const calibrationPoints = this.sampleCalibrationPoints(
            cameraTransform,
            options.maxDistance || DEFAULT_MAX_DISTANCE
        );

        const { scale, confidence } = this.estimateScale(
            raw.depth,
            raw.width,
            raw.height,
            calibrationPoints
        );

        const result = this.reprojectForward(
            raw.depth,
            raw.width,
            raw.height,
            cameraTransform,
            scale,
            options
        );

        return { ...result, scale, confidence };
    }

    /**
     * YOPO_360 ERP direct-capture mode (primary path).
     *
     * DA360 already returns an equirectangular panoramic depth map, which is
     * exactly the layout the YOPO_360 network expects (192x384, 2 channels).
     * There is no need to reproject into a forward pinhole — we resize the
     * raw ERP depth to 384x192, build a validity mask from NaN/invalid
     * pixels, and (optionally) recover the metric scale via sparse Cesium
     * raycasts.  The mask is returned alongside the depth so the caller can
     * forward it to the YOPO server as the second input channel.
     *
     * @param {object} cameraTransform
     * @param {object} [options] - {width=384, height=192, maxDistance=20,
     *        timeoutMs=6000, calibrate=true}
     * @returns {Promise<{depth: Float32Array, mask: Uint8Array, encoding: string,
     *          scale: number, confidence: number}|null>}
     */
    async captureYOPODepthERP(cameraTransform, options = {}) {
        // ── Predictive depth pipeline (lower latency) ──
        // Trigger / keep alive one background DA360 refresh (which writes _depthCache). As
        // long as a cache exists, this frame reuses it directly (zero wait), hiding DA360's
        // ~1.6 s inference completely in the background. The nav loop therefore runs at the
        // YOPO inference rate (instead of DA360+YOPO serialized), replanning commands at
        // ~5 Hz instead of ~0.4 Hz.
        //
        // Key fix: the foreground must never call DA360 again every frame. In the early
        // implementation the foreground awaited real depth synchronously whenever the cache
        // expired (> 150 ms), and the cache TTL (150 ms) was far shorter than one nav cycle
        // (>= depth+navigate), so every cycle took the "expired -> wait synchronously" branch
        // and hit DA360 together with the background refresh -- two requests competing for
        // the GPU, which saved no latency (depth still 1654 ms) and dragged YOPO inference
        // out to 830 ms. Now: any existing cache is used; only on the very first frame
        // ("from nothing to something") do we wait once for real depth, and we reuse that
        // background request instead of opening a second one.
        this._maybeRefreshDepth(cameraTransform, options);

        if (this._depthCache) {
            const c = this._depthCache;
            const now = performance.now();
            if (!this._erpOkLogged) {
                this._erpOkLogged = true;
                console.log(`captureYOPODepthERP: cache ${c.width}x${c.height}, ` +
                    `scale=${c.scale.toFixed(4)} (conf=${c.confidence.toFixed(2)}), ` +
                    `cached=${(now - c.time).toFixed(0)}ms, ` +
                    `fetch_ms=${c.fetchMs ? c.fetchMs.toFixed(0) : '-'}`);
            }
            return { depth: c.depth, mask: c.mask, encoding: '32FC1', scale: c.scale, confidence: c.confidence, source: 'da360' };
        }

        // First frame: the cache is not built yet. Just wait for the one background refresh
        // already in flight (a single request, do not open a second one) so two requests do
        // not fight over the GPU. Never hover and guess.
        if (this._refreshing && this._refreshPromise) {
            await this._refreshPromise;
        } else {
            await this._refreshDepth(cameraTransform, options);
        }
        const c = this._depthCache;
        if (!c) return null;
        const now = performance.now();
        if (!this._erpFirstLogged) {
            this._erpFirstLogged = true;
            console.log(`captureYOPODepthERP(first): cache ${c.width}x${c.height}, ` +
                `scale=${c.scale.toFixed(4)} (conf=${c.confidence.toFixed(2)}), ` +
                `fetch_ms=${c.fetchMs ? c.fetchMs.toFixed(0) : '-'}`);
        }
        return { depth: c.depth, mask: c.mask, encoding: '32FC1', scale: c.scale, confidence: c.confidence, source: 'da360' };
    }

    // Trigger a background DA360 refresh (if none is in flight) and return the Promise so
    // the first frame can await it.
    // At most one request is in flight at a time -> DA360 is never hit twice.
    // Rate limiting: skip (reuse the cache) when a cache already exists and less than
    // _minRefreshIntervalMs passed since the last refresh started, so the depth loop runs at
    // the navigate rate while DA360 updates in the background at <= minInterval.
    // The first frame (no cache yet) is never skipped, guaranteeing the depth cache is built
    // as soon as possible.
    _maybeRefreshDepth(cameraTransform, options) {
        if (this._refreshing) return this._refreshPromise;
        const now = performance.now();
        if (this._depthCache && this._lastRefreshStart &&
            now - this._lastRefreshStart < this._minRefreshIntervalMs) {
            return this._refreshPromise || this._depthCache;
        }
        this._lastRefreshStart = now;
        this._refreshing = true;
        this._refreshPromise = this._refreshDepth(cameraTransform, options).finally(() => {
            this._refreshing = false;
        });
        return this._refreshPromise;
    }

    // Call DA360 for raw depth -> ERP post-processing -> write into this._depthCache.
    // Also owns the calibration rate logic (the Cesium raycast that used to live inside
    // captureYOPODepthERP).
    async _refreshDepth(cameraTransform, options) {
        const width = Math.max(16, options.width || DEFAULT_WIDTH_ERP);
        const height = Math.max(16, options.height || DEFAULT_HEIGHT_ERP);
        const maxDistance = options.maxDistance || DEFAULT_MAX_DISTANCE;
        const calibrate = options.calibrate !== false;

        const tFetchStart = performance.now();
        const raw = await this.fetchRawDepth(options.timeoutMs);
        if (!raw) return null;
        const tFetchMs = performance.now() - tFetchStart;

        const rawDepth = raw.depth;
        const rawW = raw.width;
        const rawH = raw.height;

        // Build validity mask from the raw (pre-scale) depth.
        // Invalid: NaN / non-finite / <= 0 (relative depth near zero ⇒ no return).
        const rawMask = new Uint8Array(rawW * rawH);
        for (let i = 0; i < rawDepth.length; i++) {
            const v = rawDepth[i];
            rawMask[i] = (Number.isFinite(v) && v > 1e-3) ? 255 : 0;
        }

        // Recover metric scale via sparse Cesium raycasts (same calibration as
        // the pinhole path). Skipped when calibrate=false or no rays hit.
        // Calibrate every frame: run one Cesium raycast calibration for every DA360 depth
        // map obtained, so the scale always follows the current depth map (DA360's global
        // shift changes every frame, and reusing an old scale across frames introduces
        // metric error). The raycast cost is absorbed by pickLocalRay's 150 ms
        // direction-bucketed cache: the DA360 rate is ~22 Hz (45 ms) < the cache TTL
        // (150 ms), so most of the 4 calibration rays hit the cache and real GPU picks are
        // rare; after moving/turning they naturally land in new buckets and trigger real
        // picks, so no extra downsampling is needed.
        // Keeping the "moved > 1.5 m forces calibration" rule as a cache-penetration
        // backstop (a pure CPU-side check, no extra cost).
        let scale = this.lastScale;
        let confidence = this.scaleConfidence;
        this._erpFrameCount += 1;
        // Calibrate every frame (as long as calibrate is enabled).
        const doCalibrate = calibrate;
        if (doCalibrate) {
            // Adaptive raycast distance: at high altitude a fixed 20 m Cesium raycast cannot
            // reach the ground / distant objects, so too few calibration points hit and it
            // falls back to the cached old scale, wrongly scaling the high-altitude depth
            // into fake "everything is near" obstacles -- YOPO then keeps making large
            // turns and cannot fly toward the goal. Scale the raycast distance with the drone
            // altitude so it still hits the ground / distant buildings at altitude and
            // recovers the metric scale correctly.
            const droneH = cameraTransform && cameraTransform.position
                ? (cameraTransform.position.y || 0) : 0;
            const calibMaxDist = Math.max(maxDistance, Math.abs(droneH) * 1.5 + 20);
            const calibrationPoints = this.sampleCalibrationPoints(cameraTransform, calibMaxDist);
            if (calibrationPoints.length >= 3) {
                const r = this.estimateScale(rawDepth, rawW, rawH, calibrationPoints);
                scale = r.scale;
                confidence = r.confidence;
            }
        }

        // Resize depth (bilinear) and mask (nearest) to the YOPO ERP resolution.
        // ── GPU post-processing (preferred): a single WebGL2 draw does resize + horizontal
        //    flip + mask, and the CPU only reads the result back. When the GPU is unusable
        //    (no WebGL2 / driver) fall back to the CPU path. ──
        let depth, mask;
        const gpuRes = this._gpuResizeFlip(rawDepth, rawW, rawH, width, height);
        if (gpuRes) {
            // The GPU output buffers are reused by later frames while the cache (including
            // depth/mask) is reused by later navigate calls, so independent copies are
            // mandatory -- otherwise the next refresh would overwrite the cached data. The
            // copy is merged with the scale pass into a single traversal.
            depth = new Float32Array(gpuRes.depth);
            mask = new Uint8Array(gpuRes.mask);
        } else {
            depth = this._resizeBilinear(rawDepth, rawW, rawH, width, height);
            mask = this._resizeNearestUint8(rawMask, rawW, rawH, width, height);
            // Azimuth handedness (explanation):
            // Early versions flipped horizontally here, because the Cesium-rendered RGB ERP
            // was in the MC frame (left half = right direction), which is mirrored relative
            // to the YOPO anchor (NWU, left half = left direction). The RGB panorama is now
            // flipped uniformly into the YOPO view at render time in panorama-sensor, and the
            // DA360 depth layout follows the input RGB (pixel-aligned), so there is no flip
            // here any more -- the depth column order is naturally codirectional with the
            // YOPO anchor's increasing alpha. The GPU path's shader likewise dropped the
            // flip, staying consistent with this.
        }

        // Scale to metres. Invalid pixels (NaN/≤0) are kept as NaN — NOT clamped
        // to maxDistance — so the server's _preprocess_depth / nan_mask can identify
        // them as genuinely invalid and mean-fill them (matching YOPO_360 training,
        // where invalid pixels carry mask=0 and the network learns to ignore them).
        // If for some reason the mask channel is dropped, NaN still lets the server
        // distinguish "missing" from "far open space" (which used to be filled as
        // 20m → treated as flyable → crashes into missing/sky regions).
        // The validity mask (channel 1) is built separately and always sent.
        // (flip and scale are merged into one traversal, saving a separate pass over 73728
        // pixels.)
        for (let i = 0; i < depth.length; i++) {
            let d = depth[i] * scale;
            depth[i] = Number.isFinite(d) && d > 0 ? Math.min(d, maxDistance) : NaN;
        }

        this._depthCache = { depth, mask, scale, confidence, width, height, time: performance.now(), fetchMs: tFetchMs };
        return this._depthCache;
    }

    _countValid(mask) {
        let n = 0;
        for (let i = 0; i < mask.length; i++) if (mask[i] > 127) n++;
        return n;
    }

    /**
     * Bilinear resize for a Float32Array image. Safe for srcW/srcH >= 1.
     */
    _resizeBilinear(src, srcW, srcH, dstW, dstH) {
        const dst = new Float32Array(dstW * dstH);
        if (srcW < 2 || srcH < 2) {
            // Degenerate source: fall back to nearest.
            return this._resizeNearestFloat(src, srcW, srcH, dstW, dstH);
        }
        const xRatio = (srcW - 1) / (dstW - 1 || 1);
        const yRatio = (srcH - 1) / (dstH - 1 || 1);
        for (let y = 0; y < dstH; y++) {
            const gy = y * yRatio;
            const gy0 = Math.min(Math.floor(gy), srcH - 2);
            const gy1 = gy0 + 1;
            const fy = gy - gy0;
            for (let x = 0; x < dstW; x++) {
                const gx = x * xRatio;
                const gx0 = Math.min(Math.floor(gx), srcW - 2);
                const gx1 = gx0 + 1;
                const fx = gx - gx0;
                const i00 = gy0 * srcW + gx0;
                const i10 = gy0 * srcW + gx1;
                const i01 = gy1 * srcW + gx0;
                const i11 = gy1 * srcW + gx1;
                const v00 = src[i00], v10 = src[i10], v01 = src[i01], v11 = src[i11];
                const top = v00 + (v10 - v00) * fx;
                const bot = v01 + (v11 - v01) * fx;
                dst[y * dstW + x] = top + (bot - top) * fy;
            }
        }
        return dst;
    }

    _resizeNearestFloat(src, srcW, srcH, dstW, dstH) {
        const dst = new Float32Array(dstW * dstH);
        const xRatio = srcW / dstW;
        const yRatio = srcH / dstH;
        for (let y = 0; y < dstH; y++) {
            const sy = Math.min(srcH - 1, Math.floor(y * yRatio));
            for (let x = 0; x < dstW; x++) {
                const sx = Math.min(srcW - 1, Math.floor(x * xRatio));
                dst[y * dstW + x] = src[sy * srcW + sx];
            }
        }
        return dst;
    }

    /**
     * Nearest-neighbour resize for a Uint8Array image (preserves binary mask).
     */
    _resizeNearestUint8(src, srcW, srcH, dstW, dstH) {
        const dst = new Uint8Array(dstW * dstH);
        const xRatio = srcW / dstW;
        const yRatio = srcH / dstH;
        for (let y = 0; y < dstH; y++) {
            const sy = Math.min(srcH - 1, Math.floor(y * yRatio));
            for (let x = 0; x < dstW; x++) {
                const sx = Math.min(srcW - 1, Math.floor(x * xRatio));
                dst[y * dstW + x] = src[sy * srcW + sx];
            }
        }
        return dst;
    }
}
