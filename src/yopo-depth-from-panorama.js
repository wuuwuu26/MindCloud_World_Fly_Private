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

export class YOPODepthFromPanorama {
    constructor(world, panoramaSensor) {
        this.world = world;
        this.panoramaSensor = panoramaSensor;
        this.lastScale = 1.0;
        this.lastRelativeDepth = null;
        this.lastScaleTimestamp = 0;
        this.scaleConfidence = 0;
        this._erpFrameCount = 0;       // ERP 帧计数（标定降频用）
        this._calibrateInterval = 20;  // 每 N 帧做一次 Cesium 标定（9 raycast 较慢, scale 变化慢）
        // ── 预测式深度缓存 (降低延迟用) ──
        // 深度刷新(打 DA360)在后台异步进行, 导航环每帧直接复用最近一次
        // 已处理好的 ERP 深度, 把 DA360 的 140ms 等待"藏"到后台, 使深度环
        // 周期从 ~230ms(DA360+YOPO) 降到 ≈YOPO 推理时间, 控制更跟手。
        this._depthCache = null;       // {depth, mask, scale, confidence, time}
        this._refreshing = false;      // 是否有 DA360 刷新在途
        this._depthCacheTtlMs = 150;   // 缓存新鲜度阈值; 超过则本帧同步等真实深度
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
            const hit = this.world.pickLocalRay(pos, n, maxDistance);
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
        // Equirectangular mapping consistent with the Cesium panorama shader
        // (cesium-world.js PanoramaProjection):
        //   yaw = PI - u*2PI, pitch = (v-0.5)*fov, 像素顶部(v=0)=向上(天空)
        //   u=0.5(中心)=前方(-z), u<0.5(左半)=+x(右), u>0.5(右半)=-x(左)
        //   MC body: forward=-z, right=+x, up=+y
        // 因此: 右转(x+)→u<0.5(图像左半); 上(+y)→v<0.5(图像顶部)。
        const yaw = Math.atan2(dir.x, -dir.z);
        const pitch = Math.asin(Math.max(-1, Math.min(1, dir.y)));
        let u = 0.5 - yaw / (2 * Math.PI);
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
            if (rel !== null && rel > 1e-3) {
                ratios.push(p.distance / rel);
            }
        }
        if (ratios.length < 3) {
            return { scale: this.lastScale, confidence: this.scaleConfidence * 0.8 };
        }
        ratios.sort((a, b) => a - b);
        const median = ratios[Math.floor(ratios.length / 2)];
        const mad = ratios.map(r => Math.abs(r - median)).sort((a, b) => a - b)[Math.floor(ratios.length / 2)];
        const inliers = ratios.filter(r => Math.abs(r - median) <= Math.max(0.05 * median, 3 * mad));
        const scale = inliers.length > 0
            ? inliers.reduce((a, b) => a + b, 0) / inliers.length
            : median;
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
        // ── 预测式深度流水线 (降低延迟) ──
        // 触发/维持一次后台 DA360 刷新(写入 _depthCache)。本帧只要缓存存在就
        // 直接复用(零等待), 把 DA360 的 ~1.6s 推理完全"藏"到后台。导航环因此
        // 以 YOPO 推理频率(而非 DA360+YOPO 串行)运行, 命令重规划 ≈5Hz 而非 ≈0.4Hz。
        //
        // 关键修正: 前台绝不每帧重复打 DA360。早期实现里前台在缓存过期(>150ms)
        // 时同步 await 一次真实深度, 而缓存 TTL(150ms) 远小于一个导航周期
        // (≥ depth+navigate), 导致每个周期都走"过期→同步等"分支, 与后台刷新
        // 同时打 DA360——双重请求挤占 GPU, 既没省到延迟(深度仍 1654ms), 还把
        // YOPO 推理拖到 830ms。现在: 只要有过缓存就用缓存; 仅在"从无到有"的首帧
        // 等一次真实深度, 且复用后台那个请求(不另开第二个)。
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

        // 首帧: 缓存尚未建立。等"正在进行的那一次"后台刷新完成即可(单请求,
        // 不另开第二个), 避免双请求抢 GPU。绝不悬停靠猜。
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

    // 触发后台 DA360 刷新(若当前无刷新在途), 并返回该 Promise 供首帧 await。
    // 同一时刻最多一个在途请求 → 绝不会对 DA360 发双重请求。
    _maybeRefreshDepth(cameraTransform, options) {
        if (this._refreshing) return this._refreshPromise;
        this._refreshing = true;
        this._refreshPromise = this._refreshDepth(cameraTransform, options).finally(() => {
            this._refreshing = false;
        });
        return this._refreshPromise;
    }

    // 打 DA360 取原始深度 → ERP 后处理 → 写入 this._depthCache。
    // 同时承担标定降频逻辑(原 captureYOPODepthERP 内的 Cesium raycast)。
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
        // 标定降频：Cesium raycast (9 次/帧) 是导航帧率的主要瓶颈，而 scale 随
        // 无人机移动变化缓慢，故每 N 帧才标定一次，其余帧复用缓存 scale。
        // 前两帧强制标定以建立可靠的初始 scale。
        let scale = this.lastScale;
        let confidence = this.scaleConfidence;
        this._erpFrameCount += 1;
        const doCalibrate = calibrate && (this._erpFrameCount <= 1 || this._erpFrameCount % this._calibrateInterval === 0);
        if (doCalibrate) {
            // 自适应 raycast 距离: 高空中固定 20m 的 Cesium raycast 射不到地面/
            // 远处物体, 命中的标定点不足, 会退回缓存旧 scale, 导致高空深度被
            // 错误缩放成"四周全近距"的假近障, YOPO 因而持续大幅掉头、无法朝
            // 目标飞行。根据无人机高度放大 raycast 距离, 使高空也能命中地面/
            // 远处建筑来正确恢复 metric scale。
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
        const depth = this._resizeBilinear(rawDepth, rawW, rawH, width, height);
        const mask = this._resizeNearestUint8(rawMask, rawW, rawH, width, height);

        // 方位角旋向对齐 (关键):
        // 全景着色器的 ERP 约定为 yaw = PI - u * 2PI, 即方位角随列号 *递减*
        // (第 0 列 = 正后方 +PI, 中心列 = 正前方 0, 末列 = 正后方 -PI)。
        // 而 YOPO 锚点 primitive.py 中 alpha = -d*(N-1)/2 + j*d 随序号 *递增*。
        // 两者旋向相反 => 网络第 j 个锚点会读到 alpha = -alpha_j 处的深度,
        // 前后不变但左右完全镜像, 导致朝障碍物方向飞行。
        // 水平翻转列序即可令 alpha 与锚点序号同向。
        // (basis 由无人机四元数构建, 全景已随机头旋转, 故无需再做 yaw 移位。)
        this._flipHorizontalInPlace(depth, width, height);
        this._flipHorizontalInPlace(mask, width, height);

        // Scale to metres and clamp. Invalid pixels become maxDistance; the
        // server's _preprocess_depth will additionally mean-fill them using
        // the mask we send as channel 1.
        // (flip 与 scale 合并为一趟遍历, 省去一次 73728 像素的单独遍历。)
        for (let i = 0; i < depth.length; i++) {
            let d = depth[i] * scale;
            if (!Number.isFinite(d) || d <= 0) d = maxDistance;
            depth[i] = Math.min(d, maxDistance);
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
     * 就地水平翻转 (列序反转), 适用于 Float32Array / Uint8Array。
     * 用于把 ERP 方位角旋向对齐到 YOPO 锚点的 alpha 递增方向。
     */
    _flipHorizontalInPlace(buf, width, height) {
        const half = width >> 1;
        for (let y = 0; y < height; y++) {
            const row = y * width;
            for (let x = 0; x < half; x++) {
                const a = row + x;
                const b = row + width - 1 - x;
                const t = buf[a];
                buf[a] = buf[b];
                buf[b] = t;
            }
        }
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
