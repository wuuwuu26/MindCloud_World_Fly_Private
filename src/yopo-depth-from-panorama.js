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
// DA360 相对深度上限: relative_to_nearest 中近处物体 rel 通常 <40, 超过视为
// 远景/天空伪值(单目模型 1/disp 对远景输出数百), 标定须过滤以防 scale 被压小。
const DA360_REL_MAX = 40;
// scale 物理合理范围: relative_to_nearest 场景最近点通常 0.5~30m (低空贴近 0.5m,
// 高空最近结构数十米), 超出即标定被污染, 回退历史值。可用 ?da360ScaleMin/Max 覆盖。
const DA360_SCALE_MIN = 0.5;
const DA360_SCALE_MAX = 30.0;
// scale 时间平滑系数(新值占比): 0.5 即与历史各半, 抑制 DA360 帧间相对深度漂移。
const DA360_SCALE_SMOOTH = 0.5;

export class YOPODepthFromPanorama {
    constructor(world, panoramaSensor) {
        this.world = world;
        this.panoramaSensor = panoramaSensor;
        this.lastScale = 1.0;
        this.lastRelativeDepth = null;
        this.lastScaleTimestamp = 0;
        this.scaleConfidence = 0;
        this._erpFrameCount = 0;       // ERP 帧计数
        // 标定策略: 每获取到一张 DA360 深度图就标定一次(见 _refreshDepth)。
        // raycast 开销由 pickLocalRay 的 150ms 方向分桶缓存吸收, 无需降频。
        // ── 预测式深度缓存 (降低延迟用) ──
        // 深度刷新(打 DA360)在后台异步进行, 导航环每帧直接复用最近一次
        // 已处理好的 ERP 深度, 把 DA360 的 140ms 等待"藏"到后台, 使深度环
        // 周期从 ~230ms(DA360+YOPO) 降到 ≈YOPO 推理时间, 控制更跟手。
        this._depthCache = null;       // {depth, mask, scale, confidence, time}
        this._refreshing = false;      // 是否有 DA360 刷新在途
        this._depthCacheTtlMs = 150;   // 缓存新鲜度阈值; 超过则本帧同步等真实深度
        // DA360 刷新最小间隔 (ms): 深度环高频触发时(如 navigate 走 33ms 节流缓存)若不
        // 限频会持续打 DA360, 慢推理叠加排队反而拖慢实时性。限频后缓存复用, 深度环以
        // navigate 频率运行, DA360 在后台以 <=minInterval 的节奏更新缓存。
        // GPU 后处理提速后刷新本身更便宜: 30→10→5→3ms, 真实深度获取频率进一步提高 (DA360 在途
        // 由 _refreshing 互斥保证, 不会并发打爆; 间隔越小缓存越新鲜, YOPO 越实时)。
        // 该值同时作为"运动指令(重规划)更新频率"的基准 —— main.js 会把 navigate 客户端节流
        // (_requestInterval) 直接绑定到此值, 保证两者频率严格一致且一并提高。
        this._minRefreshIntervalMs = 3;
        // ── GPU 后处理 (懒初始化) ──
        // rawDepth/resize/flip/mask 这些逐像素循环在 CPU 主线程跑, 384x192 每次几万次
        // 插值+翻转, 是深度环周期的固定开销。用 WebGL2 单次 draw 完成
        // "bilinear resize + 水平翻转 + mask + clamp" 全部步骤, CPU 只 readPixels 读回
        // 最终结果, 显著缩短 _refreshDepth 耗时 → 提高深度获取频率/实时性。
        // 创建失败(无 WebGL2)时自动回退 CPU 路径, 不影响功能。
        this._gpu = null;
    }

    /** 懒初始化 GPU 后处理管线。返回是否可用。 */
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

            // 顶点着色器: 全屏三角形
            const vsSrc = `#version 300 es
                in vec2 a_pos;
                void main() { gl_Position = vec4(a_pos, 0.0, 1.0); }`;
            // 片元着色器: 采样深度纹理, 完成 bilinear resize + mask + clamp。
            // u_tex 为 R32F 深度纹理。注意: RGB 全景已在 panorama-sensor 渲染时水平
            // 翻转为 YOPO 训练视角(左半=左方向), DA360 输出的深度布局跟随输入 RGB
            // (逐像素对齐), 因此这里【不再翻转】列序 —— 深度直接就是 YOPO 锚点视角。
            // 无效像素(NaN/<=0)写 0 便于下游 mask 判定。
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
                    // mask: 有效(v>1e-3 且有限)为 255, 否则 0
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
            // 单通道纹理单元
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
     * GPU 后处理: 把 rawDepth(Float32Array) resize 到 dstW×dstH 并做水平翻转,
     * 同时生成 mask(Uint8Array)。返回 {depth, mask}。失败时返回 null。
     */
    _gpuResizeFlip(rawDepth, rawW, rawH, dstW, dstH) {
        if (!this._ensureGpu()) return null;
        const gl = this._gpu.gl;
        const n = rawW * rawH;
        if (!this._gpu.floatTex || this._gpu.floatTex.n < n) {
            // (重)建 R32F 纹理: 只在尺寸增大时重建
            const old = this._gpu.floatTex;
            if (old) { gl.deleteTexture(old.tex); }
            const tex = gl.createTexture();
            gl.bindTexture(gl.TEXTURE_2D, tex);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
            // 预分配一个足够大的存储
            gl.texStorage2D(gl.TEXTURE_2D, 1, gl.R32F, rawW, rawH);
            this._gpu.floatTex = { tex, n };
            this._gpu.nRaw = n;
        }
        const ftex = this._gpu.floatTex.tex;
        gl.bindTexture(gl.TEXTURE_2D, ftex);
        gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, rawW, rawH, gl.RED, gl.FLOAT, rawDepth);

        // 输出缓冲: depth(R32F) + mask(R8) 两张 FBO 附件
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

        // 渲染到 FBO (MRT: depth + mask)
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

        // 读回: 复用缓冲(尺寸不变不重建), 减少每帧 GC 压力与分配开销。
        // 输出数组复用给调用方(调用方在下一次刷新前用完, 见 _refreshDepth 用法)。
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
            // forceFresh=true: 标定射线必须返回当前真实距离, 不走 pickLocalRay 的方向分桶
            // 缓存(缓存命中会返回 ≤150ms/≤0.5m 漂移的陈旧距离, 导致标定 scale 偏差)。
            // 代价是每次深度图 4 条射线都真实 GPU 拾取, 但保证米制标定始终新鲜。
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
        // 高空补充标定点: 前向 2x2 网格的射线在开阔/高空场景可能全部落空(命中天空
        // → pickLocalRay 返回 null), 导致 calibrationPoints 不足、scale 回退历史值
        // (高空下历史 scale 可能严重偏小 → 深度图被压缩成"四面近障")。补充:
        //   1. 正前方 add(0,0): 前方最近障碍, 避障最关心的方向
        //   2. 正下方 add(0,-1): 地面距离≈无人机高度, 高空时唯一稳定可命中点
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
        // YOPO_360 训练视角的 ERP 采样 (与 sensor_simulator.cu / primitive.py 一致):
        //   yaw = PI - (u+0.5)/W * 2PI  =>  yaw 随列号递增, u=0.5 中心=前方
        //   body NWU: x=fwd, y=left, z=up => 左半(u<0.5)=左方向, 右半(u>0.5)=右方向
        // RGB 全景已在 panorama-sensor 渲染时翻转为该视角, DA360 深度布局跟随,
        // 故此处直接按此约定采样(不再采用 Cesium shader 的镜像布局)。
        // MindCloud 系: forward=-z, right=+x, up=+y → MC 右(+x) 对应 YOPO 右方向。
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
            // 只信任"近/中距离"标定点: DA360 是单目深度模型, 输出 relative_to_nearest
            // (场景最近点=1.0)。对高空俯瞰的地面/远景/天空, 1/disp 会输出巨大伪值
            // (rel 可达数百), 用它算 scale = 真实距离/rel 会把 scale 压到极小(实测
            // 0.39) → 整幅深度图被缩成"四面近障", YOPO 误判处处是墙, 只敢选急转/
            // 俯冲的规避轨迹而无法朝目标前进。真实近处物体的 rel 通常 <40, 过滤掉
            // 巨大 rel 的不可靠远点, 只用可靠近距标定。
            if (rel !== null && rel > 1e-3 && rel < DA360_REL_MAX) {
                ratios.push(p.distance / rel);
            }
        }
        if (ratios.length < 2) {
            // 标定点不足(全被过滤): 回退历史 scale, 避免单点离群主导
            return { scale: this.lastScale, confidence: this.scaleConfidence * 0.7 };
        }
        ratios.sort((a, b) => a - b);
        const median = ratios[Math.floor(ratios.length / 2)];
        const mad = ratios.map(r => Math.abs(r - median)).sort((a, b) => a - b)[Math.floor(ratios.length / 2)];
        const inliers = ratios.filter(r => Math.abs(r - median) <= Math.max(0.05 * median, 3 * mad));
        let scale = inliers.length > 0
            ? inliers.reduce((a, b) => a + b, 0) / inliers.length
            : median;
        // 物理合理性钳制: relative_to_nearest 场景最近点通常 0.3~8m, 故 scale 应落在
        // [DA360_SCALE_MIN, DA360_SCALE_MAX]。超出范围说明标定被 DA360 远点伪值污染,
        // 回退历史 scale, 防止输出荒谬米制(如 0.39m 的中位数)误导网络。
        if (!Number.isFinite(scale) || scale < DA360_SCALE_MIN || scale > DA360_SCALE_MAX) {
            return { scale: this.lastScale, confidence: this.scaleConfidence * 0.6 };
        }
        // 时间平滑: 新 scale 与历史混合, 抵抗 DA360 帧间相对深度漂移导致的 scale 跳变
        // (跳变会让同一场景在"近障/开阔"间反复, 网络决策抖动、飞行不连贯)。
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
    // 限频: 距上次刷新启动 < _minRefreshIntervalMs 且已有缓存时跳过(复用缓存),
    // 让深度环以 navigate 频率运行、DA360 以 <=minInterval 节奏后台更新。
    // 首帧(无缓存)不跳过, 保证尽快建立深度缓存。
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
        // 每帧标定: 每获取到一张 DA360 深度图就做一次 Cesium raycast 标定, 保证
        // scale 始终跟随当前深度图(DA360 每帧的全局 shift 会变, 帧间复用旧 scale
        // 会有米制误差)。raycast 开销已被 pickLocalRay 的 150ms 方向分桶缓存吸收:
        // DA360 频率 ~22Hz(45ms) < 缓存 TTL(150ms), 4 条标定射线大多命中缓存,
        // 实际 GPU 拾取很少; 位移/转向后自动落新桶触发真实拾取, 无需额外降频。
        // 保留位移>1.5m 强制标定作为缓存穿透兜底(纯 CPU 侧判断, 无额外开销)。
        let scale = this.lastScale;
        let confidence = this.scaleConfidence;
        this._erpFrameCount += 1;
        // 每帧标定(只要开启了 calibrate)。
        const doCalibrate = calibrate;
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
        // ── GPU 后处理 (优先): WebGL2 单次 draw 完成 resize + 水平翻转 + mask,
        //    CPU 只读回。GPU 不可用(无 WebGL2/驱动)时回退 CPU 路径。 ──
        let depth, mask;
        const gpuRes = this._gpuResizeFlip(rawDepth, rawW, rawH, width, height);
        if (gpuRes) {
            // GPU 输出缓冲被复用于后续帧, 而缓存(含 depth/mask)会被后续 navigate 复用,
            // 必须拷贝独立副本, 否则下一帧刷新会覆盖缓存数据。拷贝与 scale 合并为一次遍历。
            depth = new Float32Array(gpuRes.depth);
            mask = new Uint8Array(gpuRes.mask);
        } else {
            depth = this._resizeBilinear(rawDepth, rawW, rawH, width, height);
            mask = this._resizeNearestUint8(rawMask, rawW, rawH, width, height);
            // 方位角旋向 (说明):
            // 早期版本这里做水平翻转, 因为当时 Cesium 渲染的 RGB ERP 是 MC 系
            // (左半=右方向), 与 YOPO 锚点(NWU 左半=左方向)镜像。现 RGB 全景已在
            // panorama-sensor 渲染时统一翻转为 YOPO 视角, DA360 输出的深度布局跟随
            // 输入 RGB(逐像素对齐), 因此这里不再翻转 —— 深度列序天然与 YOPO 锚点
            // alpha 递增同向。GPU 路径 shader 同样已去除翻转, 与此一致。
        }

        // Scale to metres. Invalid pixels (NaN/≤0) are kept as NaN — NOT clamped
        // to maxDistance — so the server's _preprocess_depth / nan_mask can identify
        // them as genuinely invalid and mean-fill them (matching YOPO_360 training,
        // where invalid pixels carry mask=0 and the network learns to ignore them).
        // If for some reason the mask channel is dropped, NaN still lets the server
        // distinguish "missing" from "far open space" (which used to be filled as
        // 20m → treated as flyable → crashes into missing/sky regions).
        // The validity mask (channel 1) is built separately and always sent.
        // (flip 与 scale 合并为一趟遍历, 省去一次 73728 像素的单独遍历。)
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
