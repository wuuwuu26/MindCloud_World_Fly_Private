import { reportUserError } from './error-report.js';

function urlNumber(name, fallback, min, max) {
    const value = new URLSearchParams(window.location.search).get(name);
    if (value == null || value === '') return fallback;
    const n = Number(value);
    if (!Number.isFinite(n)) return fallback;
    return Math.max(min, Math.min(max, n));
}

function evenNumber(value) {
    const n = Math.max(2, Math.round(value));
    return n % 2 === 0 ? n : n + 1;
}

const CAPTURE_INTERVAL_MS = urlNumber('panoMs', 12, 8, 10000);
// 深度请求最小间隔: 100→50→33ms, 深度图获取频率持续提高, 配合 DA360 后台刷新缓存提高实时性。
// DA360 已降至 384x192 推理(≈44ms/帧 → ~22Hz), 33ms 节流不再成为上限。
// 可 ?depthMs=50/100 还原。
const DEPTH_INTERVAL_MS = urlNumber('depthMs', 33, 33, 10000);
const DA360_TIMEOUT_MS = urlNumber('da360TimeoutMs', 12000, 1000, 60000);
// 上传到 DA360 的分辨率系数: 默认 0.5 即把全景缩到 1/2(从 768×384 → 384×192) 再发。
// DA360 的 DINOv2 深度模型推理耗时随像素数超线性增长(实测: 512×256≈71ms, 384×192≈44ms)。
// YOPO 网络只消费 384×192 的 ERP 深度(客户端恰好 resize 到 384×192), 因此 DA360 直接
// 出 384×192 与最终消费尺寸一致, 无需再降采样 → 深度帧刷新率提升到 ~22Hz(约 14Hz 的
// 1.5 倍)。8GB 显存(92% 占用)约束下, 384×192 是深度推理精度与实时性的最佳平衡点,
// 且避免 OOM(此前 576×288 曾触发)。要更高精度可 ?da360UploadScale=0.667(512×256,~14Hz),
// 要更实时可再降; OOM 时 _requestDepth 会自动降到当前的一半重试一次。
const DA360_UPLOAD_SCALE = urlNumber('da360UploadScale', 0.5, 0.05, 1);
const DA360_UPLOAD_WIDTH = Math.round(urlNumber('da360UploadWidth', 0, 0, 5760));
const DA360_UPLOAD_HEIGHT = Math.round(urlNumber('da360UploadHeight', 0, 0, 2880));
// 全景分辨率: 672x336 → 768x384, face 192→224。比参考(672)高 14% 提升深度精度, 又给
// DA360 上传缩放留余量(默认 0.75× 后 576×288), 显著降低 8GB GPU OOM 概率。
// 若显存富余可 ?panoWidth=896/1024 + ?da360UploadScale=1.0 提精度; 帧率紧可 ?panoWidth=672。
const PANORAMA_WIDTH = evenNumber(urlNumber('panoWidth', 768, 280, 5760));
const PANORAMA_HEIGHT = evenNumber(urlNumber('panoHeight', Math.round(PANORAMA_WIDTH / 2), 140, 2880));
const PANORAMA_FACE_SIZE = Math.round(urlNumber('panoFace', 224, 128, 2048));
// 导航中(fast) face 渲染分辨率 160px: GPU 像素量 224²→160² 减半, 每 face 同步渲染
// viewer.render 更快 → capturing 更快, 深度刷新更跟手。深度模型输入 384×192, 160px
// 足够。?panoFaceFast=224 还原。
const PANORAMA_FACE_SIZE_FAST = Math.round(urlNumber('panoFaceFast', 160, 128, 2048));
const PANORAMA_VERTICAL_FOV = urlNumber('panoVfov', 180, 30, 180);
const PANORAMA_JPEG_QUALITY = urlNumber('panoJpeg', 0.74, 0.35, 0.95);
const PANORAMA_FACE_FOV = urlNumber('panoFaceFov', 130, 90, 170);
const PANORAMA_TOP_POLE_GUARD = urlNumber('panoTopPoleGuard', 10, 0, 45);
const PANORAMA_BOTTOM_POLE_GUARD = urlNumber('panoBottomPoleGuard', 2, 0, 45);
const PANORAMA_FRAME_DELAY_MS = urlNumber('panoFrameDelayMs', 8, 0, 1000);
// quiet 180→40: 只缩短"瓦片就绪确认防抖"(pending=0 后 40ms 无新任务即上传), 不牺牲 LOD
// (LOD 由 timeout=900 决定)。配合 waitForTilesIdle 的 tick 自适应(≈13ms), 6 face 单帧
// 采集更快完成、主线程更早释放 -> 帧率与 RGB 刷新率双提升。?panoFaceTileQuietMs=180 还原。
// 飞行采集瓦片超时 900→600→200ms: capturing 耗时的最大来源是"每 face 等待全景
// tileset 瓦片 idle"。无人机移动时 LOD 持续更新, 瓦片队列几乎永不为空, 每个 face
// 都可能等满超时 → 6 face 单帧采集可到 3.6s。200ms 足够拉到当前视角主要瓦片
// (近处/可见 LOD 优先), 缺失的精细 LOD 由下次采集补上, RGB 刷新率大幅提升。
// 若遇 LOD 不一致虚影(远处细节模糊)可 ?panoFaceTileTimeoutMs=600/900 还原。
const PANORAMA_FACE_TILE_TIMEOUT_MS = urlNumber('panoFaceTileTimeoutMs', 200, 0, 10000);
// 导航中(fast)瓦片超时: 150ms —— 首 face 瓦片加载(冷启动/移动到新区域)通常需
// 200-500ms, 60ms 几乎必超时 → 触发 loadingTiles 快速失败 + 推迟重试, 反而"卡在
// 1/6"。150ms 是平衡点: 给主要瓦片足够到达时间(避免 1/6 卡住), 又不会等满 LOD
// 拖慢 capturing。?panoFaceTileTimeoutMsFast=60/300 可调。
const PANORAMA_FACE_TILE_TIMEOUT_MS_FAST = urlNumber('panoFaceTileTimeoutMsFast', 150, 0, 10000);
const PANORAMA_FACE_TILE_QUIET_MS = urlNumber('panoFaceTileQuietMs', 40, 0, 5000);
// 增量复用参数: 相机移动 <2.5m 且朝向余弦 >0.995(~5.7°) 且距上次成功采集 <800ms 时,
// 直接复用上帧全景(不重渲染 6 face), 避免 capturing 拖慢低速/悬停场景。?panoReuseMs 可调。
const PANORAMA_REUSE_MS = urlNumber('panoReuseMs', 800, 0, 10000);
const PANORAMA_REUSE_DIST_M = urlNumber('panoReuseDistM', 2.5, 0, 20);
const PANORAMA_REUSE_DOT_Q = urlNumber('panoReuseDotQ', 0.995, 0.9, 1);
const PANORAMA_PRELOAD_FRAME_DELAY_MS = urlNumber(
    'panoPreloadFrameDelayMs',
    Math.max(96, PANORAMA_FRAME_DELAY_MS),
    0,
    1000
);
const PANORAMA_PRELOAD_FACE_TILE_TIMEOUT_MS = urlNumber('panoPreloadFaceTileTimeoutMs', 6000, 500, 30000);
const PANORAMA_PRELOAD_FACE_TILE_QUIET_MS = urlNumber('panoPreloadFaceTileQuietMs', 650, 0, 5000);
const PANORAMA_PRELOAD_TIMEOUT_MS = urlNumber('panoPreloadTimeoutMs', 60000, 500, 120000);

function getDA360Endpoint() {
    const params = new URLSearchParams(window.location.search);
    const explicit = params.get('da360Url');
    if (explicit) return explicit;

    const host = params.get('da360Host') || window.location.hostname || '127.0.0.1';
    const port = params.get('da360Port') || '5688';
    return `http://${host}:${port}/depth`;
}

function shortError(error) {
    const message = error && error.message ? error.message : String(error || 'error');
    return message.length > 52 ? `${message.slice(0, 49)}...` : message;
}

function isDrawableImageSource(value) {
    if (!value || !Number.isFinite(value.width) || !Number.isFinite(value.height)) return false;
    if (value.width <= 0 || value.height <= 0) return false;
    if (typeof HTMLCanvasElement !== 'undefined' && value instanceof HTMLCanvasElement) return true;
    if (typeof OffscreenCanvas !== 'undefined' && value instanceof OffscreenCanvas) return true;
    if (typeof ImageBitmap !== 'undefined' && value instanceof ImageBitmap) return true;
    if (typeof HTMLImageElement !== 'undefined' && value instanceof HTMLImageElement) return true;
    if (typeof HTMLVideoElement !== 'undefined' && value instanceof HTMLVideoElement) return true;
    return false;
}

function captureProgressStatus(result, hasRgb) {
    const faceIndex = result && Number.isFinite(result.faceIndex) ? result.faceIndex : 0;
    const faceCount = result && Number.isFinite(result.faces) ? result.faces : 6;
    if (result && result.loadingTiles) return `tiles ${faceIndex + 1}/${faceCount}`;
    if (hasRgb) return 'ready';
    return `scanning ${faceIndex}/${faceCount}`;
}

export class PanoramaSensor {
    constructor() {
        this.panel = document.getElementById('panorama-sensor-panel');
        this.rgbCanvas = document.getElementById('panorama-rgb-canvas');
        this.depthImg = document.getElementById('panorama-depth-image');
        this.rgbStatusEl = document.getElementById('panorama-rgb-status');
        this.depthStatusEl = document.getElementById('panorama-depth-status');
        this.depthNearLabelEl = document.getElementById('panorama-depth-near-label');
        this.depthFarLabelEl = document.getElementById('panorama-depth-far-label');
        this.depthUnitEl = document.getElementById('panorama-depth-unit');
        this.endpoint = getDA360Endpoint();
        this.active = false;
        this.capturing = false;
        this.depthPending = false;
        this.depthSuppress = false;  // yopo_nav 时抑制 UI 深度请求, 避免与导航环竞争 DA360
        this.captureIntervalOverride = 0;  // yopo_nav 时降低全景捕获频率 (ms), 0=用默认值
        this.lastCaptureStartTime = 0;
        this.lastCaptureTime = 0;
        this.lastDepthTime = 0;
        this.hasRgb = false;
        this.hasDepth = false;

        if (this.rgbCanvas) {
            this.rgbCanvas.width = PANORAMA_WIDTH;
            this.rgbCanvas.height = PANORAMA_HEIGHT;
            this._drawPlaceholder(this.rgbCanvas, 'RGB PANORAMA');
        }
        this._setDepthPlaceholder('DA360 offline');
        this._setStatus('idle', 'offline');
    }

    setActive(active) {
        this.active = !!active;
        this._applyVisibility();
    }

    reset() {
        this.capturing = false;
        this.depthPending = false;
        this.lastCaptureStartTime = 0;
        this.lastCaptureTime = 0;
        this.lastDepthTime = 0;
        this.hasRgb = false;
        this.hasDepth = false;
        if (this.rgbCanvas) this._drawPlaceholder(this.rgbCanvas, 'RGB PANORAMA');
        this._setDepthPlaceholder('DA360 offline');
        this._setStatus('idle', 'offline');
        this._setDepthLegend(null);
    }

    hasRgbFrame() {
        return this.hasRgb;
    }

    getCaptureOptions(options = {}) {
        const preload = !!options.preload;
        // 导航中(fast=true): 全景主要用于 DA360 深度, RGB 精细 LOD 非必需 ——
        //   1. 瓦片超时 60ms/face(见 PANORAMA_FACE_TILE_TIMEOUT_MS_FAST)
        //   2. face 渲染分辨率 224→160px: Cesium 每 face 的 GPU 渲染像素从 224²(5万)
        //      降到 160²(2.5万), 像素量减半 → 同步渲染(viewer.render)更快;
        //      深度模型输入 384×192, face 160px 足够, 视觉精度影响可忽略
        //   3. frameDelay 8→0ms: 跳过二次同步渲染确认(少一次完整 viewer.render)
        // 非导航 UI 全景仍用 224px + 8ms, 保清晰度。
        const fast = !!options.fast;
        return {
            width: PANORAMA_WIDTH,
            height: PANORAMA_HEIGHT,
            faceSize: fast ? PANORAMA_FACE_SIZE_FAST : PANORAMA_FACE_SIZE,
            verticalFovDeg: PANORAMA_VERTICAL_FOV,
            faceFovDeg: PANORAMA_FACE_FOV,
            topPoleGuardDeg: PANORAMA_TOP_POLE_GUARD,
            bottomPoleGuardDeg: PANORAMA_BOTTOM_POLE_GUARD,
            frameDelayMs: preload ? PANORAMA_PRELOAD_FRAME_DELAY_MS
                : (fast ? 0 : PANORAMA_FRAME_DELAY_MS),
            tileTimeoutMs: preload ? PANORAMA_PRELOAD_FACE_TILE_TIMEOUT_MS
                : (fast ? PANORAMA_FACE_TILE_TIMEOUT_MS_FAST : PANORAMA_FACE_TILE_TIMEOUT_MS),
            tileQuietMs: preload ? PANORAMA_PRELOAD_FACE_TILE_QUIET_MS : PANORAMA_FACE_TILE_QUIET_MS,
            timeoutMs: preload ? PANORAMA_PRELOAD_TIMEOUT_MS : 0,
        };
    }

    primeFromCaptureResult(result, captureMs = 0) {
        if (!this.rgbCanvas) return false;
        const structuredResult = result && typeof result === 'object' && 'complete' in result;
        const panoCanvas = structuredResult ? result.canvas : result;
        const complete = structuredResult ? result.complete !== false : true;
        if (!complete || !isDrawableImageSource(panoCanvas)) return false;

        const ctx = this.rgbCanvas.getContext('2d');
        ctx.clearRect(0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
        // 水平翻转: 把 Cesium 渲染的 ERP(左半=右方向, MC 系) 转成 YOPO 训练视角
        // (左半=左方向, NWU 系)。深度链路已同步移除翻转(见 yopo-depth-from-panorama.js),
        // RGB 全景图从此与 YOPO 网络所见方向一致, 避免"左右镜像/畸变"感。
        this._drawImageFlipped(ctx, panoCanvas);
        const now = performance.now();
        this.lastCaptureStartTime = now;
        this.lastCaptureTime = now;
        this.hasRgb = true;
        this._setStatus(`preloaded ${Math.round(captureMs)}ms`, this.hasDepth ? 'ready' : 'offline');
        return true;
    }

    update(world, transform, now = performance.now()) {
        if (!this.panel || !this.rgbCanvas || !world || !transform) return;
        this._applyVisibility();
        if (!this._shouldRun()) return;
        const capInterval = this.captureIntervalOverride > 0 ? this.captureIntervalOverride : CAPTURE_INTERVAL_MS;
        if (this.capturing || now - this.lastCaptureStartTime < capInterval) return;
        // 增量复用: 相机位置/朝向变化小于阈值时, 直接复用上帧全景(不重新渲染 6 face)。
        // capturing 最贵的是 6 个 face 的瓦片等待(即使降到 200ms/face 仍 ~1.2s), 而
        // 无人机低速/悬停时视野几乎不变, 每帧重采是浪费。阈值下复用保证 RGB 刷新率
        // 不随"缓慢移动"被 capturing 拖死, 快速移动(>2.5m)时仍正常重采保新鲜。
        // 注意: 导航中(captureIntervalOverride>0)禁用复用 —— 导航环依赖每次 _capture
        // 成功后的 _requestDepth 更新深度, 复用会跳过该路径导致深度停更。
        if (this.captureIntervalOverride <= 0 && this.hasRgb && this._lastPanoTransform &&
            now - this.lastCaptureTime < PANORAMA_REUSE_MS) {
            const p = transform.position || {};
            const lp = this._lastPanoTransform.position || {};
            const moved = Math.hypot(p.x - lp.x, p.y - lp.y, p.z - lp.z);
            if (moved < PANORAMA_REUSE_DIST_M) {
                const q = transform.orientation || {};
                const lq = this._lastPanoTransform.orientation || {};
                const dotQ = (q.x || 0) * (lq.x || 0) + (q.y || 0) * (lq.y || 0) +
                    (q.z || 0) * (lq.z || 0) + (q.w || 0) * (lq.w || 0);
                if (dotQ > PANORAMA_REUSE_DOT_Q) {
                    // 复用上帧: 仅更新"最后活动"时间, 保持 UI 状态"新鲜"
                    this.lastCaptureStartTime = now;
                    return;
                }
            }
        }
        this._lastPanoTransform = JSON.parse(JSON.stringify(transform));
        this._capture(world, transform);
    }

    _enabledByUi() {
        const toggle = document.getElementById('panorama-toggle');
        return toggle ? toggle.checked : true;
    }

    _shouldRun() {
        const cleanMode = document.getElementById('clean-mode-toggle')?.checked ? true : false;
        return this.active && this._enabledByUi() && !cleanMode;
    }

    _applyVisibility() {
        if (!this.panel) return;
        this.panel.classList.toggle('visible', this._shouldRun());
    }

    _drawPlaceholder(canvas, label) {
        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        const gradient = ctx.createLinearGradient(0, 0, canvas.width, canvas.height);
        gradient.addColorStop(0, '#030712');
        gradient.addColorStop(0.55, '#111827');
        gradient.addColorStop(1, '#020617');
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        ctx.strokeStyle = 'rgba(125, 211, 252, 0.28)';
        ctx.lineWidth = 2;
        for (let x = 0; x <= canvas.width; x += 64) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, canvas.height);
            ctx.stroke();
        }
        for (let y = 0; y <= canvas.height; y += 64) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(canvas.width, y);
            ctx.stroke();
        }
        ctx.fillStyle = 'rgba(226, 232, 240, 0.78)';
        ctx.font = '24px Courier New, monospace';
        ctx.textAlign = 'center';
        ctx.fillText(label, canvas.width * 0.5, canvas.height * 0.52);
    }

    _setDepthPlaceholder(label) {
        if (!this.depthImg) return;
        const canvas = document.createElement('canvas');
        canvas.width = PANORAMA_WIDTH;
        canvas.height = PANORAMA_HEIGHT;
        this._drawPlaceholder(canvas, label);
        try {
            this.depthImg.src = canvas.toDataURL('image/png');
        } catch (error) {
            reportUserError('Depth placeholder render failed', error, {
                key: 'depth-placeholder',
                intervalMs: 10000,
            });
        }
    }

    _setStatus(rgbStatus, depthStatus) {
        if (this.rgbStatusEl) this.rgbStatusEl.textContent = rgbStatus;
        if (this.depthStatusEl) this.depthStatusEl.textContent = depthStatus;
    }

    _formatRelativeDepth(value) {
        const n = Number(value);
        if (!Number.isFinite(n) || n <= 0) return '--';
        if (n < 10) return `${n.toFixed(1)}x`;
        if (n < 100) return `${Math.round(n)}x`;
        return `${n.toExponential(1)}x`;
    }

    _setDepthLegend(scale) {
        const valid = scale && scale.valid;
        if (this.depthUnitEl) {
            this.depthUnitEl.textContent = valid ? 'x nearest' : 'relative';
        }
        if (this.depthNearLabelEl) {
            const near = valid ? this._formatRelativeDepth(scale.near) : '1x';
            this.depthNearLabelEl.textContent = `near ${near}`;
        }
        if (this.depthFarLabelEl) {
            const far = valid ? this._formatRelativeDepth(scale.far) : '--';
            this.depthFarLabelEl.textContent = `far ${far}`;
        }
    }

    async _capture(world, transform) {
        this.capturing = true;
        this.lastCaptureStartTime = performance.now();
        this._setStatus('capturing', this.depthPending ? 'inferring' : (this.hasRgb ? 'ready' : 'offline'));

        try {
            const capture = typeof world.capturePanoramaIncrementalAsync === 'function'
                ? world.capturePanoramaIncrementalAsync.bind(world)
                : typeof world.capturePanoramaAsync === 'function'
                ? world.capturePanoramaAsync.bind(world)
                : world.capturePanorama.bind(world);
            const result = await capture(transform, this.getCaptureOptions({ fast: this.captureIntervalOverride > 0 }));
            const structuredResult = result && typeof result === 'object' && 'complete' in result;
            const panoCanvas = structuredResult ? result.canvas : result;
            const complete = structuredResult ? result.complete !== false : true;
            if (!isDrawableImageSource(panoCanvas)) {
                if (!complete || structuredResult) {
                    const rgbStatus = captureProgressStatus(result, this.hasRgb);
                    this._setStatus(rgbStatus, this.depthPending ? 'inferring' : (this.hasDepth ? 'ready' : 'offline'));
                    // 瓦片未就绪(loadingTiles): 若按 capInterval(导航时 50ms) 立即重试,
                    // 会每 50ms 发起一次长瓦片等待 -> capturing 永续、RGB 永不显示。
                    // 推迟下次重试到"本次瓦片超时 + 缓冲"之后, 打破 busy loop。
                    // 关键: 不能写死 900ms —— fast 模式瓦片超时仅 150ms, 若仍等 900ms,
                    // 会人为拉长"卡在 1/6"的等待。用 result.tileTimeoutMs 联动(若服务端
                    // 返回了)或按当前模式取超时值。
                    if (result && result.loadingTiles) {
                        const tw = (result && result.tileTimeoutMs) || 150;
                        this.lastCaptureStartTime = performance.now() + Math.max(50, tw + 50);
                    }
                    return;
                }
                throw new Error('panorama capture returned non-drawable frame');
            }
            if (!complete) {
                const rgbStatus = captureProgressStatus(result, this.hasRgb);
                this._setStatus(rgbStatus, this.depthPending ? 'inferring' : (this.hasDepth ? 'ready' : 'offline'));
                return;
            }

            const ctx = this.rgbCanvas.getContext('2d');
            ctx.clearRect(0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
            // 同 primeFromCaptureResult: 水平翻转 RGB 全景到 YOPO 训练视角(左半=左)。
            this._drawImageFlipped(ctx, panoCanvas);
            this.lastCaptureTime = performance.now();
            const captureMs = this.lastCaptureTime - this.lastCaptureStartTime;
            this.hasRgb = true;
            const rgbStatus = `${Math.round(captureMs)}ms`;
            this._setStatus(rgbStatus, this.depthPending ? 'inferring' : (this.hasDepth ? 'ready' : 'offline'));

            if (!this.depthPending && !this.depthSuppress && this.lastCaptureTime - this.lastDepthTime >= DEPTH_INTERVAL_MS) {
                this._requestDepth(this.rgbCanvas);
            }
        } catch (error) {
            reportUserError('Panorama capture failed', error, {
                key: 'panorama-capture',
                intervalMs: 3000,
            });
            this._setStatus(shortError(error), this.depthPending ? 'inferring' : 'offline');
        } finally {
            this.capturing = false;
        }
    }

    /**
     * 水平翻转绘制: 把 Cesium 渲染的 ERP(左半=右方向) 转成 YOPO 训练视角(左半=左方向)。
     * 与深度后处理的翻转移除配套 —— RGB 全景与 YOPO 深度从此同视角, 消除左右镜像错位。
     */
    _drawImageFlipped(ctx, source) {
        const w = this.rgbCanvas.width;
        const h = this.rgbCanvas.height;
        ctx.save();
        ctx.translate(w, 0);
        ctx.scale(-1, 1);
        ctx.drawImage(source, 0, 0, w, h);
        ctx.restore();
    }

    _depthUploadCanvas(canvas, factor) {
        if (!canvas || !canvas.width || !canvas.height) return canvas;
        const scaleFactor = factor || 1.0;
        const explicitWidth = DA360_UPLOAD_WIDTH > 0 ? DA360_UPLOAD_WIDTH : 0;
        const explicitHeight = DA360_UPLOAD_HEIGHT > 0 ? DA360_UPLOAD_HEIGHT : 0;
        let width = explicitWidth;
        let height = explicitHeight;

        if (width && !height) {
            height = Math.max(2, Math.round(width * canvas.height / canvas.width));
        } else if (!width && height) {
            width = Math.max(2, Math.round(height * canvas.width / canvas.height));
        } else if (!width && !height) {
            // 应用 OOM 重试 factor: factor=0.5 时再减半
            const effScale = DA360_UPLOAD_SCALE * scaleFactor;
            width = Math.max(2, Math.round(canvas.width * effScale));
            height = Math.max(2, Math.round(canvas.height * effScale));
        }

        width = Math.min(canvas.width, Math.max(2, width));
        height = Math.min(canvas.height, Math.max(2, height));
        if (width === canvas.width && height === canvas.height) return canvas;

        if (!this.depthUploadCanvas) this.depthUploadCanvas = document.createElement('canvas');
        this.depthUploadCanvas.width = width;
        this.depthUploadCanvas.height = height;
        const ctx = this.depthUploadCanvas.getContext('2d', { alpha: false });
        ctx.imageSmoothingEnabled = true;
        ctx.imageSmoothingQuality = 'medium';
        ctx.clearRect(0, 0, width, height);
        ctx.drawImage(canvas, 0, 0, width, height);
        return this.depthUploadCanvas;
    }

    _canvasToJpegBlob(canvas) {
        return new Promise(resolve => {
            if (!canvas || typeof canvas.toBlob !== 'function') {
                resolve(null);
                return;
            }
            canvas.toBlob(resolve, 'image/jpeg', PANORAMA_JPEG_QUALITY);
        });
    }

    async _requestDepth(canvas) {
        this.depthPending = true;
        this._setStatus('ready', 'inferring');
        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), DA360_TIMEOUT_MS);
        const started = performance.now();

        try {
            // 默认尺寸 + OOM 自动降级重试一次: 服务端 CUDA OOM 时返回 500 含
            // "CUDA out of memory" 字符串, 此时把上传 canvas 缩到当前一半重试,
            // 不影响功能(深度图仍可用, 仅精度略降)。
            const sendOnce = async (factor) => {
                const uploadCanvas = this._depthUploadCanvas(canvas, factor);
                const blob = await this._canvasToJpegBlob(uploadCanvas);
                const headers = {};
                let body;
                if (blob) {
                    headers['Content-Type'] = blob.type || 'image/jpeg';
                    body = blob;
                } else {
                    headers['Content-Type'] = 'application/json';
                    body = JSON.stringify({ image: uploadCanvas.toDataURL('image/jpeg', PANORAMA_JPEG_QUALITY) });
                }
                return fetch(this.endpoint, {
                    method: 'POST',
                    headers,
                    body,
                    signal: controller.signal,
                });
            };

            let response = await sendOnce(1.0);
            if (!response.ok) {
                // 读取错误体判定是否 OOM
                let detail = '';
                let isOom = false;
                try {
                    const text = await response.text();
                    if (text) {
                        detail = ` body=${text.slice(0, 240)}`;
                        if (/out\s*of\s*memory|oom/i.test(text)) isOom = true;
                    }
                } catch (e) {
                    detail = ` body-read-failed: ${(e && e.message) || e}`;
                }
                if (isOom) {
                    // 自动降级重试: 缩到一半, 释放服务端临时 buffer
                    response = await sendOnce(0.5);
                    if (!response.ok) {
                        throw new Error(`DA360 HTTP ${response.status} after OOM retry`);
                    }
                } else {
                    throw new Error(`DA360 HTTP ${response.status}${detail}`);
                }
            }
            const payload = await response.json();
            if (!payload || !payload.depth_image) {
                throw new Error('DA360 response missing depth_image');
            }
            this.depthImg.src = payload.depth_image;
            this._setDepthLegend(payload.depth_scale);
            this.hasDepth = true;
            this.lastDepthTime = performance.now();
            const latency = Number.isFinite(payload.latency_ms)
                ? `${Math.round(payload.latency_ms)}ms`
                : `${Math.round(this.lastDepthTime - started)}ms`;
            this._setStatus('ready', latency);
        } catch (error) {
            reportUserError('DA360 depth request failed', error, {
                key: 'da360-depth-request',
                intervalMs: 3000,
            });
            this.lastDepthTime = performance.now();
            this._setStatus('ready', shortError(error));
        } finally {
            window.clearTimeout(timeout);
            this.depthPending = false;
        }
    }
}
