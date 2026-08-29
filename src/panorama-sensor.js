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
// Minimum interval between depth requests: 100 -> 50 -> 33 ms, continuously raising the
// depth-frame refresh rate; combined with DA360's background cache refresh this improves
// real-time behaviour. DA360 already infers at 384x192 (~44 ms/frame -> ~22 Hz), so the
// 33 ms throttle is no longer the limiting factor. Restore with ?depthMs=50/100.
const DEPTH_INTERVAL_MS = urlNumber('depthMs', 33, 33, 10000);
const DA360_TIMEOUT_MS = urlNumber('da360TimeoutMs', 12000, 1000, 60000);
// Resolution factor for the upload to DA360: the default 0.5 halves the panorama
// (768x384 -> 384x192) before sending it.
// DA360's DINOv2 depth model cost grows super-linearly with pixel count (measured:
// 512x256 ~= 71 ms, 384x192 ~= 44 ms).
// The YOPO network only consumes 384x192 ERP depth (the client resizes to exactly
// 384x192), so having DA360 emit 384x192 directly matches the final consumed size and no
// further downsampling is needed -> the depth frame rate rises to ~22 Hz (about 1.5x the
// previous ~14 Hz). Under an 8 GB VRAM budget (92% used), 384x192 is the best trade-off
// between depth accuracy and real-time behaviour, and it avoids OOM (576x288 triggered it
// before). For more accuracy use ?da360UploadScale=0.667 (512x256, ~14 Hz); lower it
// further for more real-time behaviour; on OOM _requestDepth automatically halves the
// current size and retries once.
const DA360_UPLOAD_SCALE = urlNumber('da360UploadScale', 0.5, 0.05, 1);
const DA360_UPLOAD_WIDTH = Math.round(urlNumber('da360UploadWidth', 0, 0, 5760));
const DA360_UPLOAD_HEIGHT = Math.round(urlNumber('da360UploadHeight', 0, 0, 2880));
// Panorama resolution: 672x336 -> 768x384, face 192 -> 224. That is 14% above the
// reference (672), which improves depth accuracy while leaving headroom for the DA360
// upload scaling (default 0.75x -> 576x288), significantly lowering OOM probability on an
// 8 GB GPU. With spare VRAM use ?panoWidth=896/1024 plus ?da360UploadScale=1.0 for more
// accuracy; if the frame rate is tight use ?panoWidth=672.
const PANORAMA_WIDTH = evenNumber(urlNumber('panoWidth', 768, 280, 5760));
const PANORAMA_HEIGHT = evenNumber(urlNumber('panoHeight', Math.round(PANORAMA_WIDTH / 2), 140, 2880));
const PANORAMA_FACE_SIZE = Math.round(urlNumber('panoFace', 224, 128, 2048));
// Face render resolution while navigating (fast): 160 px halves the GPU pixel count
// (224^2 -> 160^2), so the synchronous viewer.render per face is faster -> capturing is
// faster and the depth refresh feels more responsive. The depth model input is 384x192,
// so 160 px is enough. Restore with ?panoFaceFast=224.
const PANORAMA_FACE_SIZE_FAST = Math.round(urlNumber('panoFaceFast', 160, 128, 2048));
const PANORAMA_VERTICAL_FOV = urlNumber('panoVfov', 180, 30, 180);
const PANORAMA_JPEG_QUALITY = urlNumber('panoJpeg', 0.74, 0.35, 0.95);
const PANORAMA_FACE_FOV = urlNumber('panoFaceFov', 130, 90, 170);
const PANORAMA_TOP_POLE_GUARD = urlNumber('panoTopPoleGuard', 10, 0, 45);
const PANORAMA_BOTTOM_POLE_GUARD = urlNumber('panoBottomPoleGuard', 2, 0, 45);
const PANORAMA_FRAME_DELAY_MS = urlNumber('panoFrameDelayMs', 8, 0, 1000);
// quiet 180 -> 40: this only shortens the "tiles ready" confirmation debounce (upload once
// nothing new appears for 40 ms after pending=0) without sacrificing LOD (LOD is decided
// by timeout=900). Together with waitForTilesIdle's adaptive tick (~13 ms), a 6-face
// single-frame capture finishes sooner and frees the main thread earlier -> both frame
// rate and RGB refresh rate improve. Restore with ?panoFaceTileQuietMs=180.
// In-flight tile timeout 900 -> 600 -> 200 ms: the biggest cost of capturing is "waiting
// for the panorama tileset to go idle" on every face. While the drone moves, LOD keeps
// updating and the tile queue is almost never empty, so every face can burn its full
// timeout -> a 6-face single-frame capture can take 3.6 s. 200 ms is enough to pull in the
// main tiles of the current view (near/visible LOD first); the missing finer LOD is picked
// up by the next capture, which greatly improves the RGB refresh rate.
// If LOD inconsistency ghosts appear (blurry distant detail), restore with
// ?panoFaceTileTimeoutMs=600/900.
const PANORAMA_FACE_TILE_TIMEOUT_MS = urlNumber('panoFaceTileTimeoutMs', 200, 0, 10000);
// Tile timeout while navigating (fast): 150 ms -- loading the first face's tiles (cold
// start / moving into a new area) usually takes 200-500 ms, so 60 ms almost always timed
// out -> it triggered the fast loadingTiles failure plus a deferred retry, which actually
// got it "stuck at 1/6". 150 ms is the balance: enough time for the main tiles to arrive
// (avoiding the 1/6 stall) without waiting for the full LOD and slowing capturing down.
// Tunable via ?panoFaceTileTimeoutMsFast=60/300.
const PANORAMA_FACE_TILE_TIMEOUT_MS_FAST = urlNumber('panoFaceTileTimeoutMsFast', 150, 0, 10000);
const PANORAMA_FACE_TILE_QUIET_MS = urlNumber('panoFaceTileQuietMs', 40, 0, 5000);
// Incremental reuse parameters: when the camera moves < 2.5 m, the orientation cosine is
// > 0.995 (~5.7 deg) and less than 800 ms passed since the last successful capture, reuse
// the previous panorama frame directly (without re-rendering 6 faces) so capturing does not
// slow down low-speed / hovering scenarios. Tunable via ?panoReuseMs.
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
        this.depthSuppress = false;  // yopo_nav suppresses UI depth requests so they do not compete with the nav loop for DA360
        this.captureIntervalOverride = 0;  // yopo_nav lowers the panorama capture rate (ms), 0 = use the default
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
        // While navigating (fast=true): the panorama mainly feeds DA360 depth, so fine RGB
        // LOD is not required --
        //   1. tile timeout 60 ms/face (see PANORAMA_FACE_TILE_TIMEOUT_MS_FAST)
        //   2. face render resolution 224 -> 160 px: Cesium's GPU pixels per face drop from
        //      224^2 (50k) to 160^2 (25k), halving the pixel count -> the synchronous
        //      render (viewer.render) is faster; the depth model input is 384x192, so a
        //      160 px face is enough and the visual accuracy impact is negligible
        //   3. frameDelay 8 -> 0 ms: skip the second synchronous render confirmation
        //      (one less full viewer.render)
        // The non-navigating UI panorama still uses 224 px + 8 ms to stay sharp.
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
        // Horizontal flip: turn the Cesium-rendered ERP (left half = right direction, MC
        // frame) into the YOPO training view (left half = left direction, NWU frame). The
        // flip was removed from the depth pipeline at the same time (see
        // yopo-depth-from-panorama.js), so the RGB panorama now shares the orientation the
        // YOPO network sees, avoiding a "mirrored / distorted" feel.
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
        // Incremental reuse: when the camera position/orientation changes by less than the
        // thresholds, reuse the previous panorama frame directly (without re-rendering the
        // 6 faces).
        // The most expensive part of capturing is waiting for tiles on 6 faces (still
        // ~1.2 s even at 200 ms/face), while the view barely changes when the drone hovers
        // or moves slowly, so re-capturing every frame is waste. Reusing below the
        // thresholds keeps the RGB refresh rate from being dragged down by capturing during
        // "slow movement", while fast movement (> 2.5 m) still re-captures normally to stay
        // fresh.
        // Note: reuse is disabled while navigating (captureIntervalOverride > 0) -- the nav
        // loop relies on _requestDepth updating depth after each successful _capture, and
        // reuse would skip that path and stall the depth updates.
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
                    // Reuse the previous frame: only bump the "last activity" time so the
                    // UI state stays fresh
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
                    // Tiles not ready (loadingTiles): retrying immediately at capInterval
                    // (50 ms while navigating) would start a long tile wait every 50 ms ->
                    // capturing never ends and RGB never shows.
                    // Defer the next retry past "this tile timeout + buffer" to break the
                    // busy loop.
                    // Key point: do not hard-code 900 ms -- the fast-mode tile timeout is
                    // only 150 ms, and still waiting 900 ms would artificially stretch the
                    // "stuck at 1/6" wait. Link it to result.tileTimeoutMs (when the server
                    // returns one) or take the timeout value for the current mode.
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
            // Same as primeFromCaptureResult: horizontally flip the RGB panorama into the
            // YOPO training view (left half = left).
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
     * Draw horizontally flipped: turn the Cesium-rendered ERP (left half = right
     * direction) into the YOPO training view (left half = left direction).
     * Paired with removing the flip in depth post-processing -- the RGB panorama and the
     * YOPO depth now share the same view, eliminating the left/right mirror mismatch.
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
            // Apply the OOM retry factor: halve again when factor = 0.5
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
            // Default size plus one automatic downgrade retry on OOM: when the server hits
            // CUDA OOM it returns 500 with the string "CUDA out of memory", at which point
            // the upload canvas is shrunk to half its current size and the request is
            // retried. Functionality is unaffected (the depth map is still usable, only
            // slightly less accurate).
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
                // Read the error body to decide whether this was an OOM
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
                    // Automatic downgrade retry: shrink to half and free the server-side
                    // temporary buffer
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
