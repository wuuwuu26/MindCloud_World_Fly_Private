function urlNumber(name, fallback, min, max) {
    try {
        const value = new URLSearchParams(window.location.search).get(name);
        const n = Number(value);
        if (!Number.isFinite(n)) return fallback;
        return Math.max(min, Math.min(max, n));
    } catch (_) {
        return fallback;
    }
}

const CAPTURE_INTERVAL_MS = urlNumber('panoMs', 1000, 250, 10000);
const DEPTH_INTERVAL_MS = urlNumber('depthMs', 1200, 250, 10000);
const DA360_TIMEOUT_MS = 30000;
const PANORAMA_WIDTH = Math.round(urlNumber('panoWidth', 512, 256, 1536));
const PANORAMA_HEIGHT = Math.round(urlNumber('panoHeight', Math.round(PANORAMA_WIDTH / 2), 128, 768));
const PANORAMA_FACE_SIZE = Math.round(urlNumber('panoFace', 128, 96, 512));

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

export class PanoramaSensor {
    constructor() {
        this.panel = document.getElementById('panorama-sensor-panel');
        this.rgbCanvas = document.getElementById('panorama-rgb-canvas');
        this.depthImg = document.getElementById('panorama-depth-image');
        this.rgbStatusEl = document.getElementById('panorama-rgb-status');
        this.depthStatusEl = document.getElementById('panorama-depth-status');
        this.endpoint = getDA360Endpoint();
        this.active = false;
        this.capturing = false;
        this.depthPending = false;
        this.lastCaptureTime = 0;
        this.lastDepthTime = 0;
        this.lastRgbDataUrl = null;
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
        this.lastCaptureTime = 0;
        this.lastDepthTime = 0;
        this.lastRgbDataUrl = null;
        this.hasDepth = false;
        if (this.rgbCanvas) this._drawPlaceholder(this.rgbCanvas, 'RGB PANORAMA');
        this._setDepthPlaceholder('DA360 offline');
        this._setStatus('idle', 'offline');
    }

    update(world, transform, now = performance.now()) {
        if (!this.panel || !this.rgbCanvas || !world || !transform) return;
        this._applyVisibility();
        if (!this._shouldRun()) return;
        if (this.capturing || now - this.lastCaptureTime < CAPTURE_INTERVAL_MS) return;
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
        } catch (_) {}
    }

    _setStatus(rgbStatus, depthStatus) {
        if (this.rgbStatusEl) this.rgbStatusEl.textContent = rgbStatus;
        if (this.depthStatusEl) this.depthStatusEl.textContent = depthStatus;
    }

    async _capture(world, transform) {
        this.capturing = true;
        this._setStatus('capturing', this.depthPending ? 'inferring' : (this.lastRgbDataUrl ? 'ready' : 'offline'));

        try {
            const capture = typeof world.capturePanoramaAsync === 'function'
                ? world.capturePanoramaAsync.bind(world)
                : world.capturePanorama.bind(world);
            const panoCanvas = await capture(transform, {
                width: PANORAMA_WIDTH,
                height: PANORAMA_HEIGHT,
                faceSize: PANORAMA_FACE_SIZE,
            });
            if (!panoCanvas) throw new Error('panorama capture returned empty frame');

            const ctx = this.rgbCanvas.getContext('2d');
            ctx.clearRect(0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
            ctx.drawImage(panoCanvas, 0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
            this.lastCaptureTime = performance.now();
            this.lastRgbDataUrl = this.rgbCanvas.toDataURL('image/jpeg', 0.82);
            this._setStatus('ready', this.depthPending ? 'inferring' : (this.hasDepth ? 'ready' : 'offline'));

            if (!this.depthPending && this.lastCaptureTime - this.lastDepthTime >= DEPTH_INTERVAL_MS) {
                this._requestDepth(this.lastRgbDataUrl);
            }
        } catch (error) {
            console.warn('[PanoramaSensor] capture failed:', error);
            this._setStatus(shortError(error), this.depthPending ? 'inferring' : 'offline');
        } finally {
            this.capturing = false;
        }
    }

    async _requestDepth(imageDataUrl) {
        this.depthPending = true;
        this._setStatus('ready', 'inferring');
        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), DA360_TIMEOUT_MS);
        const started = performance.now();

        try {
            const response = await fetch(this.endpoint, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ image: imageDataUrl }),
                signal: controller.signal,
            });
            if (!response.ok) {
                throw new Error(`DA360 HTTP ${response.status}`);
            }
            const payload = await response.json();
            if (!payload || !payload.depth_image) {
                throw new Error('DA360 response missing depth_image');
            }
            this.depthImg.src = payload.depth_image;
            this.hasDepth = true;
            this.lastDepthTime = performance.now();
            const latency = Number.isFinite(payload.latency_ms)
                ? `${Math.round(payload.latency_ms)}ms`
                : `${Math.round(this.lastDepthTime - started)}ms`;
            this._setStatus('ready', latency);
        } catch (error) {
            console.warn('[PanoramaSensor] DA360 request failed:', error);
            this.lastDepthTime = performance.now();
            this._setStatus('ready', 'offline');
        } finally {
            window.clearTimeout(timeout);
            this.depthPending = false;
        }
    }
}
