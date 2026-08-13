/*
 * YOPO Navigation Client
 *
 * HTTP client for the YOPO autonomous navigation backend.
 * Handles depth image encoding, odometry packaging, and
 * PositionCommand decoding.
 */

const YOPO_DEFAULT_SERVER = 'http://localhost:5689';
// YOPO_360 ERP panorama resolution (192x384, 2 channels: depth + validity mask).
const YOPO_DEPTH_HEIGHT = 192;
const YOPO_DEPTH_WIDTH = 384;

export class YOPONavigator {
    constructor(serverUrl = YOPO_DEFAULT_SERVER) {
        this.serverUrl = serverUrl;
        this.goal = null;          // {x, y, z}
        this.arrived = false;
        this.distToGoal = 0;
        this.lastCmd = null;       // {position, velocity, acceleration, yaw, yaw_dot}
        this.inferenceCount = 0;
        this._lastRequestTime = 0;
        this._requestInterval = 33; // ms (~30 Hz)

        // ── WebSocket transport (efficient alternative to per-call HTTP) ──
        // Derive ws:// URL from the HTTP server URL (same host, port 5690).
        this.wsUrl = null;
        this.ws = null;
        this.wsReady = false;
        this._wsPending = new Map(); // id -> resolve()
        this._wsId = 0;
        try {
            const u = new URL(serverUrl);
            u.protocol = (u.protocol === 'https:' ? 'wss:' : 'ws:');
            u.port = '5690';
            this.wsUrl = u.toString();
            this._wsConnect();
        } catch (e) {
            this.wsUrl = null;
        }
    }

    _wsConnect() {
        if (typeof WebSocket === 'undefined' || !this.wsUrl) return;
        try {
            const ws = new WebSocket(this.wsUrl);
            this.ws = ws;
            ws.onopen = () => {
                this.wsReady = true;
                console.log(`[YOPO] WebSocket connected (${this.wsUrl})`);
            };
            ws.onclose = () => {
                this.wsReady = false;
                this.ws = null;
                console.warn('[YOPO] WebSocket closed; falling back to HTTP');
                // Retry later if the server comes up.
                setTimeout(() => this._wsConnect(), 2000);
            };
            ws.onerror = () => { /* onclose will handle cleanup */ };
            ws.onmessage = (ev) => this._wsOnMessage(ev);
        } catch (e) {
            this.ws = null;
        }
    }

    _wsOnMessage(ev) {
        try {
            const obj = JSON.parse(ev.data);
            const id = obj.id;
            if (id !== undefined && this._wsPending.has(id)) {
                const resolve = this._wsPending.get(id);
                this._wsPending.delete(id);
                resolve(obj);
            }
        } catch (e) { /* ignore malformed */ }
    }

    _wsSendJson(obj) {
        return new Promise((resolve) => {
            const id = ++this._wsId;
            obj.id = id;
            this._wsPending.set(id, resolve);
            try {
                this.ws.send(JSON.stringify(obj));
            } catch (e) {
                this._wsPending.delete(id);
                resolve(null);
            }
            setTimeout(() => {
                if (this._wsPending.has(id)) { this._wsPending.delete(id); resolve(null); }
            }, 3000);
        });
    }

    async _wsControl(position, velocity, orientation) {
        return this._wsSendJson({
            type: 'control',
            position, velocity, orientation,
        });
    }

    _wsSendBinary(header, blob) {
        return new Promise((resolve) => {
            const id = header.id;
            this._wsPending.set(id, resolve);
            try {
                this.ws.send(blob);
            } catch (e) {
                this._wsPending.delete(id);
                resolve(null);
            }
            setTimeout(() => {
                if (this._wsPending.has(id)) { this._wsPending.delete(id); resolve(null); }
            }, 3000);
        });
    }

    async _wsNavigate(depthData, position, velocity, orientation) {
        const id = ++this._wsId;
        const header = {
            type: 'navigate',
            id,
            depth_shape: [YOPO_DEPTH_HEIGHT, YOPO_DEPTH_WIDTH],
            depth_encoding: '32FC1',
            mask: false,
            position, velocity, orientation,
        };
        // Pack binary frame: [uint32 BE header_len][utf8 json][depth bytes]
        const enc = new TextEncoder();
        const hdrBytes = enc.encode(JSON.stringify(header));
        const hdrLen = new Uint8Array(4);
        new DataView(hdrLen.buffer).setUint32(0, hdrBytes.length, false);
        const depthBuf = depthData.buffer; // Float32Array raw bytes
        const blob = new Blob([hdrLen, hdrBytes, depthBuf]);
        return this._wsSendBinary(header, blob);
    }

    setServerUrl(url) {
        this.serverUrl = url;
    }

    /**
     * Set the navigation goal.
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    async setGoal(x, y, z) {
        try {
            const resp = await fetch(`${this.serverUrl}/yopo/set_goal`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ x, y, z }),
            });
            if (!resp.ok) {
                const text = await resp.text().catch(() => '');
                console.warn(`YOPO set_goal failed: ${resp.status} ${text}`);
                return false;
            }
            const data = await resp.json();
            this.goal = { x, y, z };
            this.arrived = false;
            console.log(`YOPO goal set: (${x.toFixed(1)}, ${y.toFixed(1)}, ${z.toFixed(1)})`);
            return true;
        } catch (err) {
            console.warn('YOPO set_goal error:', err);
            return false;
        }
    }

    /**
     * Check server status.
     */
    async getStatus() {
        try {
            const resp = await fetch(`${this.serverUrl}/yopo/status`);
            if (!resp.ok) return null;
            return await resp.json();
        } catch {
            return null;
        }
    }

    /**
     * Perform a single navigation step.
     *
     * @param {Float32Array|Uint16Array} depthData - Raw depth buffer
     * @param {string} depthEncoding - '32FC1' or '16UC1'
     * @param {{x,y,z}} position - World position
     * @param {{x,y,z}} velocity - World velocity
     * @param {{x,y,z,w}} orientation - Quaternion
     * @param {Uint8Array} [maskData] - Optional uint8 validity mask (255=valid,
     *        0=invalid), same HxW as depth. Sent as the second YOPO channel.
     * @returns {Promise<object|null>} Navigation command or null on failure
     */
    async navigate(depthData, depthEncoding, position, velocity, orientation, maskData) {
        const now = performance.now();
        if (now - this._lastRequestTime < this._requestInterval) {
            return this.lastCmd;
        }
        this._lastRequestTime = now;

        // ── Preferred: WebSocket (raw binary depth, no base64/HTTP overhead) ──
        if (this.wsReady && this.ws && depthData instanceof Float32Array) {
            const cmd = await this._wsNavigate(depthData, position, velocity, orientation);
            if (cmd && !cmd.error) {
                this.lastCmd = cmd;
                this.arrived = cmd.arrived || false;
                this.distToGoal = cmd.dist_to_goal || 0;
                this.inferenceCount++;
                return cmd;
            }
            // otherwise fall through to HTTP
        }

        // ── Fallback: HTTP ──
        // Encode depth as base64
        let depthBytes;
        if (depthData instanceof Float32Array) {
            depthBytes = new Uint8Array(depthData.buffer);
        } else if (depthData instanceof Uint16Array) {
            depthBytes = new Uint8Array(depthData.buffer);
        } else {
            console.warn('YOPO: unknown depth data type');
            return null;
        }

        const depthBase64 = this._arrayBufferToBase64(depthBytes);

        // Optional validity mask (uint8/mono8). Encoded to base64 and sent
        // alongside depth; the server stacks it as channel 1 when running in
        // 2-channel ERP mode.
        let maskBase64 = '';
        if (maskData instanceof Uint8Array && maskData.length > 0) {
            maskBase64 = this._arrayBufferToBase64(maskData);
        }

        try {
            const resp = await fetch(`${this.serverUrl}/yopo/navigate`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    depth: depthBase64,
                    depth_encoding: depthEncoding,
                    depth_shape: [YOPO_DEPTH_HEIGHT, YOPO_DEPTH_WIDTH],
                    position,
                    velocity,
                    orientation,
                    mask: maskBase64,
                }),
            });

            if (!resp.ok) {
                const text = await resp.text().catch(() => '');
                console.warn(`YOPO navigate failed: ${resp.status} ${text}`);
                return null;
            }

            const cmd = await resp.json();
            if (cmd.error) {
                console.warn('YOPO navigate error:', cmd.error);
                return null;
            }

            this.lastCmd = cmd;
            this.arrived = cmd.arrived || false;
            this.distToGoal = cmd.dist_to_goal || 0;
            this.inferenceCount++;
            return cmd;
        } catch (err) {
            console.warn('YOPO navigate error:', err);
            return null;
        }
    }

    /**
     * High-frequency control update (no depth/inference).
     *
     * Advances ctrl_time on the server and evaluates the last polynomial.
     * Called at ~60Hz by the render loop; navigate() replans at ~0.4Hz.
     * This separation prevents blind flight between depth frames.
     *
     * @param {{x,y,z}} position - World position
     * @param {{x,y,z}} velocity - World velocity
     * @param {{x,y,z,w}} orientation - Quaternion
     * @returns {Promise<object|null>} Control command or null on failure
     */
    async control(position, velocity, orientation) {
        // ── Preferred: WebSocket (no per-call HTTP handshake/headers) ──
        if (this.wsReady && this.ws) {
            const cmd = await this._wsControl(position, velocity, orientation);
            if (cmd && !cmd.error) {
                this.lastCmd = cmd;
                this.arrived = cmd.arrived || false;
                this.distToGoal = cmd.dist_to_goal || 0;
                return cmd;
            }
            // otherwise fall through to HTTP
        }

        try {
            const resp = await fetch(`${this.serverUrl}/yopo/control`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    position,
                    velocity,
                    orientation,
                }),
            });

            if (!resp.ok) {
                const text = await resp.text().catch(() => '');
                console.warn(`YOPO control failed: ${resp.status} ${text}`);
                return null;
            }

            const cmd = await resp.json();
            if (cmd.error) {
                console.warn('YOPO control error:', cmd.error);
                return null;
            }

            this.lastCmd = cmd;
            this.arrived = cmd.arrived || false;
            this.distToGoal = cmd.dist_to_goal || 0;
            return cmd;
        } catch (err) {
            // Silently handle transient errors at high frequency
            return null;
        }
    }

    /**
     * Check if the server is reachable.
     */
    async ping() {
        try {
            const resp = await fetch(`${this.serverUrl}/yopo/status`, {
                method: 'GET',
                signal: AbortSignal.timeout(2000),
            });
            return resp.ok;
        } catch {
            return false;
        }
    }

    /**
     * Convert an ArrayBuffer to a base64-encoded string.
     */
    _arrayBufferToBase64(buffer) {
        const bytes = new Uint8Array(buffer);
        let binary = '';
        const chunkSize = 8192;
        for (let i = 0; i < bytes.length; i += chunkSize) {
            binary += String.fromCharCode(...bytes.subarray(i, i + chunkSize));
        }
        return btoa(binary);
    }
}