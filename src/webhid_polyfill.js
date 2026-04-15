/**
 * WebHID Bridge Polyfill
 *
 * Implements the navigator.hid API for browsers that don't support WebHID
 * (e.g. Firefox) by proxying HID device I/O through a local WebSocket server
 * (hid_server.py on ws://localhost:8766).
 *
 * Injected before the main module in index.html.
 * Has no effect in Chrome/Edge where navigator.hid is already present.
 */
(function () {
    if (typeof navigator.hid !== 'undefined') return; // Already supported

    const WS_URL = 'ws://localhost:8766';

    // ── WebSocket state ──────────────────────────────────────────────────────
    let _ws = null;
    let _wsReady = false;

    // One pending resolver per expected response type (protocol is sequential)
    const _pending = {}; // responseType -> { resolve, reject }

    // All currently open virtual devices (id → HIDDeviceBridge)
    const _openDevices = new Map();

    function _connectWS() {
        return new Promise((resolve, reject) => {
            const ws = new WebSocket(WS_URL);
            ws.onopen = () => {
                _ws = ws;
                _wsReady = true;
                resolve();
            };
            ws.onerror = () => {
                reject(new Error(
                    'Cannot connect to WebHID bridge (ws://localhost:8766).\n' +
                    'Please start hid_server.py before connecting your device.'
                ));
            };
            ws.onclose = () => {
                _wsReady = false;
                _ws = null;
            };
            ws.onmessage = (ev) => {
                let msg;
                try { msg = JSON.parse(ev.data); } catch { return; }

                if (msg.type === 'report') {
                    // Stream: forward to every open device
                    _openDevices.forEach(dev => dev._dispatchReport(msg.reportId, msg.data));
                    return;
                }

                // Error response: reject every pending promise
                if (msg.type === 'error') {
                    const err = new Error(msg.message || 'HID bridge error');
                    Object.keys(_pending).forEach(k => {
                        const p = _pending[k];
                        delete _pending[k];
                        p.reject(err);
                    });
                    return;
                }

                // One-shot response: resolve the matching pending promise
                const p = _pending[msg.type];
                if (p) {
                    delete _pending[msg.type];
                    p.resolve(msg);
                }
            };
        });
    }

    function _send(msg, responseType) {
        return new Promise((resolve, reject) => {
            _pending[responseType] = { resolve, reject };
            _ws.send(JSON.stringify(msg));
        });
    }

    // ── HIDDeviceBridge ──────────────────────────────────────────────────────
    class HIDDeviceBridge extends EventTarget {
        constructor(info) {
            super();
            this._id          = info.id;
            this.productName  = info.productName  || 'HID Device';
            this.vendorId     = info.vendorId     || 0;
            this.productId    = info.productId    || 0;
            this.collections  = info.collections  || [];
            this.opened       = false;
        }

        async open() {
            const resp = await _send({ type: 'open', id: this._id }, 'opened');
            if (resp.type === 'error') throw new Error(resp.message);
            this.opened = true;
            _openDevices.set(this._id, this);
        }

        async close() {
            _openDevices.delete(this._id);
            this.opened = false;
            try {
                await _send({ type: 'close' }, 'closed');
            } catch { /* ignore if already disconnected */ }
        }

        /** Called by the WS message handler for every incoming HID report. */
        _dispatchReport(reportId, dataBytes) {
            const buffer = new Uint8Array(dataBytes).buffer;
            const event  = new Event('inputreport');
            event.reportId = reportId;
            event.data     = new DataView(buffer);
            event.device   = this;
            this.dispatchEvent(event);
        }
    }

    // ── Device picker dialog ─────────────────────────────────────────────────
    function _showPicker(devices, filters) {
        return new Promise((resolve) => {
            // Apply filters (fall back to full list if nothing matches)
            let list = devices;
            if (filters && filters.length > 0) {
                const filtered = devices.filter(d =>
                    filters.some(f => {
                        if (f.vendorId  !== undefined && f.vendorId  !== d.vendorId)  return false;
                        if (f.productId !== undefined && f.productId !== d.productId) return false;
                        if (f.usagePage !== undefined && f.usagePage !== d.usagePage) return false;
                        if (f.usage     !== undefined && f.usage     !== d.usage)     return false;
                        return true;
                    })
                );
                if (filtered.length > 0) list = filtered;
            }

            // ── overlay ──
            const overlay = document.createElement('div');
            Object.assign(overlay.style, {
                position: 'fixed', top: 0, left: 0, width: '100%', height: '100%',
                background: 'rgba(0,0,0,0.75)', zIndex: 10000,
                display: 'flex', alignItems: 'center', justifyContent: 'center',
                fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif",
            });

            // ── dialog box ──
            const dialog = document.createElement('div');
            Object.assign(dialog.style, {
                background: 'rgba(18,18,28,0.98)', border: '1px solid #4272F5',
                borderRadius: '12px', padding: '24px', minWidth: '420px',
                maxWidth: '560px', color: '#ddd', maxHeight: '80vh', overflowY: 'auto',
            });

            const title = document.createElement('h3');
            title.textContent = 'Connect HID Device';
            Object.assign(title.style, { color: '#4272F5', margin: '0 0 6px 0', fontSize: '1.1em' });
            dialog.appendChild(title);

            const sub = document.createElement('p');
            sub.textContent = 'Select your RC transmitter or controller:';
            Object.assign(sub.style, { color: '#888', fontSize: '12px', margin: '0 0 14px 0' });
            dialog.appendChild(sub);

            if (list.length === 0) {
                const msg = document.createElement('p');
                msg.textContent = 'No HID devices found. Connect your device and try again.';
                msg.style.color = '#f88';
                dialog.appendChild(msg);
            } else {
                list.forEach(dev => {
                    const btn = document.createElement('button');
                    Object.assign(btn.style, {
                        display: 'block', width: '100%', textAlign: 'left',
                        padding: '10px 14px', marginBottom: '8px',
                        background: '#0d1a2a', border: '1px solid #2a3a55',
                        borderRadius: '6px', color: '#ddd', cursor: 'pointer',
                        fontSize: '13px', lineHeight: '1.5',
                    });
                    const vid = dev.vendorId.toString(16).padStart(4, '0').toUpperCase();
                    const pid = dev.productId.toString(16).padStart(4, '0').toUpperCase();
                    const mfr = dev.manufacturerName ? `${dev.manufacturerName} · ` : '';
                    btn.innerHTML =
                        `<span style="color:#8cf;font-weight:bold;">${dev.productName}</span><br>` +
                        `<span style="color:#556;font-size:11px;">${mfr}VID:${vid}  PID:${pid}</span>`;

                    btn.addEventListener('mouseenter', () => { btn.style.background = '#162640'; btn.style.borderColor = '#4272F5'; });
                    btn.addEventListener('mouseleave', () => { btn.style.background = '#0d1a2a'; btn.style.borderColor = '#2a3a55'; });
                    btn.addEventListener('click', () => {
                        document.body.removeChild(overlay);
                        resolve([new HIDDeviceBridge(dev)]);
                    });
                    dialog.appendChild(btn);
                });
            }

            // ── Cancel ──
            const cancelBtn = document.createElement('button');
            cancelBtn.textContent = 'Cancel';
            Object.assign(cancelBtn.style, {
                marginTop: '8px', padding: '7px 20px', background: 'transparent',
                border: '1px solid #444', borderRadius: '6px', color: '#777',
                cursor: 'pointer', fontSize: '13px',
            });
            cancelBtn.addEventListener('click', () => {
                document.body.removeChild(overlay);
                resolve([]);
            });
            dialog.appendChild(cancelBtn);

            overlay.appendChild(dialog);
            document.body.appendChild(overlay);
        });
    }

    // ── navigator.hid polyfill object ────────────────────────────────────────
    const hidPolyfill = {
        _listeners: {},

        addEventListener(type, handler) {
            (this._listeners[type] = this._listeners[type] || []).push(handler);
        },

        removeEventListener(type, handler) {
            if (this._listeners[type])
                this._listeners[type] = this._listeners[type].filter(h => h !== handler);
        },

        async getDevices() {
            return []; // Pre-authorized devices not tracked across sessions
        },

        async requestDevice(options = {}) {
            // Ensure WebSocket connection to bridge server
            if (!_wsReady) {
                try {
                    await _connectWS();
                } catch (err) {
                    alert('[WebHID Bridge]\n' + err.message);
                    return [];
                }
            }

            // Fetch available devices from server
            let resp;
            try {
                resp = await _send({ type: 'list' }, 'devices');
            } catch (err) {
                alert('[WebHID Bridge] Failed to list devices: ' + err.message);
                return [];
            }

            return _showPicker(resp.devices || [], options.filters || []);
        },
    };

    Object.defineProperty(navigator, 'hid', {
        value: hidPolyfill,
        writable: false,
        configurable: false,
        enumerable: true,
    });

    console.info('[WebHID Polyfill] Active — bridging via ws://localhost:8766');
})();
