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
 * Controller input system — RC transmitter via Gamepad API + keyboard.
 * Supports channel assignment, axis inversion, dead zones, and listen-mode mapping.
 */

const ACTIONS = ['roll', 'pitch', 'throttle', 'yaw', 'cameraTilt'];
const BUTTON_ACTIONS = ['arm', 'reset'];

// All persisted slider/select IDs
const SETTINGS_IDS = [
    'flight-mode-select',
    'cam-hfov',
    'cam-mount-angle',
    'ctrl-pos-kp', 'ctrl-pos-ki', 'ctrl-pos-kd', 'ctrl-vel-kp', 'ctrl-vel-ki', 'ctrl-vel-kd', 'ctrl-alt-kp', 'ctrl-alt-ki', 'ctrl-alt-kd',
    'phys-mass', 'phys-thrust', 'phys-drag-cd', 'phys-drag-area',
    'phys-drone-size', 'phys-collision-radius',
    'clean-mode-toggle', 'osd-toggle',
];

// Settings that are stored separately per flight mode (drone vs fpv)
const PER_MODE_SETTINGS_IDS = [
    'ctrl-pos-kp', 'ctrl-pos-ki', 'ctrl-pos-kd',
    'ctrl-vel-kp', 'ctrl-vel-ki', 'ctrl-vel-kd',
    'ctrl-alt-kp', 'ctrl-alt-ki', 'ctrl-alt-kd',
];

const DEFAULT_MAPPING = {
    roll:       { axisIndex: 0, inverted: false, deadzone: 0, rate: 1.0, expo: 0.0 },
    pitch:      { axisIndex: 1, inverted: false, deadzone: 0, rate: 1.0, expo: 0.0 },
    throttle:   { axisIndex: 2, inverted: false, deadzone: 0, rate: 1.0, expo: 0.0 },
    yaw:        { axisIndex: 3, inverted: false, deadzone: 0, rate: 1.0, expo: 0.0 },
    cameraTilt: { axisIndex: -1, inverted: false, deadzone: 0, rate: 1.0, expo: 0.0 },
};

const DEFAULT_BUTTON_MAPPING = {
    arm:   { source: 'button', buttonIndex: 0, axisIndex: -1, axisThreshold: 0.5 },
    reset: { source: 'button', buttonIndex: 1, axisIndex: -1, axisThreshold: 0.5 },
};

const KEYBOARD_MAP = {
    'KeyW':       { action: 'throttle', value: 1 },
    'KeyS':       { action: 'throttle', value: -1 },
    'KeyA':       { action: 'yaw',      value: 1 },
    'KeyD':       { action: 'yaw',      value: -1 },
    'ArrowUp':    { action: 'pitch',    value: -1 },
    'ArrowDown':  { action: 'pitch',    value: 1 },
    'ArrowLeft':  { action: 'roll',     value: -1 },
    'ArrowRight': { action: 'roll',     value: 1 },
    'KeyQ':       { action: 'cameraTilt', value: 1 },
    'KeyE':       { action: 'cameraTilt', value: -1 },
};

export class Controller {
    constructor() {
        this.mapping = JSON.parse(JSON.stringify(DEFAULT_MAPPING));
        this.buttonMapping = JSON.parse(JSON.stringify(DEFAULT_BUTTON_MAPPING));
        this.gamepadIndex = -1;
        this.gamepadName = '';
        this.connected = false;

        // Per-mode rate/expo snapshots (mapping.rate and mapping.expo per axis)
        this._modeRateExpo = {
            drone: this._snapshotRateExpo(),
            fpv:   this._snapshotRateExpo(),
        };
        // Per-mode PID settings (slider values keyed by element id)
        this._modePidSettings = {
            drone: null, // populated on first mode switch or from saved config
            fpv:   null,
        };
        this._currentMode = 'drone';

        // WebHID support for RC transmitters
        this._hidDevice = null;
        this._hidAxes = new Array(16).fill(0); // Up to 16 channels, normalized to -1..1
        this._hidRawAxes = new Array(16).fill(0); // raw 16-bit values before calibration
        this._hidCalibration = Array.from({length: 16}, () => ({ min: null, center: null, max: null }));
        this._hidConnected = false;
        this._hidDeviceName = '';
        
        // Option to disable Gamepad API (allows WebHID to claim the device)
        this._gamepadApiDisabled = false;

        // Current input state (merged keyboard + gamepad, range [-1, 1])
        this.axes = { roll: 0, pitch: 0, throttle: 0, yaw: 0, cameraTilt: 0 };
        this._cameraTiltKeyboard = 0;
        this._cameraTiltAxis = 0;
        this._prevCameraTiltAxis = 0;
        this.buttons = { arm: false, reset: false };
        this.boost = false;

        // Separate current/previous state for keyboard and gamepad edge detection
        this._gpButtons = { arm: false, reset: false };
        this._prevKbButtons = { arm: false, reset: false };
        this._prevGpButtons = { arm: false, reset: false };

        // Keyboard state
        this._keysDown = new Set();

        // Listen mode (axes)
        this._listenAction = null;
        this._listenCallback = null;
        this._listenBaseline = null;

        // Listen mode (buttons)
        this._listenButtonAction = null;
        this._listenButtonCallback = null;
        this._listenButtonBaseline = null;

        // Armed state
        this.armed = false;

        // Load saved config
        this._loadConfig();

        // Setup event listeners
        this._setupKeyboard();
        this._setupGamepad();
        this._buildSettingsUI();

        // Ensure both modes have valid PID + rate/expo snapshots.
        // After _loadConfig + _buildSettingsUI, DOM has the current mode's values.
        // Snapshot them for the current mode if not yet saved.
        if (!this._modePidSettings[this._currentMode]) {
            this._modePidSettings[this._currentMode] = this._snapshotPidSettings();
        }
        if (!this._modePidSettings[this._currentMode === 'drone' ? 'fpv' : 'drone']) {
            // Other mode has no saved PID — initialize from HTML defaults (same as current)
            this._modePidSettings[this._currentMode === 'drone' ? 'fpv' : 'drone'] = this._snapshotPidSettings();
        }
    }

    /**
     * Call once per frame to poll gamepad and update axes.
     */
    update() {
        // Reset axes and buttons to 0/false each frame
        for (const action of ACTIONS) {
            this.axes[action] = 0;
        }
        this.buttons.arm = false;
        this.buttons.reset = false;
        this._gpButtons.arm = false;
        this._gpButtons.reset = false;
        this.boost = false;
        this._cameraTiltKeyboard = 0;
        this._prevCameraTiltAxis = this._cameraTiltAxis;
        this._cameraTiltAxis = 0;

        // Keyboard input
        for (const [code, map] of Object.entries(KEYBOARD_MAP)) {
            if (this._keysDown.has(code)) {
                this.axes[map.action] += map.value;
                if (map.action === 'cameraTilt') {
                    this._cameraTiltKeyboard += map.value;
                }
            }
        }
        if (this._keysDown.has('ShiftLeft') || this._keysDown.has('ShiftRight')) {
            this.boost = true;
        }

        // Gamepad input (prefer WebHID if connected)
        const hidAxes = this._getHIDAxes();
        const gp = this._getGamepad();
        
        if (hidAxes) {
            // Use WebHID input
            this.connected = true;
            this.gamepadName = this._hidDeviceName + ' (HID)';

            for (const action of ACTIONS) {
                const m = this.mapping[action];
                if (m.axisIndex >= 0 && m.axisIndex < hidAxes.length) {
                    let val = hidAxes[m.axisIndex];
                    if (m.inverted) val = -val;
                    if (Math.abs(val) < m.deadzone) val = 0;
                    // Apply expo curve
                    const e = m.expo || 0;
                    if (e > 0) {
                        val = Math.sign(val) * Math.abs(val) * (1 - e + e * val * val);
                    }
                    this.axes[action] += val;
                    if (action === 'cameraTilt') {
                        this._cameraTiltAxis = val;
                    }
                }
            }

            // HID button handling (use high channels as buttons)
            for (const bAction of BUTTON_ACTIONS) {
                const bm = this.buttonMapping[bAction];
                if (bm.source === 'axis' && bm.axisIndex >= 0 && bm.axisIndex < hidAxes.length) {
                    this._gpButtons[bAction] = Math.abs(hidAxes[bm.axisIndex]) > bm.axisThreshold;
                }
            }

            // Listen mode for HID: detect axis movement
            if (this._listenAction && this._listenBaseline) {
                let maxDelta = 0;
                let bestAxis = -1;
                let bestSign = 1;
                for (let i = 0; i < hidAxes.length; i++) {
                    const delta = hidAxes[i] - this._listenBaseline[i];
                    if (Math.abs(delta) > Math.abs(maxDelta)) {
                        maxDelta = delta;
                        bestAxis = i;
                        bestSign = Math.sign(delta);
                    }
                }
                if (Math.abs(maxDelta) > 0.3) { // Lower threshold for HID (more sensitive)
                    if (this.mapping[this._listenAction]) {
                        this.mapping[this._listenAction].axisIndex = bestAxis;
                        this.mapping[this._listenAction].inverted = bestSign < 0;
                    }
                    const action = this._listenAction;
                    this._listenAction = null;
                    this._listenBaseline = null;
                    if (this._listenCallback) this._listenCallback(action, bestAxis, bestSign < 0);
                    this._saveConfig();
                    this._buildSettingsUI();
                }
            }

            // Update HID display
            this._updateHIDDisplay(hidAxes);
        } else if (gp) {
            this.connected = true;
            this.gamepadName = gp.id;

            for (const action of ACTIONS) {
                const m = this.mapping[action];
                if (m.axisIndex >= 0 && m.axisIndex < gp.axes.length) {
                    let val = gp.axes[m.axisIndex];
                    if (m.inverted) val = -val;
                    if (Math.abs(val) < m.deadzone) val = 0;
                    // Apply expo curve: output = val * (1 - expo + expo * val²)
                    const e = m.expo || 0;
                    if (e > 0) {
                        val = Math.sign(val) * Math.abs(val) * (1 - e + e * val * val);
                    }
                    this.axes[action] += val;
                    if (action === 'cameraTilt') {
                        this._cameraTiltAxis = val;
                    }
                }
            }

            // Listen mode: detect axis movement
            if (this._listenAction && this._listenBaseline) {
                let maxDelta = 0;
                let bestAxis = -1;
                let bestSign = 1;
                for (let i = 0; i < gp.axes.length; i++) {
                    const delta = gp.axes[i] - this._listenBaseline[i];
                    if (Math.abs(delta) > Math.abs(maxDelta)) {
                        maxDelta = delta;
                        bestAxis = i;
                        bestSign = Math.sign(delta);
                    }
                }
                if (Math.abs(maxDelta) > 0.5) {
                    // Axis detected — update mapping only for stick actions
                    if (this.mapping[this._listenAction]) {
                        this.mapping[this._listenAction].axisIndex = bestAxis;
                        this.mapping[this._listenAction].inverted = bestSign < 0;
                    }
                    const action = this._listenAction;
                    this._listenAction = null;
                    this._listenBaseline = null;
                    if (this._listenCallback) this._listenCallback(action, bestAxis, bestSign < 0);
                    this._saveConfig();
                    this._buildSettingsUI();
                }
            }

            // Button listen mode: detect button press
            if (this._listenButtonAction && this._listenButtonBaseline) {
                for (let i = 0; i < gp.buttons.length; i++) {
                    if (gp.buttons[i].pressed && !this._listenButtonBaseline[i]) {
                        this.buttonMapping[this._listenButtonAction].buttonIndex = i;
                        const action = this._listenButtonAction;
                        this._listenButtonAction = null;
                        this._listenButtonBaseline = null;
                        if (this._listenButtonCallback) this._listenButtonCallback(action, i);
                        this._saveConfig();
                        this._buildSettingsUI();
                        break;
                    }
                }
            }

            // Gamepad buttons — read raw physical state (no inversion)
            for (const bAction of BUTTON_ACTIONS) {
                const bm = this.buttonMapping[bAction];
                let pressed = false;
                if (bm.source === 'axis' && bm.axisIndex >= 0 && bm.axisIndex < gp.axes.length) {
                    pressed = Math.abs(gp.axes[bm.axisIndex]) > bm.axisThreshold;
                } else if (bm.buttonIndex >= 0 && bm.buttonIndex < gp.buttons.length) {
                    pressed = gp.buttons[bm.buttonIndex].pressed;
                }
                this._gpButtons[bAction] = pressed;
            }

            // Update gamepad display
            this._updateGamepadDisplay(gp);
        } else {
            this.connected = false;
        }

        // Keyboard buttons
        const kbArm = this._keysDown.has('Space');
        const kbReset = this._keysDown.has('KeyR');

        // Clamp axes
        for (const action of ACTIONS) {
            this.axes[action] = Math.max(-1, Math.min(1, this.axes[action]));
        }

        // Separate edge detection: gamepad and keyboard independently
        const gpArmRising   = this._gpButtons.arm   && !this._prevGpButtons.arm;
        const gpResetRising = this._gpButtons.reset  && !this._prevGpButtons.reset;
        const kbArmRising   = kbArm   && !this._prevKbButtons.arm;
        const kbResetRising = kbReset  && !this._prevKbButtons.reset;

        const armRising  = gpArmRising  || kbArmRising;
        const resetRising = gpResetRising || kbResetRising;

        if (armRising) {
            this.armed = !this.armed;
        }

        this._prevGpButtons.arm   = this._gpButtons.arm;
        this._prevGpButtons.reset = this._gpButtons.reset;
        this._prevKbButtons.arm   = kbArm;
        this._prevKbButtons.reset = kbReset;

        return {
            roll: this.axes.roll,
            pitch: this.axes.pitch,
            throttle: this.axes.throttle,
            yaw: this.axes.yaw,
            cameraTilt: this.axes.cameraTilt,
            cameraTiltKeyboard: this._cameraTiltKeyboard,
            cameraTiltAxis: this._cameraTiltAxis,
            cameraTiltAxisChanged: Math.abs(this._cameraTiltAxis - this._prevCameraTiltAxis) > 0.01,
            boost: this.boost,
            armed: this.armed,
            resetTriggered: resetRising,
            rates: {
                roll:  this.mapping.roll.rate  !== undefined ? this.mapping.roll.rate  : 1.0,
                pitch: this.mapping.pitch.rate !== undefined ? this.mapping.pitch.rate : 1.0,
                yaw:   this.mapping.yaw.rate   !== undefined ? this.mapping.yaw.rate   : 1.0,
            },
        };
    }

    startListening(action, callback) {
        // Support both Gamepad API and WebHID
        if (this._hidConnected) {
            this._listenAction = action;
            this._listenCallback = callback;
            this._listenBaseline = [...this._hidAxes];
            return true;
        }
        const gp = this._getGamepad();
        if (!gp) return false;
        this._listenAction = action;
        this._listenCallback = callback;
        this._listenBaseline = Array.from(gp.axes);
        return true;
    }

    cancelListening() {
        this._listenAction = null;
        this._listenCallback = null;
        this._listenBaseline = null;
    }

    startButtonListening(action, callback) {
        const gp = this._getGamepad();
        if (!gp) return false;
        this._listenButtonAction = action;
        this._listenButtonCallback = callback;
        this._listenButtonBaseline = Array.from(gp.buttons.map(b => b.pressed));
        return true;
    }

    cancelButtonListening() {
        this._listenButtonAction = null;
        this._listenButtonCallback = null;
        this._listenButtonBaseline = null;
    }

    getConfig() {
        const settings = {};
        for (const id of SETTINGS_IDS) {
            const el = document.getElementById(id);
            if (!el) continue;
            settings[id] = el.type === 'checkbox' ? el.checked : el.value;
        }

        // Snapshot current mode before saving so both modes are up-to-date
        this._modeRateExpo[this._currentMode] = this._snapshotRateExpo();
        this._modePidSettings[this._currentMode] = this._snapshotPidSettings();

        return {
            mapping: JSON.parse(JSON.stringify(this.mapping)),
            buttonMapping: JSON.parse(JSON.stringify(this.buttonMapping)),
            hidCalibration: JSON.parse(JSON.stringify(this._hidCalibration)),
            settings,
            modeRateExpo: JSON.parse(JSON.stringify(this._modeRateExpo)),
            modePidSettings: JSON.parse(JSON.stringify(this._modePidSettings)),
            currentMode: this._currentMode,
        };
    }

    loadConfig(config) {
        if (config.mapping) this.mapping = config.mapping;
        if (config.buttonMapping) this.buttonMapping = config.buttonMapping;
        if (config.hidCalibration) this._hidCalibration = config.hidCalibration;
        if (config.modeRateExpo) this._modeRateExpo = config.modeRateExpo;
        if (config.modePidSettings) this._modePidSettings = config.modePidSettings;
        if (config.currentMode) this._currentMode = config.currentMode;
        if (config.settings) this._restoreSettings(config.settings);
        this._saveConfig();
        this._buildSettingsUI();
    }

    _restoreSettings(settings) {
        for (const [id, val] of Object.entries(settings)) {
            const el = document.getElementById(id);
            if (!el) continue;
            if (el.type === 'checkbox') {
                el.checked = !!val;
                el.dispatchEvent(new Event('change'));
            } else {
                el.value = val;
                // Sync paired number input if present
                const numEl = document.getElementById(id + '-num');
                if (numEl) numEl.value = val;
                // Sync paired span display if present
                const spanEl = document.getElementById(id + '-val');
                if (spanEl) spanEl.textContent = parseFloat(val).toFixed(el.step && el.step.includes('.') ? 2 : 0);
                el.dispatchEvent(new Event('input'));
            }
        }
    }

    // ---- Per-mode helpers ----

    _snapshotRateExpo() {
        const snap = {};
        for (const action of ACTIONS) {
            const m = this.mapping[action];
            snap[action] = { rate: m.rate !== undefined ? m.rate : 1.0, expo: m.expo !== undefined ? m.expo : 0.0 };
        }
        return snap;
    }

    _restoreRateExpo(snap) {
        if (!snap) return;
        for (const action of ACTIONS) {
            if (snap[action]) {
                this.mapping[action].rate = snap[action].rate;
                this.mapping[action].expo = snap[action].expo;
            }
        }
    }

    _snapshotPidSettings() {
        const snap = {};
        for (const id of PER_MODE_SETTINGS_IDS) {
            const el = document.getElementById(id);
            if (el) snap[id] = el.value;
        }
        return snap;
    }

    _restorePidSettings(snap) {
        if (!snap) return;
        for (const [id, val] of Object.entries(snap)) {
            const el = document.getElementById(id);
            if (!el) continue;
            el.value = val;
            const numEl = document.getElementById(id + '-num');
            if (numEl) numEl.value = val;
            el.dispatchEvent(new Event('input'));
        }
    }

    /**
     * Called when the flight-mode-select dropdown changes.
     * Saves current mode's rate/expo + PID, restores the new mode's values.
     */
    _onModeSwitch(newMode) {
        const oldMode = this._currentMode;
        if (newMode === oldMode) return;

        // Save current mode's values
        this._modeRateExpo[oldMode] = this._snapshotRateExpo();
        this._modePidSettings[oldMode] = this._snapshotPidSettings();

        // Restore new mode's values
        this._restoreRateExpo(this._modeRateExpo[newMode]);
        this._restorePidSettings(this._modePidSettings[newMode]);

        this._currentMode = newMode;
        this._saveConfig();
        this._buildSettingsUI();
    }

    // ---- Private methods ----

    _getGamepad() {
        // If Gamepad API is disabled, return null to allow WebHID to claim device
        if (this._gamepadApiDisabled) return null;
        
        const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
        for (const gp of gamepads) {
            if (gp && gp.connected) {
                this.gamepadIndex = gp.index;
                return gp;
            }
        }
        return null;
    }

    _setupKeyboard() {
        window.addEventListener('keydown', (e) => {
            if (e.code === 'Tab') {
                e.preventDefault();
                this._toggleSettings();
                return;
            }
            if (e.code === 'Escape' && this.isSettingsOpen()) {
                e.preventDefault();
                e.stopImmediatePropagation(); // prevent main.js handler from seeing this Esc
                this.closeSettings();
                return;
            }
            this._keysDown.add(e.code);
        });
        window.addEventListener('keyup', (e) => {
            this._keysDown.delete(e.code);
        });
        // Clear keys on blur
        window.addEventListener('blur', () => {
            this._keysDown.clear();
        });
    }

    _setupGamepad() {
        window.addEventListener('gamepadconnected', (e) => {
            this.connected = true;
            this.gamepadIndex = e.gamepad.index;
            this.gamepadName = e.gamepad.id;
            console.log(`Gamepad connected: ${e.gamepad.id}`);
            this._buildSettingsUI();
        });
        window.addEventListener('gamepaddisconnected', (e) => {
            if (e.gamepad.index === this.gamepadIndex) {
                this.connected = false;
                this.gamepadIndex = -1;
                this.gamepadName = '';
            }
            this._buildSettingsUI();
        });
    }

    // ---- WebHID Support for RC Transmitters ----

    async connectHID() {
        if (!navigator.hid) {
            alert('WebHID is not supported in this browser. Please use Chrome or Edge.');
            return false;
        }

        try {
            // Request HID device - filter for common RC transmitter vendor IDs
            const devices = await navigator.hid.requestDevice({
                filters: [
                    // RadioMaster transmitters (may vary by model)
                    { vendorId: 0x1209 }, // Generic HID
                    { vendorId: 0x0483 }, // STMicroelectronics (common in RC transmitters)
                    { vendorId: 0x239A }, // Adafruit
                    { vendorId: 0x2341 }, // Arduino
                ],
            });

            if (devices.length === 0) {
                // Try without filter if no device found
                const allDevices = await navigator.hid.requestDevice({ filters: [] });
                if (allDevices.length === 0) {
                    console.log('No HID device selected');
                    return false;
                }
                return this._openHIDDevice(allDevices[0]);
            }

            return this._openHIDDevice(devices[0]);
        } catch (error) {
            console.error('HID connection error:', error);
            return false;
        }
    }

    async _openHIDDevice(device) {
        try {
            if (!device.opened) {
                await device.open();
            }

            this._hidDevice = device;
            this._hidConnected = true;
            this._hidDeviceName = device.productName || 'HID Device';

            console.log('HID Device connected:', this._hidDeviceName);
            console.log('  Vendor ID:', device.vendorId.toString(16));
            console.log('  Product ID:', device.productId.toString(16));
            console.log('  Collections:', device.collections);

            // Set up input report handler
            device.addEventListener('inputreport', (event) => {
                this._handleHIDInputReport(event);
            });

            this._buildSettingsUI();
            return true;
        } catch (error) {
            console.error('Failed to open HID device:', error);
            return false;
        }
    }

    _handleHIDInputReport(event) {
        const { data, reportId } = event;
        const bytes = new Uint8Array(data.buffer);

        // Debug: log reports periodically to understand format
        if (!this._hidReportCount) this._hidReportCount = 0;
        this._hidReportCount++;
        if (this._hidReportCount % 100 === 1) { // Every 100 reports (~1-2 seconds)
            console.log('=== HID Report ===');
            console.log('Report ID:', reportId, 'Length:', bytes.length, 'bytes');
            console.log('Raw hex:', Array.from(bytes.slice(0, 32)).map(b => b.toString(16).padStart(2, '0')).join(' '));
            
            // Try to interpret as 16-bit little-endian values
            const vals16 = [];
            for (let i = 0; i < Math.min(16, bytes.length / 2); i++) {
                vals16.push(bytes[i * 2] | (bytes[i * 2 + 1] << 8));
            }
            console.log('As 16-bit LE:', vals16.join(', '));
            
            // Try to interpret as 16-bit big-endian values
            const vals16BE = [];
            for (let i = 0; i < Math.min(16, bytes.length / 2); i++) {
                vals16BE.push((bytes[i * 2] << 8) | bytes[i * 2 + 1]);
            }
            console.log('As 16-bit BE:', vals16BE.join(', '));
            
            console.log('Parsed axes:', this._hidAxes.slice(0, 8).map(v => v.toFixed(4)).join(', '));
        }

        // Parse the HID report - try different formats
        this._parseHIDReport(bytes);
    }

    _parseHIDReport(bytes) {
        const channelCount = Math.min(16, Math.floor(bytes.length / 2));
        for (let i = 0; i < channelCount; i++) {
            const raw = bytes[i * 2] | (bytes[i * 2 + 1] << 8);
            this._hidRawAxes[i] = raw;
            this._hidAxes[i] = this._applyCalibration(i, raw);
        }
    }

    _applyCalibration(i, raw) {
        const cal = this._hidCalibration[i];
        if (cal.min === null || cal.max === null || cal.center === null) {
            // No calibration: assume 0-2047 range, center 1024
            return Math.max(-1, Math.min(1, (raw - 1024) / 1024));
        }
        const center = cal.center;
        const span = raw >= center
            ? Math.max(1, cal.max - center)
            : Math.max(1, center - cal.min);
        return Math.max(-1, Math.min(1, (raw - center) / span));
    }

    _hasCalibration() {
        return this._hidCalibration.some(c => c.min !== null);
    }

    startCalibration() {
        if (!this._hidConnected) {
            alert('Connect a HID device first.');
            return;
        }

        const calMin = new Array(16).fill(null);
        const calMax = new Array(16).fill(null);

        // ── overlay ──
        const overlay = document.createElement('div');
        Object.assign(overlay.style, {
            position: 'fixed', top: 0, left: 0, width: '100%', height: '100%',
            background: 'rgba(0,0,0,0.8)', zIndex: 20000,
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif",
        });

        const dialog = document.createElement('div');
        Object.assign(dialog.style, {
            background: 'rgba(14,18,28,0.98)', border: '1px solid #f80',
            borderRadius: '12px', padding: '24px', width: '460px',
            color: '#ddd', userSelect: 'none',
        });

        const titleEl = document.createElement('h3');
        Object.assign(titleEl.style, { color: '#f80', margin: '0 0 6px', fontSize: '1.1em' });
        titleEl.textContent = '● Recording Calibration…';

        const instrEl = document.createElement('p');
        Object.assign(instrEl.style, { fontSize: '13px', color: '#aaa', margin: '0 0 14px', lineHeight: '1.5' });
        instrEl.innerHTML = 'Move <b>all sticks and dials</b> to their full extremes in every direction, then center them and click <b>Stop & Save</b>.';

        const barsEl = document.createElement('div');
        Object.assign(barsEl.style, { fontFamily: 'monospace', fontSize: '11px', marginBottom: '14px' });

        const btnRow = document.createElement('div');
        Object.assign(btnRow.style, { display: 'flex', gap: '8px', justifyContent: 'flex-end' });

        const cancelBtn = document.createElement('button');
        cancelBtn.textContent = 'Cancel';
        Object.assign(cancelBtn.style, {
            padding: '7px 18px', background: 'transparent', border: '1px solid #555',
            borderRadius: '6px', color: '#888', cursor: 'pointer', fontSize: '13px',
        });

        const saveBtn = document.createElement('button');
        saveBtn.textContent = 'Stop & Save';
        Object.assign(saveBtn.style, {
            padding: '7px 22px', background: '#f80', border: 'none',
            borderRadius: '6px', color: '#000', cursor: 'pointer', fontSize: '13px', fontWeight: 'bold',
        });

        btnRow.appendChild(cancelBtn);
        btnRow.appendChild(saveBtn);
        dialog.appendChild(titleEl);
        dialog.appendChild(instrEl);
        dialog.appendChild(barsEl);
        dialog.appendChild(btnRow);
        overlay.appendChild(dialog);
        document.body.appendChild(overlay);

        let rafId = null;

        const tick = () => {
            for (let i = 0; i < 16; i++) {
                const raw = this._hidRawAxes[i];
                if (calMin[i] === null || raw < calMin[i]) calMin[i] = raw;
                if (calMax[i] === null || raw > calMax[i]) calMax[i] = raw;
            }

            let html = '';
            for (let i = 0; i < 16; i++) {
                const raw = this._hidRawAxes[i];
                const mn = calMin[i] ?? raw;
                const mx = calMax[i] ?? raw;
                const range = Math.max(1, mx - mn);
                const pct = Math.max(0, Math.min(100, ((raw - mn) / range) * 100));
                const active = (mx - mn) > 10;
                const barColor = active ? '#f80' : '#334';
                html += `<div style="display:flex;align-items:center;gap:4px;margin-bottom:2px;">` +
                    `<span style="width:32px;text-align:right;color:${active ? '#aaa' : '#445'};">CH${i+1}</span>` +
                    `<div style="flex:1;height:8px;background:#1a2030;border-radius:2px;overflow:hidden;">` +
                    `<div style="width:${pct}%;height:100%;background:${barColor};border-radius:2px;"></div></div>` +
                    `<span style="width:90px;text-align:right;color:${active ? '#888' : '#334'};font-size:10px;">${mn}…${mx}</span>` +
                    `</div>`;
            }
            barsEl.innerHTML = html;
            rafId = requestAnimationFrame(tick);
        };
        rafId = requestAnimationFrame(tick);

        saveBtn.addEventListener('click', () => {
            cancelAnimationFrame(rafId);
            // Validate: at least one axis with meaningful range
            const valid = calMin.filter((mn, i) => mn !== null && calMax[i] - mn > 10).length;
            if (valid === 0) {
                instrEl.innerHTML = '<span style="color:#f44">⚠ No movement detected — move sticks first, then click Stop & Save.</span>';
                rafId = requestAnimationFrame(tick);
                return;
            }
            // Center = midpoint of recorded range (works for both self-centering sticks and throttle)
            for (let i = 0; i < 16; i++) {
                const mn = calMin[i]  ?? 0;
                const mx = calMax[i]  ?? 2047;
                this._hidCalibration[i] = {
                    min:    mn,
                    max:    mx,
                    center: Math.round((mn + mx) / 2),
                };
            }
            this._saveConfig();
            this._buildSettingsUI();
            document.body.removeChild(overlay);
        });

        cancelBtn.addEventListener('click', () => {
            cancelAnimationFrame(rafId);
            document.body.removeChild(overlay);
        });
    }

    clearCalibration() {
        this._hidCalibration = Array.from({length: 16}, () => ({ min: null, center: null, max: null }));
        this._saveConfig();
        this._buildSettingsUI();
    }

    disconnectHID() {
        if (this._hidDevice) {
            this._hidDevice.close();
            this._hidDevice = null;
            this._hidConnected = false;
            this._hidDeviceName = '';
            this._hidAxes.fill(0);
            this._buildSettingsUI();
        }
    }

    _getHIDAxes() {
        return this._hidConnected ? this._hidAxes : null;
    }

    isSettingsOpen() {
        const panel = document.getElementById('settings-panel');
        return panel && panel.classList.contains('visible');
    }

    closeSettings() {
        const panel = document.getElementById('settings-panel');
        if (panel) panel.classList.remove('visible');
    }

    _toggleSettings() {
        const panel = document.getElementById('settings-panel');
        panel.classList.toggle('visible');
    }

    _buildSettingsUI() {
        const container = document.getElementById('channel-assignments');
        if (!container) return;
        container.innerHTML = '';

        for (const action of ACTIONS) {
            const m = this.mapping[action];
            const row = document.createElement('div');
            row.className = 'setting-row';

            const label = document.createElement('label');
            label.textContent = action.charAt(0).toUpperCase() + action.slice(1);
            row.appendChild(label);

            const controls = document.createElement('div');
            controls.className = 'controls';

            // Axis label
            const axisLabel = document.createElement('span');
            axisLabel.className = 'axis-label';
            axisLabel.textContent = m.axisIndex >= 0 ? `Axis ${m.axisIndex}` : 'None';
            controls.appendChild(axisLabel);

            // Invert checkbox
            const invertLabel = document.createElement('label');
            invertLabel.style.fontSize = '12px';
            invertLabel.style.color = '#aaa';
            const invertCb = document.createElement('input');
            invertCb.type = 'checkbox';
            invertCb.checked = m.inverted;
            invertCb.addEventListener('change', () => {
                this.mapping[action].inverted = invertCb.checked;
                this._saveConfig();
            });
            invertLabel.appendChild(invertCb);
            invertLabel.appendChild(document.createTextNode(' Inv'));
            controls.appendChild(invertLabel);

            // Dead zone slider
            const dzLabel = document.createElement('span');
            dzLabel.style.cssText = 'font-size:11px;color:#888;margin-left:6px;';
            dzLabel.textContent = 'DZ';
            controls.appendChild(dzLabel);
            const dzSlider = document.createElement('input');
            dzSlider.type = 'range';
            dzSlider.min = '0'; dzSlider.max = '0.5'; dzSlider.step = '0.01';
            dzSlider.value = m.deadzone;
            dzSlider.style.width = '60px';
            const dzVal = document.createElement('span');
            dzVal.className = 'deadzone-val';
            dzVal.textContent = m.deadzone.toFixed(2);
            dzSlider.addEventListener('input', () => {
                this.mapping[action].deadzone = parseFloat(dzSlider.value);
                dzVal.textContent = parseFloat(dzSlider.value).toFixed(2);
                this._saveConfig();
            });
            controls.appendChild(dzSlider);
            controls.appendChild(dzVal);

            // Assign button
            const assignBtn = document.createElement('button');
            assignBtn.className = 'assign-btn';
            assignBtn.textContent = 'Assign';
            assignBtn.addEventListener('click', () => {
                if (this._listenAction === action) {
                    this.cancelListening();
                    assignBtn.classList.remove('listening');
                    assignBtn.textContent = 'Assign';
                    return;
                }
                // Cancel any other listening
                this.cancelListening();
                document.querySelectorAll('.assign-btn.listening').forEach(b => {
                    b.classList.remove('listening');
                    b.textContent = b._origText || 'Assign';
                });

                const started = this.startListening(action, (a, axis, inverted) => {
                    assignBtn.classList.remove('listening');
                    assignBtn.textContent = 'Assign';
                    axisLabel.textContent = `Axis ${axis}`;
                    invertCb.checked = inverted;
                });
                if (started) {
                    assignBtn.classList.add('listening');
                    assignBtn.textContent = 'Move stick...';
                } else {
                    alert('No gamepad detected. Connect your RC transmitter first.');
                }
            });
            assignBtn._origText = 'Assign';
            controls.appendChild(assignBtn);

            // None button (unassign)
            const noneBtn = document.createElement('button');
            noneBtn.className = 'assign-btn';
            noneBtn.textContent = 'None';
            noneBtn.style.cssText = 'font-size:11px;padding:2px 6px;';
            noneBtn.addEventListener('click', () => {
                this.cancelListening();
                this.mapping[action].axisIndex = -1;
                axisLabel.textContent = 'None';
                this._saveConfig();
            });
            controls.appendChild(noneBtn);

            row.appendChild(controls);
            container.appendChild(row);
        }

        // Rates & Expo section
        this._buildRatesExpoUI();

        // Button assignments (support button or axis source)
        const btnContainer = document.getElementById('button-assignments');
        if (btnContainer) {
            btnContainer.innerHTML = '';
            for (const bAction of BUTTON_ACTIONS) {
                const bm = this.buttonMapping[bAction];
                // Ensure new fields exist (migration from old config)
                if (!bm.source) bm.source = 'button';
                if (bm.axisIndex === undefined) bm.axisIndex = -1;
                if (bm.axisThreshold === undefined) bm.axisThreshold = 0.5;

                const row = document.createElement('div');
                row.className = 'setting-row';

                const label = document.createElement('label');
                label.textContent = bAction.charAt(0).toUpperCase() + bAction.slice(1);
                row.appendChild(label);

                const controls = document.createElement('div');
                controls.className = 'controls';

                // Current assignment label
                const srcLabel = document.createElement('span');
                srcLabel.className = 'axis-label';
                if (bm.source === 'axis') {
                    srcLabel.textContent = `Axis ${bm.axisIndex}`;
                } else {
                    srcLabel.textContent = bm.buttonIndex >= 0 ? `Btn ${bm.buttonIndex}` : 'None';
                }
                controls.appendChild(srcLabel);

                // Assign Button (detect button press)
                const assignBtnBtn = document.createElement('button');
                assignBtnBtn.className = 'assign-btn';
                assignBtnBtn.textContent = 'Btn';
                assignBtnBtn.title = 'Assign to a button';
                assignBtnBtn.addEventListener('click', () => {
                    if (this._listenButtonAction === bAction) {
                        this.cancelButtonListening();
                        assignBtnBtn.classList.remove('listening');
                        assignBtnBtn.textContent = 'Btn';
                        return;
                    }
                    this.cancelButtonListening();
                    this.cancelListening();
                    document.querySelectorAll('.assign-btn.listening').forEach(b => {
                        b.classList.remove('listening');
                        b.textContent = b._origText || 'Assign';
                    });

                    const started = this.startButtonListening(bAction, (a, btnIdx) => {
                        assignBtnBtn.classList.remove('listening');
                        assignBtnBtn.textContent = 'Btn';
                        bm.source = 'button';
                        bm.buttonIndex = btnIdx;
                        srcLabel.textContent = `Btn ${btnIdx}`;
                    });
                    if (started) {
                        assignBtnBtn.classList.add('listening');
                        assignBtnBtn.textContent = 'Press...';
                    } else {
                        alert('No gamepad detected.');
                    }
                });
                assignBtnBtn._origText = 'Btn';
                controls.appendChild(assignBtnBtn);

                // Assign Axis (detect axis movement)
                const assignAxisBtn = document.createElement('button');
                assignAxisBtn.className = 'assign-btn';
                assignAxisBtn.textContent = 'Axis';
                assignAxisBtn.title = 'Assign to an axis/channel';
                assignAxisBtn.addEventListener('click', () => {
                    if (this._listenAction === bAction) {
                        this.cancelListening();
                        assignAxisBtn.classList.remove('listening');
                        assignAxisBtn.textContent = 'Axis';
                        return;
                    }
                    this.cancelListening();
                    this.cancelButtonListening();
                    document.querySelectorAll('.assign-btn.listening').forEach(b => {
                        b.classList.remove('listening');
                        b.textContent = b._origText || 'Assign';
                    });

                    const started = this.startListening(bAction, (a, axisIdx, inverted) => {
                        assignAxisBtn.classList.remove('listening');
                        assignAxisBtn.textContent = 'Axis';
                        bm.source = 'axis';
                        bm.axisIndex = axisIdx;
                        srcLabel.textContent = `Axis ${axisIdx}`;
                        this._saveConfig();
                        this._buildSettingsUI();
                    });
                    if (started) {
                        assignAxisBtn.classList.add('listening');
                        assignAxisBtn.textContent = 'Move...';
                    } else {
                        alert('No gamepad detected.');
                    }
                });
                assignAxisBtn._origText = 'Axis';
                controls.appendChild(assignAxisBtn);

                // None button (unassign)
                const noneBtnB = document.createElement('button');
                noneBtnB.className = 'assign-btn';
                noneBtnB.textContent = 'None';
                noneBtnB.style.cssText = 'font-size:11px;padding:2px 6px;';
                noneBtnB.addEventListener('click', () => {
                    this.cancelButtonListening();
                    this.cancelListening();
                    bm.source = 'button';
                    bm.buttonIndex = -1;
                    bm.axisIndex = -1;
                    srcLabel.textContent = 'None';
                    this._saveConfig();
                    this._buildSettingsUI();
                });
                controls.appendChild(noneBtnB);

                // Threshold slider (for axis mode)
                if (bm.source === 'axis') {
                    const thLabel = document.createElement('span');
                    thLabel.style.cssText = 'font-size:11px;color:#888;margin-left:4px;';
                    thLabel.textContent = 'TH';
                    controls.appendChild(thLabel);
                    const thSlider = document.createElement('input');
                    thSlider.type = 'range';
                    thSlider.min = '0.1'; thSlider.max = '0.9'; thSlider.step = '0.05';
                    thSlider.value = bm.axisThreshold;
                    thSlider.style.width = '50px';
                    const thVal = document.createElement('span');
                    thVal.className = 'deadzone-val';
                    thVal.textContent = bm.axisThreshold.toFixed(2);
                    thSlider.addEventListener('input', () => {
                        bm.axisThreshold = parseFloat(thSlider.value);
                        thVal.textContent = parseFloat(thSlider.value).toFixed(2);
                        this._saveConfig();
                    });
                    controls.appendChild(thSlider);
                    controls.appendChild(thVal);
                }

                row.appendChild(controls);
                btnContainer.appendChild(row);
            }
        }

        // Gamepad status and WebHID button
        const statusEl = document.getElementById('gamepad-status');
        if (statusEl) {
            let statusHtml = '';
            
            // Disable Gamepad API checkbox
            const disabledChecked = this._gamepadApiDisabled ? 'checked' : '';
            statusHtml += `<div style="margin-bottom:8px;">
                <label style="display:flex;align-items:center;gap:8px;cursor:pointer;">
                    <input type="checkbox" id="disable-gamepad-api" ${disabledChecked}>
                    <span style="color:#fa0;">Disable Gamepad API (for WebHID)</span>
                </label>
            </div>`;
            
            if (this._hidConnected) {
                statusHtml += `<span style="color:#4f4;">HID Connected: ${this._hidDeviceName}</span>` +
                    `<button id="disconnect-hid-btn" style="margin-left:12px;padding:4px 12px;background:#533;border:1px solid #f44;color:#f44;border-radius:4px;cursor:pointer;font-size:12px;">Disconnect</button>`;
            } else if (this.connected && !this._gamepadApiDisabled) {
                statusHtml += `<span style="color:#4af;">Gamepad: ${this.gamepadName}</span>`;
            } else if (this._gamepadApiDisabled) {
                statusHtml += `<span style="color:#fa0;">Gamepad API disabled - use WebHID</span>`;
            } else {
                statusHtml += `<span style="color:#888;">No gamepad detected</span>`;
            }
            
            // Always show Connect HID button if not HID connected
            if (!this._hidConnected) {
                statusHtml += `<button id="connect-hid-btn" style="margin-left:12px;padding:4px 12px;background:#335;border:1px solid #4f4;color:#4f4;border-radius:4px;cursor:pointer;font-size:12px;">Connect HID</button>`;
            }

            // Calibration status + buttons (only when HID connected)
            if (this._hidConnected) {
                const calStatus = this._hasCalibration()
                    ? `<span style="color:#4f4;font-size:12px;">✓ Calibrated</span>`
                    : `<span style="color:#fa0;font-size:12px;">⚠ Not calibrated (using defaults)</span>`;
                statusHtml += `<div style="margin-top:10px;display:flex;align-items:center;gap:8px;flex-wrap:wrap;">` +
                    calStatus +
                    `<button id="calibrate-hid-btn" style="padding:4px 12px;background:#223;border:1px solid #4272F5;color:#4272F5;border-radius:4px;cursor:pointer;font-size:12px;">Calibrate…</button>` +
                    (this._hasCalibration() ? `<button id="clear-cal-btn" style="padding:4px 10px;background:transparent;border:1px solid #555;color:#888;border-radius:4px;cursor:pointer;font-size:11px;">Clear Cal</button>` : '') +
                    `</div>`;
            }
            
            statusEl.innerHTML = statusHtml;
            
            // Bind disable checkbox
            const disableCheckbox = document.getElementById('disable-gamepad-api');
            if (disableCheckbox) {
                disableCheckbox.addEventListener('change', (e) => {
                    this._gamepadApiDisabled = e.target.checked;
                    this._buildSettingsUI();
                });
            }
            
            // Bind HID buttons
            const connectBtn = document.getElementById('connect-hid-btn');
            if (connectBtn) {
                connectBtn.addEventListener('click', () => this.connectHID());
            }
            const disconnectBtn = document.getElementById('disconnect-hid-btn');
            if (disconnectBtn) {
                disconnectBtn.addEventListener('click', () => this.disconnectHID());
            }
            const calibrateBtn = document.getElementById('calibrate-hid-btn');
            if (calibrateBtn) {
                calibrateBtn.addEventListener('click', () => this.startCalibration());
            }
            const clearCalBtn = document.getElementById('clear-cal-btn');
            if (clearCalBtn) {
                clearCalBtn.addEventListener('click', () => this.clearCalibration());
            }
        }

        // Settings panel buttons
        this._setupSettingsButtons();
    }

    _buildRatesExpoUI() {
        const container = document.getElementById('rates-expo');
        if (!container) return;
        container.innerHTML = '';

        const RATE_EXPO_AXES = ['roll', 'pitch', 'throttle', 'yaw'];
        const RATE_AXES = ['roll', 'pitch', 'yaw']; // throttle has no rate
        const AXIS_COLORS = {
            roll: '#4af', pitch: '#f44', throttle: '#4f4', yaw: '#fa4'
        };

        for (const action of RATE_EXPO_AXES) {
            const m = this.mapping[action];
            // Ensure rate/expo exist (migration from old config)
            if (m.rate === undefined) m.rate = 1.0;
            if (m.expo === undefined) m.expo = 0.0;

            const row = document.createElement('div');
            row.className = 'setting-row';
            row.style.flexWrap = 'wrap';

            const label = document.createElement('label');
            label.style.minWidth = '80px';
            const dot = document.createElement('span');
            dot.style.cssText = `display:inline-block;width:8px;height:8px;border-radius:50%;background:${AXIS_COLORS[action]};margin-right:6px;`;
            label.appendChild(dot);
            label.appendChild(document.createTextNode(action.charAt(0).toUpperCase() + action.slice(1)));
            row.appendChild(label);

            const controls = document.createElement('div');
            controls.className = 'controls';

            // Rate slider + number input (only for roll/pitch/yaw)
            if (RATE_AXES.includes(action)) {
                const rateLabel = document.createElement('span');
                rateLabel.style.cssText = 'font-size:11px;color:#888;';
                rateLabel.textContent = 'Rate';
                controls.appendChild(rateLabel);
                const rateSlider = document.createElement('input');
                rateSlider.type = 'range';
                rateSlider.min = '0'; rateSlider.max = '10'; rateSlider.step = '0.1';
                rateSlider.value = m.rate;
                rateSlider.style.width = '70px';
                const rateNum = document.createElement('input');
                rateNum.type = 'number';
                rateNum.min = '0'; rateNum.max = '10'; rateNum.step = '0.1';
                rateNum.value = m.rate;
                rateNum.style.cssText = 'width:48px;background:#223;color:#ddd;border:1px solid #446;border-radius:3px;padding:2px 4px;font-size:11px;';
                rateSlider.addEventListener('input', () => {
                    const v = parseFloat(rateSlider.value);
                    this.mapping[action].rate = v;
                    rateNum.value = v;
                    this._saveConfig();
                });
                rateNum.addEventListener('change', () => {
                    const v = Math.max(0, Math.min(10, parseFloat(rateNum.value) || 0));
                    this.mapping[action].rate = v;
                    rateSlider.value = v;
                    rateNum.value = v;
                    this._saveConfig();
                });
                controls.appendChild(rateSlider);
                controls.appendChild(rateNum);
            }

            // Expo slider + number input
            const expoLabel = document.createElement('span');
            expoLabel.style.cssText = `font-size:11px;color:#888;${RATE_AXES.includes(action) ? 'margin-left:8px;' : ''}`;
            expoLabel.textContent = 'Expo';
            controls.appendChild(expoLabel);
            const expoSlider = document.createElement('input');
            expoSlider.type = 'range';
            expoSlider.min = '0'; expoSlider.max = '1.0'; expoSlider.step = '0.05';
            expoSlider.value = m.expo;
            expoSlider.style.width = '70px';
            const expoNum = document.createElement('input');
            expoNum.type = 'number';
            expoNum.min = '0'; expoNum.max = '1'; expoNum.step = '0.05';
            expoNum.value = m.expo;
            expoNum.style.cssText = 'width:48px;background:#223;color:#ddd;border:1px solid #446;border-radius:3px;padding:2px 4px;font-size:11px;';
            expoSlider.addEventListener('input', () => {
                const v = parseFloat(expoSlider.value);
                this.mapping[action].expo = v;
                expoNum.value = v;
                this._saveConfig();
                this._drawExpoCurve(action);
            });
            expoNum.addEventListener('change', () => {
                const v = Math.max(0, Math.min(1, parseFloat(expoNum.value) || 0));
                this.mapping[action].expo = v;
                expoSlider.value = v;
                expoNum.value = v;
                this._saveConfig();
                this._drawExpoCurve(action);
            });
            controls.appendChild(expoSlider);
            controls.appendChild(expoNum);

            row.appendChild(controls);
            container.appendChild(row);
        }

        // Create 4 canvases in the grid container
        const gridContainer = document.getElementById('expo-curves-container');
        if (gridContainer) {
            gridContainer.innerHTML = '';
            for (const action of RATE_EXPO_AXES) {
                const canvas = document.createElement('canvas');
                canvas.id = `expo-curve-${action}`;
                canvas.width = 220;
                canvas.height = 220;
                canvas.style.cssText = 'width:100%;background:#112;border:1px solid #334;border-radius:6px;';
                gridContainer.appendChild(canvas);
                this._drawExpoCurve(action);
            }
        }
    }

    _drawExpoCurve(action) {
        const AXIS_COLORS = {
            roll: '#4af', pitch: '#f44', throttle: '#4f4', yaw: '#fa4'
        };
        const canvas = document.getElementById(`expo-curve-${action}`);
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const W = canvas.width;
        const H = canvas.height;
        const pad = 24;
        const plotW = W - pad * 2;
        const plotH = H - pad * 2;

        ctx.clearRect(0, 0, W, H);

        // Title
        ctx.fillStyle = AXIS_COLORS[action] || '#4af';
        ctx.font = 'bold 11px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(action.charAt(0).toUpperCase() + action.slice(1), W / 2, 14);

        // Grid
        ctx.strokeStyle = '#334';
        ctx.lineWidth = 0.5;
        for (let i = 0; i <= 4; i++) {
            const x = pad + (plotW * i / 4);
            const y = pad + (plotH * i / 4);
            ctx.beginPath(); ctx.moveTo(x, pad); ctx.lineTo(x, pad + plotH); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(pad + plotW, y); ctx.stroke();
        }

        // Tick labels (-1, 0, 1)
        ctx.fillStyle = '#555';
        ctx.font = '9px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('-1', pad, pad + plotH + 12);
        ctx.fillText('0', pad + plotW / 2, pad + plotH + 12);
        ctx.fillText('1', pad + plotW, pad + plotH + 12);
        ctx.textAlign = 'right';
        ctx.fillText('-1', pad - 4, pad + plotH + 3);
        ctx.fillText('0', pad - 4, pad + plotH / 2 + 3);
        ctx.fillText('1', pad - 4, pad + 3);

        // Linear reference line (dashed)
        ctx.strokeStyle = '#444';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(pad, pad + plotH);
        ctx.lineTo(pad + plotW, pad);
        ctx.stroke();
        ctx.setLineDash([]);

        // Expo curve (pure expo, no rate scaling)
        const m = this.mapping[action];
        const expo = m.expo || 0;

        ctx.strokeStyle = AXIS_COLORS[action] || '#4af';
        ctx.lineWidth = 2;
        ctx.beginPath();
        for (let px = 0; px <= plotW; px++) {
            const input = (px / plotW) * 2 - 1; // -1..1
            const absIn = Math.abs(input);
            const output = Math.sign(input) * absIn * (1 - expo + expo * absIn * absIn);
            const x = pad + px;
            const y = pad + plotH / 2 - output * (plotH / 2);
            if (px === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
    }

    _updateGamepadDisplay(gp) {
        const el = document.getElementById('gamepad-axes-display');
        if (!el) return;

        let html = '';
        // Axes as bars
        const numAxes = Math.min(gp.axes.length, 8);
        for (let i = 0; i < numAxes; i++) {
            const val = gp.axes[i];
            const pct = ((val + 1) / 2) * 100; // -1..1 → 0..100%
            html += `<div style="display:flex;align-items:center;gap:4px;margin-bottom:2px;">` +
                `<span style="width:24px;text-align:right;color:#aaa;">A${i}</span>` +
                `<div style="flex:1;height:10px;background:#223;border-radius:3px;overflow:hidden;position:relative;">` +
                `<div style="position:absolute;left:50%;top:0;width:1px;height:100%;background:#444;"></div>` +
                `<div style="width:${pct}%;height:100%;background:#4af;border-radius:3px;transition:width 0.05s;"></div>` +
                `</div>` +
                `<span style="width:40px;text-align:right;font-size:10px;">${val.toFixed(2)}</span>` +
                `</div>`;
        }
        // Buttons as bars
        const numBtns = Math.min(gp.buttons.length, 16);
        for (let i = 0; i < numBtns; i++) {
            const val = gp.buttons[i].value;
            const pct = val * 100;
            const color = gp.buttons[i].pressed ? '#4af' : '#335';
            html += `<div style="display:flex;align-items:center;gap:4px;margin-bottom:2px;">` +
                `<span style="width:24px;text-align:right;color:#aaa;">B${i}</span>` +
                `<div style="flex:1;height:10px;background:#223;border-radius:3px;overflow:hidden;">` +
                `<div style="width:${pct}%;height:100%;background:${color};border-radius:3px;transition:width 0.05s;"></div>` +
                `</div>` +
                `<span style="width:40px;text-align:right;font-size:10px;">${val.toFixed(2)}</span>` +
                `</div>`;
        }
        el.innerHTML = html;
    }

    _updateHIDDisplay(hidAxes) {
        const el = document.getElementById('gamepad-axes-display');
        if (!el) return;

        let html = '<div style="color:#4f4;margin-bottom:8px;font-size:12px;">WebHID Mode - Full Precision</div>';
        // Show all HID channels with high precision
        const numChannels = Math.min(hidAxes.length, 8);
        for (let i = 0; i < numChannels; i++) {
            const val = hidAxes[i];
            // Calculate bar position: center is 50%, val -1 to 1 maps to 0% to 100%
            const pct = ((val + 1) / 2) * 100;
            // Color based on magnitude
            const mag = Math.abs(val);
            const barColor = mag > 0.01 ? '#4f4' : '#335';
            html += `<div style="display:flex;align-items:center;gap:4px;margin-bottom:3px;">` +
                `<span style="width:30px;text-align:right;color:#aaa;">CH${i + 1}</span>` +
                `<div style="flex:1;height:12px;background:#223;border-radius:3px;overflow:hidden;position:relative;">` +
                `<div style="position:absolute;left:50%;top:0;width:1px;height:100%;background:#555;"></div>` +
                `<div style="position:absolute;left:${Math.min(pct, 50)}%;width:${Math.abs(pct - 50)}%;height:100%;background:${barColor};"></div>` +
                `</div>` +
                `<span style="width:70px;text-align:right;font-size:11px;font-family:monospace;color:${mag > 0.01 ? '#4f4' : '#666'};">${val >= 0 ? '+' : ''}${val.toFixed(4)}</span>` +
                `</div>`;
        }
        el.innerHTML = html;
    }

    _setupSettingsButtons() {
        const closeBtn = document.getElementById('close-settings-btn');
        if (closeBtn && !closeBtn._bound) {
            closeBtn._bound = true;
            closeBtn.addEventListener('click', () => this._toggleSettings());
        }

        const gearBtn = document.getElementById('gear-btn');
        if (gearBtn && !gearBtn._bound) {
            gearBtn._bound = true;
            gearBtn.addEventListener('click', () => this._toggleSettings());
        }

        const exportBtn = document.getElementById('export-config-btn');
        if (exportBtn && !exportBtn._bound) {
            exportBtn._bound = true;
            exportBtn.addEventListener('click', () => {
                const config = this.getConfig();
                const blob = new Blob([JSON.stringify(config, null, 2)], { type: 'application/json' });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = 'drone_controller_config.json';
                a.click();
                URL.revokeObjectURL(url);
            });
        }

        const importBtn = document.getElementById('import-config-btn');
        const importInput = document.getElementById('import-config-input');
        if (importBtn && importInput && !importBtn._bound) {
            importBtn._bound = true;
            importBtn.addEventListener('click', () => importInput.click());
            importInput.addEventListener('change', (e) => {
                const file = e.target.files[0];
                if (!file) return;
                const reader = new FileReader();
                reader.onload = (ev) => {
                    try {
                        const config = JSON.parse(ev.target.result);
                        this.loadConfig(config);
                    } catch (err) {
                        alert('Invalid config file');
                    }
                };
                reader.readAsText(file);
            });
        }

        // Flight mode select — swap per-mode settings on change
        const modeSelect = document.getElementById('flight-mode-select');
        if (modeSelect && !modeSelect._bound) {
            modeSelect._bound = true;
            modeSelect.addEventListener('change', () => this._onModeSwitch(modeSelect.value));
        }

        // Physics slider+number pairs (bidirectional sync)
        this._bindSliderNum('phys-mass', 'phys-mass-num');
        this._bindSliderNum('phys-thrust', 'phys-thrust-num');
        this._bindSliderNum('phys-drag-cd', 'phys-drag-cd-num');
        this._bindSliderNum('phys-drag-area', 'phys-drag-area-num');
        this._bindSliderNum('phys-drone-size', 'phys-drone-size-num');
        this._bindSliderNum('phys-collision-radius', 'phys-collision-radius-num');
        this._bindSliderNum('cam-hfov', 'cam-hfov-num');
        this._bindSliderNum('cam-mount-angle', 'cam-mount-angle-num');

        // Controller gain sliders
        this._bindSliderNum('ctrl-pos-kp', 'ctrl-pos-kp-num');
        this._bindSliderNum('ctrl-pos-ki', 'ctrl-pos-ki-num');
        this._bindSliderNum('ctrl-vel-kp', 'ctrl-vel-kp-num');
        this._bindSliderNum('ctrl-vel-ki', 'ctrl-vel-ki-num');
        this._bindSliderNum('ctrl-alt-kp', 'ctrl-alt-kp-num');
        this._bindSliderNum('ctrl-alt-ki', 'ctrl-alt-ki-num');
        this._bindSliderNum('ctrl-pos-kd', 'ctrl-pos-kd-num');
        this._bindSliderNum('ctrl-vel-kd', 'ctrl-vel-kd-num');
        this._bindSliderNum('ctrl-alt-kd', 'ctrl-alt-kd-num');

        // Display toggle checkboxes
        for (const cbId of ['clean-mode-toggle', 'osd-toggle']) {
            const cb = document.getElementById(cbId);
            if (cb && !cb._bound) {
                cb._bound = true;
                cb.addEventListener('change', () => this._saveConfig());
            }
        }
    }

    _bindSliderNum(sliderId, numId) {
        const slider = document.getElementById(sliderId);
        const numEl = document.getElementById(numId);
        if (slider && numEl && !slider._boundSN) {
            slider._boundSN = true;
            slider.addEventListener('input', () => {
                numEl.value = slider.value;
                this._saveConfig();
            });
            numEl.addEventListener('input', () => {
                const min = parseFloat(slider.min);
                const max = parseFloat(slider.max);
                const v = Math.max(min, Math.min(max, parseFloat(numEl.value) || min));
                slider.value = v;
                numEl.value = v;
                this._saveConfig();
            });
        }
    }

    _bindPhysicsSlider(sliderId, valId) {
        const slider = document.getElementById(sliderId);
        const valEl = document.getElementById(valId);
        if (slider && valEl && !slider._bound) {
            slider._bound = true;
            slider.addEventListener('input', () => {
                valEl.textContent = parseFloat(slider.value).toFixed(slider.step.includes('.') ? 2 : 0);
                this._saveConfig();
            });
        }
    }

    _saveConfig() {
        try {
            localStorage.setItem('drone_sim_controller_config', JSON.stringify(this.getConfig()));
        } catch (e) { /* ignore */ }
    }

    _loadConfig() {
        try {
            const saved = localStorage.getItem('drone_sim_controller_config');
            if (saved) {
                const config = JSON.parse(saved);
                if (config.mapping) {
                    for (const action of ACTIONS) {
                        if (config.mapping[action]) {
                            this.mapping[action] = { ...this.mapping[action], ...config.mapping[action] };
                        }
                    }
                }
                if (config.buttonMapping) {
                    for (const bAction of BUTTON_ACTIONS) {
                        if (config.buttonMapping[bAction]) {
                            this.buttonMapping[bAction] = { ...this.buttonMapping[bAction], ...config.buttonMapping[bAction] };
                            delete this.buttonMapping[bAction].inverted; // legacy cleanup
                        }
                    }
                }
                if (config.hidCalibration) {
                    this._hidCalibration = config.hidCalibration;
                }
                if (config.modeRateExpo) {
                    this._modeRateExpo = config.modeRateExpo;
                }
                if (config.modePidSettings) {
                    this._modePidSettings = config.modePidSettings;
                }
                if (config.currentMode) {
                    this._currentMode = config.currentMode;
                }
                if (config.settings) {
                    this._restoreSettings(config.settings);
                }
            }
        } catch (e) { /* ignore */ }
    }
}
