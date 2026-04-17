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
 * EngineAudio — sample-playback FPV drone motor sound.
 *
 * Loads a loop-friendly WAV (asset/fpv_loop.wav — checked into the repo as a
 * runtime asset) and modulates its playbackRate + master gain as a function
 * of normalized throttle. Because the file is already trimmed and cross-faded,
 * runtime processing is just "load, loop, play".
 *
 * The loop WAV is regenerated manually from asset/fpv.wav by running
 * `python3 tools/prep_audio.py --flatten` — see that script for options.
 * No build step is involved at launch time.
 *
 * Public API (stable, must not break — main.js depends on it):
 *   new EngineAudio(url?)    construct; no audio work until a user gesture
 *   resume()                 call once inside a user-gesture handler
 *   update(t, armed)         per-frame; t in [0,1], armed boolean
 *   setMuted(bool)           hard mute toggle
 */

// Default clip location, relative to index.html.
const DEFAULT_URL = 'asset/fpv_loop.wav';

// Playback-rate range. 1.0 = native pitch/speed of the sample.
// 0.7 ≈ minor-3rd below native, 1.5 ≈ perfect-5th above. Tune to taste.
const RATE_IDLE = 0.7;
const RATE_FULL = 1.5;

// Master gain per state. WAV samples are usually normalised near 0 dBFS,
// so keep gains well below 1.0 to avoid clipping.
const GAIN_IDLE     = 0.18;
const GAIN_FULL     = 0.70;
const GAIN_DISARMED = 0.0;

// Time constant (seconds) for setTargetAtTime ramps. Larger = more sluggish.
const SMOOTH_TAU = 0.08;

export class EngineAudio {
    constructor(url = DEFAULT_URL) {
        this.url = url;
        this.ctx = null;
        this.started = false;   // user gesture happened; AudioContext created
        this.ready = false;     // buffer decoded, source playing
        this.muted = false;

        this._master = null;
        this._buffer = null;
        this._source = null;
        this._lastThrottle = -1;
        this._lastArmed = null;
    }

    /** Create AudioContext and kick off the fetch. Call from a user gesture. */
    start() {
        if (this.started) return;
        const Ctx = window.AudioContext || window.webkitAudioContext;
        if (!Ctx) {
            console.warn('[EngineAudio] Web Audio API not available');
            return;
        }
        try {
            this.ctx = new Ctx();
        } catch (e) {
            console.warn('[EngineAudio] could not create AudioContext:', e);
            return;
        }
        this.started = true;

        // Master gain starts silent; update() will ramp it once armed.
        this._master = this.ctx.createGain();
        this._master.gain.value = GAIN_DISARMED;
        this._master.connect(this.ctx.destination);

        // Kick off the asynchronous fetch+decode.
        this._load().catch(err => {
            console.warn(
                `[EngineAudio] failed to load ${this.url} — did you run ` +
                `tools/prep_audio.py? Error:`, err
            );
        });
    }

    /** Resume the AudioContext if suspended (browser autoplay policy). */
    resume() {
        if (!this.started) this.start();
        if (this.ctx && this.ctx.state === 'suspended') {
            this.ctx.resume().catch(() => { /* ignore */ });
        }
    }

    setMuted(muted) {
        this.muted = !!muted;
    }

    /**
     * Update sound parameters.
     * @param {number} throttle01  normalized thrust in [0, 1]
     * @param {boolean} armed      whether the drone is armed / motors spinning
     */
    update(throttle01, armed) {
        if (!this.started || !this.ctx) return;
        const t = Math.max(0, Math.min(1, throttle01 || 0));
        // Skip redundant scheduling when nothing changed meaningfully.
        if (Math.abs(t - this._lastThrottle) < 0.002 && armed === this._lastArmed) return;
        this._lastThrottle = t;
        this._lastArmed = armed;

        const now = this.ctx.currentTime;

        // Master gain envelope.
        const targetGain = (this.muted || !armed)
            ? GAIN_DISARMED
            : GAIN_IDLE + (GAIN_FULL - GAIN_IDLE) * t;
        this._master.gain.setTargetAtTime(targetGain, now, SMOOTH_TAU);

        // Playback rate (pitch + tempo).
        if (this.ready && this._source) {
            const rate = RATE_IDLE + (RATE_FULL - RATE_IDLE) * t;
            this._source.playbackRate.setTargetAtTime(rate, now, SMOOTH_TAU);
        }
    }

    // ---- internal ----

    async _load() {
        // force-cache keeps subsequent reloads instant.
        const resp = await fetch(this.url, { cache: 'force-cache' });
        if (!resp.ok) throw new Error(`HTTP ${resp.status} fetching ${this.url}`);
        const data = await resp.arrayBuffer();
        // decodeAudioData handles WAV / MP3 / OGG / FLAC / M4A natively.
        const buffer = await this.ctx.decodeAudioData(data);
        this._buffer = buffer;
        console.info(
            `[EngineAudio] loaded ${this.url} — ${buffer.duration.toFixed(2)}s ` +
            `${buffer.numberOfChannels}ch ${buffer.sampleRate}Hz`
        );

        const src = this.ctx.createBufferSource();
        src.buffer = buffer;
        src.loop = true;
        src.playbackRate.value = RATE_IDLE;
        src.connect(this._master);
        src.start(0);
        this._source = src;
        this.ready = true;
    }
}
