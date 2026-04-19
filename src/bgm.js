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
 * BgmAudio — playlist-based background music player.
 *
 * One AudioContext, one _userGain node, one-at-a-time AudioBufferSource for
 * whichever track is currently playing. Tracks within a playlist are played
 * sequentially; when the last track ends the playlist is reshuffled and looped.
 * Switching playlists fades the current track out and starts the new one from
 * the top.
 *
 * Tracks are fetched and decoded lazily, cached by URL. The next track in the
 * queue is prefetched while the current one plays so the inter-track gap is
 * usually imperceptible.
 *
 * Signal chain:
 *   source (current track) → _userGain → destination
 *
 * Public API:
 *   new BgmAudio()              construct; no audio work until a user gesture
 *   start()                     create AudioContext (call from a user gesture)
 *   resume()                    resume a suspended AudioContext
 *   registerPlaylist(name, urls)   define a named list of track URLs
 *   playPlaylist(name)          switch to playlist `name`; fades if needed
 *   stopPlaylist()              fade out and stop
 *   setVolume(v)                user volume in [0, 1]
 *   getVolume()                 current user volume
 *   setMuted(bool)              hard mute toggle (orthogonal to volume)
 *   isMuted()                   current mute flag
 *   getCurrentPlaylist()        name of the active playlist, or null
 */

// Fade-in / fade-out time when switching between playlists or stopping.
const FADE_SEC = 0.6;

// Time constant used for volume-slider ramps — same feel as EngineAudio.
const SMOOTH_TAU = 0.08;

export class BgmAudio {
    constructor() {
        this.ctx = null;
        this.started = false;
        this.muted = false;
        this._userVolume = 0.5;  // sensible default for background music

        this._userGain = null;
        this._currentSource = null;
        this._currentUrl = null;

        // name → [url]
        this._playlists = new Map();
        // Active playlist state
        this._activeName = null;
        this._shuffled = [];       // current shuffle order
        this._shuffleIdx = 0;      // index into _shuffled

        // URL → AudioBuffer cache (populated on demand)
        this._bufferCache = new Map();
        // URL → Promise<AudioBuffer> in flight
        this._loadingPromises = new Map();

        // Monotonic counter used to cancel in-flight playlist switches when a
        // newer switch arrives while a fade-out is still running.
        this._switchToken = 0;
    }

    /** Create AudioContext. Call inside a user-gesture handler. */
    start() {
        if (this.started) return;
        const Ctx = window.AudioContext || window.webkitAudioContext;
        if (!Ctx) {
            console.warn('[BgmAudio] Web Audio API not available');
            return;
        }
        try {
            this.ctx = new Ctx();
        } catch (e) {
            console.warn('[BgmAudio] could not create AudioContext:', e);
            return;
        }
        this.started = true;

        this._userGain = this.ctx.createGain();
        this._userGain.gain.value = this.muted ? 0 : this._userVolume;
        this._userGain.connect(this.ctx.destination);

        // If a playlist was queued before the gesture, start it now.
        if (this._activeName) this._beginPlaylist(this._activeName);
    }

    resume() {
        if (!this.started) this.start();
        if (this.ctx && this.ctx.state === 'suspended') {
            this.ctx.resume().catch(() => { /* ignore */ });
        }
    }

    setVolume(v) {
        const clamped = Math.max(0, Math.min(1, Number(v) || 0));
        this._userVolume = clamped;
        this._applyUserGain();
    }

    getVolume() { return this._userVolume; }

    setMuted(muted) {
        this.muted = !!muted;
        this._applyUserGain();
    }

    isMuted() { return this.muted; }

    registerPlaylist(name, urls) {
        if (!name || !Array.isArray(urls) || urls.length === 0) return;
        this._playlists.set(name, urls.slice());
    }

    getCurrentPlaylist() { return this._activeName; }

    /**
     * Switch to the named playlist. If the same playlist is already playing,
     * this is a no-op. Otherwise the current track fades out and the first
     * track of the new playlist starts.
     */
    playPlaylist(name) {
        if (!this._playlists.has(name)) {
            console.warn(`[BgmAudio] unknown playlist: ${name}`);
            return;
        }
        if (this._activeName === name && this._currentSource) return;

        this._activeName = name;
        // If we have no AudioContext yet, _beginPlaylist() will be invoked
        // from start() once a user gesture arrives.
        if (!this.started) return;
        this._beginPlaylist(name);
    }

    /** Fade out and stop. Next playPlaylist() starts from a clean state. */
    stopPlaylist() {
        this._activeName = null;
        this._switchToken++;
        this._fadeOutAndStop(this._currentSource, FADE_SEC);
        this._currentSource = null;
        this._currentUrl = null;
    }

    // ---- internal ----

    _applyUserGain() {
        if (!this._userGain || !this.ctx) return;
        const target = this.muted ? 0 : this._userVolume;
        this._userGain.gain.setTargetAtTime(target, this.ctx.currentTime, SMOOTH_TAU);
    }

    _beginPlaylist(name) {
        const token = ++this._switchToken;
        const fadingOld = this._currentSource;
        const fadeSec = fadingOld ? FADE_SEC : 0;
        this._fadeOutAndStop(fadingOld, fadeSec);
        this._currentSource = null;
        this._currentUrl = null;

        // Build shuffle order for this session.
        const urls = this._playlists.get(name);
        this._shuffled = _shuffle(urls);
        this._shuffleIdx = 0;

        // Kick off the first track after the old one has faded.
        const delayMs = Math.round(fadeSec * 1000);
        setTimeout(() => {
            if (token !== this._switchToken) return;  // a newer switch won
            this._playNext();
        }, delayMs);
    }

    _playNext() {
        if (!this._activeName || !this._shuffled.length) return;
        if (this._shuffleIdx >= this._shuffled.length) {
            // Reshuffle, but try to avoid immediately repeating the last
            // track we played (common when the playlist is short).
            const lastUrl = this._shuffled[this._shuffled.length - 1];
            this._shuffled = _shuffle(this._playlists.get(this._activeName));
            if (this._shuffled.length > 1 && this._shuffled[0] === lastUrl) {
                const swap = this._shuffled[1];
                this._shuffled[1] = this._shuffled[0];
                this._shuffled[0] = swap;
            }
            this._shuffleIdx = 0;
        }

        const url = this._shuffled[this._shuffleIdx++];
        const token = this._switchToken;

        this._loadBuffer(url).then(buf => {
            // A playlist switch may have happened while the fetch was in flight.
            if (token !== this._switchToken || !this._userGain || !this.ctx) return;
            this._startSource(url, buf);

            // Prefetch the next track so the inter-track gap is minimal.
            const nextIdx = this._shuffleIdx < this._shuffled.length
                ? this._shuffleIdx
                : 0;
            const nextName = this._activeName && this._playlists.has(this._activeName)
                ? this._playlists.get(this._activeName)
                : null;
            const nextUrl = nextName
                ? (this._shuffled[nextIdx] || nextName[0])
                : null;
            if (nextUrl && !this._bufferCache.has(nextUrl)) {
                // Fire and forget; errors are swallowed by _loadBuffer.
                this._loadBuffer(nextUrl).catch(() => {});
            }
        }).catch(err => {
            console.warn(`[BgmAudio] failed to load ${url}:`, err);
            // Skip this track and try the next one.
            if (token === this._switchToken) this._playNext();
        });
    }

    _startSource(url, buffer) {
        const src = this.ctx.createBufferSource();
        src.buffer = buffer;
        src.loop = false;
        src.connect(this._userGain);

        // Fade in over a short window so the transition from a just-finished
        // track (or from silence on first start) is smooth.
        const now = this.ctx.currentTime;
        this._userGain.gain.cancelScheduledValues(now);
        this._userGain.gain.setValueAtTime(0, now);
        this._userGain.gain.linearRampToValueAtTime(
            this.muted ? 0 : this._userVolume,
            now + FADE_SEC,
        );

        const token = this._switchToken;
        src.onended = () => {
            // Only advance if we are still the active source for the same
            // session — otherwise a fade-out has already handled cleanup.
            if (token !== this._switchToken) return;
            if (this._currentSource !== src) return;
            this._currentSource = null;
            this._currentUrl = null;
            this._playNext();
        };

        src.start(0);
        this._currentSource = src;
        this._currentUrl = url;

        const secs = buffer.duration.toFixed(1);
        console.info(`[BgmAudio] playing ${url} (${secs}s)`);
    }

    _fadeOutAndStop(src, fadeSec) {
        if (!src || !this._userGain || !this.ctx) return;
        const now = this.ctx.currentTime;
        try {
            this._userGain.gain.cancelScheduledValues(now);
            this._userGain.gain.setValueAtTime(this._userGain.gain.value, now);
            this._userGain.gain.linearRampToValueAtTime(0, now + fadeSec);
            // Detach onended so the fade doesn't trigger _playNext.
            src.onended = null;
            src.stop(now + fadeSec + 0.02);
        } catch (e) {
            // Source may already have stopped.
        }
    }

    async _loadBuffer(url) {
        const cached = this._bufferCache.get(url);
        if (cached) return cached;

        const inflight = this._loadingPromises.get(url);
        if (inflight) return inflight;

        const p = (async () => {
            const resp = await fetch(url, { cache: 'force-cache' });
            if (!resp.ok) throw new Error(`HTTP ${resp.status} fetching ${url}`);
            const data = await resp.arrayBuffer();
            const buf = await this.ctx.decodeAudioData(data);
            this._bufferCache.set(url, buf);
            this._loadingPromises.delete(url);
            return buf;
        })();

        this._loadingPromises.set(url, p);
        return p;
    }
}

// Fisher-Yates; returns a new array without mutating the input.
function _shuffle(arr) {
    const out = arr.slice();
    for (let i = out.length - 1; i > 0; i--) {
        const j = Math.floor(Math.random() * (i + 1));
        [out[i], out[j]] = [out[j], out[i]];
    }
    return out;
}
