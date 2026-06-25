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
 * Main entry point — PlayCanvas app initialization, GSplat loading (.ply/.sog/.splat), game loop.
 */

import { parsePlyForPositions, parsePlySceneData, analyzePlyDistances, countFilteredPoints, sanitizePlyBuffer } from './ply-parser.js';
import { parseSplatForPositions, parseSplatOpacities, parseSplatRawCentroid, splatToPlyBuffer } from './splat-parser.js';
import { parseSogForPositions, parseSogOpacities, parseSogRawCentroid } from './sog-parser.js';
import { Octree } from './collision.js';
import { Controller } from './controller.js';
import { Drone } from './drone.js';
import { HUD } from './hud.js';
import { OSD } from './osd.js';
import { EngineAudio } from './audio.js';
import { BgmAudio } from './bgm.js';
import { GateCourse } from './gates.js';
import * as pathStore from './path-store.js';

// ---- Globals ----
let app = null;
let cameraEntity = null;
let drone = null;
let controller = null;
let hud = null;
let osd = null;
let engineAudio = null;
let bgmAudio = null;
let octree = null;
let sceneLoaded = false;
let pcInitialized = false;

// Race-course subsystem. Constructed at module init (same as controller) so
// the settings panel works before a scene is loaded; entities materialise
// only when `rebuildGateCourse` fires after a filter commit, and they stay
// hidden until the user presses G in flight mode (see gateMode below).
let gateCourse = null;
let sceneFilteredBounds = null; // { min:[x,y,z], max:[x,y,z] } — set after filter commit

// The File object the user dropped; needed by path-store.js to build the
// per-scene storage key (filename + size).  Reset on _exitToLoading.
let currentSceneFile = null;

// G-key toggle — always starts `false` on a new scene load (per spec);
// flipping via G in flight mode shows/hides the whole race course. We
// also use this as the gate for whether lap timing is engaged.
let gateMode = false;


// Mode: 'loading' | 'placement' | 'flight'
let mode = 'loading';

// Orbit camera state (placement mode)
let orbitYaw = 0;
let orbitPitch = -30;
let orbitDistance = 10;
let orbitTarget = { x: 0, y: 0, z: 0 };
let isDragging = false;
let dragButton = -1;
let lastMouseX = 0;
let lastMouseY = 0;
let orbitKeysDown = new Set();

// Spawn point marker
let spawnMarkerEntity = null;
let spawnPoint = null;
let sceneBounds = null;

// Coordinate system
let coordSystem = 'zup'; // 'zup' or 'yup'
let cachedArrayBuffer = null;
let cachedFilename = null;
let cachedFormat = 'ply'; // 'ply', 'sog', or 'splat'
let rawArrayBuffer = null; // unfiltered original buffer

// File System Access API state. When available (Chromium on https/localhost)
// we use showOpenFilePicker with `startIn: _lastFileHandle` so that the open
// dialog re-opens at the directory of the previously-picked scene file and
// highlights it — this survives ESC → re-pick cycles, which the legacy
// <input type=file> element does not reliably preserve. Browsers without
// File System Access API fall back to the legacy input in setupFileLoading().
let _lastFileHandle = null;

function showError(msg) {
    console.error(msg);
    const el = document.getElementById('loading-progress');
    if (el) { el.textContent = msg; el.style.color = '#f44'; }
}

// Allow user to disable engine audio entirely via URL param ?noaudio=1 for
// debugging or if any Web Audio glitch ever causes trouble. ?nobgm=1
// independently silences the background music.
const _engineAudioDisabled = (() => {
    try { return new URLSearchParams(window.location.search).has('noaudio'); }
    catch { return false; }
})();
const _bgmAudioDisabled = (() => {
    try { return new URLSearchParams(window.location.search).has('nobgm'); }
    catch { return false; }
})();

// BGM playlists are discovered at runtime from the subdirectories of
// asset/music/ (see _discoverPlaylist / _loadBgmPlaylists below).
//   asset/music/init/   → 'init'   playlist (loading / filtering / placement)
//   asset/music/flight/ → 'flight' playlist (in-flight shuffle)
// Drop a .flac / .mp3 / .ogg / .wav into either folder and it will be
// picked up automatically. See scripts/gen-bgm-manifests.py for keeping
// manifest.json in sync on static hosts that don't serve directory listings.
const BGM_FOLDERS = {
    init:   'asset/music/init/',
    flight: 'asset/music/flight/',
};
const BGM_AUDIO_EXT_RE = /\.(flac|mp3|ogg|wav|m4a)$/i;

// Browsers block AudioContext.start() until a user gesture (autoplay policy).
// Attach one-shot listeners that resume both audio contexts on the first user
// interaction of any kind — click, tap, or key press — so BGM starts as soon
// as the user clicks "Choose File" (or drops a file, or presses any key).
// The listeners only call resume() and remove themselves; they don't call
// preventDefault / stopPropagation, so they don't consume transient user
// activation and won't interfere with subsequent UI actions like the file
// picker dialog or navigator.hid.requestDevice().
function _installAudioGestureHook() {
    const resume = () => {
        try { if (engineAudio) engineAudio.resume(); }
        catch (e) { console.warn('[EngineAudio] resume failed:', e); }
        try { if (bgmAudio) bgmAudio.resume(); }
        catch (e) { console.warn('[BgmAudio] resume failed:', e); }
        window.removeEventListener('keydown',    resume, true);
        window.removeEventListener('pointerdown', resume, true);
        window.removeEventListener('touchstart',  resume, true);
    };
    window.addEventListener('keydown',    resume, true);
    window.addEventListener('pointerdown', resume, true);
    window.addEventListener('touchstart',  resume, true);
}

// Switch the active BGM playlist for a given game mode. Safe to call when the
// BGM subsystem is disabled or uninitialised. If the playlist has not yet been
// registered (discovery still in flight on first load), BgmAudio logs a warn
// and no-ops; _loadBgmPlaylists re-invokes this after registration completes.
function _bgmForMode(modeName) {
    if (!bgmAudio) return;
    const playlist = modeName === 'flight' ? 'flight' : 'init';
    try { bgmAudio.playPlaylist(playlist); }
    catch (e) { console.warn('[BgmAudio] playPlaylist failed:', e); }
}

// Discover all audio tracks in a folder. Tries two strategies in order:
//   1) HTTP directory listing (e.g. Python http.server, node http-server).
//      Most dev servers return an HTML page with <a href="..."> entries for
//      every file; we parse those and keep the ones with an audio extension.
//   2) manifest.json fallback ({"tracks": ["a.flac", ...]}) — works on any
//      static host, including file://, GitHub Pages, Netlify. Regenerate via
//      scripts/gen-bgm-manifests.py after adding / removing tracks.
// Returns an array of fully-qualified URLs (relative to index.html) ready
// for BgmAudio.registerPlaylist().
async function _discoverPlaylist(folderUrl) {
    if (!folderUrl.endsWith('/')) folderUrl += '/';

    // ---- Strategy 1: directory listing ----
    try {
        const r = await fetch(folderUrl, { headers: { Accept: 'text/html' } });
        const ct = r.headers.get('content-type') || '';
        if (r.ok && ct.toLowerCase().includes('html')) {
            const html = await r.text();
            const doc = new DOMParser().parseFromString(html, 'text/html');
            const hrefs = [...doc.querySelectorAll('a[href]')]
                .map(a => a.getAttribute('href'))
                .filter(h => h && BGM_AUDIO_EXT_RE.test(h));
            // Extract bare filenames (strip leading slashes or absolute URLs
            // so the result is always relative to folderUrl).
            const base = new URL(folderUrl, window.location.href);
            const names = hrefs
                .map(h => {
                    try {
                        const u = new URL(h, base);
                        // Only keep entries whose resolved URL sits under base.
                        if (!u.pathname.startsWith(base.pathname)) return null;
                        return decodeURIComponent(u.pathname.slice(base.pathname.length));
                    } catch { return null; }
                })
                .filter(n => n && BGM_AUDIO_EXT_RE.test(n));
            const unique = [...new Set(names)];
            if (unique.length) return unique.map(n => folderUrl + n);
        }
    } catch (_) { /* fall through to manifest */ }

    // ---- Strategy 2: manifest.json ----
    try {
        const r = await fetch(folderUrl + 'manifest.json', { cache: 'no-cache' });
        if (r.ok) {
            const m = await r.json();
            if (m && Array.isArray(m.tracks)) {
                const tracks = m.tracks.filter(t => typeof t === 'string' && BGM_AUDIO_EXT_RE.test(t));
                if (tracks.length) return tracks.map(t => folderUrl + t);
            }
        }
    } catch (_) { /* nothing else to try */ }

    return [];
}

// Kick off discovery for every configured playlist folder and register the
// results on bgmAudio. Fire-and-forget at startup; once complete, replays the
// current mode's playlist so the first playPlaylist() call (issued before
// discovery resolved) actually starts audio.
async function _loadBgmPlaylists() {
    if (!bgmAudio || _bgmAudioDisabled) return;
    const names = Object.keys(BGM_FOLDERS);
    const lists = await Promise.all(names.map(n => _discoverPlaylist(BGM_FOLDERS[n])));
    names.forEach((name, i) => {
        const urls = lists[i];
        if (urls.length === 0) {
            console.warn(`[BgmAudio] no tracks found for playlist "${name}" in ${BGM_FOLDERS[name]}`);
            return;
        }
        bgmAudio.registerPlaylist(name, urls);
        console.info(`[BgmAudio] playlist "${name}": ${urls.length} track(s)`);
    });
    _bgmForMode(mode);
}

// ---- Initialize PlayCanvas (called once, before first PLY load) ----
function initPlayCanvas() {
    if (pcInitialized) return;

    const canvas = document.getElementById('app-canvas');

    app = new pc.Application(canvas, {
        mouse: new pc.Mouse(canvas),
        keyboard: new pc.Keyboard(window),
        graphicsDeviceOptions: {
            alpha: false,
            antialias: false,
            powerPreference: 'high-performance',
            preserveDrawingBuffer: false,
        },
    });

    app.setCanvasFillMode(pc.FILLMODE_FILL_WINDOW);
    app.setCanvasResolution(pc.RESOLUTION_AUTO);
    window.addEventListener('resize', () => app.resizeCanvas());

    // Create camera
    cameraEntity = new pc.Entity('camera');
    cameraEntity.addComponent('camera', {
        clearColor: new pc.Color(0, 0, 0),
        farClip: 1000,
        nearClip: 0.05,
        fov: 90,
    });
    app.root.addChild(cameraEntity);

    // Create a simple directional light
    const light = new pc.Entity('light');
    light.addComponent('light', {
        type: 'directional',
        color: new pc.Color(1, 1, 1),
        intensity: 1.0,
    });
    light.setEulerAngles(45, 30, 0);
    app.root.addChild(light);

    app.start();

    // Apply initial HFOV
    updateCameraFov();

    // Game loop
    app.on('update', (dt) => gameLoop(dt));

    // Setup orbit camera mouse/keyboard controls
    setupOrbitControls();

    pcInitialized = true;
    console.log('PlayCanvas initialized');
}

// ---- Orbit Camera (Placement Mode) ----
function setupOrbitControls() {
    const canvas = document.getElementById('app-canvas');

    canvas.addEventListener('mousedown', (e) => {
        if (mode !== 'placement' && mode !== 'filtering') return;
        isDragging = true;
        dragButton = e.button;
        lastMouseX = e.clientX;
        lastMouseY = e.clientY;
    });

    window.addEventListener('mousemove', (e) => {
        if ((mode !== 'placement' && mode !== 'filtering') || !isDragging) return;
        const dx = e.clientX - lastMouseX;
        const dy = e.clientY - lastMouseY;
        lastMouseX = e.clientX;
        lastMouseY = e.clientY;

        if (dragButton === 0) {
            // Left drag: orbit
            orbitYaw += dx * 0.15;
            orbitPitch = Math.max(-89, Math.min(89, orbitPitch - dy * 0.15));
        } else if (dragButton === 2 && mode !== 'filtering') {
            // Right drag: pan (disabled in filtering mode)
            const panSpeed = orbitDistance * 0.001;
            const yawRad = orbitYaw * Math.PI / 180;
            // Pan in camera-local horizontal and vertical
            orbitTarget.x -= (Math.cos(yawRad) * dx - Math.sin(yawRad) * 0) * panSpeed;
            orbitTarget.z -= (Math.sin(yawRad) * dx + Math.cos(yawRad) * 0) * panSpeed;
            orbitTarget.y += dy * panSpeed;
        }
    });

    window.addEventListener('mouseup', () => {
        isDragging = false;
        dragButton = -1;
    });

    canvas.addEventListener('wheel', (e) => {
        if (mode !== 'placement' && mode !== 'filtering') return;
        e.preventDefault();
        orbitDistance *= (1 + e.deltaY * 0.0005);
        orbitDistance = Math.max(0.5, Math.min(500, orbitDistance));
    }, { passive: false });

    // Click no longer places spawn — WASD/QE controls spawn position directly

    // Context menu disable
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());

    // Keyboard for placement/flight modes
    window.addEventListener('keydown', (e) => {
        // Esc with no settings panel open → exit scene (with confirmation)
        // (If settings panel was open, controller already consumed the event via stopImmediatePropagation)
        if (e.code === 'Escape' && sceneLoaded) {
            if (!confirm('Exit current scene?')) return;
            _exitToLoading();
            return;
        }

        if (mode === 'placement') {
            orbitKeysDown.add(e.code);
            if (e.code === 'Enter' && spawnPoint) {
                confirmSpawnAndFly();
            }
        } else if (mode === 'flight' && e.code === 'KeyP') {
            // Re-enter placement mode
            enterPlacementMode();
        } else if (mode === 'flight' && e.code === 'KeyG') {
            // Toggle gate-race mode. No-op when no path has been drawn
            // for this scene — print a console hint so the user knows
            // to open Settings → Gate Path → Edit Path… first.
            e.preventDefault();
            _toggleGateMode();
        }
    });
    window.addEventListener('keyup', (e) => {
        orbitKeysDown.delete(e.code);
    });
    window.addEventListener('blur', () => orbitKeysDown.clear());
}

function updateOrbitCamera(dt) {
    if (!cameraEntity) return;

    // WASD moves spawn point in placement mode (relative to camera view direction)
    if (mode === 'placement' && spawnPoint) {
        const moveSpeed = 3.0 * dt;
        const yawRad = orbitYaw * Math.PI / 180;
        if (orbitKeysDown.has('KeyW')) {
            spawnPoint.x -= Math.sin(yawRad) * moveSpeed;
            spawnPoint.z -= Math.cos(yawRad) * moveSpeed;
        }
        if (orbitKeysDown.has('KeyS')) {
            spawnPoint.x += Math.sin(yawRad) * moveSpeed;
            spawnPoint.z += Math.cos(yawRad) * moveSpeed;
        }
        if (orbitKeysDown.has('KeyA')) {
            spawnPoint.x -= Math.cos(yawRad) * moveSpeed;
            spawnPoint.z += Math.sin(yawRad) * moveSpeed;
        }
        if (orbitKeysDown.has('KeyD')) {
            spawnPoint.x += Math.cos(yawRad) * moveSpeed;
            spawnPoint.z -= Math.sin(yawRad) * moveSpeed;
        }
        // QE for height
        if (orbitKeysDown.has('KeyQ')) {
            spawnPoint.y -= moveSpeed;
        }
        if (orbitKeysDown.has('KeyE')) {
            spawnPoint.y += moveSpeed;
        }

        // Update orbit target to follow spawn point
        orbitTarget.x = spawnPoint.x;
        orbitTarget.y = spawnPoint.y;
        orbitTarget.z = spawnPoint.z;
        updateSpawnMarker(spawnPoint);
        _updateSpawnCoordsUI();
    }

    // Compute camera position on orbit sphere
    const yawRad = orbitYaw * Math.PI / 180;
    const pitchRad = orbitPitch * Math.PI / 180;
    const camX = orbitTarget.x + orbitDistance * Math.cos(pitchRad) * Math.sin(yawRad);
    const camY = orbitTarget.y + orbitDistance * Math.sin(pitchRad);
    const camZ = orbitTarget.z + orbitDistance * Math.cos(pitchRad) * Math.cos(yawRad);

    cameraEntity.setPosition(camX, camY, camZ);
    cameraEntity.lookAt(orbitTarget.x, orbitTarget.y, orbitTarget.z);
}

function placeSpawnAtScreenCenter() {
    // Raycast from screen center into the scene
    // Use the camera's forward direction and find the closest point in the octree
    if (!octree || !cameraEntity) return;

    const pos = cameraEntity.getPosition();
    const fwd = cameraEntity.forward;

    // March along the ray and find first point within threshold
    const maxDist = orbitDistance * 3;
    const step = 0.1;
    let bestDist = Infinity;
    let bestPoint = null;

    // Sample points along the ray
    for (let t = 0.5; t < maxDist; t += step) {
        const rx = pos.x + fwd.x * t;
        const ry = pos.y + fwd.y * t;
        const rz = pos.z + fwd.z * t;
        const results = octree.querySphere(rx, ry, rz, step * 2);
        if (results.length > 0) {
            bestPoint = { x: rx, y: ry, z: rz };
            break;
        }
    }

    if (!bestPoint) {
        // Fallback: place at orbit target
        bestPoint = { x: orbitTarget.x, y: orbitTarget.y, z: orbitTarget.z };
    }

    // Offset slightly above the hit point
    spawnPoint = { x: bestPoint.x, y: bestPoint.y + 1.0, z: bestPoint.z };

    // Update spawn marker (3D sphere)
    updateSpawnMarker(spawnPoint);

    // Update UI
    const coordsEl = document.getElementById('spawn-coords');
    if (coordsEl) {
        coordsEl.style.display = 'block';
        coordsEl.textContent = `Spawn: (${spawnPoint.x.toFixed(2)}, ${spawnPoint.y.toFixed(2)}, ${spawnPoint.z.toFixed(2)})`;
    }

    console.log('Spawn point set:', spawnPoint);
}

function updateSpawnMarker(pos) {
    if (!spawnMarkerEntity) {
        spawnMarkerEntity = new pc.Entity('spawn-marker');
        spawnMarkerEntity.addComponent('render', {
            type: 'sphere',
        });
        // Blue material
        const material = new pc.StandardMaterial();
        material.diffuse.set(0, 0.4, 1);
        material.emissive.set(0, 0.4, 1);
        material.emissiveIntensity = 0.5;
        material.update();
        spawnMarkerEntity.render.meshInstances[0].material = material;
        app.root.addChild(spawnMarkerEntity);
    }
    spawnMarkerEntity.setLocalScale(0.03, 0.03, 0.03);
    spawnMarkerEntity.setPosition(pos.x, pos.y, pos.z);
    spawnMarkerEntity.enabled = true;
}

// Full reset back to the "just-loaded page, no scene" state. Called when the
// user confirms Esc from placement/flight. Mirrors the pristine DOM layout
// (drop-zone visible, no HUD / key-guide / logo / placement overlay, black
// canvas), silences engine audio, and tears down scene-specific PlayCanvas
// entities so the next file drop gets a clean slate.
function _exitToLoading() {
    mode = 'loading';
    sceneLoaded = false;
    _bgmForMode(mode);

    // Silence the engine sound — gameLoop returns early once sceneLoaded is
    // false, so if we don't ramp it down here it stays frozen at whatever
    // throttle/armed state the last flight frame left behind.
    try { if (engineAudio) engineAudio.update(0, false); }
    catch (e) { console.warn('[EngineAudio] silence on exit failed:', e); }

    // Destroy the gsplat entity so nothing is rendered; loadGSplat() also
    // destroys any existing 'gsplat-scene' entity before adding a new one, so
    // this is safe across re-loads.
    try {
        const gs = app?.root?.findByName('gsplat-scene');
        if (gs) gs.destroy();
    } catch (_) {}
    if (spawnMarkerEntity) spawnMarkerEntity.enabled = false;

    // Tear down the race course so no stale gate entities linger. The
    // next rebuildGateCourse() call (triggered after the next scene's
    // filter commit) will materialise fresh gates from whatever path
    // the per-scene JSON file contains for that map.
    try { gateCourse?.destroy(); } catch (_) {}
    sceneFilteredBounds = null;
    currentSceneFile = null;
    gateMode = false;  // always off on new scene; G toggles after load

    // Disarm so there is no "ARMED" flash if the HUD ever re-shows before a
    // fresh confirmSpawnAndFly().
    if (controller) controller.armed = false;

    // Hide every mode-specific overlay and reset inline-styled readouts.
    document.getElementById('placement-overlay')?.classList.remove('visible');
    document.getElementById('game-logo')?.classList.remove('visible');
    document.getElementById('hud')?.classList.add('hidden');
    document.getElementById('key-guide')?.classList.remove('visible');
    document.getElementById('collision-flash')?.classList.remove('active');
    document.getElementById('loading-overlay')?.classList.remove('visible');
    document.getElementById('filter-overlay')?.classList.remove('visible');
    const filterGuide = document.getElementById('filter-guide');
    if (filterGuide) filterGuide.style.display = 'none';
    const coordsEl = document.getElementById('spawn-coords');
    if (coordsEl) coordsEl.style.display = 'none';

    // Clear the OSD canvas so its last flight frame does not linger once the
    // HUD container becomes visible again.
    if (osd && typeof osd.clear === 'function') {
        try { osd.clear(); } catch (_) {}
    } else if (osd?.canvas) {
        const ctx = osd.canvas.getContext('2d');
        if (ctx) ctx.clearRect(0, 0, osd.canvas.width, osd.canvas.height);
    }

    document.getElementById('drop-zone')?.classList.remove('hidden');
    console.log('Exited to loading state');
}

function enterPlacementMode() {
    mode = 'placement';
    _bgmForMode(mode);

    // Show placement UI, hide flight HUD and logo
    document.getElementById('placement-overlay').classList.add('visible');
    document.getElementById('hud').classList.add('hidden');
    document.getElementById('game-logo')?.classList.remove('visible');
    applyDisplaySettings();

    // Auto-set spawn point at origin (or keep current if re-entering)
    if (drone && sceneLoaded && spawnPoint) {
        // Re-entering placement: keep current spawn point
    } else {
        spawnPoint = { x: 0, y: 0, z: 0 };
    }

    // Re-enable unified rendering for filter shader (work-buffer modifier)
    const gsplatEntity = app.root.findByName('gsplat-scene');
    if (gsplatEntity && gsplatEntity.gsplat) {
        try { gsplatEntity.gsplat.unified = true; } catch (_) {}
    }

    // Orbit camera around spawn point
    orbitTarget = { x: spawnPoint.x, y: spawnPoint.y, z: spawnPoint.z };
    updateSpawnMarker(spawnPoint);
    _updateSpawnCoordsUI();

    console.log('Entered placement mode');
}

function _updateSpawnCoordsUI() {
    const coordsEl = document.getElementById('spawn-coords');
    if (coordsEl && spawnPoint) {
        coordsEl.style.display = 'block';
        coordsEl.textContent = `Spawn: (${spawnPoint.x.toFixed(2)}, ${spawnPoint.y.toFixed(2)}, ${spawnPoint.z.toFixed(2)})`;
    }
}

function applyDisplaySettings() {
    const cleanToggle = document.getElementById('clean-mode-toggle');
    const cleanMode = cleanToggle ? cleanToggle.checked : false;
    const displayMode = getDisplayMode();

    if (osd) {
        const osdToggle = document.getElementById('osd-toggle');
        osd.setEnabled(osdToggle ? osdToggle.checked : true);
    }

    const logo = document.getElementById('game-logo');
    const keyGuide = document.getElementById('key-guide');
    if (cleanMode) {
        logo?.classList.remove('visible');
        keyGuide?.classList.remove('visible');
    } else {
        if (displayMode === 'flight') logo?.classList.add('visible');
        else logo?.classList.remove('visible');
        updateKeyGuide();
    }
}

function getDisplayMode() {
    const hudEl = document.getElementById('hud');
    const placementEl = document.getElementById('placement-overlay');
    if (hudEl && !hudEl.classList.contains('hidden') && !placementEl?.classList.contains('visible')) return 'flight';
    if (placementEl?.classList.contains('visible')) return 'placement';
    if (mode === 'flight' || mode === 'placement') return mode;
    return mode;
}

function setupDisplaySettingsListeners() {
    for (const id of ['clean-mode-toggle', 'osd-toggle']) {
        const el = document.getElementById(id);
        if (!el || el._mainDisplayBound) continue;
        el._mainDisplayBound = true;
        el.addEventListener('change', () => applyDisplaySettings());
    }
}

function confirmSpawnAndFly() {
    if (!spawnPoint) return;

    mode = 'flight';
    _bgmForMode(mode);

    // Hide placement UI, show flight HUD and logo
    document.getElementById('placement-overlay').classList.remove('visible');
    hud.show();
    document.getElementById('game-logo')?.classList.add('visible');
    applyDisplaySettings();

    // Hide spawn marker
    if (spawnMarkerEntity) spawnMarkerEntity.enabled = false;
    const coordsEl = document.getElementById('spawn-coords');
    if (coordsEl) coordsEl.style.display = 'none';

    // Disable unified rendering so that SH (spherical-harmonic) colours are
    // evaluated per-frame with the current camera view direction.  In unified
    // mode the work buffer bakes SH at copy-time; since the gsplat entity
    // never moves during flight the buffer is never refreshed, causing
    // rainbow flickering when the camera yaws.
    const gsplatEntity = app.root.findByName('gsplat-scene');
    if (gsplatEntity && gsplatEntity.gsplat) {
        try { gsplatEntity.gsplat.unified = false; } catch (_) {}
    }

    // Set drone spawn and reset
    drone.setSpawnPoint(spawnPoint.x, spawnPoint.y, spawnPoint.z);
    drone.reset();
    controller.armed = true; // Auto-arm so drone flies immediately

    console.log('Confirmed spawn, entering flight mode');
}

// ---- File Loading (setup immediately on page load) ----
function setupFileLoading() {
    const dropZone = document.getElementById('drop-zone');
    const fileInput = document.getElementById('file-input');
    const filePickerBtn = document.getElementById('file-picker-btn');

    if (!filePickerBtn || !fileInput || !dropZone) {
        console.error('File loading elements not found in DOM');
        return;
    }

    // File picker button. Prefer window.showOpenFilePicker when available
    // (Chromium-based browsers on secure contexts): it lets us pass the last
    // picked FileSystemFileHandle as `startIn`, so the dialog re-opens at the
    // previous directory with the previous file highlighted — even across
    // ESC → re-pick cycles. Browsers without that API fall back to the legacy
    // <input type=file>.click() path, which has less reliable memory but at
    // least gives the OS's own last-directory behaviour.
    const hasFsAccess = typeof window.showOpenFilePicker === 'function';
    filePickerBtn.addEventListener('click', async (e) => {
        e.preventDefault();
        e.stopPropagation();

        if (!hasFsAccess) {
            fileInput.click();
            return;
        }

        try {
            const opts = {
                multiple: false,
                excludeAcceptAllOption: false,
                types: [{
                    description: 'Gaussian Splat scene',
                    accept: { 'application/octet-stream': ['.ply', '.sog', '.splat'] },
                }],
            };
            if (_lastFileHandle) opts.startIn = _lastFileHandle;
            const [handle] = await window.showOpenFilePicker(opts);
            _lastFileHandle = handle;
            const file = await handle.getFile();
            loadSceneFile(file);
        } catch (err) {
            // AbortError = user cancelled the dialog. Anything else is a real
            // failure — fall back to the legacy input so the user still has a
            // way to pick a file.
            if (err && err.name !== 'AbortError') {
                console.warn('showOpenFilePicker failed, falling back to <input>:', err);
                fileInput.click();
            }
        }
    });
    fileInput.addEventListener('change', (e) => {
        if (e.target.files.length > 0) {
            loadSceneFile(e.target.files[0]);
        }
    });

    // Drag and drop
    dropZone.addEventListener('dragover', (e) => {
        e.preventDefault();
        dropZone.classList.add('dragover');
    });
    dropZone.addEventListener('dragleave', () => {
        dropZone.classList.remove('dragover');
    });
    dropZone.addEventListener('drop', (e) => {
        e.preventDefault();
        dropZone.classList.remove('dragover');
        const files = e.dataTransfer.files;
        if (files.length > 0) {
            loadSceneFile(files[0]);
        }
    });

    // Also allow drop on entire window after scene is loaded (for switching scenes)
    window.addEventListener('dragover', (e) => e.preventDefault());
    window.addEventListener('drop', (e) => {
        e.preventDefault();
        if (sceneLoaded && e.dataTransfer.files.length > 0) {
            loadSceneFile(e.dataTransfer.files[0]);
        }
    });

    console.log('File loading ready');
}

async function loadSceneFile(file) {
    const ext = file.name.toLowerCase().split('.').pop();
    if (!['ply', 'sog', 'splat'].includes(ext)) {
        alert('Please select a .ply, .sog, or .splat file');
        return;
    }

    const dropZone = document.getElementById('drop-zone');
    const loadingOverlay = document.getElementById('loading-overlay');
    const loadingProgress = document.getElementById('loading-progress');

    dropZone.classList.add('hidden');
    loadingOverlay.classList.add('visible');
    loadingProgress.textContent = 'Reading file...';
    loadingProgress.style.color = '#4272F5';

    try {
        // Initialize PlayCanvas if not yet done
        if (!pcInitialized) {
            loadingProgress.textContent = 'Initializing PlayCanvas engine...';
            await sleep(10);
            initPlayCanvas();
        }

        // Initialize subsystems if not yet done
        if (!controller) controller = new Controller();
        if (!drone) drone = new Drone();
        if (!hud) hud = new HUD();
        // Audio is initialised once at page startup (see the bottom of this
        // file); this block is a safety-net for any corner case where the
        // top-level init failed (e.g. race with the module evaluation order).
        if (!engineAudio && !bgmAudio) _initAudio();

        const arrayBuffer = await file.arrayBuffer();
        console.log(`File read: ${file.name} (${(arrayBuffer.byteLength / 1024 / 1024).toFixed(1)} MB)`);

        // Remember the file for per-scene persistence (gate path + coord
        // system). Must be set before the filter UI opens so the coord
        // selector can be pre-populated with the remembered choice.
        currentSceneFile = file;

        // Best-lap is per-scene: clear whatever the previous scene
        // contributed BEFORE we try to load the new scene's record, so
        // that a fresh map (no saved JSON yet) or a map without a
        // stored PB starts at "--:--.---" instead of inheriting the
        // previous map's time.
        if (gateCourse) gateCourse.setBestLapMs(null);
        // Same goes for the gate path: if the incoming scene has no
        // saved layout we must not keep the outgoing scene's path
        // around, otherwise the user would see the old gates ghosting
        // into the new map.
        if (controller && controller.gatePathSettings) {
            controller.gatePathSettings.path = null;
        }

        // Load any previously-saved gate path + coord system for this
        // scene. File-based persistence key is filename + size (see
        // path-store.js#keyFor); missing file is normal for first-time
        // loads of a scene. Fetch is non-blocking for UI responsiveness.
        const savedScene = await pathStore.loadForScene(file);
        if (savedScene) {
            if (savedScene.coordSystem === 'zup' || savedScene.coordSystem === 'yup') {
                coordSystem = savedScene.coordSystem;
                console.log(`[path-store] restored coord=${coordSystem} for ${file.name}`);
            }
            if (savedScene.path && Array.isArray(savedScene.path.points) && controller) {
                // Hydrate controller's copy so the settings UI shows the
                // right gate count + best lap immediately. gateSize +
                // clearance come bundled with the path record.
                controller.gatePathSettings.path = {
                    closed: true,
                    points: savedScene.path.points.map(p => ({ x: p.x, y: p.y, z: p.z })),
                    yMin:   Number(savedScene.path.yMin),
                    yMax:   Number(savedScene.path.yMax),
                };
                if (Number.isFinite(Number(savedScene.path.gateSize))) {
                    controller.gatePathSettings.gateSize = Number(savedScene.path.gateSize);
                }
                if (Number.isFinite(Number(savedScene.path.clearance))) {
                    controller.gatePathSettings.clearance = Number(savedScene.path.clearance);
                }
            }
            if (gateCourse && Number.isFinite(Number(savedScene.bestLapMs))) {
                gateCourse.setBestLapMs(Number(savedScene.bestLapMs));
            }
        }
        if (controller && typeof controller.refreshSettingsUI === 'function') {
            controller.refreshSettingsUI();
        }

        const defaultCoord = coordSystem;
        let initialParse = null;
        let analysis = null;

        // ---- Format-specific: extract opacities & render buffer (coord-independent) ----
        let opacities = null;
        let rawCentroid = null;
        let renderBuffer = arrayBuffer;
        let renderFilename = file.name;

        if (ext === 'ply') {
            loadingProgress.textContent = 'Parsing PLY scene data...';
            await sleep(10);
            const plyData = parsePlySceneData(arrayBuffer, { zUp: defaultCoord === 'zup' });
            opacities = plyData.opacities;
            rawCentroid = plyData.rawCentroid;
            initialParse = {
                positions: plyData.positions,
                vertexCount: plyData.vertexCount,
                bounds: plyData.bounds,
            };
            analysis = plyData.analysis;

        } else if (ext === 'splat') {
            loadingProgress.textContent = 'Parsing opacity values...';
            await sleep(10);
            opacities = parseSplatOpacities(arrayBuffer);
            rawCentroid = parseSplatRawCentroid(arrayBuffer);
            loadingProgress.textContent = 'Converting .splat to PLY for rendering...';
            await sleep(10);
            renderBuffer = splatToPlyBuffer(arrayBuffer);
            renderFilename = file.name.replace(/\.splat$/i, '.ply');

        } else if (ext === 'sog') {
            loadingProgress.textContent = 'Extracting SOG data...';
            await sleep(10);
            opacities = await parseSogOpacities(arrayBuffer);
        }

        // ---- Initial parse with default coord system (for first filter display) ----
        if (!initialParse) {
            initialParse = await parseForCoord(ext, arrayBuffer, defaultCoord);
            analysis = analyzePlyDistances(initialParse.positions, initialParse.vertexCount);
        }

        if (ext === 'sog') {
            rawCentroid = {
                x: (initialParse.bounds.min[0] + initialParse.bounds.max[0]) * 0.5,
                y: (initialParse.bounds.min[1] + initialParse.bounds.max[1]) * 0.5,
                z: (initialParse.bounds.min[2] + initialParse.bounds.max[2]) * 0.5,
            };
        }

        console.log(`Initial parse: ${initialParse.vertexCount} vertices (${ext}, ${defaultCoord}), bounds:`, initialParse.bounds);

        // ---- Sanitize render buffer (replace NaN/Inf in all float properties) ----
        if (ext === 'ply' || ext === 'splat') {
            loadingProgress.textContent = 'Sanitizing point cloud data...';
            await sleep(10);
            sanitizePlyBuffer(renderBuffer);
        }

        // ---- Load GSplat into PlayCanvas ----
        loadingProgress.textContent = 'Loading 3D Gaussian Splat...';
        await sleep(10);
        rawArrayBuffer = arrayBuffer;
        const initialRotation = getEntityRotation(ext, defaultCoord);
        await loadGSplat(renderBuffer, renderFilename, initialRotation);

        const gsplatEntity = app.root.findByName('gsplat-scene');
        setupGSplatFilter(gsplatEntity, rawCentroid, analysis.maxDistance);

        // ---- Enter filtering mode ----
        sceneBounds = initialParse.bounds;
        orbitTarget = { x: 0, y: 0, z: 0 };
        orbitDistance = 1;
        orbitYaw = 0;
        orbitPitch = -30;
        spawnPoint = null;
        sceneLoaded = true;
        mode = 'filtering';
        _bgmForMode(mode);

        // Hide logo during filtering
        document.getElementById('game-logo')?.classList.remove('visible');
        loadingOverlay.classList.remove('visible');

        // Show filter UI — user picks coord system + filter sliders
        // Callback for live coord switching during filtering
        const onCoordChange = async (newCoord) => {
            let parsed;
            if (ext === 'ply') {
                const plyData = parsePlySceneData(arrayBuffer, { zUp: newCoord === 'zup', includeOpacities: false });
                parsed = {
                    positions: plyData.positions,
                    vertexCount: plyData.vertexCount,
                    bounds: plyData.bounds,
                };
                analysis = plyData.analysis;
            } else {
                parsed = await parseForCoord(ext, arrayBuffer, newCoord);
                analysis = analyzePlyDistances(parsed.positions, parsed.vertexCount);
            }
            const rot = getEntityRotation(ext, newCoord);
            if (gsplatEntity) gsplatEntity.setEulerAngles(rot[0], rot[1], rot[2]);
            if (ext === 'sog') {
                rawCentroid = {
                    x: (parsed.bounds.min[0] + parsed.bounds.max[0]) * 0.5,
                    y: (parsed.bounds.min[1] + parsed.bounds.max[1]) * 0.5,
                    z: (parsed.bounds.min[2] + parsed.bounds.max[2]) * 0.5,
                };
            }
            setupGSplatFilter(gsplatEntity, rawCentroid, analysis.maxDistance);
            return { positions: parsed.positions, vertexCount: parsed.vertexCount, bounds: parsed.bounds, analysis };
        };

        const filterResult = await showFilterUI(
            gsplatEntity, initialParse.positions, opacities, initialParse.vertexCount,
            analysis, initialParse.bounds, rawCentroid, ext, defaultCoord, onCoordChange
        );

        // ---- Apply final coord system choice ----
        coordSystem = filterResult.coordSystem;
        const settingsSelect = document.getElementById('coord-system-select');
        if (settingsSelect) settingsSelect.value = coordSystem;
        console.log(`Coord system locked to: ${coordSystem}`);

        // ---- Build collision octree with final coord + filter ----
        loadingOverlay.classList.add('visible');
        loadingProgress.textContent = 'Applying filter and building collision...';
        await sleep(10);

        // Use the final positions from filterResult (already parsed with chosen coord)
        const finalPositions = filterResult.finalPositions;
        const finalVertexCount = filterResult.finalVertexCount;
        const finalAnalysis = filterResult.finalAnalysis;

        const {
            positions: filteredPosArray,
            keptCount,
            bounds: filteredBounds,
        } = buildFilteredPositionBuffer(
            finalPositions,
            opacities,
            finalVertexCount,
            finalAnalysis,
            filterResult.maxDist,
            filterResult.minOpacity,
            filterResult.finalBounds
        );
        console.log(`Final filter: ${keptCount}/${finalVertexCount} kept (dist=${filterResult.maxDist}, opacity=${filterResult.minOpacity})`);

        // Build collision spatial index
        octree = new Octree();
        octree.build(filteredPosArray, filteredBounds);
        cachedArrayBuffer = arrayBuffer;
        cachedFilename = file.name;
        cachedFormat = ext;
        sceneFilteredBounds = filteredBounds;

        // Persist the (possibly updated) coord system back to the
        // per-scene JSON file. Overwrites previous value if any; path +
        // bestLap are preserved. Fire-and-forget — UI shouldn't wait.
        _persistSceneRecord();

        // Build the race course entities for this scene+filter. Entities
        // start hidden (gateMode is off on scene load); the user presses
        // G in flight mode to reveal them.
        rebuildGateCourse();

        // Done loading
        loadingOverlay.classList.remove('visible');
        enterPlacementMode();
        console.log('Scene loaded — entering placement mode');

    } catch (err) {
        console.error('Error loading scene:', err);
        showError(`Error: ${err.message}`);
        setTimeout(() => {
            loadingOverlay.classList.remove('visible');
            dropZone.classList.remove('hidden');
        }, 3000);
    }
}

/**
 * Parse positions for a given coordinate system. Works for all formats.
 */
function buildFilteredPositionBuffer(positions, opacities, vertexCount, analysis, maxDist, minOpacity, fullBounds) {
    const maxDistSq = maxDist * maxDist;
    const cx = analysis.centroid.x;
    const cy = analysis.centroid.y;
    const cz = analysis.centroid.z;
    const checkOpacity = !!opacities && minOpacity > 0;
    const allDistanceKept = maxDist >= Math.ceil(analysis.maxDistance);

    if (!checkOpacity && allDistanceKept) {
        return {
            positions,
            keptCount: vertexCount,
            bounds: fullBounds || computePositionBounds(positions, vertexCount),
        };
    }

    let keptCount = 0;
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;

    for (let i = 0, off = 0; i < vertexCount; i++, off += 3) {
        if (checkOpacity && opacities[i] < minOpacity) continue;
        const x = positions[off];
        const y = positions[off + 1];
        const z = positions[off + 2];
        const dx = x - cx, dy = y - cy, dz = z - cz;
        if (dx * dx + dy * dy + dz * dz > maxDistSq) continue;
        keptCount++;
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }

    if (keptCount === 0) {
        return {
            positions: new Float32Array(0),
            keptCount: 0,
            bounds: { min: [0, 0, 0], max: [0, 0, 0] },
        };
    }

    const filtered = new Float32Array(keptCount * 3);
    let out = 0;
    for (let i = 0, off = 0; i < vertexCount; i++, off += 3) {
        if (checkOpacity && opacities[i] < minOpacity) continue;
        const x = positions[off];
        const y = positions[off + 1];
        const z = positions[off + 2];
        const dx = x - cx, dy = y - cy, dz = z - cz;
        if (dx * dx + dy * dy + dz * dz > maxDistSq) continue;
        filtered[out++] = x;
        filtered[out++] = y;
        filtered[out++] = z;
    }

    return {
        positions: filtered,
        keptCount,
        bounds: { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] },
    };
}

function computePositionBounds(positions, vertexCount) {
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    for (let i = 0, off = 0; i < vertexCount; i++, off += 3) {
        const x = positions[off], y = positions[off + 1], z = positions[off + 2];
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }
    if (vertexCount === 0) return { min: [0, 0, 0], max: [0, 0, 0] };
    return { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] };
}

async function parseForCoord(ext, arrayBuffer, coord) {
    const isZup = coord === 'zup';
    if (ext === 'sog') {
        return await parseSogForPositions(arrayBuffer, { zUp: isZup });
    } else if (ext === 'splat') {
        return parseSplatForPositions(arrayBuffer, { zUp: isZup });
    } else {
        return parsePlyForPositions(arrayBuffer, { zUp: isZup });
    }
}

/**
 * Get entity rotation for a given format and coordinate system.
 */
function getEntityRotation(ext, coord) {
    const isZup = coord === 'zup';
    if (ext === 'sog') return isZup ? [-90, 0, 0] : [0, 0, 0];
    return isZup ? [-90, 0, 0] : [180, 0, 0];
}

async function loadGSplat(arrayBuffer, filename, entityRotation) {
    return new Promise((resolve, reject) => {
        // Create a blob URL from the array buffer
        const blob = new Blob([arrayBuffer], { type: 'application/octet-stream' });
        const blobUrl = URL.createObjectURL(blob);
        // Append filename as hash so PlayCanvas can detect format from extension
        const url = blobUrl + '#/' + filename;

        // Remove existing gsplat entity if any
        const existing = app.root.findByName('gsplat-scene');
        if (existing) existing.destroy();

        // Create asset
        const asset = new pc.Asset(filename, 'gsplat', { url: url, filename: filename });

        asset.on('load', () => {
            const entity = new pc.Entity('gsplat-scene');
            entity.addComponent('gsplat', {
                asset: asset,
            });
            // Apply rotation (format-dependent, passed from caller)
            const rot = entityRotation || [0, 0, 0];
            entity.setEulerAngles(rot[0], rot[1], rot[2]);
            app.root.addChild(entity);

            // Set a large custom AABB to prevent frustum culling from clipping
            // splats at viewport edges during fast camera rotation
            entity.gsplat.customAabb = new pc.BoundingBox(
                new pc.Vec3(0, 0, 0),
                new pc.Vec3(1000, 1000, 1000)
            );

            URL.revokeObjectURL(blobUrl);
            resolve();
        });

        asset.on('error', (err) => {
            URL.revokeObjectURL(blobUrl);
            reject(new Error(`Failed to load GSplat: ${err}`));
        });

        app.assets.add(asset);
        app.assets.load(asset);
    });
}

// ---- GPU shader filter setup ----
function setupGSplatFilter(entity, rawCentroid, maxDistance) {
    if (!entity || !entity.gsplat) {
        console.warn('GSplat entity not available for shader filter');
        return;
    }

    const gsplat = entity.gsplat;

    // Enable unified rendering (required for setWorkBufferModifier)
    try {
        gsplat.unified = true;
    } catch (e) {
        console.warn('Could not enable unified mode:', e);
    }

    // Install custom shader modifier for distance + opacity filtering
    try {
        gsplat.setWorkBufferModifier({
            glsl: `
                uniform vec3 filterCenter;
                uniform float filterMaxDistSq;
                uniform float filterMinOpacity;

                void modifySplatCenter(inout vec3 center) {}

                void modifySplatRotationScale(vec3 originalCenter, vec3 modifiedCenter, inout vec4 rotation, inout vec3 scale) {
                    vec3 delta = originalCenter - filterCenter;
                    if (dot(delta, delta) > filterMaxDistSq) {
                        scale = vec3(0.0);
                    }
                }

                void modifySplatColor(vec3 center, inout vec4 color) {
                    if (color.a < filterMinOpacity) {
                        color.a = 0.0;
                    }
                }
            `
        });
    } catch (e) {
        console.warn('setWorkBufferModifier not available:', e);
    }

    // Set initial uniform values (no filtering)
    const maxDist = Math.ceil(maxDistance);
    try {
        gsplat.setParameter('filterCenter', [rawCentroid.x, rawCentroid.y, rawCentroid.z]);
        gsplat.setParameter('filterMaxDistSq', maxDist * maxDist);
        gsplat.setParameter('filterMinOpacity', 0.0);
    } catch (e) {
        console.warn('setParameter not available:', e);
    }
}

function updateGSplatFilter(entity, maxDist, minOpacity) {
    if (!entity || !entity.gsplat) return;
    try {
        entity.gsplat.setParameter('filterMaxDistSq', maxDist * maxDist);
        entity.gsplat.setParameter('filterMinOpacity', minOpacity);
        // Trigger work buffer re-render
        if (pc.WORKBUFFER_UPDATE_ONCE !== undefined) {
            entity.gsplat.workBufferUpdate = pc.WORKBUFFER_UPDATE_ONCE;
        }
    } catch (e) {
        console.warn('updateGSplatFilter error:', e);
    }
}

// ---- Scene filter UI ----
function showFilterUI(gsplatEntity, positions, opacities, vertexCount, analysis, bounds, rawCentroid, ext, defaultCoord, onCoordChange) {
    return new Promise((resolve) => {
        const overlay = document.getElementById('filter-overlay');
        const filterGuide = document.getElementById('filter-guide');
        const distSlider = document.getElementById('filter-dist-slider');
        const distInput = document.getElementById('filter-dist-input');
        const opacitySlider = document.getElementById('filter-opacity-slider');
        const opacityInput = document.getElementById('filter-opacity-input');
        const statsEl = document.getElementById('filter-stats');
        const applyBtn = document.getElementById('filter-apply-btn');
        const coordSelect = document.getElementById('filter-coord-select');
        const coordLabel = document.getElementById('filter-coord-label');

        // Mutable state (updated when coord changes)
        let curPositions = positions;
        let curVertexCount = vertexCount;
        let curAnalysis = analysis;
        let curBounds = bounds;
        let curCoord = defaultCoord;

        // Configure coord selector (shown for all formats including SOG)
        coordSelect.style.display = '';
        if (coordLabel) coordLabel.style.display = '';
        coordSelect.value = defaultCoord;

        let maxExtent = Math.ceil(curAnalysis.maxDistance);

        // Configure distance slider + input
        distSlider.max = maxExtent;
        distInput.max = maxExtent;
        const defaultDist = maxExtent;
        distSlider.value = defaultDist;
        distInput.value = defaultDist;

        // Configure opacity slider + input
        opacitySlider.value = 0;
        opacityInput.value = '0.00';

        let statsRaf = 0;
        const runStatsUpdate = () => {
            statsRaf = 0;
            const dist = parseFloat(distSlider.value);
            const minOp = parseFloat(opacitySlider.value);
            const count = countFilteredPoints(
                curPositions, opacities, curVertexCount,
                curAnalysis.centroid.x, curAnalysis.centroid.y, curAnalysis.centroid.z,
                dist, minOp
            );
            statsEl.textContent = `${count.toLocaleString()} / ${curVertexCount.toLocaleString()} points kept`;
        };
        const scheduleStatsUpdate = () => {
            if (statsRaf) return;
            statsRaf = requestAnimationFrame(runStatsUpdate);
        };

        // Update stats + GPU filter
        const updateFilter = (immediateStats = false) => {
            const dist = parseFloat(distSlider.value);
            const minOp = parseFloat(opacitySlider.value);
            distInput.value = dist;
            opacityInput.value = minOp.toFixed(2);

            // Update GPU shader uniforms (instant visual update)
            updateGSplatFilter(gsplatEntity, dist, minOp);

            if (immediateStats === true) {
                if (statsRaf) cancelAnimationFrame(statsRaf);
                statsRaf = 0;
                runStatsUpdate();
            } else {
                scheduleStatsUpdate();
            }
        };

        // Sync number inputs → sliders
        const onDistInput = () => {
            distSlider.value = distInput.value;
            updateFilter();
        };
        const onOpacityInput = () => {
            opacitySlider.value = opacityInput.value;
            updateFilter();
        };

        // Coord system change handler
        const onCoordSelect = async () => {
            const newCoord = coordSelect.value;
            if (newCoord === curCoord) return;
            curCoord = newCoord;
            statsEl.textContent = 'Re-parsing...';

            const result = await onCoordChange(newCoord);
            curPositions = result.positions;
            curVertexCount = result.vertexCount;
            curAnalysis = result.analysis;
            curBounds = result.bounds;

            // Update distance slider range
            maxExtent = Math.ceil(curAnalysis.maxDistance);
            distSlider.max = maxExtent;
            distInput.max = maxExtent;
            if (parseFloat(distSlider.value) > maxExtent) {
                distSlider.value = maxExtent;
                distInput.value = maxExtent;
            }

            updateFilter(true);
            console.log(`Coord switched to ${newCoord} — ${curVertexCount} vertices`);
        };

        // Initial update
        updateFilter(true);
        updateGSplatFilter(gsplatEntity, defaultDist, 0);

        distSlider.addEventListener('input', updateFilter);
        opacitySlider.addEventListener('input', updateFilter);
        distInput.addEventListener('input', onDistInput);
        opacityInput.addEventListener('input', onOpacityInput);
        coordSelect.addEventListener('change', onCoordSelect);

        const onApply = () => {
            distSlider.removeEventListener('input', updateFilter);
            opacitySlider.removeEventListener('input', updateFilter);
            distInput.removeEventListener('input', onDistInput);
            opacityInput.removeEventListener('input', onOpacityInput);
            coordSelect.removeEventListener('change', onCoordSelect);
            applyBtn.removeEventListener('click', onApply);
            if (statsRaf) {
                cancelAnimationFrame(statsRaf);
                statsRaf = 0;
            }
            overlay.classList.remove('visible');
            if (filterGuide) filterGuide.style.display = 'none';
            resolve({
                maxDist: parseFloat(distSlider.value),
                minOpacity: parseFloat(opacitySlider.value),
                coordSystem: curCoord,
                finalPositions: curPositions,
                finalVertexCount: curVertexCount,
                finalAnalysis: curAnalysis,
                finalBounds: curBounds,
            });
        };
        applyBtn.addEventListener('click', onApply);

        overlay.classList.add('visible');
        if (filterGuide) filterGuide.style.display = 'block';
    });
}

// ---- Camera FOV ----
function updateCameraFov() {
    if (!cameraEntity) return;
    const hfovEl = document.getElementById('cam-hfov');
    const hfov = hfovEl ? parseFloat(hfovEl.value) : 120;
    const aspect = app.graphicsDevice.width / app.graphicsDevice.height;
    const hfovRad = hfov * Math.PI / 180;
    const vfov = 2 * Math.atan(Math.tan(hfovRad / 2) / aspect) * 180 / Math.PI;
    cameraEntity.camera.fov = vfov;
}

// ---- Race Course ----
/**
 * Rebuild the gate entities from `controller.gatePathSettings.path`.
 *
 * Called from three places:
 *   1. End of loadSceneFile(), once the octree is built.
 *   2. controller.js `_applyGatePath()` → attachGateCourse(applyCb) →
 *      here. Fires whenever gateSize/clearance/path change in the
 *      settings panel or the path editor.
 *   3. Itself is a no-op when no scene is loaded or the user hasn't
 *      drawn a path.
 *
 * Visibility follows `gateMode` (G-key toggle): entities always exist
 * after rebuild, but `setVisible(false)` hides them until the player
 * presses G in flight mode.
 */
function rebuildGateCourse() {
    if (!gateCourse) return;
    if (controller && controller.gatePathSettings) {
        gateCourse.configure(controller.gatePathSettings);
    }

    // Preconditions: need a scene loaded and an app. Without these the
    // course silently destroys itself so nothing stale lingers.
    if (!app || !octree || !sceneFilteredBounds) {
        gateCourse.destroy();
        return;
    }

    const path = controller && controller.gatePathSettings && controller.gatePathSettings.path;
    const points = (path && Array.isArray(path.points)) ? path.points : [];
    if (points.length < 3) {
        // Not enough to form a closed loop — wipe any prior entities.
        gateCourse.destroy();
        return;
    }

    gateCourse.rebuild({ app, octree, points });
    gateCourse.setVisible(gateMode);

    console.log(`[Race] rebuilt ${gateCourse.gates.length} gates`
        + (gateCourse.bestLapMs != null ? ` (best ${(gateCourse.bestLapMs / 1000).toFixed(3)}s)` : '')
        + ` [mode=${gateMode ? 'on' : 'off'}]`);
}

/**
 * Write the current per-scene record (coord system, path, best lap) to
 * the backing JSON file via `src/path-store.js`. Silently no-ops when
 * no scene is loaded. Intentionally fire-and-forget — the UI never
 * blocks on persistence completion, and any network failure is logged
 * from inside `path-store`.
 */
function _persistSceneRecord() {
    if (!currentSceneFile) return;
    const gp = controller && controller.gatePathSettings;
    if (!gp) return;
    const path = (gp.path && Array.isArray(gp.path.points) && gp.path.points.length >= 3)
        ? {
            closed:    true,
            points:    gp.path.points.map(p => ({ x: p.x, y: p.y, z: p.z })),
            yMin:      Number(gp.path.yMin),
            yMax:      Number(gp.path.yMax),
            gateSize:  Number(gp.gateSize),
            clearance: Number(gp.clearance),
        }
        : null;
    pathStore.saveForScene(currentSceneFile, {
        coordSystem: coordSystem,
        path,
        bestLapMs: gateCourse ? gateCourse.bestLapMs : null,
    });
}

/**
 * Flip the G-key gate-race toggle. Silently refuses if no path is drawn
 * (so G becomes a no-op on a fresh scene, matching the spec). On toggle
 * ON we force-rebuild to make sure entities match the latest settings
 * even if the user changed gateSize/clearance while gates were hidden.
 */
function _toggleGateMode() {
    const gp = controller && controller.gatePathSettings;
    const hasPath = gp && gp.path && Array.isArray(gp.path.points) && gp.path.points.length >= 3;
    if (!hasPath) {
        console.info('[Race] no path drawn yet — open Settings → Gate Path → Edit Path… first');
        return;
    }
    gateMode = !gateMode;
    if (gateMode && (!gateCourse || gateCourse.gates.length === 0)) {
        // Entities not yet built for this session; build now so G reveals
        // a real course on the first press after a fresh scene load.
        rebuildGateCourse();
    } else if (gateCourse) {
        gateCourse.setVisible(gateMode);
    }
    console.log(`[Race] gate mode ${gateMode ? 'ON' : 'OFF'}`);
}

// ---- Game Loop ----
function gameLoop(dt) {
    if (!sceneLoaded) return;

    // Update camera FOV from settings (applies in all modes)
    updateCameraFov();

    if (mode === 'placement' || mode === 'filtering') {
        updateOrbitCamera(dt);
        return;
    }

    if (mode !== 'flight') return;

    // Read settings
    drone.readSettings();

    // Get controller input
    const input = controller.update();

    // Handle reset
    if (input.resetTriggered) {
        console.log('RESET triggered — drone position before:', drone.x.toFixed(2), drone.y.toFixed(2), drone.z.toFixed(2));
        drone.reset();
        controller.armed = true; // Stay armed after reset
        // Wipe current-lap progress so the player starts fresh. Layout,
        // best-lap record, and the G-mode toggle are all preserved —
        // only the in-progress lap timer + gate pass flags reset.
        gateCourse?.resetLap();
        console.log('RESET done — drone position after:', drone.x.toFixed(2), drone.y.toFixed(2), drone.z.toFixed(2));
    }

    // Camera tilt (drone mode only)
    if (drone.flightMode === 'drone') {
        // Q/E keyboard: always incremental
        if (Math.abs(input.cameraTiltKeyboard) > 0.05) {
            drone.adjustCameraTilt(input.cameraTiltKeyboard * 60 * dt);
        }
        // Assigned axis: direct mapping when axis changes
        if (input.cameraTiltAxisChanged) {
            drone.cameraTiltAngle = ((input.cameraTiltAxis + 1) / 2) * -90; // 1→0°, -1→-90°
        }
    }

    // Update drone physics
    drone.update(dt, input, octree);

    // Update engine sound (frequency/gain scale with normalized thrust)
    if (engineAudio) {
        try {
            const throttle01 = drone.maxThrust > 0
                ? Math.max(0, Math.min(1, drone.thrustOutput / drone.maxThrust))
                : 0;
            engineAudio.update(throttle01, !!input.armed);
        } catch (e) {
            console.warn('[EngineAudio] update failed, disabling:', e);
            engineAudio = null;
        }
    }

    // Apply drone transform to camera
    const transform = drone.getCameraTransform();
    cameraEntity.setPosition(transform.position.x, transform.position.y, transform.position.z);
    cameraEntity.setEulerAngles(transform.rotation.x, transform.rotation.y, transform.rotation.z);

    // Race course: pulse the next gate and run pass-through detection using
    // the drone's current world position. Safe no-op when the course is
    // disabled or empty.
    gateCourse?.update(dt, drone);

    // HUD - always visible in flight mode (not affected by Clean Mode)
    const hudEl = document.getElementById('hud');
    hudEl?.classList.remove('hidden');
    hud.update(drone, controller, gateCourse);

    applyDisplaySettings();
    if (osd) osd.update(drone, controller);
}

// ---- Key Guide ----
function updateKeyGuide() {
    const el = document.getElementById('key-guide');
    if (!el) return;

    const displayMode = getDisplayMode();
    if (displayMode === 'placement') {
        el.innerHTML =
            '<div class="guide-title">PLACEMENT MODE</div>' +
            '<kbd>W</kbd><kbd>S</kbd>  Forward / Back\n' +
            '<kbd>A</kbd><kbd>D</kbd>  Left / Right\n' +
            '<kbd>Q</kbd><kbd>E</kbd>  Down / Up\n' +
            'Mouse   Orbit view\n' +
            'Scroll  Zoom\n' +
            '<kbd>Enter</kbd> Confirm &amp; Fly\n' +
            '<kbd>Esc</kbd>   Cancel';
        el.classList.add('visible');
    } else if (displayMode === 'flight') {
        el.innerHTML =
            '<div class="guide-title">FLIGHT CONTROLS</div>' +
            '<kbd>↑</kbd><kbd>↓</kbd>  Pitch (Fwd/Back)\n' +
            '<kbd>←</kbd><kbd>→</kbd>  Roll (Left/Right)\n' +
            '<kbd>W</kbd><kbd>S</kbd>  Throttle (Up/Down)\n' +
            '<kbd>A</kbd><kbd>D</kbd>  Yaw (Left/Right)\n' +
            '<kbd>Q</kbd><kbd>E</kbd>  Camera Tilt\n' +
            '<kbd>Space</kbd> Arm / Disarm\n' +
            '<kbd>Shift</kbd> Boost\n' +
            '<kbd>R</kbd>     Reset\n' +
            '<kbd>G</kbd>     Gate race on/off\n' +
            '<kbd>M</kbd>     Flight Mode (FPV/Drone)\n' +
            '<kbd>P</kbd>     Placement mode\n' +
            '<kbd>Tab</kbd>   Settings';
        el.classList.add('visible');
    } else {
        el.classList.remove('visible');
    }
}

// ---- Utility ----
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// ---- Start ----
// Setup file loading immediately (no PlayCanvas dependency)
setupFileLoading();
// Initialize controller early so settings panel works
try {
    controller = new Controller();
} catch(e) {
    console.warn('Controller init deferred:', e);
}
try {
    osd = new OSD('osd-canvas');
} catch(e) {
    console.warn('OSD init deferred:', e);
}
setupDisplaySettingsListeners();

// Audio subsystems (engine sound + BGM). Initialised at startup so the
// initializ.flac BGM can start playing as soon as the user makes their first
// keypress — no need to wait for a scene file to be dropped.
function _initAudio() {
    if (!engineAudio && !_engineAudioDisabled) {
        try {
            engineAudio = new EngineAudio();
            console.info('[EngineAudio] enabled (press any key to unlock; add ?noaudio=1 to disable)');
        } catch (e) {
            console.warn('[EngineAudio] init failed, continuing silently:', e);
            engineAudio = null;
        }
    }
    if (!bgmAudio && !_bgmAudioDisabled) {
        try {
            bgmAudio = new BgmAudio();
            console.info('[BgmAudio] enabled (press any key to unlock; add ?nobgm=1 to disable)');
        } catch (e) {
            console.warn('[BgmAudio] init failed, continuing silently:', e);
            bgmAudio = null;
        }
    }
    _installAudioGestureHook();
    if (controller && typeof controller.attachAudio === 'function') {
        controller.attachAudio(engineAudio, bgmAudio);
    }
    // Kick off asynchronous track discovery; once complete _loadBgmPlaylists
    // calls _bgmForMode(mode) which arms playback for the current mode.
    // Playback is queued until the first user gesture resumes the AudioContext.
    _loadBgmPlaylists().catch(e => console.warn('[BgmAudio] playlist discovery failed:', e));
}
_initAudio();

// Race course. Constructed eagerly so the settings panel can edit its
// parameters before the first scene is loaded; gate entities are
// created only when rebuildGateCourse() runs after a filter commit.
gateCourse = new GateCourse();

// Persist a fresh best-lap record the instant the player sets one,
// so a browser crash mid-session doesn't cost them the new PB.
gateCourse.onBestLap = () => { _persistSceneRecord(); };

if (controller && typeof controller.attachGateCourse === 'function') {
    // applyCb: rebuild entities + push current settings through to disk.
    // ctxProvider: live getter exposing module-scoped globals to
    //              controller.js (it avoids importing main.js itself).
    controller.attachGateCourse(
        gateCourse,
        () => {
            rebuildGateCourse();
            _persistSceneRecord();
        },
        () => ({
            octree,
            bounds:     sceneFilteredBounds,
            spawnPoint: spawnPoint,
        })
    );
}

console.log('MindCloud World Fly ready — drop a .ply, .sog, or .splat file to begin');
