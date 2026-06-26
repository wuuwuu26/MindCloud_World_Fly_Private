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
 * Main entry point for the Google 3D Tiles flight mode.
 *
 * Rendering is Cesium + Google Photorealistic 3D Tiles. Flight dynamics,
 * controller mapping, WebHID/Gamepad support, HUD and OSD are retained
 * from the original simulator.
 */

import { CesiumWorld } from './cesium-world.js';
import { TilesCollisionProvider } from './tiles-collision.js';
import { Controller } from './controller.js';
import { Drone } from './drone.js';
import { HUD } from './hud.js';
import { OSD } from './osd.js';

let world = null;
let collisionProvider = null;
let drone = null;
let controller = null;
let hud = null;
let osd = null;

let mode = 'loading'; // loading | placement | view-select | flight
let cameraMode = 'first'; // first | third
let spawnPoint = null;
let spawnAltitudeMeters = 100;
let sceneLoaded = false;
let loopStarted = false;
let lastFrameTime = 0;
let placementKeysDown = new Set();
let screenHandler = null;
let spawnConfirmInProgress = false;
let thirdPersonPointer = {
    active: false,
    button: -1,
    x: 0,
    y: 0,
};
let thirdPersonCamera = {
    yaw: 0,
    pitch: 0.28,
    distance: 10,
    height: 0.7,
    lateral: 0,
};

const SPAWN_ALTITUDE_MIN = 0;
const SPAWN_ALTITUDE_MAX = 20000;
const SPAWN_ALTITUDE_SLIDER_DEFAULT_MAX = 1000;

function normalizeViewMode(value, fallback = 'first') {
    return value === 'third' || value === '3rd' ? 'third' : fallback;
}

function clampSpawnAltitude(value) {
    const n = Number(value);
    if (!Number.isFinite(n)) return spawnAltitudeMeters;
    return Math.max(SPAWN_ALTITUDE_MIN, Math.min(SPAWN_ALTITUDE_MAX, n));
}

function setSpawnAltitude(value, updateMarker = true) {
    spawnAltitudeMeters = clampSpawnAltitude(value);
    if (spawnPoint) {
        spawnPoint.y = spawnAltitudeMeters;
        if (updateMarker) world?.updateSpawnMarker(spawnPoint);
    }
    syncSpawnAltitudeControls();
    updateSpawnUI();
}

function syncSpawnAltitudeControls() {
    const slider = document.getElementById('spawn-altitude-range');
    const input = document.getElementById('spawn-altitude-input');
    const value = Math.round(spawnAltitudeMeters * 10) / 10;

    if (slider) {
        const neededMax = Math.max(SPAWN_ALTITUDE_SLIDER_DEFAULT_MAX, Math.ceil(value / 100) * 100);
        slider.max = String(Math.min(SPAWN_ALTITUDE_MAX, neededMax));
        slider.value = String(Math.min(Number(slider.max), value));
    }
    if (input) input.value = String(value);
}

function setProgress(message, isError = false) {
    const el = document.getElementById('loading-progress');
    if (!el) return;
    el.textContent = message;
    el.style.color = isError ? '#f44' : '#4272F5';
}

function showError(error) {
    console.error(error);
    const msg = error && error.message ? error.message : String(error);
    setProgress(msg, true);
    document.getElementById('loading-overlay')?.classList.add('visible');
}

function initSubsystems() {
    if (controller && drone && hud && osd) return;

    if (!window.pc) {
        throw new Error('PlayCanvas math library is not loaded. Check network access to cdn.jsdelivr.net.');
    }

    controller = new Controller();
    drone = new Drone();
    hud = new HUD();
    osd = new OSD('osd-canvas');

    setupDisplaySettingsListeners();
}

async function startTilesMode() {
    try {
        initSubsystems();
        document.getElementById('drop-zone')?.classList.add('hidden');
        document.getElementById('loading-overlay')?.classList.add('visible');
        setProgress('Starting Google 3D Tiles world...');

        if (screenHandler) {
            screenHandler.destroy();
            screenHandler = null;
        }
        if (world) world.destroy();
        world = new CesiumWorld('cesium-container');
        await world.init(setProgress);
        collisionProvider = new TilesCollisionProvider(world);
        sceneLoaded = true;

        setupCesiumPlacementHandler();
        setupThirdPersonPointerControls();
        await enterPlacementMode(true);
        document.getElementById('loading-overlay')?.classList.remove('visible');

        if (!loopStarted) {
            loopStarted = true;
            lastFrameTime = performance.now();
            requestAnimationFrame(gameLoop);
        }
    } catch (e) {
        showError(e);
    }
}

function setupCesiumPlacementHandler() {
    if (!world || !world.viewer || screenHandler) return;
    const Cesium = world.Cesium;
    screenHandler = new Cesium.ScreenSpaceEventHandler(world.viewer.scene.canvas);
    screenHandler.setInputAction(async (movement) => {
        if (mode !== 'placement') return;
        const picked = await world.pickSpawn(movement.position, spawnAltitudeMeters);
        if (picked) {
            spawnPoint = picked;
            setSpawnAltitude(spawnAltitudeMeters);
            updateSpawnUI();
        }
    }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
}

async function enterPlacementMode(autoPick = false) {
    if (!world) return;
    mode = 'placement';

    world.setNativeCameraControls(true);
    world.showAircraft(false);
    thirdPersonPointer.active = false;
    hud?.hide();
    document.getElementById('game-logo')?.classList.remove('visible');
    document.getElementById('key-guide')?.classList.remove('visible');
    document.getElementById('placement-overlay')?.classList.add('visible');
    document.getElementById('view-choice-overlay')?.classList.remove('visible');
    applyDisplaySettings();

    if (autoPick || !spawnPoint) {
        await new Promise(resolve => setTimeout(resolve, 300));
        const canvas = world.viewer.scene.canvas;
        const center = new world.Cesium.Cartesian2(canvas.clientWidth * 0.5, canvas.clientHeight * 0.56);
        spawnPoint = await world.pickSpawn(center, spawnAltitudeMeters);
        if (!spawnPoint) {
            spawnPoint = { x: 0, y: spawnAltitudeMeters, z: 0 };
            world.updateSpawnMarker(spawnPoint);
        }
    } else {
        spawnPoint.y = spawnAltitudeMeters;
        world.updateSpawnMarker(spawnPoint);
    }
    syncSpawnAltitudeControls();
    updateSpawnUI();

}

async function confirmSpawnAndFly() {
    if (!world || !spawnPoint || spawnConfirmInProgress) return;
    spawnConfirmInProgress = true;

    try {
        const Cesium = world.Cesium;
        const spawnCarto = world.localToCartographic({ x: spawnPoint.x, y: 0, z: spawnPoint.z });
        const origin = new Cesium.Cartographic(
            spawnCarto.longitude,
            spawnCarto.latitude,
            0
        );
        const spawnAltitude = clampSpawnAltitude(spawnAltitudeMeters);
        world.setOrigin(origin);
        spawnPoint = { x: 0, y: spawnAltitude, z: 0 };

        world.setNativeCameraControls(false);
        world.hideSpawnMarker();
        document.getElementById('placement-overlay')?.classList.remove('visible');
        const coordsEl = document.getElementById('spawn-coords');
        if (coordsEl) coordsEl.style.display = 'none';

        drone.setSpawnPoint(spawnPoint.x, spawnPoint.y, spawnPoint.z);
        drone.reset();
        controller.armed = true;

        mode = 'loading';
        applyDisplaySettings();
        document.getElementById('loading-overlay')?.classList.add('visible');
        try {
            await world.preloadLocalArea(spawnPoint, {
                radius: 130,
                lift: 95,
                progressCb: setProgress,
            });
        } catch (e) {
            console.warn('[TilesFlight] tile preload failed; continuing:', e);
        }

        mode = 'view-select';
        document.getElementById('view-choice-overlay')?.classList.add('visible');
        applyDisplaySettings();
    } catch (e) {
        console.error('[TilesFlight] failed to confirm spawn:', e);
        const msg = e && e.message ? e.message : String(e);
        setProgress(`Spawn failed: ${msg}`, true);
        try {
            await enterPlacementMode(false);
        } catch (restoreError) {
            console.warn('[TilesFlight] failed to restore placement mode:', restoreError);
        }
    } finally {
        document.getElementById('loading-overlay')?.classList.remove('visible');
        spawnConfirmInProgress = false;
    }
}

function startFlight(viewMode = 'first') {
    if (!world || !drone || !controller) return;
    cameraMode = normalizeViewMode(viewMode, 'first');

    mode = 'flight';
    document.getElementById('view-choice-overlay')?.classList.remove('visible');
    document.getElementById('game-logo')?.classList.add('visible');
    hud?.show();

    const transform = drone.getBodyTransform ? drone.getBodyTransform() : drone.getCameraTransform();
    if (cameraMode === 'third') {
        initThirdPersonCamera(transform);
        world.updateAircraftFromDroneTransform(transform);
        world.showAircraft(true);
    } else {
        world.showAircraft(false);
    }

    applyDisplaySettings();
}

function initThirdPersonCamera(transform) {
    const forward = world.getForwardLocal(transform);
    thirdPersonCamera.yaw = Math.atan2(-forward.x, -forward.z);
    thirdPersonCamera.pitch = 0.45;
    thirdPersonCamera.distance = 16;
    thirdPersonCamera.height = 1.2;
    thirdPersonCamera.lateral = 0;
}

function updateSpawnUI() {
    const coordsEl = document.getElementById('spawn-coords');
    if (coordsEl && world && spawnPoint) {
        coordsEl.style.display = 'block';
        coordsEl.textContent = `Spawn: ${world.describeSpawn(spawnPoint, spawnAltitudeMeters)}`;
    }
}

function moveSpawn(dt) {
    if (mode !== 'placement' || !spawnPoint || !world) return;
    const fast = placementKeysDown.has('ShiftLeft') || placementKeysDown.has('ShiftRight');
    const speed = (fast ? 25 : 6) * dt;
    const heading = world.viewer.camera.heading || 0;
    const fwd = { x: Math.sin(heading), z: Math.cos(heading) };
    const right = { x: Math.cos(heading), z: -Math.sin(heading) };

    if (placementKeysDown.has('KeyW')) {
        spawnPoint.x += fwd.x * speed;
        spawnPoint.z += fwd.z * speed;
    }
    if (placementKeysDown.has('KeyS')) {
        spawnPoint.x -= fwd.x * speed;
        spawnPoint.z -= fwd.z * speed;
    }
    if (placementKeysDown.has('KeyD')) {
        spawnPoint.x += right.x * speed;
        spawnPoint.z += right.z * speed;
    }
    if (placementKeysDown.has('KeyA')) {
        spawnPoint.x -= right.x * speed;
        spawnPoint.z -= right.z * speed;
    }
    spawnPoint.y = spawnAltitudeMeters;

    world.updateSpawnMarker(spawnPoint);
    updateSpawnUI();
}

function getCameraHFov() {
    const el = document.getElementById('cam-hfov');
    const v = el ? parseFloat(el.value) : 120;
    return Number.isFinite(v) ? v : 120;
}

function gameLoop(now) {
    const dt = Math.min(0.05, Math.max(0.001, (now - lastFrameTime) / 1000));
    lastFrameTime = now;

    try {
        if (mode === 'placement') {
            moveSpawn(dt);
            updateKeyGuide();
        } else if (mode === 'view-select') {
            updateKeyGuide();
        } else if (mode === 'flight') {
            updateFlight(dt);
        }
    } catch (e) {
        console.error('[gameLoop]', e);
    }
    requestAnimationFrame(gameLoop);
}

function updateFlight(dt) {
    if (!drone || !controller || !world) return;

    drone.readSettings();
    const input = controller.update();
    if (input.resetTriggered) {
        drone.reset();
        controller.armed = true;
    }

    if (drone.flightMode === 'drone') {
        if (Math.abs(input.cameraTiltKeyboard) > 0.05) {
            drone.adjustCameraTilt(input.cameraTiltKeyboard * 60 * dt);
        }
        if (input.cameraTiltAxisChanged) {
            drone.cameraTiltAngle = ((input.cameraTiltAxis + 1) / 2) * -90;
        }
    }

    drone.update(dt, input, collisionProvider);

    // Camera mode only selects visualization; controller and physics stay shared.
    const cameraTransform = drone.getCameraTransform();
    const bodyTransform = drone.getBodyTransform ? drone.getBodyTransform() : cameraTransform;
    if (cameraMode === 'third') {
        world.updateAircraftFromDroneTransform(bodyTransform);
        world.showAircraft(true);
        world.setThirdPersonCamera(bodyTransform, thirdPersonCamera);
    } else {
        world.showAircraft(false);
        world.setCameraFromDroneTransform(cameraTransform, getCameraHFov());
    }

    hud?.show();
    hud?.update(drone, controller, null);
    applyDisplaySettings();
    osd?.update(drone, controller);
    updateKeyGuide();
}

function applyDisplaySettings() {
    const cleanToggle = document.getElementById('clean-mode-toggle');
    const cleanMode = cleanToggle ? cleanToggle.checked : false;
    const osdToggle = document.getElementById('osd-toggle');
    if (osd) {
        osd.setEnabled(!cleanMode && (osdToggle ? osdToggle.checked : true) && mode === 'flight' && cameraMode === 'first');
    }

    const logo = document.getElementById('game-logo');
    const keyGuide = document.getElementById('key-guide');
    const hudEl = document.getElementById('hud');
    if (cleanMode) {
        logo?.classList.remove('visible');
        keyGuide?.classList.remove('visible');
        if (hudEl && mode === 'flight') hudEl.classList.add('hidden');
    } else if (mode === 'flight') {
        logo?.classList.add('visible');
        hudEl?.classList.remove('hidden');
    } else if (mode === 'placement' || mode === 'view-select') {
        logo?.classList.remove('visible');
        hudEl?.classList.add('hidden');
    }
}

function setupDisplaySettingsListeners() {
    for (const id of ['clean-mode-toggle', 'osd-toggle']) {
        const el = document.getElementById(id);
        if (!el || el._tilesDisplayBound) continue;
        el._tilesDisplayBound = true;
        el.addEventListener('change', applyDisplaySettings);
    }
}

function setupSpawnAltitudeControls() {
    const slider = document.getElementById('spawn-altitude-range');
    const input = document.getElementById('spawn-altitude-input');
    const panel = document.getElementById('spawn-altitude-panel');
    if (!slider || !input || !panel || panel._spawnAltitudeBound) return;
    panel._spawnAltitudeBound = true;

    const commit = (value) => setSpawnAltitude(value);
    slider.addEventListener('input', () => commit(slider.value));
    input.addEventListener('input', () => {
        if (input.value !== '') commit(input.value);
    });
    input.addEventListener('change', () => commit(input.value));

    panel.addEventListener('wheel', (e) => {
        if (mode !== 'placement') return;
        e.preventDefault();
        e.stopPropagation();
        const step = e.shiftKey ? 25 : 5;
        const direction = e.deltaY < 0 ? 1 : -1;
        commit(spawnAltitudeMeters + direction * step);
    }, { passive: false });

    for (const el of [slider, input]) {
        el.addEventListener('pointerdown', (e) => e.stopPropagation());
        el.addEventListener('keydown', (e) => e.stopPropagation());
    }
    syncSpawnAltitudeControls();
}

function updateKeyGuide() {
    const el = document.getElementById('key-guide');
    if (!el) return;
    if (mode !== 'flight') {
        el.classList.remove('visible');
        return;
    }
    const isFPV = drone && drone.flightMode === 'fpv';
    const title = isFPV ? 'FLIGHT CONTROLS - FPV' : 'FLIGHT CONTROLS - EASY';
    const rows = isFPV ? [
        '<kbd>↑ ↓</kbd>  Pitch Forward / Back',
        '<kbd>← →</kbd>  Roll Left / Right',
        '<kbd>W S</kbd>  Motor Thrust',
        '<kbd>A D</kbd>  Yaw Left / Right',
    ] : [
        '<kbd>↑ ↓</kbd>  Forward / Back',
        '<kbd>← →</kbd>  Strafe Left / Right',
        '<kbd>W S</kbd>  Climb / Descend',
        '<kbd>A D</kbd>  Yaw Left / Right',
        '<kbd>Q E</kbd>  Camera Tilt',
    ];
    rows.push(
        '<kbd>Space</kbd> Arm / Disarm',
        '<kbd>Shift</kbd> Boost',
        '<kbd>R</kbd>    Reset',
        `<kbd>V</kbd>    View (${cameraMode === 'third' ? 'Third' : 'First'})`,
        '<kbd>M</kbd>    Flight Mode (FPV/Easy)',
        '<kbd>P</kbd>    Placement mode',
        '<kbd>Tab</kbd>  Settings',
    );
    if (cameraMode === 'third') {
        rows.push(
            '<kbd>L/R Mouse</kbd> Orbit observer',
            '<kbd>Wheel</kbd> Zoom',
            '<kbd>Middle</kbd> Pan / height',
        );
    }
    el.innerHTML = `<div class="guide-title">${title}</div>\n${rows.join('\n')}`;
    if (!document.getElementById('clean-mode-toggle')?.checked) {
        el.classList.add('visible');
    }
}

function clampNumber(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function isThirdPersonObserverActive() {
    return mode === 'flight' &&
        cameraMode === 'third' &&
        !(controller && controller.isSettingsOpen && controller.isSettingsOpen());
}

function setupThirdPersonPointerControls() {
    if (!world || !world.viewer) return;
    const canvas = world.viewer.scene.canvas;
    if (!canvas || canvas._mcwfThirdPersonBound) return;
    canvas._mcwfThirdPersonBound = true;

    canvas.addEventListener('contextmenu', (e) => {
        if (isThirdPersonObserverActive()) e.preventDefault();
    });

    canvas.addEventListener('pointerdown', (e) => {
        if (!isThirdPersonObserverActive()) return;
        if (![0, 1, 2].includes(e.button)) return;
        e.preventDefault();
        thirdPersonPointer.active = true;
        thirdPersonPointer.button = e.button;
        thirdPersonPointer.x = e.clientX;
        thirdPersonPointer.y = e.clientY;
        try { canvas.setPointerCapture(e.pointerId); } catch (_) {}
    });

    canvas.addEventListener('pointermove', (e) => {
        if (!thirdPersonPointer.active || !isThirdPersonObserverActive()) return;
        e.preventDefault();
        const dx = e.clientX - thirdPersonPointer.x;
        const dy = e.clientY - thirdPersonPointer.y;
        thirdPersonPointer.x = e.clientX;
        thirdPersonPointer.y = e.clientY;

        if (thirdPersonPointer.button === 1) {
            thirdPersonCamera.lateral = clampNumber(thirdPersonCamera.lateral + dx * 0.025, -25, 25);
            thirdPersonCamera.height = clampNumber(thirdPersonCamera.height - dy * 0.025, -8, 20);
        } else {
            thirdPersonCamera.yaw -= dx * 0.005;
            thirdPersonCamera.pitch = clampNumber(thirdPersonCamera.pitch - dy * 0.004, -0.75, 1.05);
        }
    });

    const stopPointer = () => {
        thirdPersonPointer.active = false;
        thirdPersonPointer.button = -1;
    };
    canvas.addEventListener('pointerup', stopPointer);
    canvas.addEventListener('pointercancel', stopPointer);
    canvas.addEventListener('pointerleave', stopPointer);

    canvas.addEventListener('wheel', (e) => {
        if (!isThirdPersonObserverActive()) return;
        e.preventDefault();
        thirdPersonCamera.distance = clampNumber(
            thirdPersonCamera.distance * Math.exp(e.deltaY * 0.001),
            2.0,
            120.0
        );
    }, { passive: false });
}

function setupKeyboard() {
    window.addEventListener('keydown', (e) => {
        if (controller && controller.isSettingsOpen && controller.isSettingsOpen()) return;
        if (e.target && e.target.closest && e.target.closest('#spawn-altitude-panel')) return;

        if (mode === 'placement') {
            placementKeysDown.add(e.code);
            if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'Enter'].includes(e.code)) {
                e.preventDefault();
            }
            if (e.code === 'Enter' && spawnPoint) {
                confirmSpawnAndFly();
            }
        } else if (mode === 'view-select') {
            if (['Digit1', 'Numpad1', 'Enter'].includes(e.code)) {
                e.preventDefault();
                startFlight('first');
            } else if (['Digit2', 'Numpad2'].includes(e.code)) {
                e.preventDefault();
                startFlight('third');
            } else if (e.code === 'Escape' || e.code === 'KeyP') {
                e.preventDefault();
                enterPlacementMode(false);
            }
        } else if (mode === 'flight') {
            if (e.code === 'KeyV') {
                e.preventDefault();
                cameraMode = cameraMode === 'third' ? 'first' : 'third';
                if (cameraMode === 'third') initThirdPersonCamera(drone.getBodyTransform ? drone.getBodyTransform() : drone.getCameraTransform());
                applyDisplaySettings();
                return;
            }
            if (e.code === 'KeyP') {
                e.preventDefault();
                enterPlacementMode(false);
            }
            if (e.code === 'Escape' && sceneLoaded) {
                e.preventDefault();
                if (window.confirm('Return to placement mode?')) enterPlacementMode(false);
            }
        }
    }, true);

    window.addEventListener('keyup', (e) => {
        placementKeysDown.delete(e.code);
    }, true);
    window.addEventListener('blur', () => placementKeysDown.clear());
}

function setupStartUI() {
    const startBtn = document.getElementById('file-picker-btn');
    const dropZone = document.getElementById('drop-zone');
    if (startBtn) {
        startBtn.textContent = 'Start Google 3D Tiles Flight';
        startBtn.addEventListener('click', () => startTilesMode());
    }
    if (dropZone) {
        dropZone.addEventListener('dragover', (e) => {
            e.preventDefault();
            dropZone.classList.add('dragover');
        });
        dropZone.addEventListener('dragleave', () => dropZone.classList.remove('dragover'));
        dropZone.addEventListener('drop', (e) => {
            e.preventDefault();
            dropZone.classList.remove('dragover');
            startTilesMode();
        });
    }

    for (const btn of document.querySelectorAll('[data-view-choice]')) {
        if (btn._mcwfViewChoiceBound) continue;
        btn._mcwfViewChoiceBound = true;
        btn.addEventListener('click', () => startFlight(btn.getAttribute('data-view-choice')));
    }
}

document.addEventListener('DOMContentLoaded', () => {
    setupStartUI();
    setupKeyboard();
    setupSpawnAltitudeControls();
    setProgress('');
});
