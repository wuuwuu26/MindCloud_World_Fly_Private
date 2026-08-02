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

import { CesiumWorld } from './cesium-world.js?v=20260703-panorama-tile-idle';
import { TilesCollisionProvider } from './tiles-collision.js';
import { Controller } from './controller.js';
import { Drone } from './drone.js';
import { HUD } from './hud.js';
import { OSD } from './osd.js';
import { PanoramaSensor } from './panorama-sensor.js';
import { YOPONavigator } from './yopo-navigator.js';
import { YOPODepthFromPanorama } from './yopo-depth-from-panorama.js';
import { reportUserError } from './error-report.js';

let world = null;
let collisionProvider = null;
let drone = null;
let controller = null;
let hud = null;
let osd = null;
let panoramaSensor = null;
let yopoNavigator = null;
let yopoDepthFromPanorama = null;

let mode = 'loading'; // loading | placement | view-select | flight
let cameraMode = 'first'; // first | third
let spawnPoint = null;
let spawnAltitudeMeters = 100;
let sceneLoaded = false;
let loopStarted = false;
let lastFrameTime = 0;
let placementKeysDown = new Set();
let placementInitClickUntil = 0;
let screenHandler = null;
let spawnConfirmInProgress = false;
let startTilesModeInProgress = false;
let yopoTargetSelectMode = false;
let yopoTargetMarker = null;
let yopoNavInProgress = false;
let yopoControlInProgress = false;
let panoramaWarmupPromise = null;
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
const SPAWN_PRELOAD_RADIUS_METERS = Math.round(urlNumber('flightPreloadRadius', 420, 120, 2000));
const FLIGHT_PRELOAD_MIN_COVERAGE = urlNumber('flightPreloadMinCoverage', 0.95, 0.5, 1);
const FLIGHT_PRELOAD_VIEW_TIMEOUT_MS = Math.round(urlNumber('flightPreloadViewTimeoutMs', 20000, 3000, 60000));
const FLIGHT_PRELOAD_VIEW_ATTEMPTS = Math.round(urlNumber('flightPreloadViewAttempts', 2, 1, 5));
const FLIGHT_PRELOAD_STRICT = urlNumber('flightPreloadStrict', 0, 0, 1) >= 0.5;
const PANORAMA_PRELOAD_REQUIRED = urlNumber('panoPreloadRequired', 0, 0, 1) >= 0.5;
const VIEW_CHOICE_HINT_HTML = '1 / O: First Person &nbsp;|&nbsp; 2: Third Person<br>Easy speed: ↑/↓ forward/back, Shift boost, Tab &gt; Easy Max Speed';
const MAX_PHYSICS_FRAME_DT = 0.25;
const PHYSICS_SUBSTEP_DT = 0.05;
const MAX_PHYSICS_SUBSTEPS = 3;
const SETTINGS_READ_INTERVAL_MS = 100;

let lastSettingsReadTime = 0;
let lastKeyGuideState = '';
let lastDisplaySettingsState = '';
let lastHFovReadTime = 0;
let cachedHFov = 120;
let flightStartWarnings = [];

function urlNumber(name, fallback, min = -Infinity, max = Infinity) {
    const value = new URLSearchParams(window.location.search).get(name);
    if (value == null || value === '') return fallback;
    const n = Number(value);
    if (!Number.isFinite(n)) return fallback;
    return Math.max(min, Math.min(max, n));
}

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

function escapeHtml(value) {
    return String(value)
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;')
        .replaceAll("'", '&#039;');
}

function shortStatusMessage(value, maxLength = 96) {
    const message = value && value.message ? value.message : String(value || '');
    if (message.length <= maxLength) return message;
    return `${message.slice(0, maxLength - 3)}...`;
}

function rememberFlightStartWarning(message) {
    const text = String(message || '').trim();
    if (!text || flightStartWarnings.includes(text)) return;
    flightStartWarnings.push(text);
}

function updateViewChoiceHint() {
    const el = document.getElementById('view-choice-hint');
    if (!el) return;
    if (!flightStartWarnings.length) {
        el.innerHTML = VIEW_CHOICE_HINT_HTML;
        return;
    }
    const warnings = flightStartWarnings
        .map(message => escapeHtml(message))
        .join('<br>');
    el.innerHTML = `${VIEW_CHOICE_HINT_HTML}<br><span style="color:#fbbf24">Preload warning: ${warnings}. Tiles may continue loading after takeoff.</span>`;
}

function showError(error) {
    reportUserError('Startup failed', error, { overlay: true, intervalMs: 0 });
}

function withTimeout(promise, timeoutMs, label) {
    if (!Number.isFinite(timeoutMs) || timeoutMs <= 0) return promise;
    let timeout = null;
    return Promise.race([
        promise,
        new Promise((_, reject) => {
            timeout = window.setTimeout(() => reject(new Error(`${label} timed out`)), timeoutMs);
        }),
    ]).finally(() => {
        if (timeout !== null) window.clearTimeout(timeout);
    });
}

async function waitForCesiumReady(timeoutMs = 15000) {
    if (window.Cesium) return;
    if (!window.googleTilesCesiumReady || typeof window.googleTilesCesiumReady.then !== 'function') return;
    let timeout = null;
    try {
        await Promise.race([
            window.googleTilesCesiumReady,
            new Promise((_, reject) => {
                timeout = window.setTimeout(() => reject(new Error('Timed out loading CesiumJS.')), timeoutMs);
            }),
        ]);
    } finally {
        if (timeout !== null) window.clearTimeout(timeout);
    }
}

function initSubsystems() {
    if (controller && drone && hud && osd && panoramaSensor) return;

    if (!window.pc) {
        throw new Error('PlayCanvas math library is not loaded. Check network access to cdn.jsdelivr.net.');
    }

    controller = new Controller();
    drone = new Drone();
    hud = new HUD();
    osd = new OSD('osd-canvas');
    panoramaSensor = new PanoramaSensor();
    yopoNavigator = new YOPONavigator();

    setupDisplaySettingsListeners();
    setupYOPOUI();
    yopoDepthFromPanorama = null;
}

export async function startTilesMode() {
    if (startTilesModeInProgress) return;
    startTilesModeInProgress = true;
    try {
        initSubsystems();
        document.getElementById('drop-zone')?.classList.add('hidden');
        document.getElementById('loading-overlay')?.classList.add('visible');
        setProgress('Starting Google 3D Tiles world...');
        await waitForCesiumReady();

        if (screenHandler) {
            screenHandler.destroy();
            screenHandler = null;
        }
        if (world) world.destroy();
        panoramaWarmupPromise = null;
        world = new CesiumWorld('cesium-container');
        await world.init(setProgress);
        collisionProvider = new TilesCollisionProvider(world);
        sceneLoaded = true;
        yopoDepthFromPanorama = new YOPODepthFromPanorama(world, panoramaSensor);

        setupCesiumPlacementHandler();
        setupThirdPersonPointerControls();
        await enterPlacementMode(true);
        warmPanoramaViewerInBackground();
        document.getElementById('loading-overlay')?.classList.remove('visible');

        if (!loopStarted) {
            loopStarted = true;
            lastFrameTime = performance.now();
            requestAnimationFrame(gameLoop);
        }
    } catch (e) {
        showError(e);
    } finally {
        startTilesModeInProgress = false;
    }
}

function warmPanoramaViewerInBackground() {
    if (!world || !panoramaSensor || panoramaWarmupPromise) return panoramaWarmupPromise;
    if (typeof world.warmPanoramaCaptureViewer !== 'function') return null;

    const options = typeof panoramaSensor.getCaptureOptions === 'function'
        ? panoramaSensor.getCaptureOptions({ preload: true })
        : { faceSize: 256 };
    panoramaWarmupPromise = world.warmPanoramaCaptureViewer(options.faceSize)
        .catch((error) => {
            reportUserError('Panorama viewer warmup failed', error, {
                key: 'panorama-warmup',
                intervalMs: 10000,
            });
            panoramaWarmupPromise = null;
            return false;
        });
    return panoramaWarmupPromise;
}

async function preloadPanoramaBeforeFlight() {
    if (
        !world ||
        !drone ||
        !panoramaSensor ||
        typeof world.preloadPanoramaAtTransform !== 'function' ||
        typeof panoramaSensor.getCaptureOptions !== 'function'
    ) {
        return false;
    }

    const transform = drone.getPanoramaTransform
        ? drone.getPanoramaTransform()
        : (drone.getBodyTransform ? drone.getBodyTransform() : drone.getCameraTransform());
    if (!transform) return false;

    const options = {
        ...panoramaSensor.getCaptureOptions({ preload: true }),
        progressCb: (message) => setProgress(`Preloading 360 panorama sensor (${message})...`),
    };
    const started = performance.now();
    setProgress('Preloading 360 panorama sensor before flight...');

    try {
        const result = await withTimeout(
            (async () => {
                const warmup = warmPanoramaViewerInBackground();
                if (warmup) await warmup;
                return world.preloadPanoramaAtTransform(transform, options);
            })(),
            options.timeoutMs,
            '360 panorama preload'
        );
        const ready = panoramaSensor.primeFromCaptureResult(result, performance.now() - started);
        if (!ready && (PANORAMA_PRELOAD_REQUIRED || FLIGHT_PRELOAD_STRICT)) {
            throw new Error('360 panorama preload did not produce a complete frame.');
        }
        return ready;
    } catch (error) {
        if (PANORAMA_PRELOAD_REQUIRED || FLIGHT_PRELOAD_STRICT) throw error;
        reportUserError('Panorama preload failed; live capture will retry in flight', error, {
            key: 'panorama-preload',
            intervalMs: 10000,
        });
        return false;
    }
}

async function preloadInitialFlightViewsBeforeControl() {
    if (
        !world ||
        !drone ||
        typeof world.settleCurrentCameraView !== 'function'
    ) {
        return;
    }

    const bodyTransform = drone.getBodyTransform ? drone.getBodyTransform() : drone.getCameraTransform();
    const cameraTransform = drone.getCameraTransform();
    const settleOptions = {
        dwellMs: 260,
        timeoutMs: FLIGHT_PRELOAD_VIEW_TIMEOUT_MS,
        quietMs: 650,
    };

    world.setFlightPerformanceMode(true);

    setProgress('Preloading first-person flight view...');
    const firstReady = await settleFlightView('first-person flight view', () => {
        world.setCameraFromDroneTransform(cameraTransform, getCameraHFov());
    }, settleOptions);
    if (!firstReady && FLIGHT_PRELOAD_STRICT) {
        throw new Error('First-person flight view tiles did not finish loading before control.');
    }

    setProgress('Preloading third-person flight view...');
    initThirdPersonCamera(bodyTransform);
    world.updateAircraftFromDroneTransform(bodyTransform);
    world.showAircraft(true);
    let thirdReady = false;
    try {
        thirdReady = await settleFlightView('third-person flight view', () => {
            world.setThirdPersonCamera(bodyTransform, thirdPersonCamera);
        }, settleOptions);
    } finally {
        world.showAircraft(false);
    }
    if (!thirdReady && FLIGHT_PRELOAD_STRICT) {
        throw new Error('Third-person flight view tiles did not finish loading before control.');
    }
}

async function settleFlightView(label, applyView, settleOptions) {
    let ready = false;
    for (let attempt = 1; attempt <= FLIGHT_PRELOAD_VIEW_ATTEMPTS; attempt++) {
        if (attempt > 1) {
            setProgress(`Waiting for ${label} tiles (${attempt}/${FLIGHT_PRELOAD_VIEW_ATTEMPTS})...`);
        }
        applyView();
        ready = await world.settleCurrentCameraView(settleOptions);
        if (ready) return true;
    }
    return ready;
}

function setupCesiumPlacementHandler() {
    if (!world || !world.viewer || screenHandler) return;
    const Cesium = world.Cesium;
    const canvas = world.viewer.scene.canvas;

    const rememberInitClick = () => {
        if (mode !== 'placement' || !placementKeysDown.has('KeyI')) return;
        placementInitClickUntil = performance.now() + 1500;
    };
    canvas.addEventListener('pointerdown', rememberInitClick, true);
    canvas.addEventListener('click', rememberInitClick, true);

    screenHandler = new Cesium.ScreenSpaceEventHandler(world.viewer.scene.canvas);
    screenHandler.setInputAction(async (movement) => {
        if (mode !== 'placement') return;
        const initClickActive =
            placementKeysDown.has('KeyI') ||
            performance.now() <= placementInitClickUntil;
        if (!initClickActive) return;

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

    world.setFlightPerformanceMode(false);
    world.setNativeCameraControls(true);
    world.showAircraft(false);
    thirdPersonPointer.active = false;
    panoramaSensor?.setActive(false);
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
    flightStartWarnings = [];
    updateViewChoiceHint();

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
        panoramaSensor?.reset();

        mode = 'loading';
        applyDisplaySettings();
        document.getElementById('loading-overlay')?.classList.add('visible');
        setProgress(`Preloading ${SPAWN_PRELOAD_RADIUS_METERS} m flight area before control...`);
        try {
            const preload = await world.preloadLocalArea(spawnPoint, {
                radius: SPAWN_PRELOAD_RADIUS_METERS,
                lift: 220,
                gridSpacing: 160,
                viewDistance: 240,
                maxTargets: 22,
                dwellMs: 220,
                perViewTimeoutMs: 3200,
                finalIdleTimeoutMs: 20000,
                verifyCoverage: true,
                coverageSpacing: 160,
                minCoverageRatio: FLIGHT_PRELOAD_MIN_COVERAGE,
                repairPasses: 2,
                repairTargets: 22,
                progressCb: setProgress,
            });
            const coverage = preload && preload.coverage ? preload.coverage.ratio : 0;
            const pct = Math.round(coverage * 100);
            if (preload && preload.coverage && coverage < FLIGHT_PRELOAD_MIN_COVERAGE) {
                reportUserError(
                    'Flight tile preload coverage low',
                    new Error(`coverage ${pct}% below required ${Math.round(FLIGHT_PRELOAD_MIN_COVERAGE * 100)}%`),
                    { key: 'flight-preload-coverage-low', intervalMs: 10000 }
                );
            }
            const coverageReady = preload && preload.coverage
                ? coverage >= FLIGHT_PRELOAD_MIN_COVERAGE
                : preload && preload.finalIdle === true;
            const preloadReady = preload &&
                coverageReady &&
                (!FLIGHT_PRELOAD_STRICT || preload.finalIdle === true);
            if (!preloadReady) {
                const coverageText = preload && preload.coverage ? `${pct}%` : 'unknown';
                const message = `flight tile preload incomplete: idle=${preload ? preload.finalIdle : false}, coverage=${coverageText}`;
                if (FLIGHT_PRELOAD_STRICT) {
                    throw new Error(message);
                }
                rememberFlightStartWarning(message);
            }
        } catch (e) {
            const msg = e && e.message ? e.message : String(e);
            if (FLIGHT_PRELOAD_STRICT) {
                reportUserError('Required flight tile preload failed', e, { intervalMs: 0 });
                throw new Error(`Required flight tile preload failed: ${msg}`);
            }
            reportUserError('Flight tile preload failed; continuing to view selection', e, {
                key: 'flight-tile-preload',
                intervalMs: 10000,
            });
            rememberFlightStartWarning(`flight tile preload skipped: ${shortStatusMessage(msg)}`);
        }

        try {
            await preloadInitialFlightViewsBeforeControl();
        } catch (e) {
            if (FLIGHT_PRELOAD_STRICT) throw e;
            reportUserError('Initial flight view preload failed; continuing', e, {
                key: 'initial-flight-view-preload',
                intervalMs: 10000,
            });
        }

        try {
            await preloadPanoramaBeforeFlight();
        } catch (e) {
            if (PANORAMA_PRELOAD_REQUIRED || FLIGHT_PRELOAD_STRICT) throw e;
            reportUserError('Panorama preload failed; continuing', e, {
                key: 'panorama-preload-before-flight',
                intervalMs: 10000,
            });
        }

        mode = 'view-select';
        updateViewChoiceHint();
        document.getElementById('view-choice-overlay')?.classList.add('visible');
        applyDisplaySettings();
    } catch (e) {
        reportUserError('Spawn failed', e, { overlay: true, intervalMs: 0 });
        try {
            await enterPlacementMode(false);
        } catch (restoreError) {
            reportUserError('Failed to restore placement mode', restoreError, {
                key: 'restore-placement',
                intervalMs: 10000,
            });
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
    drone.readSettings();
    lastSettingsReadTime = performance.now();
    world.setFlightPerformanceMode(true);
    document.getElementById('view-choice-overlay')?.classList.remove('visible');
    document.getElementById('game-logo')?.classList.add('visible');
    hud?.show();
    if (!panoramaSensor?.hasRgbFrame?.()) panoramaSensor?.reset();
    panoramaSensor?.setActive(true);

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
    // 右手系: right = cross(fwd, up) = (-cos h, 0, sin h)
    // 原代码 right=(cos h, 0, -sin h)=cross(up,fwd)=left → D键左移, A键右移(反了)
    const right = { x: -Math.cos(heading), z: Math.sin(heading) };

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

function getCameraHFov(now = performance.now()) {
    if (now - lastHFovReadTime < 250) return cachedHFov;
    lastHFovReadTime = now;
    const el = document.getElementById('cam-hfov');
    const v = el ? parseFloat(el.value) : 120;
    cachedHFov = Number.isFinite(v) ? v : 120;
    return cachedHFov;
}

function gameLoop(now) {
    const frameDt = Math.min(MAX_PHYSICS_FRAME_DT, Math.max(0.001, (now - lastFrameTime) / 1000));
    lastFrameTime = now;

    try {
        if (mode === 'placement') {
            moveSpawn(Math.min(PHYSICS_SUBSTEP_DT, frameDt));
            updateKeyGuide();
        } else if (mode === 'view-select') {
            updateKeyGuide();
        } else if (mode === 'flight') {
            updateFlight(frameDt);
        }
    } catch (e) {
        reportUserError('Frame update failed', e, {
            key: 'game-loop',
            intervalMs: 3000,
        });
    }
    requestAnimationFrame(gameLoop);
}

function updateFlight(dt) {
    if (!drone || !controller || !world) return;

    const now = performance.now();
    const input = controller.update();
    const modeSelect = document.getElementById('flight-mode-select');
    if (
        now - lastSettingsReadTime >= SETTINGS_READ_INTERVAL_MS ||
        (modeSelect && modeSelect.value !== drone.flightMode)
    ) {
        drone.readSettings();
        lastSettingsReadTime = now;
    }
    if (input.resetTriggered) {
        drone.reset();
        controller.armed = true;
    }

    if (drone.flightMode === 'drone' || drone.flightMode === 'simpleflight') {
        if (Math.abs(input.cameraTiltKeyboard) > 0.05) {
            drone.adjustCameraTilt(input.cameraTiltKeyboard * 60 * dt);
        }
        if (input.cameraTiltAxisChanged) {
            drone.cameraTiltAngle = ((input.cameraTiltAxis + 1) / 2) * -90;
        }
    }

    let remainingDt = Math.max(0, Math.min(dt, PHYSICS_SUBSTEP_DT * MAX_PHYSICS_SUBSTEPS));
    let substeps = 0;
    while (remainingDt > 1e-6 && substeps < MAX_PHYSICS_SUBSTEPS) {
        const stepDt = Math.min(PHYSICS_SUBSTEP_DT, remainingDt);
        drone.update(stepDt, input, collisionProvider);
        remainingDt -= stepDt;
        substeps++;
    }

    // ---- YOPO 导航更新 (深度/控制分离) ----
    // 模仿 YOPO 原始架构: control_pub(50Hz) + callback_depth(30Hz重规划)
    //   - 控制环 (~60Hz, 每渲染帧): /yopo/control 推进 ctrl_time, 评估多项式
    //   - 深度环 (~0.4Hz, 深度到达时): /yopo/navigate 重新推理, 重建多项式
    // 两者独立, 互不阻塞。控制命令始终新鲜, 无人机不盲飞。
    if (drone.flightMode === 'yopo_nav' && drone.yopoNavActive && yopoNavigator && !drone.yopoArrived) {
        const pos = { x: drone.x, y: drone.y, z: drone.z };
        const vel = { x: drone.vx, y: drone.vy, z: drone.vz };
        const orient = {
            x: drone.orientation.x,
            y: drone.orientation.y,
            z: drone.orientation.z,
            w: drone.orientation.w,
        };

        // ── 控制环 (高频, 每帧) ──
        // 不带深度, 用上次多项式推进 ctrl_time, 返回 poly(ctrl_time) 的 pos/vel/acc/yaw。
        // 一次只允许一个 control 请求在途, HTTP 往返自然限频 (~50-100Hz)。
        if (!yopoControlInProgress) {
            yopoControlInProgress = true;
            (async () => {
                try {
                    const cmd = await yopoNavigator.control(pos, vel, orient);
                    if (cmd && !cmd.error) {
                        drone.yopoCmdPos = cmd.position;
                        drone.yopoCmdVel = cmd.velocity;
                        drone.yopoCmdAcc = cmd.acceleration;
                        drone.yopoCmdYaw = cmd.yaw;
                        drone.yopoCmdYawDot = cmd.yaw_dot || 0;
                        drone.yopoCmdTime = performance.now();
                        drone.yopoArrived = cmd.arrived || false;
                        drone.yopoDistToGoal = cmd.dist_to_goal || 0;
                    }
                } catch (e) {
                    // 高频调用, 静默处理瞬态错误
                }
                yopoControlInProgress = false;
            })();
        }

        // ── 深度环 (低频, 深度到达时) ──
        // 带深度+odom, 运行 YOPO 推理, 重建多项式, 重置 ctrl_time=0。
        // 一次只允许一个 navigate 请求在途, 与控制环并行不阻塞。
        if (!yopoNavInProgress) {
            yopoNavInProgress = true;
            if (drone.yopoInferenceCount === 0) {
                console.log('YOPO nav loop: starting first inference cycle');
            }
            (async () => {
                try {
                    const t0 = performance.now();
                    const cameraTransform = drone.getCameraTransform();
                    let depthResult = null;

                    // Prefer DA360 ERP panoramic depth (YOPO_360 native input).
                    if (yopoDepthFromPanorama) {
                        depthResult = await yopoDepthFromPanorama.captureYOPODepthERP(cameraTransform, {
                            width: 384,   // YOPO_360 ERP image_width  (columns)
                            height: 192,  // YOPO_360 ERP image_height (rows)
                            maxDistance: 20,
                            timeoutMs: 6000,
                        });
                    }
                    const t1 = performance.now();
                    if (!depthResult) {
                        if (drone.yopoInferenceCount < 3 || drone.yopoInferenceCount % 30 === 0) {
                            console.warn('YOPO: DA360 ERP depth capture failed, using Cesium ray fallback');
                        }
                        depthResult = world.captureForwardDepth(cameraTransform, {
                            width: 384,
                            height: 192,
                            gridCols: 24,
                            gridRows: 12,
                            hfovDeg: 90,
                            maxDistance: 20,
                        });
                    }

                    if (!depthResult || !depthResult.depth) {
                        throw new Error('depth capture failed');
                    }

                    const cmd = await yopoNavigator.navigate(
                        depthResult.depth,
                        depthResult.encoding,
                        pos,
                        vel,
                        orient,
                        depthResult.mask
                    );
                    const t2 = performance.now();
                    if (drone.yopoInferenceCount < 5 || drone.yopoInferenceCount % 20 === 0) {
                        console.log(`YOPO timing: depth=${(t1-t0).toFixed(0)}ms navigate=${(t2-t1).toFixed(0)}ms total=${(t2-t0).toFixed(0)}ms`);
                    }

                    if (cmd && !cmd.error) {
                        // navigate 返回 ctrl_time=0 处的命令; 控制环下一帧会推进
                        // ctrl_time 并覆盖。首次推理后立即更新避免悬停等待。
                        drone.yopoCmdPos = cmd.position;
                        drone.yopoCmdVel = cmd.velocity;
                        drone.yopoCmdAcc = cmd.acceleration;
                        drone.yopoCmdYaw = cmd.yaw;
                        drone.yopoCmdYawDot = cmd.yaw_dot || 0;
                        drone.yopoCmdTime = performance.now();
                        drone.yopoArrived = cmd.arrived || false;
                        drone.yopoDistToGoal = cmd.dist_to_goal || 0;
                        if (drone.yopoInferenceCount < 5 || drone.yopoInferenceCount % 30 === 0) {
                            const dx = cmd.position.x - drone.x;
                            const dy = cmd.position.y - drone.y;
                            const dz = cmd.position.z - drone.z;
                            const posErrMag = Math.sqrt(dx*dx + dy*dy + dz*dz);
                            console.log(`YOPO #${drone.yopoInferenceCount}: cmd_pos=(${cmd.position.x.toFixed(1)},${cmd.position.y.toFixed(1)},${cmd.position.z.toFixed(1)}) ` +
                                `drone_pos=(${drone.x.toFixed(1)},${drone.y.toFixed(1)},${drone.z.toFixed(1)}) ` +
                                `posErr=(${dx.toFixed(2)},${dy.toFixed(2)},${dz.toFixed(2)}) mag=${posErrMag.toFixed(2)} ` +
                                `cmd_vel=(${cmd.velocity.x.toFixed(2)},${cmd.velocity.y.toFixed(2)},${cmd.velocity.z.toFixed(2)}) ` +
                                `cmd_acc=(${cmd.acceleration.x.toFixed(2)},${cmd.acceleration.y.toFixed(2)},${cmd.acceleration.z.toFixed(2)})`);
                        }
                    } else if (cmd && cmd.error) {
                        if (drone.yopoInferenceCount < 3 || drone.yopoInferenceCount % 30 === 0) {
                            console.warn('YOPO server error:', cmd.error);
                        }
                    }
                } catch (e) {
                    // Silently handle YOPO errors during flight
                    if (drone.yopoInferenceCount % 30 === 0) {
                        console.warn('YOPO navigation error:', e);
                    }
                }
                drone.yopoInferenceCount++;
                yopoNavInProgress = false;
            })();
        }
    }

    // Locally compute distance to goal every frame (independent of server
    // response) so the UI always reflects the true distance.
    if (drone.yopoNavTarget && drone.flightMode === 'yopo_nav') {
        const dx = drone.yopoNavTarget.x - drone.x;
        const dy = drone.yopoNavTarget.y - drone.y;
        const dz = drone.yopoNavTarget.z - drone.z;
        drone.yopoDistToGoal = Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Camera mode only selects visualization; controller and physics stay shared.
    const cameraTransform = drone.getCameraTransform();
    const bodyTransform = drone.getBodyTransform ? drone.getBodyTransform() : cameraTransform;
    if (cameraMode === 'third') {
        world.updateAircraftFromDroneTransform(bodyTransform);
        world.showAircraft(true);
        world.setThirdPersonCamera(bodyTransform, thirdPersonCamera);
    } else {
        world.showAircraft(false);
        world.setCameraFromDroneTransform(cameraTransform, getCameraHFov(now));
    }

    const panoramaTransform = drone.getPanoramaTransform ? drone.getPanoramaTransform() : bodyTransform;
    panoramaSensor?.update(world, panoramaTransform, now);
    hud?.update(drone, controller, null);
    applyDisplaySettings();
    osd?.update(drone, controller);
    updateKeyGuide();
    updateYOPOStatusUI();
}

function applyDisplaySettings() {
    const cleanToggle = document.getElementById('clean-mode-toggle');
    const cleanMode = cleanToggle ? cleanToggle.checked : false;
    const osdToggle = document.getElementById('osd-toggle');
    const osdEnabled = !cleanMode && (osdToggle ? osdToggle.checked : true) && mode === 'flight' && cameraMode === 'first';
    const panoToggle = document.getElementById('panorama-toggle');
    const panoEnabled = panoToggle ? panoToggle.checked : true;
    const state = `${mode}|${cameraMode}|${cleanMode ? 1 : 0}|${osdEnabled ? 1 : 0}|${panoEnabled ? 1 : 0}`;
    if (state === lastDisplaySettingsState) return;
    lastDisplaySettingsState = state;

    if (osd) {
        osd.setEnabled(osdEnabled);
    }
    panoramaSensor?.setActive(mode === 'flight');

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
    for (const id of ['clean-mode-toggle', 'osd-toggle', 'panorama-toggle']) {
        const el = document.getElementById(id);
        if (!el || el._tilesDisplayBound) continue;
        el._tilesDisplayBound = true;
        el.addEventListener('change', applyDisplaySettings);
    }
}

function setupYOPOUI() {
    if (yopoNavigator && yopoNavigator._uiBound) return;
    if (!yopoNavigator) return;

    const selectTargetBtn = document.getElementById('yopo-select-target-btn');
    const startNavBtn = document.getElementById('yopo-start-nav-btn');
    const stopNavBtn = document.getElementById('yopo-stop-nav-btn');
    if (!selectTargetBtn || !startNavBtn || !stopNavBtn) return;

    yopoNavigator._uiBound = true;

    // 选取目标点 button: enter keyboard-driven target selection mode.
    // Target starts at the drone's current position; user moves it with
    // arrow keys and presses Enter to confirm (which also auto-starts nav).
    selectTargetBtn.addEventListener('click', () => {
        if (mode !== 'flight' || !drone) return;
        if (yopoTargetSelectMode) return; // already selecting
        yopoTargetSelectMode = true;
        // Initialise target at the drone's current position
        const x = drone.x;
        const y = drone.y;
        const z = drone.z;
        document.getElementById('yopo-target-x').value = x.toFixed(1);
        document.getElementById('yopo-target-y').value = y.toFixed(1);
        document.getElementById('yopo-target-z').value = z.toFixed(1);
        createYOPOTargetMarker(x, y, z);
        document.getElementById('yopo-status-text').textContent =
            '状态: 目标选择模式 (小键盘8/2/4/6/9/3移动, 5确认, 0取消)';
        console.log('YOPO target select mode: starting at drone pos', { x, y, z });
    });

    // Start Navigation button (manual fallback — normally Enter in select mode)
    startNavBtn.addEventListener('click', async () => {
        if (!drone.yopoNavTarget) {
            document.getElementById('yopo-status-text').textContent = '状态: 请先设置目标点';
            return;
        }
        // Check YOPO server connectivity
        const status = await yopoNavigator.getStatus();
        if (!status) {
            document.getElementById('yopo-status-text').textContent = '状态: YOPO服务器未响应 (端口5689)';
            console.warn('YOPO server not reachable at', yopoNavigator.serverUrl);
            return;
        }
        drone.flightMode = 'yopo_nav';
        drone.yopoNavActive = true;
        if (panoramaSensor) {
            panoramaSensor.depthSuppress = true;  // 抑制 UI 深度, 导航环独占 DA360
            panoramaSensor.captureIntervalOverride = 100;  // 10Hz 全景, 释放 GPU 给深度管线
        }
        drone.yopoArrived = false;
        drone.yopoInferenceCount = 0;
        drone.yopoCmdPos = null;
        drone.yopoCmdVel = null;
        drone.yopoCmdTime = 0;
        // Sync the flight mode dropdown
        const modeSelect = document.getElementById('flight-mode-select');
        if (modeSelect) modeSelect.value = 'yopo_nav';
        document.getElementById('yopo-status-text').textContent = '状态: 导航中...';
        document.getElementById('yopo-start-nav-btn').textContent = '导航中...';
        console.log('YOPO navigation started, goal:', drone.yopoNavTarget);
    });

    // Stop Navigation button
    stopNavBtn.addEventListener('click', () => {
        drone.yopoNavActive = false;
        if (panoramaSensor) {
            panoramaSensor.depthSuppress = false;  // 恢复 UI 深度显示
            panoramaSensor.captureIntervalOverride = 0;  // 恢复 60Hz 全景
        }
        drone.yopoCmdPos = null;
        drone.yopoCmdVel = null;
        drone.yopoCmdTime = 0;
        // Switch back to simpleflight or drone mode
        drone.flightMode = 'simpleflight';
        const modeSelect = document.getElementById('flight-mode-select');
        if (modeSelect) modeSelect.value = 'simpleflight';
        document.getElementById('yopo-status-text').textContent = '状态: 已停止';
        document.getElementById('yopo-start-nav-btn').textContent = '开始导航';
        removeYOPOTargetMarker();
    });
}

// ── YOPO target selection helpers ───────────────────────────────

const YOPO_TARGET_STEP = 0.5; // metres per key press

/** Create (or reuse) a Cesium entity marking the YOPO target position. */
function createYOPOTargetMarker(x, y, z) {
    if (!world || !world.viewer) return;
    const Cesium = world.Cesium;
    const position = world.localToCartesian({ x, y, z });
    if (yopoTargetMarker) {
        yopoTargetMarker.position = position;
        yopoTargetMarker.show = true;
    } else {
        yopoTargetMarker = world.viewer.entities.add({
            name: 'yopo-target',
            position,
            point: {
                pixelSize: 16,
                color: Cesium.Color.fromBytes(255, 200, 0, 255), // amber
                outlineColor: Cesium.Color.WHITE,
                outlineWidth: 2,
                disableDepthTestDistance: Number.POSITIVE_INFINITY,
            },
            label: {
                text: 'YOPO TARGET',
                font: '12px sans-serif',
                pixelOffset: new Cesium.Cartesian2(0, -24),
                fillColor: Cesium.Color.fromBytes(255, 200, 0, 255),
                outlineColor: Cesium.Color.BLACK,
                outlineWidth: 2,
                style: Cesium.LabelStyle.FILL_AND_OUTLINE,
                disableDepthTestDistance: Number.POSITIVE_INFINITY,
            },
        });
    }
    world.viewer.scene.requestRender();
}

/** Move the existing target marker to a new position. */
function updateYOPOTargetMarker(x, y, z) {
    if (!yopoTargetMarker || !world || !world.viewer) return;
    yopoTargetMarker.position = world.localToCartesian({ x, y, z });
    world.viewer.scene.requestRender();
}

/** Hide / destroy the target marker. */
function removeYOPOTargetMarker() {
    if (yopoTargetMarker && world && world.viewer) {
        world.viewer.entities.remove(yopoTargetMarker);
    }
    yopoTargetMarker = null;
}

/**
 * Keyboard handler for YOPO target selection.  Called from the main
 * keydown listener (capture phase) when yopoTargetSelectMode is active.
 * Uses the numeric keypad (e.code so NumLock state does not matter).
 *
 *   Numpad 8/2 = forward/back (north/south, -z/+z)
 *   Numpad 4/6 = left/right   (west/east,  -x/+x)
 *   Numpad 9/3 = up/down      (+y/-y)
 *   Numpad 5   = confirm (start navigation)
 *   Numpad 0   = cancel
 *
 * Returns true if the event was consumed.
 */
function handleYOPOKeyDown(e) {
    if (!yopoTargetSelectMode) return false;

    const xInput = document.getElementById('yopo-target-x');
    const yInput = document.getElementById('yopo-target-y');
    const zInput = document.getElementById('yopo-target-z');
    if (!xInput || !yInput || !zInput) return false;

    let x = parseFloat(xInput.value);
    let y = parseFloat(yInput.value);
    let z = parseFloat(zInput.value);
    if (!Number.isFinite(x)) x = 0;
    if (!Number.isFinite(y)) y = 2;
    if (!Number.isFinite(z)) z = 0;

    let consumed = true;
    switch (e.code) {
        case 'Numpad8': case 'NumpadArrowUp':    z -= YOPO_TARGET_STEP; break; // north (-z)
        case 'Numpad2': case 'NumpadArrowDown':  z += YOPO_TARGET_STEP; break; // south (+z)
        case 'Numpad4': case 'NumpadArrowLeft':  x -= YOPO_TARGET_STEP; break; // west  (-x)
        case 'Numpad6': case 'NumpadArrowRight': x += YOPO_TARGET_STEP; break; // east  (+x)
        case 'Numpad9':                          y += YOPO_TARGET_STEP; break; // up    (+y)
        case 'Numpad3':                          y -= YOPO_TARGET_STEP; break; // down  (-y)
        case 'Numpad5': case 'NumpadEnter':
            confirmYOPOTarget(x, y, z);
            break;
        case 'Numpad0': case 'NumpadDecimal': case 'Escape':
            cancelYOPOTarget();
            break;
        default:
            consumed = false;
    }

    if (consumed) {
        xInput.value = x.toFixed(1);
        yInput.value = y.toFixed(1);
        zInput.value = z.toFixed(1);
        // Update marker only; do NOT set drone.yopoNavTarget here —
        // that must only happen in confirmYOPOTarget so the drone
        // doesn't start flying before the user presses Numpad 5.
        updateYOPOTargetMarker(x, y, z);
        e.preventDefault();
        e.stopImmediatePropagation(); // prevent controller from flying
    }
    return consumed;
}

/** Confirm the selected target: set goal on server and auto-start nav. */
async function confirmYOPOTarget(x, y, z) {
    yopoTargetSelectMode = false;
    document.getElementById('yopo-status-text').textContent =
        `状态: 设置目标 (${x.toFixed(1)}, ${y.toFixed(1)}, ${z.toFixed(1)})...`;

    const ok = await yopoNavigator.setGoal(x, y, z);
    if (!ok) {
        document.getElementById('yopo-status-text').textContent = '状态: 设置目标失败';
        removeYOPOTargetMarker();
        return;
    }
    drone.yopoNavTarget = { x, y, z };

    // Check YOPO server connectivity before starting
    const status = await yopoNavigator.getStatus();
    if (!status) {
        document.getElementById('yopo-status-text').textContent =
            '状态: YOPO服务器未响应 (端口5689)';
        console.warn('YOPO server not reachable at', yopoNavigator.serverUrl);
        removeYOPOTargetMarker();
        return;
    }

    // Auto-start navigation
    drone.flightMode = 'yopo_nav';
    drone.yopoNavActive = true;
    if (panoramaSensor) {
        panoramaSensor.depthSuppress = true;  // 抑制 UI 深度, 导航环独占 DA360
        panoramaSensor.captureIntervalOverride = 100;  // 10Hz 全景, 释放 GPU 给深度管线
    }
    drone.yopoArrived = false;
    drone.yopoInferenceCount = 0;
    drone.yopoCmdPos = null;
    drone.yopoCmdVel = null;
    drone.yopoCmdTime = 0;
    const modeSelect = document.getElementById('flight-mode-select');
    if (modeSelect) modeSelect.value = 'yopo_nav';
    document.getElementById('yopo-status-text').textContent = '状态: 导航中...';
    document.getElementById('yopo-start-nav-btn').textContent = '导航中...';
    console.log('YOPO navigation started, goal:', drone.yopoNavTarget);
}

/** Cancel target selection mode. */
function cancelYOPOTarget() {
    yopoTargetSelectMode = false;
    // Clear the temporary target set during selection (not yet confirmed
    // with the server, so no goal to revoke there).
    drone.yopoNavTarget = null;
    drone.yopoDistToGoal = 0;
    removeYOPOTargetMarker();
    document.getElementById('yopo-status-text').textContent = '状态: 已取消目标选择';
}

function updateYOPOStatusUI() {
    if (!drone || !yopoNavigator) return;
    const statusEl = document.getElementById('yopo-status-text');
    const distEl = document.getElementById('yopo-dist-text');
    const countEl = document.getElementById('yopo-count-text');
    if (!statusEl || !distEl || !countEl) return;

    // Show distance during target selection mode too (compute from inputs,
    // NOT from drone.yopoNavTarget which is not set until confirmation).
    if (yopoTargetSelectMode && drone) {
        const tx = parseFloat(document.getElementById('yopo-target-x')?.value);
        const ty = parseFloat(document.getElementById('yopo-target-y')?.value);
        const tz = parseFloat(document.getElementById('yopo-target-z')?.value);
        if (Number.isFinite(tx) && Number.isFinite(ty) && Number.isFinite(tz)) {
            const dx = tx - drone.x, dy = ty - drone.y, dz = tz - drone.z;
            distEl.textContent = `到目标距离: ${Math.sqrt(dx*dx+dy*dy+dz*dz).toFixed(2)} m`;
        }
        return;
    }

    if (drone.flightMode === 'yopo_nav' && drone.yopoNavActive) {
        if (drone.yopoArrived) {
            statusEl.textContent = '状态: 已到达目标 ✓';
            // Remove target marker once arrived
            if (yopoTargetMarker) removeYOPOTargetMarker();
        } else {
            statusEl.textContent = '状态: 导航中...';
        }
        distEl.textContent = `到目标距离: ${drone.yopoDistToGoal.toFixed(2)} m`;
        countEl.textContent = `推理计数: ${drone.yopoInferenceCount}`;
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
    const cleanMode = document.getElementById('clean-mode-toggle')?.checked ? 1 : 0;
    const guideState = `${mode}|${cameraMode}|${drone ? drone.flightMode : ''}|${cleanMode}`;
    if (guideState === lastKeyGuideState) return;
    lastKeyGuideState = guideState;

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
        '<span style="color:#8cff8c">Nose down builds forward speed</span>',
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
        `<kbd>Tab</kbd>  ${isFPV ? 'Settings' : 'Settings / Easy Max Speed'}`,
    );
    if (cameraMode === 'third') {
        rows.push(
            '<kbd>L/R Mouse</kbd> Orbit observer',
            '<kbd>Wheel</kbd> Zoom',
            '<kbd>Middle</kbd> Pan / height',
        );
    }
    const html = `<div class="guide-title">${title}</div>\n${rows.join('\n')}`;
    if (el.innerHTML !== html) el.innerHTML = html;
    if (!cleanMode) {
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

function isTextEntryTarget(target) {
    if (!target || !target.closest) return false;
    return !!target.closest('input, textarea, select, [contenteditable="true"], [contenteditable=""]');
}

function isPointerOverCesiumCanvas() {
    const canvas = world?.viewer?.scene?.canvas;
    return !!(canvas && typeof canvas.matches === 'function' && canvas.matches(':hover'));
}

function setupThirdPersonPointerControls() {
    if (!world || !world.viewer) return;
    const canvas = world.viewer.scene.canvas;
    if (!canvas || canvas._flightThirdPersonBound) return;
    canvas._flightThirdPersonBound = true;

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
        try {
            canvas.setPointerCapture(e.pointerId);
        } catch (error) {
            reportUserError('Pointer capture failed', error, {
                key: 'pointer-capture',
                intervalMs: 10000,
            });
        }
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
        if (isTextEntryTarget(e.target)) {
            if (mode === 'placement' && e.code === 'KeyI' && isPointerOverCesiumCanvas()) {
                placementKeysDown.add(e.code);
                e.preventDefault();
            }
            return;
        }

        // YOPO target selection mode intercepts arrow keys / Enter / Esc
        // before they reach the flight controller.
        if (yopoTargetSelectMode && handleYOPOKeyDown(e)) return;

        if (mode === 'placement') {
            placementKeysDown.add(e.code);
            if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyI', 'KeyO'].includes(e.code)) {
                e.preventDefault();
            }
            if (e.code === 'KeyO' && spawnPoint) {
                confirmSpawnAndFly();
            }
        } else if (mode === 'view-select') {
            if (['Digit1', 'Numpad1', 'KeyO'].includes(e.code)) {
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
    if (startBtn && !startBtn._flightStartBound) {
        startBtn._flightStartBound = true;
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
        if (btn._flightViewChoiceBound) continue;
        btn._flightViewChoiceBound = true;
        btn.addEventListener('click', () => startFlight(btn.getAttribute('data-view-choice')));
    }
}

function initializeAppShell() {
    setupStartUI();
    setupKeyboard();
    setupSpawnAltitudeControls();
    setProgress('');
    window.googleTilesFlightStart = startTilesMode;
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initializeAppShell, { once: true });
} else {
    initializeAppShell();
}
