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

import { CesiumWorld } from './cesium-world.js?v=20260822-hz-trt25';
import { TilesCollisionProvider } from './tiles-collision.js';
import { Controller } from './controller.js';
import { Drone } from './drone.js?v=20260822-hz-trt25';
import { HUD } from './hud.js';
import { OSD } from './osd.js';
import { PanoramaSensor } from './panorama-sensor.js?v=20260820-nobusy';
import { YOPONavigator } from './yopo-navigator.js';
import { YOPODepthFromPanorama } from './yopo-depth-from-panorama.js?v=20260820-fps3';
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
let yopoControlTimer = null;  // Independent 50 Hz control timer (see yopoControlTick)
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
    el.innerHTML = `${VIEW_CHOICE_HINT_HTML}<br><span style="color:#9fb5ff">Preload warning: ${warnings}. Tiles may continue loading after takeoff.</span>`;
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

        window.world = world;
        window.drone = drone;
        window.yopoDepthFromPanorama = yopoDepthFromPanorama;
        window.startTilesMode = startTilesMode;

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
    // Right-handed: right = cross(fwd, up) = (-cos h, 0, sin h)
    // The original code used right=(cos h, 0, -sin h) = cross(up, fwd) = left, which made D
    // move left and A move right (reversed)
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

// ---------------------------------------------------------------------------
// YOPO minimap (a real 3D Cesium viewer, a full mirror of the placement-mode main world) --
// it originally lived in the user's uncommitted main.js (initYOPOMinimapViewer) and was
// overwritten by an accidental revert, so it was rebuilt here from the main world config.
// Key point: the minimap viewer reaches the main world instance through window.world
// (window.world.token / window.world.assetId / window.world.localToCartesian). The real root
// cause of the previous black screen was that window.world was never assigned (the main world
// was a local `let`, never attached to window), so init returned on its very first line and
// the viewer was never created; main.js now exposes window.world = world, so this initialises
// normally.
// It must mirror the main world's globe:false + requestRenderMode:false (continuous
// rendering), otherwise streaming 3D Tiles are not drawn frame by frame and it stays black.
// ---------------------------------------------------------------------------
let yopoMinimapViewer = null;
let yopoMinimapWorld = null; // The CesiumWorld instance behind the minimap (same class as the main world, so rendering matches)
let yopoMinimapDroneEntity = null;
let yopoMinimapTargetEntity = null;
let _yopoMiniInitPromise = null;
let _yopoMiniRange = null; // Smoothed lookAt distance (zoom)
let _yopoMiniHeading = null; // Smoothed minimap orientation (rad): heading-up map, the drone's forward direction always points up
let _yopoMiniStop = false; // Disabled after a minimap update error, so it does not throw every frame and drag down the main loop

async function initYOPOMinimapViewer() {
    // Skip while window.world is not ready / already created (root cause: !window.world used to
    // be always true -> it always returned)
    if (yopoMinimapViewer || !window.world || !window.world.ready) return;
    const Cesium = window.world.Cesium;
    const host = document.getElementById('yopo-minimap');
    if (!host) return;
    // While the HUD is hidden the container has zero size; wait until it is visible before
    // creating the viewer (retried every frame)
    if (host.clientWidth < 2 || host.clientHeight < 2) return;

    // Build the minimap with the same CesiumWorld class as the main world: rendering is
    // identical to the main view (placement mode), with the globe hidden, Google 3D Tiles
    // loaded, setOrigin / camera etc. handled correctly internally, so the default globe never
    // shows up.
    const mini = new CesiumWorld('yopo-minimap', {
        token: window.world.token,
        assetId: window.world.assetId,
    });
    await mini.init();
    yopoMinimapWorld = mini;
    yopoMinimapViewer = mini.viewer;
    const scene = yopoMinimapViewer.scene;
    scene.screenSpaceCameraController.enableInputs = false; // Minimap is read-only, no user interaction
    // Hide the main-view widgets (the minimap does not need them): geocoder / home button /
    // help etc.
    ['geocoder', 'homeButton', 'navigationHelpButton', 'baseLayerPicker',
     'fullscreenButton', 'timeline', 'animation', 'selectionIndicator', 'infoBox'].forEach((n) => {
        const w = yopoMinimapViewer[n];
        if (w && w.container && w.container.style) w.container.style.display = 'none';
    });

    // Drone (blue) / target (yellow) points, always drawn in front of everything
    const alwaysFront = Number.POSITIVE_INFINITY;
    yopoMinimapDroneEntity = yopoMinimapViewer.entities.add({
        point: { pixelSize: 14, color: Cesium.Color.fromCssColorString('#39a0ff'),
            outlineColor: Cesium.Color.WHITE, outlineWidth: 2,
            disableDepthTestDistance: alwaysFront },
        label: {
            text: 'UAV', fillColor: Cesium.Color.WHITE,
            font: '12px sans-serif', pixelOffset: new Cesium.Cartesian2(0, -18),
            style: Cesium.LabelStyle.FILL_AND_OUTLINE, outlineWidth: 2,
            outlineColor: Cesium.Color.BLACK, showBackground: true,
            backgroundColor: Cesium.Color.fromCssColorString('#39a0ff').withAlpha(0.6),
            disableDepthTestDistance: alwaysFront,
        },
    });
    yopoMinimapTargetEntity = yopoMinimapViewer.entities.add({
        point: { pixelSize: 12, color: Cesium.Color.fromCssColorString('#ffd23a'),
            outlineColor: Cesium.Color.WHITE, outlineWidth: 1,
            disableDepthTestDistance: alwaysFront },
        label: {
            text: 'TARGET', fillColor: Cesium.Color.WHITE,
            font: '12px sans-serif', pixelOffset: new Cesium.Cartesian2(0, -18),
            style: Cesium.LabelStyle.FILL_AND_OUTLINE, outlineWidth: 2,
            outlineColor: Cesium.Color.BLACK, showBackground: true,
            backgroundColor: Cesium.Color.fromCssColorString('#ffd23a').withAlpha(0.6),
            disableDepthTestDistance: alwaysFront,
        },
    });

    // Put the initial camera in place: use the main world's W.localToCartesian to unify the
    // coordinate frame (the minimap frame's origin is not reset with placement mode, which
    // would offset it), looking straight down (-89.9 deg).
    const initialLocal = { x: 0, y: 0, z: 0 };
    const initialRange = 500;
    const initPitch = Cesium.Math.toRadians(-89.9); // Nearly straight-down view
    const initEyeLocal = {
        x: initialLocal.x,
        y: initialLocal.y + initialRange * Math.sin(-initPitch),
        z: initialLocal.z - initialRange * Math.cos(initPitch),
    };
    yopoMinimapViewer.scene.camera.setView({
        destination: window.world.localToCartesian(initEyeLocal),
        orientation: {
            heading: Cesium.Math.toRadians(0),
            pitch: initPitch,
            roll: 0,
        },
    });
}

function _getYOPOMinimapTarget() {
    if (drone && drone.yopoNavTarget) {
        return { x: drone.yopoNavTarget.x, y: drone.yopoNavTarget.y, z: drone.yopoNavTarget.z };
    }
    if (yopoTargetSelectMode) {
        const tx = parseFloat(document.getElementById('yopo-target-x')?.value);
        const ty = parseFloat(document.getElementById('yopo-target-y')?.value);
        const tz = parseFloat(document.getElementById('yopo-target-z')?.value);
        if (Number.isFinite(tx) && Number.isFinite(ty) && Number.isFinite(tz)) {
            return { x: tx, y: ty, z: tz };
        }
    }
    return null;
}

function updateYOPOMinimap() {
    if (!window.world || !window.world.ready) return;
    if (_yopoMiniStop) return; // Disabled after an earlier error, do not throw every frame
    // Lazy init: create the minimap viewer once the window is ready (async, does not block the
    // main loop)
    if (!yopoMinimapViewer) {
        if (!_yopoMiniInitPromise) {
            _yopoMiniInitPromise = initYOPOMinimapViewer()
                .catch(e => console.warn('[YOPO minimap] init failed', e))
                .finally(() => { _yopoMiniInitPromise = null; });
        }
        return;
    }
    const Cesium = window.world.Cesium;
    const W = window.world;
    const target = _getYOPOMinimapTarget();

    // Delta text
    const hText = document.getElementById('yopo-target-height-text');
    const dText = document.getElementById('yopo-target-delta-text');
    if (hText) hText.textContent = target ? `Target altitude y: ${target.y.toFixed(2)}` : 'Target altitude y: --';
    if (dText) {
        if (target && drone) {
            const dx = target.x - drone.x, dy = target.y - drone.y, dz = target.z - drone.z;
            dText.textContent = `Δx/Δy/Δz to target: ${dx.toFixed(2)} / ${dy.toFixed(2)} / ${dz.toFixed(2)}`;
        } else dText.textContent = 'Δx/Δy/Δz to target: --';
    }

    // Entity positions (localToCartesian converts ENU local coordinates into world Cartesian3)
    try {
        if (drone) {
            yopoMinimapDroneEntity.position = W.localToCartesian({ x: drone.x, y: drone.y, z: drone.z });
            yopoMinimapDroneEntity.show = true;
        } else yopoMinimapDroneEntity.show = false;
        if (target) {
            yopoMinimapTargetEntity.position = W.localToCartesian(target);
            yopoMinimapTargetEntity.show = true;
        } else yopoMinimapTargetEntity.show = false;

        // Camera follow: looking down (-89.9 deg) pins the drone to the centre of the view.
        // Always use the main world's W.localToCartesian coordinate frame, so the drone does not
        // drift off screen because the minimap CesiumWorld instance's origin was not reset with
        // placement mode.
        // The zoom (range) adapts to the drone <-> target distance so the target point always
        // stays in view.
        const centerLocal = drone ? { x: drone.x, y: drone.y, z: drone.z } : (target || { x: 0, y: 0, z: 0 });
        const dist = (drone && target) ? Math.hypot(drone.x - target.x, drone.z - target.z) : 0;
        // range = top-down camera height; the base field of view is widened (~150 m by default)
        // and adapts to the drone -> target distance: a factor of 2.4 plus a 100 m margin, so the
        // target point never leaves the view while setting it up / navigating.
        const wantRange = Math.max(150, 2.4 * dist + 100);
        if (_yopoMiniRange === null) _yopoMiniRange = wantRange;
        else _yopoMiniRange += (wantRange - _yopoMiniRange) * 0.2; // Mild smoothing: zoom reacts in time without jumping

        const R = _yopoMiniRange;
        const pitchRad = Cesium.Math.toRadians(-89.9); // Nearly straight-down view

        // Heading-up minimap: the drone's "nose direction projected onto the horizontal plane"
        // is always the top of the minimap.
        // It must be the nose direction (not the velocity direction): when the drone moves
        // backwards / sideways the velocity direction opposes the nose, so using it would flip
        // the minimap by 180 deg (moving backwards would turn the map over). The nose direction
        // comes from yopoBodyMoveAxes().fwd -- local -Z projected onto the horizontal plane (ENU),
        // exactly the same forward used by numpad 8/2/4/6 when moving the target point, so
        // selection / flight / forward / backward all agree and never flip.
        // In Cesium's top-down view, heading=0 means screen up = north (ENU +z), hence
        // heading = atan2(fwdEast, fwdNorth).
        let fwdHx = 0, fwdHz = -1; // Default faces south (-Z), matching the identity nose direction
        if (drone) {
            const axes = yopoBodyMoveAxes();
            fwdHx = axes.fwd.x;
            fwdHz = axes.fwd.z;
        }
        const targetHeading = Math.atan2(fwdHx, fwdHz);
        if (_yopoMiniHeading === null) {
            _yopoMiniHeading = targetHeading;
        } else {
            // Angle interpolation with +/-PI wrap handling, smoothly following turns (no
            // violent swings)
            let d = targetHeading - _yopoMiniHeading;
            while (d > Math.PI) d -= 2 * Math.PI;
            while (d < -Math.PI) d += 2 * Math.PI;
            _yopoMiniHeading += d * 0.2;
        }
        const headingRad = _yopoMiniHeading;

        // Put the camera straight above the drone (height R) looking straight down
        // (pitch = -89.9), equivalent to lookAt(center, HPR(heading, -89.9, R))
        const eyeLocal = {
            x: centerLocal.x,
            y: centerLocal.y + R * Math.sin(-pitchRad),
            z: centerLocal.z - R * Math.cos(pitchRad),
        };
        const eyeCart = W.localToCartesian(eyeLocal);
        yopoMinimapViewer.scene.camera.setView({
            destination: eyeCart,
            orientation: {
                heading: headingRad,
                pitch: pitchRad,
                roll: 0,
            },
        });
    } catch (e) {
        // An error in any minimap step (a Cesium call) must not drag down the main loop /
        // flight; disable the minimap and log the real error to the console
        console.error('[YOPO minimap] update failed, minimap disabled to avoid throwing every frame:', e);
        _yopoMiniStop = true;
    }
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
        // Errors in the flight/physics update are reported separately from the "minimap" ones,
        // which makes them easier to locate
        reportUserError('Flight update failed', e, {
            key: 'flight-loop',
            intervalMs: 3000,
        });
    }
    try {
        // Refresh the bottom-left YOPO top-down minimap every frame (target point + drone
        // position + delta text)
        updateYOPOMinimap();
    } catch (e) {
        reportUserError('Minimap update failed', e, {
            key: 'minimap-loop',
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

    // ---- YOPO navigation update (depth / control separation) ----
    // Mirrors YOPO's original architecture: control_pub (50 Hz) + callback_depth (replan at
    // 30 Hz)
    //   - control loop (~60 Hz, every rendered frame): /yopo/control advances ctrl_time and
    //     evaluates the polynomial
    //   - depth loop (~0.4 Hz, when depth arrives): /yopo/navigate re-runs inference and
    //     rebuilds the polynomial
    // The two are independent and never block each other. Commands always stay fresh and the
    // drone never flies blind.
    if (drone.flightMode === 'yopo_nav' && drone.yopoNavActive && yopoNavigator && !drone.yopoArrived) {
        // The motion command (replan) update rate matches the depth image update rate: the
        // navigate client-side throttle is bound straight to the depth refresh interval, so the
        // two "agree on frequency" and rise together with _minRefreshIntervalMs.
        if (yopoDepthFromPanorama) {
            yopoNavigator._requestInterval = yopoDepthFromPanorama._minRefreshIntervalMs;
        }
        const pos = { x: drone.x, y: drone.y, z: drone.z };
        const vel = { x: drone.vx, y: drone.vy, z: drone.vz };
        const orient = {
            x: drone.orientation.x,
            y: drone.orientation.y,
            z: drone.orientation.z,
            w: drone.orientation.w,
        };

        // ── The control loop has been split out into the independent 50 Hz yopoControlTick
        // timer ──
        // (It no longer depends on the render frame rate: when the main thread is slowed by
        //  Cesium rendering / DA360 uploads, calling it every frame would advance ctrl_time by
        //  only 20 ms per frame and it would be reset constantly by the high-rate navigate
        //  calls, leaving the polynomial stuck at its start -> commanded speed ~= 0 -> the drone
        //  suddenly slows down. See the yopoControlTick comment.)

        // ── Depth loop (low rate, when depth arrives) ──
        // Sends depth + odom, runs YOPO inference, rebuilds the polynomial and resets
        // ctrl_time = 0.
        // Only one navigate request may be in flight at a time; it runs in parallel with the
        // control loop without blocking it.
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

                    // Feed the DA360 estimated depth (metric scale calibrated by sparse Cesium
                    // rays) into YOPO.
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
                        // Depth unavailable (DA360 failure/timeout): do NOT fall back to Cesium
                        // raycasting.
                        // Hover in place and wait for the depth map to come back; the depth loop
                        // keeps retrying at a low rate until valid depth arrives.
                        if (drone.yopoInferenceCount < 3 || drone.yopoInferenceCount % 30 === 0) {
                            console.warn('YOPO: DA360 depth unavailable, hovering to retry (no Cesium fallback)');
                        }
                        drone.yopoDepthUnavailable = true;
                        drone.yopoCmdPos = { x: drone.x, y: drone.y, z: drone.z };
                        drone.yopoCmdVel = { x: 0, y: 0, z: 0 };
                        drone.yopoCmdAcc = { x: 0, y: 0, z: 0 };
                        drone.yopoCmdYaw = drone.yaw * (Math.PI / 180);   // Freeze yaw: hold the current nose heading (deg -> rad)
                        drone.yopoCmdYawDot = 0;
                        drone.yopoCmdTime = performance.now();
                        yopoNavInProgress = false;
                        return;
                    }
                    drone.yopoDepthUnavailable = false;

                    // Record the current depth source (ground-truth rays / DA360 estimate);
                    // rendered by updateYOPOStatusUI
                    drone.yopoDepthSource = depthResult.source;

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
                        const navMs = t2 - t1;
                        const navCached = navMs < 30;   // < 30 ms means the client-side 33 ms throttle hit and returned a cached command instead of really calling the server
                        console.log(`YOPO timing: depth=${(t1-t0).toFixed(0)}ms navigate=${navMs.toFixed(0)}ms total=${(t2-t0).toFixed(0)}ms${navCached ? ' [nav-cache]' : ''} src=${depthResult.source}`);
                    }

                    if (cmd && !cmd.error) {
                        // navigate returns the command at ctrl_time = 0; the control loop
                        // advances ctrl_time and overwrites it on the next tick. Updating right
                        // after the first inference avoids hovering while waiting.
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

    // "Select goal" button: enter keyboard-driven goal selection mode.
    // Target starts at the drone's current position; user moves it with
    // arrow keys and presses Enter to confirm (which also auto-starts nav).
    selectTargetBtn.addEventListener('click', enterYOPOTargetSelectMode);

    // Start Navigation button (manual fallback — normally Enter in select mode)
    startNavBtn.addEventListener('click', async () => {
        if (!drone.yopoNavTarget) {
            document.getElementById('yopo-status-text').textContent = 'Status: set a goal first';
            return;
        }
        // Keep the goal marker visible: it may have been removed after the first navigation
        // arrived / stopped, so recreate it when "Start Navigation" is clicked directly (a
        // second navigation) to keep the goal point always visible.
        createYOPOTargetMarker(
            drone.yopoNavTarget.x, drone.yopoNavTarget.y, drone.yopoNavTarget.z
        );
        // Check YOPO server connectivity
        const status = await yopoNavigator.getStatus();
        if (!status) {
            document.getElementById('yopo-status-text').textContent = 'Status: YOPO server unreachable (port 5689)';
            console.warn('YOPO server not reachable at', yopoNavigator.serverUrl);
            return;
        }
        // Prewarm the depth cache: build the first depth frame before activating navigation so
        // the first navigate has depth available immediately
        prewarmYOPODepth();
        drone.flightMode = 'yopo_nav';
        drone.yopoNavActive = true;
        if (panoramaSensor) {
            // While navigating, UI depth requests are no longer suppressed, so the DA360 depth
            // window keeps refreshing with the nav loop (previously depthSuppress=true froze the
            // window on the last image).
            panoramaSensor.depthSuppress = false;
            panoramaSensor.captureIntervalOverride = 50;  // 20 Hz panorama requests: update() skips automatically while capturing, so raising the request rate
            // only starts the next capture sooner after one completes (6 faces take ~300-600 ms); it does not take extra main-thread time.
        }
        drone.yopoArrived = false;
        drone.yopoInferenceCount = 0;
        drone.yopoCmdPos = null;
        drone.yopoCmdVel = null;
        drone.yopoCmdTime = 0;
        // Start the independent 50 Hz control timer: it guarantees the trajectory keeps
        // advancing (independent of the render frame rate).
        // Control loop and depth loop run in parallel: rebuilding the polynomial in the depth
        // loop resets ctrl_time, but the control loop advances it every 20 ms, so even with
        // replanning at 3-5 Hz it still advances 0.2-0.3 s of trajectory within each 200-300 ms
        // window, letting the commanded speed climb to cruise level (measured ~5-9 m/s).
        if (yopoControlTimer) clearInterval(yopoControlTimer);
        yopoControlTimer = setInterval(yopoControlTick, 20);
        // Sync the flight mode dropdown
        const modeSelect = document.getElementById('flight-mode-select');
        if (modeSelect) modeSelect.value = 'yopo_nav';
        document.getElementById('yopo-status-text').textContent = 'Status: navigating...';
        document.getElementById('yopo-start-nav-btn').textContent = 'Navigating...';
        console.log('YOPO navigation started, goal:', drone.yopoNavTarget);
    });

    // Stop Navigation button
    stopNavBtn.addEventListener('click', stopYOPONavigation);
}

// ── YOPO target selection helpers ───────────────────────────────

const YOPO_TARGET_STEP = 0.5; // metres per key press

/** Enter target selection mode (shared by the button and the T shortcut). If navigation is
 * running, stop it first. */
function enterYOPOTargetSelectMode() {
    if (mode !== 'flight' || !drone) return;
    if (yopoTargetSelectMode) return; // already selecting
    // Stop navigating first, then enter target selection
    if (drone.flightMode === 'yopo_nav' && drone.yopoNavActive) {
        stopYOPONavigation();
    }
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
        'Status: goal select mode (numpad 8/2/4/6/9/3 move, 5 confirm, 0 cancel)';
    console.log('YOPO target select mode: starting at drone pos', { x, y, z });
}

/** Stop navigation (shared by the button and the X shortcut). The goal marker is kept so it
 * can be compared against on the next navigation. */
function stopYOPONavigation() {
    if (!drone) return;
    // Not navigating: if target selection mode is active, just cancel the selection
    if (drone.flightMode !== 'yopo_nav' || !drone.yopoNavActive) {
        if (yopoTargetSelectMode) cancelYOPOTarget();
        return;
    }
    drone.yopoNavActive = false;
    drone.yopoArrived = false;
    if (yopoControlTimer) {
        clearInterval(yopoControlTimer);
        yopoControlTimer = null;
    }
    if (panoramaSensor) {
        panoramaSensor.depthSuppress = false;  // Restore the UI depth display
        panoramaSensor.captureIntervalOverride = 0;  // Restore the 60 Hz panorama
    }
    drone.yopoCmdPos = null;
    drone.yopoCmdVel = null;
    drone.yopoCmdTime = 0;
    // Switch back to simpleflight or drone mode
    drone.flightMode = 'simpleflight';
    const modeSelect = document.getElementById('flight-mode-select');
    if (modeSelect) modeSelect.value = 'simpleflight';
    document.getElementById('yopo-status-text').textContent = 'Status: stopped';
    document.getElementById('yopo-start-nav-btn').textContent = 'Start Navigation';
    // Keep the goal marker: the "selected goal" stays visible after stopping, so it can be
    // compared against when navigating again
}

// ── YOPO control loop (independent 50 Hz timer) ─────────────────
// Why the control loop was split out of the render frame loop:
//   When the browser main thread is dragged down by Cesium 3D Tiles rendering / panorama
//   capture / DA360 uploads, the frame rate can drop to 3-10 fps. If the control loop depended
//   on "every rendered frame", then:
//     - control loop frequency = frame rate (3-10 Hz), ctrl_time advances only 20 ms per frame
//       (server-side cap)
//     - the depth loop's navigate() resets ctrl_time = 0 every time it rebuilds the polynomial
//       (~3 Hz)
//     -> ctrl_time stays forever near the trajectory start, and the polynomial speed ~= the
//        start speed (the loop depends on the true speed)
//     -> the commanded speed approaches 0 and the drone "suddenly becomes very slow, not even
//        1 m/s".
// An independent 20 ms timer guarantees the control loop advances ctrl_time steadily at 50 Hz:
// even if navigate resets it every 200-300 ms, it still advances 0.2-0.3 s of trajectory within
// that window, and the commanded speed climbs to cruise level (measured 5-9 m/s).
// It runs in parallel with the depth loop (neither blocks the other), commands always stay
// fresh, and the drone never flies blind.
function yopoControlTick() {
    if (!drone || !yopoNavigator) return;
    if (drone.flightMode !== 'yopo_nav' || !drone.yopoNavActive || drone.yopoArrived) return;
    // Skip while depth is unavailable (hovering and waiting) to keep the hover command and stop
    // an old polynomial from overwriting it.
    if (drone.yopoDepthUnavailable || yopoControlInProgress) return;
    const pos = { x: drone.x, y: drone.y, z: drone.z };
    const vel = { x: drone.vx, y: drone.vy, z: drone.vz };
    const orient = {
        x: drone.orientation.x,
        y: drone.orientation.y,
        z: drone.orientation.z,
        w: drone.orientation.w,
    };
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
            // Called at a high rate, swallow transient errors silently
        }
        yopoControlInProgress = false;
    })();
}

/**
 * Compute, from the drone's current heading, the unit vectors of its nose direction (fwd) and
 * its right side (right) on the horizontal plane, used to move the goal point: 8/2 move along
 * the nose, 4/6 move left/right relative to it (the current heading is "forward").
 * The nose direction is local -Z (world.getForwardLocal) projected onto the horizontal plane;
 * the right vector is right = forward x up = (-fwd.z, 0, fwd.x), consistent with the local +X
 * convention of getTransformBasisLocal in cesium-world.
 */
function yopoBodyMoveAxes() {
    let fwd = { x: 0, z: -1 }; // Default faces south (-Z), matching the identity nose direction
    if (world && drone && typeof world.getForwardLocal === 'function') {
        const f = world.getForwardLocal(drone.getBodyTransform());
        const fl = Math.hypot(f.x, f.z);
        if (fl > 1e-4) fwd = { x: f.x / fl, z: f.z / fl };
    }
    const right = { x: -fwd.z, z: fwd.x }; // forward × up
    return { fwd, right };
}

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
                color: Cesium.Color.fromCssColorString('#ffd23a'),
                outlineColor: Cesium.Color.WHITE,
                outlineWidth: 2,
                disableDepthTestDistance: Number.POSITIVE_INFINITY,
            },
            label: {
                text: 'YOPO TARGET',
                font: '12px sans-serif',
                pixelOffset: new Cesium.Cartesian2(0, -24),
                fillColor: Cesium.Color.fromCssColorString('#cfe'),
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
 *   Numpad 8/2 = forward/back (along the drone's current nose direction)
 *   Numpad 4/6 = left/right   (perpendicular to the nose)
 *   Numpad 9/3 = up/down      (+y/-y)
 *   Numpad 5   = confirm (start navigation)
 *   Numpad 0   = cancel
 *
 * Every 8/2 and 4/6 press is relative to the drone's current heading (not to a fixed world
 * direction), so even after the drone turns, the numpad always moves the goal consistently with
 * the nose direction.
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

    // Compute forward / right (horizontal-plane unit vectors) from the drone's current heading
    const { fwd, right } = yopoBodyMoveAxes();

    let consumed = true;
    switch (e.code) {
        case 'Numpad8': case 'NumpadArrowUp': // Forward along the nose
            x += fwd.x * YOPO_TARGET_STEP; z += fwd.z * YOPO_TARGET_STEP; break;
        case 'Numpad2': case 'NumpadArrowDown': // Backward along the nose
            x -= fwd.x * YOPO_TARGET_STEP; z -= fwd.z * YOPO_TARGET_STEP; break;
        case 'Numpad4': case 'NumpadArrowLeft': // Right of the nose (swapped with the original 4 = left)
            x += right.x * YOPO_TARGET_STEP; z += right.z * YOPO_TARGET_STEP; break;
        case 'Numpad6': case 'NumpadArrowRight': // Left of the nose (swapped with the original 6 = right)
            x -= right.x * YOPO_TARGET_STEP; z -= right.z * YOPO_TARGET_STEP; break;
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

/** Prewarm the YOPO depth cache: trigger one DA360 depth capture in the background before
 * navigation activates, so the first navigate can use cached depth immediately and the
 * "drone hovers in place for several seconds after navigation starts" wait disappears.
 * Returns true if the prewarm was triggered, false if the preconditions were not met. */
function prewarmYOPODepth() {
    if (!drone || !yopoDepthFromPanorama || typeof drone.getCameraTransform !== 'function') return false;
    try {
        const cameraTransform = drone.getCameraTransform();
        if (!cameraTransform) return false;
        yopoDepthFromPanorama.captureYOPODepthERP(cameraTransform, {
            width: 384, height: 192, maxDistance: 20, timeoutMs: 6000,
        }).then(() => {
            console.log('[prewarm] YOPO depth cache warmed');
        }).catch((e) => {
            // A prewarm failure must not block navigation: the depth loop's first frame still
            // retries normally
            console.warn('[prewarm] YOPO depth prewarm failed:', e);
        });
        return true;
    } catch (e) {
        return false;
    }
}

/** Confirm the selected target: set goal on server and auto-start nav. */
async function confirmYOPOTarget(x, y, z) {
    yopoTargetSelectMode = false;
    document.getElementById('yopo-status-text').textContent =
        `Status: setting goal (${x.toFixed(1)}, ${y.toFixed(1)}, ${z.toFixed(1)})...`;

    // Prewarm the depth cache in parallel: DA360 inference is slow (~1.6 s), so if we waited
    // for navigation to be active before fetching the first depth frame, the drone would hover
    // in place for several seconds ("wandering for a long time before acting"). Triggering one
    // capture in the background during setGoal/getStatus lets the first navigate reuse the cache
    // and emit a command immediately.
    const prewarm = prewarmYOPODepth();

    const ok = await yopoNavigator.setGoal(x, y, z);
    if (!ok) {
        document.getElementById('yopo-status-text').textContent = 'Status: failed to set goal';
        removeYOPOTargetMarker();
        return;
    }
    drone.yopoNavTarget = { x, y, z };
    // Make sure the goal marker exists (covers the case where it was removed earlier, so it is
    // still visible on the second navigation)
    createYOPOTargetMarker(x, y, z);

    // Check YOPO server connectivity before starting
    const status = await yopoNavigator.getStatus();
    if (!status) {
        document.getElementById('yopo-status-text').textContent =
            'Status: YOPO server unreachable (port 5689)';
        console.warn('YOPO server not reachable at', yopoNavigator.serverUrl);
        removeYOPOTargetMarker();
        return;
    }

    // Navigation is about to activate; make sure the prewarm was triggered (retry here if the
    // preconditions were not met above)
    if (prewarm !== true) prewarmYOPODepth();

    // Auto-start navigation
    drone.flightMode = 'yopo_nav';
    drone.yopoNavActive = true;
                if (panoramaSensor) {
                    // Do NOT set depthSuppress back to true: that would freeze the UI DA360
                    // depth window once navigation starts from goal selection (same reason as in
                    // the setGoal path). Keep it false so the window keeps requesting depth maps
                    // as the nav loop refreshes RGB.
                    panoramaSensor.depthSuppress = false;
                    panoramaSensor.captureIntervalOverride = 50;  // 20 Hz panorama requests (same reason as above)
                }
    drone.yopoArrived = false;
    drone.yopoInferenceCount = 0;
    drone.yopoCmdPos = null;
    drone.yopoCmdVel = null;
    drone.yopoCmdTime = 0;
    // Start the independent 50 Hz control timer (same path as the manual "Start Navigation"
    // button)
    if (yopoControlTimer) clearInterval(yopoControlTimer);
    yopoControlTimer = setInterval(yopoControlTick, 20);
    const modeSelect = document.getElementById('flight-mode-select');
    if (modeSelect) modeSelect.value = 'yopo_nav';
    document.getElementById('yopo-status-text').textContent = 'Status: navigating...';
    document.getElementById('yopo-start-nav-btn').textContent = 'Navigating...';
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
    document.getElementById('yopo-status-text').textContent = 'Status: goal selection cancelled';
}

function updateYOPOStatusUI() {
    if (!drone || !yopoNavigator) return;
    const statusEl = document.getElementById('yopo-status-text');
    const distEl = document.getElementById('yopo-dist-text');
    const countEl = document.getElementById('yopo-count-text');
    const avoidEl = document.getElementById('yopo-avoid-text');
    if (!statusEl || !distEl || !countEl) return;

    // Depth source status: shows whether ground-truth rays or DA360 estimated depth is in use
    // (handy for A/B comparison)
    if (avoidEl) {
        const src = drone.yopoDepthSource;
        let srcHtml;
        if (src === 'true') {
            srcHtml = 'Depth: <span style="color:#9fb5ff">ground truth (rays)</span>';
        } else if (src === 'da360') {
            srcHtml = 'Depth: <span style="color:#cfe">DA360</span>';
        } else {
            srcHtml = 'Depth: --';
        }
        avoidEl.innerHTML = srcHtml;
    }

    // Show distance during target selection mode too (compute from inputs,
    // NOT from drone.yopoNavTarget which is not set until confirmation).
    if (yopoTargetSelectMode && drone) {
        const tx = parseFloat(document.getElementById('yopo-target-x')?.value);
        const ty = parseFloat(document.getElementById('yopo-target-y')?.value);
        const tz = parseFloat(document.getElementById('yopo-target-z')?.value);
        if (Number.isFinite(tx) && Number.isFinite(ty) && Number.isFinite(tz)) {
            const dx = tx - drone.x, dy = ty - drone.y, dz = tz - drone.z;
            distEl.textContent = `Distance to goal: ${Math.sqrt(dx*dx+dy*dy+dz*dz).toFixed(2)} m`;
        }
        return;
    }

    if (drone.flightMode === 'yopo_nav' && drone.yopoNavActive) {
        if (drone.yopoArrived) {
            statusEl.textContent = 'Status: goal reached ✓';
            // Keep the goal marker after arrival: the "selected goal" always stays visible, so
            // the goal position is visible on the second navigation too. The marker is only
            // removed when reselecting / cancelling the goal.
        } else {
            statusEl.textContent = 'Status: navigating...';
        }
        distEl.textContent = `Distance to goal: ${drone.yopoDistToGoal.toFixed(2)} m`;
        countEl.textContent = `Inference count: ${drone.yopoInferenceCount}`;
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
        '<span style="color:#9fb5ff">Nose down builds forward speed</span>',
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
            if (e.code === 'KeyT') {
                // T: start setting the navigation goal (enter goal selection mode)
                e.preventDefault();
                enterYOPOTargetSelectMode();
                return;
            }
            if (e.code === 'KeyX') {
                // X: stop navigation
                e.preventDefault();
                stopYOPONavigation();
                return;
            }
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
    window.startTilesMode = startTilesMode;
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initializeAppShell, { once: true });
} else {
    initializeAppShell();
}
