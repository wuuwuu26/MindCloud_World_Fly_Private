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

import { parsePlyForPositions, analyzePlyDistances, parsePlyOpacities, parsePlyRawCentroid, countFilteredPoints, sanitizePlyBuffer } from './ply-parser.js';
import { parseSplatForPositions, parseSplatOpacities, parseSplatRawCentroid, splatToPlyBuffer } from './splat-parser.js';
import { parseSogForPositions, parseSogOpacities, parseSogRawCentroid } from './sog-parser.js';
import { Octree } from './collision.js';
import { Controller } from './controller.js';
import { Drone } from './drone.js';
import { HUD } from './hud.js';
import { OSD } from './osd.js';

// ---- Globals ----
let app = null;
let cameraEntity = null;
let drone = null;
let controller = null;
let hud = null;
let osd = null;
let octree = null;
let sceneLoaded = false;
let pcInitialized = false;


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

function showError(msg) {
    console.error(msg);
    const el = document.getElementById('loading-progress');
    if (el) { el.textContent = msg; el.style.color = '#f44'; }
}

// ---- Initialize PlayCanvas (called once, before first PLY load) ----
function initPlayCanvas() {
    if (pcInitialized) return;

    const canvas = document.getElementById('app-canvas');

    app = new pc.Application(canvas, {
        mouse: new pc.Mouse(canvas),
        keyboard: new pc.Keyboard(window),
    });

    app.setCanvasFillMode(pc.FILLMODE_FILL_WINDOW);
    app.setCanvasResolution(pc.RESOLUTION_AUTO);
    window.addEventListener('resize', () => app.resizeCanvas());

    // Create camera
    cameraEntity = new pc.Entity('camera');
    cameraEntity.addComponent('camera', {
        clearColor: new pc.Color(0.05, 0.05, 0.1),
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
            mode = 'loading';
            sceneLoaded = false;
            document.getElementById('placement-overlay')?.classList.remove('visible');
            document.getElementById('game-logo')?.classList.remove('visible');
            document.getElementById('drop-zone').classList.remove('hidden');
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

function enterPlacementMode() {
    mode = 'placement';

    // Show placement UI, hide flight HUD and logo
    document.getElementById('placement-overlay').classList.add('visible');
    document.getElementById('hud').classList.add('hidden');
    document.getElementById('game-logo')?.classList.remove('visible');
    updateKeyGuide();

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

function confirmSpawnAndFly() {
    if (!spawnPoint) return;

    mode = 'flight';

    // Hide placement UI, show flight HUD and logo
    document.getElementById('placement-overlay').classList.remove('visible');
    hud.show();
    document.getElementById('game-logo')?.classList.add('visible');

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

    // File picker button
    filePickerBtn.addEventListener('click', (e) => {
        e.preventDefault();
        e.stopPropagation();
        fileInput.click();
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

        const arrayBuffer = await file.arrayBuffer();
        console.log(`File read: ${file.name} (${(arrayBuffer.byteLength / 1024 / 1024).toFixed(1)} MB)`);

        // ---- Format-specific: extract opacities & render buffer (coord-independent) ----
        let opacities = null;
        let rawCentroid = null;
        let renderBuffer = arrayBuffer;
        let renderFilename = file.name;

        if (ext === 'ply') {
            loadingProgress.textContent = 'Parsing opacity values...';
            await sleep(10);
            opacities = parsePlyOpacities(arrayBuffer);
            rawCentroid = parsePlyRawCentroid(arrayBuffer);

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
        const defaultCoord = coordSystem;
        const initialParse = await parseForCoord(ext, arrayBuffer, defaultCoord);

        if (ext === 'sog') {
            rawCentroid = {
                x: (initialParse.bounds.min[0] + initialParse.bounds.max[0]) * 0.5,
                y: (initialParse.bounds.min[1] + initialParse.bounds.max[1]) * 0.5,
                z: (initialParse.bounds.min[2] + initialParse.bounds.max[2]) * 0.5,
            };
        }

        console.log(`Initial parse: ${initialParse.vertexCount} vertices (${ext}, ${defaultCoord}), bounds:`, initialParse.bounds);
        let analysis = analyzePlyDistances(initialParse.positions, initialParse.vertexCount);

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

        // Hide logo during filtering
        document.getElementById('game-logo')?.classList.remove('visible');
        loadingOverlay.classList.remove('visible');

        // Show filter UI — user picks coord system + filter sliders
        // Callback for live coord switching during filtering
        const onCoordChange = async (newCoord) => {
            const parsed = await parseForCoord(ext, arrayBuffer, newCoord);
            analysis = analyzePlyDistances(parsed.positions, parsed.vertexCount);
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
            analysis, rawCentroid, ext, defaultCoord, onCoordChange
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

        const maxDistSq = filterResult.maxDist * filterResult.maxDist;
        const filteredPositions = [];
        for (let i = 0; i < finalVertexCount; i++) {
            if (opacities && opacities[i] < filterResult.minOpacity) continue;
            const dx = finalPositions[i * 3] - finalAnalysis.centroid.x;
            const dy = finalPositions[i * 3 + 1] - finalAnalysis.centroid.y;
            const dz = finalPositions[i * 3 + 2] - finalAnalysis.centroid.z;
            if (dx * dx + dy * dy + dz * dz <= maxDistSq) {
                filteredPositions.push(finalPositions[i * 3]);
                filteredPositions.push(finalPositions[i * 3 + 1]);
                filteredPositions.push(finalPositions[i * 3 + 2]);
            }
        }
        const keptCount = filteredPositions.length / 3;
        const filteredPosArray = new Float32Array(filteredPositions);
        console.log(`Final filter: ${keptCount}/${finalVertexCount} kept (dist=${filterResult.maxDist}, opacity=${filterResult.minOpacity})`);

        // Build collision octree
        let fMinX = Infinity, fMinY = Infinity, fMinZ = Infinity;
        let fMaxX = -Infinity, fMaxY = -Infinity, fMaxZ = -Infinity;
        for (let i = 0; i < keptCount; i++) {
            const x = filteredPosArray[i * 3], y = filteredPosArray[i * 3 + 1], z = filteredPosArray[i * 3 + 2];
            if (x < fMinX) fMinX = x; if (x > fMaxX) fMaxX = x;
            if (y < fMinY) fMinY = y; if (y > fMaxY) fMaxY = y;
            if (z < fMinZ) fMinZ = z; if (z > fMaxZ) fMaxZ = z;
        }
        const filteredBounds = { min: [fMinX, fMinY, fMinZ], max: [fMaxX, fMaxY, fMaxZ] };
        octree = new Octree();
        octree.build(filteredPosArray, filteredBounds);
        cachedArrayBuffer = arrayBuffer;
        cachedFilename = file.name;
        cachedFormat = ext;

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
                uniform float filterMaxDist;
                uniform float filterMinOpacity;

                void modifySplatCenter(inout vec3 center) {}

                void modifySplatRotationScale(vec3 originalCenter, vec3 modifiedCenter, inout vec4 rotation, inout vec3 scale) {
                    float dist = distance(originalCenter, filterCenter);
                    if (dist > filterMaxDist) {
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
        gsplat.setParameter('filterMaxDist', maxDist);
        gsplat.setParameter('filterMinOpacity', 0.0);
    } catch (e) {
        console.warn('setParameter not available:', e);
    }
}

function updateGSplatFilter(entity, maxDist, minOpacity) {
    if (!entity || !entity.gsplat) return;
    try {
        entity.gsplat.setParameter('filterMaxDist', maxDist);
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
function showFilterUI(gsplatEntity, positions, opacities, vertexCount, analysis, rawCentroid, ext, defaultCoord, onCoordChange) {
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

        // Update stats + GPU filter
        const updateFilter = () => {
            const dist = parseFloat(distSlider.value);
            const minOp = parseFloat(opacitySlider.value);
            distInput.value = dist;
            opacityInput.value = minOp.toFixed(2);

            // Update GPU shader uniforms (instant visual update)
            updateGSplatFilter(gsplatEntity, dist, minOp);

            // Update CPU-side point count
            const count = countFilteredPoints(
                curPositions, opacities, curVertexCount,
                curAnalysis.centroid.x, curAnalysis.centroid.y, curAnalysis.centroid.z,
                dist, minOp
            );
            statsEl.textContent = `${count.toLocaleString()} / ${curVertexCount.toLocaleString()} points kept`;
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

            // Update distance slider range
            maxExtent = Math.ceil(curAnalysis.maxDistance);
            distSlider.max = maxExtent;
            distInput.max = maxExtent;
            if (parseFloat(distSlider.value) > maxExtent) {
                distSlider.value = maxExtent;
                distInput.value = maxExtent;
            }

            updateFilter();
            console.log(`Coord switched to ${newCoord} — ${curVertexCount} vertices`);
        };

        // Initial update
        updateFilter();
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
            overlay.classList.remove('visible');
            if (filterGuide) filterGuide.style.display = 'none';
            resolve({
                maxDist: parseFloat(distSlider.value),
                minOpacity: parseFloat(opacitySlider.value),
                coordSystem: curCoord,
                finalPositions: curPositions,
                finalVertexCount: curVertexCount,
                finalAnalysis: curAnalysis,
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

    // Apply drone transform to camera
    const transform = drone.getCameraTransform();
    cameraEntity.setPosition(transform.position.x, transform.position.y, transform.position.z);
    cameraEntity.setEulerAngles(transform.rotation.x, transform.rotation.y, transform.rotation.z);

    // Clean mode: hide key guide and logo only (OSD is independent)
    const cleanToggle = document.getElementById('clean-mode-toggle');
    const cleanMode = cleanToggle ? cleanToggle.checked : false;

    // HUD - always visible in flight mode (not affected by Clean Mode)
    const hudEl = document.getElementById('hud');
    hudEl?.classList.remove('hidden');
    hud.update(drone, controller);

    // FPV OSD - independent of Clean Mode, only controlled by its own toggle
    if (osd) {
        const osdToggle = document.getElementById('osd-toggle');
        osd.setEnabled(osdToggle ? osdToggle.checked : true);
        osd.update(drone, controller);
    }

    // Key guide and Logo - controlled by Clean Mode
    const logo = document.getElementById('game-logo');
    const keyGuide = document.getElementById('key-guide');
    if (cleanMode) {
        logo?.classList.remove('visible');
        keyGuide?.classList.remove('visible');
    } else {
        if (mode === 'flight') logo?.classList.add('visible');
        updateKeyGuide();
    }
}

// ---- Key Guide ----
function updateKeyGuide() {
    const el = document.getElementById('key-guide');
    if (!el) return;

    if (mode === 'placement') {
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
    } else if (mode === 'flight') {
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
console.log('MindCloud World Fly ready — drop a .ply, .sog, or .splat file to begin');
