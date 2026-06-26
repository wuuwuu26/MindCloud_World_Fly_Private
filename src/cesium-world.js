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
 * CesiumWorld
 *
 * Wraps Cesium/Google Photorealistic 3D Tiles behind the local metre-based
 * coordinate convention already used by the drone physics:
 *
 *   local x = east, local y = up, local z = north
 *
 * Cesium itself renders in ECEF. The conversion is anchored at a user-selected
 * origin so the existing controller, physics and HUD do not need to know about
 * longitude/latitude.
 */

const DEFAULT_ION_TOKEN = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiJlMTg2MGFhOS02YTdhLTQ1NWMtYjkzMi05YjQ2ODRlZjI5YTgiLCJpZCI6MjUxNzM1LCJpYXQiOjE3MzAyODI0ODN9.prWAxx4RB8teelutQQbVqdxhgRZpZ4zjw8wzM-8k1Ug';
const DEFAULT_ASSET_ID = 2275207;
const DEFAULT_VIEW = {
    longitude: 114.1690321,
    latitude: 22.3246282,
    height: 1800,
};

function urlNumber(name, fallback) {
    try {
        const v = new URLSearchParams(window.location.search).get(name);
        if (v == null || v === '') return fallback;
        const n = Number(v);
        return Number.isFinite(n) ? n : fallback;
    } catch (_) {
        return fallback;
    }
}

function urlString(name, fallback) {
    try {
        const v = new URLSearchParams(window.location.search).get(name);
        return v == null || v === '' ? fallback : v;
    } catch (_) {
        return fallback;
    }
}

function requireCesium() {
    if (!window.Cesium) {
        throw new Error('CesiumJS is not loaded. Run via the Docker image or provide /ThirdParty/Cesium/Cesium.js.');
    }
    return window.Cesium;
}

function rotateVectorByQuat(q, v) {
    // q * v * q^-1; q is expected to rotate body-local vectors into the
    // app-local world frame used by Drone.
    const x = v.x, y = v.y, z = v.z;
    const qx = q.x, qy = q.y, qz = q.z, qw = q.w;

    const ix =  qw * x + qy * z - qz * y;
    const iy =  qw * y + qz * x - qx * z;
    const iz =  qw * z + qx * y - qy * x;
    const iw = -qx * x - qy * y - qz * z;

    return {
        x: ix * qw + iw * -qx + iy * -qz - iz * -qy,
        y: iy * qw + iw * -qy + iz * -qx - ix * -qz,
        z: iz * qw + iw * -qz + ix * -qy - iy * -qx,
    };
}

function normalize3(v) {
    const len = Math.hypot(v.x, v.y, v.z);
    if (len < 1e-9) return { x: 0, y: 0, z: 0 };
    return { x: v.x / len, y: v.y / len, z: v.z / len };
}

export class CesiumWorld {
    constructor(containerId, options = {}) {
        this.containerId = containerId;
        this.token = options.token || urlString('ionToken', DEFAULT_ION_TOKEN);
        this.assetId = Number(options.assetId || urlNumber('assetId', DEFAULT_ASSET_ID));
        this.initialView = {
            longitude: urlNumber('lon', options.longitude ?? DEFAULT_VIEW.longitude),
            latitude: urlNumber('lat', options.latitude ?? DEFAULT_VIEW.latitude),
            height: urlNumber('height', options.height ?? DEFAULT_VIEW.height),
        };

        this.Cesium = null;
        this.viewer = null;
        this.tileset = null;
        this.ready = false;

        this.originCartographic = null;
        this.enuToFixed = null;
        this.fixedToEnu = null;
        this.spawnMarker = null;
        this.aircraftEntities = [];
        this._aircraftLines = {
            body: [],
            armA: [],
            armB: [],
            rotorFL: [],
            rotorFR: [],
            rotorRL: [],
            rotorRR: [],
            heading: [],
        };
        this._lastPickWarning = 0;
    }

    async init(progressCb = null) {
        const Cesium = requireCesium();
        this.Cesium = Cesium;
        Cesium.Ion.defaultAccessToken = this.token;

        if (progressCb) progressCb('Creating Cesium viewer...');
        this.viewer = new Cesium.Viewer(this.containerId, {
            animation: false,
            timeline: false,
            baseLayerPicker: false,
            geocoder: true,
            homeButton: true,
            infoBox: false,
            navigationHelpButton: true,
            sceneModePicker: false,
            selectionIndicator: false,
            fullscreenButton: false,
            scene3DOnly: true,
            shouldAnimate: true,
            globe: false,
            skyAtmosphere: new Cesium.SkyAtmosphere(),
            requestRenderMode: false,
        });

        this.viewer.scene.fog.enabled = false;
        this.viewer.scene.highDynamicRange = true;
        this.viewer.scene.screenSpaceCameraController.enableCollisionDetection = false;

        const origin = Cesium.Cartographic.fromDegrees(
            this.initialView.longitude,
            this.initialView.latitude,
            0
        );
        this.setOrigin(origin);

        if (progressCb) progressCb('Loading Google Photorealistic 3D Tiles...');
        this.tileset = await this._createGoogleTileset(progressCb);
        this._configureTilesetStreaming();
        this.viewer.scene.primitives.add(this.tileset);
        this._wireTilesetDiagnostics(progressCb);

        this.viewer.scene.camera.setView({
            destination: Cesium.Cartesian3.fromDegrees(
                this.initialView.longitude,
                this.initialView.latitude,
                this.initialView.height
            ),
            orientation: {
                heading: Cesium.Math.toRadians(0),
                pitch: Cesium.Math.toRadians(-35),
                roll: 0,
            },
        });
        this._configureHomeButton();
        this.viewer.scene.requestRender();
        if (progressCb) progressCb('Waiting for initial Google 3D Tiles...');
        await new Promise(resolve => window.setTimeout(resolve, 150));
        await this.waitForTilesIdle(4500, 250);

        this.ready = true;
        this.viewer.scene.requestRender();
        return this;
    }

    _configureHomeButton() {
        if (!this.viewer || !this.viewer.homeButton) return;
        const Cesium = this.Cesium;
        const command = this.viewer.homeButton.viewModel.command;
        command.beforeExecute.addEventListener((e) => {
            e.cancel = true;
            this.viewer.camera.flyTo({
                destination: Cesium.Cartesian3.fromDegrees(
                    this.initialView.longitude,
                    this.initialView.latitude,
                    this.initialView.height
                ),
                orientation: {
                    heading: Cesium.Math.toRadians(0),
                    pitch: Cesium.Math.toRadians(-35),
                    roll: 0,
                },
                duration: 1.2,
            });
        });
    }

    async _createGoogleTileset(progressCb = null) {
        const Cesium = this.Cesium;
        if (typeof Cesium.createGooglePhotorealistic3DTileset === 'function') {
            try {
                if (progressCb) progressCb('Loading Google Photorealistic 3D Tiles...');
                return await Cesium.createGooglePhotorealistic3DTileset();
            } catch (e) {
                console.warn('[CesiumWorld] createGooglePhotorealistic3DTileset failed; falling back to ion asset:', e);
            }
        }

        if (progressCb) progressCb(`Loading Google Photorealistic 3D Tiles asset ${this.assetId}...`);
        return Cesium.Cesium3DTileset.fromIonAssetId(this.assetId);
    }

    _wireTilesetDiagnostics(progressCb = null) {
        if (!this.tileset) return;
        const onFailure = (error) => {
            const message = error && error.message ? error.message : String(error || 'unknown tile error');
            console.warn('[CesiumWorld] Google tiles failed:', error);
            if (progressCb) progressCb(`Google 3D Tiles request failed: ${message}`, true);
        };

        if (this.tileset.tileFailed && typeof this.tileset.tileFailed.addEventListener === 'function') {
            this.tileset.tileFailed.addEventListener(onFailure);
        }
        if (this.tileset.errorEvent && typeof this.tileset.errorEvent.addEventListener === 'function') {
            this.tileset.errorEvent.addEventListener(onFailure);
        }
    }

    _configureTilesetStreaming() {
        const tileset = this.tileset;
        if (!tileset) return;

        const setIfPresent = (key, value) => {
            if (key in tileset) tileset[key] = value;
        };

        setIfPresent('maximumScreenSpaceError', 12);
        setIfPresent('cullRequestsWhileMoving', false);
        setIfPresent('preloadWhenHidden', true);
        setIfPresent('preloadFlightDestinations', true);
        setIfPresent('foveatedScreenSpaceError', false);
        setIfPresent('loadSiblings', true);
        setIfPresent('skipLevelOfDetail', false);

        if ('maximumMemoryUsage' in tileset) {
            tileset.maximumMemoryUsage = Math.max(tileset.maximumMemoryUsage || 0, 1024);
        }
        if ('cacheBytes' in tileset) {
            tileset.cacheBytes = Math.max(tileset.cacheBytes || 0, 768 * 1024 * 1024);
        }
        if ('maximumCacheOverflowBytes' in tileset) {
            tileset.maximumCacheOverflowBytes = Math.max(tileset.maximumCacheOverflowBytes || 0, 384 * 1024 * 1024);
        }
    }

    waitForTilesIdle(timeoutMs = 1600, quietMs = 180) {
        if (!this.tileset) return Promise.resolve();

        return new Promise((resolve) => {
            const started = performance.now();
            let idleSince = null;
            let done = false;

            const finish = () => {
                if (done) return;
                done = true;
                resolve();
            };

            const tick = () => {
                if (done) return;
                const now = performance.now();
                const loaded = this.tileset.tilesLoaded === true;

                if (loaded) {
                    if (idleSince == null) idleSince = now;
                    if (now - idleSince >= quietMs) return finish();
                } else {
                    idleSince = null;
                }

                if (now - started >= timeoutMs) return finish();
                window.setTimeout(tick, 80);
            };

            tick();
        });
    }

    async preloadLocalArea(centerLocal, options = {}) {
        if (!this.viewer || !this.ready || !centerLocal) return;
        const Cesium = this.Cesium;
        const camera = this.viewer.camera;
        const saved = {
            position: Cesium.Cartesian3.clone(camera.positionWC),
            direction: Cesium.Cartesian3.clone(camera.directionWC),
            up: Cesium.Cartesian3.clone(camera.upWC),
        };

        const radius = Math.max(30, Number.isFinite(options.radius) ? options.radius : 120);
        const lift = Math.max(35, Number.isFinite(options.lift) ? options.lift : 85);
        const dwellMs = Math.max(40, Number.isFinite(options.dwellMs) ? options.dwellMs : 120);
        const perViewTimeoutMs = Math.max(250, Number.isFinite(options.perViewTimeoutMs) ? options.perViewTimeoutMs : 850);
        const progressCb = typeof options.progressCb === 'function' ? options.progressCb : null;
        const target = {
            x: centerLocal.x,
            y: centerLocal.y + 8,
            z: centerLocal.z,
        };

        const views = [
            { x: 0, y: lift * 1.15, z: radius * 0.25 },
            { x: 0, y: lift * 0.85, z: radius },
            { x: radius, y: lift * 0.8, z: 0 },
            { x: -radius, y: lift * 0.8, z: 0 },
            { x: 0, y: lift * 0.8, z: -radius },
        ];

        const delay = (ms) => new Promise(resolve => window.setTimeout(resolve, ms));

        try {
            for (let i = 0; i < views.length; i++) {
                const v = views[i];
                if (progressCb) progressCb(`Preloading nearby Google tiles (${i + 1}/${views.length})...`);
                const eye = {
                    x: centerLocal.x + v.x,
                    y: centerLocal.y + v.y,
                    z: centerLocal.z + v.z,
                };
                const surfaceY = this.sampleHeightAtLocal(eye.x, eye.z, 1.0);
                if (Number.isFinite(surfaceY)) eye.y = Math.max(eye.y, surfaceY + 18);

                const directionLocal = normalize3({
                    x: target.x - eye.x,
                    y: target.y - eye.y,
                    z: target.z - eye.z,
                });
                camera.setView({
                    destination: this.localToCartesian(eye),
                    orientation: {
                        direction: this.localDirectionToFixed(directionLocal),
                        up: this.localDirectionToFixed({ x: 0, y: 1, z: 0 }),
                    },
                });
                this.viewer.scene.requestRender();
                await delay(dwellMs);
                await this.waitForTilesIdle(perViewTimeoutMs);
            }
        } finally {
            camera.setView({
                destination: saved.position,
                orientation: {
                    direction: saved.direction,
                    up: saved.up,
                },
            });
            this.viewer.scene.requestRender();
        }
    }

    destroy() {
        if (this.viewer && !this.viewer.isDestroyed()) {
            this.viewer.destroy();
        }
        this.viewer = null;
        this.tileset = null;
        this.ready = false;
    }

    setOrigin(cartographic) {
        const Cesium = this.Cesium || requireCesium();
        this.originCartographic = new Cesium.Cartographic(
            cartographic.longitude,
            cartographic.latitude,
            cartographic.height || 0
        );
        const originCartesian = Cesium.Cartesian3.fromRadians(
            this.originCartographic.longitude,
            this.originCartographic.latitude,
            this.originCartographic.height
        );
        this.enuToFixed = Cesium.Transforms.eastNorthUpToFixedFrame(originCartesian);
        this.fixedToEnu = Cesium.Matrix4.inverse(this.enuToFixed, new Cesium.Matrix4());
    }

    localToCartesian(local) {
        const Cesium = this.Cesium;
        const enu = new Cesium.Cartesian3(local.x, local.z, local.y);
        return Cesium.Matrix4.multiplyByPoint(this.enuToFixed, enu, new Cesium.Cartesian3());
    }

    localToCartographic(local) {
        const Cesium = this.Cesium;
        return Cesium.Cartographic.fromCartesian(this.localToCartesian(local));
    }

    cartesianToLocal(cartesian) {
        const Cesium = this.Cesium;
        const enu = Cesium.Matrix4.multiplyByPoint(this.fixedToEnu, cartesian, new Cesium.Cartesian3());
        return { x: enu.x, y: enu.z, z: enu.y };
    }

    localDirectionToFixed(direction) {
        const Cesium = this.Cesium;
        const enu = new Cesium.Cartesian3(direction.x, direction.z, direction.y);
        const fixed = Cesium.Matrix4.multiplyByPointAsVector(this.enuToFixed, enu, new Cesium.Cartesian3());
        return Cesium.Cartesian3.normalize(fixed, fixed);
    }

    setNativeCameraControls(enabled) {
        if (!this.viewer) return;
        const c = this.viewer.scene.screenSpaceCameraController;
        c.enableRotate = enabled;
        c.enableTranslate = enabled;
        c.enableZoom = enabled;
        c.enableTilt = enabled;
        c.enableLook = enabled;
    }

    async pickSpawn(windowPosition, altitudeMeters = 100) {
        const Cesium = this.Cesium;
        const scene = this.viewer.scene;
        let cartesian = null;

        try {
            const picked = scene.pick(windowPosition);
            if (picked && scene.pickPositionSupported) {
                const p = scene.pickPosition(windowPosition);
                if (Cesium.defined(p)) cartesian = p;
            }
        } catch (_) {
            cartesian = null;
        }

        if (!cartesian) {
            try {
                const ray = this.viewer.camera.getPickRay(windowPosition);
                if (ray && typeof scene.pickFromRay === 'function') {
                    const hit = scene.pickFromRay(ray);
                    if (hit && Cesium.defined(hit.position)) cartesian = hit.position;
                }
            } catch (_) {
                cartesian = null;
            }
        }

        if (!cartesian) {
            const ray = this.viewer.camera.getPickRay(windowPosition);
            const ellipsoidHit = ray
                ? Cesium.IntersectionTests.rayEllipsoid(ray, Cesium.Ellipsoid.WGS84)
                : null;
            if (ellipsoidHit) {
                cartesian = Cesium.Ray.getPoint(ray, ellipsoidHit.start, new Cesium.Cartesian3());
            }
        }

        if (!cartesian) return null;

        const cartographic = Cesium.Cartographic.fromCartesian(cartesian);
        const surfaceAtOriginHeight = Cesium.Cartesian3.fromRadians(
            cartographic.longitude,
            cartographic.latitude,
            this.originCartographic ? this.originCartographic.height : 0
        );
        const surface = this.cartesianToLocal(surfaceAtOriginHeight);
        const spawn = { x: surface.x, y: Math.max(0, altitudeMeters || 0), z: surface.z };
        this.updateSpawnMarker(spawn);
        return spawn;
    }

    updateSpawnMarker(local) {
        if (!this.viewer || !local) return;
        const Cesium = this.Cesium;
        const position = this.localToCartesian(local);
        if (!this.spawnMarker) {
            this.spawnMarker = this.viewer.entities.add({
                name: 'spawn-point',
                position,
                point: {
                    pixelSize: 14,
                    color: Cesium.Color.CYAN,
                    outlineColor: Cesium.Color.WHITE,
                    outlineWidth: 2,
                    disableDepthTestDistance: Number.POSITIVE_INFINITY,
                },
                label: {
                    text: 'SPAWN',
                    font: '12px sans-serif',
                    pixelOffset: new Cesium.Cartesian2(0, -24),
                    fillColor: Cesium.Color.CYAN,
                    outlineColor: Cesium.Color.BLACK,
                    outlineWidth: 2,
                    style: Cesium.LabelStyle.FILL_AND_OUTLINE,
                    disableDepthTestDistance: Number.POSITIVE_INFINITY,
                },
            });
        } else {
            this.spawnMarker.position = position;
            this.spawnMarker.show = true;
        }
        this.viewer.scene.requestRender();
    }

    hideSpawnMarker() {
        if (this.spawnMarker) this.spawnMarker.show = false;
    }

    _collisionExclusions() {
        const excluded = [];
        if (this.spawnMarker) excluded.push(this.spawnMarker);
        for (const entity of this.aircraftEntities) {
            if (entity) excluded.push(entity);
        }
        return excluded;
    }

    _isExcludedCollisionHit(hit) {
        if (!hit || !hit.object) return false;
        const object = hit.object;
        const entity = object.id || object;
        if (this.spawnMarker && (object === this.spawnMarker || entity === this.spawnMarker)) return true;
        return this.aircraftEntities.some(e => e && (object === e || entity === e));
    }

    _ensureAircraft() {
        if (this.aircraftEntities.length || !this.viewer) return;
        const Cesium = this.Cesium;
        const makeLine = (key, color, width = 4) => this.viewer.entities.add({
            name: `drone-${key}`,
            polyline: {
                positions: new Cesium.CallbackProperty(() => this._aircraftLines[key], false),
                width,
                material: color,
                depthFailMaterial: Cesium.Color.fromAlpha(color, 0.35),
            },
            show: false,
        });

        this.aircraftEntities.push(makeLine('body', Cesium.Color.CYAN, 5));
        this.aircraftEntities.push(makeLine('armA', Cesium.Color.WHITE, 4));
        this.aircraftEntities.push(makeLine('armB', Cesium.Color.WHITE, 4));
        this.aircraftEntities.push(makeLine('rotorFL', Cesium.Color.fromAlpha(Cesium.Color.CYAN, 0.9), 2));
        this.aircraftEntities.push(makeLine('rotorFR', Cesium.Color.fromAlpha(Cesium.Color.CYAN, 0.9), 2));
        this.aircraftEntities.push(makeLine('rotorRL', Cesium.Color.fromAlpha(Cesium.Color.ORANGE, 0.9), 2));
        this.aircraftEntities.push(makeLine('rotorRR', Cesium.Color.fromAlpha(Cesium.Color.ORANGE, 0.9), 2));
        this.aircraftEntities.push(makeLine('heading', Cesium.Color.LIME, 3));
    }

    showAircraft(show) {
        this._ensureAircraft();
        for (const e of this.aircraftEntities) e.show = !!show;
    }

    _aircraftPoint(transform, offset) {
        const rotated = rotateVectorByQuat(transform.orientation, offset);
        return this.localToCartesian({
            x: transform.position.x + rotated.x,
            y: transform.position.y + rotated.y,
            z: transform.position.z + rotated.z,
        });
    }

    _aircraftRotor(transform, center, radius = 0.22, segments = 20) {
        const points = [];
        for (let i = 0; i <= segments; i++) {
            const a = (i / segments) * Math.PI * 2;
            points.push(this._aircraftPoint(transform, {
                x: center.x + Math.cos(a) * radius,
                y: center.y,
                z: center.z + Math.sin(a) * radius,
            }));
        }
        return points;
    }

    updateAircraftFromDroneTransform(transform) {
        if (!this.viewer || !transform || !transform.orientation) return;
        this._ensureAircraft();

        const frontLeft = { x: -0.68, y: 0.02, z: -0.68 };
        const frontRight = { x: 0.68, y: 0.02, z: -0.68 };
        const rearLeft = { x: -0.68, y: 0.02, z: 0.68 };
        const rearRight = { x: 0.68, y: 0.02, z: 0.68 };

        this._aircraftLines.body = [
            this._aircraftPoint(transform, { x: 0, y: 0.10, z: -0.24 }),
            this._aircraftPoint(transform, { x: 0.24, y: 0.06, z: 0 }),
            this._aircraftPoint(transform, { x: 0, y: 0.10, z: 0.24 }),
            this._aircraftPoint(transform, { x: -0.24, y: 0.06, z: 0 }),
            this._aircraftPoint(transform, { x: 0, y: 0.10, z: -0.24 }),
        ];
        this._aircraftLines.armA = [
            this._aircraftPoint(transform, frontLeft),
            this._aircraftPoint(transform, rearRight),
        ];
        this._aircraftLines.armB = [
            this._aircraftPoint(transform, frontRight),
            this._aircraftPoint(transform, rearLeft),
        ];
        this._aircraftLines.rotorFL = this._aircraftRotor(transform, frontLeft);
        this._aircraftLines.rotorFR = this._aircraftRotor(transform, frontRight);
        this._aircraftLines.rotorRL = this._aircraftRotor(transform, rearLeft);
        this._aircraftLines.rotorRR = this._aircraftRotor(transform, rearRight);
        this._aircraftLines.heading = [
            this._aircraftPoint(transform, { x: 0, y: 0.18, z: -0.18 }),
            this._aircraftPoint(transform, { x: 0, y: 0.18, z: -1.08 }),
        ];
    }

    sampleHeightAtLocal(x, z, width = 0.4) {
        if (!this.viewer || !this.ready) return null;
        const Cesium = this.Cesium;
        const scene = this.viewer.scene;
        if (typeof scene.sampleHeight !== 'function') return null;

        const carto = this.localToCartographic({ x, y: 0, z });
        let sampledHeight;
        try {
            sampledHeight = scene.sampleHeight(carto, this._collisionExclusions(), width);
        } catch (_) {
            try {
                sampledHeight = scene.sampleHeight(carto, undefined, width);
            } catch (_) {
                return null;
            }
        }
        if (!Number.isFinite(sampledHeight)) return null;

        const surfaceCartesian = Cesium.Cartesian3.fromRadians(
            carto.longitude,
            carto.latitude,
            sampledHeight
        );
        return this.cartesianToLocal(surfaceCartesian).y;
    }

    pickLocalRay(originLocal, directionLocal, maxDistance) {
        if (!this.viewer || !this.ready) return null;
        const Cesium = this.Cesium;
        const scene = this.viewer.scene;
        if (typeof scene.pickFromRay !== 'function') {
            const now = performance.now();
            if (now - this._lastPickWarning > 5000) {
                console.warn('[CesiumWorld] scene.pickFromRay is unavailable; collision uses height sampling only.');
                this._lastPickWarning = now;
            }
            return null;
        }

        const dir = normalize3(directionLocal);
        if (Math.hypot(dir.x, dir.y, dir.z) < 1e-6) return null;

        const origin = this.localToCartesian(originLocal);
        const direction = this.localDirectionToFixed(dir);
        const ray = new Cesium.Ray(origin, direction);

        let hit;
        try {
            hit = scene.pickFromRay(ray, this._collisionExclusions());
        } catch (_) {
            return null;
        }
        if (!hit || !Cesium.defined(hit.position)) return null;
        if (this._isExcludedCollisionHit(hit)) return null;

        const local = this.cartesianToLocal(hit.position);
        const dx = local.x - originLocal.x;
        const dy = local.y - originLocal.y;
        const dz = local.z - originLocal.z;
        const distance = Math.hypot(dx, dy, dz);
        if (!Number.isFinite(distance) || distance > maxDistance) return null;
        return { position: local, distance };
    }

    setCameraFromDroneTransform(transform, hfovDeg) {
        if (!this.viewer || !this.ready || !transform || !transform.orientation) return;
        const Cesium = this.Cesium;
        const aspect = Math.max(0.1, this.viewer.canvas.clientWidth / Math.max(1, this.viewer.canvas.clientHeight));
        const hfov = Cesium.Math.toRadians(Math.max(30, Math.min(140, hfovDeg || 100)));
        const vfov = 2 * Math.atan(Math.tan(hfov * 0.5) / aspect);
        if (this.viewer.camera.frustum && Number.isFinite(vfov)) {
            this.viewer.camera.frustum.fov = vfov;
            this.viewer.camera.frustum.near = 0.03;
            this.viewer.camera.frustum.far = 15000000;
        }

        const q = transform.orientation;
        const forwardLocal = rotateVectorByQuat(q, { x: 0, y: 0, z: -1 });
        const upLocal = rotateVectorByQuat(q, { x: 0, y: 1, z: 0 });

        const destination = this.localToCartesian(transform.position);
        const direction = this.localDirectionToFixed(forwardLocal);
        const up = this.localDirectionToFixed(upLocal);

        this.viewer.camera.setView({
            destination,
            orientation: { direction, up },
        });
    }

    getForwardLocal(transform) {
        if (!transform || !transform.orientation) return { x: 0, y: 0, z: -1 };
        return normalize3(rotateVectorByQuat(transform.orientation, { x: 0, y: 0, z: -1 }));
    }

    setThirdPersonCamera(transform, state = {}) {
        if (!this.viewer || !this.ready || !transform || !transform.position) return;
        const Cesium = this.Cesium;
        const distance = Math.max(2.0, Math.min(120.0, state.distance || 16.0));
        const pitch = Math.max(-1.1, Math.min(1.15, state.pitch ?? 0.28));
        const yaw = Number.isFinite(state.yaw) ? state.yaw : 0;
        const lateral = Number.isFinite(state.lateral) ? state.lateral : 0;
        const height = Number.isFinite(state.height) ? state.height : 0.6;

        const cosPitch = Math.cos(pitch);
        const target = {
            x: transform.position.x,
            y: transform.position.y + height,
            z: transform.position.z,
        };
        const offset = {
            x: Math.sin(yaw) * cosPitch * distance + Math.cos(yaw) * lateral,
            y: Math.sin(pitch) * distance + height,
            z: Math.cos(yaw) * cosPitch * distance - Math.sin(yaw) * lateral,
        };
        const cameraLocal = {
            x: transform.position.x + offset.x,
            y: transform.position.y + offset.y,
            z: transform.position.z + offset.z,
        };
        const cameraSurfaceY = this.sampleHeightAtLocal(cameraLocal.x, cameraLocal.z, 0.8);
        if (Number.isFinite(cameraSurfaceY)) {
            cameraLocal.y = Math.max(cameraLocal.y, cameraSurfaceY + 4.0);
        }
        const directionLocal = normalize3({
            x: target.x - cameraLocal.x,
            y: target.y - cameraLocal.y,
            z: target.z - cameraLocal.z,
        });

        const destination = this.localToCartesian(cameraLocal);
        const direction = this.localDirectionToFixed(directionLocal);
        const up = this.localDirectionToFixed({ x: 0, y: 1, z: 0 });

        if (this.viewer.camera.frustum) {
            this.viewer.camera.frustum.near = 0.03;
            this.viewer.camera.frustum.far = 15000000;
        }
        this.viewer.camera.setView({
            destination,
            orientation: { direction, up },
        });
        this.viewer.scene.requestRender();
    }

    describeLocal(local) {
        if (!local) return '';
        const carto = this.localToCartographic(local);
        return [
            `lon ${this.Cesium.Math.toDegrees(carto.longitude).toFixed(6)}`,
            `lat ${this.Cesium.Math.toDegrees(carto.latitude).toFixed(6)}`,
            `alt ${local.y.toFixed(1)} m`,
        ].join(' | ');
    }

    describeSpawn(local, altitudeMeters) {
        if (!local) return '';
        const carto = this.localToCartographic({ x: local.x, y: 0, z: local.z });
        return [
            `lon ${this.Cesium.Math.toDegrees(carto.longitude).toFixed(6)}`,
            `lat ${this.Cesium.Math.toDegrees(carto.latitude).toFixed(6)}`,
            `alt ${Number(altitudeMeters || 0).toFixed(1)} m`,
        ].join(' | ');
    }
}
