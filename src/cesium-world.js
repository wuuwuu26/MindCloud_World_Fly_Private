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
const CESIUM_DRONE_MODEL_URI = 'asset/models/CesiumDrone.glb';
const HEIGHT_CACHE_TTL_MS = 140;
const HEIGHT_CACHE_LIMIT = 256;
const PANORAMA_FACE_DEFS = [
    { name: 'front', dir: { x: 0, y: 0, z: -1 }, up: { x: 0, y: 1, z: 0 } },
    { name: 'right', dir: { x: 1, y: 0, z: 0 }, up: { x: 0, y: 1, z: 0 } },
    { name: 'back', dir: { x: 0, y: 0, z: 1 }, up: { x: 0, y: 1, z: 0 } },
    { name: 'left', dir: { x: -1, y: 0, z: 0 }, up: { x: 0, y: 1, z: 0 } },
    { name: 'up', dir: { x: 0, y: 1, z: 0 }, up: { x: 0, y: 0, z: 1 } },
    { name: 'down', dir: { x: 0, y: -1, z: 0 }, up: { x: 0, y: 0, z: -1 } },
];
const PANORAMA_SIDE_FACE_DEFS = PANORAMA_FACE_DEFS.filter(face => (
    face.name === 'front' || face.name === 'right' || face.name === 'back' || face.name === 'left'
));

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

function cross3(a, b) {
    return {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    };
}

function negate3(v) {
    return { x: -v.x, y: -v.y, z: -v.z };
}

function choosePanoramaFaceComponents(x, y, z) {
    const ax = Math.abs(x);
    const ay = Math.abs(y);
    const az = Math.abs(z);
    if (ay >= ax && ay >= az) return y >= 0 ? 'up' : 'down';
    if (ax >= az) return x >= 0 ? 'right' : 'left';
    return z >= 0 ? 'back' : 'front';
}

function writeBilinearPixel(image, sx, sy, outData, dstIdx) {
    const width = image.width;
    const height = image.height;
    const x = Math.max(0, Math.min(width - 1, sx));
    const y = Math.max(0, Math.min(height - 1, sy));
    const x0 = Math.floor(x);
    const y0 = Math.floor(y);
    const x1 = Math.min(width - 1, x0 + 1);
    const y1 = Math.min(height - 1, y0 + 1);
    const tx = x - x0;
    const ty = y - y0;
    const wx0 = 1 - tx;
    const wy0 = 1 - ty;
    const w00 = wx0 * wy0;
    const w10 = tx * wy0;
    const w01 = wx0 * ty;
    const w11 = tx * ty;
    const data = image.data;
    const i00 = (y0 * width + x0) * 4;
    const i10 = (y0 * width + x1) * 4;
    const i01 = (y1 * width + x0) * 4;
    const i11 = (y1 * width + x1) * 4;

    outData[dstIdx] = (
        data[i00] * w00 + data[i10] * w10 + data[i01] * w01 + data[i11] * w11 + 0.5
    ) | 0;
    outData[dstIdx + 1] = (
        data[i00 + 1] * w00 + data[i10 + 1] * w10 + data[i01 + 1] * w01 + data[i11 + 1] * w11 + 0.5
    ) | 0;
    outData[dstIdx + 2] = (
        data[i00 + 2] * w00 + data[i10 + 2] * w10 + data[i01 + 2] * w01 + data[i11 + 2] * w11 + 0.5
    ) | 0;
    outData[dstIdx + 3] = 255;
}

function imageBlackRatio(image) {
    const data = image.data;
    const pixels = Math.max(1, image.width * image.height);
    let black = 0;
    for (let i = 0; i < data.length; i += 4) {
        if (data[i] < 10 && data[i + 1] < 10 && data[i + 2] < 10) black++;
    }
    return black / pixels;
}

function clonePanoramaTransform(transform) {
    return {
        position: {
            x: transform.position.x,
            y: transform.position.y,
            z: transform.position.z,
        },
        orientation: {
            x: transform.orientation.x,
            y: transform.orientation.y,
            z: transform.orientation.z,
            w: transform.orientation.w,
        },
    };
}

function compilePanoramaShader(gl, type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
        const message = gl.getShaderInfoLog(shader) || 'unknown shader compile error';
        gl.deleteShader(shader);
        throw new Error(message);
    }
    return shader;
}

function createPanoramaProgram(gl, vertexSource, fragmentSource) {
    const vertex = compilePanoramaShader(gl, gl.VERTEX_SHADER, vertexSource);
    const fragment = compilePanoramaShader(gl, gl.FRAGMENT_SHADER, fragmentSource);
    const program = gl.createProgram();
    gl.attachShader(program, vertex);
    gl.attachShader(program, fragment);
    gl.linkProgram(program);
    gl.deleteShader(vertex);
    gl.deleteShader(fragment);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
        const message = gl.getProgramInfoLog(program) || 'unknown shader link error';
        gl.deleteProgram(program);
        throw new Error(message);
    }
    return program;
}

class PanoramaEquirectProjector {
    constructor() {
        this.canvas = document.createElement('canvas');
        const gl = this.canvas.getContext('webgl', {
            alpha: false,
            antialias: false,
            depth: false,
            stencil: false,
            preserveDrawingBuffer: true,
            powerPreference: 'high-performance',
        });
        if (!gl) throw new Error('WebGL is unavailable for panorama projection.');
        this.gl = gl;
        this.readyFaces = new Set();
        this.faceNames = ['front', 'right', 'back', 'left', 'up', 'down'];
        this.textures = new Map();

        this.program = createPanoramaProgram(gl, `
            attribute vec2 a_position;
            varying vec2 v_uv;
            void main() {
                v_uv = a_position * 0.5 + 0.5;
                gl_Position = vec4(a_position, 0.0, 1.0);
            }
        `, `
            precision mediump float;
            varying vec2 v_uv;
            uniform float u_vertical_fov;
            uniform float u_tan_half_face_fov;
            uniform int u_projection_mode;
            uniform sampler2D u_front;
            uniform sampler2D u_right;
            uniform sampler2D u_back;
            uniform sampler2D u_left;
            uniform sampler2D u_up;
            uniform sampler2D u_down;

            const float PI = 3.141592653589793;
            const float TWO_PI = 6.283185307179586;

            vec2 faceUv(vec3 dir, vec3 faceDir, vec3 faceRight, vec3 faceUp) {
                float denom = max(dot(dir, faceDir), 0.000001);
                float u = dot(dir, faceRight) / (denom * u_tan_half_face_fov);
                float v = dot(dir, faceUp) / (denom * u_tan_half_face_fov);
                return clamp(vec2(u * 0.5 + 0.5, 0.5 - v * 0.5), 0.001, 0.999);
            }

            vec4 sampleXFace(vec3 dir) {
                if (dir.x >= 0.0) {
                    return texture2D(u_right, faceUv(dir, vec3(1.0, 0.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 1.0, 0.0)));
                }
                return texture2D(u_left, faceUv(dir, vec3(-1.0, 0.0, 0.0), vec3(0.0, 0.0, -1.0), vec3(0.0, 1.0, 0.0)));
            }

            vec4 sampleYFace(vec3 dir) {
                if (dir.y >= 0.0) {
                    return texture2D(u_up, faceUv(dir, vec3(0.0, 1.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(0.0, 0.0, 1.0)));
                }
                return texture2D(u_down, faceUv(dir, vec3(0.0, -1.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(0.0, 0.0, -1.0)));
            }

            vec4 sampleZFace(vec3 dir) {
                if (dir.z >= 0.0) {
                    return texture2D(u_back, faceUv(dir, vec3(0.0, 0.0, 1.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0)));
                }
                return texture2D(u_front, faceUv(dir, vec3(0.0, 0.0, -1.0), vec3(1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0)));
            }

            vec4 sampleSideRing(vec3 dir) {
                vec3 horizontal = normalize(vec3(dir.x, 0.0, dir.z));
                vec3 a = abs(horizontal);
                if (a.x >= a.z) {
                    return sampleXFace(dir);
                }
                return sampleZFace(dir);
            }

            vec4 sampleHybridRing(vec3 dir) {
                vec4 side = sampleSideRing(dir);
                vec4 cap = sampleYFace(dir);
                float capBlend = smoothstep(0.78, 0.90, abs(dir.y));
                return mix(side, cap, capBlend);
            }

            vec4 sampleFace(vec3 dir) {
                if (u_projection_mode == 1) {
                    return sampleSideRing(dir);
                }
                if (u_projection_mode == 2) {
                    return sampleHybridRing(dir);
                }

                vec3 a = abs(dir);
                vec4 cx = sampleXFace(dir);
                vec4 cy = sampleYFace(dir);
                vec4 cz = sampleZFace(dir);
                vec4 primary;
                vec4 secondary;
                float primaryAxis;
                float secondaryAxis;

                if (a.y >= a.x && a.y >= a.z) {
                    primary = cy;
                    primaryAxis = a.y;
                    if (a.x >= a.z) {
                        secondary = cx;
                        secondaryAxis = a.x;
                    } else {
                        secondary = cz;
                        secondaryAxis = a.z;
                    }
                } else if (a.x >= a.z) {
                    primary = cx;
                    primaryAxis = a.x;
                    if (a.y >= a.z) {
                        secondary = cy;
                        secondaryAxis = a.y;
                    } else {
                        secondary = cz;
                        secondaryAxis = a.z;
                    }
                } else {
                    primary = cz;
                    primaryAxis = a.z;
                    if (a.y >= a.x) {
                        secondary = cy;
                        secondaryAxis = a.y;
                    } else {
                        secondary = cx;
                        secondaryAxis = a.x;
                    }
                }

                float seam = 1.0 - smoothstep(0.0, 0.08, primaryAxis - secondaryAxis);
                return mix(primary, secondary, seam * 0.35);
            }

            void main() {
                float pitch = (v_uv.y - 0.5) * u_vertical_fov;
                float yaw = PI - v_uv.x * TWO_PI;
                float cosPitch = cos(pitch);
                float forward = cosPitch * cos(yaw);
                float left = cosPitch * sin(yaw);
                vec3 dir = normalize(vec3(-left, sin(pitch), -forward));
                gl_FragColor = sampleFace(dir);
            }
        `);

        this.positionBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
        gl.bufferData(
            gl.ARRAY_BUFFER,
            new Float32Array([-1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1]),
            gl.STATIC_DRAW
        );

        gl.useProgram(this.program);
        this.locations = {
            position: gl.getAttribLocation(this.program, 'a_position'),
            verticalFov: gl.getUniformLocation(this.program, 'u_vertical_fov'),
            tanHalfFaceFov: gl.getUniformLocation(this.program, 'u_tan_half_face_fov'),
            projectionMode: gl.getUniformLocation(this.program, 'u_projection_mode'),
        };
        this.faceNames.forEach((name, i) => {
            const texture = gl.createTexture();
            gl.activeTexture(gl.TEXTURE0 + i);
            gl.bindTexture(gl.TEXTURE_2D, texture);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
            gl.uniform1i(gl.getUniformLocation(this.program, `u_${name}`), i);
            this.textures.set(name, texture);
        });
    }

    updateFace(name, sourceCanvas) {
        const gl = this.gl;
        const texture = this.textures.get(name);
        if (!texture || !sourceCanvas || !sourceCanvas.width || !sourceCanvas.height) return;
        gl.bindTexture(gl.TEXTURE_2D, texture);
        gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, sourceCanvas);
        this.readyFaces.add(name);
    }

    render(width, height, verticalFovDeg, projectionMode = 'hybrid', faceFovDeg = 90) {
        const sideMode = projectionMode === 'side';
        const hybridMode = projectionMode === 'hybrid';
        const requiredFaces = sideMode ? ['front', 'right', 'back', 'left'] : this.faceNames;
        if (!requiredFaces.every(name => this.readyFaces.has(name))) return null;
        const gl = this.gl;
        if (this.canvas.width !== width) this.canvas.width = width;
        if (this.canvas.height !== height) this.canvas.height = height;

        gl.viewport(0, 0, width, height);
        gl.disable(gl.DEPTH_TEST);
        gl.disable(gl.BLEND);
        gl.useProgram(this.program);
        gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
        gl.enableVertexAttribArray(this.locations.position);
        gl.vertexAttribPointer(this.locations.position, 2, gl.FLOAT, false, 0, 0);
        this.faceNames.forEach((name, i) => {
            gl.activeTexture(gl.TEXTURE0 + i);
            gl.bindTexture(gl.TEXTURE_2D, this.textures.get(name));
        });
        const verticalFov = Math.max(1, Math.min(180, verticalFovDeg || 180)) * Math.PI / 180;
        const faceFov = Math.max(45, Math.min(170, faceFovDeg || 90)) * Math.PI / 180;
        gl.uniform1f(this.locations.verticalFov, verticalFov);
        gl.uniform1f(this.locations.tanHalfFaceFov, Math.tan(faceFov * 0.5));
        gl.uniform1i(this.locations.projectionMode, sideMode ? 1 : (hybridMode ? 2 : 0));
        gl.drawArrays(gl.TRIANGLES, 0, 6);
        gl.flush();
        return this.canvas;
    }
}

function getTransformBasisLocal(transform) {
    if (!transform || !transform.orientation) {
        const right = { x: 1, y: 0, z: 0 };
        const up = { x: 0, y: 1, z: 0 };
        const back = { x: 0, y: 0, z: 1 };
        return {
            right,
            left: negate3(right),
            up,
            down: negate3(up),
            back,
            forward: negate3(back),
        };
    }

    const q = transform.orientation;
    const right = normalize3(rotateVectorByQuat(q, { x: 1, y: 0, z: 0 }));
    const up = normalize3(rotateVectorByQuat(q, { x: 0, y: 1, z: 0 }));
    const back = normalize3(rotateVectorByQuat(q, { x: 0, y: 0, z: 1 }));
    return {
        right,
        left: negate3(right),
        up,
        down: negate3(up),
        back,
        forward: negate3(back),
    };
}

function clampNumber(value, min, max, fallback) {
    const n = Number(value);
    if (!Number.isFinite(n)) return fallback;
    return Math.max(min, Math.min(max, n));
}

function rotateXZ(v, radians) {
    const c = Math.cos(radians);
    const s = Math.sin(radians);
    return {
        x: v.x * c - v.z * s,
        z: v.x * s + v.z * c,
    };
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
        this.flightResolutionScale = clampNumber(
            urlNumber('resolutionScale', options.resolutionScale ?? 0.72),
            0.45,
            1,
            0.72
        );
        this.placementResolutionScale = clampNumber(
            urlNumber('placementResolutionScale', options.placementResolutionScale ?? 0.88),
            0.5,
            1,
            0.88
        );
        this.flightTileSSE = clampNumber(
            urlNumber('flightTileSse', options.flightTileSSE ?? 24),
            8,
            64,
            24
        );
        this.placementTileSSE = clampNumber(
            urlNumber('placementTileSse', options.placementTileSSE ?? 16),
            8,
            64,
            16
        );
        this.tileCacheMb = Math.round(clampNumber(
            urlNumber('tileCacheMb', options.tileCacheMb ?? 2048),
            512,
            8192,
            2048
        ));
        this.panoramaTileSSE = clampNumber(
            urlNumber('panoramaTileSse', options.panoramaTileSSE ?? 48),
            4,
            128,
            48
        );
        this.panoramaSettleMs = clampNumber(
            urlNumber('panoSettleMs', options.panoramaSettleMs ?? 0),
            0,
            5200,
            0
        );

        this.Cesium = null;
        this.viewer = null;
        this.tileset = null;
        this.ready = false;
        this._panoramaViewer = null;
        this._panoramaTileset = null;
        this._panoramaContainer = null;
        this._panoramaInitPromise = null;
        this._panoramaFaceSize = 0;
        this._panoramaTileLoadPending = null;
        this._panoramaTileLoadProcessing = null;
        this._panoramaProjector = null;
        this._panoramaScanState = null;

        this.originCartographic = null;
        this.enuToFixed = null;
        this.fixedToEnu = null;
        this.spawnMarker = null;
        this.aircraftEntities = [];
        this.aircraftModelEntity = null;
        this._aircraftModelPosition = null;
        this._aircraftModelOrientation = null;
        this._tileLoadPending = null;
        this._tileLoadProcessing = null;
        this._lastPickWarning = 0;
        this._heightSampleCache = new Map();
        this._flightPerformanceMode = false;
    }

    async init(progressCb = null) {
        const Cesium = requireCesium();
        this.Cesium = Cesium;
        Cesium.Ion.defaultAccessToken = this.token;

        if (Cesium.RequestScheduler && 'maximumRequestsPerServer' in Cesium.RequestScheduler) {
            Cesium.RequestScheduler.maximumRequestsPerServer = Math.max(
                Cesium.RequestScheduler.maximumRequestsPerServer || 0,
                18
            );
        }

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
            targetFrameRate: 60,
            useBrowserRecommendedResolution: true,
            orderIndependentTranslucency: false,
            contextOptions: {
                webgl: {
                    alpha: false,
                    antialias: false,
                    preserveDrawingBuffer: true,
                    powerPreference: 'high-performance',
                    failIfMajorPerformanceCaveat: false,
                },
            },
        });

        this.viewer.scene.fog.enabled = false;
        this.viewer.scene.highDynamicRange = false;
        this.viewer.scene.screenSpaceCameraController.enableCollisionDetection = false;
        this._configureScenePerformance(false);

        const origin = Cesium.Cartographic.fromDegrees(
            this.initialView.longitude,
            this.initialView.latitude,
            0
        );
        this.setOrigin(origin);

        if (progressCb) progressCb('Loading Google Photorealistic 3D Tiles...');
        this.tileset = await this._createGoogleTileset(progressCb);
        this._configureTilesetStreaming(false);
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
        if (this.tileset.loadProgress && typeof this.tileset.loadProgress.addEventListener === 'function') {
            this.tileset.loadProgress.addEventListener((pending, processing) => {
                this._tileLoadPending = Math.max(0, Number(pending) || 0);
                this._tileLoadProcessing = Math.max(0, Number(processing) || 0);
            });
        }
    }

    _configureScenePerformance(flightMode = this._flightPerformanceMode) {
        if (!this.viewer || !this.viewer.scene) return;
        const scene = this.viewer.scene;
        const resolutionScale = flightMode ? this.flightResolutionScale : this.placementResolutionScale;

        if ('resolutionScale' in this.viewer) {
            this.viewer.resolutionScale = resolutionScale;
        }
        if ('msaaSamples' in scene) {
            scene.msaaSamples = 1;
        }
        if (scene.postProcessStages && scene.postProcessStages.fxaa) {
            scene.postProcessStages.fxaa.enabled = false;
        }
        scene.highDynamicRange = false;
    }

    _configureTilesetStreaming(flightMode = this._flightPerformanceMode) {
        const tileset = this.tileset;
        if (!tileset) return;

        const setIfPresent = (key, value) => {
            if (key in tileset) tileset[key] = value;
        };

        setIfPresent('maximumScreenSpaceError', flightMode ? this.flightTileSSE : this.placementTileSSE);
        setIfPresent('cullRequestsWhileMoving', true);
        setIfPresent('cullRequestsWhileMovingMultiplier', flightMode ? 90 : 60);
        setIfPresent('preloadWhenHidden', false);
        setIfPresent('preloadFlightDestinations', false);
        setIfPresent('foveatedScreenSpaceError', true);
        setIfPresent('foveatedConeSize', flightMode ? 0.2 : 0.28);
        setIfPresent('foveatedMinimumScreenSpaceErrorRelaxation', flightMode ? 4 : 2);
        setIfPresent('foveatedTimeDelay', flightMode ? 0.08 : 0.15);
        setIfPresent('dynamicScreenSpaceError', true);
        setIfPresent('dynamicScreenSpaceErrorDensity', flightMode ? 0.0035 : 0.0025);
        setIfPresent('dynamicScreenSpaceErrorFactor', flightMode ? 12 : 8);
        setIfPresent('loadSiblings', false);
        setIfPresent('skipLevelOfDetail', true);
        setIfPresent('baseScreenSpaceError', flightMode ? 1536 : 1024);
        setIfPresent('skipScreenSpaceErrorFactor', flightMode ? 18 : 12);
        setIfPresent('skipLevels', flightMode ? 2 : 1);
        setIfPresent('immediatelyLoadDesiredLevelOfDetail', false);
        setIfPresent('preferLeaves', false);

        if ('maximumMemoryUsage' in tileset) {
            tileset.maximumMemoryUsage = Math.max(tileset.maximumMemoryUsage || 0, this.tileCacheMb);
        }
        if ('cacheBytes' in tileset) {
            tileset.cacheBytes = Math.max(tileset.cacheBytes || 0, this.tileCacheMb * 1024 * 1024);
        }
        if ('maximumCacheOverflowBytes' in tileset) {
            const overflowMb = Math.min(768, Math.max(256, Math.round(this.tileCacheMb * 0.35)));
            tileset.maximumCacheOverflowBytes = Math.max(
                tileset.maximumCacheOverflowBytes || 0,
                overflowMb * 1024 * 1024
            );
        }
    }

    setFlightPerformanceMode(enabled) {
        const flightMode = !!enabled;
        if (this._flightPerformanceMode === flightMode) return;
        this._flightPerformanceMode = flightMode;
        this._configureScenePerformance(flightMode);
        this._configureTilesetStreaming(flightMode);
        this.viewer?.scene?.requestRender();
    }

    getTileLoadStatus() {
        return {
            pending: this._tileLoadPending,
            processing: this._tileLoadProcessing,
            tilesLoaded: !!(this.tileset && this.tileset.tilesLoaded === true),
        };
    }

    waitForTilesIdle(timeoutMs = 1600, quietMs = 180) {
        if (!this.tileset) return Promise.resolve(true);

        return new Promise((resolve) => {
            const started = performance.now();
            let idleSince = null;
            let done = false;

            const finish = (idle) => {
                if (done) return;
                done = true;
                resolve(!!idle);
            };

            const tick = () => {
                if (done) return;
                const now = performance.now();
                const queueKnown = this._tileLoadPending !== null || this._tileLoadProcessing !== null;
                const queueIdle = !queueKnown ||
                    ((this._tileLoadPending || 0) <= 0 && (this._tileLoadProcessing || 0) <= 0);
                const loaded = this.tileset.tilesLoaded === true && queueIdle;

                if (loaded) {
                    if (idleSince == null) idleSince = now;
                    if (now - idleSince >= quietMs) return finish(true);
                } else {
                    idleSince = null;
                }

                if (now - started >= timeoutMs) return finish(false);
                window.setTimeout(tick, 80);
            };

            tick();
        });
    }

    _buildPreloadTargets(radius, spacing, maxTargets = 36) {
        const targets = [{ x: 0, z: 0 }];
        const steps = Math.max(1, Math.ceil(radius / spacing));

        for (let iz = -steps; iz <= steps; iz++) {
            for (let ix = -steps; ix <= steps; ix++) {
                const x = ix * spacing;
                const z = iz * spacing;
                const d = Math.hypot(x, z);
                if (d < 1 || d > radius) continue;
                targets.push({ x, z, d });
            }
        }

        targets.sort((a, b) => (a.d || 0) - (b.d || 0));
        return targets.slice(0, Math.max(1, maxTargets));
    }

    _makePreloadView(centerLocal, offset, index, lift, viewDistance) {
        const dist = Math.hypot(offset.x, offset.z);
        const cardinals = [
            { x: 0, z: -1 },
            { x: 1, z: 0 },
            { x: 0, z: 1 },
            { x: -1, z: 0 },
        ];
        const baseDir = dist > 1
            ? { x: -offset.x / dist, z: -offset.z / dist }
            : cardinals[index % cardinals.length];
        const dir = rotateXZ(baseDir, ((index % 3) - 1) * 0.38);
        const target = {
            x: centerLocal.x + offset.x,
            y: centerLocal.y + 8,
            z: centerLocal.z + offset.z,
        };
        return {
            eye: {
                x: target.x - dir.x * viewDistance,
                y: centerLocal.y + lift,
                z: target.z - dir.z * viewDistance,
            },
            target,
        };
    }

    _buildLocalAreaPreloadViews(centerLocal, radius, lift, viewDistance, gridSpacing, maxTargets) {
        const views = [];
        const overviewLift = Math.max(lift * 1.35, 240);
        const overviewDistance = Math.max(viewDistance, Math.min(radius * 0.45, 420));
        const overviewTarget = { x: centerLocal.x, y: centerLocal.y + 20, z: centerLocal.z };
        const overviewDirs = [
            { x: 0, z: 1 },
            { x: 1, z: 0 },
            { x: -1, z: 0 },
            { x: 0, z: -1 },
        ];

        views.push({
            eye: { x: centerLocal.x, y: centerLocal.y + Math.max(overviewLift, radius * 0.35), z: centerLocal.z + Math.min(radius * 0.15, 160) },
            target: overviewTarget,
        });
        for (const dir of overviewDirs) {
            views.push({
                eye: {
                    x: centerLocal.x + dir.x * overviewDistance,
                    y: centerLocal.y + overviewLift,
                    z: centerLocal.z + dir.z * overviewDistance,
                },
                target: overviewTarget,
            });
        }
        for (const dir of overviewDirs) {
            views.push({
                eye: { x: centerLocal.x, y: centerLocal.y + 4, z: centerLocal.z },
                target: {
                    x: centerLocal.x + dir.x * Math.min(radius, 500),
                    y: centerLocal.y + 3,
                    z: centerLocal.z + dir.z * Math.min(radius, 500),
                },
            });
        }

        const targets = this._buildPreloadTargets(radius, gridSpacing, maxTargets);
        for (let i = 0; i < targets.length; i++) {
            views.push(this._makePreloadView(centerLocal, targets[i], i, lift, viewDistance));
        }
        return views;
    }

    _sampleLoadedCoverage(centerLocal, radius, spacing) {
        const samples = this._buildPreloadTargets(radius, spacing, 80);
        let loaded = 0;
        const missing = [];

        for (const sample of samples) {
            const y = this.sampleHeightAtLocal(centerLocal.x + sample.x, centerLocal.z + sample.z, 1.0);
            if (Number.isFinite(y)) {
                loaded++;
            } else {
                missing.push(sample);
            }
        }

        return {
            loaded,
            total: samples.length,
            ratio: samples.length ? loaded / samples.length : 1,
            missing,
        };
    }

    async preloadLocalArea(centerLocal, options = {}) {
        if (!this.viewer || !this.ready || !centerLocal) return null;
        const Cesium = this.Cesium;
        const camera = this.viewer.camera;
        const saved = {
            position: Cesium.Cartesian3.clone(camera.positionWC),
            direction: Cesium.Cartesian3.clone(camera.directionWC),
            up: Cesium.Cartesian3.clone(camera.upWC),
        };

        const radius = Math.max(60, Number.isFinite(options.radius) ? options.radius : 220);
        const lift = Math.max(80, Number.isFinite(options.lift) ? options.lift : (radius >= 800 ? 260 : 150));
        const gridSpacing = clampNumber(options.gridSpacing, 180, 600, radius >= 800 ? 330 : Math.max(180, radius * 0.75));
        const viewDistance = clampNumber(options.viewDistance, 140, 420, radius >= 800 ? 260 : Math.max(160, radius * 0.75));
        const maxTargets = Math.round(clampNumber(options.maxTargets, 4, 60, radius >= 800 ? 34 : 12));
        const dwellMs = Math.max(80, Number.isFinite(options.dwellMs) ? options.dwellMs : 180);
        const perViewTimeoutMs = Math.max(450, Number.isFinite(options.perViewTimeoutMs) ? options.perViewTimeoutMs : 1600);
        const finalIdleTimeoutMs = Math.max(perViewTimeoutMs, Number.isFinite(options.finalIdleTimeoutMs) ? options.finalIdleTimeoutMs : 5000);
        const verifyCoverage = options.verifyCoverage !== false && radius >= 350;
        const coverageSpacing = clampNumber(options.coverageSpacing, 180, 600, Math.max(240, gridSpacing));
        const minCoverageRatio = clampNumber(options.minCoverageRatio, 0, 1, 0.72);
        const repairPasses = Math.round(clampNumber(options.repairPasses, 0, 3, verifyCoverage ? 1 : 0));
        const repairTargets = Math.round(clampNumber(options.repairTargets, 4, 32, 16));
        const progressCb = typeof options.progressCb === 'function' ? options.progressCb : null;
        const label = radius >= 1000 ? `${(radius / 1000).toFixed(1)} km` : `${Math.round(radius)} m`;
        const delay = (ms) => new Promise(resolve => window.setTimeout(resolve, ms));
        const report = {
            radius,
            views: 0,
            timedOutViews: 0,
            coverage: null,
        };

        const runViews = async (views, passLabel) => {
            for (let i = 0; i < views.length; i++) {
                const v = views[i];
                const status = this.getTileLoadStatus();
                const queue = status.pending !== null || status.processing !== null
                    ? `; queue ${status.pending || 0}/${status.processing || 0}`
                    : '';
                if (progressCb) progressCb(`Preloading ${label} collision tiles ${passLabel} (${i + 1}/${views.length}${queue})...`);

                const eye = { x: v.eye.x, y: v.eye.y, z: v.eye.z };
                const surfaceY = this.sampleHeightAtLocal(eye.x, eye.z, 1.0);
                if (Number.isFinite(surfaceY)) eye.y = Math.max(eye.y, surfaceY + 18);

                const directionLocal = normalize3({
                    x: v.target.x - eye.x,
                    y: v.target.y - eye.y,
                    z: v.target.z - eye.z,
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
                const idle = await this.waitForTilesIdle(perViewTimeoutMs);
                if (!idle) report.timedOutViews++;
                report.views++;
            }
        };

        try {
            const initialViews = this._buildLocalAreaPreloadViews(
                centerLocal,
                radius,
                lift,
                viewDistance,
                gridSpacing,
                maxTargets
            );
            await runViews(initialViews, 'scan');
            await this.waitForTilesIdle(finalIdleTimeoutMs, 350);

            for (let pass = 0; verifyCoverage && pass <= repairPasses; pass++) {
                if (progressCb) progressCb(`Verifying ${label} collision tile coverage...`);
                report.coverage = this._sampleLoadedCoverage(centerLocal, radius, coverageSpacing);
                const pct = Math.round(report.coverage.ratio * 100);
                if (progressCb) progressCb(`Collision preload coverage ${report.coverage.loaded}/${report.coverage.total} (${pct}%).`);
                if (report.coverage.ratio >= minCoverageRatio || pass === repairPasses || !report.coverage.missing.length) break;

                const repairViews = report.coverage.missing
                    .slice(0, repairTargets)
                    .map((offset, i) => this._makePreloadView(centerLocal, offset, i + pass * repairTargets, lift, viewDistance));
                await runViews(repairViews, `repair ${pass + 1}`);
                await this.waitForTilesIdle(finalIdleTimeoutMs, 350);
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

        return report;
    }

    destroy() {
        this._destroyPanoramaCaptureViewer();
        if (this.viewer && !this.viewer.isDestroyed()) {
            this.viewer.destroy();
        }
        this.viewer = null;
        this.tileset = null;
        this.ready = false;
        this._heightSampleCache.clear();
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
        this.aircraftModelEntity = this.viewer.entities.add({
            name: 'cesium-drone-model',
            position: new Cesium.CallbackProperty(() => (
                this._aircraftModelPosition || Cesium.Cartesian3.ZERO
            ), false),
            orientation: new Cesium.CallbackProperty(() => (
                this._aircraftModelOrientation || new Cesium.Quaternion(0, 0, 0, 1)
            ), false),
            model: {
                uri: CESIUM_DRONE_MODEL_URI,
                scale: 1.35,
                minimumPixelSize: 44,
                maximumScale: 18,
                runAnimations: true,
                incrementallyLoadTextures: false,
                shadows: Cesium.ShadowMode.DISABLED,
                silhouetteColor: Cesium.Color.fromAlpha(Cesium.Color.CYAN, 0.8),
                silhouetteSize: 1.0,
            },
            show: false,
        });
        this.aircraftEntities.push(this.aircraftModelEntity);
    }

    showAircraft(show) {
        this._ensureAircraft();
        for (const e of this.aircraftEntities) e.show = !!show;
    }

    updateAircraftFromDroneTransform(transform) {
        if (!this.viewer || !transform || !transform.orientation) return;
        this._ensureAircraft();
        const Cesium = this.Cesium;
        this._aircraftModelPosition = this.localToCartesian(transform.position);

        const basis = this.getTransformBasisFixed(transform);
        // Cesium axis-corrects glTF 2.0 models from Y-up/Z-forward into its
        // runtime model frame: +X forward, +Y left, +Z up.
        const xAxis = basis.forward;
        const yAxis = basis.right;
        const zAxis = basis.up;
        const rotation = Cesium.Matrix3.fromColumnMajorArray([
            xAxis.x, xAxis.y, xAxis.z,
            yAxis.x, yAxis.y, yAxis.z,
            zAxis.x, zAxis.y, zAxis.z,
        ], new Cesium.Matrix3());
        this._aircraftModelOrientation = Cesium.Quaternion.fromRotationMatrix(rotation, new Cesium.Quaternion());
    }

    sampleHeightAtLocal(x, z, width = 0.4) {
        if (!this.viewer || !this.ready) return null;
        const Cesium = this.Cesium;
        const scene = this.viewer.scene;
        if (typeof scene.sampleHeight !== 'function') return null;
        const now = performance.now();
        const grid = Math.max(0.75, width * 1.5);
        const key = `${Math.round(x / grid)}:${Math.round(z / grid)}:${Math.round(width * 10)}`;
        const cached = this._heightSampleCache.get(key);
        if (cached && now - cached.time <= HEIGHT_CACHE_TTL_MS) {
            return cached.value;
        }

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
        if (!Number.isFinite(sampledHeight)) {
            this._rememberHeightSample(key, null, now);
            return null;
        }

        const surfaceCartesian = Cesium.Cartesian3.fromRadians(
            carto.longitude,
            carto.latitude,
            sampledHeight
        );
        const localY = this.cartesianToLocal(surfaceCartesian).y;
        this._rememberHeightSample(key, localY, now);
        return localY;
    }

    _rememberHeightSample(key, value, time) {
        this._heightSampleCache.set(key, { value, time });
        if (this._heightSampleCache.size <= HEIGHT_CACHE_LIMIT) return;
        const firstKey = this._heightSampleCache.keys().next().value;
        if (firstKey !== undefined) this._heightSampleCache.delete(firstKey);
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

        const basis = this.getTransformBasisFixed(transform);

        const destination = this.localToCartesian(transform.position);
        const direction = basis.forward;
        const up = basis.up;

        this.viewer.camera.setView({
            destination,
            orientation: { direction, up },
        });
    }

    getTransformBasisFixed(transform) {
        const basis = getTransformBasisLocal(transform);
        return {
            right: this.localDirectionToFixed(basis.right),
            left: this.localDirectionToFixed(basis.left),
            up: this.localDirectionToFixed(basis.up),
            down: this.localDirectionToFixed(basis.down),
            back: this.localDirectionToFixed(basis.back),
            forward: this.localDirectionToFixed(basis.forward),
        };
    }

    getForwardLocal(transform) {
        if (!transform || !transform.orientation) return { x: 0, y: 0, z: -1 };
        return getTransformBasisLocal(transform).forward;
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

    _componentDirectionToFixed(basis, component) {
        const Cesium = this.Cesium;
        const out = new Cesium.Cartesian3();
        const tmp = new Cesium.Cartesian3();

        Cesium.Cartesian3.multiplyByScalar(basis.right, component.x, out);
        Cesium.Cartesian3.multiplyByScalar(basis.up, component.y, tmp);
        Cesium.Cartesian3.add(out, tmp, out);
        Cesium.Cartesian3.multiplyByScalar(basis.back, component.z, tmp);
        Cesium.Cartesian3.add(out, tmp, out);
        return Cesium.Cartesian3.normalize(out, out);
    }

    _renderViewerNow(viewer = this.viewer) {
        if (!viewer || !viewer.scene) return;
        try {
            if (typeof viewer.render === 'function') {
                viewer.render();
                return;
            }
        } catch (_) {}
        try {
            if (typeof viewer.scene.render === 'function') {
                viewer.scene.render(viewer.clock ? viewer.clock.currentTime : undefined);
            }
        } catch (_) {
            viewer.scene.requestRender();
        }
    }

    _renderNow() {
        this._renderViewerNow(this.viewer);
    }

    _getPanoramaProjector() {
        if (this._panoramaProjector === false) return null;
        if (this._panoramaProjector) return this._panoramaProjector;
        try {
            this._panoramaProjector = new PanoramaEquirectProjector();
            return this._panoramaProjector;
        } catch (error) {
            console.warn('[CesiumWorld] GPU panorama projection unavailable; falling back to CPU stitch:', error);
            this._panoramaProjector = false;
            return null;
        }
    }

    _captureCanvasToFace(source, faceCanvas) {
        if (!source || !source.width || !source.height) {
            throw new Error('Cesium canvas is not ready for panorama capture.');
        }
        const faceSize = faceCanvas.width;
        const side = Math.max(1, Math.min(source.width, source.height));
        const sx = Math.max(0, (source.width - side) * 0.5);
        const sy = Math.max(0, (source.height - side) * 0.5);
        const ctx = faceCanvas.getContext('2d', { willReadFrequently: true });
        ctx.clearRect(0, 0, faceSize, faceSize);
        ctx.drawImage(source, sx, sy, side, side, 0, 0, faceSize, faceSize);
        return ctx.getImageData(0, 0, faceSize, faceSize);
    }

    _captureCurrentCanvasToFace(faceCanvas) {
        return this._captureCanvasToFace(this.viewer?.scene?.canvas, faceCanvas);
    }

    _stitchPanoramaFaces(faceData, width, height, faceSize, verticalFovDeg = 180) {
        const canvas = document.createElement('canvas');
        canvas.width = width;
        canvas.height = height;
        const ctx = canvas.getContext('2d', { willReadFrequently: true });
        const out = ctx.createImageData(width, height);
        const outData = out.data;
        const verticalFov = Math.max(1, Math.min(180, verticalFovDeg || 180)) * Math.PI / 180;

        for (let py = 0; py < height; py++) {
            const pitch = 0.5 * verticalFov - ((py + 0.5) / height) * verticalFov;
            const cosPitch = Math.cos(pitch);
            const sinPitch = Math.sin(pitch);

            for (let px = 0; px < width; px++) {
                const yaw = Math.PI - ((px + 0.5) / width) * Math.PI * 2;
                const forward = cosPitch * Math.cos(yaw);
                const left = cosPitch * Math.sin(yaw);
                const up = sinPitch;
                const dirX = -left;
                const dirY = up;
                const dirZ = -forward;
                const face = faceData.get(choosePanoramaFaceComponents(dirX, dirY, dirZ));
                const denom = Math.max(1e-6, dirX * face.dir.x + dirY * face.dir.y + dirZ * face.dir.z);
                const u = (dirX * face.right.x + dirY * face.right.y + dirZ * face.right.z) / denom;
                const v = (dirX * face.up.x + dirY * face.up.y + dirZ * face.up.z) / denom;
                const sx = (u * 0.5 + 0.5) * (faceSize - 1);
                const sy = (0.5 - v * 0.5) * (faceSize - 1);
                const dstIdx = (py * width + px) * 4;
                writeBilinearPixel(face.image, sx, sy, outData, dstIdx);
            }
        }

        ctx.putImageData(out, 0, 0);
        return canvas;
    }

    _configurePanoramaTileset(tileset) {
        if (!tileset) return;

        const setIfPresent = (key, value) => {
            if (key in tileset) tileset[key] = value;
        };

        setIfPresent('maximumScreenSpaceError', this.panoramaTileSSE);
        setIfPresent('cullRequestsWhileMoving', false);
        setIfPresent('preloadWhenHidden', true);
        setIfPresent('preloadFlightDestinations', true);
        setIfPresent('foveatedScreenSpaceError', false);
        setIfPresent('dynamicScreenSpaceError', true);
        setIfPresent('dynamicScreenSpaceErrorDensity', 0.004);
        setIfPresent('dynamicScreenSpaceErrorFactor', 12);
        setIfPresent('loadSiblings', false);
        setIfPresent('skipLevelOfDetail', true);
        setIfPresent('baseScreenSpaceError', 1024);
        setIfPresent('skipScreenSpaceErrorFactor', 12);
        setIfPresent('skipLevels', 1);
        setIfPresent('immediatelyLoadDesiredLevelOfDetail', false);
        setIfPresent('preferLeaves', false);

        if ('maximumMemoryUsage' in tileset) tileset.maximumMemoryUsage = 768;
        if ('cacheBytes' in tileset) tileset.cacheBytes = 768 * 1024 * 1024;
        if ('maximumCacheOverflowBytes' in tileset) tileset.maximumCacheOverflowBytes = 256 * 1024 * 1024;
    }

    _destroyPanoramaCaptureViewer() {
        if (this._panoramaViewer && !this._panoramaViewer.isDestroyed()) {
            this._panoramaViewer.destroy();
        }
        if (this._panoramaContainer && this._panoramaContainer.parentNode) {
            this._panoramaContainer.parentNode.removeChild(this._panoramaContainer);
        }
        this._panoramaViewer = null;
        this._panoramaTileset = null;
        this._panoramaContainer = null;
        this._panoramaInitPromise = null;
        this._panoramaFaceSize = 0;
        this._panoramaTileLoadPending = null;
        this._panoramaTileLoadProcessing = null;
        this._panoramaScanState = null;
    }

    async _createPanoramaCaptureViewer(faceSize) {
        const Cesium = this.Cesium || requireCesium();
        this._destroyPanoramaCaptureViewer();

        const container = document.createElement('div');
        container.className = 'cesium-panorama-capture';
        Object.assign(container.style, {
            position: 'fixed',
            left: '0',
            top: '0',
            width: `${faceSize}px`,
            height: `${faceSize}px`,
            overflow: 'hidden',
            pointerEvents: 'none',
            opacity: '0.001',
            zIndex: '0',
        });
        document.body.appendChild(container);

        const viewer = new Cesium.Viewer(container, {
            animation: false,
            timeline: false,
            baseLayerPicker: false,
            geocoder: false,
            homeButton: false,
            infoBox: false,
            navigationHelpButton: false,
            sceneModePicker: false,
            selectionIndicator: false,
            fullscreenButton: false,
            scene3DOnly: true,
            shouldAnimate: false,
            globe: false,
            skyAtmosphere: new Cesium.SkyAtmosphere(),
            requestRenderMode: true,
            useDefaultRenderLoop: false,
            useBrowserRecommendedResolution: false,
            orderIndependentTranslucency: false,
            contextOptions: {
                webgl: {
                    alpha: false,
                    antialias: false,
                    preserveDrawingBuffer: true,
                    powerPreference: 'high-performance',
                    failIfMajorPerformanceCaveat: false,
                },
            },
        });

        viewer.scene.fog.enabled = false;
        viewer.scene.highDynamicRange = false;
        if ('resolutionScale' in viewer) viewer.resolutionScale = 1;
        if ('msaaSamples' in viewer.scene) viewer.scene.msaaSamples = 1;
        if (viewer.scene.postProcessStages && viewer.scene.postProcessStages.fxaa) {
            viewer.scene.postProcessStages.fxaa.enabled = false;
        }

        const tileset = await this._createGoogleTileset(null);
        this._configurePanoramaTileset(tileset);
        if (tileset.loadProgress && typeof tileset.loadProgress.addEventListener === 'function') {
            tileset.loadProgress.addEventListener((pending, processing) => {
                this._panoramaTileLoadPending = Math.max(0, Number(pending) || 0);
                this._panoramaTileLoadProcessing = Math.max(0, Number(processing) || 0);
            });
        }
        viewer.scene.primitives.add(tileset);
        viewer.resize();

        this._panoramaViewer = viewer;
        this._panoramaTileset = tileset;
        this._panoramaContainer = container;
        this._panoramaFaceSize = faceSize;
        return viewer;
    }

    async _ensurePanoramaCaptureViewer(faceSize) {
        if (
            this._panoramaViewer &&
            !this._panoramaViewer.isDestroyed() &&
            this._panoramaFaceSize === faceSize
        ) {
            return this._panoramaViewer;
        }

        if (!this._panoramaInitPromise) {
            this._panoramaInitPromise = this._createPanoramaCaptureViewer(faceSize)
                .finally(() => {
                    this._panoramaInitPromise = null;
                });
        }

        return this._panoramaInitPromise;
    }

    async _waitForPanoramaTilesIdle(viewer, timeoutMs = 2200, quietMs = 220) {
        if (!this._panoramaTileset) return true;
        const started = performance.now();
        let idleSince = null;
        const sleep = (ms) => new Promise(resolve => window.setTimeout(resolve, ms));

        while (performance.now() - started < timeoutMs) {
            const queueKnown = this._panoramaTileLoadPending !== null || this._panoramaTileLoadProcessing !== null;
            const queueIdle = !queueKnown ||
                ((this._panoramaTileLoadPending || 0) <= 0 && (this._panoramaTileLoadProcessing || 0) <= 0);
            const loaded = this._panoramaTileset.tilesLoaded === true && queueIdle;

            if (loaded) {
                if (idleSince == null) idleSince = performance.now();
                if (performance.now() - idleSince >= quietMs) return true;
            } else {
                idleSince = null;
            }

            viewer.scene.requestRender();
            this._renderViewerNow(viewer);
            await sleep(80);
        }

        return false;
    }

    _panoramaTilesStable(scanState, quietMs = 180, minSettleMs = 80, faceKey = '') {
        if (!this._panoramaTileset) return true;
        const now = performance.now();
        if (scanState.tileFaceKey !== faceKey) {
            scanState.tileFaceKey = faceKey;
            scanState.tileFaceSince = now;
            scanState.tileIdleSince = null;
        }
        const faceSince = Number.isFinite(scanState.tileFaceSince) ? scanState.tileFaceSince : now;
        if (now - faceSince < minSettleMs) {
            scanState.tileIdleSince = null;
            return false;
        }

        const queueKnown = this._panoramaTileLoadPending !== null || this._panoramaTileLoadProcessing !== null;
        const queueIdle = !queueKnown ||
            ((this._panoramaTileLoadPending || 0) <= 0 && (this._panoramaTileLoadProcessing || 0) <= 0);
        const loaded = this._panoramaTileset.tilesLoaded === true && queueIdle;

        if (!loaded) {
            scanState.tileIdleSince = null;
            return false;
        }
        if (scanState.tileIdleSince == null) {
            scanState.tileIdleSince = now;
            return quietMs <= 0;
        }
        return now - scanState.tileIdleSince >= quietMs;
    }

    async _settlePanoramaCaptureFace(viewer, maxMs = 5200) {
        const started = performance.now();
        const sleep = (ms) => new Promise(resolve => window.setTimeout(resolve, ms));

        while (performance.now() - started < maxMs) {
            viewer.scene.requestRender();
            this._renderViewerNow(viewer);
            await this._waitForPanoramaTilesIdle(viewer, Math.min(1200, Math.max(200, maxMs - (performance.now() - started))), 180);

            const remaining = maxMs - (performance.now() - started);
            if (remaining <= 0) break;
            await sleep(Math.min(120, remaining));
            if (this._panoramaTileset && this._panoramaTileset.tilesLoaded === true) break;
        }

        this._renderViewerNow(viewer);
    }

    async _capturePanoramaFaceImage(viewer, faceCanvas, faceName) {
        const started = performance.now();
        let image = null;

        while (performance.now() - started < 5200) {
            await this._settlePanoramaCaptureFace(viewer, 1400);
            image = this._captureCanvasToFace(viewer.scene.canvas, faceCanvas);
            if (faceName === 'up' || imageBlackRatio(image) < 0.45) return image;
            await new Promise(resolve => window.setTimeout(resolve, 180));
        }

        return image || this._captureCanvasToFace(viewer.scene.canvas, faceCanvas);
    }

    async _capturePanoramaWithViewerAsync(viewer, transform, width, height, faceSize, verticalFovDeg = 180, options = {}) {
        const camera = viewer.camera;
        const frustum = camera.frustum;
        const saved = {
            fov: frustum && 'fov' in frustum ? frustum.fov : undefined,
            near: frustum && 'near' in frustum ? frustum.near : undefined,
            far: frustum && 'far' in frustum ? frustum.far : undefined,
        };
        const faceData = new Map();
        const basis = this.getTransformBasisFixed(transform);
        const destination = this.localToCartesian(transform.position);
        const projector = options.useGpuProjector === false ? null : this._getPanoramaProjector();
        const settleMs = Math.max(0, Number.isFinite(options.settleMs) ? options.settleMs : this.panoramaSettleMs);
        const requestedProjectionMode = (
            options.projectionMode === 'cube' || options.projectionMode === 'side'
        ) ? options.projectionMode : 'hybrid';
        const projectionMode = projector ? requestedProjectionMode : 'cube';
        const faceFovDeg = projectionMode === 'cube'
            ? 90
            : Math.max(90, Math.min(170, Number(options.faceFovDeg) || 130));
        const faceDefs = projectionMode === 'side' ? PANORAMA_SIDE_FACE_DEFS : PANORAMA_FACE_DEFS;

        try {
            if (frustum) {
                if ('fov' in frustum) frustum.fov = faceFovDeg * Math.PI / 180;
                if ('near' in frustum) frustum.near = 0.03;
                if ('far' in frustum) frustum.far = 15000000;
            }

            for (const faceDef of faceDefs) {
                camera.setView({
                    destination,
                    orientation: {
                        direction: this._componentDirectionToFixed(basis, faceDef.dir),
                        up: this._componentDirectionToFixed(basis, faceDef.up),
                    },
                });
                viewer.scene.requestRender();
                this._renderViewerNow(viewer);
                if (settleMs > 0) {
                    await this._settlePanoramaCaptureFace(viewer, settleMs);
                }

                if (projector) {
                    projector.updateFace(faceDef.name, viewer.scene.canvas);
                    continue;
                }

                const faceCanvas = document.createElement('canvas');
                faceCanvas.width = faceSize;
                faceCanvas.height = faceSize;
                faceData.set(faceDef.name, {
                    dir: faceDef.dir,
                    up: faceDef.up,
                    right: normalize3(cross3(faceDef.dir, faceDef.up)),
                    image: await this._capturePanoramaFaceImage(viewer, faceCanvas, faceDef.name),
                });
            }

            if (projector) {
                const canvas = projector.render(width, height, verticalFovDeg, projectionMode, faceFovDeg);
                if (canvas) return canvas;
                throw new Error('GPU panorama projection did not receive all required faces.');
            }
            return this._stitchPanoramaFaces(faceData, width, height, faceSize, verticalFovDeg);
        } finally {
            if (frustum) {
                if (saved.fov !== undefined && 'fov' in frustum) frustum.fov = saved.fov;
                if (saved.near !== undefined && 'near' in frustum) frustum.near = saved.near;
                if (saved.far !== undefined && 'far' in frustum) frustum.far = saved.far;
            }
        }
    }

    async capturePanoramaAsync(transform, options = {}) {
        if (!this.viewer || !this.ready || !transform || !transform.position || !transform.orientation) {
            return null;
        }

        const width = Math.max(256, Math.round(options.width || 512));
        const height = Math.max(128, Math.round(options.height || Math.round(width / 2)));
        const faceSize = Math.max(96, Math.round(options.faceSize || 128));
        const verticalFovDeg = Math.max(1, Math.min(180, Number(options.verticalFovDeg) || 180));
        const viewer = await this._ensurePanoramaCaptureViewer(faceSize);
        return this._capturePanoramaWithViewerAsync(viewer, transform, width, height, faceSize, verticalFovDeg, {
            settleMs: Number.isFinite(options.settleMs) ? options.settleMs : this.panoramaSettleMs,
            useGpuProjector: options.useGpuProjector !== false,
            projectionMode: options.projectionMode,
            faceFovDeg: options.faceFovDeg,
        });
    }

    async capturePanoramaIncrementalAsync(transform, options = {}) {
        if (!this.viewer || !this.ready || !transform || !transform.position || !transform.orientation) {
            return { canvas: null, complete: false, ready: false };
        }

        const width = Math.max(256, Math.round(options.width || 512));
        const height = Math.max(128, Math.round(options.height || Math.round(width / 2)));
        const faceSize = Math.max(96, Math.round(options.faceSize || 128));
        const verticalFovDeg = Math.max(1, Math.min(180, Number(options.verticalFovDeg) || 180));
        const viewer = await this._ensurePanoramaCaptureViewer(faceSize);
        const projector = options.useGpuProjector === false ? null : this._getPanoramaProjector();

        if (!projector) {
            const canvas = await this._capturePanoramaWithViewerAsync(
                viewer,
                transform,
                width,
                height,
                faceSize,
                verticalFovDeg,
                {
                    settleMs: Number.isFinite(options.settleMs) ? options.settleMs : this.panoramaSettleMs,
                    useGpuProjector: false,
                    projectionMode: 'cube',
                    faceFovDeg: options.faceFovDeg,
                }
            );
            return { canvas, complete: true, ready: !!canvas, fallback: true };
        }

        const requestedProjectionMode = (
            options.projectionMode === 'cube' || options.projectionMode === 'side'
        ) ? options.projectionMode : 'hybrid';
        const projectionMode = requestedProjectionMode;
        const faceFovDeg = projectionMode === 'cube'
            ? 90
            : Math.max(90, Math.min(170, Number(options.faceFovDeg) || 130));
        const faceDefs = projectionMode === 'side' ? PANORAMA_SIDE_FACE_DEFS : PANORAMA_FACE_DEFS;
        const facesPerStep = Math.max(1, Math.min(faceDefs.length, Math.round(Number(options.facesPerStep) || 1)));
        const settleMs = Math.max(0, Number.isFinite(options.settleMs) ? options.settleMs : this.panoramaSettleMs);
        const requireTiles = options.requireTiles !== false;
        const tileQuietMs = Math.max(0, Math.min(1500, Number(options.tileQuietMs) || 180));
        const tileMinSettleMs = Math.max(0, Math.min(1000, Number(options.tileMinSettleMs) || 80));
        const key = `${faceSize}|${projectionMode}|${faceFovDeg}|${verticalFovDeg}`;
        let state = this._panoramaScanState;
        if (!state || state.key !== key) {
            state = {
                key,
                index: 0,
                completedCycles: 0,
                cycleTransform: clonePanoramaTransform(transform),
                tileIdleSince: null,
                tileFaceKey: null,
                tileFaceSince: null,
            };
            this._panoramaScanState = state;
        }
        if (!state.cycleTransform || state.index >= faceDefs.length) {
            state.cycleTransform = clonePanoramaTransform(transform);
            state.index = 0;
            state.tileIdleSince = null;
            state.tileFaceKey = null;
            state.tileFaceSince = null;
        }

        const camera = viewer.camera;
        const frustum = camera.frustum;
        const saved = {
            fov: frustum && 'fov' in frustum ? frustum.fov : undefined,
            near: frustum && 'near' in frustum ? frustum.near : undefined,
            far: frustum && 'far' in frustum ? frustum.far : undefined,
        };
        const basis = this.getTransformBasisFixed(state.cycleTransform);
        const destination = this.localToCartesian(state.cycleTransform.position);
        let complete = false;

        try {
            if (frustum) {
                if ('fov' in frustum) frustum.fov = faceFovDeg * Math.PI / 180;
                if ('near' in frustum) frustum.near = 0.03;
                if ('far' in frustum) frustum.far = 15000000;
            }

            for (let i = 0; i < facesPerStep; i++) {
                const faceDef = faceDefs[state.index];
                camera.setView({
                    destination,
                    orientation: {
                        direction: this._componentDirectionToFixed(basis, faceDef.dir),
                        up: this._componentDirectionToFixed(basis, faceDef.up),
                    },
                });
                viewer.scene.requestRender();
                this._renderViewerNow(viewer);
                if (settleMs > 0) {
                    await this._settlePanoramaCaptureFace(viewer, settleMs);
                }
                const faceKey = `${state.completedCycles}|${state.index}|${faceDef.name}`;
                if (requireTiles && !this._panoramaTilesStable(state, tileQuietMs, tileMinSettleMs, faceKey)) {
                    return {
                        canvas: null,
                        complete: false,
                        ready: state.completedCycles > 0,
                        loadingTiles: true,
                        faceIndex: state.index,
                        faces: faceDefs.length,
                        pending: this._panoramaTileLoadPending,
                        processing: this._panoramaTileLoadProcessing,
                    };
                }
                projector.updateFace(faceDef.name, viewer.scene.canvas);
                state.tileIdleSince = null;

                state.index++;
                if (state.index >= faceDefs.length) {
                    state.index = 0;
                    state.completedCycles++;
                    state.cycleTransform = null;
                    state.tileIdleSince = null;
                    state.tileFaceKey = null;
                    state.tileFaceSince = null;
                    complete = true;
                    break;
                }
            }
        } finally {
            if (frustum) {
                if (saved.fov !== undefined && 'fov' in frustum) frustum.fov = saved.fov;
                if (saved.near !== undefined && 'near' in frustum) frustum.near = saved.near;
                if (saved.far !== undefined && 'far' in frustum) frustum.far = saved.far;
            }
        }

        if (!complete) {
            return {
                canvas: null,
                complete: false,
                ready: state.completedCycles > 0,
                faceIndex: state.index,
                faces: faceDefs.length,
            };
        }

        const canvas = projector.render(width, height, verticalFovDeg, projectionMode, faceFovDeg);
        if (!canvas) {
            return {
                canvas: null,
                complete: false,
                ready: state.completedCycles > 0,
                renderPending: true,
                cycle: state.completedCycles,
                faces: faceDefs.length,
            };
        }
        return {
            canvas,
            complete: true,
            ready: !!canvas,
            cycle: state.completedCycles,
            faces: faceDefs.length,
        };
    }

    capturePanorama(transform, options = {}) {
        if (!this.viewer || !this.ready || !transform || !transform.position || !transform.orientation) {
            return null;
        }

        const Cesium = this.Cesium;
        const width = Math.max(256, Math.round(options.width || 768));
        const height = Math.max(128, Math.round(options.height || Math.round(width / 2)));
        const faceSize = Math.max(96, Math.round(options.faceSize || 256));
        const verticalFovDeg = Math.max(1, Math.min(180, Number(options.verticalFovDeg) || 180));
        const camera = this.viewer.camera;
        const frustum = camera.frustum;
        const saved = {
            position: Cesium.Cartesian3.clone(camera.positionWC),
            direction: Cesium.Cartesian3.clone(camera.directionWC),
            up: Cesium.Cartesian3.clone(camera.upWC),
            fov: frustum && 'fov' in frustum ? frustum.fov : undefined,
            near: frustum && 'near' in frustum ? frustum.near : undefined,
            far: frustum && 'far' in frustum ? frustum.far : undefined,
            aircraftShows: this.aircraftEntities.map(entity => entity.show),
        };
        const faceData = new Map();
        const basis = this.getTransformBasisFixed(transform);
        const destination = this.localToCartesian(transform.position);

        try {
            for (const entity of this.aircraftEntities) entity.show = false;
            if (frustum) {
                if ('fov' in frustum) frustum.fov = Math.PI / 2;
                if ('near' in frustum) frustum.near = 0.03;
                if ('far' in frustum) frustum.far = 15000000;
            }

            for (const faceDef of PANORAMA_FACE_DEFS) {
                camera.setView({
                    destination,
                    orientation: {
                        direction: this._componentDirectionToFixed(basis, faceDef.dir),
                        up: this._componentDirectionToFixed(basis, faceDef.up),
                    },
                });
                this.viewer.scene.requestRender();
                this._renderNow();

                const faceCanvas = document.createElement('canvas');
                faceCanvas.width = faceSize;
                faceCanvas.height = faceSize;
                faceData.set(faceDef.name, {
                    dir: faceDef.dir,
                    up: faceDef.up,
                    right: normalize3(cross3(faceDef.dir, faceDef.up)),
                    image: this._captureCurrentCanvasToFace(faceCanvas),
                });
            }

            return this._stitchPanoramaFaces(faceData, width, height, faceSize, verticalFovDeg);
        } finally {
            camera.setView({
                destination: saved.position,
                orientation: {
                    direction: saved.direction,
                    up: saved.up,
                },
            });
            if (frustum) {
                if (saved.fov !== undefined && 'fov' in frustum) frustum.fov = saved.fov;
                if (saved.near !== undefined && 'near' in frustum) frustum.near = saved.near;
                if (saved.far !== undefined && 'far' in frustum) frustum.far = saved.far;
            }
            this.aircraftEntities.forEach((entity, i) => {
                entity.show = !!saved.aircraftShows[i];
            });
            this.viewer.scene.requestRender();
        }
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
