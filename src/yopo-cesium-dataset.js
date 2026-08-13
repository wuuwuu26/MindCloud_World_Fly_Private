/**
 * Cesium → YOPO_360 训练数据集采集器
 * ==================================
 * 在 Cesium / Google 3D Tiles 真实世界里采集训练样本, 落盘格式与合成 Simulator
 * (YOPO_360/Simulator/src/src/dataset_generator.cpp) 完全一致, 因此 train_yopo.py
 * 无需任何改动即可直接训练 (其 SafetyLoss 会从 pointcloud-*.ply 自动构建 ESDF)。
 *
 * 输出的目录结构 (由 cesium_dataset_server.py 写入):
 *   dataset/<map_i>/img_<n>.png        uint16, 值 = depth / maxDepth * 65535
 *   dataset/<map_i>/img_<n>_m.png      uint8 掩码, 255 = 有效
 *   dataset/pose-<map_i>.csv           表头 yaw,pitch,roll,px,py,pz
 *   dataset/pointcloud-<map_i>.ply     Cesium 障碍几何点云 (ESDF 来源)
 *   dataset/max_depth.txt              maxDepth (文本)
 *
 * 设计要点 (呼应"不保留真实深度方法"):
 *   - 运行时绝不把 Cesium 射线真值当深度喂给网络。
 *   - 训练用的"真实几何"通过把【同一份 DA360 ERP 深度】(运行时喂给网络的那份,
 *     已用稀疏 Cesium 射线标定过尺度) 反投影成点云得到, 再交给 SafetyLoss 建 ESDF。
 *     即: 输入分布 = 运行时分布(DA360), 标签几何 = Cesium 真值, sim2real 鸿沟在
 *     训练期被消除, 而非在运行时 hack。
 *   - 另提供 mode='raycast' 作为离线快速通道 (直接用 Cesium 射线生成 ERP 真值深度),
 *     仅用于造数据, 不影响运行时。
 *
 * 注意: SafetyLoss 的 ESDF 体素尺寸为 0.2m, 因此每个 map 的区域应保持较小
 * (radius 建议 <= 15m), 否则 ESDF 张量会过大。
 */

const CDS_DEFAULTS = {
    numMaps: 4,
    samplesPerMap: 300,
    radius: 12,            // 米, 采样圆盘半径 (建议 <= 15)
    heightJitter: 0.5,     // 米, 高度抖动
    maxDepth: 20,          // 米, 深度归一化上限
    width: 384,
    height: 192,
    mode: 'da360',         // 'da360' | 'raycast'
    rayW: 96,              // raycast 模式下的粗网格宽
    rayH: 48,              // raycast 模式下的粗网格高
    stride: 4,             // 点云反投影的像素步长
    maxPtsPerMap: 200000,  // 每个 map 点云上限
    serverUrl: 'http://localhost:8003',
    hfov: 100,             // 传给 setCameraFromDroneTransform 的视场角
};

function _axisAngle(ax, ay, az, angle) {
    const h = angle * 0.5;
    const s = Math.sin(h);
    return { x: ax * s, y: ay * s, z: az * s, w: Math.cos(h) };
}

function _quatMul(a, b) {
    return {
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

function _quatFromYawPitchRoll(yaw, pitch, roll) {
    const qy = _axisAngle(0, 1, 0, yaw);
    const qp = _axisAngle(1, 0, 0, pitch);
    const qr = _axisAngle(0, 0, 1, roll);
    return _quatMul(_quatMul(qy, qp), qr);
}

function _f32ToB64(arr) {
    const u8 = new Uint8Array(arr.buffer, arr.byteOffset, arr.byteLength);
    let s = '';
    for (let i = 0; i < u8.length; i++) s += String.fromCharCode(u8[i]);
    return btoa(s);
}

function _u8ToB64(arr) {
    let s = '';
    for (let i = 0; i < arr.length; i++) s += String.fromCharCode(arr[i]);
    return btoa(s);
}

function _waitFrames(n) {
    return new Promise((resolve) => {
        const step = (k) => {
            if (k <= 0) resolve();
            else requestAnimationFrame(() => step(k - 1));
        };
        step(n);
    });
}

// ERP 像素(u,v) -> 世界方向 (local ENU, 与已删除的 captureYOPODepthTrueERP /
// _samplePanoramaDepth 逆映射一致; DA360 经翻转后亦遵循此约定)
function _erpDir(u, v) {
    const yaw = Math.PI - 2 * Math.PI * u;
    const pitch = Math.PI * (0.5 - v);
    const cp = Math.cos(pitch);
    const sp = Math.sin(pitch);
    return { x: Math.sin(yaw) * cp, y: sp, z: -Math.cos(yaw) * cp };
}

class CesiumYOPODataset {
    constructor(world, drone, yopoDepthFromPanorama, opts = {}) {
        this.world = world;
        this.drone = drone;
        this.yopoDepth = yopoDepthFromPanorama;
        this.opts = Object.assign({}, CDS_DEFAULTS, opts);
        this.regionCenter = null;   // {x,y,z} local ENU
        this._stop = false;
        this.onProgress = null;      // (info) => void
    }

    setRegionFromDrone() {
        const t = this.drone.getCameraTransform();
        this.regionCenter = { x: t.position.x, y: t.position.y, z: t.position.z };
        return this.regionCenter;
    }

    setRegion(x, y, z) {
        this.regionCenter = { x, y, z };
    }

    _mapCenter(mapIdx) {
        const c = this.regionCenter || { x: 0, y: 0, z: 0 };
        const spacing = this.opts.radius * 2.5;
        return { x: c.x + (mapIdx + 1) * spacing, y: c.y, z: c.z };
    }

    _samplePose(center) {
        const o = this.opts;
        const ang = Math.random() * 2 * Math.PI;
        const r = o.radius * Math.sqrt(Math.random());
        const pos = {
            x: center.x + Math.cos(ang) * r,
            y: center.y + (Math.random() - 0.5) * 2 * o.heightJitter,
            z: center.z + Math.sin(ang) * r,
        };
        const yaw = Math.random() * 2 * Math.PI;
        const pitch = (Math.random() - 0.5) * 0.4;
        const roll = (Math.random() - 0.5) * 0.4;
        const orientation = _quatFromYawPitchRoll(yaw, pitch, roll);
        return { pos, yaw, pitch, roll, orientation };
    }

    async _captureERP(cameraTransform) {
        const o = this.opts;
        if (o.mode === 'raycast') {
            return this._captureRaycastERP(cameraTransform);
        }
        const res = await this.yopoDepth.captureYOPODepthERP(cameraTransform, {
            width: o.width,
            height: o.height,
            maxDistance: o.maxDepth,
            timeoutMs: 6000,
        });
        if (!res) return null;
        return { depth: res.depth, mask: res.mask };
    }

    // 纯 Cesium 射线生成 ERP 深度 (离线快速通道, 不经 DA360)
    async _captureRaycastERP(cameraTransform) {
        const o = this.opts;
        const W = o.rayW, H = o.rayH;
        const depth = new Float32Array(W * H);
        const mask = new Uint8Array(W * H);
        const pos = cameraTransform.position;
        if (!this.world || typeof this.world.pickLocalRay !== 'function') return null;
        for (let py = 0; py < H; py++) {
            const v = (py + 0.5) / H;
            for (let px = 0; px < W; px++) {
                const u = (px + 0.5) / W;
                const dir = _erpDir(u, v);
                const hit = this.world.pickLocalRay(pos, dir, o.maxDepth);
                const idx = py * W + px;
                if (hit && hit.distance > 0.2 && hit.distance < o.maxDepth) {
                    depth[idx] = hit.distance;
                    mask[idx] = 255;
                } else {
                    depth[idx] = o.maxDepth;
                    mask[idx] = 0;
                }
            }
        }
        // 上采样到训练分辨率
        const outDepth = this._resizeBilinear(depth, W, H, o.width, o.height);
        const outMask = this._resizeNearest(mask, W, H, o.width, o.height);
        return { depth: outDepth, mask: outMask };
    }

    _resizeBilinear(src, sw, sh, dw, dh) {
        const out = new Float32Array(dw * dh);
        for (let y = 0; y < dh; y++) {
            const sy = (y + 0.5) / dh * sh - 0.5;
            const y0 = Math.max(0, Math.min(sh - 1, Math.floor(sy)));
            const y1 = Math.min(sh - 1, y0 + 1);
            const fy = sy - y0;
            for (let x = 0; x < dw; x++) {
                const sx = (x + 0.5) / dw * sw - 0.5;
                const x0 = Math.max(0, Math.min(sw - 1, Math.floor(sx)));
                const x1 = Math.min(sw - 1, x0 + 1);
                const fx = sx - x0;
                const i00 = y0 * sw + x0, i10 = y0 * sw + x1;
                const i01 = y1 * sw + x0, i11 = y1 * sw + x1;
                const top = src[i00] + (src[i10] - src[i00]) * fx;
                const bot = src[i01] + (src[i11] - src[i01]) * fx;
                out[y * dw + x] = top + (bot - top) * fy;
            }
        }
        return out;
    }

    _resizeNearest(src, sw, sh, dw, dh) {
        const out = new Uint8Array(dw * dh);
        for (let y = 0; y < dh; y++) {
            const sy = Math.min(sh - 1, Math.floor((y + 0.5) / dh * sh));
            for (let x = 0; x < dw; x++) {
                const sx = Math.min(sw - 1, Math.floor((x + 0.5) / dw * sw));
                out[y * dw + x] = src[sy * sw + sx];
            }
        }
        return out;
    }

    // 把 DA360 ERP 深度反投影为 local ENU 点云 (与 pose 同一坐标系)
    _accumulatePoints(pts, depth, mask, pos, maxDepth) {
        const W = this.opts.width, H = this.opts.height, stride = this.opts.stride;
        for (let py = 0; py < H; py += stride) {
            const v = (py + 0.5) / H;
            for (let px = 0; px < W; px += stride) {
                const idx = py * W + px;
                const d = depth[idx];
                if (mask[idx] < 128 || !Number.isFinite(d) || d < 0.3 || d >= maxDepth) continue;
                const dir = _erpDir((px + 0.5) / W, v);
                pts.push(pos.x + dir.x * d, pos.y + dir.y * d, pos.z + dir.z * d);
                if (pts.length / 3 >= this.opts.maxPtsPerMap) return;
            }
        }
    }

    async _postSample(mapId, index, depth, mask, pose) {
        const payload = {
            map_id: mapId,
            index,
            max_depth: this.opts.maxDepth,
            width: this.opts.width,
            height: this.opts.height,
            depth_b64: _f32ToB64(depth),
            mask_b64: _u8ToB64(mask),
            // 训练 yopo_dataset.py 读取 7 列: px,py,pz,qw,qx,qy,qz (skiprows=1)
            pose: {
                px: pose.pos.x, py: pose.pos.y, pz: pose.pos.z,
                qw: pose.orientation.w, qx: pose.orientation.x,
                qy: pose.orientation.y, qz: pose.orientation.z,
            },
        };
        const resp = await fetch(`${this.opts.serverUrl}/dataset/sample`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload),
        });
        if (!resp.ok) throw new Error(`dataset server /sample HTTP ${resp.status}`);
    }

    async _postPointCloud(mapId, pts) {
        const arr = [];
        for (let i = 0; i < pts.length; i += 3) arr.push([pts[i], pts[i + 1], pts[i + 2]]);
        const resp = await fetch(`${this.opts.serverUrl}/dataset/pointcloud`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ map_id: mapId, points: arr }),
        });
        if (!resp.ok) throw new Error(`dataset server /pointcloud HTTP ${resp.status}`);
    }

    async _begin() {
        const resp = await fetch(`${this.opts.serverUrl}/dataset/begin`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                max_depth: this.opts.maxDepth,
                num_maps: this.opts.numMaps,
                clear: true,
            }),
        });
        if (!resp.ok) throw new Error(`dataset server /begin HTTP ${resp.status}`);
    }

    async collect() {
        if (!this.regionCenter) this.setRegionFromDrone();
        this._stop = false;
        await this._begin();

        const o = this.opts;
        for (let m = 0; m < o.numMaps; m++) {
            if (this._stop) break;
            const center = this._mapCenter(m);
            const pts = [];
            for (let i = 0; i < o.samplesPerMap; i++) {
                if (this._stop) break;
                const pose = this._samplePose(center);
                // 把 Cesium 相机移到该位姿并渲染几帧
                const cameraTransform = { position: pose.pos, orientation: pose.orientation };
                this.drone.x = pose.pos.x; this.drone.y = pose.pos.y; this.drone.z = pose.pos.z;
                this.drone.orientation = pose.orientation;
                if (this.world.setCameraFromDroneTransform) {
                    this.world.setCameraFromDroneTransform(cameraTransform, o.hfov);
                }
                await _waitFrames(3);

                const erp = await this._captureERP(cameraTransform);
                if (!erp) {
                    if (this.onProgress) this.onProgress({ map: m, sample: i, skipped: true });
                    continue;
                }
                this._accumulatePoints(pts, erp.depth, erp.mask, pose.pos, o.maxDepth);
                await this._postSample(m, i, erp.depth, erp.mask, pose);

                if (this.onProgress) {
                    this.onProgress({ map: m, sample: i, total: o.samplesPerMap,
                                      pts: pts.length / 3, mode: o.mode });
                }
            }
            await this._postPointCloud(m, pts);
        }
        return { done: !this._stop, center: this.regionCenter };
    }

    stop() {
        this._stop = true;
    }
}

if (typeof window !== 'undefined') window.CesiumYOPODataset = CesiumYOPODataset;
export { CesiumYOPODataset };
