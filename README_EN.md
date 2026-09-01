<div align="center">

**🌐 [English](README_EN.md) | [简体中文](README.md)**

</div>

# MindCloud World Fly with YOPO

![YOPO navigation interface](asset/display/screen.png)

> **YOPO navigation interface**: the bottom-right corner shows the nose-mounted 360° ERP panorama
> RGB and the DA360 depth map, the bottom-left corner holds the Target Map top-down minimap (the
> drone's position relative to the goal), and the panel on the right picks the goal, starts/stops
> navigation, and displays navigation status, distance to the goal and the inference count.

A browser-based FPV drone flying through Google Photorealistic 3D Tiles, with YOPO end-to-end
neural-network autonomous navigation (3D obstacle avoidance). Pick a city, place a spawn point,
then fly with the keyboard, a gamepad or an RC transmitter — or set a goal and let YOPO fly there
autonomously. The bottom-right corner shows the nose-mounted 360° ERP panorama RGB and the DA360
depth map.

## Clone the Repository

```bash
git clone https://github.com/wuuwuu26/MindCloud_World_Fly_Private.git
cd MindCloud_World_Fly_Private
```

> Note: the DA360 **source code** is version-controlled with this repository (as far as DA360 is
> concerned, `.gitignore` only excludes the weights directory `third_party/DA360/checkpoints/`; see
> the repository-root `.gitignore` for the full list), so cloning already gives you the source. The
> **weights** (`DA360_large.pth`, ~1.3GB, over GitHub's 100MB limit) are not in the repository —
> download them before running the depth service.

## Quick Start (Bring Up Every Service at Once)

`restart_all.sh` is the recommended way to start the main process, DA360 and YOPO in one shot:

```bash
./restart_all.sh
```

This is equivalent to starting them in order: DA360 depth service → YOPO navigation service →
main flight process (`launch.sh`). Then open `http://127.0.0.1:8080` in a browser, click
**Start Google 3D Tiles Flight**, set a spawn point in placement mode and press `O` to take off —
you can now fly with the keyboard.

> YOPO inference uses TensorRT acceleration by default (the engine `asset/yopo-trt/yopo_trt.pth`
> ships with the repository). `restart_all.sh` sets `YOPO_USE_TRT=1` **unconditionally**, so no extra
> step is needed; if the engine is missing it only prints a WARN and `scripts/start_yopo_api.sh`
> builds the engine with the GPU inside the YOPO container. See "YOPO TensorRT Acceleration".

```bash
# Restart only part of the services (keep the rest; all three run detached in the background by default)
./restart_all.sh --no-da360        # restart YOPO + main flight only
./restart_all.sh --no-yopo         # restart DA360 + main flight only
./restart_all.sh --no-main         # restart DA360 + YOPO only

# Follow one service's log (container names are in the table below)
docker logs -f mindcloud-yopo-api

# Stop all background containers (same stop logic as restart_all.sh, -v cleans anonymous volumes)
docker rm -fv google-tiles-flight mindcloud-da360-api mindcloud-yopo-api
```

The three container names are defined by `MAIN_NAME` / `DA360_NAME` / `YOPO_NAME` near the top of
[restart_all.sh](restart_all.sh) (matching the defaults of each entry script):

| Container | Purpose | Defined in |
|-----------|---------|------------|
| `google-tiles-flight` | Main flight process (`http://127.0.0.1:8080`) | `launch.sh`, `NAME="${NAME:-google-tiles-flight}"` |
| `mindcloud-da360-api` | DA360 depth service (`http://127.0.0.1:5688`) | `scripts/start_da360_api.sh`, `DA360_CONTAINER_NAME` |
| `mindcloud-yopo-api` | YOPO avoidance backend (`http://127.0.0.1:5689`) | `scripts/start_yopo_api.sh`, `YOPO_CONTAINER_NAME` |

The container names in `restart_all.sh` are **hard-coded assignments** (`DA360_NAME=` / `YOPO_NAME=` /
`MAIN_NAME=`), which **do not accept environment-variable overrides** — `DA360_CONTAINER_NAME=my-da360
./restart_all.sh` has no effect. To rename them, edit those three variables; or call the individual
entry scripts, which do honour `DA360_CONTAINER_NAME` / `YOPO_CONTAINER_NAME` / `NAME`.

If you only want to fly first (pure keyboard/gamepad/RC, no sub-services needed), you can also run
the main process alone:

```bash
./launch.sh
```

## Model Weights

| Model | Shipped with the repo | How to obtain |
|-------|----------------------|---------------|
| YOPO navigation weights | **Yes** (committed directly, ≤100MB) | Included on clone, under `third_party/yopo/saved/` (default `YOPO_40/epoch50.pth`) |
| YOPO TensorRT engine | **Yes** (committed directly, ≤100MB) | At `asset/yopo-trt/yopo_trt.pth` (fp16); converted from `epoch50.pth`, rebuild when changing GPU/model — see "YOPO TensorRT Acceleration" |
| DA360 depth weights | No (~1.3GB, over the limit) | Download script: `./scripts/download_da360_model.sh` (Google Drive, needs `gdown`) |

### YOPO Navigation Weights

Committed directly to the repository, so after cloning they sit at
`third_party/yopo/saved/YOPO_40/epoch50.pth`. The default path is set in
`scripts/start_yopo_api.sh` (`YOPO_MODEL_PATH`) and can be overridden with an environment variable:

```bash
YOPO_MODEL_PATH=/abs/path/to/your_yopo.pth ./scripts/start_yopo_api.sh
```

### DA360 Depth Weights

A single file exceeds GitHub's 100MB limit, so it is not part of the repository. Run the download
script (install `gdown` first):

```bash
./scripts/download_da360_model.sh
# The script puts the weights at third_party/DA360/checkpoints/DA360_large.pth
```

## Requirements

- Docker Engine
- A modern browser with WebGL support (to open `http://127.0.0.1:8080` and use the simulator)
- The browser must be able to reach Cesium Ion and Google 3D Tiles
- Python 3 for local development mode
- DA360 depth inference needs an NVIDIA GPU, the NVIDIA Container Toolkit, Python 3 + pip, and
  network access to the model download URL

### Verified Environment

This project has been fully validated on the following machine (DA360 depth + YOPO navigation +
main flight all brought up together):

| Item | Configuration |
|------|---------------|
| GPU | NVIDIA GeForce RTX 4070 Laptop GPU (8 GB VRAM) |
| Driver / CUDA | 595.84 / 13.2 |
| DA360 config | `DA360_large` + `DA360_INPUT_SCALE=0.65` (model input 672×336), ~92% usage on a single 8GB card |
| YOPO config | TensorRT acceleration, `YOPO_VELOCITY=15` |

> On GPUs with less VRAM (6GB or below), lower `DA360_INPUT_SCALE` or `da360UploadScale` to reduce
> usage; on cards with more headroom you can raise them for better depth accuracy.

## Docker Build Notes

The project builds three independent containers, each with its own image name, base image and
rebuild trigger:

| Container | Image | Base image | Dockerfile | Entry script |
|-----------|-------|-----------|------------|--------------|
| Main flight process | `google-tiles-flight` | `tumgis/3dcitydb-web-map:alpine-v2.0.0` (bundles Node + Cesium) | `Dockerfile.cesium` | `launch.sh` |
| YOPO avoidance backend | `mindcloud-yopo` | `pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime` (CUDA) | `Dockerfile.yopo` | `scripts/start_yopo_api.sh` |
| DA360 depth service | `mindcloud-da360` | `pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime` (CUDA) | `Dockerfile.da360` | `scripts/start_da360_api.sh` |

### Main Flight Process (`Dockerfile.cesium`)

- `COPY`s the whole project into `/var/www/google-tiles-flight` inside the container, with
  `CMD node scripts/server.js` starting the Express static server (which also serves the
  `/api/path/*.json` gate-route persistence API), `EXPOSE 8000`.
- Built from `launch.sh`: it only runs `docker build` when the image does **not** exist or when
  `--rebuild` is passed; otherwise the existing image is reused.
- At runtime `src/` and `index.html` are mounted read-only and `asset/gate-paths` read-write, so
  **changing the frontend JS/HTML never requires an image rebuild** — restart the container and
  hard-refresh the browser (Ctrl+F5).

### YOPO Avoidance Backend (`Dockerfile.yopo`)

- System dependencies: `libgl1-mesa-glx`, `libglib2.0-0`, `ca-certificates`; Python dependencies:
  `numpy<2`, `pillow`, `opencv-python-headless`, `scipy`, `flask`, `flask-cors`, `ruamel.yaml`,
  `websockets`, plus the TensorRT-related `tensorrt==8.6.1.post1` and `onnx`.
- `scripts/yopo_server.py` and `third_party/yopo/` (weights included) are copied straight into the
  image.
- Build/run highlights of `scripts/start_yopo_api.sh`:
  - Builds with `--network=host` so the container's `127.0.0.1:7890` can reach the host proxy for
    pip; the host's `HTTP(S)/FTP/ALL/NO_PROXY` are forwarded as build args (disable with
    `YOPO_PIP_NO_PROXY=1`).
  - Rebuild trigger: the image is missing, or `YOPO_FORCE_BUILD=1`; failures are retried
    `YOPO_BUILD_RETRIES` times (default 3).
  - At runtime the model weights, `yopo_server.py` and the `third_party/yopo` source are mounted
    read-only — **changing the Python or the weights needs no image rebuild**, restarting the
    container is enough.
  - Ports 5689 (HTTP) + 5690 (WebSocket); `--gpus all` by default, `YOPO_GPUS=none` runs on CPU.
  - TensorRT inside the image is pinned to `8.6.1` (compatible with CUDA 12.1 and matching
    `yopo_server`'s TRT 8 load API); the TRT 8.6 pip package ships no cuDNN, so the cuDNN8 bundled
    with torch inside the image provides `libcudnn.so.8` (see `LD_LIBRARY_PATH` in
    `Dockerfile.yopo`). See "YOPO TensorRT Acceleration" for the inference speedup.

### DA360 Depth Service (`Dockerfile.da360`)

- Python dependencies: `numpy<2`, `flask`, `flask-cors`, `opencv-python-headless`, `pillow`,
  `timm`, `tqdm`, `xformers`.
- `COPY third_party/DA360` into the image: the DA360 **source** ships with the repository (both
  `.gitignore` and `.dockerignore` only exclude the DA360 weights directory
  `third_party/DA360/checkpoints/`; see the repository-root `.dockerignore` for the full list), so
  there is nothing to fetch before building. The **weights** `checkpoints/DA360_large.pth` are not
  baked into the image: `scripts/start_da360_api.sh` downloads them (or you mount local weights)
  before running and passes them via `--model-path`.
- Build/run highlights of `scripts/start_da360_api.sh`:
  - Build network and proxy forwarding work the same as for YOPO (`DA360_BUILD_NETWORK`; it also
    probes the host proxy from `git config` as `DA360_BUILD_PROXY`).
  - The image is labelled `mindcloud.da360.server_sha`. There are two skip paths: with the default
    `DA360_MOUNT_SERVER=1` the rebuild is **skipped unconditionally whenever the image exists** (no
    SHA comparison, because the script gets mounted read-only over the same path); with
    `DA360_MOUNT_SERVER=0` it compares the SHA instead — the rebuild is skipped when an image
    exists and the server script SHA matches, and only `DA360_FORCE_BUILD=1` or a changed script
    triggers a rebuild.
  - On build failure it does **not** start a stale image by default (relax with
    `DA360_ALLOW_STALE_IMAGE=1`).
  - Port 5688; `da360_server.py` and the model weights are mounted read-only at runtime.

### Common Causes of Build Timeouts / Failures

- Both CUDA backends build on `pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime`, which is very large;
  pulling it from Docker Hub easily times out on a poor network (the failure message of
  `start_da360_api.sh` points this out explicitly).
- Fixes: the script retries 3 times, so just rerun it once the network recovers; or make sure a
  proxy is reachable (on the YOPO side `Dockerfile.yopo` defaults to `127.0.0.1:7890`; the DA360
  side hard-codes no port and only forwards the host's `HTTP(S)/FTP/ALL/NO_PROXY`); or point
  `YOPO_BASE_IMAGE` / `DA360_BASE_IMAGE` at a local or mirrored image.
- If pip dependency installation fails, verify that you build with `--network=host` and that the
  proxy is reachable.

### `.dockerignore`

The build context excludes `.git`, `node_modules`, `__pycache__`, `*.pyc`, `scene/*`,
`asset/gate-paths/*.tmp` and `third_party/DA360/checkpoints` (the DA360 weights), keeping
unrelated / large files out of the image; the DA360 source still ships inside the image.

## DA360 Depth Estimation

The DA360 depth service is brought up by `restart_all.sh` (using the `large` model by default). The
DA360 **source** ships with the repository, but the **weights** (`DA360_large.pth`, ~1.3GB, over
GitHub's 100MB limit) do not — **downloading the weights is all you need before the first run**
(there is no source to fetch before building the image):

```bash
python3 -m pip install --user gdown
./scripts/download_da360_model.sh
# The script puts the weights at third_party/DA360/checkpoints/DA360_large.pth
```

Heartbeat self-check after startup:

```text
curl http://127.0.0.1:5688/health
```

To stop or restart DA360, just rerun `restart_all.sh` (or `docker rm -fv mindcloud-da360-api`).

Note that `DA360_large` is used by default and `scripts/start_da360_api.sh` starts its container with
`DA360_INPUT_SCALE=0.65`, giving a model input of about `672x336` (checkpoint baseline 1036×518 ×
0.65; verified stable on an RTX 4070 Laptop GPU 8GB; `da360_server.py`'s own default is `1.0`, i.e.
inference at the checkpoint's native resolution).

The panorama RGB is captured at `384x192` ERP by default, and that raw size is exactly what the
bottom-right preview shows. This size is identical to what DA360 outputs and what YOPO consumes, so
`da360UploadScale` defaults to `1.0` — uploaded as-is with no scaling; the server resizes it to the
`672x336` model input, infers, and maps the depth back onto `384x192`. The frontend defaults to
`depthMs=33` (minimum ~30Hz interval between depth requests) and never queues up requests while
inference is still running.

Switching models is not recommended by default; in experiments the fast tier of `DA360_large`
preserves better depth ordering and edge consistency than `DA360_small`. Only override the model
name when VRAM, power or deployment size is constrained:

```bash
DA360_MODEL=<large|base|small> ./scripts/download_da360_model.sh
DA360_MODEL=<large|base|small> ./scripts/start_da360_api.sh
```

To actively change the DA360 server-side model input size, set the inference scale or specify the
model input width/height; too low a `DA360_INPUT_SCALE` can make the large model output banded
depth, so values below `0.46` are discouraged. The resample filter has **different defaults in two
places**: `da360_server.py` itself defaults to `bilinear`, while `scripts/start_da360_api.sh`
overrides it with `bicubic` and forwards it into the container (so `bicubic` is what actually takes
effect when you use the one-shot launcher):

```bash
DA360_INPUT_SCALE=1.0 ./scripts/start_da360_api.sh
DA360_INPUT_SCALE=0.46 ./scripts/start_da360_api.sh
DA360_INPUT_WIDTH=476 ./scripts/start_da360_api.sh
DA360_INPUT_WIDTH=672 DA360_INPUT_HEIGHT=336 ./scripts/start_da360_api.sh
DA360_RESAMPLE=bilinear ./scripts/start_da360_api.sh
```

When the inference service does not run on this machine:

```text
http://127.0.0.1:8080/?da360Url=http://<host>:5688/depth
```

## Usage Flow

1. Click **Start Google 3D Tiles Flight**.
2. Wait for the page to enter **PLACEMENT MODE**.
3. Search for a city or place with the Cesium search box.
4. Hold `I` and click a building, road or the ground to set the spawn point.
5. Fine-tune the horizontal position with `W/A/S/D`; hold `Shift` to speed up the adjustment.
6. Set **SPAWN ALTITUDE (m)**.
7. Press `O` to confirm the spawn point.
8. Choose **First Person** or **Third Person** to start flying.

Common keys:

```text
↑ / ↓       forward / backward
← / →       strafe left / right
W / S       ascend / descend
A / D       yaw left / right
Shift       boost
R           reset
V           cycle view
P           back to placement mode
Tab         settings panel
```

The keyboard works out of the box; gamepads are supported too (but the mapping is yours to tune) and
are usually detected automatically by Chrome's Gamepad API. RC transmitters or WebHID devices can be
connected from the settings panel; to check Linux input permissions:

```bash
./launch.sh --input-status
./launch.sh --setup-input
```

## How the Panoramic Camera Works

The panorama RGB is captured from the nose-mounted 360° camera by default and output as a `384x192`
ERP image. It works by sampling the Cesium/Google Tiles render result in 6 directions and then
re-projecting it on the GPU with the ERP ray model:

```text
yaw   = pi - (u + 0.5) / W * 2pi
pitch = vfov / 2 - (v + 0.5) / H * vfov
```

This keeps the projection model identical to upstream YOPO's ERP camera; the difference is that the
data comes from the Cesium rendered view instead of a direct raycast of a simulated grid. The
panorama sampling viewer is created in the background during placement; after the spawn point is
confirmed, one panorama first frame is pre-sampled before the user takes control. In flight the
defaults are `panoMs=12`, `panoFace=128`, a per-direction wait of `panoFrameDelayMs=8`, and a wait of
up to `panoFaceTileTimeoutMs=140` (`panoFaceTileTimeoutMsFast=110` while navigating) for the tiles of
that direction to go idle; the first-frame preload uses `panoPreloadFrameDelayMs=96`,
`panoPreloadFaceTileTimeoutMs=6000` and `panoPreloadTimeoutMs=60000`, with `panoPreloadRequired=0` by
default, so flight may start even if the first frame is incomplete and live sampling keeps filling it
in (force a complete first frame with `?panoPreloadRequired=1`). To avoid mirage-like artefacts at
the top of the ERP caused by Google Tiles sky / polar sampling, a polar guard is applied to the top
10° and bottom 2° by default; the guard region keeps ERP coordinates and only fades the sampling
toward the poles, without squeezing the whole image onto the guard boundary. Tune or disable it with
`panoTopPoleGuard` / `panoBottomPoleGuard` (set to 0).

Before controllable flight, the main Cesium view preloads the area around the spawn point and waits
for the first-person and third-person initial view tiles to go idle. `flightPreloadStrict=0` by
default, so the main view continues as soon as the coverage of the target area is good enough; the
panorama first-frame preload separately checks that the 6 directions of the hidden viewer are idle.
`panoPreloadRequired=0` by default means flight may start after an incomplete panorama first frame,
with live sampling retrying in the background.

Common parameters:

```text
# Higher output resolution (default is already 384×192; raise to 672/896 with spare VRAM,
# or drop to 320 when bandwidth is tight)
http://127.0.0.1:8080/?panoWidth=896&panoFace=224

# Tune the sampling view wait time
http://127.0.0.1:8080/?panoFrameDelayMs=16&panoPreloadFrameDelayMs=120

# Tune the first-frame panorama preload timeout; or allow flight after a failed first frame
http://127.0.0.1:8080/?panoPreloadTimeoutMs=90000&panoPreloadFaceTileTimeoutMs=9000
http://127.0.0.1:8080/?panoPreloadRequired=0

# Tune the pre-takeoff main view preload radius and coverage threshold
http://127.0.0.1:8080/?flightPreloadRadius=600&flightPreloadMinCoverage=0.98

# Tune the RGB / depth update interval
http://127.0.0.1:8080/?panoMs=1000&depthMs=1200

# Tune the ERP polar guard
http://127.0.0.1:8080/?panoTopPoleGuard=0&panoBottomPoleGuard=0

# DA360 upload size (da360UploadScale defaults to 1.0, i.e. 384×192 uploaded as-is, no scaling)
# Lower it to save bandwidth / raise responsiveness (becomes 192×96); for more accuracy raise
# panoWidth instead of the upload scale
http://127.0.0.1:8080/?da360UploadScale=0.5
http://127.0.0.1:8080/?da360UploadWidth=512
```

## YOPO Autonomous Navigation

Powered by the YOPO end-to-end navigation network, the drone can fly autonomously to a given goal.
YOPO takes the ERP panoramic depth map, odometry and the goal, outputs position/velocity/
acceleration/yaw commands, and drives the drone through the SimpleFlight cascaded PID controller.

### Navigation Architecture (Aligned with Upstream YOPO)

- **Network input**: `depth (1,2,192,384)` (channel 0 = normalized depth, channel 1 = validity mask)
  plus a 9-dimensional observation (camera-frame velocity / acceleration / goal direction),
  expanded by `prepare_input` into `(1,9,6,12)`.
- **Trajectory selection (server: pure YOPO argmin)**: the network outputs the terminal states (PVA)
  and the score of 72 candidate trajectories (12 horizontal × 6 vertical anchors). **The server
  strictly follows the deployment implementation of upstream YOPO's `test_yopo_ros.py` and selects
  the best trajectory with a plain `argmin(score)`,** without stacking any geometric collision cost.
  The first layer of avoidance comes entirely from the score the network learned through
  `safety_loss` during training (**learning-based avoidance**), exactly as in the official
  deployment.
- **Client-side reactive safety layer (geometric potential field)**: on top of the server
  trajectory, the frontend `src/drone.js` adds a geometric reactive avoidance layer based on a Cesium
  ray ring, to cover sudden near obstacles during the depth replanning gap (~70 ms per replan).
  When the path is clear this layer automatically goes to zero and never interferes with the
  network's planning — see "Avoidance Architecture and Tuning".
- **Goal guidance**: the score already contains the goal-direction cost (trained with `wg=0.15`), so
  the network natively points at the goal.
- **3D navigation**: no projection onto the horizontal plane; vertical avoidance is decided by the
  network-predicted z terminal state.
- **Trajectory generation**: three-axis quintic polynomial (Poly5Solver) starting from the last
  commanded state (`plan_from_reference=True`), so trajectories are continuous and never backtrack.
- **Control output**: the polynomial is evaluated at 50 Hz → position / velocity / acceleration +
  yaw → tracked by the frontend cascaded PID.
- **Final-approach takeover (< 12m)**: there is one layer on each side. Server-side, within
  `FINAL_APPROACH_DIST=12.0` it stops network inference and directly plans a quintic polynomial with
  **zero terminal velocity/acceleration** — near the goal `argmin(score)` repeatedly picks
  overshoot/turn-back trajectories, which combined with `plan_from_reference` flips the goal-direction
  observation and makes position/velocity oscillate forever. Client-side, within
  `yopoFinalApproachDist=12.0` (judged by the **3D distance**, so a horizontally-close but
  vertically-far goal does not switch) a PD loop takes over (position loop + velocity/acceleration
  feed-forward, with a rate limit on the velocity change to avoid jitter).
  - **The ray layer stays active inside the takeover zone and keeps priority**: probing and braking
    run throughout; once the ray layer sees a real threat (filtered `brake < 0.97`), the tangential
    detour (tan) fades in over 0.4 s with hysteresis and goes through the same lateral speed budget
    as the PD (budget base ≥ `yopoCruiseMinSpd`), so avoidance takes **priority** over the
    PD's straight-in component (`rep`/`vGo`/`vRep` stay off to avoid fighting the PD along the goal
    line); it falls back to the pure PD once the corridor is clear again (`brake > 0.995`).
    **The ray layer stands fully DOWN at the takeover end-game**: inside the last
    `yopoTakeoverSteerEndDist = 3.0` m, or when the goal sits against a wall (the `gateBeyondGoal`
    release puts `vCloseMax = ∞`, i.e. the threat IS the goal), the ray layer immediately backs off —
    `brake` is forced to 1 (no more velTarget scaling), the tangential steer is switched off and the
    braking feed-forward is dropped, handing full control back to the PD. Otherwise the ray layer reads
    the wall-adjacent goal as an obstacle: tan keeps shoving the drone off the wall while the PD pulls
    it back, the two fight back and forth → `yopoArrived` never latches because the speed is held off
    the goal → the drone sways at the goal forever. This stand-down only happens once the drone is on
    (or almost on) the goal; the PD's `holdMaxV = √(2ad)` already guarantees a physically stoppable
    run-in, so the ray layer adds only jitter here and no real benefit.
  - **Faster, calmer settle after takeover**: the velocity-target slew cap `yopoTakeoverSlew`
    20 → 14 m/s² (still above the airframe's acceleration ceiling, so it only filters frame-to-frame
    steps) and the slew now covers the **vertical axis** too (removes the vertical bobbing at the
    goal); the damping `holdKd` is distance-scheduled — 1.5 across the zone, ramping linearly to 2.8
    over the last 3 m — and `holdKp` is scheduled the same way (5 → 8) to compensate: the terminal
    steady-state coefficient returns to ~2.1× the remaining distance and the settling time constant
    drops from 0.76 s to 0.48 s. The vertical velocity target now gets a
    `min(√(2a·|Δh|), 4 m/s)` cap as well (the vertical loop previously had no holdMaxV-style
    ceiling, so a 2-3 m height error commanded a 9-13 m/s climb/descent that then took ages to undo).
  - **A wall-adjacent goal no longer pins the drone metres short**: with the goal against a wall the
    forward ray measures `dAhead ≈ distGoalH`, so the strict `dAhead > distGoalH` test missed the
    beyond-goal exemption → `brakeClear ≤ standoff + reaction distance` gave `brake = 0` (the 0.40
    floor sat inside the branch that never ran) AND the closing gate clipped the goalward component
    to 0 — the drone stalled metres short and took forever to arrive. A new
    `yopoAvoidGoalGateMargin = 1.0` treats a threat within 1 m of the goal's horizontal distance as
    beyond-goal (the `yopoAvoidGoalBrakeFloor = 0.40` floor now applies after the branch, covering
    the `brake = 0` case, and the gate gains a third release); the final deceleration stays with the
    PD, whose `holdMaxV = √(2ad)` already guarantees a physically stoppable run-in.
- **Depth availability**: when DA360 depth fails or times out it does **not** fall back to raycasting;
  the drone hovers in place and keeps retrying until a valid depth map arrives (see "Depth Map").
  Note: an earlier version had an "abnormal depth" check that hovered when the whole frame was
  surrounded within 2 m; it mistook "many near pixels" for a depth failure in real urban building
  clusters and hovered constantly, so it was removed to match upstream. Depth validity is now left to
  the mask channel and the network itself.
- **Cruise speed floor (`yopoCruiseMinSpd=12`)**: when the path is clear and the goal is far, it tops
  up the forward speed along the goal bearing so the network cannot squeeze the speed down to a
  crawl; it yields while the avoidance brake is active and switches off within
  `yopoCruiseMinDist=5` m of the goal, respecting the takeover / arrival deceleration.
- **Vertical-first direct climb/descent (`yopoVertFirst*`)**: when the height error dominates
  (horizontal distance < 20 m and |Δh| > 5 m and > 1.2× the horizontal offset) it takes over the
  vertical channel with a P-converging climb/descent and keeps only 30% of the horizontal command,
  removing large circling; it yields back to the network when the clearance straight above / below is
  insufficient.
- **Arrival and deadband**: arrival latches when within 3.5 m of the goal and below 1 m/s; inside the
  2.5 m horizontal deadband it switches to a direct climb/descent to converge the altitude, and below
  0.35 m the PD stops correcting and only trims the altitude slightly, avoiding jitter around the goal.

### Avoidance Architecture and Tuning

Avoidance has two layers with non-overlapping responsibilities:

| Layer | Location | Mechanism | Role |
|-------|----------|-----------|------|
| Learning-based avoidance | Server `scripts/yopo_server.py` | Network `argmin(score)` trajectory selection (`safety_loss` during training) | Global path planning, steering around large-scale structures |
| Geometric reactive potential field | Frontend `src/drone.js` | Live 360° ray ring (24 rays, 15° spacing) | Covers sudden near obstacles during the depth replanning gap |

How the client-side geometric layer works (see `_avoidanceVelocity`):

- **Probing**: 24 horizontal rays are cast from the body (radius 55 m, 15° spacing); the 3 rays best
  aligned with the forward direction additionally probe **two layers up and one layer down**
  (`high`/`high2`/`low`, 3 layers in total) for the vertical clearing decision; plus straight
  up/straight down vertical rays.
- **Output components**: `rep` (radial push-away) / `tan` (tangential detour) / `brake` (near-obstacle
  braking) / `vRep` (vertical obstacle clearing) / `vGo` (horizontal detour around a vertical
  obstacle footprint) / `upPush` + `vSafeDown` (ground and descent safety).
- **Braking (the ray layer takes priority over the network)**: the hard kinematic brake
  `v_safe = √(2·a·(d − standoff))` plans a safe speed (`a` uses the conservative
  `yopoAvoidBrakeDecel≈7.5 m/s²` to leave margin). When braking fires it (1) **suppresses the YOPO
  network's acceleration feed-forward** (otherwise the network trajectory's acceleration pushes
  straight into the obstacle and cancels the braking deceleration), and (2) injects the strongest
  deceleration feed-forward directly opposite the current velocity (up to
  `yopoAvoidBrakeAccel≈17.0 m/s²`, matching the 60° tilt ceiling `droneMaxAngle=60`), delivering at
  least `yopoAvoidBrakeMinFrac=0.85` (≈14.5 m/s²) as soon as braking starts so deceleration is
  immediate and strong enough. The threat distance `dAhead` takes the **smaller** of the
  "network-commanded direction" and the "drone's actual heading", so the network cannot turn the
  command aside and thereby exclude an obstacle straight ahead and skip braking.
- **Lateral speed budget**: while detouring, "forward" and "lateral detour" are budgeted separately —
  lateral takes at most 68% of the budget base and forward keeps at least 10%, so the velocity
  vector really tilts tangentially and slides along the obstacle instead of "charging at full speed
  while grazing it". The budget base is `max(yopoCruiseMinSpd, actual commanded speed)`: the network
  itself slows its commands when the depth shows obstacles, so keying the budget to the commanded
  speed alone made the detour collapse exactly when it was needed (commanded 8 m/s → only ~5.4 m/s
  of steering authority). Also, **tan is NOT decayed by `repHold`** (0.85 floor):
  `repHold = dMin/standoff` linearly scales the field down close to an obstacle — right for `rep`
  ("once stopped, do not keep pushing away") but backwards for `tan`, which needs MORE authority the
  closer the obstacle is.
- **Clear straight flight (`goalClear`)**: a corridor is measured along "body → goal" (`dPath`) and
  along the "commanded velocity direction" (`dCmd`) separately, with a corridor half-width of 2.5 m;
  **either** corridor being clear within `reach = min(yopoAvoidRepRange, horizontal distance to the
  goal)` counts as clear (truncating at the goal keeps a wall *behind* the goal from permanently
  blocking the corridor). Near-goal exception: within `yopoCorridorGuardDist` (12 m) of the goal, if
  `dPath` is blocked then the `dCmd` escape hatch is refused, so the drone cannot charge straight at
  an obstacle that is only a few metres away. **When the corridor is clear, `rep`/`tan`/`brake`/`vRep`
  all go to zero and `vGo` is suppressed**, so the drone flies straight at the goal at full speed,
  never pushed away or detouring without reason.

Key parameters (all in the `src/drone.js` constructor):

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `yopoAvoidEnabled` | `true` | Master switch of the geometric layer |
| `yopoAvoidRayCount` | 24 | Number of 360° rays (15° spacing) |
| `yopoAvoidRange` | 55.0 | Obstacle detection radius (m); ray length is free, lengthening only helps |
| `yopoAvoidRepRange` | 28.0 | Repulsion / tangential / braking range (m); also `goalClear`'s clear threshold — do **not** raise |
| `yopoAvoidRepRangeHi` | 50.0 | The same range at `yopoAvoidRefSpeed` (m) |
| `yopoAvoidRepGain` | 20.0 | Maximum radial push-away speed (m/s) |
| `yopoAvoidTanGain` | 54.0 | Tangential detour gain (m/s); higher = more decisive detour |
| `yopoTanConeCos` | 0.34 | Only use obstacles within a ±70° cone around the goal bearing as the detour reference, so buildings behind/beside cannot steer it off |
| `yopoTanAwayCos` | -0.2 | Drop the remembered tangent when it points >100° away from the goal, allowing a turn back |
| `yopoTanAwayScale` | 0.5 | Scale applied to a tangent pointing >90° away from the goal, avoiding being pushed off the goal |
| `yopoAvoidDecel` | 8.5 | Assumed deceleration used by the *vertical* brake threshold (m/s²) |
| `yopoAvoidBrakeDecel` | 7.5 | *Horizontal* brake planning deceleration (m/s²): deliberately below the reachable value to leave ~2× margin |
| `yopoAvoidBrakeAccel` | 17.0 | Max *actual* deceleration the ray layer may command while braking (m/s²): matches the 60° tilt ceiling (`droneMaxAngle=60`). It injects a deceleration feed-forward opposing velocity and **suppresses the network's acceleration feed-forward** |
| `yopoAvoidBrakeMinFrac` | 0.85 | Deliver at least 0.85×`BrakeAccel` (≈14.5 m/s²) as soon as braking starts |
| `yopoAvoidBrakeReaction` | 0.32 / 0.48 | Brake reaction time (s): base / high-speed (≥ `yopoAvoidRefSpeed`) |
| `yopoAvoidBrakeRange` / `BrakeRangeHi` | 24.0 / 40.0 | Progressive soft-brake zone (m): low / high speed (raised together with `yopoAvoidStopH` 6→7.5 so the `(brakeClear − standoff×2)` normalisation does not degenerate) |
| `yopoAvoidBrakeFloor` | 0.85 | Soft-brake speed floor ratio (still decelerates when close, without over-compressing the cruise) |
| `yopoAvoidStopH` | 7.5 | **Horizontal** brake safety standoff (m): drives the forward brake standoff and the repulsion decay — keeps further off walls / buildings (raised from 6.0 per request) |
| `yopoAvoidStop` | 6.0 | **Vertical** safety clearance (m): drives the up/down clearance brakes (vSafeUp/vSafeDown) and the vertical-clearing block distance; deliberately NOT raised with StopH, because a clearance below it forbids descending entirely (`vSafeDown=0`) and a low goal / the ground would become unreachable |
| `yopoMinAlt` | 3.0 | Minimum ground/roof clearance (m): below it the upward push engages (2.5 → 3.0, less terrain hugging) |
| `yopoAvoidVClimbScale` | 2.2 | Vertical clearing climb strength |
| `yopoAvoidVBlock` | 20.0 | Forward clearance below which vertical clearing triggers (m) |
| `yopoAvoidVGoBase` / `VGoSpan` | 0.60 / 0.42 | Near / far strength of the "leave the footprint" speed (vGo) for an obstacle underfoot |
| `yopoAvoidVClear` | 0.38 | Fraction above which an upper layer counts as clear; lower = stronger clearing willingness |
| `yopoCorridorGuardDist` | 12.0 | Near-goal corridor guard (m): within this distance, a blocked goal-bearing corridor forces braking even if the velocity-direction corridor is clear |
| `yopoCruiseMinSpd` | 12.0 | Cruise speed floor (m/s): tops up forward speed along the goal bearing when the path is clear and the goal is far; yields while braking |
| `yopoCruiseMinDist` | 5.0 | Distance to the goal below which the cruise floor is switched off, respecting the takeover / arrival deceleration |
| `yopoFinalApproachDist` | 12.0 | Final-approach takeover radius (m): inside it a PD loop converges onto the goal |
| `yopoVertFirstEnabled` | `true` | Master switch of the cruise-phase "vertical-first" direct climb/descent |
| `droneMaxVSpeed` | 15.0 | Hard vertical speed ceiling (m/s) |
| `droneMaxAngle` | 60 | Maximum tilt angle (°): the physical tilt ceiling |

> **Tuning tips**: if the detour is not decisive enough, raise `yopoAvoidTanGain` (strength); do
> **not** raise `yopoAvoidRepRange` — it is also `goalClear`'s clearance threshold, so a larger value
> makes "the path is actually clear" cases get misjudged as blocked. After changing frontend
> parameters, press **Ctrl+F5** in the browser for them to take effect.

#### High-speed responsiveness (ray budget & adaptive action range)

The root cause of "avoidance is too late + depth/command updates are too slow when flying fast" is a
single choke point: a full ring probe casts 35 `forceFresh` `pickLocalRay` calls (24 horizontal +
9 across the 3 vertical layers + 2 straight up/down; each is a full GPU render plus a read-back
stall) and runs synchronously inside the render frame loop — so a single probe can take tens to over
a hundred ms. That both stales the avoidance data and collapses the frame rate, which in turn slows
panorama capture, DA360 depth and command replanning. Instead of throttling the whole probe (which
would only stale the forward direction that decides braking), the high-speed profile bounds the
number of GPU picks per cycle:

- **Core cone `yopoAvoidCoreDeg` (±25°)**: keeps the full 15° resolution (24 rays → 360/24) and is
  re-probed every cycle at every speed — this is the sector that decides the braking distance;
  dropping resolution there would open ~30° gaps (a ~15 m hole at 30 m) right where a miss is least
  affordable.
- **Outer cone `yopoAvoidConeDeg` (±55°, narrows to ±45° at speed)**: re-probed every cycle but
  downsampled at speed by `yopoAvoidStrideHi`.
- **Periphery `yopoAvoidSliceMax`**: 6 rays polled per cycle, rotating round-robin (~3 cycles ≈ 60 ms
  to refresh the full ring); directions not re-probed this cycle carry their last measured distance,
  so the repulsion/detour sums always see a complete 360° ring.
- **Vertical layers (high/high2/low)**: only emitted when the previous cycle found the goal corridor
  blocked, saving 9 rays in the normal case.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `yopoAvoidRange` | 55.0 | Obstacle detection radius (m); ray length is free, so lengthening only helps |
| `yopoAvoidFastSpeed` | 6.0 | Speed (m/s) at which the high-speed profile starts |
| `yopoAvoidRefSpeed` | 15.0 | Speed (m/s) at which the high-speed profile is fully applied |
| `yopoAvoidStrideHi` | 2 | Ring ray stride at speed (2 → 30° spacing) |
| `yopoAvoidCoreDeg` | 25 | Core cone half-angle (°), always full 15° resolution |
| `yopoAvoidConeDeg` / `ConeDegHi` | 55 / 45 | Outer cone half-angle (°), low / high speed |
| `yopoAvoidSliceMax` | 6 | Peripheral rays polled per cycle |
| `yopoAvoidRepRangeHi` | 50.0 | Repulsion/detour/brake action range at speed (m) |
| `yopoAvoidTanGain` | 54.0 | Tangential detour gain (m/s), more decisive than 30 |
| `yopoAvoidRepGain` | 20.0 | Maximum radial push-away speed (m/s), more decisive than 12 |
| `yopoAvoidBrakeRangeHi` | 40.0 | Soft-brake start distance at speed (m) |
| `yopoAvoidBrakeReaction` | 0.48 | Brake reaction time at speed (s): the lag (attitude build-up + control loop) is converted to a reaction distance `spd × reaction` subtracted from the stopping room, so at 15 m/s braking starts ~3 m earlier and still stops inside the standoff |

Measured picks per cycle: about 35 at low speed (24 horizontal + 9 vertical layers + 2 straight
up/down), reduced by the budget to about 12 horizontal + vertical layers at high speed (more when
blocked). `yopoAvoidRepRange` (= `goalClear`'s clear threshold) does **not** scale with speed, so
widening the high-speed action range does not make "the path is actually clear" get misjudged as
blocked. Run `__yopoPerf()` in the browser console to read live metrics (`fps` / `probeMsAvg` /
`probeHz` / `depthHz` / `cmdHz` / `ringAgeMaxMs`).

### Start the YOPO Backend

> The YOPO avoidance backend is brought up by `restart_all.sh` (TensorRT enabled by default with
> `YOPO_VELOCITY=15`); the commands below are only for manually building/starting it on its own.

```bash
# The Docker image needs to be built the first time
YOPO_FORCE_BUILD=1 ./scripts/start_yopo_api.sh

# Later starts (build is skipped automatically, local yopo_server.py is mounted)
./scripts/start_yopo_api.sh

# If you hit proxy issues while building, make sure a proxy is available on host port 7890
# Dockerfile.yopo uses --network=host + http://127.0.0.1:7890
```

The service runs at `http://127.0.0.1:5689`. `yopo_server.py` is mounted through a Docker volume, so
editing it needs no image rebuild.

### Key YOPO Backend Environment Variables

Forwarded into the container by `scripts/start_yopo_api.sh`:

| Variable | Default / recommended | Description |
|----------|----------------------|-------------|
| `YOPO_VELOCITY` | 15.0 (set by `restart_all.sh`) | Network planned cruise speed `vel_max` (m/s), which decides the actual flight speed; falls back to the yaml config when unset |
| `YOPO_CTRL_TIME_SCALE` | 1.0 | Command "fast forward" factor. `>1` advances at `vel_max × SCALE` (2 gives ≈30 m/s); it is clamped back by `YOPO_SPEED_CAP`, but the planned position runs ahead and the drone lags behind permanently, so keep it at 1 |
| `YOPO_SPEED_CAP` | 15.0 | Absolute hard ceiling of the commanded speed (m/s), guaranteeing "no speed limit ever goes above 15 m/s" |
| `YOPO_TRAJ_EXTEND_S` | 2.0 | Trajectory tail extrapolation time (s), fixing the command-freeze sawtooth during replan intervals; if replanning still has not happened after that, it falls back to the frozen behaviour to avoid flying blind forever |
| `YOPO_USE_TRT` | 1 (set by `restart_all.sh`) | TensorRT acceleration switch, see "YOPO TensorRT Acceleration" |

> The reactive-budget speed governor (which used to cap speed dynamically by the replan interval) was
> removed: flight speed is decided directly by `YOPO_VELOCITY × YOPO_CTRL_TIME_SCALE` and hard-clamped
> by `YOPO_SPEED_CAP`; avoidance is now guaranteed jointly by "server-side network `argmin(score)` +
> client-side geometric reactive potential field" (see "Avoidance Architecture and Tuning").

### YOPO TensorRT Acceleration

YOPO inference uses TensorRT (TRT) acceleration by default. Freezing `epoch50.pth` into an fp16
engine drops the per-inference latency from ~100-350 ms (PyTorch eager) to 1-5 ms, making
replanning more frequent, the blind-flight segments shorter and avoidance smoother.

- **Engine path**: `asset/yopo-trt/yopo_trt.pth` (committed; fp16, tied to this machine's GPU
  architecture).
- **One-step enable**: `restart_all.sh` sets `YOPO_USE_TRT=1` unconditionally, and the backend log
  prints `[TensorRT] loaded … -- inference acceleration enabled` after startup. Note that the
  assignment in `restart_all.sh` is hard-coded, so `YOPO_USE_TRT=0 ./restart_all.sh` has **no
  effect**; to force the PyTorch eager fallback, call the backend script directly:
  ```bash
  YOPO_USE_TRT=0 ./scripts/start_yopo_api.sh
  ```
  (`scripts/start_yopo_api.sh` only auto-detects the engine when `YOPO_USE_TRT` is unset: enable it
  if the engine exists, otherwise fall back to PyTorch eager.)
- **Automatic build**: if TRT is enabled but the engine is missing,
  `scripts/start_yopo_api.sh` uses the GPU inside the YOPO container to freeze the current model
  (`YOPO_MODEL_PATH`, default `epoch50.pth`) into an engine and writes it to `asset/yopo-trt/`
  (mounted read-write); the next start loads it directly, no manual preprocessing needed.
- **Manual conversion**: when changing the model or rebuilding the engine, run the conversion script
  inside a GPU-enabled container (`scripts/yopo_trt_transfer.py`: `epoch50.pth` →
  `torch.onnx.export` (`depth[1,2,192,384]` + `obs[1,9,6,12]`) → TRT fp16 engine, compatible with
  TRT 8.x / 10+):
  ```bash
  docker run --rm --gpus all \
    -v "$PWD/third_party/yopo:/opt/mindcloud-yopo/third_party/yopo:ro" \
    -v "$PWD/scripts/yopo_trt_transfer.py:/opt/mindcloud-yopo/scripts/yopo_trt_transfer.py:ro" \
    -v "$PWD/third_party/yopo/saved/YOPO_40/epoch50.pth:/models/epoch50.pth:ro" \
    -v "$PWD/asset/yopo-trt:/opt/mindcloud-yopo/trt:rw" \
    mindcloud-yopo:latest python /opt/mindcloud-yopo/scripts/yopo_trt_transfer.py \
      --model /models/epoch50.pth --out /opt/mindcloud-yopo/trt/yopo_trt.pth
  ```
- **Changing GPU / architecture**: a TRT engine is tied to the GPU's SM compute capability, and the
  current engine was built on an RTX 4070. When deploying to an Orin NX or another card, delete the
  old engine on the target machine and let `start_yopo_api.sh` rebuild it (or rerun the command
  above).
- **Environment constraint**: TensorRT inside the container is pinned to `8.6.1` (matching the CUDA
  12.1 runtime and `yopo_server`'s TRT 8 load API); the TRT 8.6 pip package ships no cuDNN, so the
  cuDNN8 bundled with torch inside the image provides `libcudnn.so.8` (see `LD_LIBRARY_PATH` in
  `Dockerfile.yopo`).

### Goal Selection and Navigation

1. In flight mode, press **`T`** (or click **"Pick Target"** in the YOPO panel on the right) to start
   setting the goal.
2. The goal starts at the drone's current position; move it with the **numpad** (directions are
   relative to the **drone's current nose heading**):
   - `Numpad 8 / 2`: forward / backward along the nose
   - `Numpad 4 / 6`: strafe **right / left**, perpendicular to the nose (numlock layout is inverted:
     4 = the drone's right side, 6 = its left; see `handleYOPOKeyDown` in `src/main.js`)
   - `Numpad 9 / 3`: ascend / descend
3. **`Numpad 5`**: confirm the goal and **start navigation automatically**.
4. **`Numpad 0`** or **`Esc`**: cancel the selection.

During navigation:
- The drone follows the path with YOPO trajectory commands plus velocity feed-forward
- Moving the sticks temporarily switches to manual control (navigation resumes when released)
- **Avoidance (server-side learning-based + client-side geometric reactive, two layers)**: the
  server strictly follows YOPO and selects the trajectory by `argmin(score)` (learning-based
  avoidance); while tracking the commands, the client additionally stacks a geometric reactive
  potential field (360° ray ring: radial push-away, tangential detour, near-obstacle braking,
  vertical obstacle clearing, detour around vertical obstacle footprints) to cover sudden near
  obstacles during the depth replanning gap. When the horizontal corridor toward the goal is clear
  this geometric layer goes to zero and does not interfere with navigation — see "Avoidance
  Architecture and Tuning".
- Arrival has two layers: the server flags arrival within 2 m of the goal (`ARRIVE_THRESHOLD`); the
  client additionally latches arrival within 3.5 m and below 1 m/s, so the asynchronous server reply
  cannot leave it "always one step short"
- Press **`X`** (or click **"Stop Nav"**) to end navigation

The goal marker stays visible while navigating, after arrival and after stopping, until you pick a
new goal or cancel it, so you can still see the goal position on the second navigation.

### Top-Down Minimap (Target Map)

A **Target Map (Top-Down)** minimap is permanently docked in the bottom-left corner of the main
interface and refreshes live in flight, giving an intuitive view of the drone's position relative to
the goal:

- Centred on the drone, it shows the drone's current heading, the goal position and the line between
  them, projected onto the horizontal plane.
- The two text rows below the map give the **target altitude y** (the goal's `y` in the **local
  coordinate system**, i.e. its height above the local origin, in m) and the **Δx/Δy/Δz to target**
  (the goal's east/up/north displacement relative to the drone, in m).
- After entering goal selection mode (press `T`), the map updates in sync with numpad movements of
  the goal, making it easy to align the goal in space.
- The minimap carries no data-attribution watermark; it is drawn purely on the frontend and depends
  on no external map service.

### Depth Map

YOPO needs a **384×192 ERP panoramic depth map** (its native input format) with two channels:
channel 0 = normalized depth [0,1], channel 1 = validity mask. The acquisition flow:

1. DA360 panoramic depth estimation → ERP depth map (metric)
2. The frontend reprojects/crops it to 384×192 ERP and attaches the validity mask
3. It is fed straight into the network (the depth values themselves come from DA360 and are never
   mixed with ray-synthesised geometric depth; only 4 sparse Cesium rays are used for **metric scale
   calibration**, converting DA360's relative depth into metres)

**When depth is unavailable (DA360 failure/timeout) it does not fall back to Cesium raycasting** —
YOPO's network input still requires real depth, so the drone hovers in place and keeps retrying until
a valid depth map arrives and navigation resumes.

> This does not conflict with the client-side geometric reactive avoidance: the latter (see
> "Avoidance Architecture and Tuning") is an independent safety backstop layer that probes live with a
> Cesium ray ring, depends on no DA360 depth, and does not take part in the network input.

### Coordinate Systems

| Coordinate system | x | y | z | Forward |
|-------------------|---|---|---|---------|
| MindCloud / Cesium | East | Up | North | -z |
| YOPO / ROS FLU | Forward | Left | Up | +x |

Goals are set in the MindCloud coordinate system and converted automatically by the server.
