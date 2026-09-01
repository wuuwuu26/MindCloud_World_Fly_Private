<div align="center">

**🌐 [English](README_EN.md) | [简体中文](README.md)**

</div>

# MindCloud World Fly with YOPO

![YOPO 导航界面展示](asset/display/screen.png)

> **YOPO 导航界面展示**：右下角为机头 360° ERP 全景 RGB 与 DA360 深度图，左下角为 Target Map 俯视小地图（无人机与目标点的相对位置），右侧面板用于选取目标点、开始/停止导航，并显示导航状态、到目标距离与推理计数。

浏览器中的 Google Photorealistic 3D Tiles 穿越机驾驶器，集成 YOPO 端到端神经网络自主导航（3D 避障）。进入页面后选择城市、放置出生点，然后用键盘、手柄或 RC 遥控器飞行，或设置目标点让 YOPO 自主导航。右下角可显示机头 360 ERP 全景 RGB 和 DA360 深度。

## 克隆仓库

```bash
git clone https://github.com/wuuwuu26/MindCloud_World_Fly_Private.git
cd MindCloud_World_Fly_Private
```

> 注意：DA360 的**源码**已随仓库纳入版本管理（对 DA360 而言 `.gitignore` 只忽略权重目录 `third_party/DA360/checkpoints/`，完整忽略清单见仓库根目录 `.gitignore`），克隆后即可获得源码；但**权重**（`DA360_large.pth`，约 1.3GB，超 GitHub 100MB 限制）未入库，运行深度服务前需先下载权重。

> **新机器请看下一节「从零开始（首次部署）」**：它包含 Docker / NVIDIA Container Toolkit 安装、权重下载、Cesium Ion token 配置和首次镜像构建的完整步骤。已经部署过的机器直接看「日常启动 / 部分重启 / 停止」。

## 从零开始（首次部署）

下面按「一台全新机器 → 能手动飞 + 能 YOPO 自主导航」的顺序走一遍。**首次部署完成后，日常只需 `./restart_all.sh`**（见下一节「日常启动 / 部分重启 / 停止」）。

### 第 0 步：安装前置软件

```bash
# 1) Docker Engine（其它系统见 https://docs.docker.com/engine/install/）
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER"    # 注销并重新登录后生效
newgrp docker                      # 或只在当前 shell 临时生效

# 2) NVIDIA 驱动（能跑 nvidia-smi 即可）
nvidia-smi

# 3) NVIDIA Container Toolkit（让容器能用 GPU）
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 4) 验证容器里能看到 GPU
docker run --rm --gpus all nvidia/cuda:12.1.1-base-ubuntu22.04 nvidia-smi
```

其它必要条件：

| 项目 | 要求 |
|------|------|
| 磁盘 | 三个镜像实测共约 **64 GB**（YOPO ≈35 GB、DA360 ≈28 GB、主飞行 ≈1 GB），再加 1.3 GB 的 DA360 权重，**建议预留 ≥80 GB** |
| 网络 | 首次需从 Docker Hub 拉基础镜像、从 Google Drive 拉权重；浏览器需能访问 Cesium Ion / Google 3D Tiles 与 `cdn.jsdelivr.net`（PlayCanvas） |
| `curl` | `restart_all.sh` 用它等待服务就绪（一般系统自带） |
| `python3` + `gdown` | 仅下载 DA360 权重时需要 |

> 只想先用键盘飞（不跑 DA360 / YOPO）的话，可以跳过 GPU 与权重相关步骤，直接 `./launch.sh`，只构建约 1 GB 的主飞行镜像。

### 第 1 步：拉取代码

若还没克隆，先执行上面「克隆仓库」的两行命令并进入仓库目录：

```bash
cd MindCloud_World_Fly_Private
```

### 第 2 步：下载 DA360 深度权重

YOPO 权重与 TRT 引擎已随仓库提供，**只有 DA360 权重**（约 1.3 GB）需要单独下载：

```bash
python3 -m pip install --user gdown
./scripts/download_da360_model.sh
# 写入 third_party/DA360/checkpoints/DA360_large.pth
```

DA360 源码已随仓库提供，脚本检测到源码存在时只下载权重。

### 第 3 步：配置 Cesium Ion token（推荐）

主视图与全景相机都通过 Cesium Ion 拉取 Google Photorealistic 3D Tiles。仓库内置的默认 token（`src/cesium-world.js` 里的 `DEFAULT_ION_TOKEN`）仅供体验，随时可能失效或撞配额，**建议换成自己的**：

1. 在 <https://ion.cesium.com/tokens> 新建 token；
2. 为它开通 Google Photorealistic 3D Tiles / `assets:read` 权限；
3. 用 URL 参数传入：

```text
http://127.0.0.1:8080/?ionToken=你的token
```

也可以直接改 `src/cesium-world.js` 的 `DEFAULT_ION_TOKEN` 常量把它固化下来（`src/` 是只读挂载进容器的，改完浏览器 Ctrl+F5 强刷即可，**不用重建镜像**）。若 token 没有 Google 3D Tiles 权限，页面会自动回退到内置 ion 资产 `2275207`（`DEFAULT_ASSET_ID`），也可用 `?assetId=` 覆盖。

### 第 4 步：一键启动（首次会自动构建三个镜像）

```bash
./restart_all.sh
```

脚本依次拉起：DA360 深度服务 → YOPO 导航服务 → 主飞行进程。各入口脚本**只在镜像不存在时才构建**，所以首次会比较慢（主要是拉取 CUDA 基础镜像 + 装 pip 依赖，通常需要几十分钟），之后都是秒级重启。

构建/启动日志分别落在：

```bash
tail -f /tmp/restart_da360.log
tail -f /tmp/restart_yopo.log
tail -f /tmp/restart_main.log
```

也可以提前或单独构建某一个镜像：

```bash
./launch.sh                                      # 主飞行镜像（缺镜像才构建；加 --rebuild 强制）
YOPO_FORCE_BUILD=1 ./scripts/start_yopo_api.sh   # YOPO 镜像
DA360_FORCE_BUILD=1 ./scripts/start_da360_api.sh # DA360 镜像
```

> YOPO 推理默认走 TensorRT 加速（引擎 `asset/yopo-trt/yopo_trt.pth` 已随仓库提供）。`restart_all.sh` **无条件**设 `YOPO_USE_TRT=1`，无需额外操作；引擎缺失时它只打印一条 WARN，实际由 `scripts/start_yopo_api.sh` 在 YOPO 容器内用 GPU 自动构建引擎。详情见「YOPO TensorRT 加速」。

### 第 5 步：确认三个服务都活着

```bash
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'  # 应看到下面 3 个容器
curl http://127.0.0.1:8080/              # 主飞行：返回页面 HTML
curl http://127.0.0.1:5688/health        # DA360：健康自检
curl http://127.0.0.1:5689/yopo/status   # YOPO：服务状态
```

### 第 6 步：打开浏览器起飞

访问 `http://127.0.0.1:8080` → 点 **Start Google 3D Tiles Flight** → 搜索框选城市 → 按住 `I` 点地面设出生点 → 按 `O` 起飞，详见「使用流程说明」。

### 首次部署常见问题

| 现象 | 原因 / 处理 |
|------|-------------|
| `docker: permission denied ...` | 当前用户不在 `docker` 组：`sudo usermod -aG docker $USER` 后重新登录 |
| `could not select device driver "" with capabilities: [[gpu]]` | 未安装/未配置 NVIDIA Container Toolkit：`sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker` |
| 构建卡在拉 `pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime` | 基础镜像很大，网络差时易超时；脚本内置 3 次重试，网络恢复后重跑；或把 `YOPO_BASE_IMAGE` / `DA360_BASE_IMAGE` 指向镜像源/本地镜像 |
| 构建时装 pip 依赖失败 | 构建走 `--network=host`：YOPO 侧默认使用宿主 `127.0.0.1:7890` 代理，DA360 侧转发宿主 `HTTP(S)_PROXY` 并从 `git config` 探测代理 |
| `Port 8080 is already in use` | 换端口：`PORT=18081 ./launch.sh` 或 `./launch.sh --port 18081` |
| DA360 长时间不 ready | 看 `/tmp/restart_da360.log`；权重缺失时脚本会自动调用 `download_da360_model.sh`，慢通常是 Google Drive 下载慢 |
| YOPO 首次启动较慢 | 启用 TensorRT 但 `asset/yopo-trt/yopo_trt.pth` 不存在时，会在容器内用 GPU 现场固化引擎并写回该目录，之后直接加载 |
| 页面空白 / 3D Tiles 加载不出来 | 多半是 token 失效或无 Google 3D Tiles 权限，按第 3 步换自己的 token；同时确认浏览器能访问 Ion 与 `cdn.jsdelivr.net` |

## 日常启动 / 部分重启 / 停止

首次部署完成后（见上一节），日常只需 `./restart_all.sh`。镜像已存在时不会重建，所以这是最快的重启方式；三个服务默认都以后台 detach 方式运行：

```bash
./restart_all.sh                 # 全部重启
./restart_all.sh --no-da360      # 只重启 YOPO + 主飞行（DA360 保留）
./restart_all.sh --no-yopo       # 只重启 DA360 + 主飞行（YOPO 保留）
./restart_all.sh --no-main       # 只重启 DA360 + YOPO（主飞行保留）

docker logs -f mindcloud-yopo-api   # 查看某服务日志

# 停止全部后台容器（与 restart_all.sh 的停止逻辑一致，带 -v 清理匿名卷）
docker rm -fv google-tiles-flight mindcloud-da360-api mindcloud-yopo-api
```

容器名由 [restart_all.sh](restart_all.sh) 顶部的 `MAIN_NAME` / `DA360_NAME` / `YOPO_NAME` 定义（与各入口脚本的默认容器名一致）：

| 容器名 | 用途 | 定义处 |
|--------|------|--------|
| `google-tiles-flight` | 主飞行进程（`http://127.0.0.1:8080`） | `launch.sh` 的 `NAME="${NAME:-google-tiles-flight}"` |
| `mindcloud-da360-api` | DA360 深度服务（`http://127.0.0.1:5688`） | `scripts/start_da360_api.sh` 的 `DA360_CONTAINER_NAME` |
| `mindcloud-yopo-api` | YOPO 避障后端（`http://127.0.0.1:5689`） | `scripts/start_yopo_api.sh` 的 `YOPO_CONTAINER_NAME` |

容器名在 `restart_all.sh` 里是**硬编码赋值**（`DA360_NAME=` / `YOPO_NAME=` / `MAIN_NAME=`），**不接受环境变量覆盖**——`DA360_CONTAINER_NAME=my-da360 ./restart_all.sh` 不会生效。要改名请直接改这三个变量；或者单独调用对应的入口脚本，那三个脚本支持 `DA360_CONTAINER_NAME` / `YOPO_CONTAINER_NAME` / `NAME` 环境变量。

若只想先飞（纯键盘/手柄/RC，不依赖子服务），也可单独运行主进程：

```bash
./launch.sh
```

## 模型权重

| 模型 | 是否随仓库提供 | 获取方式 |
|------|----------------|----------|
| YOPO 导航权重 | **是**（直接提交，≤100MB） | 克隆即得，位于 `third_party/yopo/saved/`（默认 `YOPO_40/epoch50.pth`） |
| YOPO TensorRT 引擎 | **是**（直接提交，≤100MB） | 位于 `asset/yopo-trt/yopo_trt.pth`（fp16）；由 `epoch50.pth` 转换，换 GPU/模型时重建，见「YOPO TensorRT 加速」 |
| DA360 深度权重 | 否（约 1.3GB，超限） | 下载脚本：`./scripts/download_da360_model.sh`（Google Drive，需 `gdown`） |

### YOPO 导航权重

已直接提交到仓库，克隆后即位于 `third_party/yopo/saved/YOPO_40/epoch50.pth`。默认路径见 `scripts/start_yopo_api.sh`（`YOPO_MODEL_PATH`），可用环境变量覆盖：

```bash
YOPO_MODEL_PATH=/abs/path/to/your_yopo.pth ./scripts/start_yopo_api.sh
```

### DA360 深度权重

因单文件超过 GitHub 100MB 限制，未纳入仓库，请运行下载脚本（需先 `pip install gdown`）：

```bash
./scripts/download_da360_model.sh
# 脚本会将权重放到 third_party/DA360/checkpoints/DA360_large.pth
```

## 环境要求

必需：

- Docker Engine（建议 24+），且当前用户有权限执行 `docker`（加入 `docker` 组）
- NVIDIA GPU + 驱动（DA360 / YOPO 需要）+ NVIDIA Container Toolkit（容器用 GPU）
- 磁盘 ≥80 GB：三个镜像实测共约 64 GB（YOPO ≈35 GB、DA360 ≈28 GB、主飞行 ≈1 GB），另加 1.3 GB DA360 权重
- 一个支持 WebGL 的现代浏览器（用于打开 `http://127.0.0.1:8080` 使用模拟器）
- 浏览器可访问 Cesium Ion、Google 3D Tiles 与 `cdn.jsdelivr.net`（PlayCanvas 前端库）

可选 / 特定场景：

- `curl`：`restart_all.sh` 用它等待服务就绪（一般系统自带）
- 本地开发模式（`./launch.sh --local`）需要 Python 3
- 下载 DA360 权重需要 Python 3 + pip（`gdown`）以及可访问 Google Drive 的网络

> 首次部署的完整安装命令见「从零开始（首次部署）→ 第 0 步：安装前置软件」。

### 已验证运行环境

本项目已在以下设备完整验证可运行（DA360 深度 + YOPO 导航 + 主飞行三者同时拉起）：

| 项目 | 配置 |
|------|------|
| GPU | NVIDIA GeForce RTX 4070 Laptop GPU（8 GB 显存） |
| 驱动 / CUDA | 595.84 / 13.2 |
| DA360 配置 | `DA360_large` + `DA360_INPUT_SCALE=0.65`（模型输入 672×336），单卡 8GB 下约 92% 占用 |
| YOPO 配置 | TensorRT 加速，`YOPO_VELOCITY=15` |

> 显存更小（如 6GB 及以下）的 GPU 可下调 `DA360_INPUT_SCALE` 或 `da360UploadScale` 降低占用；显存更充裕的卡可上调以提升深度精度。

## Docker 构建说明

本项目共三套独立构建的容器，各自的镜像名、基础镜像与重建触发方式都不同：

| 容器 | 镜像 | 基础镜像 | 实测体积 | Dockerfile | 入口脚本 |
|------|------|----------|----------|------------|----------|
| 主飞行进程 | `google-tiles-flight` | `tumgis/3dcitydb-web-map:alpine-v2.0.0`（自带 Node + Cesium） | ≈1 GB | `Dockerfile.cesium` | `launch.sh` |
| YOPO 避障后端 | `mindcloud-yopo` | `pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime`（CUDA） | ≈35 GB | `Dockerfile.yopo` | `scripts/start_yopo_api.sh` |
| DA360 深度服务 | `mindcloud-da360` | `pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime`（CUDA） | ≈28 GB | `Dockerfile.da360` | `scripts/start_da360_api.sh` |

> 体积为 RTX 4070 Laptop / Ubuntu 24.04 上的实测值，仅供评估磁盘用。日常 `./restart_all.sh` **不会**重建镜像（只有镜像缺失或显式 `*_FORCE_BUILD=1` 才构建）。需要单独构建或强制重建时见「从零开始（首次部署）→ 第 4 步」。

### 主飞行进程（`Dockerfile.cesium`）

- 把整个项目 `COPY` 到容器内 `/var/www/google-tiles-flight`，`CMD node scripts/server.js` 启动 Express 静态服务（同时提供 `/api/path/*.json` 门路线路持久化 API），`EXPOSE 8000`。
- 由 `launch.sh` 触发构建：仅当镜像**不存在**或加了 `--rebuild` 时才 `docker build`，否则直接复用已有镜像。
- 运行时用只读卷挂载 `src/`、`index.html`，读写挂载 `asset/gate-paths`，因此**改前端 JS/HTML 无需重建镜像**——重启容器并在浏览器强刷（Ctrl+F5）即可生效。

### YOPO 避障后端（`Dockerfile.yopo`）

- 系统依赖：`libgl1-mesa-glx`、`libglib2.0-0`、`ca-certificates`；Python 依赖：`numpy<2`、`pillow`、`opencv-python-headless`、`scipy`、`flask`、`flask-cors`、`ruamel.yaml`、`websockets`，以及 TensorRT 相关的 `tensorrt==8.6.1.post1`、`onnx`。
- 镜像内直接拷入 `scripts/yopo_server.py` 与 `third_party/yopo/`（含权重）。
- `scripts/start_yopo_api.sh` 构建/运行要点：
  - 构建用 `--network=host`，让容器内 `127.0.0.1:7890` 能访问宿主机代理装 pip；并把宿主 `HTTP(S)/FTP/ALL/NO_PROXY` 转发为 build args（`YOPO_PIP_NO_PROXY=1` 可关闭）。
  - 触发重建：镜像不存在，或 `YOPO_FORCE_BUILD=1`；失败自动重试 `YOPO_BUILD_RETRIES`（默认 3）。
  - 运行时只读挂载模型权重、`yopo_server.py`、`third_party/yopo` 源码——**改 Python / 权重不用重建镜像**，重启容器即生效。
  - 端口 5689（HTTP）+ 5690（WebSocket）；默认 `--gpus all`，`YOPO_GPUS=none` 走 CPU。
  - 镜像内 TensorRT 固定为 `8.6.1`（兼容 CUDA 12.1，且匹配 `yopo_server` 的 TRT 8 加载 API）；TRT 8.6 pip 包不自带 cuDNN，复用镜像内 torch 捆绑的 cuDNN8 提供 `libcudnn.so.8`（见 `Dockerfile.yopo` 的 `LD_LIBRARY_PATH`）。TRT 推理加速见「YOPO TensorRT 加速」。

### DA360 深度服务（`Dockerfile.da360`）

- Python 依赖：`numpy<2`、`flask`、`flask-cors`、`opencv-python-headless`、`pillow`、`timm`、`tqdm`、`xformers`。
- `COPY third_party/DA360` 进镜像：DA360 **源码**已随仓库提供（`.gitignore` 与 `.dockerignore` 对 DA360 都只排除权重目录 `third_party/DA360/checkpoints/`，完整排除清单见仓库根目录 `.dockerignore`），构建时无需先拉源码；**权重** `checkpoints/DA360_large.pth` 不入镜像，由 `scripts/start_da360_api.sh` 在运行前下载（或挂载本地权重）后通过 `--model-path` 指定。
- `scripts/start_da360_api.sh` 构建/运行要点：
  - 构建网络与代理转发同 YOPO（`DA360_BUILD_NETWORK`；额外从 `git config` 探测宿主机代理 `DA360_BUILD_PROXY`）。
  - 会给镜像打 `mindcloud.da360.server_sha` label。跳过重建有两条路径：默认 `DA360_MOUNT_SERVER=1` 时**只要镜像存在就无条件跳过**（不比较 SHA，因为脚本会被只读挂载进去覆盖同名文件）；设 `DA360_MOUNT_SERVER=0` 后才改为比较 SHA——已有镜像且 server 脚本 SHA 一致时跳过，`DA360_FORCE_BUILD=1` 或脚本内容变化才重建。
  - 构建失败时**默认不启动过期镜像**（可用 `DA360_ALLOW_STALE_IMAGE=1` 放宽）。
  - 端口 5688；运行时只读挂载 `da360_server.py` 与模型权重。

### 构建超时 / 失败的常见原因

- 两个 CUDA 后端的基础镜像 `pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime` 体积很大，网络差时从 Docker Hub 拉取容易超时（`start_da360_api.sh` 失败提示中也直接点明了这一点）。
- 处理办法：脚本已内置 3 次重试，网络恢复后重跑即可；或保证代理可达（YOPO 侧 `Dockerfile.yopo` 内置默认走 `127.0.0.1:7890`；DA360 侧没有硬编码端口，只转发宿主的 `HTTP(S)/FTP/ALL/NO_PROXY` 并从 `git config` 探测）；也可把 `YOPO_BASE_IMAGE` / `DA360_BASE_IMAGE` 指向本地或镜像源镜像。
- 装 pip 依赖失败时，请确认以 `--network=host` 构建且代理可达。

### `.dockerignore`

构建上下文排除了 `.git`、`node_modules`、`__pycache__`、`*.pyc`、`scene/*`、`asset/gate-paths/*.tmp`、`third_party/DA360/checkpoints`（DA360 权重），避免把无关/大文件打进镜像；DA360 源码仍随镜像提供。

## DA360 深度估计

DA360 深度服务由 `restart_all.sh` 一键拉起（默认使用 `large` 模型）。DA360 **源码**已随仓库提供，但**权重**（`DA360_large.pth`，约 1.3GB，超 GitHub 100MB 限制）未入库，**首次运行深度服务前只需下载权重**（构建镜像无需先下载源码）：

```bash
python3 -m pip install --user gdown
./scripts/download_da360_model.sh
# 脚本会将权重放到 third_party/DA360/checkpoints/DA360_large.pth
```

启动后心跳自检：

```text
curl http://127.0.0.1:5688/health
```

停止或重启 DA360 直接重跑 `restart_all.sh`（或 `docker rm -fv mindcloud-da360-api`）。

注意，默认使用 `DA360_large`，`scripts/start_da360_api.sh` 以 `DA360_INPUT_SCALE=0.65` 启动其容器，模型输入约为 `672x336`（checkpoint 基准 1036×518 × 0.65；已在本机 RTX 4070 Laptop GPU 8GB 上验证可稳定运行；`da360_server.py` 自身的默认值是 `1.0`，即按 checkpoint 原分辨率推理）。

全景 RGB 默认就采集 `384x192` ERP，右下角显示即此原始尺寸；这个尺寸与 DA360 输出、YOPO 消费的尺寸完全一致，因此 `da360UploadScale` 默认为 `1.0`——原样上传、不再缩放，服务端 resize 到 `672x336` 模型输入，推理后把深度贴回 `384x192`。前端默认 `depthMs=33`（深度请求最小间隔 ≈30Hz），推理未完成时不会堆积请求。

默认不建议换模型；实验中 `DA360_large` 的 fast 档比 `DA360_small` 保留了更好的深度排序和边缘一致性。只有显存、功耗或部署体积受限时，再自行覆盖模型名：

```bash
DA360_MODEL=<large|base|small> ./scripts/download_da360_model.sh
DA360_MODEL=<large|base|small> ./scripts/start_da360_api.sh
```

如需主动调整 DA360 服务端模型输入尺寸，可设置推理 scale 或指定模型输入宽高；过低的 `DA360_INPUT_SCALE` 可能让 large 模型输出条带化深度，不建议低于 `0.46`。resize 采样方式在两处默认值不同：`da360_server.py` 自身默认 `bilinear`，而 `scripts/start_da360_api.sh` 默认用 `bicubic` 覆盖它并传入容器（即走一键启动时实际生效的是 `bicubic`）：

```bash
DA360_INPUT_SCALE=1.0 ./scripts/start_da360_api.sh
DA360_INPUT_SCALE=0.46 ./scripts/start_da360_api.sh
DA360_INPUT_WIDTH=476 ./scripts/start_da360_api.sh
DA360_INPUT_WIDTH=672 DA360_INPUT_HEIGHT=336 ./scripts/start_da360_api.sh
DA360_RESAMPLE=bilinear ./scripts/start_da360_api.sh
```

推理服务不在本机时：

```text
http://127.0.0.1:8080/?da360Url=http://<host>:5688/depth
```

## 使用流程说明

1. 点击 **Start Google 3D Tiles Flight**。
2. 等页面进入 **PLACEMENT MODE**。
3. 用 Cesium 搜索框搜索城市或地点。
4. 按住 `I` 并点击建筑、道路或地面设置出生点。
5. 用 `W/A/S/D` 微调水平位置，`Shift` 加快微调。
6. 设置 **SPAWN ALTITUDE (m)**。
7. 按 `O` 确认出生点。
8. 选择 **First Person** 或 **Third Person** 开始飞行。

常用按键：

```text
↑ / ↓       前进 / 后退
← / →       左右平移
W / S       上升 / 下降
A / D       左右偏航
Shift       加速
R           重置
V           切换视角
P           返回放置模式
Tab         设置面板
```

键盘可直接使用，也支持手柄（但需要自己优化映射），手柄通常会被 Chrome 的 Gamepad API 自动识别。RC 遥控器或 WebHID 设备可在设置面板中连接；如需检查 Linux 输入权限：

```bash
./launch.sh --input-status
./launch.sh --setup-input
```

## 全景相机实现原理

全景 RGB 默认从机头 360 相机位置采集，输出 `384x192` ERP 图。实现方式是对 Cesium/Google Tiles 渲染结果进行 6 个方向采样，然后在 GPU 中按 ERP 射线模型重投影：

```text
yaw   = pi - (u + 0.5) / W * 2pi
pitch = vfov / 2 - (v + 0.5) / H * vfov
```

这保证投影模型与 YOPO 原版的 ERP 相机一致；区别是数据来源为 Cesium 渲染视图，而不是仿真栅格的直接 raycast。放置阶段会后台创建全景采样 viewer；确认出生点后会在用户可控前预采样一张全景首帧。飞行中默认 `panoMs=12`、`panoFace=128`、每个采样方向等待 `panoFrameDelayMs=8`，并最多等待 `panoFaceTileTimeoutMs=140`（导航中 `panoFaceTileTimeoutMsFast=110`）让当前方向 tiles idle；首帧预加载使用 `panoPreloadFrameDelayMs=96`、`panoPreloadFaceTileTimeoutMs=6000` 和 `panoPreloadTimeoutMs=60000`，默认 `panoPreloadRequired=0`（允许首帧未集齐也进入飞行，实时采样继续补齐）。为了避免 Google Tiles 天空/极区采样在 ERP 顶部形成海市蜃楼状伪影，默认对顶部 10 度和底部 2 度做极区 guard；guard 区域保持 ERP 坐标，只向上下极点采样淡出，不会把整张图压缩到 guard 边界。可用 `panoTopPoleGuard` / `panoBottomPoleGuard` 调整或设为 0 关闭。

进入可控飞行前，主 Cesium 视图会预加载出生点周围区域，并分别等待第一人称和第三人称初始视角 tiles idle。默认 `flightPreloadStrict=0`，主视图只要目标区域覆盖率达标就继续；全景首帧预加载独立检查隐藏 viewer 的 6 个方向 tiles idle。默认 `panoPreloadRequired=0`：全景首帧未集齐也允许进入飞行，由实时采样继续补齐；如需强制首帧完成后再飞，设 `?panoPreloadRequired=1`。

常用参数：

```text
# 提升精度（默认已为 384×192；显存充裕可升到 672/896，带宽吃紧可降到 320）
http://127.0.0.1:8080/?panoWidth=896&panoFace=224

# 调整采样视图等待时间
http://127.0.0.1:8080/?panoFrameDelayMs=16&panoPreloadFrameDelayMs=120

# 调整首帧全景预加载超时；或允许首帧失败后继续进入飞行
http://127.0.0.1:8080/?panoPreloadTimeoutMs=90000&panoPreloadFaceTileTimeoutMs=9000
http://127.0.0.1:8080/?panoPreloadRequired=0

# 调整起飞前主视图预加载范围和覆盖率门槛
http://127.0.0.1:8080/?flightPreloadRadius=600&flightPreloadMinCoverage=0.98

# 调整 RGB / 深度更新间隔
http://127.0.0.1:8080/?panoMs=1000&depthMs=1200

# 调整 ERP 极区 guard
http://127.0.0.1:8080/?panoTopPoleGuard=0&panoBottomPoleGuard=0

# DA360 上传尺寸（默认 da360UploadScale=1.0，即 384×192 原样上传不缩放）
# 想减带宽 / 提实时性可下调（变为 192×96）；要更高精度应放大 panoWidth 而非上传缩放
http://127.0.0.1:8080/?da360UploadScale=0.5
http://127.0.0.1:8080/?da360UploadWidth=512
```

## YOPO 自主导航

基于 YOPO 端到端导航网络，无人机可自主飞行到指定目标点。YOPO 接收 ERP 全景深度图、里程计和目标点，输出位置/速度/加速度/偏航指令，通过 SimpleFlight 级联 PID 控制器驱动无人机。

### 导航架构（对齐 YOPO 原版）

- **网络输入**：`depth (1,2,192,384)`（通道 0 = 归一化深度，通道 1 = 有效 mask）+ 9 维观测（相机系速度/加速度/目标方向），经 `prepare_input` 展开为 `(1,9,6,12)`。
- **轨迹选择（服务端：纯 YOPO argmin）**：网络输出 72 条候选轨迹（12 水平 × 6 垂直锚点）的终端状态（PVA）+ score。**服务端严格遵循 YOPO 原版 `test_yopo_ros.py` 部署实现，直接 `argmin(score)` 选最优轨迹**，不叠加任何几何碰撞代价。避障的第一层完全由网络在训练期 `safety_loss` 中学到的 score 提供（**学习式避障**），与官方部署实现完全一致。
- **客户端反应式安全层（几何势场）**：在服务端轨迹之上，前端 `src/drone.js` 额外叠加一层基于 Cesium 射线环的几何反应式避障，用于兜住深度重规划（约 70 ms/次）间隙内的突发近障。路径畅通时该层自动完全归零、不干扰网络规划，详见「避障架构与调参」。
- **目标引导**：score 内已含目标方向代价（训练时 `wg=0.15`），网络原生指向目标。
- **3D 导航**：不做水平面投影，垂直避障由网络预测的 z 终端状态决定。
- **轨迹生成**：三轴五阶多项式（Poly5Solver），从上次指令状态出发（`plan_from_reference=True`），轨迹连续、无往复。
- **控制输出**：50Hz 评估多项式 → 位置/速度/加速度 + 偏航 → 前端级联 PID 跟踪。
- **终点接管（< 12m）**：两端各有一层。服务端 `FINAL_APPROACH_DIST=12.0` 内停止网络推理，直接规划一条"末端速度/加速度为零"的五阶多项式——近目标处 `argmin(score)` 会反复挑出过冲/折返轨迹，叠加 `plan_from_reference` 会让目标方向观测翻转、位置速度持续振荡而永远到不了。客户端同样在 `yopoFinalApproachDist=12.0`（按**3D 距离**判定，水平近但垂直远时不切换）内由 PD 接管收敛（位置环 + 速度/加速度前馈，带速度变化率上限避免抖动）。
- **深度可用性**：DA360 深度失败/超时时**不回退射线检测**，无人机原地悬停并持续重试，直到拿到有效深度图才恢复导航（详见「深度图」）。注：早期版本曾有"整帧被 2 m 内包围即判定深度异常并悬停"的检测，因在城市楼群中会把"近处像素多"误判为深度失效而频繁悬停，已按上游实现移除；深度有效性现交由 mask 通道与网络自身判断。
- **巡航速度地板（`yopoCruiseMinSpd=12`）**：路径畅通且目标较远时，沿目标方位补齐前进速度，避免网络把速度压到爬行；避障刹车时自动让位，距目标 < `yopoCruiseMinDist=5` m 时关闭，尊重接管/到达减速。
- **垂直优先直升降（`yopoVertFirst*`）**：当高度差占主导（水平距离 < 20 m 且 |Δh| > 5 m 且 > 1.2× 水平偏移）时，直接接管垂直通道做 P 收敛升降、水平只留 30%，消除大幅盘旋；正上/正下净空不足时让位回网络。
- **到达与死区**：锁定且距目标 < 3.5 m、速度 < 1 m/s 判定到达；进入 2.5 m 水平死区时改为直升降收敛高度，< 0.35 m 时 PD 停止校正仅做轻微高度收敛，避免围绕目标抖动。

### 避障架构与调参

避障分两层，职责不重叠：

| 层 | 位置 | 机制 | 作用 |
|----|------|------|------|
| 学习式避障 | 服务端 `scripts/yopo_server.py` | 网络 `argmin(score)` 选轨迹（训练期 `safety_loss`） | 全局路径规划、绕开大尺度结构 |
| 几何反应式势场 | 前端 `src/drone.js` | 360° 射线环（24 条、15° 间隔）实时探测 | 兜住深度重规划间隙内的突发近障 |

客户端几何层工作机制（见 `_avoidanceVelocity`）：

- **探测**：以机体为圆心发 24 条水平射线（半径 55 m，15° 间隔）；另对最对齐前进方向的 3 条射线做**上两层 + 下一层**（`high`/`high2`/`low`）共 3 层探测（供竖直越障判断）；另有正上/正下竖直射线。
- **输出分量**：`rep`（径向推离）/ `tan`（切向绕行）/ `brake`（近障刹车）/ `vRep`（竖直越障）/ `vGo`（竖直障碍足迹水平绕行）/ `upPush` + `vSafeDown`（地面与下降安全）。
- **刹车（射线层优先于网络）**：运动学硬刹车 `v_safe = √(2·a·(d − standoff))` 规划安全速度（`a` 用保守的 `yopoAvoidBrakeDecel≈7.5 m/s²` 留足余量）。触发刹车时：①**压制 YOPO 网络的加速度前馈**（否则网络轨迹加速度会正顶着障碍、与刹车减速相互抵消）；②沿当前速度反方向直接注入最强减速前馈（最高 `yopoAvoidBrakeAccel≈17.0 m/s²`，对应 60° 倾转上限 `droneMaxAngle=60`），且进入刹车即至少交付 `yopoAvoidBrakeMinFrac=0.85`（≈14.5 m/s²）让减速一踩就猛、够及时。威胁距离 `dAhead` 同时按"网络指令方向"与"无人机实际航向"取较小值，避免网络把指令拐向旁边就把正前方障碍排除、导致不刹车。
- **侧向速度预算**：绕行时把"前进"与"侧向绕行"拆开预算——侧向最多占速度上限的 68%、前向至少保留 10%，让速度矢量真正偏向切向、贴着障碍滑过，而不是"边全速前冲边轻蹭"。
- **畅通直飞（`goalClear`）**：分别沿"机体→目标"（`dPath`）和"命令速度方向"（`dCmd`）各算一次走廊，走廊半宽 2.5 m，**任一**走廊在 `reach = min(yopoAvoidRepRange, 到目标的水平距离)` 内无障即判定畅通（截断到目标距离是为了让"目标背后的墙"不会把走廊永久判为被挡）。近距例外：若距目标 < `yopoCorridorGuardDist`（12 m）且 `dPath` 被挡，则封掉 `dCmd` 这条逃生通道，避免贴着障碍直冲。**通道畅通时 `rep`/`tan`/`brake`/`vRep` 全部归零、`vGo` 被抑制**，无人机全速直飞目标，不会被无谓推离或莫名绕行。

关键参数（均位于 `src/drone.js` 构造函数）：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `yopoAvoidEnabled` | `true` | 几何层总开关 |
| `yopoAvoidRayCount` | 24 | 360° 射线数（15° 间隔） |
| `yopoAvoidRange` | 55.0 | 障碍探测半径 (m)，射线长度免费，加长只增不改 GPU 成本 |
| `yopoAvoidRepRange` | 28.0 | 排斥/切向/刹车作用距离 (m)；同时是 `goalClear` 的畅通判定阈值，**不要**调大 |
| `yopoAvoidRepRangeHi` | 50.0 | 高速档（≥ `yopoAvoidRefSpeed`）下上述作用距离 (m) |
| `yopoAvoidRepGain` | 20.0 | 径向推离最大速度 (m/s) |
| `yopoAvoidTanGain` | 54.0 | 切向绕行增益 (m/s)，越大绕得越果断 |
| `yopoTanConeCos` | 0.34 | 仅在"目标方位 ±70° 锥内"取障碍当绕行基准，避免被侧后方楼带偏 |
| `yopoTanAwayCos` | -0.2 | 旧切线偏离目标 >100° 即弃用，允许拐回目标 |
| `yopoTanAwayScale` | 0.5 | 切线偏离目标 >90° 时按 0.5 衰减，避免被推离目标 |
| `yopoAvoidDecel` | 8.5 | 竖直刹车阈值所用假定减速度 (m/s²) |
| `yopoAvoidBrakeDecel` | 7.5 | **水平刹车规划减速度 (m/s²)**：刻意低于可达值，给真实减速留 ~2× 余量 |
| `yopoAvoidBrakeAccel` | 17.0 | **刹车时允许的最大实际减速度 (m/s²)**：对应 60° 倾转上限（`droneMaxAngle=60`），直接注入反方向减速前馈并**压制网络加速度前馈** |
| `yopoAvoidBrakeMinFrac` | 0.85 | 进入刹车即至少交付 0.85×`BrakeAccel`（≈14.5 m/s²），一踩就猛 |
| `yopoAvoidBrakeReaction` | 0.32 / 0.48 | 刹车反应时间 (s)：基础 / 高速档（≥ `yopoAvoidRefSpeed`） |
| `yopoAvoidBrakeRange` / `BrakeRangeHi` | 18.0 / 32.0 | 渐进软刹车区间 (m)：低速 / 高速 |
| `yopoAvoidBrakeFloor` | 0.85 | 软刹车速度下限比例（接近时仍减速但不过度压缩巡航） |
| `yopoAvoidStop` | 6.0 | 与障碍保持的安全净距 (m)：驱动刹车 standoff、升降净空、vGo 触发 |
| `yopoAvoidVClimbScale` | 2.2 | 竖直越障爬升力度 |
| `yopoAvoidVBlock` | 20.0 | 前方净空低于此值触发竖直越障 (m) |
| `yopoAvoidVGoBase` / `VGoSpan` | 0.60 / 0.42 | 足下障碍"水平移出足迹"速度 (vGo) 的近/远强度 |
| `yopoAvoidVClear` | 0.38 | 上层"畅通"判定占比，越低越障意愿越强 |
| `yopoCorridorGuardDist` | 12.0 | 近距目标走廊守卫 (m)：此距离内即便速度方向走廊通畅，若目标方位走廊被挡也强制刹车 |
| `yopoCruiseMinSpd` | 12.0 | 巡航最小速度地板 (m/s)：路径畅通且目标较远时沿目标方位补齐前进速度，避障刹车时让位 |
| `yopoCruiseMinDist` | 5.0 | 距目标小于此值时关闭巡航地板，尊重接管/到达减速 |
| `yopoFinalApproachDist` | 12.0 | 终端接管区半径 (m)：区内由 PD 接管收敛到目标 |
| `yopoVertFirstEnabled` | `true` | 巡航阶段"垂直优先"直升降总开关 |
| `droneMaxVSpeed` | 15.0 | 竖直速度硬上限 (m/s) |
| `droneMaxAngle` | 60 | 最大倾转角 (°)：倾转物理上限 |

> **调参建议**：绕行不够果断请调大 `yopoAvoidTanGain`（力度）；**不要**调 `yopoAvoidRepRange`——它同时是 `goalClear` 的畅通判定阈值，调大会让"路径其实畅通"时误判被挡。改前端参数后需浏览器 **Ctrl+F5 强刷**生效。

#### 高速响应（射线预算与自适应作用距离）

飞得快时"避障来不及 + 深度/指令更新慢"的根因是同一次阻塞：完整的环形探测要发 35 条
`forceFresh` 的 `pickLocalRay`（24 条水平 + 3 层竖直共 9 条 + 正上/正下 2 条；每条都是一次完整
GPU 渲染 + 回读同步），且**同步跑在渲染帧循环里**。单次探测耗时可达数十至上百毫秒——既让避障数据一出炉就过期，又把帧率拖垮，进而连带拖慢
全景采集、DA360 深度与指令重规划。因此高速档改为给"每周期射线数"设预算，而不是靠拉长节流间隔
（那只会让最要命的正前方数据变得更旧）：

- **核心锥 `yopoAvoidCoreDeg`（±25°）**：任何速度下都保持 15° 全分辨率（24 条射线 → 360/24）、每周期必探——刹车距离由
  这一段决定，此处降采样会留下 30° 空隙（30 m 处约 15 m 盲区），是撞墙的直接原因。
- **外锥 `yopoAvoidConeDeg`（±55°，高速收窄到 ±45°）**：每周期必探，但高速时按
  `yopoAvoidStrideHi` 隔一条采样。
- **外围 `yopoAvoidSliceMax`**：每周期轮询 6 条，约 3 个周期（≈60 ms）刷完整圈；未在本周期探测的
  方向沿用上次测得值，保证排斥/绕行求和始终拿到完整 360° 环。
- **竖直层（high/high2/low）**：仅当上一周期判定"去往目标的走廊被挡"时才发射，省掉常态下 9 条射线。

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `yopoAvoidRange` | 55.0 | 障碍物探测半径 (m)，射线长度免费，加长只增不改 GPU 成本 |
| `yopoAvoidFastSpeed` | 6.0 | 高速档起始速度 (m/s) |
| `yopoAvoidRefSpeed` | 15.0 | 高速档完全生效的速度 (m/s) |
| `yopoAvoidStrideHi` | 2 | 高速时环形射线隔几条采样（2 → 30° 间隔） |
| `yopoAvoidCoreDeg` | 25 | 核心锥半角 (°)，始终保持 15° 全分辨率 |
| `yopoAvoidConeDeg` / `ConeDegHi` | 55 / 45 | 外锥半角 (°)，低速 / 高速 |
| `yopoAvoidSliceMax` | 6 | 每周期轮询的外围射线数 |
| `yopoAvoidRepRangeHi` | 50.0 | 高速时排斥/绕行/刹车的作用距离 (m) |
| `yopoAvoidTanGain` | 54.0 | 切向绕行增益 (m/s)，比 30 更果断 |
| `yopoAvoidRepGain` | 20.0 | 径向推离最大速度 (m/s)，比 12 更果断 |
| `yopoAvoidBrakeRangeHi` | 32.0 | 高速时软刹车起始距离 (m) |
| `yopoAvoidBrakeReaction` | 0.48 | 高速档刹车反应时间 (s)：姿态建立+控制环延迟，折算成反应距离 `spd×反应时间` 从可刹车距离中扣除，使 15 m/s 下提前约 3 m 开始减速、并在 standoff 内稳稳停住 |

实测每周期射线数：低速约 35（24 水平 + 9 竖直层 + 2 正上下），高速档按预算减少至约 12 水平 + 竖直层（被挡时更多）。`yopoAvoidRepRange`（= `goalClear`
的畅通阈值）**不随速度变化**，因此加大高速作用距离不会让"路径其实畅通"被误判为被挡。

浏览器控制台执行 `__yopoPerf()` 可查看实测指标（`fps` / `probeMsAvg` / `probeHz` / `depthHz` /
`cmdHz` / `ringAgeMaxMs`），用于判断瓶颈是否已解除。

### 启动 YOPO 后端

> YOPO 避障后端由 `restart_all.sh` 一键拉起（默认启用 TensorRT + `YOPO_VELOCITY=15`）；以下命令仅在需要手动单独构建/启动时使用。

```bash
# 首次需要构建 Docker 镜像
YOPO_FORCE_BUILD=1 ./scripts/start_yopo_api.sh

# 后续启动（自动跳过构建，挂载本地 yopo_server.py）
./scripts/start_yopo_api.sh

# 构建时如遇代理问题，确保本机 7890 端口代理可用
# Dockerfile.yopo 使用 --network=host + http://127.0.0.1:7890
```

服务运行在 `http://127.0.0.1:5689`。`yopo_server.py` 通过 Docker volume 挂载，修改后无需重建镜像。

### YOPO 后端关键环境变量

由 `scripts/start_yopo_api.sh` 转发进容器：

| 变量 | 默认值 / 推荐值 | 说明 |
|------|------------------|------|
| `YOPO_VELOCITY` | 15.0（`restart_all.sh` 设定） | 网络规划巡航速度 `vel_max` (m/s)，决定实际飞行速度；环境未设时回退到 yaml 配置值 |
| `YOPO_CTRL_TIME_SCALE` | 1.0 | 指令"快进"倍率。`>1` 会按 `vel_max × SCALE` 推进（2 即 ≈30 m/s），虽被 `YOPO_SPEED_CAP` 钳回，但会造成规划位置超前、无人机持续追迹滞后，故保持 1 |
| `YOPO_SPEED_CAP` | 15.0 | 指令速度绝对硬上限 (m/s)，保证"所有限速最高到 15 m/s" |
| `YOPO_TRAJ_EXTEND_S` | 2.0 | 轨迹末端外推时长 (s)，修重规划间隔内的指令冻结锯齿；超时仍无重规划则退回冻结行为，避免无限盲飞 |
| `YOPO_USE_TRT` | 1（`restart_all.sh` 设定） | TensorRT 加速开关，见「YOPO TensorRT 加速」 |

> 反应预算限速器（原按重规划间隔动态限速）已移除：飞行速度直接由 `YOPO_VELOCITY × YOPO_CTRL_TIME_SCALE` 决定，并受 `YOPO_SPEED_CAP` 硬钳制；避障改由「服务端网络 `argmin(score)` + 客户端几何反应式势场」两层共同保证（见「避障架构与调参」）。

### YOPO TensorRT 加速

YOPO 推理默认走 TensorRT（TRT）加速。将 `epoch50.pth` 固化为 fp16 引擎后，单次推理延迟从 PyTorch eager 的约 100~350ms 降到 1~5ms，使导航重规划更频繁、盲飞段更短、避障更顺。

- **引擎路径**：`asset/yopo-trt/yopo_trt.pth`（已提交；fp16，绑定本机 GPU 架构）。
- **一键启用**：`restart_all.sh` 无条件设 `YOPO_USE_TRT=1`，启动后后端日志打印 `[TensorRT] loaded … -- inference acceleration enabled`。注意该赋值在 `restart_all.sh` 里是硬编码的，因此 `YOPO_USE_TRT=0 ./restart_all.sh` **不会生效**；要强制退回 PyTorch eager 请直接调用后端脚本：
  ```bash
  YOPO_USE_TRT=0 ./scripts/start_yopo_api.sh
  ```
  （`scripts/start_yopo_api.sh` 只在 `YOPO_USE_TRT` 未被设置时自行检测引擎：存在则自动启用，不存在则回退 PyTorch eager。）
- **自动构建**：若启用 TRT 但引擎缺失，`scripts/start_yopo_api.sh` 会在 YOPO 容器内用 GPU 自动把当前模型（`YOPO_MODEL_PATH`，默认 `epoch50.pth`）固化为引擎并写入 `asset/yopo-trt/`（读写挂载），下次启动直接加载，无需手动预处理。
- **手动转换**：更换模型或重建引擎时，在带 GPU 的容器内运行转换脚本（`scripts/yopo_trt_transfer.py`：`epoch50.pth` → `torch.onnx.export`（`depth[1,2,192,384]` + `obs[1,9,6,12]`）→ TRT fp16 引擎，兼容 TRT 8.x / 10+）：
  ```bash
  docker run --rm --gpus all \
    -v "$PWD/third_party/yopo:/opt/mindcloud-yopo/third_party/yopo:ro" \
    -v "$PWD/scripts/yopo_trt_transfer.py:/opt/mindcloud-yopo/scripts/yopo_trt_transfer.py:ro" \
    -v "$PWD/third_party/yopo/saved/YOPO_40/epoch50.pth:/models/epoch50.pth:ro" \
    -v "$PWD/asset/yopo-trt:/opt/mindcloud-yopo/trt:rw" \
    mindcloud-yopo:latest python /opt/mindcloud-yopo/scripts/yopo_trt_transfer.py \
      --model /models/epoch50.pth --out /opt/mindcloud-yopo/trt/yopo_trt.pth
  ```
- **换 GPU / 架构**：TRT 引擎与 GPU 的 SM 计算能力绑定，当前引擎在 RTX 4070 上构建。部署到 Orin NX 或其它卡时，在目标机删除旧引擎并让 `start_yopo_api.sh` 自动重建（或重新跑上面命令）。
- **环境约束**：容器内 TensorRT 固定为 `8.6.1`（匹配 CUDA 12.1 运行时与 `yopo_server` 的 TRT 8 加载 API）；TRT 8.6 pip 包不自带 cuDNN，转而复用镜像内 torch 捆绑的 cuDNN8 提供 `libcudnn.so.8`（见 `Dockerfile.yopo` 的 `LD_LIBRARY_PATH`）。

### 目标点选择与导航

1. 飞行模式下，按 **`T`**（或点击右侧 YOPO 面板 **"Pick Target"**）开始设置目标。
2. 目标初始位置为无人机当前位置，用**数字键盘**移动（方向以**无人机当前机头朝向**为前方）：
   - `Numpad 8 / 2`：沿机头方向前进 / 后退
   - `Numpad 4 / 6`：垂直机头方向右移 / 左移（4 = 机身右侧、6 = 机身左侧，与小键盘的左右布局相反；见 `src/main.js` 的 `handleYOPOKeyDown`）
   - `Numpad 9 / 3`：上升 / 下降
3. **`Numpad 5`**：确认目标点并**自动开始导航**。
4. **`Numpad 0`** 或 **`Esc`**：取消选择。

导航期间：
- 无人机使用 YOPO 轨迹指令 + 速度前馈跟踪路径
- 推动摇杆临时切换人工控制（松杆恢复导航）
- **避障（服务端学习式 + 客户端几何反应式，双层）**：服务端严格遵循 YOPO，按
  `argmin(score)` 选轨迹（学习式避障）；客户端在跟踪指令的同时，叠加一层几何
  反应式势场（360° 射线环：径向推离、切向绕行、近障刹车、竖直越障、竖直障碍
  足迹绕行），用于兜住深度重规划间隙内的突发近障。去往目标的水平通道畅通时该
  几何层自动归零、不干扰导航，详见「避障架构与调参」。
- 到达判定分两层：服务端在距目标 2 m（`ARRIVE_THRESHOLD`）内标记到达；客户端另有 3.5 m + 速度 < 1 m/s 的兜底锁定，避免服务端异步回传导致"总差一步"
- 按 **`X`**（或点击 **"Stop Nav"**）结束导航

目标点标记在导航、到达后、停止后均保持可见，直到重新选取或取消目标，因此第二次导航时仍能看到目标位置。

### 俯视小地图（目标地图）

主界面左下角常驻一块 **Target Map (Top-Down)** 俯视小地图，飞行中实时刷新，用于直观掌握无人机与目标点的相对位置：

- 地图以无人机为中心，按水平面投影显示无人机当前朝向、目标点位置，以及两者连线。
- 地图下方两行文字分别给出**目标高度 y**（目标点在**局部坐标系**下的 y，即相对局部原点的高度，单位 m）和**坐标差 Δx/Δy/Δz to target**（目标相对无人机的东/上/北方向位移，单位 m）。
- 进入目标选择模式（按 `T`）后，地图会随数字键盘对目标点的移动同步更新，方便在空间上对齐目标。
- 该小地图不含任何数据归属水印，纯前端绘制，不依赖外部地图服务。

### 深度图

YOPO 需要 **384×192 ERP 全景深度图**（YOPO 原生输入格式），双通道：通道 0 = 归一化深度 [0,1]，通道 1 = 有效 mask。获取流程：

1. DA360 全景深度估计 → ERP 深度图（米制）
2. 前端重投影/裁剪为 384×192 ERP，附加有效 mask
3. 直接作为网络输入（深度值本身由 DA360 给出，不会掺入射线合成的几何深度；仅用 4 条稀疏 Cesium 射线做**米制尺度标定**，把 DA360 的相对深度换算成米）

**深度不可用时（DA360 失败/超时）不回退 Cesium 射线检测**——YOPO 的网络输入仍要求真实深度，此时无人机原地悬停并持续重试，直到拿到有效深度图才恢复导航。

> 这与客户端几何反应式避障互不冲突：后者（见「避障架构与调参」）是独立的安全兜底层，用 Cesium 射线环实时探测、不依赖 DA360 深度，也不参与网络输入。

### 坐标系

| 坐标系 | x | y | z | 前向 |
|--------|---|---|---|------|
| MindCloud / Cesium | 东 | 上 | 北 | -z |
| YOPO / ROS FLU | 前 | 左 | 上 | +x |

目标点在 MindCloud 坐标系下设置，服务端自动转换。
