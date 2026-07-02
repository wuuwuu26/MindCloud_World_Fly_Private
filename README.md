# Google 3D Tiles Flight

浏览器中的 Google Photorealistic 3D Tiles 穿越机驾驶器。进入页面后选择城市、放置出生点，然后用键盘、手柄或 RC 遥控器飞行。右下角可显示机头 360 ERP 全景 RGB 和 DA360 深度。

## 环境要求

- Docker Engine
- Chrome / Chromium
- 浏览器可以访问 Cesium Ion 和 Google 3D Tiles
- 本地开发模式需要 Python 3
- DA360 深度推理需要 NVIDIA GPU、NVIDIA Container Toolkit、Python 3 + pip，以及可访问模型下载地址的网络

## 启动

```bash
./launch.sh
```

打开：

```text
http://127.0.0.1:8080
```

常用方式：

```bash
# 端口被占用时
PORT=18081 ./launch.sh

# 只启动服务，不自动打开浏览器
./launch.sh --no-open

# Docker 后台运行
./launch.sh --detach

# 停止后台容器
docker rm -f google-tiles-flight

# 本地开发模式
./launch.sh --local
```

## DA360 深度

首次使用前下载模型并启动推理服务：

```bash
python3 -m pip install --user gdown
./scripts/download_da360_model.sh
DA360_DETACH=1 ./scripts/start_da360_api.sh
curl http://127.0.0.1:5688/health
```

默认使用 `DA360_small`。需要更高精度时：

```bash
DA360_MODEL=large ./scripts/download_da360_model.sh
DA360_MODEL=large DA360_DETACH=1 ./scripts/start_da360_api.sh
```

推理服务不在本机时：

```text
http://127.0.0.1:8080/?da360Url=http://<host>:5688/depth
```

停止推理服务：

```bash
docker rm -f mindcloud-da360-api
```

## 飞行流程

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

## 全景相机

全景 RGB 默认从机头 360 相机位置采集，输出 `672x336` ERP 图。实现方式是对 Cesium/Google Tiles 渲染结果进行 6 个方向采样，然后在 GPU 中按 ERP 射线模型重投影：

```text
yaw   = pi - (u + 0.5) / W * 2pi
pitch = vfov / 2 - (v + 0.5) / H * vfov
```

这保证投影模型与 YOPO_360 的 ERP 相机一致；区别是数据来源为 Cesium 渲染视图，而不是仿真栅格的直接 raycast。放置阶段会后台创建全景采样 viewer；确认出生点后会在用户可控前预采样一张全景首帧。飞行中默认每个采样方向等待 `panoFrameDelayMs=16`，优先提高移动时实时性；首帧预加载使用 `panoPreloadFrameDelayMs=120`，让隐藏 viewer 有时间拉取初始 tiles。

常用参数：

```text
# 更高输出分辨率
http://127.0.0.1:8080/?panoWidth=1036&panoFace=768

# 调整采样视图等待时间
http://127.0.0.1:8080/?panoFrameDelayMs=32&panoPreloadFrameDelayMs=160

# 调整首帧全景预加载超时
http://127.0.0.1:8080/?panoPreloadTimeoutMs=10000

# 调整 RGB / 深度更新间隔
http://127.0.0.1:8080/?panoMs=1000&depthMs=1200
```

## 常用 URL 参数

```text
# 初始视角
http://127.0.0.1:8080/?lon=114.1690321&lat=22.3246282&height=1800

# 自定义 Cesium Ion token / asset
http://127.0.0.1:8080/?ionToken=<your_token>&assetId=2275207
```

## 输入设备

键盘可直接使用。手柄通常会被 Chrome 的 Gamepad API 自动识别。RC 遥控器或 WebHID 设备可在设置面板中连接；如需检查 Linux 输入权限：

```bash
./launch.sh --input-status
./launch.sh --setup-input
```

## 排查

- 页面一直加载 Google Tiles：确认网络能访问 Cesium Ion 和 Google 3D Tiles，并检查 `ionToken` / `assetId`。
- 看不到全景或深度：确认右下角设置中 **Nose 360 Panorama Sensor** 已开启；深度还需要 DA360 服务健康。
- Docker 拉取或构建失败：网络恢复后重跑，或用 `DA360_BASE_IMAGE=<本地或镜像源中的pytorch镜像>` 指定基础镜像。
