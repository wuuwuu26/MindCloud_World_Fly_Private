# Google 3D Tiles Flight 用户手册

浏览器中的 Google Photorealistic 3D Tiles 穿越机驾驶器。启动本地服务后，在 Chrome / Chromium 里选择城市、放置出生点，然后用键盘、手柄或 RC 遥控器飞行。

## 环境要求

- Docker Engine
- Git
- Chrome / Chromium
- 浏览器可以访问 Cesium Ion 和 Google 3D Tiles
- 本地开发模式需要 Python 3
- 全景 RGB / DA360 深度需要 NVIDIA GPU、NVIDIA Container Toolkit、Python 3 + pip，以及可访问 Google Drive 的网络；默认下载 DA360 small 模型，large 模型约 1.3 GB

## 开始飞行

在项目目录执行：

```bash
./launch.sh
```

启动后打开：

```text
http://127.0.0.1:8080
```

如果脚本没有执行权限，先运行：

```bash
chmod +x launch.sh scripts/*.sh
```

常用启动方式：

```bash
# 端口被占用时
PORT=18081 ./launch.sh

# 只启动服务，不自动打开浏览器
./launch.sh --no-open

# Docker 后台运行
./launch.sh --detach

# 停止后台容器
docker rm -f google-tiles-flight

# 本地开发模式，不走 Docker
./launch.sh --local
```

## 全景 RGB / DA360 深度

第一次使用 DA360 前，先安装模型下载工具：

```bash
python3 -m pip install --user gdown
```

先启动 DA360 GPU 推理服务：

```bash
./scripts/download_da360_model.sh
DA360_DETACH=1 ./scripts/start_da360_api.sh
curl http://127.0.0.1:5688/health
```

默认使用 `DA360_small`，实时性优先。需要更高精度时可以切换模型：

```bash
DA360_MODEL=large ./scripts/download_da360_model.sh
DA360_MODEL=large DA360_DETACH=1 ./scripts/start_da360_api.sh
```

再启动飞行页面：

```bash
./launch.sh
```

进入飞行后，右下角会显示机头全景 RGB 和 DA360 深度。推理服务不在本机时：

```text
http://127.0.0.1:8080/?da360Url=http://<host>:5688/depth
```

右下角传感器默认用低分辨率隐藏渲染器采集全景，避免主飞行视角闪烁。需要调试频率和分辨率时：

```text
http://127.0.0.1:8080/?panoMs=1000&depthMs=1200&panoWidth=512&panoFace=128
```

停止 DA360 推理服务：

```bash
docker rm -f mindcloud-da360-api
```

从零重跑或模拟新用户 clone 后的首次运行：

```bash
docker rm -f google-tiles-flight mindcloud-da360-api 2>/dev/null || true
rm -rf third_party/DA360
./scripts/download_da360_model.sh
DA360_DETACH=1 ./scripts/start_da360_api.sh
curl http://127.0.0.1:5688/health
./launch.sh --no-open --detach
curl -I http://127.0.0.1:8080/
curl -I http://127.0.0.1:8080/ThirdParty/Cesium/Cesium.js
```

进入页面后的飞行流程：

1. 点 **Start Google 3D Tiles Flight**。
2. 等页面进入 **PLACEMENT MODE**。
3. 用 Cesium 搜索框搜索城市或地点，也可以用鼠标浏览场景。
4. 按住 `I` 并点击建筑、道路或地面设置出生点；普通点击 / 拖拽只用于移动视角。
5. 用 `W/A/S/D` 微调位置，按住 `Shift` 可以加快微调。
6. 在 **SPAWN ALTITUDE (m)** 设置出生高度。
7. 按 `O` 确认出生点。
8. 选择 **First Person** 或 **Third Person** 开始飞行。

需要固定初始视角或更换 Cesium Ion 资源时，可以在 URL 中加入参数：

```text
# 初始视角
http://127.0.0.1:8080/?lon=114.1690321&lat=22.3246282&height=1800

# 自定义 Cesium Ion token / asset
http://127.0.0.1:8080/?ionToken=<your_token>&assetId=2275207
```

## 输入设备

键盘可以直接使用，手柄和 RC 遥控器是可选设备。`launch.sh` 启动时会检查 `/dev/input/js*` 和 `/dev/hidraw*`，并在终端显示当前输入设备状态。

### 手柄

普通手柄通常会被 Chrome 通过 Gamepad API 自动识别。启动页面后如果没有识别到手柄，可以插上设备后刷新页面。

RC 遥控器或需要 WebHID 的设备，建议先检查权限：

```bash
./launch.sh --input-status
```

如果提示 HID 权限不足，运行一次：

```bash
./launch.sh --setup-input
```

然后重新插拔设备，再刷新页面。

WebHID 连接步骤：

1. 插上遥控器。
2. 打开页面后按 `Tab` 打开设置面板。
3. 如果设备被 Gamepad API 抢占，勾选 **Disable Gamepad API (use Chrome WebHID)**。
4. 点 **Connect HID**，在 Chrome 弹窗里选择设备。
5. 需要时点 **Calibrate...** 校准通道。

默认 AETR 映射：

```text
Axis 0   Roll
Axis 1   Pitch
Axis 2   Throttle
Axis 3   Yaw
Button 0 Arm toggle
```

### 键盘

共享按键：

```text
Space        ARM / DISARM
Shift        Boost
R            回到出生点
M            Drone / FPV 飞行模式
V            第一人称 / 第三人称视角切换
P            回到放置模式
Tab          打开设置面板
Esc          飞行中返回放置模式
```

`Drone (Easy)` 模式：

```text
↑ / ↓        前进 / 后退
← / →        左 / 右平移
W / S        上升 / 下降
A / D        左 / 右偏航
Q / E        相机俯仰角
```

`FPV (Manual)` 模式：

```text
↑ / ↓        向前 / 向后俯仰
← / →        左 / 右横滚
W / S        电机推力
A / D        左 / 右偏航
```

设置面板里的 **Easy Max Speed** 控制 Easy 模式水平速度，默认 18 m/s；按住 `Shift` 会临时加倍。**Easy W/S Vertical** 控制 Easy 模式升降速度。FPV 模式的 `W/S` 是电机推力，想往前飞需要同时压低机头；FPV 相机固定俯仰角在设置面板的 **FPV Cam Angle** 调整，不使用 `Q/E` 实时调节。

第三人称观察相机：

```text
鼠标左键拖动      环绕观察
鼠标右键拖动      环绕观察
滚轮              拉近 / 拉远
按住滚轮拖动      平移 / 调观察高度
```

## troubleshooting

检查服务是否正常：

```bash
curl -I http://127.0.0.1:8080/
curl -I http://127.0.0.1:8080/ThirdParty/Cesium/Cesium.js
```

查看输入设备状态：

```bash
./launch.sh --input-status
```

清掉容器和镜像后重建：

```bash
docker rm -f google-tiles-flight 2>/dev/null || true
docker rmi google-tiles-flight:latest 2>/dev/null || true
./launch.sh
```

常见问题：

- 页面一直加载 Google tiles：确认浏览器可以访问 Cesium Ion 和 Google 3D Tiles，并检查 `ionToken` / `assetId` 是否有效。
- 页面空白、按钮无反应或 WebGL 报错：用 Chrome / Chromium 打开 `http://127.0.0.1:8080`，不要用 `file://` 打开 `index.html`。浏览器会把 `localhost` 和 `127.0.0.1` 当作不同站点，历史缓存或权限状态可能不同。
- 端口冲突：用 `PORT=18081 ./launch.sh` 或 `./launch.sh --port 18081`。
- Chrome 看不到 HID：运行 `./launch.sh --setup-input` 后重新插拔设备。
- 碰撞偶尔穿墙：这是当前 Cesium 查询代理的限制。
