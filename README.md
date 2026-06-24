# MindCloud World Fly 用户手册

这是一个浏览器里的 3DGS 无人机飞行模拟器。当前版本默认使用 **Chrome + NVIDIA GPU**，手柄走 Chrome 的 Gamepad API，RC 遥控器可以走 Chrome 原生 WebHID。旧的 Firefox 兼容路径已经删掉，不再需要 `python3-hid` 或 `python3-websockets`。

## 环境准备

先确认这几个东西在本机存在：

```bash
python3 --version
google-chrome --version || google-chrome-stable --version || chromium --version
nvidia-smi
```

如果缺 `python3`：

```bash
sudo apt update
sudo apt install python3
```

如果缺 Chrome，安装 Google Chrome 或 Chromium。项目没有 npm 构建步骤，也不需要 pip 包；PlayCanvas 和 JSZip 由页面从 CDN 加载，所以首次打开页面需要能访问网络。

如果要接 RC 遥控器 / HID 设备，第一次建议执行：

```bash
cd /home/dzp/projects/MindCloud_World_Fly
sudo bash setup_udev.sh
```

执行后重新插拔遥控器；如果脚本提示加入了 `plugdev` 组，注销重登一次。

## 启动

```bash
cd /home/dzp/projects/MindCloud_World_Fly
./launch.sh
```

正常会看到类似输出：

```text
Starting HTTP server on port 8080...
  HTTP server ready (pid ...)
Opening Chrome with NVIDIA GPU acceleration...

Simulator:     http://localhost:8080
Gamepad:       Chrome Gamepad API
RC HID:        Chrome WebHID (Settings / Tab -> Connect HID)
```

Chrome 会自动打开 `http://localhost:8080`。这个启动脚本会设置 NVIDIA PRIME 环境变量：

```bash
__NV_PRIME_RENDER_OFFLOAD=1
__GLX_VENDOR_LIBRARY_NAME=nvidia
__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
```

想确认 Chrome 是否跑在 NVIDIA 上，可以启动后另开终端：

```bash
nvidia-smi pmon -c 1 | grep -E 'chrome|Idx'
```

如果只看到一串被截断的 `...track-uuid=...`，也可能就是 Chrome 的 GPU 子进程。用 PID 反查完整命令：

```bash
ps -p <PID> -o pid,comm,args
```

## 场景文件

本机已经测试过这两个文件：

```text
scene/field_z-up.sog       112.4 MB, 11,800,059 points
scene/3dgs_nanjing.ply     726.7 MB, 3,072,608 points
```

页面不会自动列出 `scene/` 目录。打开页面后点击 **Choose File**，选择上面的 `.sog` / `.ply` 文件；也可以把文件直接拖进页面。

支持格式：

```text
.ply    标准 3D Gaussian Splatting 点云
.splat  splat 格式，会先转成 PLY 渲染
.sog    PlayCanvas SOG 压缩格式
```

## 第一次飞起来

1. 打开 `http://localhost:8080`。
2. 点 **Choose File**，选择 `scene/field_z-up.sog`。
3. 等页面显示 Filter 面板。这个场景正常会显示 `11,800,059 / 11,800,059 points kept`。
4. `field_z-up.sog` 选择 **Z-Up**，然后点 **Apply**。Apply 后会构建碰撞 octree，可能卡几十秒，等它进入 **PLACEMENT MODE**。
5. 在 Placement Mode 里用 `W/A/S/D` 移动出生点，`Q/E` 调高度，鼠标左键拖动观察。
6. 按 `Enter` 进入飞行。当前代码会自动进入 `ARMED` 状态，HUD 右侧会显示 `ARMED`。

`scene/3dgs_nanjing.ply` 也按同样流程加载；我测试时使用默认 **Z-Up**，过滤面板显示 `3,072,608 / 3,072,608 points kept`，按 Apply 后可进入飞行。

## 飞行键位

```text
W / S        油门上 / 下
A / D        偏航左 / 右
↑ / ↓        俯仰前 / 后
← / →        横滚左 / 右
Q / E        相机俯仰角
Space        ARM / DISARM 切换
R            回到出生点
M            Drone / FPV 模式切换
Shift        Boost
P            回到放置模式
Tab          打开设置面板
G            显示 / 隐藏赛道 gate
Esc          关闭设置面板；未打开设置时退出当前场景
```

飞行模式：

```text
Drone (Easy)   稳定模式，松杆后尽量悬停，适合看场景。
FPV (Manual)   手动 FPV 速率控制，需要持续输入，适合练穿越。
```

## 手柄和 RC 遥控器

普通手柄：插 USB 或蓝牙连接后，Chrome 会通过 Gamepad API 自动识别。打开设置面板 `Tab`，在 **Gamepad Status** 和 **Channel Monitor** 能看到输入。

RC 遥控器 / HID：

1. 插上遥控器。
2. 按 `Tab` 打开设置。
3. 如果设备被 Gamepad API 抢占，勾选 **Disable Gamepad API (use Chrome WebHID)**。
4. 点 **Connect HID**，在 Chrome 弹窗里选择遥控器。
5. 如需要，点 **Calibrate...** 校准通道。

如果弹窗里看不到设备，执行：

```bash
sudo bash setup_udev.sh
```

然后重新插拔遥控器。当前版本走 Chrome 原生 WebHID，不需要启动额外本地桥接服务，也不需要安装 `python3-hid` / `python3-websockets`。

默认通道按 AETR 使用：

```text
Axis 0   Roll
Axis 1   Pitch
Axis 2   Throttle
Axis 3   Yaw
Button 0 Arm toggle
```

可以在设置面板里重新 Assign、Invert、调 Dead Zone、Rate 和 Expo。

## 赛道功能

赛道是自己画的闭环 gate 路线。

```text
Tab -> Race Course -> Edit path...
```

编辑器里：

```text
鼠标左键      添加 / 选择 gate
拖动          移动 gate
Z / X         降低 / 升高 gate
Delete        删除选中 gate
Backspace     撤销最后一个 gate
Enter         接受，至少需要 3 个 gate
Esc           取消
```

保存后回到飞行，按 `G` 显示 gate。HUD 会出现 `FPV RACE`、当前 gate、lap time 和 best lap。赛道记录保存在：

```text
asset/gate-paths/<sceneName>_<fileSize>.json
```

这些 JSON 是本地个人记录，已被 `.gitignore` 忽略。

## 音频和 BGM

背景音乐目录：

```text
asset/music/init/     加载 / 过滤 / 放置阶段
asset/music/flight/   飞行阶段
```

放入 `.flac` / `.mp3` / `.ogg` / `.wav` / `.m4a` 后，开发服务器会自动通过目录列表发现。静态部署时更新 manifest：

```bash
python3 scripts/gen-bgm-manifests.py
```

临时关闭音频：

```text
http://localhost:8080/?nobgm=1     关闭 BGM
http://localhost:8080/?noaudio=1   关闭引擎声
```

两个参数可以一起用：

```text
http://localhost:8080/?nobgm=1&noaudio=1
```

## 常见问题

页面空白或控制台报 CDN 错误：确认能访问 `cdn.jsdelivr.net`。项目没有本地 npm vendor。

不要用 `file://` 打开 `index.html`：请用 `./launch.sh` 或 `python3 serve.py`，否则 ES module、SOG、路径保存 API 可能不工作。

Apply 后卡住：大场景会同步构建碰撞 octree。`field_z-up.sog` 有 1180 万点，Apply 后等几十秒是正常的。可以先调低 Distance 或提高 Opacity 再 Apply。

手柄没有反应：先按一下手柄按钮唤醒 Chrome Gamepad API；RC HID 则用 `Tab -> Connect HID`。

Chrome 没走 NVIDIA：先确认 `nvidia-smi` 正常；启动后用 `nvidia-smi | grep -i chrome` 或打开 `chrome://gpu` 查看。

端口占用：`launch.sh` 会自动杀掉本项目旧的 `serve.py`。如果想避开 8080，使用 `python3 serve.py 18080`。


## 项目结构

```text
index.html                 前端页面
launch.sh                  一键启动：HTTP server + Chrome/NVIDIA
serve.py                   threaded 本地 HTTP server + gate path API
setup_udev.sh              RC/HID 权限规则
scripts/gen-bgm-manifests.py
src/
  main.js                  场景加载、过滤、主循环
  controller.js            键盘、手柄、Chrome WebHID、设置面板
  drone.js                 飞行动力学和控制
  collision.js             octree 碰撞
  gates.js                 赛道 gate 和计时
  path-editor.js           gate 路线编辑器
  path-store.js            asset/gate-paths 持久化
  ply-parser.js            PLY 解析
  splat-parser.js          SPLAT 解析
  sog-parser.js            SOG 解析
asset/
  display/                 logo、demo 图
  gate-paths/              本地赛道 JSON，git 忽略
  music/                   引擎声和 BGM
scene/                     本地场景文件，git 忽略
```
