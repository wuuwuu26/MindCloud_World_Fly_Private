#!/usr/bin/env python3
"""将 YOPO PyTorch 权重固化为 TensorRT 引擎 (fp16), 供 yopo_server.py 的
_TrtYopoModel 直接反序列化加载。

用法 (在含 torch + tensorrt + onnx 的 GPU 机器 / 容器内执行):
    python scripts/yopo_trt_transfer.py \
        --model third_party/yopo/saved/YOPO_40/epoch50.pth \
        --out  asset/yopo-trt/yopo_trt.pth

流程:
    1) torch.onnx.export 把 YopoNetwork(depth, obs) 导出为 ONNX,
       输入 depth=[1, C, H, W]、obs=[1, 9]; 输出 endstate、score。
    2) TensorRT Builder 解析 ONNX, 以 FP16 构建引擎并序列化写到 --out。
       (--out 文件本质是 TensorRT 序列化引擎, 与 yopo_server 的
        trt.Runtime.deserialize_cuda_engine 对应; 命名沿用 .pth 仅为了与
        start_yopo_api.sh / yopo_server.py 里约定的 YOPO_TRT_PATH 一致。)

注意:
    - 引擎与构建时的 GPU 架构 (SM) 绑定, 需在与部署相同的 GPU 上构建。
    - 推理端必须 YOPO_USE_TRT=1 且 YOPO_TRT_PATH 指向本文件才会启用。
"""
import argparse
import os
import sys

import torch

# ── YOPO 源码路径 (与 yopo_server.py 保持一致) ──────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
YOPO_DIR = os.path.join(SCRIPT_DIR, "..", "third_party", "yopo")
if os.path.isdir(YOPO_DIR):
    sys.path.insert(0, YOPO_DIR)

from config.config import cfg            # noqa: E402
from policy.yopo_network import YopoNetwork  # noqa: E402


def export_onnx(model, out_onnx, in_channels, height, width):
    model.eval()
    # 与真实推理保持一致: 模型/输入都放 cuda (state_transform 内部旋转矩阵在 cuda,
    # prepare_input 要求 obs 同设备, 否则 matmul 报设备不匹配)。
    model.to('cuda')
    dummy_depth = torch.zeros(1, in_channels, height, width, dtype=torch.float32, device='cuda')
    # obs 与真实推理保持一致: 先经 state_transform.prepare_input 扩成 4D,
    # 才能和 4D 的 depth_feature 在 dim=1 拼接 (forward 里 state_backbone 为空 Sequential)。
    dummy_obs_raw = torch.zeros(1, 9, dtype=torch.float32, device='cuda')
    dummy_obs = model.state_transform.prepare_input(dummy_obs_raw)
    with torch.inference_mode():
        torch.onnx.export(
            model,
            (dummy_depth, dummy_obs),
            out_onnx,
            input_names=["depth", "obs"],
            output_names=["endstate", "score"],
            opset_version=17,
        )
    print(f"[ONNX] 导出完成: {out_onnx}  "
          f"(depth={tuple(dummy_depth.shape)}, obs={tuple(dummy_obs.shape)})")


def build_trt(out_onnx, out_engine, fp16=True, workspace_gb=2):
    """把 ONNX 解析为 TensorRT 引擎并序列化。

    兼容两种 API:
      - TensorRT < 10: 需显式 EXPLICIT_BATCH flag + FP16 flag, 用 build_engine + serialize。
      - TensorRT >= 10: 默认即为 explicit batch, FP16 改用 config.set_precision。
    """
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    trt_ver = tuple(int(x) for x in trt.__version__.split(".")[:2])
    if trt_ver < (10, 0):
        network = builder.create_network(
            1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    else:
        network = builder.create_network()
    parser = trt.OnnxParser(network, logger)
    with open(out_onnx, "rb") as f:
        blob = f.read()
    if not parser.parse(blob):
        for i in range(parser.num_errors):
            print("[ONNX parse error]", parser.get_error(i).desc(), file=sys.stderr)
        raise RuntimeError("TensorRT 解析 ONNX 失败")
    config = builder.create_builder_config()
    ws = int(workspace_gb * (1024 ** 3))
    if trt_ver < (10, 0):
        config.max_workspace_size = ws
        if fp16:
            config.set_flag(trt.BuilderFlag.FP16)
        engine = builder.build_engine(network, config)
        engine_bytes = engine.serialize() if engine is not None else None
    else:
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, ws)
        if fp16:
            config.set_precision(getattr(trt, "float16", trt.DataType.HALF))
        engine_bytes = builder.build_serialized_network(network, config)
    if engine_bytes is None:
        raise RuntimeError("TensorRT 引擎构建返回空 (显存/算子不支持?)")
    with open(out_engine, "wb") as f:
        f.write(engine_bytes)
    print(f"[TRT] 引擎构建完成: {out_engine}  (fp16={fp16}, tensorrt {trt.__version__})")


def main():
    ap = argparse.ArgumentParser(description="YOPO PyTorch -> TensorRT 引擎")
    ap.add_argument("--model",
                    default=os.path.join(YOPO_DIR, "saved", "YOPO_40", "epoch50.pth"))
    ap.add_argument("--out",
                    default=os.path.join(SCRIPT_DIR, "..", "asset", "yopo-trt", "yopo_trt.pth"))
    ap.add_argument("--onnx", default="",
                    help="中间 ONNX 路径 (默认: --out 同目录的 yopo_trt.onnx)")
    ap.add_argument("--fp16", action="store_true", default=True)
    ap.add_argument("--no-fp16", dest="fp16", action="store_false")
    ap.add_argument("--workspace-gb", type=int, default=2)
    ap.add_argument("--height", type=int, default=None)
    ap.add_argument("--width", type=int, default=None)
    ap.add_argument("--in-channels", type=int, default=None)
    args = ap.parse_args()

    in_channels = args.in_channels if args.in_channels else int(cfg["image_channels"])
    height = args.height if args.height else int(cfg["image_height"])
    width = args.width if args.width else int(cfg["image_width"])
    print(f"[cfg] in_channels={in_channels} height={height} width={width}")

    print(f"[load] {args.model}")
    state_dict = torch.load(args.model, map_location="cpu", weights_only=True)
    model = YopoNetwork()
    model.load_state_dict(state_dict)

    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    onnx_path = args.onnx or os.path.join(out_dir, "yopo_trt.onnx")
    export_onnx(model, onnx_path, in_channels, height, width)
    build_trt(onnx_path, args.out, fp16=args.fp16, workspace_gb=args.workspace_gb)
    print("[done] TensorRT 引擎已生成 ->", args.out)
    print("       交由 yopo_server 通过 YOPO_USE_TRT=1 + YOPO_TRT_PATH 加载。")


if __name__ == "__main__":
    main()
