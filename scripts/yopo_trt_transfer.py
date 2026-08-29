#!/usr/bin/env python3
"""Freeze the YOPO PyTorch weights into a TensorRT engine (fp16) that yopo_server.py's
_TrtYopoModel can deserialize directly.

Usage (on a GPU machine / container with torch + tensorrt + onnx):
    python scripts/yopo_trt_transfer.py \
        --model third_party/yopo/saved/YOPO_40/epoch50.pth \
        --out  asset/yopo-trt/yopo_trt.pth

Pipeline:
    1) torch.onnx.export turns YopoNetwork(depth, obs) into ONNX with
       inputs depth=[1, C, H, W] and obs=[1, 9], outputs endstate and score.
    2) The TensorRT builder parses the ONNX, builds an FP16 engine and serializes
       it to --out. (The --out file is really a serialized TensorRT engine, matching
       yopo_server's trt.Runtime.deserialize_cuda_engine; it keeps the .pth suffix
       only to stay consistent with the YOPO_TRT_PATH convention used by
       start_yopo_api.sh / yopo_server.py.)

Notes:
    - An engine is bound to the GPU architecture (SM) it was built on, so build it
      on the same GPU class you deploy to.
    - Inference only picks it up when YOPO_USE_TRT=1 and YOPO_TRT_PATH points to
      this file.
"""
import argparse
import os
import sys

import torch

# ── YOPO source path (kept in sync with yopo_server.py) ─────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
YOPO_DIR = os.path.join(SCRIPT_DIR, "..", "third_party", "yopo")
if os.path.isdir(YOPO_DIR):
    sys.path.insert(0, YOPO_DIR)

from config.config import cfg            # noqa: E402
from policy.yopo_network import YopoNetwork  # noqa: E402


def export_onnx(model, out_onnx, in_channels, height, width):
    model.eval()
    # Keep this identical to real inference: model and inputs live on cuda (the rotation
    # matrix inside state_transform is on cuda, and prepare_input requires obs on the
    # same device, otherwise matmul raises a device mismatch).
    model.to('cuda')
    dummy_depth = torch.zeros(1, in_channels, height, width, dtype=torch.float32, device='cuda')
    # obs mirrors real inference too: state_transform.prepare_input first expands it to
    # 4D so it can be concatenated with the 4D depth_feature along dim=1 (state_backbone
    # is an empty Sequential inside forward).
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
    print(f"[ONNX] export done: {out_onnx}  "
          f"(depth={tuple(dummy_depth.shape)}, obs={tuple(dummy_obs.shape)})")


def build_trt(out_onnx, out_engine, fp16=True, workspace_gb=2):
    """Parse the ONNX into a TensorRT engine and serialize it.

    Supports both APIs:
      - TensorRT < 10: needs an explicit EXPLICIT_BATCH flag plus the FP16 flag,
        and uses build_engine + serialize.
      - TensorRT >= 10: explicit batch is the default and FP16 switches to
        config.set_precision.
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
        raise RuntimeError("TensorRT failed to parse the ONNX")
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
        raise RuntimeError("TensorRT engine build returned empty (out of memory / unsupported op?)")
    with open(out_engine, "wb") as f:
        f.write(engine_bytes)
    print(f"[TRT] engine built: {out_engine}  (fp16={fp16}, tensorrt {trt.__version__})")


def main():
    ap = argparse.ArgumentParser(description="YOPO PyTorch -> TensorRT engine")
    ap.add_argument("--model",
                    default=os.path.join(YOPO_DIR, "saved", "YOPO_40", "epoch50.pth"))
    ap.add_argument("--out",
                    default=os.path.join(SCRIPT_DIR, "..", "asset", "yopo-trt", "yopo_trt.pth"))
    ap.add_argument("--onnx", default="",
                    help="Intermediate ONNX path (default: yopo_trt.onnx next to --out)")
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
    print("[done] TensorRT engine written ->", args.out)
    print("       yopo_server loads it via YOPO_USE_TRT=1 + YOPO_TRT_PATH.")


if __name__ == "__main__":
    main()
