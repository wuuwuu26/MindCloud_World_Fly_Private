#!/usr/bin/env python3
"""Flask API for DA360 panoramic depth inference."""

import argparse
import base64
import io
import os
import sys
import time
from pathlib import Path

try:
    import numpy as np
    from PIL import Image, ImageOps
except ImportError as exc:
    raise SystemExit(
        "Missing DA360 API dependencies. Install at least: "
        "pip install numpy pillow flask flask-cors torch torchvision opencv-python timm"
    ) from exc

try:
    from flask import Flask, jsonify, request
except ImportError as exc:
    raise SystemExit("Missing Flask. Install with: pip install flask flask-cors") from exc

try:
    from flask_cors import CORS
except ImportError:
    CORS = None

try:
    import torch
except ImportError as exc:
    raise SystemExit("Missing PyTorch. Install DA360 dependencies before starting this server.") from exc


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DA360_ROOT = Path(os.environ.get("DA360_ROOT", PROJECT_ROOT / "third_party" / "DA360")).resolve()
DEFAULT_MODEL_NAME = os.environ.get("DA360_MODEL", "small")
DEFAULT_MODEL = Path(os.environ.get(
    "DA360_MODEL_PATH",
    DA360_ROOT / "checkpoints" / f"DA360_{DEFAULT_MODEL_NAME}.pth",
))


def load_torch_checkpoint(path, device):
    try:
        return torch.load(path, map_location=device, weights_only=False)
    except TypeError:
        return torch.load(path, map_location=device)


def decode_data_url(data_url):
    if not data_url:
        raise ValueError("empty image")
    if "," in data_url:
        data_url = data_url.split(",", 1)[1]
    raw = base64.b64decode(data_url)
    image = Image.open(io.BytesIO(raw))
    image = ImageOps.exif_transpose(image).convert("RGB")
    return image


def image_to_tensor(image, width, height, device):
    image = image.resize((width, height), Image.Resampling.BICUBIC)
    arr = np.asarray(image, dtype=np.float32) / 255.0
    tensor = torch.from_numpy(arr).permute(2, 0, 1).unsqueeze(0).to(device)
    mean = torch.tensor([0.485, 0.456, 0.406], device=device).view(1, 3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225], device=device).view(1, 3, 1, 1)
    return (tensor - mean) / std


def depth_to_color(depth):
    depth = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0)
    if not np.any(valid):
        return np.zeros((*depth.shape, 3), dtype=np.uint8)

    near = np.percentile(depth[valid], 2.0)
    far = np.percentile(depth[valid], 98.0)
    if not np.isfinite(near) or not np.isfinite(far) or far <= near:
        near = float(depth[valid].min())
        far = float(depth[valid].max() + 1e-6)

    t = 1.0 - (np.clip(depth, near, far) - near) / max(far - near, 1e-6)
    t = np.clip(t, 0.0, 1.0)
    stops = np.array([
        [4, 3, 30],
        [20, 25, 210],
        [0, 210, 255],
        [92, 255, 120],
        [255, 238, 67],
        [255, 64, 43],
        [210, 38, 255],
    ], dtype=np.float32)
    scaled = t * (len(stops) - 1)
    lo = np.floor(scaled).astype(np.int32)
    hi = np.clip(lo + 1, 0, len(stops) - 1)
    frac = (scaled - lo)[..., None]
    color = stops[lo] * (1.0 - frac) + stops[hi] * frac
    color[~valid] = 0
    return color.astype(np.uint8)


def encode_png(image):
    out = io.BytesIO()
    Image.fromarray(image).save(out, format="PNG")
    return "data:image/png;base64," + base64.b64encode(out.getvalue()).decode("ascii")


class DA360Runner:
    def __init__(self, model_path):
        if not DA360_ROOT.is_dir():
            raise FileNotFoundError(f"DA360 repo is missing: {DA360_ROOT}")
        if not Path(model_path).is_file():
            raise FileNotFoundError(f"DA360 checkpoint is missing: {model_path}")

        sys.path.insert(0, str(DA360_ROOT))
        import networks  # pylint: disable=import-error,import-outside-toplevel

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        checkpoint = load_torch_checkpoint(model_path, self.device)
        checkpoint.setdefault("net", "DA360")
        checkpoint.setdefault("dinov2_encoder", "vits")
        checkpoint.setdefault("height", 518)
        checkpoint.setdefault("width", 1036)

        net_cls = getattr(networks, checkpoint["net"])
        self.model = net_cls(
            checkpoint["height"],
            checkpoint["width"],
            dinov2_encoder=checkpoint["dinov2_encoder"],
        ).to(self.device)
        model_state = self.model.state_dict()
        self.model.load_state_dict(
            {key: value for key, value in checkpoint.items() if key in model_state},
            strict=False,
        )
        self.model.eval()
        self.height = int(checkpoint["height"])
        self.width = int(checkpoint["width"])
        self.model_name = Path(model_path).stem

        if os.environ.get("DA360_NO_WARMUP") != "1":
            warmup = Image.new("RGB", (self.width, self.height), (0, 0, 0))
            self.infer(warmup)

    def infer(self, image):
        tensor = image_to_tensor(image, self.width, self.height, self.device)
        with torch.no_grad():
            if self.device.type == "cuda":
                with torch.cuda.amp.autocast():
                    outputs = self.model(tensor)
            else:
                outputs = self.model(tensor)
        disp = outputs["pred_disp"].detach().float().cpu().numpy()[0, 0]
        depth = 1.0 / np.maximum(disp, 1e-6)
        valid = np.isfinite(depth) & (depth > 0)
        if np.any(valid):
            depth = depth / max(float(depth[valid].min()), 1e-6)
        return depth


def create_app(runner):
    app = Flask(__name__)
    if CORS is not None:
        CORS(app, resources={r"/*": {"origins": "*"}})

    @app.after_request
    def add_cors_headers(response):
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type"
        return response

    @app.route("/health", methods=["GET"])
    def health():
        return jsonify({
            "ok": True,
            "model": runner.model_name,
            "device": str(runner.device),
            "width": runner.width,
            "height": runner.height,
        })

    @app.route("/depth", methods=["POST", "OPTIONS"])
    def depth():
        if request.method == "OPTIONS":
            return ("", 204)
        started = time.time()
        data = request.get_json(silent=True) or {}
        if "image" not in data:
            return jsonify({"error": "No image data received"}), 400

        try:
            image = decode_data_url(data["image"])
            pred_depth = runner.infer(image)
            colored = depth_to_color(pred_depth)
            return jsonify({
                "depth_image": encode_png(colored),
                "latency_ms": (time.time() - started) * 1000.0,
                "model": runner.model_name,
                "device": str(runner.device),
            })
        except Exception as exc:  # pylint: disable=broad-except
            print(f"[DA360] inference failed: {exc}", file=sys.stderr)
            return jsonify({"error": str(exc)}), 500

    return app


def parse_args():
    parser = argparse.ArgumentParser(description="Start the DA360 panoramic depth API.")
    parser.add_argument("--model-path", default=str(DEFAULT_MODEL), help="Path to DA360 .pth checkpoint.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=5688, type=int)
    parser.add_argument("--debug", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    runner = DA360Runner(args.model_path)
    app = create_app(runner)
    print(f"DA360 API running at http://127.0.0.1:{args.port}")
    print(f"Model: {args.model_path}")
    print(f"Device: {runner.device}")
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)


if __name__ == "__main__":
    main()
