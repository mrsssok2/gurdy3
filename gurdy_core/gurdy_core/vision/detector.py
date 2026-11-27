"""YOLO-style ONNX detector runtime: letterbox, decode (YOLOv8/11 head), class-aware NMS. numpy + PIL only.

The ONNX file is the interface: the ROS node never imports the training library (Ultralytics is AGPL-3.0).
Output layout expected: (1, 4 + n_classes, N) with xywh centre boxes in input pixels (YOLOv8/YOLO11 export);
`decode_yolo` also accepts the transposed (1, N, 4 + n_classes) layout.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image

from .classes import CLASSES, names_for_outputs


@dataclass
class Detection:
    cls: int
    name: str
    score: float
    box: tuple  # x1, y1, x2, y2 in ORIGINAL image pixels


def letterbox(rgb: np.ndarray, hw: tuple, fill: int = 114):
    """Resize keeping aspect ratio, pad to (H, W). Returns (image, scale, (pad_x, pad_y))."""
    h0, w0 = rgb.shape[:2]
    H, W = hw
    s = min(H / h0, W / w0)
    nh, nw = int(round(h0 * s)), int(round(w0 * s))
    img = np.asarray(Image.fromarray(rgb).resize((nw, nh), Image.BILINEAR)) if (nh, nw) != (h0, w0) else rgb
    out = np.full((H, W, 3), fill, np.uint8)
    px, py = (W - nw) // 2, (H - nh) // 2
    out[py:py + nh, px:px + nw] = img
    return out, s, (px, py)


def preprocess(rgb: np.ndarray, hw: tuple):
    img, s, pad = letterbox(rgb, hw)
    x = img.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
    return np.ascontiguousarray(x), s, pad


def iou_xyxy(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """IoU matrix between (N,4) and (M,4) xyxy boxes."""
    a, b = np.asarray(a, float).reshape(-1, 4), np.asarray(b, float).reshape(-1, 4)
    lt = np.maximum(a[:, None, :2], b[None, :, :2])
    rb = np.minimum(a[:, None, 2:], b[None, :, 2:])
    inter = np.clip(rb - lt, 0, None).prod(-1)
    aa = (a[:, 2:] - a[:, :2]).clip(0).prod(-1)
    ab = (b[:, 2:] - b[:, :2]).clip(0).prod(-1)
    return inter / np.maximum(aa[:, None] + ab[None] - inter, 1e-9)


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thr: float) -> list:
    order = np.argsort(-scores)
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        order = rest[iou_xyxy(boxes[i:i + 1], boxes[rest])[0] <= iou_thr]
    return keep


def decode_yolo(out: np.ndarray, scale: float, pad: tuple, orig_hw: tuple, conf_thr=0.35, iou_thr=0.5,
                names=CLASSES) -> list:
    p = np.asarray(out)
    p = p[0] if p.ndim == 3 else p
    nc = len(names)
    if p.shape[0] == 4 + nc and p.shape[1] != 4 + nc:
        p = p.T                                   # (N, 4+nc)
    elif p.shape[1] != 4 + nc:
        raise ValueError(f"unexpected detector output shape {np.asarray(out).shape} for {nc} classes")
    cls_scores = p[:, 4:]
    cls = cls_scores.argmax(1)
    conf = cls_scores[np.arange(len(p)), cls]
    m = conf >= conf_thr
    p, cls, conf = p[m], cls[m], conf[m]
    if not len(p):
        return []
    xyxy = np.stack([p[:, 0] - p[:, 2] / 2, p[:, 1] - p[:, 3] / 2, p[:, 0] + p[:, 2] / 2, p[:, 1] + p[:, 3] / 2], 1)
    xyxy[:, [0, 2]] = (xyxy[:, [0, 2]] - pad[0]) / scale
    xyxy[:, [1, 3]] = (xyxy[:, [1, 3]] - pad[1]) / scale
    h, w = orig_hw
    xyxy[:, [0, 2]] = xyxy[:, [0, 2]].clip(0, w)
    xyxy[:, [1, 3]] = xyxy[:, [1, 3]].clip(0, h)
    off = cls[:, None] * 10000.0                  # class-aware NMS by coordinate offset
    keep = nms(xyxy + off, conf, iou_thr)
    return [Detection(int(cls[i]), names[int(cls[i])], float(conf[i]), tuple(float(v) for v in xyxy[i])) for i in keep]


class OnnxDetector:
    """providers='auto' -> CUDA if onnxruntime-gpu offers it, else CPU. `provider` tells which one is active."""

    def __init__(self, path: str, providers: str = "auto", conf=0.35, iou=0.5, names=None, threads: int = 2):
        """names=None: class list from the model's output width (7 classes -> CLASSES, 12 -> CLASSES_V3)."""
        import onnxruntime as ort
        avail = ort.get_available_providers()
        if providers == "auto":
            order = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in avail]
        else:
            order = [p for p in (providers,) if p in avail] + ["CPUExecutionProvider"]
        so = ort.SessionOptions()
        so.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(path, so, providers=order)
        self.provider = self.sess.get_providers()[0]
        inp = self.sess.get_inputs()[0]
        self.input_name = inp.name
        h, w = inp.shape[2], inp.shape[3]
        self.hw = (h if isinstance(h, int) else 256, w if isinstance(w, int) else 320)
        if names is None:
            oshape = self.sess.get_outputs()[0].shape            # (1, 4+nc, N)
            names = names_for_outputs(min(d for d in oshape[1:] if isinstance(d, int)))
        self.conf, self.iou, self.names = conf, iou, tuple(names)

    def __call__(self, rgb: np.ndarray) -> list:
        x, s, pad = preprocess(rgb, self.hw)
        out = self.sess.run(None, {self.input_name: x})[0]
        return decode_yolo(out, s, pad, rgb.shape[:2], self.conf, self.iou, self.names)
