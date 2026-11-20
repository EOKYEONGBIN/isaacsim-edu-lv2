"""The AI's eyes: YOLO (ONNX) on the fork camera, and "box in the image -> how far the pallet is".

Pure numpy, shared by tools/learn_paths.py (learning the distance table from the recorded boxes)
and the AI driver in Isaac Sim (onnxruntime comes from Isaac Sim's isaacsim.pip.onnx extension).

How far the pallet is, from its box alone (camera between the forks, looking straight ahead):
  * far away, the box's bottom edge -- where the pallet stands on the floor -- moves down the
    image as the pallet comes closer;
  * in the last ~0.5 m that edge is cut off by the image's bottom, but the box's top edge (the
    pallet's top deck, just below the camera) drops quickly instead.
Both relations are learned from the recordings: the recorder saved, for every image, the exact
pallet box and where the pallet really was.
"""

from __future__ import annotations

import glob
import os

import numpy as np

CLASSES = ["pallet", "rack_level2"]
PALLET = 0
FLOOR_MIN_BOTTOM = 0.53  # a pallet standing on the floor ends below the image centre (racked ones end above)
CLIPPED_BOTTOM = 0.97  # box bottom this low = cut off by the image's lower edge
TABLE_BINS = 16
MIN_CONF = 0.4


def target_pallet(boxes):
    """The pallet to pick up: on the floor and closest (lowest in the image).

    boxes: [(cls, cx, cy, w, h, conf)] normalized to the image (0..1). Returns one box or None.
    """
    floor = [b for b in boxes if b[0] == PALLET and b[2] + b[4] / 2 >= FLOOR_MIN_BOTTOM]
    if not floor:
        return None
    return max(floor, key=lambda b: (b[2] + b[4] / 2, b[3] * b[4]))


def _monotone_table(feature, ahead):
    """Median distance per feature bin, forced to shrink as the feature grows."""
    order = np.argsort(feature)
    f, a = np.asarray(feature)[order], np.asarray(ahead)[order]
    edges = np.unique(np.quantile(f, np.linspace(0, 1, TABLE_BINS + 1)))
    xs, ys = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (f >= lo) & (f <= hi)
        if m.sum() >= 3:
            xs.append(float(np.median(f[m])))
            ys.append(float(np.median(a[m])))
    ys = list(np.minimum.accumulate(ys))  # further down the image never means further away
    return [[round(x, 4) for x in xs], [round(y, 3) for y in ys]]


def learn_table(demo_dirs, pallet_ahead_fn):
    """Distance table from the recorded label files + true pallet positions.

    pallet_ahead_fn(x, y, yaw_deg, px, py) -> (ahead, side). Only the drive in, before lifting.
    Returns ({"far": [bottoms, aheads], "near": [tops, aheads]}, samples) or (None, 0).
    """
    import csv

    far_f, far_a, near_f, near_a = [], [], [], []
    for d in demo_dirs:
        rows = list(csv.DictReader(open(os.path.join(d, "steps.csv"), encoding="utf-8")))
        if not rows:
            continue
        z0 = float(rows[0]["pallet_z"])
        lifted = next((i for i, r in enumerate(rows) if float(r["pallet_z"]) > z0 + 0.05), len(rows) - 1)
        for r in rows[:lifted + 1]:
            if not r.get("image"):
                continue
            ahead, side = pallet_ahead_fn(float(r["x"]), float(r["y"]), float(r["yaw_deg"]),
                                          float(r["pallet_x"]), float(r["pallet_y"]))
            if not (0.5 < ahead < 8.0 and abs(side) < 1.5):
                continue
            label = os.path.join(d, "labels", os.path.splitext(os.path.basename(r["image"]))[0] + ".txt")
            if not os.path.exists(label):
                continue
            boxes = []
            for line in open(label, encoding="utf-8"):
                v = line.split()
                if len(v) == 5:
                    boxes.append((int(v[0]), *map(float, v[1:]), 1.0))
            b = target_pallet(boxes)
            if b is None:
                continue
            bottom, top = b[2] + b[4] / 2, b[2] - b[4] / 2
            if bottom < CLIPPED_BOTTOM:
                far_f.append(bottom)
                far_a.append(ahead)
            else:
                near_f.append(top)
                near_a.append(ahead)
    if len(far_f) < 20 or len(near_f) < 10:
        return None, len(far_f) + len(near_f)
    return {"far": _monotone_table(far_f, far_a), "near": _monotone_table(near_f, near_a)}, len(far_f) + len(near_f)


def estimate_ahead(box, table):
    """Metres from the forklift to the pallet centre, from the pallet's box; None if out of the table."""
    if box is None or not table:
        return None
    bottom, top = box[2] + box[4] / 2, box[2] - box[4] / 2
    xs, ys = table["far"] if bottom < CLIPPED_BOTTOM else table["near"]
    v = bottom if bottom < CLIPPED_BOTTOM else top
    if len(xs) < 2 or v < xs[0] - 0.05 or v > xs[-1] + 0.05:
        return None
    return float(np.interp(v, xs, ys))


def newest_model(runs_dir):
    """The most recently exported YOLO model (runs/<name>/weights/best.onnx), or None."""
    found = glob.glob(os.path.join(str(runs_dir), "*", "weights", "best.onnx"))
    return max(found, key=os.path.getmtime) if found else None


class Detector:
    """YOLO (Ultralytics export, 640x640 input, one output [1, 4 + classes, anchors]) with onnxruntime."""

    SIZE = 640

    def __init__(self, model_path):
        import onnxruntime as ort  # from Isaac Sim's isaacsim.pip.onnx (or any onnxruntime install)

        available = ort.get_available_providers()
        providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in available]
        try:
            self.session = ort.InferenceSession(model_path, providers=providers)
        except Exception:  # noqa: BLE001 - CUDA libraries missing: run on the CPU
            self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0].name
        self.provider = self.session.get_providers()[0]
        self.path = model_path

    def detect(self, rgb, conf=MIN_CONF, iou=0.6):
        """rgb: HxWx3 uint8. Returns [(cls, cx, cy, w, h, conf)] normalized to the image."""
        h, w = rgb.shape[:2]
        s = self.SIZE / max(h, w)
        nh, nw = int(round(h * s)), int(round(w * s))
        img = rgb if (nh, nw) == (h, w) else _resize(rgb, nh, nw)
        top, left = (self.SIZE - nh) // 2, (self.SIZE - nw) // 2
        canvas = np.full((self.SIZE, self.SIZE, 3), 114, dtype=np.uint8)  # Ultralytics letterbox grey
        canvas[top:top + nh, left:left + nw] = img
        x = canvas.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.session.run(None, {self.input: x})[0][0]  # [4 + classes, anchors]
        scores = out[4:]
        cls = scores.argmax(0)
        best = scores[cls, np.arange(scores.shape[1])]
        keep = best >= conf
        if not keep.any():
            return []
        bx, cls, best = out[:4, keep].T, cls[keep], best[keep]
        result = []
        for c in np.unique(cls):
            m = cls == c
            for i in _nms(bx[m], best[m], iou):
                cx, cy, bw, bh = bx[m][i]
                result.append((int(c), float((cx - left) / s / w), float((cy - top) / s / h),
                               float(bw / s / w), float(bh / s / h), float(best[m][i])))
        return result


def _resize(img, nh, nw):
    ys = (np.arange(nh) * img.shape[0] / nh).astype(int)
    xs = (np.arange(nw) * img.shape[1] / nw).astype(int)
    return img[ys][:, xs]


def _nms(boxes, scores, iou):
    x0, y0 = boxes[:, 0] - boxes[:, 2] / 2, boxes[:, 1] - boxes[:, 3] / 2
    x1, y1 = boxes[:, 0] + boxes[:, 2] / 2, boxes[:, 1] + boxes[:, 3] / 2
    area = (x1 - x0) * (y1 - y0)
    order, keep = list(np.argsort(-scores)), []
    while order:
        i = order.pop(0)
        keep.append(i)
        rest = np.array(order, dtype=int)
        if not len(rest):
            break
        w = np.clip(np.minimum(x1[i], x1[rest]) - np.maximum(x0[i], x0[rest]), 0, None)
        h = np.clip(np.minimum(y1[i], y1[rest]) - np.maximum(y0[i], y0[rest]), 0, None)
        overlap = w * h / (area[i] + area[rest] - w * h + 1e-9)
        order = [j for j, o in zip(order, overlap) if o < iou]
    return keep
