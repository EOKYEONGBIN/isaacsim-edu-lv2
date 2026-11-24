"""AI의 눈: 포크 카메라 영상에 YOLO(ONNX)를 돌리고, 이미지 속 박스로 파렛트까지 거리를 추정한다.

순수 numpy. tools/learn_paths.py(녹화된 박스로 거리 표 학습)와 Isaac Sim 안의 AI 운전자가 함께 쓴다
(onnxruntime은 Isaac Sim의 isaacsim.pip.onnx 익스텐션에서 가져옴).

박스만으로 거리 추정 (카메라는 포크 사이에서 정면을 봄):
  * 멀리 있을 때: 박스 아래 변(파렛트가 바닥에 닿는 곳)이 가까워질수록 이미지 아래로 내려온다.
  * 마지막 ~0.5 m: 아래 변은 이미지 밖으로 잘리지만, 위 변(카메라 바로 아래의 파렛트 상판)이 빠르게 내려온다.
두 관계 모두 녹화 데이터(이미지마다 정확한 파렛트 박스와 실제 파렛트 위치)로 학습한다.
"""

from __future__ import annotations

import glob
import os

import numpy as np

CLASSES = ["pallet", "rack_level2"]
PALLET = 0
FLOOR_MIN_BOTTOM = 0.53  # 바닥의 파렛트는 이미지 중앙보다 아래에서 끝남 (랙 위 파렛트는 위에서 끝남)
CLIPPED_BOTTOM = 0.97  # 박스 아래 변이 이보다 낮으면 이미지 아래 가장자리에 잘린 것
TABLE_BINS = 16
MIN_CONF = 0.4


def target_pallet(boxes):
    """집어 올릴 파렛트: 바닥에 있고 가장 가까운 것(이미지에서 가장 아래).

    boxes: 이미지 기준 정규화(0..1)된 [(cls, cx, cy, w, h, conf)]. 박스 하나 또는 None 반환.
    """
    floor = [b for b in boxes if b[0] == PALLET and b[2] + b[4] / 2 >= FLOOR_MIN_BOTTOM]
    if not floor:
        return None
    return max(floor, key=lambda b: (b[2] + b[4] / 2, b[3] * b[4]))


def _monotone_table(feature, ahead):
    """feature 구간별 거리 중앙값. feature가 커질수록 거리가 줄어들도록 강제한다."""
    order = np.argsort(feature)
    f, a = np.asarray(feature)[order], np.asarray(ahead)[order]
    edges = np.unique(np.quantile(f, np.linspace(0, 1, TABLE_BINS + 1)))
    xs, ys = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (f >= lo) & (f <= hi)
        if m.sum() >= 3:
            xs.append(float(np.median(f[m])))
            ys.append(float(np.median(a[m])))
    ys = list(np.minimum.accumulate(ys))  # 이미지에서 더 아래인데 더 멀 수는 없음
    return [[round(x, 4) for x in xs], [round(y, 3) for y in ys]]


def learn_table(demo_dirs, pallet_ahead_fn):
    """녹화된 라벨 파일과 실제 파렛트 위치로 거리 표를 만든다.

    pallet_ahead_fn(x, y, yaw_deg, px, py) -> (ahead, side). 들어올리기 전 진입 구간만 사용.
    ({"far": [bottoms, aheads], "near": [tops, aheads]}, samples) 또는 (None, 0) 반환.
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
    """파렛트 박스로 추정한 지게차~파렛트 중심 거리 [m]. 표 범위 밖이면 None."""
    if box is None or not table:
        return None
    bottom, top = box[2] + box[4] / 2, box[2] - box[4] / 2
    xs, ys = table["far"] if bottom < CLIPPED_BOTTOM else table["near"]
    v = bottom if bottom < CLIPPED_BOTTOM else top
    if len(xs) < 2 or v < xs[0] - 0.05 or v > xs[-1] + 0.05:
        return None
    return float(np.interp(v, xs, ys))


def newest_model(runs_dir):
    """가장 최근에 내보낸 YOLO 모델(runs/<name>/weights/best.onnx). 없으면 None."""
    found = glob.glob(os.path.join(str(runs_dir), "*", "weights", "best.onnx"))
    return max(found, key=os.path.getmtime) if found else None


class Detector:
    """onnxruntime으로 돌리는 YOLO (Ultralytics export, 입력 640x640, 출력 하나 [1, 4 + classes, anchors])."""

    SIZE = 640

    def __init__(self, model_path):
        import onnxruntime as ort  # Isaac Sim의 isaacsim.pip.onnx (또는 별도 설치한 onnxruntime)

        available = ort.get_available_providers()
        providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in available]
        try:
            self.session = ort.InferenceSession(model_path, providers=providers)
        except Exception:  # noqa: BLE001 - CUDA 라이브러리가 없으면 CPU로 실행
            self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0].name
        self.provider = self.session.get_providers()[0]
        self.path = model_path

    def detect(self, rgb, conf=MIN_CONF, iou=0.6):
        """rgb: HxWx3 uint8. 이미지 기준 정규화된 [(cls, cx, cy, w, h, conf)] 반환."""
        h, w = rgb.shape[:2]
        s = self.SIZE / max(h, w)
        nh, nw = int(round(h * s)), int(round(w * s))
        img = rgb if (nh, nw) == (h, w) else _resize(rgb, nh, nw)
        top, left = (self.SIZE - nh) // 2, (self.SIZE - nw) // 2
        canvas = np.full((self.SIZE, self.SIZE, 3), 114, dtype=np.uint8)  # Ultralytics letterbox 회색
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
