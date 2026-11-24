"""모방 학습용으로 사람의 지게차 시연을 녹화합니다.

샘플마다(`STATE_HZ`) 눌린 조작 키, 지게차 상태(위치·자세, 속도, 조향, 포크 높이), 파렛트 자세를 기록합니다.
`IMAGE_EVERY` 샘플마다 카메라 이미지와 파렛트·목표 랙 단의 YOLO 형식 박스도 저장합니다.
박스는 `Replicator`의 tight 2D bbox에서 가져오므로 사람이 라벨링할 필요가 없습니다.

출력: <DEMOS_DIR>/demo_<time>/ steps.csv, meta.json, images/*.png, labels/*.txt
"""

from __future__ import annotations

import csv
import json
import math
import os
import time
from pathlib import Path

import carb
import numpy as np
import omni.physx
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, Usd, UsdGeom

import Isaacsim_Edu_Lv2_Lesson1.forklift_omnigraph_nodes as fon

ROOT = Path(__file__).resolve().parents[4]
DEMOS_DIR = ROOT / "forklift_ai" / "demos"

PALLET_PATH = "/Warehouse/Shelf_1/SM_PaletteA_357"
PALLET_BOXES_PATH = "/Warehouse/Shelf_1/CardBoxSet1_01"  # 파렛트 위 박스 (패널이 파렛트와 함께 옮김)
SHELF_PATH = "/Warehouse/Shelf_1/SM_RackShelf_156"
CLASSES = ["pallet", "rack_level2"]  # YOLO 클래스 id 0, 1
LEVEL2_TOP_Z = (1.2, 1.45)  # 윗면이 이 높이(m)인 랙 선반을 '2단'으로 봄

# AI 카메라: 포크 사이에서 캐리지와 함께 오르내림 (운전석 시야는 마스트와 계기판에 가림).
# 포크와 똑같이 움직이는 리프트 마스트 단의 자식으로 둠.
AI_CAMERA_PATH = f"{fon.MAST_LIFT_PATHS[0]}/AICamera"
AI_CAMERA_OFFSET = (0.0, -0.28, 0.09)  # 마스트 단 좌표계 기준: 캐리지 바로 앞, 정지 시 약 0.3 m 높이
AI_CAMERA_FOCAL_MM = 12.0  # 수평 화각 약 82°

STATE_HZ = 15.0
IMAGE_EVERY = 3  # -> 초당 이미지 5장
IMAGE_SIZE = (640, 480)
MIN_BOX_PX = 4  # 너무 얇은 박스는 무시

KEYS = [
    ("forward", fon.KEY_FORWARD_PATH),
    ("backward", fon.KEY_BACKWARD_PATH),
    ("left", fon.KEY_LEFT_PATH),
    ("right", fon.KEY_RIGHT_PATH),
    ("lift_up", fon.KEY_LIFT_UP_PATH),
    ("lift_down", fon.KEY_LIFT_DOWN_PATH),
]
COLUMNS = (["t"] + [k for k, _ in KEYS]
           + ["x", "y", "yaw_deg", "speed_mps", "steer_deg", "lift_m",
              "pallet_x", "pallet_y", "pallet_z", "pallet_yaw_deg", "image"])


def ensure_semantic_labels(stage):
    """bbox annotator용 클래스 라벨을 session layer에 붙입니다 (씬 파일은 그대로).

    모든 파렛트는 "pallet", 모든 2단 랙 선반은 "rack_level2"로 라벨링합니다.
    똑같이 생긴 물체는 라벨도 같아야 검출기가 모순된 예시를 받지 않습니다.
    """
    from isaacsim.core.experimental.utils.semantics import add_labels

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    labeled = {c: 0 for c in CLASSES}
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/Warehouse")):
            name = prim.GetName()
            if name.startswith("SM_PaletteA"):
                add_labels(prim, labels=["pallet"], taxonomy="class")
                labeled["pallet"] += 1
            elif name.startswith("SM_RackShelf"):
                top = cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMax()[2]
                if LEVEL2_TOP_Z[0] <= top <= LEVEL2_TOP_Z[1]:
                    add_labels(prim, labels=["rack_level2"], taxonomy="class")
                    labeled["rack_level2"] += 1
    return labeled


def ensure_ai_camera(stage):
    """포크 사이 카메라를 session layer에 만들고 경로를 반환합니다."""
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        cam = UsdGeom.Camera.Define(stage, AI_CAMERA_PATH)
        cam.CreateFocalLengthAttr(AI_CAMERA_FOCAL_MM)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
        xf = UsdGeom.Xformable(cam)
        xf.ClearXformOpOrder()
        # 카메라는 -Z를 보고 +Y가 위, 지게차는 로컬 -Y로 전진하고 +Z가 위
        m = Gf.Matrix4d(-1, 0, 0, 0,
                        0, 0, 1, 0,
                        0, 1, 0, 0,
                        AI_CAMERA_OFFSET[0], AI_CAMERA_OFFSET[1], AI_CAMERA_OFFSET[2], 1)
        xf.AddTransformOp().Set(m)
    return AI_CAMERA_PATH


def pallet_pose(stage):
    """파렛트의 (x, y, z, yaw_deg). 시뮬레이션 중에는 물리 자세, 아니면 USD 값."""
    try:
        res = omni.physx.get_physx_interface().get_rigidbody_transformation(PALLET_PATH)
        if res and res.get("ret_val"):
            p, q = res["position"], res["rotation"]  # q = (x, y, z, w)
            yaw = math.degrees(math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)))
            return float(p[0]), float(p[1]), float(p[2]), yaw
    except Exception:  # noqa: BLE001 - 실패하면 아래 USD 값 사용
        pass
    prim = stage.GetPrimAtPath(PALLET_PATH)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    x_axis = m.TransformDir((1.0, 0.0, 0.0))
    return float(t[0]), float(t[1]), float(t[2]), math.degrees(math.atan2(x_axis[1], x_axis[0]))


def pallet_tilt_deg(stage):
    """파렛트가 수평에서 기울어진 각도 [deg]. 시뮬레이션 중에는 물리 자세, 아니면 USD 값."""
    try:
        res = omni.physx.get_physx_interface().get_rigidbody_transformation(PALLET_PATH)
        if res and res.get("ret_val"):
            q = res["rotation"]  # (x, y, z, w)
            up_z = 1.0 - 2.0 * (q[0] ** 2 + q[1] ** 2)  # 파렛트 자체 위쪽 축의 월드 z 성분
            return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    except Exception:  # noqa: BLE001 - 실패하면 아래 USD 값 사용
        pass
    m = UsdGeom.Xformable(stage.GetPrimAtPath(PALLET_PATH)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    up = m.TransformDir((0.0, 0.0, 1.0)).GetNormalized()
    return math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))


def shelf_footprint(stage):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(SHELF_PATH)).ComputeAlignedRange()
    return [list(r.GetMin()), list(r.GetMax())]


def forklift_state(stage):
    prim = stage.GetPrimAtPath(fon.FORKLIFT_PATH)
    pos = prim.GetAttribute("xformOp:translate").Get()
    yaw = prim.GetAttribute("xformOp:rotateXYZ").Get()[2]
    return {
        "x": float(pos[0]), "y": float(pos[1]), "yaw_deg": float(yaw),
        "speed_mps": float(fon._current_speed_mps),
        "steer_deg": float(fon._front_wheel_steer_deg(stage)),
        "lift_m": float(fon._lift_offset(stage)),
    }


# 포크 끝은 지게차 원점에서 약 2.45 m 앞, 파렛트 깊이는 약 1 m.
# 지게차와 파렛트 중심 거리가 이보다 크면 포크가 파렛트에서 완전히 빠진 상태.
FORKS_CLEAR_M = 2.95


def placed_on_shelf(pallet, footprint, margin=-0.15):
    """파렛트가 목표 랙 단 위에 놓였는지 (파렛트 원점은 바닥면). 기본 margin은 파렛트 중심이
    선반 가장자리보다 15 cm 이상 안쪽이어야 한다는 뜻으로, 앞 빔에 걸친 상태를 걸러 냅니다."""
    (x0, y0, _), (x1, y1, z1) = footprint
    x, y, z, _ = pallet
    return x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin and abs(z - z1) < 0.12


class DemoRecorder:
    def __init__(self):
        self.active = False
        self.dir = None
        self._rows = []
        self._t_last = None
        self._sample = 0
        self._images = 0
        self._boxes = {c: 0 for c in CLASSES}
        self._rp = None
        self._rgb = None
        self._bbox = None
        self._start = None
        self._wall0 = None

    # ---------------------------------------------------------------- 시작 / 종료

    def start(self, stage):
        self._labeled = ensure_semantic_labels(stage)
        camera = ensure_ai_camera(stage)
        self.dir = DEMOS_DIR / time.strftime("demo_%Y%m%d_%H%M%S")
        (self.dir / "images").mkdir(parents=True, exist_ok=True)
        (self.dir / "labels").mkdir(parents=True, exist_ok=True)
        self._rows, self._t_last, self._sample, self._images = [], None, 0, 0
        self._boxes = {c: 0 for c in CLASSES}
        self._rp = rep.create.render_product(camera, IMAGE_SIZE)
        self._rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        self._bbox = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight", init_params={"semanticTypes": ["class"]})
        self._rgb.attach(self._rp)
        self._bbox.attach(self._rp)
        self._start = {"forklift": forklift_state(stage), "pallet": pallet_pose(stage)}
        self._wall0 = time.time()
        self.active = True

    def stop(self, stage, sim_time):
        """시연을 끝내고 steps.csv + meta.json을 저장합니다. 요약 dict를 반환합니다."""
        if not self.active:
            return None
        self.active = False
        for annot in (self._rgb, self._bbox):
            try:
                annot.detach()
            except Exception:  # noqa: BLE001
                pass
        if self._rp is not None:
            try:
                self._rp.destroy()
            except Exception:  # noqa: BLE001
                pass
        self._rp = self._rgb = self._bbox = None

        with open(self.dir / "steps.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(COLUMNS)
            w.writerows(self._rows)
        end_pallet = list(self._rows[-1][COLUMNS.index("pallet_x"):COLUMNS.index("pallet_yaw_deg") + 1]) if self._rows else None
        footprint = shelf_footprint(stage)
        success = bool(end_pallet) and placed_on_shelf(end_pallet, footprint)
        meta = {
            "created": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration_s": round(sim_time, 2),
            "samples": len(self._rows),
            "state_hz": STATE_HZ,
            "images": self._images,
            "image_size": list(IMAGE_SIZE),
            "boxes": self._boxes,
            "classes": CLASSES,
            "start": self._start,
            "end_pallet": end_pallet,
            "target_shelf_footprint": footprint,
            "success": success,
            "camera": AI_CAMERA_PATH,
            "camera_offset_in_mast_stage": list(AI_CAMERA_OFFSET),
            "camera_focal_mm": AI_CAMERA_FOCAL_MM,
            "labeled_prims": getattr(self, "_labeled", {}),
            "pallet_path": PALLET_PATH,
            "shelf_path": SHELF_PATH,
        }
        with open(self.dir / "meta.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=1)
        return meta

    # ---------------------------------------------------------------- 매 프레임

    def update(self, stage, sim_time):
        """타임라인 재생 중 앱 프레임마다 호출합니다."""
        if not self.active:
            return
        if self._t_last is not None and sim_time - self._t_last < 1.0 / STATE_HZ - 1e-6:
            return
        self._t_last = sim_time
        keys = [int(bool(fon._is_key_down(stage, path))) for _, path in KEYS]
        s = forklift_state(stage)
        pallet = pallet_pose(stage)
        image = ""
        if self._sample % IMAGE_EVERY == 0:
            image = self._save_image()
        self._sample += 1
        self._rows.append([round(sim_time, 4)] + keys
                          + [round(s["x"], 4), round(s["y"], 4), round(s["yaw_deg"], 3), round(s["speed_mps"], 4),
                             round(s["steer_deg"], 3), round(s["lift_m"], 4)]
                          + [round(v, 4) for v in pallet] + [image])

    def _save_image(self):
        from PIL import Image

        rgb = self._rgb.get_data() if self._rgb else None
        if rgb is None or not hasattr(rgb, "shape") or rgb.size == 0:
            return ""
        name = f"{self._images:06d}"
        Image.fromarray(np.asarray(rgb)[..., :3]).save(self.dir / "images" / f"{name}.png")
        lines = []
        bbox = self._bbox.get_data() if self._bbox else None
        if bbox and bbox.get("data") is not None and len(bbox["data"]):
            id_to_labels = bbox.get("info", {}).get("idToLabels", {})
            h, w = rgb.shape[0], rgb.shape[1]
            for b in bbox["data"]:
                label = id_to_labels.get(str(int(b["semanticId"])), id_to_labels.get(int(b["semanticId"]), {}))
                cls = label.get("class") if isinstance(label, dict) else None
                if cls not in CLASSES:
                    continue
                x0, y0, x1, y1 = (max(0, int(b["x_min"])), max(0, int(b["y_min"])),
                                  min(w - 1, int(b["x_max"])), min(h - 1, int(b["y_max"])))
                if x1 - x0 < MIN_BOX_PX or y1 - y0 < MIN_BOX_PX:
                    continue
                cx, cy, bw, bh = (x0 + x1) / 2 / w, (y0 + y1) / 2 / h, (x1 - x0) / w, (y1 - y0) / h
                line = f"{CLASSES.index(cls)} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}"
                if line in lines:  # 같은 물체가 두 번 잡힘 (라벨 붙은 prim과 라벨 붙은 자식)
                    continue
                lines.append(line)
                self._boxes[cls] += 1
        with open(self.dir / "labels" / f"{name}.txt", "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        self._images += 1
        return f"images/{name}.png"


# AI 주행 결과, 한 번에 한 줄 (Excel에서 열리도록 BOM 포함 UTF-8)
AI_RUNS_FILE = ROOT / "forklift_ai" / "ai_runs.csv"
AI_RUN_FIELDS = ["when", "start_vertical_m", "start_horizontal_m", "start_yaw_deg", "pallet_back_m", "result", "detail", "seconds", "model",
                 "route", "eyes", "seat_true_m", "seat_used_m"]


def load_ai_runs():
    try:
        with open(AI_RUNS_FILE, encoding="utf-8-sig", newline="") as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def _save_ai_runs(rows):
    AI_RUNS_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(AI_RUNS_FILE, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=AI_RUN_FIELDS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def append_ai_run(row):
    _save_ai_runs(load_ai_runs() + [row])


def delete_last_ai_run():
    rows = load_ai_runs()
    if rows:
        _save_ai_runs(rows[:-1])
    return bool(rows)


def clear_ai_runs():
    _save_ai_runs([])


def list_demos():
    """모든 녹화를 오래된 순으로: 폴더 이름, 성공 여부, 초, 이미지 수, 지게차 출발점 (x, y)."""
    demos = []
    if DEMOS_DIR.exists():
        for meta_path in sorted(DEMOS_DIR.glob("demo_*/meta.json")):
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            fk = meta.get("start", {}).get("forklift", {})
            demos.append({"name": meta_path.parent.name, "success": bool(meta.get("success")),
                          "duration_s": float(meta.get("duration_s", 0.0)), "images": int(meta.get("images", 0)),
                          "start": (fk.get("x"), fk.get("y")), "yaw": fk.get("yaw_deg")})
    return demos


def summarize_demos():
    """지금까지 녹화한 전체의 (개수, 성공 수, 이미지 수, 총 초)."""
    count = ok = images = 0
    secs = 0.0
    if DEMOS_DIR.exists():
        for meta_path in DEMOS_DIR.glob("demo_*/meta.json"):
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            count += 1
            ok += bool(meta.get("success"))
            images += int(meta.get("images", 0))
            secs += float(meta.get("duration_s", 0.0))
    return count, ok, images, secs
