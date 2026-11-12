"""Records a human forklift demonstration for imitation learning.

Per sample (STATE_HZ): which control keys are held, the forklift state (pose, speed,
steering, fork height) and the pallet pose. Every IMAGE_EVERY-th sample also saves the
driver-camera image plus YOLO-format boxes for the pallet and the target rack level, taken
from Replicator's tight 2D bounding boxes (the simulator knows where they are, so nobody
has to label anything).

Output: <DEMOS_DIR>/demo_<time>/ steps.csv, meta.json, images/*.png, labels/*.txt
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

ROOT = Path(__file__).resolve().parents[3]
DEMOS_DIR = ROOT / "forklift_ai" / "demos"

PALLET_PATH = "/Warehouse/Shelf_1/SM_PaletteA_357"
PALLET_BOXES_PATH = "/Warehouse/Shelf_1/CardBoxSet1_01"  # the boxes on it (moved with it by the panel)
SHELF_PATH = "/Warehouse/Shelf_1/SM_RackShelf_156"
CLASSES = ["pallet", "rack_level2"]  # YOLO class ids 0, 1
LEVEL2_TOP_Z = (1.2, 1.45)  # rack shelves whose top is at this height count as "2nd level"

# AI camera: between the forks, riding up and down with the carriage (driver view is blocked by
# the mast and dashboard). Parented to the lifting mast stage, which moves exactly with the forks.
AI_CAMERA_PATH = f"{fon.MAST_LIFT_PATHS[0]}/AICamera"
AI_CAMERA_OFFSET = (0.0, -0.28, 0.09)  # in the mast stage's frame: just ahead of the carriage, ~0.3 m up at rest
AI_CAMERA_FOCAL_MM = 12.0  # ~82 deg horizontal field of view

STATE_HZ = 15.0
IMAGE_EVERY = 3  # -> 5 images per second
IMAGE_SIZE = (640, 480)
MIN_BOX_PX = 4  # ignore slivers

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
    """Class labels for the bounding-box annotator, authored in the session layer (scene file untouched).

    Every pallet is "pallet" and every 2nd-level rack shelf is "rack_level2": objects that look the
    same must be labeled the same, or the detector gets contradicting examples.
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
    """Camera between the forks (session layer). Returns its path."""
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        cam = UsdGeom.Camera.Define(stage, AI_CAMERA_PATH)
        cam.CreateFocalLengthAttr(AI_CAMERA_FOCAL_MM)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
        xf = UsdGeom.Xformable(cam)
        xf.ClearXformOpOrder()
        # camera looks down its -Z with +Y up; the forklift drives toward its local -Y with +Z up
        m = Gf.Matrix4d(-1, 0, 0, 0,
                        0, 0, 1, 0,
                        0, 1, 0, 0,
                        AI_CAMERA_OFFSET[0], AI_CAMERA_OFFSET[1], AI_CAMERA_OFFSET[2], 1)
        xf.AddTransformOp().Set(m)
    return AI_CAMERA_PATH


def pallet_pose(stage):
    """(x, y, z, yaw_deg) of the pallet: live physics pose while simulating, USD otherwise."""
    try:
        res = omni.physx.get_physx_interface().get_rigidbody_transformation(PALLET_PATH)
        if res and res.get("ret_val"):
            p, q = res["position"], res["rotation"]  # q = (x, y, z, w)
            yaw = math.degrees(math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)))
            return float(p[0]), float(p[1]), float(p[2]), yaw
    except Exception:  # noqa: BLE001 - fall back to USD below
        pass
    prim = stage.GetPrimAtPath(PALLET_PATH)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    x_axis = m.TransformDir((1.0, 0.0, 0.0))
    return float(t[0]), float(t[1]), float(t[2]), math.degrees(math.atan2(x_axis[1], x_axis[0]))


def pallet_tilt_deg(stage):
    """How far the pallet leans from lying flat [deg]: live physics pose while simulating, USD otherwise."""
    try:
        res = omni.physx.get_physx_interface().get_rigidbody_transformation(PALLET_PATH)
        if res and res.get("ret_val"):
            q = res["rotation"]  # (x, y, z, w)
            up_z = 1.0 - 2.0 * (q[0] ** 2 + q[1] ** 2)  # world z of the pallet's own up axis
            return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    except Exception:  # noqa: BLE001 - fall back to USD below
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


# Fork tips are ~2.45 m ahead of the forklift's origin and the pallet is ~1 m deep: beyond this distance
# between the forklift and the pallet centre the forks are completely out of it.
FORKS_CLEAR_M = 2.95


def placed_on_shelf(pallet, footprint, margin=-0.15):
    """Pallet resting on the target rack level (its origin is at the pallet's bottom). The default
    margin asks for the pallet centre to be at least 15 cm inside the shelf's edges, i.e. properly on it
    rather than hanging over the front beam."""
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

    # ---------------------------------------------------------------- lifecycle

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
        """Finish the demo, write steps.csv + meta.json. Returns a summary dict."""
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

    # ---------------------------------------------------------------- per frame

    def update(self, stage, sim_time):
        """Call every app frame while the timeline plays."""
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
                if line in lines:  # the same object reported twice (labeled prim and a labeled child)
                    continue
                lines.append(line)
                self._boxes[cls] += 1
        with open(self.dir / "labels" / f"{name}.txt", "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        self._images += 1
        return f"images/{name}.png"


# Every AI drive's result, one row each (opens in Excel: UTF-8 with BOM)
AI_RUNS_FILE = ROOT / "forklift_ai" / "ai_runs.csv"
AI_RUN_FIELDS = ["when", "start_vertical_m", "start_horizontal_m", "start_yaw_deg", "pallet_back_m", "result", "detail", "seconds", "route",
                 "eyes", "seat_true_m", "seat_used_m"]


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
    """Every recording, oldest first: folder name, success, seconds, images and the forklift's start (x, y)."""
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
                          "start": (fk.get("x"), fk.get("y"))})
    return demos


def summarize_demos():
    """(count, successes, images, total seconds) over everything recorded so far."""
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
