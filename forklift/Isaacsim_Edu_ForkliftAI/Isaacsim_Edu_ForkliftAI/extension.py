"""Forklift AI: record human demonstrations and let the AI drive, in a copy of the Lesson 1 forklift scene."""

import asyncio
import datetime
import json
import os
import shutil
import time
from pathlib import Path

import carb
import carb.settings
import omni.ext
import omni.kit.app
import omni.kit.viewport.utility as vp_util
import omni.timeline
import omni.ui as ui
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

import Isaacsim_Edu_Lv2_Lesson1.forklift_omnigraph_nodes as fon

from .ai_driver import AIDriver, start_overlap
from .recorder import (AI_CAMERA_PATH, DEMOS_DIR, PALLET_BOXES_PATH, PALLET_PATH, ROOT, SHELF_PATH, DemoRecorder, append_ai_run,
                       clear_ai_runs, delete_last_ai_run, ensure_ai_camera, forklift_state, list_demos, load_ai_runs,
                       summarize_demos)
from .vision import newest_model
from .window import PANEL_SIGN, ForkliftAIWindow

MODELS_DIR = ROOT / "forklift_ai" / "models"
PATHS_FILE = MODELS_DIR / "paths.json"  # learn_paths.py without --name: the '기본' AI model
RUNS_DIR = ROOT / "forklift_ai" / "runs"  # YOLO training output (runs/<name>/weights/best.onnx)
DEFAULT_MODEL = "기본"


def list_ai_models():
    """The AI models the panel offers: models/paths.json ('기본') and every models/<name>/paths.json
    (learn_paths.py --name <name>). Each pairs with a YOLO model: models/<name>/best.onnx, else the YOLO run of
    the same name (runs/<name>/weights/best.onnx), else the newest YOLO run.
    Returns [{"name", "paths", "yolo", "mtime"}]."""
    found = [(DEFAULT_MODEL, PATHS_FILE, None)] if PATHS_FILE.exists() else []
    found += [(p.parent.name, p, p.parent) for p in sorted(MODELS_DIR.glob("*/paths.json"))]
    newest = newest_model(RUNS_DIR)
    models = []
    for name, paths, folder in found:
        own = [folder / "best.onnx", RUNS_DIR / name / "weights" / "best.onnx"] if folder else []
        yolo = next((str(c) for c in own if c.exists()), newest)
        try:
            mtime = os.stat(paths).st_mtime
        except OSError:
            continue
        models.append({"name": name, "paths": paths, "yolo": yolo, "mtime": mtime})
    return models
# This practice's own copy of the Lesson 1 scene (same folder, so the asset references still resolve);
# edit it freely without touching Lesson 1.
SCENE_FILE = ROOT / "iw_hub_warehouse_navigation" / "iw_hub_warehouse_forklift_ai.usd"

LOG = "[Isaacsim_Edu_ForkliftAI]"
START_WAIT_FRAMES = 3  # a recording begins this many frames after Play: the overlap test needs physics running
START_DRAWS = 30  # '랜덤': a start overlapping something is drawn again at most this often
# This is Lesson 2 in the AZ Edu Lv2 launcher (crash recovery resumes the lesson recorded here).
SETTING_ACTIVE_LESSON = "/persistent/exts/Isaacsim_Edu_Lv2/active_lesson"
# Lesson 1's driving keys, for the panel's key display
KEY_PATHS = {"forward": fon.KEY_FORWARD_PATH, "backward": fon.KEY_BACKWARD_PATH, "left": fon.KEY_LEFT_PATH,
             "right": fon.KEY_RIGHT_PATH, "lift_up": fon.KEY_LIFT_UP_PATH, "lift_down": fon.KEY_LIFT_DOWN_PATH}
# The forklift starts at the scene's own start pose moved by the panel's X/Y offset sliders (heading
# unchanged). Scene layout (forklift-AI copy): rack front x 0.22 | pallet x 1.22-2.22 | forklift x 6.47,
# all on y -1.0; the far rack behind the forklift begins around x 8.1.


class IsaacsimEduForkliftAIExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        carb.settings.get_settings().set(SETTING_ACTIVE_LESSON, "Isaacsim_Edu_ForkliftAI")
        self._recorder = DemoRecorder()
        self._last_dir = None
        self._pending_start = False
        self._start_redraw = False  # '랜덤' recording: draw the start again if it overlaps something
        self._window = ForkliftAIWindow({
            "start": self._on_start,
            "stop": self._on_stop,
            "discard": self._on_discard,
            "open_folder": self._on_open_folder,
            "toggle_camera": self._on_toggle_camera,
            "ai_start": self._on_ai_start,
            "ai_stop": self._on_ai_stop,
            "run_discard": self._on_run_discard,
            "run_clear": self._on_run_clear,
            "start_changed": self._on_start_changed,
            "model_changed": self._refresh_model_info,
            "open_model_folder": self._on_open_model_folder,
            "free_changed": self._on_free_changed,
        })
        self._base_pose = None  # (x, y, z, yaw_deg) the scene starts the forklift at
        self._pallet_base = {}  # prim path -> (translate op, scene value, parent world->local) of the pallet / its boxes
        self._reapply_frames = 0
        self._driver = None
        self._ai_pending = False
        self._ai_scene_held = False  # a finished AI drive's paused end scene is on screen until Stop
        self._ai_task = None
        self._ai_run_info = None  # what the running AI drive started with, for its row in the AI run table
        # '위치 자유': (x, y, yaw_deg) where the forklift was put by hand -- taken when the option is turned on and
        # each time a recording / AI drive starts from a stopped scene; recordings and AI drives start there and
        # Stop / 'AI 멈춤' bring the forklift back there (instead of the sliders' pose)
        self._free_pose = None
        self._watch = None  # (demos folder, AI models) last seen, to refresh the panel
        self._ai_models = []  # list_ai_models(), as last shown in the panel's 'AI 모델' list
        self._watch_t = 0.0
        self._keys_shown = False
        self._refresh_summary()
        self._refresh_ai_runs()
        self._refresh_model_info()
        self._update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="Isaacsim_Edu_ForkliftAI")
        events = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._stop_sub = events.create_subscription_to_pop_by_type(int(omni.timeline.TimelineEventType.STOP), self._on_timeline_stop)
        self._open_task = asyncio.ensure_future(self._open_scene())
        self._stage_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="Isaacsim_Edu_ForkliftAI stage")
        self._capture_base_pose()

    def on_shutdown(self):
        stage = self._stage()
        if stage is not None and self._pallet_base and omni.timeline.get_timeline_interface().is_stopped():
            self._apply_pallet(stage, back=0.0)  # leave the pallet where the scene file has it
        if self._open_task is not None and not self._open_task.done():
            self._open_task.cancel()
        self._open_task = None
        self._update_sub = None
        self._stop_sub = None
        self._stage_sub = None
        if self._driver is not None:
            self._driver.stop()
            self._driver = None
        if self._recorder.active:
            self._finish(stop_timeline=False)
        if self._window:
            self._window.visible = False
            self._window.destroy()
            self._window = None

    # ---------------------------------------------------------------- scene

    async def _open_scene(self):
        """Open this practice's scene copy. Lesson 1 is only a dependency here (the forklift's controls):
        it doesn't open its own scene or panel while this extension is enabled."""
        app = omni.kit.app.get_app()
        ctx = omni.usd.get_context()
        for _ in range(10):  # same settling wait as Lesson 1's own scene open
            await app.next_update_async()
        for _ in range(1800):  # up to ~30 s: opening while another scene still loads fails (UsdContext busy)
            if ctx.can_open_stage():
                break
            await app.next_update_async()
        stage = ctx.get_stage()
        current = stage.GetRootLayer().realPath if stage else ""
        if not (current and os.path.normcase(os.path.abspath(current)) == os.path.normcase(str(SCENE_FILE))):
            if not SCENE_FILE.exists():
                carb.log_error(f"{LOG} scene copy not found: {SCENE_FILE}")
                if self._window:
                    self._window.set_last(f"실습 장면 파일이 없습니다: {SCENE_FILE}", ok=False)
                return
            fon.register_forklift_node_types()  # the scene's Action Graph uses Lesson 1's node types
            for _ in range(5):  # retry if another scene was still loading ("UsdContext busy")
                ok, _err = await ctx.open_stage_async(str(SCENE_FILE).replace("\\", "/"))
                if ok:
                    break
                for _ in range(60):
                    await app.next_update_async()
        if self._window:
            self._window.deferred_dock_in("AZ Edu Lv2", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

    # ---------------------------------------------------------------- helpers

    def _stage(self):
        return omni.usd.get_context().get_stage()

    def _scene_ready(self, stage):
        return (stage is not None and stage.GetPrimAtPath(fon.FORKLIFT_PATH).IsValid()
                and stage.GetPrimAtPath(PALLET_PATH).IsValid() and stage.GetPrimAtPath(SHELF_PATH).IsValid())

    def _refresh_summary(self):
        if self._window:
            self._window.set_summary(*summarize_demos())
            self._window.set_demos(list_demos(), (self._base_pose[0], self._base_pose[1], self._base_pose[3]) if self._base_pose else None)

    def _watch_files(self):
        """Pick up new recordings and newly learned AI models (paths.json / YOLO) without restarting."""
        def mtime(path):
            try:
                return os.stat(path).st_mtime
            except OSError:
                return None

        state = (mtime(DEMOS_DIR), tuple((m["name"], m["mtime"], m["yolo"]) for m in list_ai_models()))
        if state != self._watch:
            first = self._watch is None
            self._watch = state
            if not first:
                self._refresh_summary()
                self._refresh_model_info()

    def _show_keys(self, stage):
        """Light up the panel's key display with the driving keys held right now (the AI's or the person's)."""
        if not self._window:
            return
        if stage is None:
            if self._keys_shown:
                self._window.set_keys({})
                self._keys_shown = False
            return
        self._window.set_keys({name: fon._is_key_down(stage, path) for name, path in KEY_PATHS.items()})
        self._keys_shown = True

    # ---------------------------------------------------------------- start position

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.OPENED):
            self._base_pose = None
            self._capture_base_pose()

    def _capture_base_pose(self):
        """Remember where the scene file starts the forklift (read while nothing has moved it)."""
        stage = self._stage()
        if not self._scene_ready(stage) or not omni.timeline.get_timeline_interface().is_stopped():
            return
        prim = stage.GetPrimAtPath(fon.FORKLIFT_PATH)
        pos = prim.GetAttribute("xformOp:translate").Get()
        rot = prim.GetAttribute("xformOp:rotateXYZ").Get()
        self._base_pose = (float(pos[0]), float(pos[1]), float(pos[2]), float(rot[2]))
        self._capture_pallet_base(stage)
        self._refresh_summary()  # the table shows each recording's start relative to this pose
        self._refresh_model_info()  # the panel is built by now (on_startup's call came before it existed)
        self._apply_start(stage)

    def _apply_start(self, stage):
        """Put the forklift at the scene's start pose + the panel's offset."""
        if self._base_pose is None:
            self._capture_base_pose()
            if self._base_pose is None:
                return
        prim = stage.GetPrimAtPath(fon.FORKLIFT_PATH)
        if not prim.IsValid():
            return
        x, y, z, yaw = self._base_pose
        dx, dy = self._window.start_offset if self._window else (0.0, 0.0)
        dyaw = self._window.start_yaw if self._window else 0.0
        rot = prim.GetAttribute("xformOp:rotateXYZ").Get()
        prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(x + dx, y + dy, z))
        prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(rot[0], rot[1], yaw + dyaw))
        self._apply_pallet(stage)

    def _capture_pallet_base(self, stage):
        """Where the scene file puts the pallet and the boxes on it. They are moved the way the forklift's
        start is (an edit on the scene's own layer): moved in the session layer instead, the pallet's
        colliders stopped meeting the forks (they drove right through it). A leftover session-layer offset
        from that is cleared first."""
        self._pallet_base = {}
        for path in (PALLET_PATH, PALLET_BOXES_PATH):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            op = next((o for o in UsdGeom.Xformable(prim).GetOrderedXformOps()
                       if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
            if op is None:
                continue
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                op.GetAttr().Clear()
            to_parent = UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
            self._pallet_base[path] = (op, op.Get(), to_parent)

    def _apply_pallet(self, stage, back=None):
        """Move the pallet (and its boxes) straight back by the panel's '파렛트 뒤로' (or `back` metres) --
        away from the forklift, which faces the scene's -X, so back is -X too."""
        if back is None:
            back = self._window.pallet_back if self._window else 0.0
        for op, base, to_parent in self._pallet_base.values():
            if not op.GetAttr().IsValid():
                continue
            d = to_parent.TransformDir(Gf.Vec3d(-back, 0.0, 0.0))
            value = type(base)(base[0] + d[0], base[1] + d[1], base[2] + d[2])
            if op.Get() != value:
                op.Set(value)

    def _capture_free_pose(self, stage):
        """'위치 자유': remember where the forklift stands now (put there by hand) as the place to start from and
        to come back to."""
        if self._scene_ready(stage):
            s = forklift_state(stage)
            self._free_pose = (s["x"], s["y"], s["yaw_deg"])

    def _reset_start(self, stage):
        """Where a stopped scene puts the forklift: the '위치 자유' pose if that option is on, else the sliders'."""
        if self._window and self._window.free_start and self._free_pose is not None and self._base_pose:
            self._place_forklift_free(stage)
        else:
            self._apply_start(stage)

    def _on_free_changed(self, on):
        stage = self._stage()
        if not (self._scene_ready(stage) and omni.timeline.get_timeline_interface().is_stopped()):
            return
        if on:
            self._capture_free_pose(stage)
        else:
            self._free_pose = None
            self._apply_start(stage)  # back to the sliders' pose

    def _place_forklift_free(self, stage):
        """'위치 자유': the forklift at the remembered pose -- on the floor (the scene's height) and upright,
        only its x, y and heading kept."""
        prim = stage.GetPrimAtPath(fon.FORKLIFT_PATH)
        if not prim.IsValid() or self._base_pose is None:
            return
        x, y, yaw = self._free_pose
        rot = prim.GetAttribute("xformOp:rotateXYZ").Get()
        prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(x, y, self._base_pose[2]))
        prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(rot[0], rot[1], yaw))
        self._apply_pallet(stage)

    def _clear_start(self, stage, redraw):
        """With the simulation running (the overlap test asks the physics scene): None if the forklift
        overlaps nothing where it stands, else why it cannot start (Korean). With `redraw` ('랜덤') new random
        starts are drawn -- and the forklift put there -- until one overlaps nothing."""
        for attempt in range(START_DRAWS + 1 if redraw else 1):
            if attempt:
                self._window.randomize_start()
                self._apply_start(stage)
            s = forklift_state(stage)
            problem = start_overlap(stage, s["x"], s["y"], s["yaw_deg"], s["lift_m"])
            if problem is None:
                return None
        if redraw:
            return f"랜덤 출발 위치를 {START_DRAWS}번 다시 뽑았지만 모두 다른 물체와 겹칩니다. 범위를 좁혀 주세요."
        return problem

    def _on_start_changed(self):
        stage = self._stage()
        if self._scene_ready(stage) and omni.timeline.get_timeline_interface().is_stopped():
            if self._window.free_start:  # '위치 자유': leave the forklift where it was put by hand
                self._apply_pallet(stage)
            else:
                self._apply_start(stage)  # preview: the forklift moves as the slider moves

    # ---------------------------------------------------------------- buttons

    def _on_start(self):
        if self._ai_scene_held or (self._driver is not None and (self._driver.active or self._ai_pending)):
            return  # no recording while the AI drives or its end scene is kept (the button is off then too)
        stage = self._stage()
        if not self._scene_ready(stage):
            self._window.set_last("지게차 AI 장면(파렛트·선반 포함)이 열려 있지 않습니다.", ok=False)
            return
        timeline = omni.timeline.get_timeline_interface()
        self._start_redraw = False
        if not timeline.is_playing():
            # people's demonstrations are always recorded with the pallet where the scene puts it: the routes
            # the AI learns from them only fit that pallet position
            if self._window.free_start:  # '위치 자유': record from where the forklift was put by hand
                if timeline.is_stopped():
                    self._capture_free_pose(stage)
            self._window.set_pallet_back(0.0)
            if self._window.random_start:
                self._start_redraw = True  # drawn again once physics runs if it overlaps something
                self._window.randomize_start()  # '랜덤': a new start each recording, shown on the sliders
            self._reset_start(stage)
            viewport = vp_util.get_active_viewport()
            if viewport is not None:
                fon.reset_camera_to_home(stage, fon.DRIVER_VIEW_CAMERA_PATH)
                viewport.camera_path = Sdf.Path(fon.DRIVER_VIEW_CAMERA_PATH)
            fon.reset_drive_state()
            timeline.play()
        self._pending_start = START_WAIT_FRAMES  # begin once the simulation (and its overlap test) is running
        self._window.set_recording(True)
        self._window.set_last("")

    def _on_stop(self):
        self._finish(stop_timeline=True)

    def _on_discard(self):
        if self._last_dir is None or not os.path.isdir(self._last_dir):
            self._window.set_last("지울 녹화가 없습니다.", ok=False)
            return
        shutil.rmtree(self._last_dir, ignore_errors=True)
        self._window.set_last(f"지웠습니다: {os.path.basename(self._last_dir)}", ok=False)
        self._last_dir = None
        self._refresh_summary()

    def _on_toggle_camera(self):
        stage = self._stage()
        viewport = vp_util.get_active_viewport()
        if not self._scene_ready(stage) or viewport is None:
            return
        showing_ai = str(viewport.camera_path) == AI_CAMERA_PATH
        if showing_ai:
            viewport.camera_path = Sdf.Path(fon.DRIVER_VIEW_CAMERA_PATH)
        else:
            viewport.camera_path = Sdf.Path(ensure_ai_camera(stage))
        self._window.set_camera_button(not showing_ai)

    # ---------------------------------------------------------------- AI driving

    def _selected_model(self):
        """The AI model picked in the panel (a list_ai_models() entry), or None."""
        name = self._window.ai_model_name if self._window else None
        return next((m for m in self._ai_models if m["name"] == name), None)

    def _refresh_model_info(self):
        if not self._window:
            return
        models = list_ai_models()
        if [(m["name"], m["mtime"], m["yolo"]) for m in models] != [(m["name"], m["mtime"], m["yolo"]) for m in self._ai_models]:
            self._ai_models = models
            items = []
            for m in models:
                try:
                    n = len(json.loads(Path(m["paths"]).read_text(encoding="utf-8"))["demos"])
                except (OSError, ValueError, KeyError):
                    n = "?"
                yolo = os.path.relpath(m["yolo"], RUNS_DIR).split(os.sep)[0] if m["yolo"] else "없음"
                items.append((m["name"], f"{m['name']} · 경로 {n}개 · YOLO {yolo}"))
            newest = max(models, key=lambda m: m["mtime"])["name"] if models else None
            self._window.set_ai_models(items, newest)
        model = self._selected_model()
        if model is None:
            self._window.set_model_info("학습된 AI 모델이 없습니다. tools\\learn_paths.py로 먼저 학습하세요.")
            self._window.set_ai_ready(False)
            return
        paths_file, yolo = Path(model["paths"]), model["yolo"]
        where = os.path.relpath(paths_file, MODELS_DIR.parent)
        eyes = (f"YOLO 모델: {os.path.relpath(yolo, RUNS_DIR.parent)}" if yolo
                else "YOLO 모델(runs\\...\\weights\\best.onnx)이 없어 'YOLO로 파렛트 거리 재기'를 켜도 끈 것처럼 "
                     "녹화 경로 끝에서 들어 올립니다.")
        try:
            paths = json.loads(paths_file.read_text(encoding="utf-8"))
            usable = sum(1 for d in paths["demos"] if d.get("usable"))
            self._window.set_ai_ready(usable > 0)
            if not usable:
                self._window.set_model_info(f"학습된 경로 중 AI가 쓸 수 있는 경로가 없습니다 ({where}).", eyes)
                return
            self._window.set_model_info(f"학습된 경로: 시연 {len(paths['demos'])}개 ({where})", eyes)
            if yolo and not paths.get("vision"):
                self._window.set_model_info(f"학습된 경로: 시연 {len(paths['demos'])}개 ({where})",
                                            eyes + " · 거리표가 없으니 learn_paths.py를 다시 실행하세요.")
        except (OSError, ValueError, KeyError):
            self._window.set_ai_ready(False)
            self._window.set_model_info(f"학습된 경로 파일을 읽지 못했습니다: {paths_file}", eyes)

    def _on_ai_start(self):
        if self._driver is not None and self._driver.active:
            self._toggle_ai_pause()  # while driving the button is 'AI 일시 정지' / 'AI 다시 시작'
            return
        stage = self._stage()
        if not self._scene_ready(stage):
            self._window.set_ai(False, "지게차 AI 장면이 열려 있지 않습니다.", ok=False)
            return
        selected = self._selected_model()
        if selected is None or not Path(selected["paths"]).exists():
            self._window.set_ai(False, "학습된 AI 모델(paths.json)이 없습니다.", ok=False)
            return
        if self._recorder.active:
            return
        if self._window.random_start:
            # '랜덤': a new start (and pallet position) each AI drive -- with '위치 자유' only the pallet's
            self._window.randomize_start(pallet=True, forklift=not self._window.free_start)
        if self._driver is not None:
            self._driver.stop()
        eyes = self._window.eyes_mode
        model = selected["yolo"] if eyes == "yolo" else None
        try:
            # 'YOLO로 파렛트 거리 재기' on without a trained model drives as if it were off (the panel says so)
            self._driver = AIDriver(selected["paths"], model, blind=eyes == "none" or model is None,
                                    reposition=self._window.allow_reposition)
        except Exception as e:  # noqa: BLE001 - e.g. the ONNX model could not be loaded
            carb.log_error(f"{LOG} could not start the AI driver: {e}")
            self._window.set_ai(False, f"AI를 준비하지 못했습니다: {e}", ok=False)
            self._driver = None
            return
        self._refresh_model_info()
        dx, dy = self._window.start_offset
        yaw = (self._base_pose[3] if self._base_pose else 0.0) + self._window.start_yaw
        if self._window.free_start and self._base_pose:  # '위치 자유': start where the forklift was put by hand
            if omni.timeline.get_timeline_interface().is_stopped() or self._free_pose is None:
                self._capture_free_pose(stage)  # (a paused end scene keeps the pose the last drive started from)
            fx, fy, fyaw = self._free_pose
            dx, dy, yaw = fx - self._base_pose[0], fy - self._base_pose[1], fyaw
        self._ai_run_info = {"when": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                             "start_vertical_m": f"{PANEL_SIGN['x'] * dx:+.2f}",
                             "start_horizontal_m": f"{PANEL_SIGN['y'] * dy:+.2f}",
                             "start_yaw_deg": f"{yaw % 360.0:.0f}",
                             "pallet_back_m": f"{self._window.pallet_back:.2f}",
                             "model": selected["name"]}
        self._window.set_ai(True, "AI 운전 준비 중...")
        self._ai_task = asyncio.ensure_future(self._start_ai_run())

    async def _start_ai_run(self):
        """Every AI run starts from a clean scene: Stop (Lesson 1 resets the forklift, physics resets the
        pallet), let that stop event be handled, then place the forklift and Play."""
        app = omni.kit.app.get_app()
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_stopped():
            timeline.stop()
            for _ in range(5):
                await app.next_update_async()
        stage = self._stage()
        self._reset_start(stage)  # Stop put it back at the scene's start: the sliders' pose or the '위치 자유' pose
        viewport = vp_util.get_active_viewport()
        if viewport is not None:
            fon.reset_camera_to_home(stage, fon.DRIVER_VIEW_CAMERA_PATH)
            viewport.camera_path = Sdf.Path(fon.DRIVER_VIEW_CAMERA_PATH)
        fon.reset_drive_state()
        self._window.set_ai(True, "출발 전에 학습한 경로들을 머릿속으로 연습해 보는 중...")
        timeline.play()  # the rehearsal's collision test asks the running physics scene
        for _ in range(3):
            await app.next_update_async()
        if self._driver is None or not timeline.is_playing():
            return
        redraw = self._window.random_start and not self._window.free_start
        problem = self._clear_start(stage, redraw)
        if problem:
            self._end_ai(f"실패: {problem}", ok=False, stop_timeline=True, result="출발 불가")
            return
        if redraw and self._ai_run_info:  # the start may have been drawn again: log where it really set off
            dx, dy = self._window.start_offset
            self._ai_run_info.update(start_vertical_m=f"{PANEL_SIGN['x'] * dx:+.2f}",
                                     start_horizontal_m=f"{PANEL_SIGN['y'] * dy:+.2f}",
                                     start_yaw_deg=f"{(self._base_pose[3] + self._window.start_yaw) % 360.0:.0f}")
        s = forklift_state(stage)
        problem = self._driver.check_start(stage, s["x"], s["y"], s["yaw_deg"], s["lift_m"])
        if problem:
            self._end_ai(f"실패: {problem}", ok=False, stop_timeline=True, result="출발 불가")
            return
        if not self._driver.plan(stage, s["x"], s["y"], s["yaw_deg"]):
            self._end_ai(f"실패: {self._driver.plan_text}", ok=False, stop_timeline=True, result="경로 없음")
            return
        self._driver.attach_eyes(stage)
        self._ai_pending = True

    def _on_ai_stop(self):
        timeline = omni.timeline.get_timeline_interface()
        preparing = self._ai_task is not None and not self._ai_task.done()
        driving = self._driver is not None and (self._driver.active or self._ai_pending)
        if preparing or driving:
            if preparing:
                self._ai_task.cancel()  # otherwise it would still hand the plan to the driver afterwards
            self._end_ai("사용자가 멈춤", ok=None, stop_timeline=False)
        if not timeline.is_stopped():
            timeline.stop()  # Stop puts the forklift and pallet back (also after a finished run, left paused)

    def _toggle_ai_pause(self):
        """While the AI drives, the start button pauses / resumes the simulation (the AI waits with it)."""
        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_playing():
            timeline.pause()
            self._window.set_ai(True, "일시 정지됨 · 'AI 다시 시작'을 누르면 이어서 운전합니다.")
        else:
            timeline.play()

    def _end_ai(self, text, ok, stop_timeline, result=None):
        self._ai_pending = False
        self._log_ai_run(result or {True: "성공", None: "중단", False: "실패"}[ok], text)
        if self._driver is not None:
            self._driver.stop()
        if self._window:
            self._window.set_ai(False, text, ok=ok)
        if stop_timeline:
            omni.timeline.get_timeline_interface().pause()  # keep the final scene visible; Stop resets it
            self._ai_scene_held = True
            if self._window:
                self._window.set_scene_stop(True)  # 'AI 멈춤' now stops (resets) the paused scene

    # ---------------------------------------------------------------- AI run table

    def _log_ai_run(self, result, text):
        """One row in ai_runs.csv per AI drive (only the first end of a drive counts)."""
        info, self._ai_run_info = self._ai_run_info, None
        if info is None:
            return
        d = self._driver
        took = None
        if d is not None and d._t0 is not None and d._t_last is not None:
            took = max(d._t_last - d._t0, 0.0)
        seat = d.seat_check if d is not None else None
        row = dict(info, result=result, detail=(text or "").splitlines()[0] if text else "",
                   seconds="" if took is None else f"{took:.1f}",
                   route=d.follower.demo["name"] if d is not None and d.follower is not None else "",
                   eyes="YOLO" if d is not None and d.eyes is not None else ("없음" if d is not None and d.blind else "정답"),
                   seat_true_m="" if not seat else f"{seat[0]:.3f}",
                   seat_used_m="" if not seat or seat[1] is None else f"{seat[1]:.3f}")
        try:
            append_ai_run(row)
        except OSError as e:
            carb.log_warn(f"{LOG} could not save the AI run: {e}")
        self._refresh_ai_runs()

    def _refresh_ai_runs(self):
        if self._window:
            self._window.set_ai_runs(load_ai_runs())

    def _on_run_discard(self):
        try:
            delete_last_ai_run()
        except OSError as e:
            carb.log_warn(f"{LOG} could not update the AI run file: {e}")
        self._refresh_ai_runs()

    def _on_run_clear(self):
        try:
            clear_ai_runs()
        except OSError as e:
            carb.log_warn(f"{LOG} could not update the AI run file: {e}")
        self._refresh_ai_runs()

    def _update_ai(self, stage, sim_time):
        if self._ai_pending:
            self._ai_pending = False
            self._driver.start(sim_time)
        if self._driver is None or not self._driver.active:
            return
        result = self._driver.update(stage, sim_time)
        d = self._driver
        held = " ".join(k for k, v in d.keys.items() if v) or "-"
        took = sim_time - d._t0
        if d.seat_check:
            true_m, used_m = d.seat_check
            seat = (f"\n들어 올린 순간 파렛트까지: 실제 {true_m:.2f} m · AI 판단 "
                    + (f"{used_m:.2f} m" if used_m is not None else "없음 (경로 끝에서 들어 올림)")
                    + f" (포크가 끝까지 들어가면 {d.paths.get('seat_ahead_m', 0):.2f} m)")
        else:
            seat = ""
        if result == "success":
            self._end_ai(f"성공: {took:.1f}초 만에 파렛트를 2층 선반에 올리고 포크를 뺐습니다.\n{d.plan_text}{seat}", ok=True, stop_timeline=True)
        elif result == "no_route":
            self._end_ai(f"실패: {d.plan_text}", ok=False, stop_timeline=True, result="경로 없음")
        elif result == "tilted":
            hint = ("\n'YOLO로 파렛트 거리 재기'가 꺼져 있으면 파렛트까지 거리를 재지 않아서, 파렛트가 녹화 때와 다른 곳에 있으면 포크가 덜 들어간 채 "
                    "들어 올립니다." if d.blind else "")
            self._end_ai(f"실패: 파렛트가 포크에 제대로 실리지 않아 {d.tilt_deg:.0f}° 기울어서 그 자리에서 멈췄습니다."
                         f"{hint}\n{d.plan_text}{seat}", ok=False, stop_timeline=True)
        elif result == "not_inserted":
            self._end_ai("실패: 포크가 파렛트에 80% 이상 들어가지 않아 들어 올리지 않았습니다 (뒤로 뺐다가 한 번 더 넣어 봤지만 "
                         f"안 됨).\n{d.plan_text}\n{d.eyes_text}", ok=False, stop_timeline=True)
        elif result == "stuck":
            self._end_ai(f"실패: {d.progress_text}에서 더 나아가지 못했습니다 (부딪힘?).\n{d.plan_text}{seat}", ok=False, stop_timeline=True)
        elif result == "missed":
            self._end_ai(f"실패: 경로는 끝까지 갔지만 파렛트가 2층 선반 위에 없습니다.\n{d.plan_text}{seat}", ok=False, stop_timeline=True)
        elif result == "timeout":
            self._end_ai(f"실패: {took:.0f}초 안에 끝내지 못했습니다.\n{d.plan_text}{seat}", ok=False, stop_timeline=True)
        else:
            self._window.set_ai(True, f"AI 운전 중 {took:.1f}초 · {d.progress_text} · 누르는 키: {held}\n"
                                      f"{d.eyes_text}\n{d.plan_text}")

    def _on_open_folder(self):
        DEMOS_DIR.mkdir(parents=True, exist_ok=True)
        os.startfile(os.path.normpath(str(DEMOS_DIR)))

    def _on_open_model_folder(self):
        """The chosen AI model's folder (models\\<name>, or models for '기본') in Windows Explorer; with no model
        yet the forklift_ai folder, where models\\ will appear."""
        model = self._selected_model()
        folder = Path(model["paths"]).parent if model else (MODELS_DIR if MODELS_DIR.exists() else MODELS_DIR.parent)
        if folder.exists():
            os.startfile(os.path.normpath(str(folder)))

    # ---------------------------------------------------------------- recording

    def _finish(self, stop_timeline):
        self._pending_start = False
        stage = self._stage()
        timeline = omni.timeline.get_timeline_interface()
        meta = self._recorder.stop(stage, timeline.get_current_time()) if self._recorder.active else None
        if self._window:
            self._window.set_recording(False)
        if meta:
            self._last_dir = str(self._recorder.dir)
            result = "성공 (파렛트가 2층 선반 위)" if meta["success"] else "파렛트가 2층에 놓이지 않음"
            self._window.set_last(
                f"저장: {os.path.basename(self._last_dir)} · {meta['duration_s']:.0f}초 · 이미지 {meta['images']}장 "
                f"(파렛트 박스 {meta['boxes']['pallet']}, 선반 박스 {meta['boxes']['rack_level2']}) · {result}",
                ok=meta["success"])
            self._refresh_summary()
        if stop_timeline and timeline.is_playing():
            timeline.stop()

    def _on_timeline_stop(self, _event):
        if self._recorder.active or self._pending_start:
            self._finish(stop_timeline=False)
        if self._driver is not None and (self._driver.active or self._ai_pending):
            self._end_ai("멈춤 (Stop)", ok=None, stop_timeline=False)
        self._ai_scene_held = False
        if self._window:
            self._window.set_scene_stop(False)
        self._reapply_frames = 3  # Lesson 1 puts the forklift back to the scene's start on Stop; re-apply the offset

    def _on_update(self, _event):
        timeline = omni.timeline.get_timeline_interface()
        now = time.monotonic()
        if now - self._watch_t >= 1.0:
            self._watch_t = now
            self._watch_files()
        self._show_keys(self._stage() if timeline.is_playing() else None)
        if self._window:  # also follows a pause / play from Isaac Sim's own toolbar
            driving = self._driver is not None and self._driver.active
            self._window.set_ai_pause_button(driving, paused=driving and not timeline.is_playing())
        if not timeline.is_playing():
            if self._driver is not None and self._driver.active and any(self._driver.keys.values()):
                # Lesson 1 moves the forklift every app frame, paused or not: let go of every key and stand still
                self._driver.keys = {k: False for k in self._driver.keys}
                fon._current_speed_mps = 0.0
                fon._current_turn_rate_deg_per_sec = 0.0
            if self._reapply_frames > 0 and timeline.is_stopped():
                self._reapply_frames -= 1
                if self._reapply_frames == 0 and self._scene_ready(self._stage()):
                    self._reset_start(self._stage())
            return
        stage = self._stage()
        if self._driver is not None and (self._driver.active or self._ai_pending):
            try:
                self._update_ai(stage, timeline.get_current_time())
            except Exception as e:  # noqa: BLE001
                carb.log_error(f"{LOG} AI driving stopped: {e}")
                self._end_ai(f"AI 운전 오류: {e}", ok=False, stop_timeline=False, result="오류")
        if self._pending_start:
            self._pending_start -= 1
            if self._pending_start:
                return
            problem = self._clear_start(stage, redraw=self._start_redraw)
            if problem:
                self._window.set_recording(False)
                self._window.set_last(f"녹화 시작 불가: {problem}", ok=False)
                timeline.stop()
                return
            try:
                self._recorder.start(stage)
            except Exception as e:  # noqa: BLE001 - show it instead of failing silently
                carb.log_error(f"{LOG} could not start recording: {e}")
                self._window.set_recording(False)
                self._window.set_last(f"녹화를 시작하지 못했습니다: {e}", ok=False)
                return
        if not self._recorder.active:
            return
        try:
            self._recorder.update(stage, timeline.get_current_time())
        except Exception as e:  # noqa: BLE001
            carb.log_error(f"{LOG} recording stopped: {e}")
            self._finish(stop_timeline=False)
            return
        s = self._recorder
        self._window.set_live(f"{timeline.get_current_time():.0f}초 · 기록 {len(s._rows)}개 · 이미지 {s._images}장")
