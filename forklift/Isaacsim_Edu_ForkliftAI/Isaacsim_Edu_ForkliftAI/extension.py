"""지게차 AI: Lesson 1 지게차 장면의 복사본에서 사람의 시연을 녹화하고 AI가 운전하게 한다."""

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
PATHS_FILE = MODELS_DIR / "paths.json"  # --name 없이 실행한 learn_paths.py 결과: '기본' AI 모델
RUNS_DIR = ROOT / "forklift_ai" / "runs"  # YOLO 학습 출력 (runs/<name>/weights/best.onnx)
DEFAULT_MODEL = "기본"


def list_ai_models():
    """패널에 보여 줄 AI 모델 목록: models/paths.json('기본')과 모든 models/<name>/paths.json
    (learn_paths.py --name <name>). YOLO 모델은 models/<name>/best.onnx → 같은 이름의 YOLO run
    (runs/<name>/weights/best.onnx) → 가장 최근 YOLO run 순으로 짝짓는다.
    반환: [{"name", "paths", "yolo", "mtime"}]."""
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
# 이 실습 전용 Lesson 1 장면 복사본(같은 폴더라 asset 참조가 그대로 동작). Lesson 1에 영향 없이 수정 가능.
SCENE_FILE = ROOT / "iw_hub_warehouse_navigation" / "iw_hub_warehouse_forklift_ai.usd"

LOG = "[Isaacsim_Edu_ForkliftAI]"
START_WAIT_FRAMES = 3  # Play 후 이 프레임 수만큼 지나서 녹화 시작: 겹침 검사에 물리 실행이 필요
START_DRAWS = 30  # '랜덤': 무언가와 겹치는 출발 위치는 최대 이 횟수만큼 다시 뽑음
# AZ Edu Lv2 런처에서 Lesson 2 (크래시 복구 시 여기 기록된 레슨을 다시 연다).
SETTING_ACTIVE_LESSON = "/persistent/exts/Isaacsim_Edu_Lv2/active_lesson"
# 패널 키 표시용 Lesson 1 주행 키
KEY_PATHS = {"forward": fon.KEY_FORWARD_PATH, "backward": fon.KEY_BACKWARD_PATH, "left": fon.KEY_LEFT_PATH,
             "right": fon.KEY_RIGHT_PATH, "lift_up": fon.KEY_LIFT_UP_PATH, "lift_down": fon.KEY_LIFT_DOWN_PATH}
# 지게차 출발 위치 = 장면의 기본 출발 자세 + 패널의 X/Y 오프셋 슬라이더(방향은 그대로).
# 장면 배치(지게차 AI 복사본): 선반 앞면 x 0.22 | 파렛트 x 1.22-2.22 | 지게차 x 6.47, 모두 y -1.0.
# 지게차 뒤쪽의 먼 선반은 x 8.1 부근부터 시작.


class IsaacsimEduForkliftAIExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        carb.settings.get_settings().set(SETTING_ACTIVE_LESSON, "Isaacsim_Edu_ForkliftAI")
        self._recorder = DemoRecorder()
        self._last_dir = None
        self._pending_start = False
        self._start_redraw = False  # '랜덤' 녹화: 출발 위치가 무언가와 겹치면 다시 뽑기
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
        self._base_pose = None  # 장면이 지게차를 두는 출발 자세 (x, y, z, yaw_deg)
        self._pallet_base = {}  # prim 경로 -> 파렛트/박스의 (translate op, 장면 값, 부모 world->local)
        self._reapply_frames = 0
        self._driver = None
        self._ai_pending = False
        self._ai_scene_held = False  # 끝난 AI 운전의 일시 정지된 마지막 장면을 Stop 전까지 표시 중
        self._ai_task = None
        self._ai_run_info = None  # 실행 중인 AI 운전의 시작 조건 (AI 운전 기록 표의 한 행)
        # '위치 자유': 손으로 옮겨 놓은 지게차의 (x, y, yaw_deg). 옵션을 켤 때와, 정지 상태에서 녹화/AI 운전을 시작할 때마다
        # 저장. 녹화와 AI 운전은 여기서 출발하고, Stop / 'AI 멈춤'도 슬라이더 자세 대신 여기로 되돌린다
        self._free_pose = None
        self._watch = None  # 패널 갱신용으로 마지막에 본 (시연 폴더, AI 모델)
        self._ai_models = []  # 패널 'AI 모델' 목록에 마지막으로 보여 준 list_ai_models()
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
            self._apply_pallet(stage, back=0.0)  # 파렛트를 장면 파일의 위치로 되돌려 둠
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

    # ---------------------------------------------------------------- 장면

    async def _open_scene(self):
        """이 실습의 장면 복사본을 연다. Lesson 1은 지게차 조작용 의존성일 뿐이며,
        이 익스텐션이 켜져 있으면 자체 장면이나 패널을 열지 않는다."""
        app = omni.kit.app.get_app()
        ctx = omni.usd.get_context()
        for _ in range(10):  # Lesson 1의 장면 열기와 같은 안정화 대기
            await app.next_update_async()
        for _ in range(1800):  # 최대 ~30 s: 다른 장면 로딩 중에 열면 실패 (UsdContext busy)
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
            fon.register_forklift_node_types()  # 장면의 Action Graph가 Lesson 1의 노드 타입을 사용
            for _ in range(5):  # 다른 장면이 아직 로딩 중이면 재시도 ("UsdContext busy")
                ok, _err = await ctx.open_stage_async(str(SCENE_FILE).replace("\\", "/"))
                if ok:
                    break
                for _ in range(60):
                    await app.next_update_async()
        if self._window:
            self._window.deferred_dock_in("AZ Edu Lv2", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

    # ---------------------------------------------------------------- 헬퍼

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
        """재시작 없이 새 녹화와 새로 학습된 AI 모델(paths.json / YOLO)을 반영한다."""
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
        """패널 키 표시에 지금 눌린 주행 키(AI 또는 사람)를 켠다."""
        if not self._window:
            return
        if stage is None:
            if self._keys_shown:
                self._window.set_keys({})
                self._keys_shown = False
            return
        self._window.set_keys({name: fon._is_key_down(stage, path) for name, path in KEY_PATHS.items()})
        self._keys_shown = True

    # ---------------------------------------------------------------- 출발 위치

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.OPENED):
            self._base_pose = None
            self._capture_base_pose()

    def _capture_base_pose(self):
        """장면 파일의 지게차 출발 자세를 기억한다(아무것도 움직이기 전에 읽음)."""
        stage = self._stage()
        if not self._scene_ready(stage) or not omni.timeline.get_timeline_interface().is_stopped():
            return
        prim = stage.GetPrimAtPath(fon.FORKLIFT_PATH)
        pos = prim.GetAttribute("xformOp:translate").Get()
        rot = prim.GetAttribute("xformOp:rotateXYZ").Get()
        self._base_pose = (float(pos[0]), float(pos[1]), float(pos[2]), float(rot[2]))
        self._capture_pallet_base(stage)
        self._refresh_summary()  # 표에 각 녹화의 출발 위치를 이 자세 기준으로 표시
        self._refresh_model_info()  # 이제 패널이 만들어져 있음 (on_startup의 호출은 패널 생성 전)
        self._apply_start(stage)

    def _apply_start(self, stage):
        """지게차를 장면 출발 자세 + 패널 오프셋 위치에 놓는다."""
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
        """장면 파일의 파렛트와 그 위 박스 위치를 기억한다. 지게차 출발 위치처럼 장면 자체 layer에서 편집해 옮긴다:
        session layer에서 옮기면 파렛트 collider가 포크와 충돌하지 않았다(포크가 그대로 통과).
        그때 남은 session layer 오프셋은 먼저 지운다."""
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
        """파렛트(와 박스)를 패널의 '파렛트 뒤로'(또는 `back` m)만큼 지게차에서 멀어지게 똑바로 민다.
        지게차가 장면의 -X를 향하므로 뒤쪽도 -X."""
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
        """'위치 자유': 지금 (손으로 옮겨 놓은) 지게차 위치를 출발·복귀 위치로 기억한다."""
        if self._scene_ready(stage):
            s = forklift_state(stage)
            self._free_pose = (s["x"], s["y"], s["yaw_deg"])

    def _reset_start(self, stage):
        """정지 상태에서 지게차를 둘 위치: '위치 자유'가 켜져 있으면 그 자세, 아니면 슬라이더 자세."""
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
            self._apply_start(stage)  # 슬라이더 자세로 복귀

    def _place_forklift_free(self, stage):
        """'위치 자유': 기억한 자세에 지게차를 놓는다. x, y, 방향만 유지하고 높이는 장면 값, 자세는 똑바로."""
        prim = stage.GetPrimAtPath(fon.FORKLIFT_PATH)
        if not prim.IsValid() or self._base_pose is None:
            return
        x, y, yaw = self._free_pose
        rot = prim.GetAttribute("xformOp:rotateXYZ").Get()
        prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(x, y, self._base_pose[2]))
        prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(rot[0], rot[1], yaw))
        self._apply_pallet(stage)

    def _clear_start(self, stage, redraw):
        """시뮬레이션 실행 중에 호출(겹침 검사가 물리 scene에 묻기 때문). 지게차가 아무것과도 겹치지 않으면 None,
        아니면 출발할 수 없는 이유(한국어). `redraw`('랜덤')이면 겹치지 않을 때까지 새 랜덤 출발 위치를 뽑아
        지게차를 옮긴다."""
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
            if self._window.free_start:  # '위치 자유': 손으로 놓은 위치 그대로
                self._apply_pallet(stage)
            else:
                self._apply_start(stage)  # 미리보기: 슬라이더를 움직이면 지게차도 따라 움직임

    # ---------------------------------------------------------------- 버튼

    def _on_start(self):
        if self._ai_scene_held or (self._driver is not None and (self._driver.active or self._ai_pending)):
            return  # AI 운전 중이거나 그 마지막 장면을 유지하는 동안은 녹화 불가 (버튼도 꺼져 있음)
        stage = self._stage()
        if not self._scene_ready(stage):
            self._window.set_last("지게차 AI 장면(파렛트·선반 포함)이 열려 있지 않습니다.", ok=False)
            return
        timeline = omni.timeline.get_timeline_interface()
        self._start_redraw = False
        if not timeline.is_playing():
            # 사람의 시연은 항상 장면 기본 파렛트 위치에서 녹화: 거기서 학습한 경로는 그 파렛트 위치에만 맞음
            if self._window.free_start:  # '위치 자유': 손으로 놓은 위치에서 녹화
                if timeline.is_stopped():
                    self._capture_free_pose(stage)
            self._window.set_pallet_back(0.0)
            if self._window.random_start:
                self._start_redraw = True  # 물리가 돈 뒤 무언가와 겹치면 다시 뽑음
                self._window.randomize_start()  # '랜덤': 녹화마다 새 출발 위치, 슬라이더에 표시
            self._reset_start(stage)
            viewport = vp_util.get_active_viewport()
            if viewport is not None:
                fon.reset_camera_to_home(stage, fon.DRIVER_VIEW_CAMERA_PATH)
                viewport.camera_path = Sdf.Path(fon.DRIVER_VIEW_CAMERA_PATH)
            fon.reset_drive_state()
            timeline.play()
        self._pending_start = START_WAIT_FRAMES  # 시뮬레이션(과 겹침 검사)이 돌기 시작하면 녹화 시작
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

    # ---------------------------------------------------------------- AI 운전

    def _selected_model(self):
        """패널에서 고른 AI 모델(list_ai_models() 항목), 없으면 None."""
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
            self._toggle_ai_pause()  # 운전 중에는 이 버튼이 'AI 일시 정지' / 'AI 다시 시작'
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
            # '랜덤': AI 운전마다 새 출발 위치(와 파렛트 위치). '위치 자유'면 파렛트 위치만
            self._window.randomize_start(pallet=True, forklift=not self._window.free_start)
        if self._driver is not None:
            self._driver.stop()
        eyes = self._window.eyes_mode
        model = selected["yolo"] if eyes == "yolo" else None
        try:
            # 학습된 모델 없이 'YOLO로 파렛트 거리 재기'를 켜면 끈 것처럼 운전 (패널에 안내)
            self._driver = AIDriver(selected["paths"], model, blind=eyes == "none" or model is None,
                                    reposition=self._window.allow_reposition)
        except Exception as e:  # noqa: BLE001 - 예: ONNX 모델 로드 실패
            carb.log_error(f"{LOG} could not start the AI driver: {e}")
            self._window.set_ai(False, f"AI를 준비하지 못했습니다: {e}", ok=False)
            self._driver = None
            return
        self._refresh_model_info()
        dx, dy = self._window.start_offset
        yaw = (self._base_pose[3] if self._base_pose else 0.0) + self._window.start_yaw
        if self._window.free_start and self._base_pose:  # '위치 자유': 손으로 놓은 위치에서 출발
            if omni.timeline.get_timeline_interface().is_stopped() or self._free_pose is None:
                self._capture_free_pose(stage)  # (일시 정지된 마지막 장면이면 직전 운전의 출발 자세 유지)
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
        """AI 운전은 항상 깨끗한 장면에서 시작: Stop(Lesson 1이 지게차를, 물리가 파렛트를 리셋)하고
        그 stop 이벤트가 처리되길 기다린 뒤 지게차를 놓고 Play."""
        app = omni.kit.app.get_app()
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_stopped():
            timeline.stop()
            for _ in range(5):
                await app.next_update_async()
        stage = self._stage()
        self._reset_start(stage)  # Stop이 장면 출발점으로 되돌렸으니 슬라이더 자세나 '위치 자유' 자세로 다시 놓음
        viewport = vp_util.get_active_viewport()
        if viewport is not None:
            fon.reset_camera_to_home(stage, fon.DRIVER_VIEW_CAMERA_PATH)
            viewport.camera_path = Sdf.Path(fon.DRIVER_VIEW_CAMERA_PATH)
        fon.reset_drive_state()
        self._window.set_ai(True, "출발 전에 학습한 경로들을 머릿속으로 연습해 보는 중...")
        timeline.play()  # 리허설의 충돌 검사가 실행 중인 물리 scene에 묻기 때문
        for _ in range(3):
            await app.next_update_async()
        if self._driver is None or not timeline.is_playing():
            return
        redraw = self._window.random_start and not self._window.free_start
        problem = self._clear_start(stage, redraw)
        if problem:
            self._end_ai(f"실패: {problem}", ok=False, stop_timeline=True, result="출발 불가")
            return
        if redraw and self._ai_run_info:  # 출발 위치를 다시 뽑았을 수 있음: 실제 출발 위치를 기록
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
                self._ai_task.cancel()  # 취소하지 않으면 나중에라도 plan 결과를 driver에 넘김
            self._end_ai("사용자가 멈춤", ok=None, stop_timeline=False)
        if not timeline.is_stopped():
            timeline.stop()  # Stop이 지게차와 파렛트를 되돌림 (끝나서 일시 정지된 운전 후에도)

    def _toggle_ai_pause(self):
        """AI 운전 중 시작 버튼은 시뮬레이션을 일시 정지 / 재개한다(AI도 함께 대기)."""
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
            omni.timeline.get_timeline_interface().pause()  # 마지막 장면을 보여 줌; Stop하면 리셋
            self._ai_scene_held = True
            if self._window:
                self._window.set_scene_stop(True)  # 이제 'AI 멈춤'이 일시 정지된 장면을 정지(리셋)

    # ---------------------------------------------------------------- AI 운전 기록 표

    def _log_ai_run(self, result, text):
        """AI 운전 1회당 ai_runs.csv에 한 행 (운전의 첫 종료만 기록)."""
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
        """선택한 AI 모델 폴더(models\\<name>, '기본'이면 models)를 Windows 탐색기로 연다.
        모델이 아직 없으면 models\\가 생길 forklift_ai 폴더를 연다."""
        model = self._selected_model()
        folder = Path(model["paths"]).parent if model else (MODELS_DIR if MODELS_DIR.exists() else MODELS_DIR.parent)
        if folder.exists():
            os.startfile(os.path.normpath(str(folder)))

    # ---------------------------------------------------------------- 녹화

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
        self._reapply_frames = 3  # Stop 시 Lesson 1이 지게차를 장면 출발점으로 되돌리므로 오프셋을 다시 적용

    def _on_update(self, _event):
        timeline = omni.timeline.get_timeline_interface()
        now = time.monotonic()
        if now - self._watch_t >= 1.0:
            self._watch_t = now
            self._watch_files()
        self._show_keys(self._stage() if timeline.is_playing() else None)
        if self._window:  # Isaac Sim 툴바의 일시 정지 / 재생도 반영
            driving = self._driver is not None and self._driver.active
            self._window.set_ai_pause_button(driving, paused=driving and not timeline.is_playing())
        if not timeline.is_playing():
            if self._driver is not None and self._driver.active and any(self._driver.keys.values()):
                # Lesson 1은 일시 정지 중에도 매 앱 프레임 지게차를 움직임: 키를 모두 떼고 정지
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
            except Exception as e:  # noqa: BLE001 - 조용히 실패하지 않고 표시
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
