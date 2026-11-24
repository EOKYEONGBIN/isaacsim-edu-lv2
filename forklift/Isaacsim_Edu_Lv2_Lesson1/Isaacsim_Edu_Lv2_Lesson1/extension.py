import asyncio
from pathlib import Path

import carb
import carb.settings
import omni.ext
import omni.kit.app
import omni.kit.viewport.utility as vp_util
import omni.timeline
import omni.ui as ui
import omni.usd
from pxr import Gf, Sdf, UsdGeom

from .forklift_omnigraph_nodes import (
    ALL_WHEEL_PATHS,
    CAMERA_HOME_TRANSFORMS,
    DRIVER_VIEW_CAMERA_PATH,
    FORKLIFT_PATH,
    FRONT_WHEEL_PATHS,
    LEFT_VIEW_CAMERA_PATH,
    LIFT_PART_REST_Z,
    RIGHT_VIEW_CAMERA_PATH,
    STEERING_WHEEL_HANDLE_PATH,
    STEERING_WHEEL_HANDLE_REST_TRANSLATE,
    STEERING_WHEEL_PATH,
    deregister_forklift_node_types,
    register_forklift_node_types,
    reset_drive_state,
)
from .window import ForkliftSimWindow

_ROOT = Path(__file__).resolve().parents[4]


def get_scene_usd_path():
    return str(_ROOT / "iw_hub_warehouse_navigation" / "iw_hub_warehouse_navigation.usd").replace("\\", "/")


_LEGACY_TPP_CAMERA_PATH = f"{FORKLIFT_PATH}/TPPCamera"
DRIVER_VIEW_LOCAL_TRANSFORM = Gf.Matrix4d(
    -0.9994606284811628, -0.03283979470168061, -8.737043206974437e-16, 0.0,
    0.013538644602422103, -0.4120410120111574, 0.911064712039244, 0.0,
    -0.029919178103314896, 0.9105733096817492, 0.4122633751339673, 0.0,
    0.0241911878993356, -0.2454424076963973, 1.6669631622405576, 1.0,
)
LEFT_VIEW_LOCAL_TRANSFORM = Gf.Matrix4d(
    -0.406742177958416, 0.9135429933339997, 5.3568258588273495e-15, 0.0,
    -0.3094151882896358, -0.13776276376336036, 0.9408951387779457, 0.0,
    0.8595481614926144, 0.382701737977026, 0.338698003867797, 0.0,
    2.590756928255887, -1.2052126210713645, 1.5234583656432588, 1.0,
)
RIGHT_VIEW_LOCAL_TRANSFORM = Gf.Matrix4d(
    -0.4067421779584143, -0.9135429933340007, 2.775557561562892e-17, 0.0,
    0.3094151882896382, -0.13776276376335514, 0.940895138777946, -0.0,
    -0.8595481614926146, 0.3827017379770261, 0.338698003867797, 0.0,
    -2.5907569282558875, -1.2052126210713647, 1.5234583656432594, 1.0,
)
VEHICLE_CAMERA_FOCAL_LENGTH_MM = 8.8
VEHICLE_CAMERA_CLIPPING_RANGE = Gf.Vec2f(0.05, 1000.0)


def _hide_camera_manipulator_mesh(stage, camera_path):
    mesh_prim = stage.GetPrimAtPath(f"{camera_path}/OmniverseKitViewportCameraMesh")
    if not mesh_prim.IsValid():
        return
    UsdGeom.Imageable(mesh_prim).GetPurposeAttr().Set(UsdGeom.Tokens.guide)
    model_prim = stage.GetPrimAtPath(f"{camera_path}/OmniverseKitViewportCameraMesh/CameraModel")
    if model_prim.IsValid():
        UsdGeom.Imageable(model_prim).GetPurposeAttr().Set(UsdGeom.Tokens.guide)


# 이 레슨은 Lv1 Lesson 3 복사본이다. 둘 다 지게차 씬의 Action Graph 노드 타입을 같은 이름으로 등록하므로
# (씬 파일이 그 이름을 참조), 이 레슨이 켜지면 Lv1 레슨은 모두 끈다.
LV1_LESSON_MODULES = [
    "Isaacsim_Edu_Lv1_Lesson1",
    "Isaacsim_Edu_Lv1_Lesson2",
    "Isaacsim_Edu_Lv1_Lesson3",
    "Isaacsim_Edu_Lv1_Lesson4",
    "Isaacsim_Edu_Lv1_Lesson5",
]

SETTING_ACTIVE_LESSON = "/persistent/exts/Isaacsim_Edu_Lv2/active_lesson"

# Lesson 2(지게차 AI)는 지게차 조작을 이 익스텐션에 의존하지만 씬과 패널은 따로 연다.
# Lesson 2가 켜져 있으면 이 레슨은 조작 기능만 유지한다.
FORKLIFT_AI_MODULE = "Isaacsim_Edu_ForkliftAI"


class IsaacsimEduLv2Lesson1Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        manager = omni.kit.app.get_app().get_extension_manager()
        for module in LV1_LESSON_MODULES:
            if manager.is_extension_enabled(module):
                manager.set_extension_enabled_immediate(module, False)
        carb.settings.get_settings().set(SETTING_ACTIVE_LESSON, "Isaacsim_Edu_Lv2_Lesson1")

        self._forklift_spawn_translate = None
        self._forklift_spawn_rotate = None
        self._camera_before_tpp = None

        timeline_events = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._timeline_stop_sub = timeline_events.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self._on_timeline_stop
        )
        self._timeline_play_sub = timeline_events.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self._on_timeline_play
        )
        # 나중에 이 씬의 복사본을 열어도(예: 다른 레슨) 시작 자세와 카메라를 다시 잡는다.
        self._stage_open_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="Isaacsim_Edu_Lv2_Lesson1 stage"
        )

        self._window = ForkliftSimWindow(
            reset_clicked_fn=self._reset_forklift_pose,
            start_clicked_fn=self._on_start_clicked,
            stop_clicked_fn=self._on_stop_clicked,
        )
        self._window.set_running(omni.timeline.get_timeline_interface().is_playing())

        self._open_task = asyncio.ensure_future(self._open_scene_and_play())

    def on_shutdown(self):
        deregister_forklift_node_types()

        if self._open_task is not None and not self._open_task.done():
            self._open_task.cancel()
        self._open_task = None

        self._timeline_stop_sub = None
        self._timeline_play_sub = None
        self._stage_open_sub = None

        if self._window:
            self._window.visible = False
            self._window.destroy()
            self._window = None

        self._forklift_spawn_translate = None
        self._forklift_spawn_rotate = None
        self._camera_before_tpp = None

    async def _open_scene_and_play(self):
        app = omni.kit.app.get_app()
        for _ in range(10):
            await app.next_update_async()
        register_forklift_node_types()

        if omni.kit.app.get_app().get_extension_manager().is_extension_enabled(FORKLIFT_AI_MODULE):
            if self._window:
                self._window.visible = False
                self._window.destroy()
                self._window = None
            return

        await omni.usd.get_context().open_stage_async(get_scene_usd_path())
        if self._window:
            self._window.deferred_dock_in("AZ Edu Lv2", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        self._capture_forklift_spawn_pose()

    def _on_timeline_stop(self, event):
        reset_drive_state()
        self._reset_forklift_pose()
        self._restore_original_camera()
        if self._window:
            self._window.set_running(False)

    def _on_timeline_play(self, event):
        if self._window:
            self._window.set_running(True)

    def _on_start_clicked(self):
        omni.usd.get_context().get_selection().clear_selected_prim_paths()

        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            self._ensure_driver_view_camera(stage)
            viewport = vp_util.get_active_viewport()
            if viewport is not None:
                current_camera = viewport.camera_path
                if current_camera is not None and str(current_camera) != DRIVER_VIEW_CAMERA_PATH:
                    self._camera_before_tpp = current_camera
                viewport.camera_path = Sdf.Path(DRIVER_VIEW_CAMERA_PATH)

        omni.timeline.get_timeline_interface().play()

    def _on_stop_clicked(self):
        omni.timeline.get_timeline_interface().stop()

    def _ensure_driver_view_camera(self, stage):
        legacy_prim = stage.GetPrimAtPath(_LEGACY_TPP_CAMERA_PATH)
        if legacy_prim.IsValid():
            stage.RemovePrim(_LEGACY_TPP_CAMERA_PATH)
        self._ensure_camera(stage, DRIVER_VIEW_CAMERA_PATH, DRIVER_VIEW_LOCAL_TRANSFORM)

    @staticmethod
    def _ensure_camera(stage, camera_path, local_transform):
        prim = stage.GetPrimAtPath(camera_path)
        camera = UsdGeom.Camera(prim) if prim.IsValid() else UsdGeom.Camera.Define(stage, camera_path)
        camera.GetFocalLengthAttr().Set(VEHICLE_CAMERA_FOCAL_LENGTH_MM)
        camera.GetClippingRangeAttr().Set(VEHICLE_CAMERA_CLIPPING_RANGE)
        xformable = UsdGeom.Xformable(camera.GetPrim())
        xformable.ClearXformOpOrder()
        xformable.AddTransformOp().Set(local_transform)
        CAMERA_HOME_TRANSFORMS[camera_path] = local_transform
        _hide_camera_manipulator_mesh(stage, camera_path)

    def _restore_original_camera(self):
        if self._camera_before_tpp is None:
            return
        viewport = vp_util.get_active_viewport()
        if viewport is not None:
            viewport.camera_path = self._camera_before_tpp
        self._camera_before_tpp = None

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.OPENED):
            self._capture_forklift_spawn_pose()

    def _capture_forklift_spawn_pose(self):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(FORKLIFT_PATH) if stage else None
        if prim is None or not prim.IsValid():
            return
        self._forklift_spawn_translate = prim.GetAttribute("xformOp:translate").Get()
        self._forklift_spawn_rotate = prim.GetAttribute("xformOp:rotateXYZ").Get()
        self._ensure_driver_view_camera(stage)
        self._ensure_camera(stage, LEFT_VIEW_CAMERA_PATH, LEFT_VIEW_LOCAL_TRANSFORM)
        self._ensure_camera(stage, RIGHT_VIEW_CAMERA_PATH, RIGHT_VIEW_LOCAL_TRANSFORM)

    def _reset_forklift_pose(self):
        if self._forklift_spawn_translate is None or self._forklift_spawn_rotate is None:
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        try:
            forklift = stage.GetPrimAtPath(FORKLIFT_PATH)
            if forklift.IsValid():
                forklift.GetAttribute("xformOp:translate").Set(self._forklift_spawn_translate)
                forklift.GetAttribute("xformOp:rotateXYZ").Set(self._forklift_spawn_rotate)
            for path in ALL_WHEEL_PATHS:
                wheel = stage.GetPrimAtPath(path)
                roll_attr = wheel.GetAttribute("xformOp:rotateX") if wheel.IsValid() else None
                if roll_attr and roll_attr.IsValid():
                    roll_attr.Set(0.0)
            for path in FRONT_WHEEL_PATHS:
                wheel = stage.GetPrimAtPath(path)
                steer_attr = wheel.GetAttribute("xformOp:rotateZ") if wheel.IsValid() else None
                if steer_attr and steer_attr.IsValid():
                    steer_attr.Set(0.0)
            steering_wheel = stage.GetPrimAtPath(STEERING_WHEEL_PATH)
            wheel_rotate_attr = steering_wheel.GetAttribute("xformOp:rotateZ") if steering_wheel.IsValid() else None
            if wheel_rotate_attr and wheel_rotate_attr.IsValid():
                wheel_rotate_attr.Set(0.0)
            handle = stage.GetPrimAtPath(STEERING_WHEEL_HANDLE_PATH)
            if handle.IsValid():
                handle_rotate_attr = handle.GetAttribute("xformOp:rotateZ")
                if handle_rotate_attr and handle_rotate_attr.IsValid():
                    handle_rotate_attr.Set(0.0)
                handle_translate_attr = handle.GetAttribute("xformOp:translate")
                if handle_translate_attr and handle_translate_attr.IsValid():
                    handle_translate_attr.Set(STEERING_WHEEL_HANDLE_REST_TRANSLATE)
            for path, rest_z in LIFT_PART_REST_Z.items():
                part = stage.GetPrimAtPath(path)
                if not part.IsValid():
                    continue
                translate_attr = part.GetAttribute("xformOp:translate")
                pos = translate_attr.Get()
                translate_attr.Set(Gf.Vec3f(pos[0], pos[1], rest_z))
        except Exception as e:
            carb.log_warn(f"[Isaacsim_Edu_Lv2_Lesson1] Could not reset forklift pose: {e}")
