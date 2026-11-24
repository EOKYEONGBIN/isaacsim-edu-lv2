import math
import re

import carb
import carb.input
import omni.appwindow
import omni.graph.core as og
import omni.graph.core.types as ot
import omni.kit.viewport.utility as vp_util
import omni.physx
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

FORKLIFT_PATH = "/World/Forklift"
KEY_FORWARD_PATH = "/World/ForkliftControls/KeyForward"
KEY_BACKWARD_PATH = "/World/ForkliftControls/KeyBackward"
KEY_LEFT_PATH = "/World/ForkliftControls/KeyLeft"
KEY_RIGHT_PATH = "/World/ForkliftControls/KeyRight"
KEY_LIFT_UP_PATH = "/World/ForkliftControls/KeyLiftUp"
KEY_LIFT_DOWN_PATH = "/World/ForkliftControls/KeyLiftDown"
KEY_CAMERA_CYCLE_PATH = "/World/ForkliftControls/KeyCameraCycle"
KEYBOARD_INPUT_NODE_PATHS = [
    KEY_FORWARD_PATH,
    KEY_BACKWARD_PATH,
    KEY_LEFT_PATH,
    KEY_RIGHT_PATH,
    KEY_LIFT_UP_PATH,
    KEY_LIFT_DOWN_PATH,
    KEY_CAMERA_CYCLE_PATH,
]
GEOMETRY_PATH = f"{FORKLIFT_PATH}/Geometry"

DRIVER_VIEW_CAMERA_PATH = f"{FORKLIFT_PATH}/DriverViewCamera"
LEFT_VIEW_CAMERA_PATH = f"{FORKLIFT_PATH}/LeftViewCamera"
RIGHT_VIEW_CAMERA_PATH = f"{FORKLIFT_PATH}/RightViewCamera"
CAMERA_CYCLE_PATHS = [DRIVER_VIEW_CAMERA_PATH, LEFT_VIEW_CAMERA_PATH, RIGHT_VIEW_CAMERA_PATH]
# 차량 카메라별 장착 위치(지게차 기준 transform). 카메라 생성 시 등록한다.
# 카메라 전환 시 이 위치로 되돌려, 뷰포트 조작으로 옮겨진 카메라가 다음 순환 때 그대로 남지 않게 한다.
CAMERA_HOME_TRANSFORMS = {}
FORK_PATHS = [f"{GEOMETRY_PATH}/sm_fork1_c01_obj_00", f"{GEOMETRY_PATH}/sm_fork2_c01_obj_00"]
MAST_LIFT_PATHS = [f"{GEOMETRY_PATH}/sm_mast_stage3_c01_obj_00", f"{GEOMETRY_PATH}/sm_mast_stage4_c01_obj_00"]
FRONT_WHEEL_PATHS = [f"{GEOMETRY_PATH}/sm_wheel3_c01_obj_00", f"{GEOMETRY_PATH}/sm_wheel4_c01_obj_00"]
REAR_WHEEL_PATHS = [f"{GEOMETRY_PATH}/sm_wheel1_c01_obj_00", f"{GEOMETRY_PATH}/sm_wheel2_c01_obj_00"]
ALL_WHEEL_PATHS = FRONT_WHEEL_PATHS + REAR_WHEEL_PATHS
STEERING_WHEEL_PATH = f"{GEOMETRY_PATH}/sm_steering_wheel_c01_obj_00"
STEERING_WHEEL_SIGN = 1.0

STEERING_WHEEL_HANDLE_PATH = f"{GEOMETRY_PATH}/sm_steering_wheel_handle_c01_obj_00"
STEERING_WHEEL_REST_TRANSLATE = Gf.Vec3f(0.1119375229, -0.3895743489, 1.332611799)
STEERING_WHEEL_HANDLE_REST_TRANSLATE = Gf.Vec3f(0.01662363857, -0.4356105626, 1.430831671)
STEERING_WHEEL_TILT = Gf.Rotation(Gf.Quatd(0.9499911146, Gf.Vec3d(-0.312276932, 0.0, 0.0)))
STEERING_WHEEL_HANDLE_OFFSET_LOCAL = Gf.Vec3d(-0.09531388433, -0.09533346966261369, 0.051749452070683544)

FORWARD_SPEED_MPS = 1.2
REVERSE_SPEED_MPS = 0.8
DRIVE_ACCEL_MPS2 = 0.8
DRIVE_COAST_DECEL_MPS2 = 2.5
DRIVE_BRAKE_DECEL_MPS2 = 5.0
TURN_SPEED_DEG_PER_SEC = 30.0
TURN_ACCEL_DEG_PER_SEC2 = 60.0
TURN_DECEL_DEG_PER_SEC2 = 90.0
LIFT_SPEED_MPS = 0.3
LIFT_TRAVEL_M = 1.5
LIFT_MAX_DELTA_SECONDS = 1.0 / 60.0
LIFT_PART_REST_Z = {
    FORK_PATHS[0]: 0.6023,
    FORK_PATHS[1]: 0.6023,
    MAST_LIFT_PATHS[0]: 0.2080,
    MAST_LIFT_PATHS[1]: 0.2312,
}
LIFT_FLOOR_OFFSET_M = -0.1
WHEEL_RADIUS_M = 0.29
MAX_STEER_ANGLE_DEG = 30.0
STEER_SPEED_DEG_PER_SEC = 90.0

_REGISTERED_NODE_TYPES = []

_current_speed_mps = 0.0
_current_turn_rate_deg_per_sec = 0.0
_was_window_focused = True
_FOCUS_DEBOUNCE_FRAMES = 6
_pending_focus_state = True
_focus_mismatch_streak = 0
_camera_cycle_index = 0
_camera_cycle_key_was_down = False


def reset_drive_state():
    global _current_speed_mps, _current_turn_rate_deg_per_sec
    global _was_window_focused, _pending_focus_state, _focus_mismatch_streak
    global _camera_cycle_index, _camera_cycle_key_was_down
    global _vehicle_boxes
    _vehicle_boxes = None
    _pushable_cache.clear()
    _current_speed_mps = 0.0
    _current_turn_rate_deg_per_sec = 0.0
    _was_window_focused = True
    _pending_focus_state = True
    _focus_mismatch_streak = 0
    _camera_cycle_index = 0
    _camera_cycle_key_was_down = False


def _clear_stuck_keys_on_focus_change(stage):
    global _was_window_focused, _pending_focus_state, _focus_mismatch_streak
    app_window = omni.appwindow.get_default_app_window()
    if app_window is None:
        return
    focused = app_window.is_focused()
    if focused == _was_window_focused:
        _focus_mismatch_streak = 0
        return
    if focused != _pending_focus_state:
        _pending_focus_state = focused
        _focus_mismatch_streak = 0
    _focus_mismatch_streak += 1
    if _focus_mismatch_streak < _FOCUS_DEBOUNCE_FRAMES:
        return
    _was_window_focused = focused
    _focus_mismatch_streak = 0
    for path in KEYBOARD_INPUT_NODE_PATHS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        attr = prim.GetAttribute("outputs:isPressed")
        if attr and attr.Get():
            attr.Set(False)


_keyboard_input_iface = None
_keyboard_device = None
_key_enum_cache = {}


def _keyboard_input():
    global _keyboard_input_iface, _keyboard_device
    if _keyboard_input_iface is None:
        _keyboard_input_iface = carb.input.acquire_input_interface()
    if _keyboard_device is None:
        app_window = omni.appwindow.get_default_app_window()
        if app_window is not None:
            _keyboard_device = app_window.get_keyboard()
    return _keyboard_input_iface, _keyboard_device


def _key_enum_from_name(name):
    if name in _key_enum_cache:
        return _key_enum_cache[name]
    snake = re.sub(r"(?<!^)(?=[A-Z])", "_", name).upper()
    enum_value = getattr(carb.input.KeyboardInput, snake, None)
    if enum_value is None:
        carb.log_warn(
            f"[Isaacsim_Edu_Lv2_Lesson1] Unrecognized key name '{name}' "
            f"(looked up as '{snake}') on a forklift control -- treating it as never pressed"
        )
    _key_enum_cache[name] = enum_value
    return enum_value


def _is_key_down(stage, node_path):
    prim = stage.GetPrimAtPath(node_path)
    if not prim.IsValid():
        return False
    key_in = prim.GetAttribute("inputs:keyIn").Get()
    if not key_in:
        return False
    key_enum = _key_enum_from_name(str(key_in))
    if key_enum is None:
        return False
    input_iface, keyboard = _keyboard_input()
    if keyboard is None:
        return False
    return bool(input_iface.get_keyboard_button_flags(keyboard, key_enum) & carb.input.BUTTON_FLAG_DOWN)


def _step_toward(current, target, max_delta):
    if current < target:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


def _ensure_rotate_ops(prim, axes):
    xformable = UsdGeom.Xformable(prim)
    existing = {op.GetOpName() for op in xformable.GetOrderedXformOps()}
    for axis in axes:
        op_name = f"xformOp:rotate{axis}"
        if op_name not in existing:
            op = getattr(xformable, f"AddRotate{axis}Op")()
            op.Set(0.0)
            existing.add(op_name)


GROUND_PLANE_PATH = "/Warehouse/GroundPlane"
GROUND_CLEARANCE_M = 0.1

BODY_PATH = f"{GEOMETRY_PATH}/sm_body_c01_obj_00"
MAST_FIXED_PATHS = [
    f"{GEOMETRY_PATH}/sm_mast_stage1_c01_obj_00",
    f"{GEOMETRY_PATH}/sm_mast_stage2_c01_obj_00",
]
VEHICLE_BOX_GROUPS = [
    ([BODY_PATH], False),
    (list(ALL_WHEEL_PATHS), False),
    (MAST_FIXED_PATHS, False),
    ([MAST_LIFT_PATHS[0]], True),
    ([MAST_LIFT_PATHS[1]], True),
    ([FORK_PATHS[0]], True),
    ([FORK_PATHS[1]], True),
]

_vehicle_boxes = None
_pushable_cache = {}

def _is_pushable(stage, path):
    cached = _pushable_cache.get(path)
    if cached is not None:
        return cached
    prim = stage.GetPrimAtPath(path)
    pushable = False
    if prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_body = UsdPhysics.RigidBodyAPI(prim)
        enabled = rigid_body.GetRigidBodyEnabledAttr().Get()
        kinematic = rigid_body.GetKinematicEnabledAttr().Get()
        pushable = bool(enabled) and not bool(kinematic)
    _pushable_cache[path] = pushable
    return pushable


def _box_overlaps_obstacle(stage, x, y, yaw_rad, half_extent, local_center):
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    lcx, lcy, lcz = local_center
    center = (
        x + lcx * cos_yaw - lcy * sin_yaw,
        y + lcx * sin_yaw + lcy * cos_yaw,
        lcz,
    )
    quat = (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))

    exclude_prefixes = [FORKLIFT_PATH, GROUND_PLANE_PATH]

    blocked = False

    def report_hit(hit):
        nonlocal blocked
        path = str(hit.rigid_body)
        if any(path == prefix or path.startswith(prefix + "/") for prefix in exclude_prefixes):
            return True
        if _is_pushable(stage, path):
            return True
        blocked = True
        return False

    query = omni.physx.get_physx_scene_query_interface()
    query.overlap_box(half_extent, center, quat, report_hit, False)
    return blocked


def _lift_offset(stage):
    prim = stage.GetPrimAtPath(FORK_PATHS[0])
    if not prim.IsValid():
        return 0.0
    translate = prim.GetAttribute("xformOp:translate").Get()
    if translate is None:
        return 0.0
    return translate[2] - LIFT_PART_REST_Z[FORK_PATHS[0]]


def _measure_vehicle_boxes(stage):
    forklift = stage.GetPrimAtPath(FORKLIFT_PATH)
    if not forklift.IsValid():
        return None
    # 각 메시의 local extent로 지게차 좌표계에서 직접 측정한다. BBoxCache는 월드 축 정렬 박스라
    # 지게차 방향에 따라 커지고(300 deg에서 약 2배) 실제로는 비어 있는 이동까지 막는다.
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world_to_forklift = xform_cache.GetLocalToWorldTransform(forklift).GetInverse()
    lift_offset = _lift_offset(stage)
    boxes = []
    for paths, follows_lift in VEHICLE_BOX_GROUPS:
        low = high = None
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            for part in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
                if not part.IsA(UsdGeom.Boundable) or UsdGeom.Imageable(part).ComputePurpose() != UsdGeom.Tokens.default_:
                    continue
                boundable = UsdGeom.Boundable(part)
                extent = boundable.GetExtentAttr().Get() or UsdGeom.Boundable.ComputeExtentFromPlugins(
                    boundable, Usd.TimeCode.Default()
                )
                if not extent:
                    continue
                to_forklift = xform_cache.GetLocalToWorldTransform(part) * world_to_forklift
                aligned = Gf.BBox3d(Gf.Range3d(Gf.Vec3d(*extent[0]), Gf.Vec3d(*extent[1])), to_forklift).ComputeAlignedRange()
                lo, hi = aligned.GetMin(), aligned.GetMax()
                low = [min(low[i], lo[i]) for i in range(3)] if low else [lo[0], lo[1], lo[2]]
                high = [max(high[i], hi[i]) for i in range(3)] if high else [hi[0], hi[1], hi[2]]
        if low is None:
            continue
        if follows_lift:
            low[2] -= lift_offset
            high[2] -= lift_offset
        boxes.append((
            ((high[0] - low[0]) / 2.0, (high[1] - low[1]) / 2.0, (high[2] - low[2]) / 2.0),
            ((high[0] + low[0]) / 2.0, (high[1] + low[1]) / 2.0, (high[2] + low[2]) / 2.0),
            follows_lift,
        ))
    return boxes or None


def _ensure_vehicle_boxes(stage):
    global _vehicle_boxes
    if _vehicle_boxes is None:
        _vehicle_boxes = _measure_vehicle_boxes(stage)
    return _vehicle_boxes


def _pose_blocked(stage, x, y, yaw_rad, lift_offset=None):
    boxes = _ensure_vehicle_boxes(stage)
    if not boxes:
        return False
    if lift_offset is None:
        lift_offset = _lift_offset(stage)
    for half_extent, center, follows_lift in boxes:
        cz = center[2] + lift_offset if follows_lift else center[2]
        hz = half_extent[2]
        bottom = cz - hz
        if bottom < GROUND_CLEARANCE_M:
            shrink = (GROUND_CLEARANCE_M - bottom) / 2.0
            hz -= shrink
            cz += shrink
            if hz <= 0.0:
                continue
        if _box_overlaps_obstacle(
            stage, x, y, yaw_rad, (half_extent[0], half_extent[1], hz), (center[0], center[1], cz)
        ):
            return True
    return False


def _refuse_pose(stage, from_pose, to_pose):
    if not _pose_blocked(stage, *to_pose):
        return False
    return not _pose_blocked(stage, *from_pose)


def _drive_forklift(is_forward_pressed, is_backward_pressed, delta_seconds):
    global _current_speed_mps
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(FORKLIFT_PATH)
    if not prim.IsValid():
        return
    _clear_stuck_keys_on_focus_change(stage)

    if is_forward_pressed and not is_backward_pressed:
        desired_dir = 1.0
    elif is_backward_pressed and not is_forward_pressed:
        desired_dir = -1.0
    else:
        desired_dir = 0.0

    if desired_dir != 0.0 and _current_speed_mps * desired_dir < 0.0:
        _current_speed_mps = _step_toward(_current_speed_mps, 0.0, DRIVE_BRAKE_DECEL_MPS2 * delta_seconds)
    elif desired_dir == 0.0:
        _current_speed_mps = _step_toward(_current_speed_mps, 0.0, DRIVE_COAST_DECEL_MPS2 * delta_seconds)
    else:
        target = desired_dir * (FORWARD_SPEED_MPS if desired_dir > 0.0 else REVERSE_SPEED_MPS)
        _current_speed_mps = _step_toward(_current_speed_mps, target, DRIVE_ACCEL_MPS2 * delta_seconds)

    if _current_speed_mps == 0.0:
        return

    translate_attr = prim.GetAttribute("xformOp:translate")
    rotate_attr = prim.GetAttribute("xformOp:rotateXYZ")
    pos = translate_attr.Get()
    yaw_rad = math.radians(rotate_attr.Get()[2])
    dist = _current_speed_mps * delta_seconds
    dx = math.sin(yaw_rad) * dist
    dy = -math.cos(yaw_rad) * dist
    new_x, new_y = pos[0] + dx, pos[1] + dy
    if _refuse_pose(stage, (pos[0], pos[1], yaw_rad, None), (new_x, new_y, yaw_rad, None)):
        _current_speed_mps = 0.0
        return
    translate_attr.Set(Gf.Vec3f(new_x, new_y, pos[2]))
    _roll_wheels(stage, _current_speed_mps, delta_seconds)


def _roll_wheels(stage, speed_mps, delta_seconds):
    spin_delta_deg = math.degrees((speed_mps / WHEEL_RADIUS_M) * delta_seconds)
    for path in ALL_WHEEL_PATHS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        _ensure_rotate_ops(prim, ["Z", "X"] if path in FRONT_WHEEL_PATHS else ["X"])
        spin_attr = prim.GetAttribute("xformOp:rotateX")
        spin_attr.Set(spin_attr.Get() + spin_delta_deg)


def _front_wheel_steer_deg(stage):
    prim = stage.GetPrimAtPath(FRONT_WHEEL_PATHS[0])
    if not prim.IsValid():
        return 0.0
    attr = prim.GetAttribute("xformOp:rotateZ")
    value = attr.Get() if attr else None
    return value if value is not None else 0.0


def _turn_forklift(is_left_pressed, is_right_pressed, delta_seconds):
    global _current_turn_rate_deg_per_sec
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(FORKLIFT_PATH)
    if not prim.IsValid():
        return

    steering = is_left_pressed != is_right_pressed
    if steering:
        steer_ratio = _front_wheel_steer_deg(stage) / MAX_STEER_ANGLE_DEG
        speed_ratio = _current_speed_mps / FORWARD_SPEED_MPS
        target = TURN_SPEED_DEG_PER_SEC * steer_ratio * speed_ratio
        _current_turn_rate_deg_per_sec = _step_toward(
            _current_turn_rate_deg_per_sec, target, TURN_ACCEL_DEG_PER_SEC2 * delta_seconds
        )
    else:
        _current_turn_rate_deg_per_sec = _step_toward(
            _current_turn_rate_deg_per_sec, 0.0, TURN_DECEL_DEG_PER_SEC2 * delta_seconds
        )

    if _current_turn_rate_deg_per_sec == 0.0:
        return

    rotate_attr = prim.GetAttribute("xformOp:rotateXYZ")
    rot = rotate_attr.Get()
    new_yaw = rot[2] + _current_turn_rate_deg_per_sec * delta_seconds

    pos = prim.GetAttribute("xformOp:translate").Get()
    if _refuse_pose(
        stage,
        (pos[0], pos[1], math.radians(rot[2]), None),
        (pos[0], pos[1], math.radians(new_yaw), None),
    ):
        _current_turn_rate_deg_per_sec = 0.0
        return

    rotate_attr.Set(Gf.Vec3f(rot[0], rot[1], new_yaw))


def _steer_front_wheels(is_pressed, is_opposite_pressed, steer_sign, delta_seconds):
    if not is_pressed and is_opposite_pressed:
        return
    stage = omni.usd.get_context().get_stage()
    reference_prim = stage.GetPrimAtPath(FRONT_WHEEL_PATHS[0])
    if not reference_prim.IsValid():
        return
    target_deg = steer_sign * MAX_STEER_ANGLE_DEG if is_pressed else 0.0
    step = STEER_SPEED_DEG_PER_SEC * delta_seconds
    _ensure_rotate_ops(reference_prim, ["Z", "X"])
    current = reference_prim.GetAttribute("xformOp:rotateZ").Get()
    new_angle = min(current + step, target_deg) if current < target_deg else max(current - step, target_deg)

    for path in FRONT_WHEEL_PATHS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        _ensure_rotate_ops(prim, ["Z", "X"])
        prim.GetAttribute("xformOp:rotateZ").Set(new_angle)

    wheel_prim = stage.GetPrimAtPath(STEERING_WHEEL_PATH)
    if wheel_prim.IsValid():
        _ensure_rotate_ops(wheel_prim, ["Z"])
        wheel_prim.GetAttribute("xformOp:rotateZ").Set(STEERING_WHEEL_SIGN * new_angle)

    handle_prim = stage.GetPrimAtPath(STEERING_WHEEL_HANDLE_PATH)
    if handle_prim.IsValid():
        _ensure_rotate_ops(handle_prim, ["Z"])
        handle_angle_rad = math.radians(STEERING_WHEEL_SIGN * new_angle)
        cos_a, sin_a = math.cos(handle_angle_rad), math.sin(handle_angle_rad)
        ox, oy, oz = STEERING_WHEEL_HANDLE_OFFSET_LOCAL
        rotated_local = Gf.Vec3d(ox * cos_a - oy * sin_a, ox * sin_a + oy * cos_a, oz)
        rotated_world = STEERING_WHEEL_TILT.TransformDir(rotated_local)
        handle_prim.GetAttribute("xformOp:rotateZ").Set(STEERING_WHEEL_SIGN * new_angle)
        handle_prim.GetAttribute("xformOp:translate").Set(
            Gf.Vec3f(
                STEERING_WHEEL_REST_TRANSLATE[0] + rotated_world[0],
                STEERING_WHEEL_REST_TRANSLATE[1] + rotated_world[1],
                STEERING_WHEEL_REST_TRANSLATE[2] + rotated_world[2],
            )
        )


def _lift_fork(speed_mps, is_pressed, delta_seconds):
    if not is_pressed:
        return
    stage = omni.usd.get_context().get_stage()
    reference_path = FORK_PATHS[0]
    reference_prim = stage.GetPrimAtPath(reference_path)
    if not reference_prim.IsValid():
        return
    current_offset = reference_prim.GetAttribute("xformOp:translate").Get()[2] - LIFT_PART_REST_Z[reference_path]
    capped_delta = min(delta_seconds, LIFT_MAX_DELTA_SECONDS)
    new_offset = max(LIFT_FLOOR_OFFSET_M, min(LIFT_TRAVEL_M, current_offset + speed_mps * capped_delta))
    if new_offset != current_offset:
        forklift = stage.GetPrimAtPath(FORKLIFT_PATH)
        if forklift.IsValid():
            fpos = forklift.GetAttribute("xformOp:translate").Get()
            fyaw = math.radians(forklift.GetAttribute("xformOp:rotateXYZ").Get()[2])
            if _refuse_pose(
                stage,
                (fpos[0], fpos[1], fyaw, current_offset),
                (fpos[0], fpos[1], fyaw, new_offset),
            ):
                return
    for path, rest_z in LIFT_PART_REST_Z.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        translate_attr = prim.GetAttribute("xformOp:translate")
        pos = translate_attr.Get()
        translate_attr.Set(Gf.Vec3f(pos[0], pos[1], rest_z + new_offset))


def _hide_camera_manipulator_mesh_if_present(stage):
    for path in CAMERA_CYCLE_PATHS:
        mesh_prim = stage.GetPrimAtPath(f"{path}/OmniverseKitViewportCameraMesh")
        if not mesh_prim.IsValid():
            continue
        imageable = UsdGeom.Imageable(mesh_prim)
        if imageable.GetPurposeAttr().Get() != UsdGeom.Tokens.guide:
            imageable.GetPurposeAttr().Set(UsdGeom.Tokens.guide)
        model_prim = stage.GetPrimAtPath(f"{path}/OmniverseKitViewportCameraMesh/CameraModel")
        if model_prim.IsValid():
            model_imageable = UsdGeom.Imageable(model_prim)
            if model_imageable.GetPurposeAttr().Get() != UsdGeom.Tokens.guide:
                model_imageable.GetPurposeAttr().Set(UsdGeom.Tokens.guide)


def reset_camera_to_home(stage, camera_path):
    home = CAMERA_HOME_TRANSFORMS.get(camera_path)
    prim = stage.GetPrimAtPath(camera_path) if stage is not None and home is not None else None
    if prim is None or not prim.IsValid():
        return
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(home)


def _cycle_camera_view(is_pressed):
    global _camera_cycle_index, _camera_cycle_key_was_down
    stage = omni.usd.get_context().get_stage()
    if stage is not None:
        _hide_camera_manipulator_mesh_if_present(stage)
    if is_pressed and not _camera_cycle_key_was_down:
        _camera_cycle_index = (_camera_cycle_index + 1) % len(CAMERA_CYCLE_PATHS)
        viewport = vp_util.get_active_viewport()
        if viewport is not None:
            reset_camera_to_home(stage, CAMERA_CYCLE_PATHS[_camera_cycle_index])
            viewport.camera_path = Sdf.Path(CAMERA_CYCLE_PATHS[_camera_cycle_index])
    _camera_cycle_key_was_down = is_pressed


def register_forklift_node_types():
    # 아래 노드 타입 이름은 일부러 Lv1 Lesson 3 접두사를 유지한다. 씬 파일의 Action Graph가 이 이름으로 참조한다
    # (이 레슨은 Lv1 Lesson 3 복사본).
    if _REGISTERED_NODE_TYPES:
        return

    @og.create_node_type(
        unique_name="Isaacsim_Edu_Lv1_Lesson3.ForkliftMoveForward",
        ui_name="Forklift Move Forward",
        add_execution_pins=True,
    )
    def forklift_move_forward(isPressed: ot.bool, isBackwardPressed: ot.bool, deltaSeconds: ot.double):
        """Drives the forklift: ramps smoothly up to speed while held, smoothly back
        down when released. Also owns backward/braking via the cross-wired
        isBackwardPressed input (mirrors Turn Left/Right's cross-wiring) -- see
        forklift_move_backward's docstring.

        isPressed/isBackwardPressed are no longer read -- both key states are
        polled fresh via _is_key_down instead -- and stay declared only so the
        saved Action Graph node doesn't need recreating."""
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            _drive_forklift(_is_key_down(stage, KEY_FORWARD_PATH), _is_key_down(stage, KEY_BACKWARD_PATH), deltaSeconds)
        return ()

    @og.create_node_type(
        unique_name="Isaacsim_Edu_Lv1_Lesson3.ForkliftMoveBackward",
        ui_name="Forklift Move Backward",
        add_execution_pins=True,
    )
    def forklift_move_backward(isPressed: ot.bool, deltaSeconds: ot.double):
        """Intentionally does nothing -- forklift_move_forward owns the combined
        accel/decel/braking update via its own isBackwardPressed cross-input
        (needs both key states together each frame). Stays registered, with
        Backward still wired to its isPressed, only so the saved Action Graph
        node doesn't need recreating."""
        return ()

    @og.create_node_type(
        unique_name="Isaacsim_Edu_Lv1_Lesson3.ForkliftTurnLeft",
        ui_name="Forklift Turn Left",
        add_execution_pins=True,
    )
    def forklift_turn_left(
        isPressed: ot.bool,
        isForwardPressed: ot.bool,
        isBackwardPressed: ot.bool,
        isRightPressed: ot.bool,
        deltaSeconds: ot.double,
    ):
        """Steers the forklift left. Front wheels pivot left while held and recenter
        when neither Left nor Right is held; heading follows from that angle and the
        vehicle's motion. Also owns the combined Left/Right turn-rate update via its
        own isRightPressed cross-input (mirrors Move Forward/Backward) -- wheels are
        pivoted first so the heading update reads this frame's angle.

        None of the graph-wired inputs are read anymore -- Left/Right are polled
        fresh via _is_key_down instead -- they stay declared only so the saved
        Action Graph node doesn't need recreating."""
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            left_down = _is_key_down(stage, KEY_LEFT_PATH)
            right_down = _is_key_down(stage, KEY_RIGHT_PATH)
            _steer_front_wheels(left_down, right_down, 1.0, deltaSeconds)
            _turn_forklift(left_down, right_down, deltaSeconds)
        return ()

    @og.create_node_type(
        unique_name="Isaacsim_Edu_Lv1_Lesson3.ForkliftTurnRight",
        ui_name="Forklift Turn Right",
        add_execution_pins=True,
    )
    def forklift_turn_right(
        isPressed: ot.bool,
        isForwardPressed: ot.bool,
        isBackwardPressed: ot.bool,
        isLeftPressed: ot.bool,
        deltaSeconds: ot.double,
    ):
        """Steers the forklift right. Front wheels pivot right while held and recenter
        when neither Left nor Right is held. Doesn't call _turn_forklift itself --
        forklift_turn_left owns the combined turn-rate update via its own
        isRightPressed cross-input, same pattern as Move Forward/Backward.

        Graph-wired inputs are no longer read -- Left/Right are polled fresh via
        _is_key_down instead -- they stay declared only so the saved Action Graph
        node doesn't need recreating."""
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            _steer_front_wheels(_is_key_down(stage, KEY_RIGHT_PATH), _is_key_down(stage, KEY_LEFT_PATH), -1.0, deltaSeconds)
        return ()

    @og.create_node_type(
        unique_name="Isaacsim_Edu_Lv1_Lesson3.ForkliftLiftUp",
        ui_name="Forklift Lift Up",
        add_execution_pins=True,
    )
    def forklift_lift_up(isPressed: ot.bool, deltaSeconds: ot.double):
        """Raises the forklift's fork while held, clamped to its travel range.
        isPressed is no longer read -- polled fresh via _is_key_down instead -- and
        stays declared only so the saved Action Graph node doesn't need recreating."""
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            _lift_fork(LIFT_SPEED_MPS, _is_key_down(stage, KEY_LIFT_UP_PATH), deltaSeconds)
        return ()

    @og.create_node_type(
        unique_name="Isaacsim_Edu_Lv1_Lesson3.ForkliftLiftDown",
        ui_name="Forklift Lift Down",
        add_execution_pins=True,
    )
    def forklift_lift_down(isPressed: ot.bool, deltaSeconds: ot.double):
        """Lowers the forklift's fork while held, clamped to its travel range.
        isPressed is no longer read -- polled fresh via _is_key_down instead -- and
        stays declared only so the saved Action Graph node doesn't need recreating."""
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            _lift_fork(-LIFT_SPEED_MPS, _is_key_down(stage, KEY_LIFT_DOWN_PATH), deltaSeconds)
        return ()

    @og.create_node_type(
        unique_name="Isaacsim_Edu_Lv1_Lesson3.ForkliftCycleCameraView",
        ui_name="Forklift Cycle Camera View",
        add_execution_pins=True,
    )
    def forklift_cycle_camera_view(isPressed: ot.bool):
        """Cycles the viewport camera front (driver view) -> left -> right -> front,
        once per physical key press. No deltaSeconds -- a discrete step, not
        something that ramps while held; _cycle_camera_view does its own edge
        detection since this node is still wired straight to Tick's execIn.

        isPressed is no longer read -- polled fresh via _is_key_down instead -- and
        stays declared only so the saved Action Graph node doesn't need recreating."""
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            _cycle_camera_view(_is_key_down(stage, KEY_CAMERA_CYCLE_PATH))
        return ()

    _REGISTERED_NODE_TYPES.extend(
        [
            "Isaacsim_Edu_Lv1_Lesson3.ForkliftMoveForward",
            "Isaacsim_Edu_Lv1_Lesson3.ForkliftMoveBackward",
            "Isaacsim_Edu_Lv1_Lesson3.ForkliftTurnLeft",
            "Isaacsim_Edu_Lv1_Lesson3.ForkliftTurnRight",
            "Isaacsim_Edu_Lv1_Lesson3.ForkliftLiftUp",
            "Isaacsim_Edu_Lv1_Lesson3.ForkliftLiftDown",
            "Isaacsim_Edu_Lv1_Lesson3.ForkliftCycleCameraView",
        ]
    )


def deregister_forklift_node_types():
    for name in _REGISTERED_NODE_TYPES:
        try:
            og.deregister_node_type(name)
        except Exception:
            pass
    _REGISTERED_NODE_TYPES.clear()
