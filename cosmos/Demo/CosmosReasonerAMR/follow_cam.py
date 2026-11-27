# 로봇을 따라가는 카메라 (시연용): 로봇 뒤 위쪽에서 로봇과 그 앞을 내려다본다.
#
#   import follow_cam
#   follow_cam.start()    # 뷰포트를 따라가는 카메라로 바꾸고 매 프레임 위치를 갱신
#   follow_cam.stop()     # 갱신을 멈춘다 (카메라는 마지막 자리에 남음)
#
# 로봇 위치는 lidar_drive.pose() (주행 중에는 물리 엔진 값, 멈춰 있으면 USD 값)를 쓴다.
# 카메라가 흔들리지 않게 위치·방향을 조금씩 따라가게 한다(부드럽게).

import builtins
import math

import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, UsdGeom

CAM_PATH = "/World/FollowCamera"
BACK = 3.0          # 로봇 뒤로 m
HEIGHT = 2.2        # 바닥에서 m
LOOK_AHEAD = 1.3    # 로봇 앞 몇 m 를 바라볼지 (크면 로봇이 화면 아래로 잘리고, 작으면 앞 통로가 안 보인다)
LOOK_Z = 0.3
SMOOTH = 0.08       # 0~1, 클수록 빨리 따라감


def _state():
    s = getattr(builtins, "_amr_follow_cam", None)
    if s is None:
        s = {"sub": None, "eye": None, "target": None}
        builtins._amr_follow_cam = s
    return s


def _ensure_camera(stage):
    cam = UsdGeom.Camera.Get(stage, CAM_PATH)
    if not cam:
        cam = UsdGeom.Camera.Define(stage, CAM_PATH)
        cam.CreateFocalLengthAttr(16.0)                 # 조금 넓게 (로봇과 앞 통로가 같이 보이게)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
    # 노출은 기본 카메라(Perspective)와 같게 맞춘다 (새 카메라 기본값이면 화면이 하얗게 날아간 적이 있다)
    persp = stage.GetPrimAtPath("/OmniverseKit_Persp")
    if persp:
        for name in ("exposure:fStop", "exposure:time", "exposure:responsivity", "exposure:iso", "exposure"):
            src = persp.GetAttribute(name)
            if src and src.HasAuthoredValue():
                cam.GetPrim().CreateAttribute(name, src.GetTypeName()).Set(src.Get())
    # 마우스로 움직이지 못하게 잠근다 (따라가는 중에 마우스로 돌리면 코드와 마우스가 서로 카메라를 당겨 화면이 튐).
    # 뷰포트 카메라 메뉴의 'Lock' 과 같은 속성이다
    cam.GetPrim().CreateAttribute("omni:kit:cameraLock", Sdf.ValueTypeNames.Bool, True).Set(True)
    xf = UsdGeom.Xformable(cam)
    ops = xf.GetOrderedXformOps()
    if len(ops) != 1 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTransform:
        xf.ClearXformOpOrder()
        xf.AddTransformOp()
    return cam


def _desired():
    import lidar_drive
    x, y, yaw = lidar_drive.pose()
    c, s = math.cos(yaw), math.sin(yaw)
    eye = Gf.Vec3d(x - BACK * c, y - BACK * s, HEIGHT)
    target = Gf.Vec3d(x + LOOK_AHEAD * c, y + LOOK_AHEAD * s, LOOK_Z)
    return eye, target


def update(snap=False):
    """카메라를 한 번 갱신한다. snap=True 면 부드럽게 따라가지 않고 바로 그 자리로."""
    stage = omni.usd.get_context().get_stage()
    if not stage:
        return
    try:
        eye, target = _desired()
    except Exception:
        # 로봇이 없는 장면 (다른 데모로 장면을 바꿈) → 매 프레임 헛도는 구독을 스스로 끊는다
        stop()
        return
    s = _state()
    if snap or s["eye"] is None:
        s["eye"], s["target"] = eye, target
    else:
        s["eye"] = s["eye"] + (eye - s["eye"]) * SMOOTH
        s["target"] = s["target"] + (target - s["target"]) * SMOOTH
    cam = _ensure_camera(stage)
    view = Gf.Matrix4d().SetLookAt(s["eye"], s["target"], Gf.Vec3d(0, 0, 1))   # 월드 → 카메라
    UsdGeom.Xformable(cam).GetOrderedXformOps()[0].Set(view.GetInverse())     # 카메라 → 월드


def start():
    """뷰포트를 따라가는 카메라로 바꾸고, 매 프레임 갱신을 시작한다."""
    stop()
    update(snap=True)
    s = _state()
    s["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        lambda _e: update(), name="amr_follow_cam")
    try:
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = CAM_PATH
    except Exception:
        pass


def stop():
    s = _state()
    if s["sub"] is not None:
        try:
            s["sub"].unsubscribe()
        except Exception:
            pass
        s["sub"] = None
    s["eye"] = s["target"] = None

