# RTX Lidar 로 앞 2D 스캔 읽기 (lidar_drive 의 센서 계층)
#
#   import rtx_lidar
#   rtx_lidar.ensure()                    # 타임라인이 멈춘 상태에서 한 번 (렌더 프로덕트·어노테이터 생성)
#   pts = rtx_lidar.scan_points()         # 로봇 좌표 (x 앞, y 왼쪽) 2D 점. 아직 값이 없으면 빈 배열
#
# 센서: Nova Carter 에 이미 있는 앞 RPLidar_S2E (OmniLidar). 문서(Isaac Sim 6.2)의 권장 방식:
#   LidarSensor 런타임 클래스 + 'generic-model-output' 어노테이터 + parse_generic_model_output_data()
# 지켜야 할 것 (예전에 RTX 를 뺀 이유):
#   - 렌더 프로덕트·어노테이터는 Play 전에 한 번만 만들고 계속 쓴다 (Play 중에 만들거나 껐다 켜면 값이 멈추거나 빈다)
#   - 렌더 프로덕트는 절대 RemovePrim 하지 않는다 (rtx.hydra 가 멈춘다)
#   - 값이 갱신되지 않으면 lidar_drive 의 센서 이상 안전장치(lidar_age)가 로봇을 세운다
#
# 출력 좌표: 이 라이다 모델은 구면 좌표(elementsCoordsType = SPHERICAL)로 내보낸다.
#   x = 방위각(도), y = 고도(도), z = 거리(m). 센서 좌표 → 로봇 좌표는 센서가 달린 자리(고정)를 한 번 읽어 둔다.

import builtins
import math

import numpy as np
import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom

import amr_scene

CHASSIS = amr_scene.ROBOT_PATH + "/chassis_link"
SENSOR_PATH = CHASSIS + "/sensors/front_RPLidar/RPLidar_S2E"
_KEY = "_amr_rtx_lidar"


def _st():
    s = getattr(builtins, _KEY, None)
    if s is None:
        s = {"sensor": None, "yaw": 0.0, "tx": 0.0, "ty": 0.0, "last_frame": None, "pts": np.zeros((0, 2)), "sig": None}
        setattr(builtins, _KEY, s)
    return s


def _read_mount_transform():
    """센서 → 로봇(chassis_link) 좌표 변환 (센서는 몸체에 고정이라 한 번만 읽는다)."""
    s = _st()
    stage = omni.usd.get_context().get_stage()
    tc = Usd.TimeCode.Default()
    src = UsdGeom.Xformable(stage.GetPrimAtPath(SENSOR_PATH)).ComputeLocalToWorldTransform(tc)
    chm = UsdGeom.Xformable(stage.GetPrimAtPath(CHASSIS)).ComputeLocalToWorldTransform(tc)
    rel = src * chm.GetInverse()
    fwd = rel.TransformDir((1, 0, 0))
    s["yaw"] = math.atan2(fwd[1], fwd[0])
    t = rel.ExtractTranslation()
    s["tx"], s["ty"] = t[0], t[1]
    s["last_frame"] = None


def _stage_key():
    """지금 열린 스테이지를 구별하는 값 (장면을 다시 열면 바뀐다)."""
    ctx = omni.usd.get_context()
    try:
        return ctx.get_stage_id()
    except Exception:
        return id(ctx.get_stage())


def ensure():
    """센서(렌더 프로덕트 + 어노테이터)를 한 번만 만든다. 이미 있으면 그대로. 타임라인이 멈춘 상태에서 부를 것.
    장면을 다시 열면(확장을 껐다 켤 때 저장한 장면을 다시 연다) 예전 스테이지의 센서 객체는 값이 영영 오지 않는다
    ('라이다 값이 들어오지 않아 정지' 로 AMR 이 조금 가다 섰다) → 스테이지가 바뀌었으면 새로 만든다."""
    s = _st()
    stage = omni.usd.get_context().get_stage()
    key = _stage_key()
    if s["sensor"] is not None and stage.GetPrimAtPath(SENSOR_PATH):
        if s.get("stage_key") in (None, key):
            s["stage_key"] = key                 # (예전 상태에는 키가 없다 → 지금 스테이지의 것으로 보고 그대로 쓴다)
            return s["sensor"]
    from isaacsim.sensors.experimental.rtx import LidarSensor
    s["sensor"] = LidarSensor(SENSOR_PATH, annotators=["generic-model-output"])
    s["stage_key"] = key
    _read_mount_transform()
    return s["sensor"]


def _read():
    """어노테이터 값을 읽어 (프레임 번호, 방위각[라디안], 거리[m]) 로. 값이 없으면 None."""
    s = _st()
    sensor = s["sensor"]
    if sensor is None or not sensor.has_data():
        return None
    from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data
    data, _info = sensor.get_data("generic-model-output")
    out = parse_generic_model_output_data(data)
    n = int(out.numElements)
    if n <= 0:
        return None
    az = np.asarray(out.x, dtype=float)[:n]
    rng = np.asarray(out.z, dtype=float)[:n]
    return int(out.frameId), np.radians(az), rng


def scan_points(min_range=0.5, max_range=14.9):
    """앞 라이다 점들을 로봇 좌표 (x 앞, y 왼쪽) 로. 값이 새로 오지 않았으면 직전 값을 그대로 돌려준다."""
    s = _st()
    r = _read()
    if r is None:
        return s["pts"] if s["pts"] is not None else np.zeros((0, 2))
    frame, az, rng = r
    ok = (rng > min_range) & (rng < max_range) & np.isfinite(rng)
    # 센서 좌표 (x 앞, y 왼쪽) → 로봇 좌표
    px = rng[ok] * np.cos(az[ok])
    py = rng[ok] * np.sin(az[ok])
    c, sn = math.cos(s["yaw"]), math.sin(s["yaw"])
    pts = np.stack([s["tx"] + px * c - py * sn, s["ty"] + px * sn + py * c], axis=1)
    s["pts"] = pts
    s["last_frame"] = frame
    return pts


def frame_id():
    return _st()["last_frame"]


# ---------------------------------------------------------------- 보기용 점 표시 (무지개색 점)
# 사람이 보는 뷰포트에서만 그린다 (isaacsim.util.debug_draw). Cosmos 로 보내는 카메라 사진에는 안 들어가야 하므로
# 사진을 찍는 동안에는 capture.grab_async 가 pause_debug_draw(True) 로 잠깐 지운다.
_dbg = {"on": False, "sub": None, "paused": False, "n": 0}


def set_debug_draw(on):
    """라이다 점을 뷰포트에 색깔(가까울수록 빨강 → 멀수록 파랑)로 보여 준다. 끄면 지운다."""
    on = bool(on)
    if on == _dbg["on"]:
        return
    import omni.kit.app
    if on:
        _dbg["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_on_dbg_update, name="amr_rtx_lidar_debug")
    else:
        if _dbg["sub"] is not None:
            _dbg["sub"].unsubscribe()
            _dbg["sub"] = None
        _clear_dbg()
    _dbg["on"] = on


def pause_debug_draw(paused):
    """Cosmos 사진을 찍는 동안 점 표시를 숨긴다 (True = 숨김)."""
    _dbg["paused"] = bool(paused)
    if paused:
        _clear_dbg()


def _clear_dbg():
    try:
        from isaacsim.util.debug_draw import _debug_draw
        _debug_draw.acquire_debug_draw_interface().clear_points()
    except Exception:
        pass


def _on_dbg_update(_e):
    if _dbg["paused"] or not omni.timeline.get_timeline_interface().is_playing():
        return
    _dbg["n"] += 1
    if _dbg["n"] % 4:                                   # 4프레임에 한 번만 그린다 (가볍게)
        return
    try:
        import colorsys
        import lidar_drive
        from isaacsim.util.debug_draw import _debug_draw
        s = _st()
        p = s["pts"]
        if p is None or len(p) == 0:
            return
        x, y, yaw = lidar_drive.pose()
        c, sn = math.cos(yaw), math.sin(yaw)
        wx = x + p[:, 0] * c - p[:, 1] * sn
        wy = y + p[:, 0] * sn + p[:, 1] * c
        d = np.hypot(p[:, 0], p[:, 1])
        pts, cols = [], []
        for i in range(0, len(p), 2):
            h = min(d[i] / 10.0, 1.0) * 0.7             # 0 = 빨강(가까움) → 0.7 = 파랑(멀다)
            r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
            pts.append((float(wx[i]), float(wy[i]), 0.45))
            cols.append((r, g, b, 1.0))
        dd = _debug_draw.acquire_debug_draw_interface()
        dd.clear_points()
        dd.draw_points(pts, cols, [4.0] * len(pts))
    except Exception:
        pass
