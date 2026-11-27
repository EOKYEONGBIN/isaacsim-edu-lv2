# 라이다 주행: Nova Carter 를 물리(Play)로 실제 바퀴를 굴려 움직이고, 앞 2D 라이다로 장애물에 대응한다.
# (ROS / Nav2 없이 Isaac Sim 안에서. 실제 AMR 의 '전역 경로 + 지역 회피' 구조를 단순화한 교육용)
#
#   import lidar_drive, asyncio
#   nav = lidar_drive.LidarNavigator()          # 타임라인을 멈추고 로봇을 START 에 둔 뒤 Play
#   ok = await nav.drive_to("GOAL", on_checkpoint=..., on_event=...)
#
# 구조
#   - 전역 경로: amr_nav.Navigator 의 통로 그래프 + A* 를 그대로 씀 (어느 통로로 갈지)
#   - 지역 주행: 다음 지점으로 향하는 직진·회전 속도 → 좌우 바퀴 속도 (차동 구동)
#   - 라이다: 앞 RPLidar(RTX Lidar)의 360° 스캔을 로봇 좌표로 바꿔,
#       로봇이 지나갈 띠(폭 약 0.94 m) 안의 점을 본다
#       · 가까우면 감속, 0.9 m 안이면 정지
#       · 막혔으면 옆으로 비켜 갈 틈(좌우 0.35·0.65 m)을 찾아 비켜 감
#       · 3초 넘게 못 가면 '라이다 막힘' → 그 통로를 빼고 되돌아가 다시 계획
#   - Cosmos(선택): 통로 입구에서 정지 → 사진 판단 → 통과 / 대기(사람) / 우회
#   - 로봇 위치: 시뮬레이션이 주는 정확한 값 (실제 AMR 은 라이다+지도로 추정 — 여기서는 단순화)

import asyncio
import builtins
import math
import time

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom

import amr_nav
import amr_scene

CHASSIS = amr_scene.ROBOT_PATH + "/chassis_link"
# 라이다 센서: Nova Carter 의 앞 RPLidar_S2E (RTX Lidar). rtx_lidar.py 가 읽는다. (PhysX 라이다는 뺐다 —
#   6.0 부터 Deprecated 이고, 쓰지 않아도 장면에 남아 매 물리 스텝마다 광선 720개를 쏘며 프레임을 깎았다)
#   렌더링되는 물체를 본다 → 작업자 모델의 실제 몸·박스·표지판은 보이고, 충돌체만 있고 안 그려진 것은 안 보인다.
#   3 mm 바닥 웅덩이는 안 보인다. ROS2/Nav2 로 이어 붙이기 쉬운 실제 센서 모델.
#   (렌더 프로덕트를 Play 중에 만들거나 껐다 켜면 값이 멈추거나 비었다
#    → 렌더 프로덕트·어노테이터를 Play 전에 한 번만 만들어 계속 쓰고, 값이 안 오면 로봇을 세운다)
#   점은 앞 RPLidar 와 같은 자리(높이 약 0.42 m)에서 로봇 좌표 2D 점(x 앞, y 왼쪽)으로 바꿔 쓴다
WHEELS = (amr_scene.ROBOT_PATH + "/joint_wheel_left", amr_scene.ROBOT_PATH + "/joint_wheel_right")

V_MAX = 1.2            # m/s 최고 속도 (시연용으로 약간 빠르게)
W_MAX = 0.6            # rad/s 최고 회전 속도 (1.0 에서는 회전이 넘쳐 좌우로 흔들렸다)
HALF_W = 0.47          # 로봇이 지나갈 띠의 반폭: 로봇 폭 0.73 m(반폭 0.365) + 양쪽 여유 0.1 m. 지나갈 틈에는 이 띠(0.94 m)가 들어가야 한다
                       # (예전 0.5 m 와 비켜 가기 최대 0.75 m 로는 통로 폭 3.1 m 에 박스 하나(0.7 m)만 있어도 1.2 m 틈을 못 지나갔다)
STOP_DIST = 0.95       # 이 거리 안에 띠 안 물체가 있으면 정지 (라이다 기준)
SLOW_DIST = 2.2        # 이 거리부터 감속
BACKUP_TO = 2.0        # 정지한 채 비킬 틈을 찾았으면 앞 물체까지 이 거리가 될 때까지 뒤로 물러난 뒤 비켜 간다
DODGE_DIST = 4.0      # 가운데가 이 거리 안에서 막히면 비킬 길을 찾는다 (옆으로 1 m 옮겨 가려면 미리 시작해야 몸이 닿지 않는다)
OFFSETS = (0.0, 0.4, -0.4, 0.75, -0.75, 1.0, -1.0)     # 비켜 가기 후보 (통로 반폭 약 1.55 m - 띠 반폭 0.47 m ≈ 1.08 m 까지)
BLOCK_SEC = 3.0        # 이 시간(시뮬레이션 초) 넘게 못 가면 '라이다 막힘'
REACH = 0.3            # 지점 도착 판정 거리
# 복도(통로가 아닌 구간)가 막혔을 때 물러났다 다시 시도하는 횟수. 복도에는 고정 장애물이 없고 걸어 다니는 작업자만 있어서
# 잠깐 기다리면 비켜 주므로 넉넉히 둔다 (2번이면 작업자가 서 있는 동안 주행이 '멈춤' 으로 끝난 적이 있다)
SEGMENT_RETRIES = 6
# Cosmos 가 '사람 때문에' 막힘이라고 한 통로만 있어서 갈 길이 없을 때: 이만큼(시뮬레이션 초) 기다렸다가 다시 판단 (최대 횟수까지)
PERSON_WAIT_SEC = 8.0
PERSON_RECHECK_SEC = 5.0     # 통로 앞에서 사람 때문에 막혔을 때, 이 시간(시뮬레이션 초)마다 다시 판단한다
PERSON_RECHECKS = 4         # 최대 이만큼 기다려도 안 비키면 옆 통로로 돌아간다 (약 20 초)
PERSON_WAIT_MAX = 3
LOOKAHEAD = 1.5        # 퓨어 퍼슛 앞보기 거리 (m)
W_DRIVE = 0.4          # 달리면서 꺾을 때 회전 속도 한계 (rad/s) (0.25 는 차선에서 벗어난 것을 못 되돌려 콘에 걸렸다)
# 달리면서 꺾으면 실제 회전이 명령의 약 2.7배로 나왔다 (뒤 캐스터 영향으로 보임, 제자리 회전은 명령과 같음)
# → 달리는 중에는 회전 명령을 작게 하고, 실제 회전 속도로 제동(감쇠)을 건다
MIN_RANGE = 0.5        # 이보다 가까운 점은 로봇 자신(몸체)으로 보고 버림
ACCEL = 0.5            # m/s² 가속 한계. 서 있다가 갑자기 출발하면 옆을 보던 뒤 캐스터가 끌려 출발 순간
                       # 왼쪽으로 40° 넘게 틀어졌다 → 실제 AMR 처럼 천천히 가속한다


# ---------------------------------------------------------------- 준비
def prepare(start=None, yaw=90.0):
    """타임라인을 멈추고 로봇을 시작 자세로 둔다. ROS 그래프는 끈다 (바퀴 제어와 부딪히고, 카메라 렌더 부하가 큼)."""
    tl = omni.timeline.get_timeline_interface()
    if not tl.is_stopped():
        tl.stop()
    stage = omni.usd.get_context().get_stage()
    amr_scene.tune_physics(stage)                 # PhysX GPU 버퍼 크기 (기본값이면 충돌 검사 버퍼 부족 오류)
    amr_scene.reset_robot_links(stage)            # 지난 주행에서 남은 부품 위치·바퀴 속도 지우기 (처음에 빙글 도는 문제)
    amr_scene.apply_pending_speed()               # 주행 중에 바꾼 배속은 여기서(멈춘 상태) 적용
    for p in Usd.PrimRange(stage.GetPrimAtPath(amr_scene.ROBOT_PATH)):
        if p.GetTypeName() == "OmniGraph" and p.IsActive():
            p.SetActive(False)
    x, y = start or amr_scene.START
    amr_scene.set_robot_pose(x, y, yaw)
    set_wheels(0.0, 0.0)


def ensure_lidar():
    """라이다를 준비한다 (이미 있으면 그대로). 타임라인이 멈춘 상태에서 부를 것.
    렌더 프로덕트·어노테이터를 한 번만 만든다 (Play 중에 만들면 값이 비어 나온다)."""
    import rtx_lidar
    rtx_lidar.ensure()
    return rtx_lidar.SENSOR_PATH


def _wheel_geometry():
    """바퀴 반지름과 좌우 바퀴 간격 (에셋에서 읽음, 실패하면 Nova Carter 기본값)."""
    c = getattr(builtins, "_amr_wheel_geo", None)
    if c:
        return c
    radius, base = 0.14, 0.413
    try:
        stage = omni.usd.get_context().get_stage()
        ys = []
        for j in WHEELS:
            p = stage.GetPrimAtPath(j).GetAttribute("physics:localPos0").Get()
            ys.append(p)
        base = abs(ys[0][1] - ys[1][1]) or base
        # 반지름은 메시 크기(타이어 바깥 장식 포함 약 0.19)가 아니라 실제 구름 반지름 0.14 를 쓴다
        # (0.19 로 계산하면 명령한 속도의 약 70% 만 나왔다 — 시험으로 확인)
    except Exception:
        pass
    builtins._amr_wheel_geo = (radius, base)
    return radius, base


def set_wheels(left_rad_s, right_rad_s):
    """바퀴 관절의 목표 속도 (rad/s → USD 는 deg/s)."""
    stage = omni.usd.get_context().get_stage()
    sign = getattr(builtins, "_amr_wheel_sign", 1.0)
    for j, w in zip(WHEELS, (left_rad_s, right_rad_s)):
        a = stage.GetPrimAtPath(j).GetAttribute("drive:angular:physics:targetVelocity")
        if a:
            a.Set(float(math.degrees(w) * sign))


def drive(v, w):
    """직진 속도 v (m/s), 회전 속도 w (rad/s, 왼쪽 +) → 좌우 바퀴."""
    r, b = _wheel_geometry()
    set_wheels((v - w * b / 2) / r, (v + w * b / 2) / r)


def sim_time():
    """시뮬레이션 시간 (초). 화면이 느리면 실제 시간보다 천천히 흐르므로 대기·제한 시간은 이것으로 잰다."""
    return omni.timeline.get_timeline_interface().get_current_time()


def _usd_pose():
    m = UsdGeom.Xformable(omni.usd.get_context().get_stage().GetPrimAtPath(CHASSIS)) \
        .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p = m.ExtractTranslation()
    fwd = m.TransformDir((1, 0, 0))
    return p[0], p[1], math.atan2(fwd[1], fwd[0])


def pose():
    """(x, y, yaw[rad]) — 시뮬레이션의 실제 위치."""
    # 멈춰 있을 때 PhysX 에 물으면 'getRigidBodyTransformation ... could not locate any objects' 오류가
    # 매 프레임 찍힌다 (따라가는 카메라가 매 프레임 부름) → Play 중일 때만 PhysX 에 묻는다
    # AMR 이 없는 장면(예: 다른 데모로 장면을 바꾼 뒤 따라가는 카메라 구독이 남은 경우)에서
    # PhysX 에 물으면 같은 오류가 매 프레임 찍힌다 (Franka 장면에서 실제로 발생) → 먼저 확인한다
    stage = omni.usd.get_context().get_stage()
    if not stage or not stage.GetPrimAtPath(CHASSIS):
        raise RuntimeError(f"AMR 이 장면에 없습니다 ({CHASSIS})")
    if not omni.timeline.get_timeline_interface().is_playing():
        return _usd_pose()
    t = omni.physx.get_physx_interface().get_rigidbody_transformation(CHASSIS)
    if not t.get("ret_val"):
        return _usd_pose()                       # 물리가 아직 준비되지 않았을 때 (Play 직후 몇 프레임)
    p, q = t["position"], t["rotation"]          # q = (x, y, z, w)
    yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))
    return p[0], p[1], yaw


_scan_state = {"sig": None, "t": 0.0}


def lidar_age():
    """마지막으로 라이다 값이 바뀐 뒤 지난 시뮬레이션 시간. 움직이는 중인데 값이 안 바뀌면 센서 이상."""
    return sim_time() - _scan_state["t"]


def reset_lidar():
    _scan_state.update(sig=None, t=sim_time())
    import rtx_lidar
    rtx_lidar._st()["last_frame"] = None


def _drop_own_body(p):
    # 로봇 자신의 몸체(앞 0.2 m ~ 뒤 0.7 m, 좌우 0.5 m 안)에 맞은 점은 버린다
    # (띠를 넓힌 뒤 몸체 모서리를 '앞 0.1 m 물체' 로 본 적이 있다)
    body = (p[:, 0] > -0.7) & (p[:, 0] < 0.2) & (np.abs(p[:, 1]) < 0.5)
    return p[~body]


def scan_points():
    """앞 라이다 점들을 로봇 좌표 (x 앞, y 왼쪽) 로. 아직 데이터가 없으면 빈 배열."""
    import rtx_lidar
    try:
        p = rtx_lidar.scan_points(MIN_RANGE, 14.9)
        frame = rtx_lidar.frame_id()
    except Exception:
        return np.zeros((0, 2))
    if len(p) == 0:
        return np.zeros((0, 2))
    if frame is not None and frame != _scan_state["sig"]:       # 새 스캔이 왔다 (프레임 번호가 바뀜)
        _scan_state.update(sig=frame, t=sim_time())
    return _drop_own_body(p)


def to_path_frame(pts, x, y, yaw, sx, sy, ux, uy):
    """로봇 좌표의 라이다 점 → '경로 좌표' (앞 = 구간 방향으로 로봇보다 얼마나 앞, 옆 = 차선 중심에서 왼쪽 거리).
    띠를 로봇이 바라보는 방향이 아니라 차선 방향으로 잡기 위해 쓴다
    (로봇이 비스듬히 통로에 들어설 때 띠가 선반 끝에 걸려 빈 통로를 '막힘' 으로 본 적이 있다)."""
    if len(pts) == 0:
        return pts
    c, s_ = math.cos(yaw), math.sin(yaw)
    wx = x + pts[:, 0] * c - pts[:, 1] * s_
    wy = y + pts[:, 0] * s_ + pts[:, 1] * c
    along = (wx - sx) * ux + (wy - sy) * uy - ((x - sx) * ux + (y - sy) * uy)
    lateral = -(wx - sx) * uy + (wy - sy) * ux
    return np.stack([along, lateral], axis=1)


def nearest_in_band(pts, offset=0.0, half=HALF_W, look=6.0):
    """띠 (|y - offset| < half) 안, 앞쪽 look m 안에서 가장 가까운 점까지 거리. 없으면 inf."""
    if len(pts) == 0:
        return math.inf
    # 앞 0.35 m 안(범퍼보다 가까운 곳)은 보지 않는다: 모퉁이에서 구간이 바뀌는 순간 옆 물체가 '앞 0.1 m' 로 잡혀
    # 쓸데없이 비켜 가는 일이 있었다. 실제로 그 거리의 물체는 이미 STOP_DIST 판단에서 걸러진다.
    m = (pts[:, 0] > 0.35) & (pts[:, 0] < look) & (np.abs(pts[:, 1] - offset) < half)
    return float(pts[m, 0].min()) if m.any() else math.inf


# ---------------------------------------------------------------- 주행
class LidarNavigator(amr_nav.Navigator):
    """amr_nav.Navigator 의 경로 계산은 그대로 쓰고, 이동만 물리 + 라이다로 바꾼 것."""

    def __init__(self, use_lidar=True):
        # 부모 __init__ 은 로봇을 START 로 순간 이동시키므로 그 전에 prepare() 로 같은 자리에 둔다
        super().__init__()
        self.use_lidar = use_lidar
        self.min_ahead = math.inf          # UI 표시용: 띠 안 가장 가까운 물체 거리
        self.ahead = math.inf              # 멈춤·감속 판단에 실제로 쓴 거리 (안전 영역 표시용)
        self.offset = 0.0
        self._moving = False
        self._near_count = 0
        self._clear_count = 0              # 비켜 간 뒤 가운데 길이 연속으로 비어 있던 프레임 수 (차선 가운데로 돌아올 때 센다)
        self._prev = None                  # (yaw, sim_time) — 실제 회전 속도 계산용
        self._v_cmd = 0.0                  # 직전 전진 속도 명령 (가속 한계용)
        self.on_tick = None                # 주행 중 매 프레임 on_tick(x, y) 호출 (돌발 상황 발생 등에 사용)
        self.person_blocked = set()        # Cosmos 가 '사람 때문에' 막힘이라고 한 통로 (사람은 비키므로 나중에 다시 판단)
        self.on_lidar_block = None         # 통로 안에서 라이다에 막혔을 때 부르는 함수 f(aisle) — 말풍선에 '라이다 막힘' 을 바로 보여 준다

    async def _tick(self):
        await omni.kit.app.get_app().next_update_async()
        if not omni.timeline.get_timeline_interface().is_playing() and not self.stop_requested:
            await self._ensure_playing()

    async def _ensure_playing(self):
        """Play 상태 보장. 정지 직후에 보낸 play 는 무시될 때가 있어 확인하며 다시 보낸다."""
        tl = omni.timeline.get_timeline_interface()
        for _ in range(5):
            if tl.is_playing():
                return
            tl.play()
            for _ in range(20):
                await omni.kit.app.get_app().next_update_async()
        raise RuntimeError("타임라인을 Play 할 수 없습니다")

    async def _stop_robot(self, frames=10):
        drive(0.0, 0.0)
        self._v_cmd = 0.0
        for _ in range(frames):
            await self._tick()

    async def _turn_to(self, target_yaw_deg):
        target = math.radians(target_yaw_deg)
        t0 = sim_time()
        settled = 0
        while not self.stop_requested and sim_time() - t0 < 15:
            x, y, yaw = pose()
            err = (target - yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(err) < math.radians(4):
                settled += 1
                drive(0.0, 0.0)
                if settled > 5:                    # 몇 프레임 연속 맞으면 끝 (넘침 방지)
                    break
            else:
                settled = 0
                w = 1.2 * err
                w = math.copysign(max(0.12, min(W_MAX, abs(w))), w)     # 너무 느리면 마찰로 안 돌아서 최소값
                drive(0.0, w)
            await self._tick()
        await self._stop_robot(3)
        self.x, self.y, yaw = pose()
        self.yaw = math.degrees(yaw)

    async def _backup(self, ev, dist=1.0):
        """천천히 후진 (실제 AMR 처럼 막힌 곳에서 여유를 만든 뒤 돈다). 뒤쪽 0.7 m 안에 물체가 있으면 멈춘다
        (앞 라이다가 360° 라서 뒤도 본다)."""
        x0, y0, _ = pose()
        t0 = sim_time()
        while not self.stop_requested and sim_time() - t0 < 6.0:
            x, y, _ = pose()
            if math.hypot(x - x0, y - y0) >= dist:
                break
            p = scan_points()
            if len(p) and ((p[:, 0] < -0.75) & (p[:, 0] > -1.45) & (np.abs(p[:, 1]) < HALF_W)).any():
                ev("lidar", "라이다: 뒤에 물체 → 후진 멈춤")
                break
            drive(-0.3, 0.0)
            await self._tick()
        await self._stop_robot(5)

    async def _follow_path(self, P, ev):
        """여러 지점을 멈추지 않고 이어서 주행 (모퉁이에서는 감속하며 돌아 나간다).
        P = [(x, y), ...] (P[0] 은 출발점). 결과: (상태, 구간 번호) — 상태 'ok' / 'stopped' / 'lidar_blocked' / 'sensor_fail'.
        조향은 퓨어 퍼슛: 경로선 위 앞보기 거리만큼 앞 점을 바라보며 곡률로 회전 속도를 정한다."""
        self.offset = 0.0
        self._clear_count = 0
        blocked_since = None
        backup_t0 = None                   # 정지 후 뒤로 물러나기 시작한 시각 (None=아직, False=이미 했음)
        _scan_state["t"] = sim_time()              # 서 있던 동안은 값이 안 바뀌는 게 정상이므로 기준 시각을 새로
        self._prev = None
        n = len(P) - 1
        L = [max(1e-6, math.hypot(P[k + 1][0] - P[k][0], P[k + 1][1] - P[k][1])) for k in range(n)]
        U = [((P[k + 1][0] - P[k][0]) / L[k], (P[k + 1][1] - P[k][1]) / L[k]) for k in range(n)]
        i = 0
        prog = (sim_time(), sum(L))
        while not self.stop_requested:
            x, y, yaw = pose()
            if self.on_tick:
                self.on_tick(x, y)
            sx, sy = P[i]
            ux, uy = U[i]
            proj = (x - sx) * ux + (y - sy) * uy
            # 다음 구간으로 넘어가기: 이 구간 끝을 지났거나, 모퉁이를 미리 돌아 다음 점에 가까워졌을 때
            if i < n - 1 and (proj >= L[i] - 0.05 or math.hypot(P[i + 1][0] - x, P[i + 1][1] - y) < 0.5):
                i += 1
                self.offset = 0.0
                continue
            proj = max(0.0, min(L[i], proj))
            rem = (L[i] - proj) + sum(L[i + 1:])               # 남은 경로 길이
            # 옆으로 비켜 가는 중(self.offset)에는 지점 한가운데가 아니라 옆 줄을 따라 오므로 그만큼 여유를 준다.
            # (여유가 없으면 오프셋 0.4 m 로 모퉁이에 도착했을 때 0.39 m > 0.3 m 라서 영원히 '도착' 못 하고
            #  제자리에서 빙글빙글 돌았다 — 걸어 다니는 작업자를 피하다 생긴 오프셋으로 확인)
            if i == n - 1 and math.hypot(P[-1][0] - x, P[-1][1] - y) < REACH + abs(self.offset):
                return "ok", i
            # 앞으로 가라고 했는데 4초 넘게 남은 길이가 줄지 않으면 '걸림' (라이다에 안 잡히는 곳에 몸이 닿은 경우)
            if rem < prog[1] - 0.1:
                prog = (sim_time(), rem)
            elif getattr(self, "_fwd", False) and sim_time() - prog[0] > 4.0:
                self._fwd = False
                await self._stop_robot()
                ev("lidar", "앞으로 가지 못함 (몸이 물체에 걸림)")
                return "lidar_blocked", i
            v_lim = V_MAX
            look_d = LOOKAHEAD
            # 모퉁이 감속: 다음 구간으로 크게 꺾이고 모퉁이가 가까우면 천천히, 앞보기도 짧게 (선반 끝을 스치지 않게)
            if i < n - 1:
                turn = abs((math.atan2(U[i + 1][1], U[i + 1][0]) - math.atan2(uy, ux) + math.pi) % (2 * math.pi) - math.pi)
                if turn > math.radians(45) and L[i] - proj < 1.5:
                    v_lim, look_d = 0.4, 0.8
            if self.use_lidar:
                pts = scan_points()
                # 안전장치: 라이다 값이 없거나, 움직이는 중인데 0.5초 넘게 그대로면 센서 이상 → 멈추고 기다림.
                if len(pts) == 0 or (self._moving and lidar_age() > 0.5):
                    await self._stop_robot(3)
                    self._moving = False
                    t0 = sim_time()
                    while sim_time() - t0 < 2.0 and not self.stop_requested:
                        await self._tick()
                        if len(scan_points()) and lidar_age() < 0.2:
                            break
                    else:
                        ev("stuck", "라이다 값이 들어오지 않아 정지했습니다 (타임라인을 Stop 하고 다시 시작하세요)")
                        return "sensor_fail", i
                    continue
                body_ahead = nearest_in_band(pts, 0.0)                  # 로봇 몸 바로 앞 (로봇이 바라보는 방향 기준)
                pts = to_path_frame(pts, x, y, yaw, sx, sy, ux, uy)
                seg_look = min(6.0, L[i] - proj + 0.6)                  # 이 구간 안에서만 (모퉁이 너머 벽은 제외)
                center = nearest_in_band(pts, 0.0, look=seg_look)
                self.min_ahead = center
                ahead = min(nearest_in_band(pts, self.offset, look=seg_look), body_ahead)
                self._near_count = self._near_count + 1 if center < DODGE_DIST else 0
                if center < DODGE_DIST and self.offset == 0.0 and self._near_count >= 3:   # 3프레임 연속일 때만
                    # 가운데가 막혀 오면 미리 옆으로 비킬 길을 찾는다 (물체 너머 1.5 m 까지 비어 있어야 함)
                    free = [o for o in OFFSETS[1:] if nearest_in_band(pts, o, look=center + 1.5) == math.inf]
                    if free:
                        self.offset = free[0]
                        ahead = min(nearest_in_band(pts, self.offset, look=seg_look), body_ahead)
                        ev("lidar", f"라이다: 앞 {center:.1f} m 물체 → {'왼쪽' if free[0] > 0 else '오른쪽'}으로 "
                                    f"{abs(free[0]):.2f} m 비켜 감")
                # 비켜 간 뒤 길이 비면 차선 가운데로 돌아온다. 전에는 비켜 간 줄(offset)을 구간 끝까지 그대로 두고 그동안 속도를
                # 0.5 m/s 로 묶어서, 사람이 한 번 다가온 뒤로 통로를 끝까지 느리게 갔다.
                # 돌아오는 조건: 가운데 앞이 충분히 비었고(비켜 가기를 다시 시작하는 거리보다 더 멀리), 로봇 옆·뒤 1.5 m 의 가운데 띠에도
                # 물체가 없을 때 (물체를 완전히 지나친 뒤), 그 상태가 20프레임 이어지면
                if self.offset != 0.0:
                    side_clear = len(pts) == 0 or not ((pts[:, 0] > -1.5) & (pts[:, 0] <= 0.35)
                                                        & (np.abs(pts[:, 1]) < HALF_W + 0.1)).any()
                    self._clear_count = self._clear_count + 1 if (center > DODGE_DIST + 0.5 and side_clear) else 0
                    if self._clear_count >= 20:
                        self.offset = 0.0
                        self._clear_count = 0
                        ahead = min(nearest_in_band(pts, 0.0, look=seg_look), body_ahead)
                        ev("lidar", "라이다: 앞이 비어 차선 가운데로 돌아옴")
                else:
                    self._clear_count = 0
                self.ahead = ahead                     # 실제 멈춤·감속 판단에 쓴 거리 (안전 영역 표시 색에 사용)
                if ahead < STOP_DIST:
                    drive(0.0, 0.0)
                    self._moving = False
                    self._fwd = False
                    prog = (sim_time(), rem)
                    blocked_since = blocked_since or sim_time()
                    # 정지한 채 주변 공간을 탐색한다: 앞에 사람이 갑자기 끼어든 경우처럼 미리 비키지 못했어도,
                    # 비킬 틈이 보이면 그쪽으로 제자리 회전해 몸을 틀고 우회한다 (틈이 없으면 BLOCK_SEC 뒤 옆 통로로 돌아간다)
                    cand = sorted((o for o in OFFSETS[1:]
                                   if nearest_in_band(pts, o, look=max(center, 0.6) + 1.5) == math.inf), key=abs)
                    if cand and backup_t0 is not False:
                        rear_clear = not ((pts[:, 0] < -0.6) & (pts[:, 0] > -1.3) & (np.abs(pts[:, 1]) < HALF_W)).any()
                        if backup_t0 is None:
                            backup_t0 = sim_time()
                        if rear_clear and center < BACKUP_TO and sim_time() - backup_t0 < 4.0:
                            if self.offset not in cand:             # 지금 길(비킴 폭)이 막혔으면 더 넓게 비키는 길로 바꾼다
                                self.offset = cand[0]
                                ev("lidar", f"라이다: 앞 {center:.1f} m 에서 정지 → 주변 탐색, 뒤로 물러난 뒤 "
                                            f"{'왼쪽' if cand[0] > 0 else '오른쪽'}으로 {abs(cand[0]):.2f} m 비켜 감")
                            drive(-0.25, 0.0)                       # 비킬 자리가 나오도록 곧게 뒤로 물러난다
                            self._moving = True
                            blocked_since = sim_time()              # 물러나는 동안은 '막힘' 시간을 세지 않는다
                            await self._tick()
                            continue
                        backup_t0 = False                           # 이번 구간에서는 한 번만 물러난다
                    if sim_time() - blocked_since > BLOCK_SEC:
                        await self._stop_robot()
                        return "lidar_blocked", i
                    await self._tick()
                    continue
                blocked_since = None
                if ahead < SLOW_DIST:
                    v_lim = min(v_lim, max(0.3, V_MAX * (ahead - STOP_DIST) / (SLOW_DIST - STOP_DIST)))
                if self.offset != 0.0:
                    v_lim = min(v_lim, 0.5)
            # 앞보기 점: 지금 구간 위 투영점에서 경로를 따라 look_d 만큼 (구간을 넘어가며)
            k, d_left, along = i, look_d, proj
            while k < n - 1 and along + d_left > L[k]:
                d_left -= (L[k] - along)
                k, along = k + 1, 0.0
            along = min(L[k], along + d_left)
            off = self.offset if k == i else 0.0
            px = P[k][0] + U[k][0] * along - U[k][1] * off
            py = P[k][1] + U[k][1] * along + U[k][0] * off
            alpha = (math.atan2(py - y, px - x) - yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(alpha) > math.radians(60):
                drive(0.0, math.copysign(max(0.15, min(W_MAX, 1.2 * abs(alpha))), alpha))   # 많이 틀어졌으면 제자리 회전
                self._v_cmd = 0.0
                self._moving = True
                self._fwd = False
                prog = (sim_time(), rem)          # 제자리 회전 중에는 '걸림' 시계를 멈춘다
            else:
                v = min(v_lim, 0.8 * rem + 0.15) * max(0.4, math.cos(alpha))
                now = sim_time()
                yaw_rate = 0.0
                if self._prev is not None and now > self._prev[1]:
                    yaw_rate = ((yaw - self._prev[0] + math.pi) % (2 * math.pi) - math.pi) / (now - self._prev[1])
                # 가속 한계 (감속은 바로): 직전 명령에서 ACCEL × 경과 시간만큼만 올린다
                dt = (now - self._prev[1]) if self._prev is not None and now > self._prev[1] else 1.0 / 60
                v = min(v, self._v_cmd + ACCEL * dt)
                self._v_cmd = v
                w = 1.2 * max(v, 0.3) * math.sin(alpha) / look_d - 0.35 * yaw_rate   # 느릴 때도 방향은 잡는다
                drive(v, max(-W_DRIVE, min(W_DRIVE, w)))
                self._moving = v > 0.1
                self._fwd = v > 0.1
            self._prev = (yaw, sim_time())
            await self._tick()
        await self._stop_robot()
        return "stopped", i

    async def _checkpoint(self, aisle, on_checkpoint, event):
        """통로 앞에서 Cosmos 판단. True=통과 / False=막힘 (판단은 두 가지만)."""
        event("checkpoint", f"통로{amr_scene.aisle_no(aisle)} 앞에서 정지, 판단 요청")
        await self._stop_robot()
        r = await on_checkpoint(aisle)
        ok, why = r if isinstance(r, tuple) else (bool(r), None)         # (통과 여부, 막힌 이유) 또는 True/False
        # 위험 요소가 사람뿐이면 지나가지 않고 그 자리에서 기다린다 (사람은 곧 비킨다). 비켜서 통과로 바뀌면 그대로 지나가고,
        # 끝내 안 비키면 아래에서 막힘(사람)으로 처리해 옆 통로로 돌아간다
        waits = 0
        while not ok and why == "person" and waits < PERSON_RECHECKS and not self.stop_requested:
            waits += 1
            event("wait", f"통로{amr_scene.aisle_no(aisle)} 에 사람이 있습니다 → 지나가지 않고 대기, "
                          f"{PERSON_RECHECK_SEC:.0f}초 뒤 다시 판단 ({waits}/{PERSON_RECHECKS})")
            t0 = sim_time()
            while sim_time() - t0 < PERSON_RECHECK_SEC and not self.stop_requested:
                await self._tick()
            if self.stop_requested:
                break
            r = await on_checkpoint(aisle)
            ok, why = r if isinstance(r, tuple) else (bool(r), None)
        self.judged[aisle] = bool(ok)
        self.person_blocked.discard(aisle)
        if not ok:
            self.blocked.add(aisle)
            if why == "person":
                self.person_blocked.add(aisle)
                event("blocked", f"통로{amr_scene.aisle_no(aisle)} 막힘 (사람) → 옆 통로로. 갈 길이 없으면 기다렸다 다시 판단")
            else:
                event("blocked", f"통로{amr_scene.aisle_no(aisle)} 막힘 → 옆 통로로")
        else:
            event("pass", f"통로{amr_scene.aisle_no(aisle)} 통과")
        return bool(ok)

    async def drive_to(self, goal, on_checkpoint=None, on_event=None):
        """실제 AMR 에 가깝게:
          - 지점마다 멈추지 않고 경로를 이어서 달린다 (모퉁이는 감속하며 돌아 나감)
          - Cosmos 판단은 통로 앞 복도 모퉁이에서 통로를 바라보고 한다 → 막히면 그 자리에서 옆 통로 쪽으로 90도만 돈다
          - 통로 안에서 라이다로 막힌 걸 알면 1 m 후진한 뒤 돌아서 옆 통로로 간다"""
        def event(kind, text):
            self.log.append((kind, text))
            if on_event:
                on_event(kind, text)

        for _ in range(5):                          # stop 직후 한 프레임 쉬어야 play 가 먹힌다
            await omni.kit.app.get_app().next_update_async()
        if self.use_lidar and omni.timeline.get_timeline_interface().is_stopped():
            ensure_lidar()                          # 센서 프림은 멈춘 상태에서 만든다
            reset_lidar()
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
        await self._ensure_playing()
        for _ in range(30):                         # 물리가 자리 잡을 시간
            await self._tick()
        self.stop_requested = False
        fails = {}
        person_waits = 0
        start_pt = None                             # 후진 뒤처럼 로봇이 지점 위에 있지 않을 때의 출발점
        via_start = False                           # True 면 start_pt 에서 path[0](통로 앞 지점)을 먼저 거쳐 간다: 통로 안에서 막혀 돌아 나올 때
        while self.node != goal and not self.stop_requested:
            path = self.plan(self.node, goal)
            if path is None and self.person_blocked and person_waits < PERSON_WAIT_MAX:
                # 갈 길이 없는데 그중 일부는 '사람 때문에' 막혔다고 했다 → 사람이 비킬 때까지 기다렸다가 그 통로만 다시 판단
                person_waits += 1
                waiting = sorted(self.person_blocked)
                for a_ in waiting:
                    self.blocked.discard(a_)
                    self.judged.pop(a_, None)
                self.person_blocked.clear()
                event("wait", f"갈 수 있는 통로가 없습니다 (사람 때문에 막힌 통로: {', '.join(waiting)}) → "
                              f"{PERSON_WAIT_SEC:.0f}초 기다린 뒤 다시 판단 ({person_waits}/{PERSON_WAIT_MAX})")
                await self._stop_robot()
                t0 = sim_time()
                while sim_time() - t0 < PERSON_WAIT_SEC and not self.stop_requested:
                    await self._tick()
                continue
            if path is None:
                event("stuck", f"{amr_scene.display_name(goal)} 로 가는 길이 모두 막혔습니다 (막힌 통로: {sorted(self.blocked)})")
                await self._stop_robot()
                return False
            event("plan", " → ".join(amr_scene.display_name(n) for n in path))
            try:                                    # 경로 띠: 계획이 바뀔 때마다 갈아 끼운다 (뷰포트 표시용)
                import path_ribbon
                head = [start_pt, self.nodes[path[0]]] if (start_pt and via_start) else [start_pt or self.nodes[path[0]]]
                path_ribbon.set_path(head + [self.nodes[nm] for nm in path[1:]])
            except Exception:
                pass
            # 이번에 끊어 갈 곳: 아직 판단 안 한 통로가 나오면 그 통로 간선이 시작하는 지점(통로 앞 지점 [열,0])에서 멈춘다
            stop_at, cp_aisle = len(path) - 1, None
            if on_checkpoint:
                for k in range(0, len(path) - 1):
                    a = self.aisle_of(path[k], path[k + 1])
                    if a and a not in self.judged:
                        stop_at, cp_aisle = k, a
                        break
            if stop_at > 0:
                head = [start_pt, self.nodes[path[0]]] if (start_pt and via_start) else [start_pt or self.nodes[path[0]]]
                P = head + [self.nodes[nm] for nm in path[1:stop_at + 1]]
                st, seg = await self._follow_path(P, event)
                start_pt = None
                via_start = False
                if st in ("stopped", "sensor_fail"):
                    if st == "stopped":
                        event("stopped", "정지했습니다")
                    await self._stop_robot()
                    return False
                if st == "lidar_blocked":
                    a_nm, b_nm = path[seg], path[seg + 1]
                    aisle = self.aisle_of(a_nm, b_nm)
                    where = f"통로{amr_scene.aisle_no(aisle)}" if aisle else f"{a_nm}→{b_nm} 구간"
                    if aisle:
                        if self.on_lidar_block:
                            self.on_lidar_block(aisle)             # 라이다가 막힘을 감지한 순간 바로 (Cosmos 의 이유가 오기 전에, Cosmos 를 안 쓰는 모드에서도)
                        if on_checkpoint and self.judged.get(aisle):
                            # 입구에서는 통과였는데 안에서 막힘 = 돌발 상황. 라이다가 먼저 멈췄고, Cosmos 에게 무슨 일인지 묻는다
                            event("lidar", f"라이다: {where} 안에서 갑자기 막힘 → 정지, Cosmos 에게 상황 확인")
                            await on_checkpoint(aisle, lidar_blocked=True)      # 결과와 상관없이 라이다 판단(막힘)을 따른다 (안전 우선). Cosmos 는 이유만 말한다
                        self.blocked.add(aisle)
                        self.judged[aisle] = False
                        event("lidar", f"라이다: {where} 앞이 막혀 지나갈 틈이 없음 → 1 m 후진 후 돌아서 옆 통로로")
                        await self._backup(event)
                        via_start = True                              # 통로 안에서 나오는 길: 통로 앞 지점까지 되돌아 나온 뒤 옆 통로로
                    else:
                        fails[(a_nm, b_nm)] = fails.get((a_nm, b_nm), 0) + 1
                        if fails[(a_nm, b_nm)] >= SEGMENT_RETRIES:
                            event("stuck", f"라이다: {where} 이 계속 막혀 있어 멈춥니다")
                            await self._stop_robot()
                            return False
                        event("lidar", f"라이다: {where} 막힘 → 조금 물러나 다시 시도")
                        await self._backup(event, 0.5)
                    self.node = a_nm                      # 그 구간 시작 지점에서 다시 계획 (출발점은 지금 위치)
                    start_pt = pose()[:2]
                    continue
                self.node = path[stop_at]
            if cp_aisle:
                # 통로를 바라보고 판단 (모퉁이 → 입구 방향)
                fx, fy = self.nodes[path[stop_at]]
                ex, ey = self.nodes[path[stop_at + 1]]
                await self._turn_to(math.degrees(math.atan2(ey - fy, ex - fx)))
                if self.stop_requested:
                    event("stopped", "정지했습니다")
                    return False
                await self._checkpoint(cp_aisle, on_checkpoint, event)
                continue                                  # 통과면 다음 계획에서 그 통로로, 막힘이면 옆 통로로
        await self._stop_robot()
        if self.node == goal:
            event("arrived", f"{amr_scene.display_name(goal)} 도착")
            return True
        return False
