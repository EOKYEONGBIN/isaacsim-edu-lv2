# 주행 중 돌발 상황: 창고를 걸어 다니는 작업자 3명 (익스텐션의 '작업자' 토글로 켜고 끈다)
#
#   import dynamic_events
#   dynamic_events.start(on_event=lambda kind, text: print(kind, text))   # 주행 시작 전, 타임라인이 멈춘 상태에서
#   ...                                                                    # 이후 Play 중 매 프레임 자동으로 움직인다
#   dynamic_events.stop()                                                  # 끝나면 작업자를 숨긴다
#
# - 작업자: 통로·복도 그래프(amr_scene.graph)를 따라 무작위로 걷는다. 로봇이 가까이 오면 옆으로 비켜 주고,
#           로봇 바로 앞에서는 멈춘다 (로봇과 부딪히거나 서로 막혀 주행이 멈추지 않게).
#           걷는 동작은 SkelAnimation 관절 값을 매 프레임 걸은 거리에 맞춰 바꿔서 만든다 (발이 미끄러지지 않음).
#           라이다에 잡히도록 보이지 않는 키네마틱 원기둥 충돌체를 작업자와 같이 움직인다.

import builtins
import math
import random

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdSkel, Vt

import amr_scene
import incidents

PERSON = "/Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd"
WORKER_ROOT = "/World/Workers"
MAX_WORKERS = 3
WORKERS = (3, 3)             # 한 번 주행에 나오는 작업자 수
WALK_SPEED = (0.9, 1.3)      # m/s
STRIDE = 1.5                 # 한 주기(두 걸음)에 걷는 거리 m
KEEP_OUT_START = 4.0         # 주행 시작 때 로봇 시작점에서 이만큼은 비워 둔다
# 작업자 주변 '센서 구역': 로봇이 R_SENSE 안에 들어오면 로봇의 위치·속도로 가장 가까워질 때를 미리 계산하고,
# 그 거리가 R_SAFE 보다 작으면 비켜 걷는다 (옆으로 비킴 → 로봇이 앞에서 오면 돌아서 반대로 → 뒤에서 오면 서둘러 앞으로).
# R_HARD 안으로 들어오면 로봇에서 멀어지는 쪽으로 직접 밀어낸다. 로봇 반폭(0.58)+작업자 반경(0.3)+여유
R_SENSE, R_SAFE, R_HARD = 3.5, 1.7, 1.25
P_SENSE, P_SAFE, P_HARD = 2.8, 1.1, 0.9   # 작업자끼리: 살피는 거리 / 비키는 거리 / 맞닿기 직전 (로봇보다 가깝게 지나가도 된다)
S_BEHIND = 1.5               # 길 시작점 뒤로 이만큼까지는 음수 s 로 서 있을 수 있다 (모퉁이에서 위치가 튀지 않게)
OBS_R = 0.45                # 박스·표지판 하나를 원으로 본 반경 m
OBS_LOOK = 1.8               # 걷는 방향 이 거리 안에 물체가 있으면 돌아선다
OBS_PAD = 0.7                # 작업자 몸 쪽 여유 (물체 반경에 더해 옆으로 이만큼 안이면 부딪힌다고 본다)

# 걷는 자세 (관절 각도, 도). 축·부호는 이 캐릭터(RL_BoneRoot 뼈대)에서 화면으로 확인해 정했다
A_THIGH, A_CALF, A_ARM = 28.0, 40.0, 22.0
FINGER_CURL = (20.0, 35.0, 25.0)         # 손가락 마디 1·2·3 을 굽히는 각도 (편 손바닥이 부자연스러워서 살짝 쥔 모양으로)
FINGER_CURL_THUMB = (0.0, 0.0, 0.0)      # 엄지는 펴 둔다
# 손가락 굽힘 축·방향 (손끝 위치를 직접 재서 정함): 이 뼈대에서 손가락 관절을 X 축으로 돌리면 손끝이 몸 앞뒤로
# 흔들리고(굽는 게 아님), Z 축으로 돌려야 손끝이 몸 중심선 쪽으로 말려 들어간다. 왼손·오른손은 거울이라 부호가 반대다.
FINGER_SIGN_L, FINGER_SIGN_R = -1.0, 1.0
ELBOW_BASE, ELBOW_SWING = 28.0, 22.0   # 팔꿈치: 걷는 동안 항상 28° 굽히고, 팔이 앞으로 갈 때 더 굽힌다 (+X 축 = 앞으로 굽힘, 화면으로 확인)
AXIS_X = Gf.Vec3d(1, 0, 0)
AXIS_Z = Gf.Vec3d(0, 0, 1)
THIGH_SIGN = 1.0             # +1 이면 +각도가 다리를 앞으로
ARM_SIGN = 1.0               # +1 이면 +각도가 팔을 앞으로

_ANIM_JOINTS = ("L_Thigh", "R_Thigh", "L_Calf", "R_Calf", "L_Upperarm", "R_Upperarm", "L_Forearm", "R_Forearm")


def _st():
    s = getattr(builtins, "_amr_dyn", None)
    if s is None:
        s = {"sub": None, "workers": [], "on_event": None, "rng": random.Random(), "last_t": None,
             "rig": None, "adj": None, "nodes": None, "active": False, "rv": (0.0, 0.0), "rpos": None}
        builtins._amr_dyn = s
    return s


def _sim_time():
    return omni.timeline.get_timeline_interface().get_current_time()


# ---------------------------------------------------------------- 걸어 다닐 길
def _walk_graph():
    """작업자가 다닐 수 있는 길: 앞·뒤 복도, 통로, 입구 연결선 (시작점·목적지 번호 원은 제외)."""
    s = _st()
    if s["adj"] is None:
        nodes, edges = amr_scene.graph()
        skip = {"START"} | set(amr_scene.GOALS)
        adj = {}
        for a, b, _aisle in edges:
            if a in skip or b in skip:
                continue
            adj.setdefault(a, []).append(b)
            adj.setdefault(b, []).append(a)
        s["adj"], s["nodes"] = adj, nodes
    return s["nodes"], s["adj"]


def _set_edge(w, a, b):
    nodes, _adj = _walk_graph()
    (ax, ay), (bx, by) = nodes[a], nodes[b]
    L = math.hypot(bx - ax, by - ay)
    w.update(a=a, b=b, L=L, u=((bx - ax) / L, (by - ay) / L))
    vertical = abs(bx - ax) < 1e-6                        # 통로 쪽(세로) 길은 좁다 (통로 반폭 약 1.55 m)
    w["lim"] = 1.0 if vertical else 1.3
    rnd = _st()["rng"]
    w["lat_t"] = rnd.uniform(-w["lim"], w["lim"]) * 0.8


# ---------------------------------------------------------------- 작업자 만들기
def _finger_curl(name):
    """손가락 관절(L_/R_ Pinky·Ring·Mid·Index·Thumb 1~3)이면 굽힐 각도, 아니면 0. 발가락(...Toe)은 제외."""
    if len(name) < 3 or name[:2] not in ("L_", "R_") or "Toe" in name or name[-1] not in "123":
        return 0.0
    base = name[2:-1]
    if base not in ("Pinky", "Ring", "Mid", "Middle", "Index", "Thumb"):
        return 0.0
    table = FINGER_CURL_THUMB if base == "Thumb" else FINGER_CURL
    return table[int(name[-1]) - 1]


def _rig_info(stage, skel_prim):
    """뼈대 정보(관절 이름, 기본 자세)를 한 번만 읽어 둔다. 팔은 내린 자세(idle)를 기본으로 한다."""
    s = _st()
    if s["rig"] is not None:
        return s["rig"]
    sk = UsdSkel.Skeleton(skel_prim)
    joints = list(sk.GetJointsAttr().Get())
    rest = list(sk.GetRestTransformsAttr().Get())
    T, R, S = [], [], []
    for jn, mtx in zip(joints, rest):
        m4 = Gf.Matrix4d(mtx)
        r = m4.ExtractRotationQuat()
        name = jn.split("/")[-1]
        if name in ("L_Upperarm", "R_Upperarm"):
            r = r * Gf.Rotation(AXIS_Z, -70.0 if name.startswith("L") else 70.0).GetQuat()   # 팔 내리기
        curl = _finger_curl(name)
        if curl:                                                                              # 손가락 살짝 쥐기
            r = r * Gf.Rotation(AXIS_Z, (FINGER_SIGN_L if name.startswith("L") else FINGER_SIGN_R) * curl).GetQuat()
        T.append(Gf.Vec3f(m4.ExtractTranslation()))
        R.append(r)
        S.append(Gf.Vec3h(1, 1, 1))
    idx = {n: next(i for i, j in enumerate(joints) if j.split("/")[-1] == n) for n in _ANIM_JOINTS}
    s["rig"] = {"joints": joints, "T": Vt.Vec3fArray(T), "R": R, "Rf": [Gf.Quatf(q) for q in R],
                "S": Vt.Vec3hArray(S), "idx": idx}
    return s["rig"]


def _make_worker(stage, i):
    g = f"{WORKER_ROOT}/W{i}"
    incidents._place(stage, g, PERSON, 0.0, 0.0, yaw=0.0, collide=False)
    skel = next((q for q in Usd.PrimRange(stage.GetPrimAtPath(g)) if q.GetTypeName() == "Skeleton"), None)
    rig = _rig_info(stage, skel)
    anim = UsdSkel.Animation.Define(stage, g + "/walk")
    anim.CreateJointsAttr(rig["joints"])
    anim.CreateTranslationsAttr(rig["T"])
    anim.CreateRotationsAttr(Vt.QuatfArray([Gf.Quatf(q) for q in rig["R"]]))
    anim.CreateScalesAttr(rig["S"])
    UsdSkel.BindingAPI.Apply(skel).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
    # 라이다·로봇 충돌용 보이지 않는 키네마틱 원기둥 (작업자와 같이 움직인다)
    col_path = f"{WORKER_ROOT}/W{i}_col"
    col = UsdGeom.Cylinder.Define(stage, col_path)
    col.CreateRadiusAttr(0.3)
    col.CreateHeightAttr(1.7)
    col.CreateAxisAttr("Z")
    UsdGeom.XformCommonAPI(col).SetTranslate(Gf.Vec3d(0, 0, -5.0))
    UsdGeom.Imageable(col).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(col.GetPrim()).CreateKinematicEnabledAttr(True)
    ops = {op.GetOpName(): op for op in UsdGeom.Xformable(stage.GetPrimAtPath(g)).GetOrderedXformOps()}
    col_ops = {op.GetOpName(): op for op in UsdGeom.Xformable(col.GetPrim()).GetOrderedXformOps()}
    return {"i": i, "g": g, "anim": anim, "t_op": ops["xformOp:translate"], "r_op": ops["xformOp:rotateXYZ"],
            "col": col.GetPrim(), "col_t": col_ops["xformOp:translate"], "x": 0.0, "y": 0.0, "yaw": 0.0, "lat": 0.0,
            "s": 0.0, "speed": 1.0, "phase": 0.0, "amp": 0.0, "a": None, "b": None, "turn_t": -99.0, "obs_t": -99.0}


def _ensure_workers(stage):
    s = _st()
    if not stage.GetPrimAtPath(WORKER_ROOT):
        UsdGeom.Xform.Define(stage, WORKER_ROOT)
        s["workers"] = []
    for w in list(s["workers"]):
        if not stage.GetPrimAtPath(w["g"]):
            s["workers"].remove(w)
    while len(s["workers"]) < MAX_WORKERS:
        s["workers"].append(_make_worker(stage, len(s["workers"])))
    return s["workers"]


def _set_visible(stage, w, on):
    w["on"] = bool(on)
    img = UsdGeom.Imageable(stage.GetPrimAtPath(w["g"]))
    img.MakeVisible() if on else img.MakeInvisible()
    w["col"].GetAttribute("physics:collisionEnabled").Set(bool(on)) if w["col"].HasAttribute("physics:collisionEnabled") \
        else UsdPhysics.CollisionAPI(w["col"]).CreateCollisionEnabledAttr(bool(on))
    if not on:
        w["col_t"].Set(Gf.Vec3d(0, 0, -5.0))


def _place_workers(stage, count, rnd):
    """작업자를 길 위 무작위 자리에 놓는다 (로봇 시작점·다른 작업자와 떨어뜨려서)."""
    nodes, adj = _walk_graph()
    edges = [(a, b) for a in adj for b in adj[a] if a < b]
    _refresh_obstacles()
    sx, sy = amr_scene.START
    placed = []
    for k, w in enumerate(_ensure_workers(stage)):
        on = k < count
        _set_visible(stage, w, on)
        if not on:
            continue
        for _try in range(60):
            a, b = rnd.choice(edges)
            if rnd.random() < 0.5:
                a, b = b, a
            _set_edge(w, a, b)
            w["s"] = rnd.uniform(0.0, w["L"])
            w["lat"] = w["lat_t"]
            _update_pos(w)
            if math.hypot(w["x"] - sx, w["y"] - sy) < KEEP_OUT_START:
                continue
            if any(math.hypot(w["x"] - p[0], w["y"] - p[1]) < 3.0 for p in placed):
                continue
            if _near_obstacle(w["x"], w["y"], pad=1.2) or _edge_blocked(a, b):
                continue
            break
        w["speed"] = rnd.uniform(*WALK_SPEED)
        w["phase"] = rnd.uniform(0, STRIDE)
        w["amp"] = 0.0
        # 돌아서기 쿨다운 기준 시각은 주행마다 초기화한다. 타임라인은 Stop/Play 때마다 0 초로 돌아가는데 예전 주행 값이
        # 남아 있으면(예: 70 초) '3 초 안에 이미 돌아섰다' 로 계속 판단돼, 로봇이 앞에 와도 돌아서지 못하고 서서 길을 막았다
        w["turn_t"] = -99.0
        w["obs_t"] = -99.0
        w["vx"], w["vy"] = 0.0, 0.0
        w["yaw"] = math.degrees(math.atan2(w["u"][0], -w["u"][1]))
        placed.append((w["x"], w["y"]))
        _apply(w)
        _pose(w)


# ---------------------------------------------------------------- 움직임
def _update_pos(w):
    ux, uy = w["u"]
    nodes, _ = _walk_graph()
    ax, ay = nodes[w["a"]]
    w["x"] = ax + ux * w["s"] - uy * w["lat"]
    w["y"] = ay + uy * w["s"] + ux * w["lat"]


def _apply(w):
    w["t_op"].Set(Gf.Vec3d(w["x"], w["y"], 0.0))
    w["r_op"].Set(Gf.Vec3f(0.0, 0.0, w["yaw"]))
    w["col_t"].Set(Gf.Vec3d(w["x"], w["y"], 0.85))


def _pose(w):
    """걸은 거리(phase)에 맞춰 다리·팔 관절을 흔든다. amp 가 0 이면 서 있는 자세.
    바뀌는 관절 8개만 새로 계산하고 나머지는 미리 만든 값을 재사용한다 (매 프레임·작업자 4명이라 가볍게)."""
    rig = _st()["rig"]
    ph = 2 * math.pi * w["phase"] / STRIDE
    amp = w["amp"]
    Rf = list(rig["Rf"])
    base = rig["R"]
    idx = rig["idx"]
    sw = math.sin(ph)
    cp = math.cos(ph)
    for name, axis, deg in (
            ("L_Thigh", AXIS_X, THIGH_SIGN * A_THIGH * amp * sw),
            ("R_Thigh", AXIS_X, -THIGH_SIGN * A_THIGH * amp * sw),
            ("L_Calf", AXIS_X, -THIGH_SIGN * A_CALF * amp * max(0.0, cp)),
            ("R_Calf", AXIS_X, -THIGH_SIGN * A_CALF * amp * max(0.0, -cp)),
            ("L_Upperarm", AXIS_X, -ARM_SIGN * A_ARM * amp * sw),
            ("R_Upperarm", AXIS_X, ARM_SIGN * A_ARM * amp * sw),
            ("L_Forearm", AXIS_X, amp * (ELBOW_BASE - ELBOW_SWING * sw)),      # 오른팔이 앞으로 갈 때(sw>0) 왼팔은 뒤로: 덜 굽힘
            ("R_Forearm", AXIS_X, amp * (ELBOW_BASE + ELBOW_SWING * sw))):
        i = idx[name]
        Rf[i] = Gf.Quatf(base[i] * Gf.Rotation(axis, deg).GetQuat())
    w["anim"].GetRotationsAttr().Set(Vt.QuatfArray(Rf))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _reverse(w):
    """지금 걷던 길을 되돌아간다 (같은 자리에서 반대 방향으로). 위치는 그대로, 바라보는 방향만 바뀐다."""
    a, b, s, L, lat = w["a"], w["b"], w["s"], w["L"], w["lat"]
    _set_edge(w, b, a)
    w["s"], w["lat"] = L - s, -lat
    w["lat_t"] = w["lat"]


# ---------------------------------------------------------------- 물체 피하기
def _refresh_obstacles():
    """작업자가 피할 물체(정적 박스·표지판)의 위치를 읽어 둔다. [(x, y, 반경)]"""
    stage = omni.usd.get_context().get_stage()
    obs = []
    root = stage.GetPrimAtPath(incidents.ROOT_PATH) if stage else None
    if root:
        for grp in root.GetChildren():
            if grp.GetName() != "dynamic":
                for q in grp.GetChildren():
                    if q.GetName().startswith(incidents.avoid_prefixes()):
                        p = UsdGeom.Xformable(q).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                        obs.append((p[0], p[1], OBS_R))
    _st()["obs"] = obs
    return obs


def _near_obstacle(x, y, pad=0.0):
    for ox, oy, r in _st().get("obs", ()):
        if math.hypot(x - ox, y - oy) < r + pad:
            return True
    return False


def _edge_blocked(a, b):
    """길(a→b) 위에 물체가 있는가 (선분에서 OBS_R + 0.6 m 안)."""
    nodes, _ = _walk_graph()
    (ax, ay), (bx, by) = nodes[a], nodes[b]
    L2 = (bx - ax) ** 2 + (by - ay) ** 2
    for ox, oy, r in _st().get("obs", ()):
        t = _clamp(((ox - ax) * (bx - ax) + (oy - ay) * (by - ay)) / L2, 0.0, 1.0) if L2 > 1e-9 else 0.0
        if math.hypot(ox - (ax + t * (bx - ax)), oy - (ay + t * (by - ay))) < r + 0.6:
            return True
    return False


def _obstacle_ahead(w):
    """걷는 방향 OBS_LOOK m 안에 몸이 닿을 물체가 있으면 그 거리, 없으면 None."""
    ux, uy = w["u"]
    best = None
    for ox, oy, r in _st().get("obs", ()):
        rx, ry = ox - w["x"], oy - w["y"]
        along = rx * ux + ry * uy
        perp = abs(-rx * uy + ry * ux)
        if 0.0 < along < OBS_LOOK + r and perp < r + OBS_PAD:
            best = along if best is None else min(best, along)
    return best


def _peer_avoid(w, ux, uy, nx, ny, lat_t, speed_f):
    """다른 작업자와 부딪히지 않게: 비킬 쪽(번호가 큰 작업자)만 옆걸음·감속·정지하고, 맞닿기 직전엔 둘 다 서로 밀려난다.
    돌려주는 값: (lat_t, speed_f, 밀려남 (px, py) 방향 단위 벡터와 거리, 기다리는 중인가)."""
    push = [0.0, 0.0]
    waiting = False
    for o in _st()["workers"]:
        if o is w or not o.get("on"):
            continue
        dx, dy = w["x"] - o["x"], w["y"] - o["y"]
        d = math.hypot(dx, dy)
        if d < P_HARD and d > 1e-3:                                   # 맞닿기 직전: 서로 반대쪽으로
            k = (P_HARD - d) / d
            push[0] += dx * k
            push[1] += dy * k
        if d >= P_SENSE:
            continue
        if w["i"] < o["i"]:                                           # 번호가 작은 쪽은 옆으로 비키지 않는다 (둘 다 비키면 같은 쪽으로 비켜 서로 막힌다)
            if -(dx * ux + dy * uy) > 0 and d < P_SAFE + 0.6:         # 다만 바로 앞이면 속도는 줄여 상대가 비킬 시간을 준다
                speed_f = min(speed_f, 0.5)
            continue
        dvx, dvy = w.get("vx", 0.0) - o.get("vx", 0.0), w.get("vy", 0.0) - o.get("vy", 0.0)
        dv2 = dvx * dvx + dvy * dvy
        t_star = _clamp(-(dx * dvx + dy * dvy) / dv2, 0.0, 3.0) if dv2 > 1e-4 else 0.0
        cx, cy = dx + dvx * t_star, dy + dvy * t_star
        closest = math.hypot(cx, cy)
        if closest >= P_SAFE and d >= P_SAFE:
            continue
        if closest > 0.05:                                            # 최접근 위치에서 멀어지는 쪽으로 옆걸음
            shift = (cx * nx + cy * ny) / closest * (P_SAFE - closest + 0.2)
        else:
            shift = (P_SAFE + 0.2) * (1.0 if w["lat"] >= 0 else -1.0)
        new_lat = _clamp(w["lat"] + shift, -w["lim"], w["lim"])
        lat_t = new_lat
        ahead = -(dx * ux + dy * uy) > 0                              # 상대가 내가 가는 방향 앞쪽
        room = abs(new_lat - w["lat"])                                # 옆으로 비킬 수 있는 만큼
        if ahead and d < P_SAFE + 0.6:
            if room < 0.5 * abs(shift):                               # 좁아서 못 비킨다 → 속도를 줄이고 멈춰 기다린다
                speed_f = min(speed_f, 0.0 if d < P_SAFE + 0.2 else 0.4)
                waiting = True
            else:
                speed_f = min(speed_f, 0.6)
    return lat_t, speed_f, push, waiting


def _rebase(w, x, y):
    """새 길(w["a"]→w["b"])로 바꾼 뒤, 세계 좌표 (x, y) 에 그대로 서 있도록 s·lat 을 다시 계산한다.
    예전에는 lat 만 그대로 가져가서, 길 방향이 90° 바뀌는 모퉁이에서 위치가 1 m 넘게 튀었다
    (통로 끝에서 작업자가 로봇 옆 0.28 m 로 순간이동)."""
    nodes, _ = _walk_graph()
    ax, ay = nodes[w["a"]]
    rx, ry = x - ax, y - ay
    ux, uy = w["u"]
    w["s"] = max(-S_BEHIND, rx * ux + ry * uy)        # 모퉁이에서는 새 길의 시작점 '뒤' 에 서 있을 수 있다 (음수 s 허용)
    w["lat"] = _clamp(-rx * uy + ry * ux, -w["lim"], w["lim"])


def _move(w, dt, rx, ry, rvx, rvy, rnd, now):
    ux, uy = w["u"]
    nx, ny = -uy, ux
    dx, dy = w["x"] - rx, w["y"] - ry
    dist = math.hypot(dx, dy)
    lat_t = w["lat_t"]
    speed_f = 1.0
    lat_rate = 1.2
    if _st().get("obs") and now - w["obs_t"] > 1.5:
        ahead = _obstacle_ahead(w)
        if ahead is not None:                                           # 앞에 물체: 부딪히기 전에 돌아서 간다
            _reverse(w)
            w["obs_t"] = now
            ux, uy = w["u"]
            nx, ny = -uy, ux
            lat_t = w["lat"]
    if dist < R_SENSE:
        # 지금 속도로 계속 가면 3초 안에 언제 가장 가까워지고, 그때 거리는 얼마인가
        dvx, dvy = ux * w["speed"] - rvx, uy * w["speed"] - rvy
        dv2 = dvx * dvx + dvy * dvy
        t_star = _clamp(-(dx * dvx + dy * dvy) / dv2, 0.0, 3.0) if dv2 > 1e-4 else 0.0
        cx, cy = dx + dvx * t_star, dy + dvy * t_star
        closest = math.hypot(cx, cy)
        if closest < R_SAFE or dist < R_SAFE:
            robot_ahead = -(dx * ux + dy * uy)                        # >0 이면 로봇이 작업자가 가는 방향 앞쪽에 있다
            if closest > 0.05:                                        # 1) 최접근 위치에서 멀어지는 쪽으로 옆걸음
                shift = (cx * nx + cy * ny) / closest * (R_SAFE - closest + 0.2)
            else:
                shift = (R_SAFE + 0.2) * (1.0 if w["lat"] >= 0 else -1.0)
            lat_t = _clamp(w["lat"] + shift, -w["lim"], w["lim"])
            lat_rate = 2.0
            if robot_ahead > 0:                                       # 2) 로봇이 앞에서 다가온다
                if dist < 2.6 and now - w["turn_t"] > 3.0:
                    _reverse(w)                                       #    돌아서 반대로 걷는다
                    w["turn_t"] = now
                    ux, uy = w["u"]
                    nx, ny = -uy, ux
                    lat_t = w["lat"]
                elif dist < R_SAFE + 0.4:
                    speed_f = 0.0                                     #    돌아설 수 없으면 멈춰 선다
            else:
                speed_f = 1.35                                        # 3) 로봇이 뒤에서 오면 서둘러 앞으로
    robot_lat = lat_t
    lat_t, speed_f, ppush, waiting = _peer_avoid(w, ux, uy, nx, ny, lat_t, speed_f)    # 다른 작업자 피하기
    if lat_t != robot_lat:
        lat_rate = max(lat_rate, 1.6)
    w["wait"] = w.get("wait", 0.0) + dt if waiting else 0.0
    if w["wait"] > 2.0 and now - w["turn_t"] > 3.0:                   # 2 초 넘게 막혀 있으면 돌아서 간다
        _reverse(w)
        w["turn_t"] = now
        w["wait"] = 0.0
        ux, uy = w["u"]
        nx, ny = -uy, ux
        lat_t = w["lat"]
        speed_f = 1.0
    x0, y0 = w["x"], w["y"]
    w["lat"] += _clamp(lat_t - w["lat"], -lat_rate * dt, lat_rate * dt)
    step = w["speed"] * speed_f * dt
    w["s"] += step
    if ppush[0] or ppush[1]:                                          # 작업자끼리 맞닿기 직전: 직접 밀려난다
        pl = math.hypot(ppush[0], ppush[1])
        k = min(2.4 * dt, pl) / pl
        w["s"] += ppush[0] * k * ux + ppush[1] * k * uy
        w["lat"] = _clamp(w["lat"] + ppush[0] * k * nx + ppush[1] * k * ny, -w["lim"], w["lim"])
    if dist < R_HARD and dist > 1e-3:                                 # 4) 맞닿기 직전: 로봇에서 멀어지는 쪽으로 직접 민다
        push = min(1.6 * dt, R_HARD - dist + 0.05)
        px, py = dx / dist * push, dy / dist * push
        w["s"] += px * ux + py * uy
        w["lat"] = _clamp(w["lat"] + px * nx + py * ny, -w["lim"], w["lim"])
    w["s"] = max(-S_BEHIND, w["s"])
    while w["s"] >= w["L"]:                                        # 길 끝: 다음 길을 무작위로
        _update_pos(w)                                                       # 길을 바꾸기 '전' 실제 위치 (s 를 줄이기 전에 잡아야 한다)
        px, py = w["x"], w["y"]
        _nodes, adj = _walk_graph()
        opts = [n for n in adj[w["b"]] if n != w["a"]] or adj[w["b"]]
        opts = [n for n in opts if not _edge_blocked(w["b"], n)] or opts     # 물체로 막힌 길은 피한다
        _set_edge(w, w["b"], rnd.choice(opts))
        _rebase(w, px, py)                                                   # 새 길 좌표로 옮겨도 위치가 튀지 않게
    _update_pos(w)
    # 발 미끄러짐 방지: 걸음(phase)과 다리 흔드는 크기(amp)를 '실제로 움직인 거리·속도' 에 맞춘다.
    # 옆걸음(lat)·밀려남·돌아서기로 움직인 거리도 세고, 걷는 방향(몸이 바라보는 쪽)도 실제 이동 방향을 따른다.
    moved = math.hypot(w["x"] - x0, w["y"] - y0)
    if moved > 0.5:                                                 # 길이 바뀌며 위치가 튄 프레임은 걸음으로 세지 않는다
        moved = 0.0
        vx, vy = 0.0, 0.0
    else:
        vx, vy = (w["x"] - x0) / dt, (w["y"] - y0) / dt
    w["phase"] += moved
    w["vx"] = 0.8 * w.get("vx", 0.0) + 0.2 * vx
    w["vy"] = 0.8 * w.get("vy", 0.0) + 0.2 * vy
    speed_now = math.hypot(w["vx"], w["vy"])
    if speed_now > 0.3:
        target = math.degrees(math.atan2(w["vx"], -w["vy"]))        # 캐릭터는 yaw 0 에서 -Y 를 본다
    else:
        target = math.degrees(math.atan2(w["u"][0], -w["u"][1]))
    w["yaw"] += _clamp(_wrap(target - w["yaw"]), -300.0 * dt, 300.0 * dt)
    w["amp"] += _clamp(_clamp(speed_now / 1.0, 0.0, 1.0) - w["amp"], -6.0 * dt, 6.0 * dt)
    _apply(w)
    _pose(w)


# ---------------------------------------------------------------- 켜고 끄기
def _tick(_e):
    s = _st()
    if not s["workers"] or not omni.timeline.get_timeline_interface().is_playing():
        s["last_t"] = None
        return
    t = _sim_time()
    if s["last_t"] is None or t <= s["last_t"]:
        s["last_t"] = t
        return
    dt = min(0.25, t - s["last_t"])
    s["last_t"] = t
    import lidar_drive
    rx, ry, ryaw = lidar_drive.pose()
    prev = s.get("rpos")
    if prev is not None and dt > 1e-4:                              # 로봇 속도 (작업자가 부딪힐지 미리 계산하는 데 쓴다)
        k = 0.5
        s["rv"] = ((1 - k) * s["rv"][0] + k * (rx - prev[0]) / dt, (1 - k) * s["rv"][1] + k * (ry - prev[1]) / dt)
    s["rpos"] = (rx, ry)
    rvx, rvy = s["rv"]
    rnd = s["rng"]
    if t - s.get("obs_refresh", -99.0) > 0.25 or t < s.get("obs_refresh", -99.0):   # 물체 위치는 0.25 초마다 새로 읽는다
        s["obs_refresh"] = t
        _refresh_obstacles()
    for w in s["workers"]:
        if UsdGeom.Imageable(w["col"].GetStage().GetPrimAtPath(w["g"])).ComputeVisibility() != "invisible":
            _move(w, dt, rx, ry, rvx, rvy, rnd, t)


def start(on_event=None, count=None, seed=None):
    """작업자를 무작위 자리에 놓고 매 프레임 움직이기 시작한다 (타임라인이 멈춘 상태에서 부를 것)."""
    stop(hide=False)
    s = _st()
    s["rng"] = random.Random(seed)
    s["on_event"] = on_event
    s["last_t"] = None
    s["rv"], s["rpos"] = (0.0, 0.0), None
    stage = omni.usd.get_context().get_stage()
    n = count if count is not None else s["rng"].randint(*WORKERS)
    _place_workers(stage, n, s["rng"])
    s["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_tick, name="amr_dynamic_events")
    s["active"] = True
    return n


def stop(hide=True):
    """움직임을 멈추고(구독 해제) 작업자를 숨긴다."""
    s = _st()
    if s["sub"] is not None:
        try:
            s["sub"].unsubscribe()
        except Exception:
            pass
        s["sub"] = None
    s["active"] = False
    if hide:
        s["on_event"] = None                   # 주행이 끝나면 콜백을 놓는다 (확장을 다시 켠 뒤 옛 콜백이 남지 않게)
    stage = omni.usd.get_context().get_stage()
    if stage and hide:
        for w in s["workers"]:
            if stage.GetPrimAtPath(w["g"]):
                _set_visible(stage, w, False)


def remove_all():
    """작업자 프림까지 완전히 지운다 (장면 파일을 저장할 때·확장을 끌 때)."""
    stop(hide=False)
    s = _st()
    stage = omni.usd.get_context().get_stage()
    if stage:
        if stage.GetPrimAtPath(WORKER_ROOT):
            stage.RemovePrim(WORKER_ROOT)
    s["workers"] = []


def people_in_aisle(aisle, view=10.0):
    """입구에서 view m 안쪽 통로 차선(폭 ±1.55 m) 안에 걷는 작업자가 있는가 (정확도 채점용)."""
    s = _st()
    if not s["active"]:
        return False
    stage = omni.usd.get_context().get_stage()
    cx = amr_scene.AISLES[aisle]
    for w in s["workers"]:
        if UsdGeom.Imageable(stage.GetPrimAtPath(w["g"])).ComputeVisibility() == "invisible":
            continue
        if abs(w["x"] - cx) < 1.55 and amr_scene.ENTRY_Y < w["y"] < amr_scene.ENTRY_Y + view:
            return True
    return False


def effective_state(aisle, state):
    """채점용: 배치는 '비움' 인데 입구 사진에 작업자가 통로 안에 걷고 있으면 정답을 '막힘(worker)' 으로 본다."""
    return "worker" if state == "clear" and people_in_aisle(aisle) else state
