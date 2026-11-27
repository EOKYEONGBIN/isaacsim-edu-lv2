# 통로 장애물 배치 (정답 기록 포함)
#
#   import incidents
#   incidents.set_aisle("B", "boxes")      # 박스가 통로 폭 전체에 쏟아져 막힘
#   incidents.set_aisle("C", "spill")      # 바닥 액체 + 젖은 바닥 주의 표지
#   incidents.set_aisle("D", "cone")       # 통로 한가운데에 고깔 한 개
#   incidents.truth()                      # {"A": "clear", "B": "boxes", ...}  ← 채점용 정답 (Cosmos 에는 주지 않음)
#
# 장애물은 아래 @obstacle 표 한 곳에 정의한다. 새 장애물을 추가하려면 맨 아래 '장애물 정의' 구역에
# @obstacle(...) 함수를 하나 더 쓰면 된다. 익스텐션 메뉴, 프롬프트 버튼, 정답 채점, 사진 박스 라벨, 작업자 피하기가
# 모두 이 표를 읽으므로 다른 파일을 고칠 필요가 없다.
#
# 정답(통과 가능인가)은 코드에 고정하지 않는다: 익스텐션의 '장애물별 판단' 버튼(막힘 / 통과)이 정하고,
# 같은 버튼이 Cosmos 프롬프트의 줄도 정한다 → 사용자가 보낸 프롬프트대로 Cosmos 가 판단하므로 정답도 그 설정을 따른다.
#   incidents.set_rule("cone", "pass")     # 고깔은 막힘이 아니라 통과로 본다 (프롬프트 줄은 aisle_judge.set_rule 이 바꾼다)
#   incidents.passable("cone")             # → True
# 걸어 다니는 작업자(익스텐션의 작업자 토글)는 배치 옵션이 아니다 → dynamic_events.py

import math
import random

import omni.usd
from pxr import Gf, UsdGeom
from isaacsim.storage.native import get_assets_root_path

import amr_scene

PROPS = "/Isaac/Environments/Simple_Warehouse/Props/"
BOX = PROPS + "SM_CardBoxA_01.usd"          # 0.70 x 0.50 x 0.50 m
WET = PROPS + "S_WetFloorSign.usd"
CONE = PROPS + "S_TrafficCone.usd"          # 0.34 x 0.33 x 0.46 m (바닥에 닿게 만들어져 있어 z 를 올릴 필요 없음)
ROOT_PATH = "/World/Incidents"
OBSTACLE_Y = 0.5                             # 통로 입구(y=-5.6)에서 약 6 m 앞

# ---- 장애물 표 ----------------------------------------------------------------------------------
# 정의한 순서가 익스텐션 드롭다운 순서다.
#   ko       드롭다운·정답 표시에 쓰는 이름            short  결과 표(정확도 시험)에 쓰는 짧은 이름
#   name     '장애물별 판단' 버튼에 쓰는 이름 (박스, 액체 ...)
#   passable 기본 정답: 통과 가능한가 (버튼으로 바꿀 수 있다. 채점용, Cosmos 에는 정답을 주지 않는다)
#   nav2     라이다(Nav2)만 있는 AMR 이라면 — 비교 시연 설명용
#   block_line / pass_line  Cosmos 프롬프트에 들어가는 영어 한 줄: '막힘으로 본다' / '통과시킨다' (둘 다 없으면 버튼이 없다)
#   words    Cosmos 의 답에서 이 장애물을 알아보는 단어 (한국어·영어)
#   parts    사진 박스 라벨: (장면 안 물체 이름의 앞부분, 라벨 이름, 박스 색, 작업자가 피하는가)
#   placeable  False 면 드롭다운에 나오지 않는다 (작업자처럼 배치가 아니라 채점용인 것)
OBSTACLES = {}
RULE = {}                                    # 장애물별 판단: key -> "block" (막힘) 또는 "pass" (통과). 정답과 프롬프트가 함께 따른다


def obstacle(key, ko, passable, nav2="", short=None, name=None, block_line="", pass_line="", words=(),
             parts=(), placeable=True):
    """장애물을 표에 등록하는 데코레이터. 데코레이터가 붙은 함수 build(stage, group, cx, rnd) 가 그 장애물을 만든다.
    (group = 통로 폴더 경로, cx = 통로 중심 x, rnd = random.Random)"""
    def deco(build):
        OBSTACLES[key] = dict(ko=ko, short=short or ko, name=name or short or ko, passable=passable, nav2=nav2,
                              block_line=block_line, pass_line=pass_line, words=tuple(words),
                              parts=tuple(parts), placeable=placeable, build=build)
        if block_line:
            RULE[key] = "pass" if passable else "block"
        return build
    return deco


def _register_plain(key, **kw):
    """아무것도 놓지 않는 항목(비움, 작업자)을 표에 등록한다."""
    obstacle(key, **kw)(None)


_truth = {a: "clear" for a in amr_scene.AISLES}


def _place(stage, path, asset, x, y, yaw=0.0, tilt=0.0, z=0.0, collide=True):
    # 위치는 바깥 Xform 에, 에셋은 그 아래 자식에 둔다
    # (사람 에셋처럼 자체 변환(double 정밀도)을 가진 에셋에 회전을 덮어쓰면 형식 오류가 난다)
    p = stage.DefinePrim(path, "Xform")
    stage.DefinePrim(path + "/asset", "Xform").GetReferences().AddReference(get_assets_root_path() + asset)
    api = UsdGeom.XformCommonAPI(p)
    api.SetTranslate(Gf.Vec3d(x, y, z))
    api.SetRotate(Gf.Vec3f(tilt, 0.0, yaw))
    if collide:
        _add_colliders(stage, p)
    return p


def _add_colliders(stage, root):
    """라이다 주행(물리)에서 로봇이 물체를 뚫고 지나가지 않도록 메시에 충돌 모양(볼록 껍질)을 붙인다.
    움직이지 않는 정적 충돌체다 (RigidBody 없음). 라이다는 충돌과 상관없이 화면 형상을 본다."""
    from pxr import Usd, UsdPhysics
    for q in Usd.PrimRange(root):
        if q.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(q)
            UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr("convexHull")


def set_aisle(aisle, state, seed=None):
    assert state in STATES, f"state must be one of {STATES}"
    stage = omni.usd.get_context().get_stage()
    group = f"{ROOT_PATH}/{aisle}"
    if stage.GetPrimAtPath(group):
        stage.RemovePrim(group)
    _truth[aisle] = state
    build = OBSTACLES[state]["build"]
    if build is None:
        return
    UsdGeom.Xform.Define(stage, group)
    rnd = random.Random(seed if seed is not None else hash((aisle, state)) & 0xFFFF)
    build(stage, group, amr_scene.AISLES[aisle], rnd)


# ---- 표를 읽는 도구 (다른 파일이 쓴다) ------------------------------------------------------------
def class_of(name):
    """장면 안 물체 이름 → 사진 박스 라벨 이름. 표에 없는 물체면 None."""
    for o in OBSTACLES.values():
        for prefix, label, _color, _avoid in o["parts"]:
            if name.startswith(prefix):
                return label
    return None


def box_colors():
    """{라벨 이름: (R, G, B)}"""
    return {label: color for o in OBSTACLES.values() for _p, label, color, _a in o["parts"]}


def avoid_prefixes():
    """걸어 다니는 작업자가 피하는 물체의 이름 앞부분들 (튜플)."""
    return tuple(prefix for o in OBSTACLES.values() for prefix, _l, _c, avoid in o["parts"] if avoid)


# ---- 장애물별 판단 (막힘 / 통과) -----------------------------------------------------------------
def rule_keys():
    """'장애물별 판단' 버튼이 있는 장애물 키 (표에 정의한 순서)."""
    return [k for k, o in OBSTACLES.items() if o["block_line"]]


def get_rule(key):
    return RULE[key]


def set_rule(key, mode):
    assert key in RULE and mode in ("block", "pass"), (key, mode)
    RULE[key] = mode


def reset_rules():
    for k in RULE:
        RULE[k] = "pass" if OBSTACLES[k]["passable"] else "block"


def passable(state):
    """이 상태의 통로를 '통과 가능' 으로 보는 것이 정답인가 (버튼으로 정한 값). 비움 등 버튼이 없는 상태는 표의 값."""
    if state in RULE:
        return RULE[state] == "pass"
    return bool(OBSTACLES[state]["passable"]) if state in OBSTACLES else False


def expected_action(state):
    return "pass" if passable(state) else "reroute"


def words(mode):
    """판단이 mode("block"/"pass") 인 장애물들의 알아보기 단어."""
    return tuple(w for k, o in OBSTACLES.items() if RULE.get(k) == mode for w in o["words"])


def _puddle(stage, path, x, y, radius=0.9):
    """바닥에 얇고 반짝이는 웅덩이 (높이 3 mm 라서 라이다·깊이로는 거의 안 보인다)."""
    from pxr import Sdf, UsdShade
    geom = UsdGeom.Cylinder.Define(stage, path)
    geom.CreateRadiusAttr(radius)
    geom.CreateHeightAttr(0.003)
    geom.CreateAxisAttr("Z")
    api = UsdGeom.XformCommonAPI(geom)
    api.SetTranslate(Gf.Vec3d(x, y, 0.002))
    api.SetScale(Gf.Vec3f(1.0, 0.7, 1.0))           # 약간 타원형
    mat = UsdShade.Material.Define(stage, path + "_mat")
    sh = UsdShade.Shader.Define(stage, path + "_mat/shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.05, 0.06, 0.07))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.02)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1, 1, 1))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.85)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)
    return geom


def clear_all():
    for a in amr_scene.AISLES:
        set_aisle(a, "clear")


def truth():
    return dict(_truth)


# ======================================== 장애물 정의 ========================================
# 새 장애물: 아래처럼 @obstacle(...) 함수를 하나 더 쓴다. (순서 = 드롭다운 순서)
_register_plain("clear", ko="비움", passable=True, nav2="통과")


@obstacle("boxes", ko="박스로 막힘", name="박스", passable=False, nav2="막힘 감지 후 후진·우회",
          block_line="- Boxes, cartons or pallets lying on the floor in the lane: the aisle is BLOCKED (reroute).",
          pass_line="- Boxes, cartons or pallets lying on the floor in the lane: the robot can drive around them, NOT a problem (pass).",
          words=("박스", "상자", "팔레트", "box", "carton", "pallet"),
          parts=[("box", "fallen_box", (255, 170, 0), True)])
def _build_boxes(stage, group, cx, rnd):
    # 통로 폭(약 3.1 m) 전체에 박스가 쓰러져 흩어진 모습
    xs = [cx - 1.15, cx - 0.4, cx + 0.35, cx + 1.1]
    # 옆으로 쓰러진 박스(tilt 90)는 회전축이 바닥 중심이라 0.25 m 올려야 바닥에 닿는다
    for i, x in enumerate(xs):
        tilt = rnd.choice([0, 90])
        _place(stage, f"{group}/box_{i}", BOX, x + rnd.uniform(-0.1, 0.1), OBSTACLE_Y + rnd.uniform(-0.3, 0.3),
               yaw=rnd.uniform(0, 90), tilt=tilt, z=0.25 if tilt else 0.0)
    for i, x in enumerate([cx - 0.8, cx + 0.7]):
        tilt = rnd.choice([0, 90])
        _place(stage, f"{group}/box_top_{i}", BOX, x, OBSTACLE_Y + 0.8, yaw=rnd.uniform(0, 90), tilt=tilt,
               z=0.25 if tilt else 0.0)


# 의미 기반 상황: 라이다만으로는 놓치는 경우
@obstacle("spill", ko="액체 흘림 + 통행불가 표시", short="액체 흘림", name="액체", passable=False,
          nav2="웅덩이를 못 보고 통과",
          block_line="- Liquid on the floor, a wet or shiny patch, or a wet-floor warning sign: slippery, the aisle is BLOCKED (reroute).",
          pass_line="- Liquid on the floor, a wet or shiny patch, or a wet-floor warning sign: the robot can drive through, NOT a problem (pass).",
          words=("wet floor", "wet-floor", "caution sign"),
          parts=[("sign", "wet_floor_sign", (255, 235, 0), True), ("puddle", "puddle", (0, 190, 255), False)])
def _build_spill(stage, group, cx, rnd):
    _puddle(stage, f"{group}/puddle", cx - 0.1, OBSTACLE_Y - 1.5, radius=0.95)
    # 표지판은 차선 오른쪽 끝(1.0 m)에: 라이다 주행이 표지판을 피하려고 옆으로 비키지 않고 웅덩이 위를 그대로 지나가게
    _place(stage, f"{group}/sign", WET, cx + 1.0, OBSTACLE_Y - 2.6, yaw=rnd.uniform(-20, 20))


# 통로 한가운데에 고깔 한 개. 통로 폭이 3.1 m 라서 라이다는 옆으로 비켜 지나갈 수 있다.
# passable 은 기본 정답이다 (막힘). 익스텐션의 '장애물별 판단' 버튼으로 바꾼다.
@obstacle("cone", ko="고깔 1개", short="고깔", passable=False, nav2="옆으로 비켜 통과",
          block_line="- An orange-and-white striped traffic cone anywhere in the lane, even at one side: the aisle is CLOSED (reroute).",
          pass_line="- Traffic cones are allowed in the lane and do NOT block the robot (pass).",
          words=("콘", "원뿔", "고깔", "교통", "cone"),
          parts=[("cone", "cone", (255, 120, 0), True)])
def _build_cone(stage, group, cx, rnd):
    _place(stage, f"{group}/cone_0", CONE, cx, OBSTACLE_Y)


# 작업자는 배치가 아니라 주행 중에 걸어 다닌다 (dynamic_events.py). 채점용: 입구 사진에 걸어 다니는 작업자가
# 통로 안에 보이면 정답을 '막힘' 으로 본다.
_register_plain("worker", ko="작업자 (돌발)", name="사람", passable=False, nav2="사람을 장애물로 보고 정지 후 우회",
                block_line="- A person standing or working in the lane: the aisle is BLOCKED (reroute).",
                pass_line="- A person in the lane: people move away, NOT a problem (pass).",
                words=("사람", "작업자", "person", "worker"), placeable=False)


# ---- 표에서 만든 값 (다른 파일은 이 이름들을 그대로 쓴다) -----------------------------------------
STATES = tuple(k for k, o in OBSTACLES.items() if o["placeable"])        # 드롭다운에 나오는 배치 옵션
NAV2_ONLY = {k: o["nav2"] for k, o in OBSTACLES.items()}
KO = {k: o["ko"] for k, o in OBSTACLES.items()}
SHORT = {k: o["short"] for k, o in OBSTACLES.items()}
