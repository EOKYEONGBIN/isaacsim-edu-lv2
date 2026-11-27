# AMR + Cosmos 데모 장면: 창고 + 통로 4개 + AMR(NVIDIA Nova Carter, ROS 2 그래프 포함) + 앞 카메라 + 통로 그래프
#
# Isaac Sim 6.2 Script Editor 또는 MCP 에서 실행:
#   import runpy; runpy.run_path(r"C:\Projects\Edu_Cosmos\DemoProjects\CosmosReasonerAMR\amr_scene.py")
# 실행하면 새 장면을 만든다 (열려 있던 장면은 닫힌다).
#
# 좌표 (m, z 위쪽)
#   선반 줄은 y = -4.5 ~ 13.1 로 길게 놓여 있고, 통로는 그 사이 (y 방향으로 지나감)
#   앞 복도 y = -6.5, 뒤 복도 y = 15.5
#   통로 A~D 중심 x = -7.05, -2.55, 1.95, 6.45 (폭 약 3.1 m)

import math
import os

import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux
from isaacsim.storage.native import get_assets_root_path

WAREHOUSE = "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
# Nova Carter (ROS 2 용): 바퀴·라이다·카메라·ROS 그래프가 들어 있어 나중에 Nav2 를 바로 붙일 수 있다.
# 지금은 라이다 주행(lidar_drive.py)으로 바퀴를 구동한다.
ROBOT = "/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd"
ROBOT_Z = 0.14                    # 에셋 원점이 바퀴 아래로 0.14 m 들어가 있어 그만큼 올린다

ROBOT_PATH = "/World/AMR"
# 판단용 카메라 = Nova Carter 의 앞 스테레오 카메라(왼쪽). 시야 약 90도, 높이 약 0.49 m.
# ROS 로 나가는 카메라와 같으므로 나중에 ROS 노드도 같은 시점의 이미지로 판단한다.
FRONT_CAM = ROBOT_PATH + "/chassis_link/sensors/front_hawk/left/camera_left"
TOP_CAM = "/World/TopCamera"
FREE_CAM = "/World/FreeCamera"       # '자유 시점' 버튼용 원근(perspective) 카메라. 마우스로 마음대로 돌려 보는 용도
FREE_VIEW_POS = (6.0, -10.0, 6.0)      # 자유 시점 처음 위치 (사용자가 정한 값)
FREE_VIEW_ROT = (62.0, 0.0, 55.0)       # 처음 회전 (X, Y, Z 도)
TOP_VIEW = (0.0, 3.5, 64.0)   # 창고 전체를 바로 위에서 내려다보는 위치 ('위에서 보기' 버튼이 누를 때마다 여기로 되돌림)

# 저장해 둔 장면 파일. 익스텐션이 켜지면 이 파일을 연다 (없으면 build() 로 만들고 저장)
SCENE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenes", "amr_aisle_demo.usda")

AISLES = {"A": -7.05, "B": -2.55, "C": 1.95, "D": 6.45}       # 통로 중심 x


def aisle_no(a):
    """통로 표시 이름: 내부 키 A~D 는 화면·기록에서 1~4 번 통로로 보여 준다 (왼쪽 → 오른쪽)."""
    return str(list(AISLES).index(a) + 1) if a in AISLES else str(a)

FRONT_Y, BACK_Y = -6.5, 15.5                                  # 앞·뒤 복도 (통로마다 앞 지점 [열,0] 과 뒤 지점 [열,1] 이 이 선 위에 있다)
ENTRY_Y = -5.6                                                # 통로 입구: 사진 찍는 자세·작업자 시야 판정의 기준 (이 높이에는 노드가 없다)
START = (-7.05, -9.0)
GOAL = (6.45, 16.6)                                           # 뒤 벽(y=18.0)에서 1.4 m (라이다 주행은 벽 0.95 m 앞에서 멈춘다)
# 고를 수 있는 목적지: 이름 → (화면 표시, (x, y), 연결할 뒤 복도 노드). 모두 뒤 복도 쪽이라 통로 하나는 꼭 지난다.
# 실제 AMR 은 관제 시스템이 이런 '이름 붙은 지점' 중 하나를 목적지로 내려 준다.
# 화면 표시는 번호 1~4 (왼쪽부터). 바닥에도 같은 번호를 회색 원에 표시한다 (add_goal_pads).
# 키 이름(GOAL 등)은 코드 호환용으로 그대로 둔다 — 4번이 예전 기본 목적지 GOAL.
GOALS = {
    "RECEIVE":  ("1", (AISLES["A"], GOAL[1]), "[0,1]"),
    "PACKING":  ("2", (AISLES["B"], GOAL[1]), "[1,1]"),
    "CHARGE":   ("3", (AISLES["C"], GOAL[1]), "[2,1]"),
    "GOAL":     ("4", GOAL, "[3,1]"),
}
DEFAULT_GOAL = "GOAL"

# 추가 선반 줄 (원래 선반 3줄 사이에 2줄을 더 넣어 통로 폭을 약 3 m 로 줄임)
EXTRA_SHELVES = {"Shelf_extra_0": -4.8 - (-0.305), "Shelf_extra_1": 4.2 - (-0.305)}   # Shelf_1 기준 x 이동량


def node_name(col, row):
    """지점 이름 [열,행]: 열 = 통로 번호 - 1 (왼쪽부터 0, 1, 2, 3), 행 = 0 앞 복도 / 1 뒤 복도. 예) [0,0] = 1번 통로 앞, [0,1] = 1번 통로 뒤."""
    return f"[{col},{row}]"


def graph():
    """노드 이름 → (x, y), 간선 목록 [(a, b, 통로이름 or None)]. 통로 간선만 막힐 수 있다.
    통로마다 노드가 앞([열,0]) 뒤([열,1]) 하나씩이다. 통로 간선은 앞 지점에서 뒤 지점까지 한 줄이고, 앞 지점이 Cosmos 가 판단하는 자리다."""
    nodes = {"START": START}
    nodes.update({k: xy for k, (_label, xy, _attach) in GOALS.items()})
    edges = []
    names = list(AISLES)
    for i, a in enumerate(names):
        x = AISLES[a]
        nodes[node_name(i, 0)] = (x, FRONT_Y)
        nodes[node_name(i, 1)] = (x, BACK_Y)
        edges += [(node_name(i, 0), node_name(i, 1), a)]
        if i + 1 < len(names):
            edges += [(node_name(i, 0), node_name(i + 1, 0), None), (node_name(i, 1), node_name(i + 1, 1), None)]
    edges += [("START", node_name(0, 0), None)]
    edges += [(attach, k, None) for k, (_label, _xy, attach) in GOALS.items()]
    return nodes, edges


def _yaw_quat(deg):
    h = math.radians(deg) / 2
    return Gf.Quatd(math.cos(h), 0, 0, math.sin(h))


def build():
    root = get_assets_root_path()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    wh = stage.DefinePrim("/World/Warehouse", "Xform")
    wh.GetReferences().AddReference(root + WAREHOUSE)
    for n in ("PalletBin_01", "PalletBin_02"):            # 뒤 복도를 막는 팔레트 상자는 끈다
        p = stage.GetPrimAtPath(f"/World/Warehouse/{n}")
        if p:
            p.SetActive(False)
    # 위 카메라에서 바닥이 보이도록 천장·지붕 보만 숨긴다 (조명은 그대로)
    for child in wh.GetChildren():
        n = child.GetName()
        if n.startswith(("SM_CeilingA", "SM_BeamA", "SM_PillarA_9M4")):
            UsdGeom.Imageable(child).MakeInvisible()
        elif n.startswith("SM_LampCeiling"):
            for k in child.GetChildren():
                if k.GetName().startswith("SM_"):
                    UsdGeom.Imageable(k).MakeInvisible()
    for name, dx in EXTRA_SHELVES.items():                # 같은 선반을 참조로 하나 더 놓는다
        p = stage.DefinePrim(f"/World/{name}", "Xform")
        p.GetReferences().AddReference(root + WAREHOUSE, Sdf.Path("/Root/Shelf_1"))
        # 참조한 선반에는 원래 자리(창고 Root 기준 이동값)가 들어 있으므로, 그 값에 x 이동량만 더한다
        # (덮어쓰면 창고 밖으로 날아간다 — 실제로 발생)
        t = p.GetAttribute("xformOp:translate")
        t.Set(t.Get() + Gf.Vec3d(dx, 0, 0))

    # 실내 밝기 보충
    dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
    dome.CreateIntensityAttr(300.0)

    # AMR (로봇 앞쪽 = +X, 앞 카메라는 에셋에 들어 있음)
    robot = stage.DefinePrim(ROBOT_PATH, "Xform")
    robot.GetReferences().AddReference(root + ROBOT)
    set_robot_pose(START[0], START[1], 90.0)
    add_goal_marker(stage)
    add_goal_pads(stage)
    add_robot_marker(stage)
    # 위에서 내려다보는 카메라 (화면 확인용)
    top = UsdGeom.Camera.Define(stage, TOP_CAM)
    top.CreateFocalLengthAttr(18.0)
    top.CreateClippingRangeAttr(Gf.Vec2f(0.1, 200.0))
    UsdGeom.XformCommonAPI(top).SetTranslate(Gf.Vec3d(*TOP_VIEW))
    print("[amr_scene] built. aisles:", list(AISLES), "robot:", ROBOT_PATH)
    return stage


GOAL_MARKER = "/World/GoalMarker"
GOAL_COLOR = (0.1, 1.0, 0.55)        # 네온 초록 (로봇 표시·안전 영역의 기본 색)
# 목적지 표시 색: 가야 할 곳은 빨강, 로봇이 도착하면 초록 (set_goal_state)
GOAL_STATE_COLORS = {"pending": (1.0, 0.08, 0.08), "arrived": GOAL_COLOR}


def _glow_material(stage, path, color, glow):
    from pxr import UsdShade
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(c * glow for c in color)))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def add_goal_marker(stage=None):
    """목적지(GOAL) 표시: 바닥의 빛나는 링 + 위에 떠 있는 아래 방향 화살표 + 초록 조명.
    보기용이라 충돌체가 없다 (라이다·로봇 주행에 영향 없음). Cosmos 사진에는 안 나오게 capture 가 찍는 동안 숨긴다."""
    from pxr import UsdShade
    stage = stage or omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GOAL_MARKER):
        stage.RemovePrim(GOAL_MARKER)
    root = UsdGeom.Xform.Define(stage, GOAL_MARKER)
    UsdGeom.XformCommonAPI(root).SetTranslate(Gf.Vec3d(GOAL[0], GOAL[1], 0.0))
    bright = _glow_material(stage, GOAL_MARKER + "/Looks/Neon", GOAL_STATE_COLORS["pending"], 2.5)   # 너무 세면 하얗게 날아간다

    def cyl(name, r, h, z, mat, axis="Z"):
        c = UsdGeom.Cylinder.Define(stage, f"{GOAL_MARKER}/{name}")
        c.CreateRadiusAttr(r)
        c.CreateHeightAttr(h)
        c.CreateAxisAttr(axis)
        UsdGeom.XformCommonAPI(c).SetTranslate(Gf.Vec3d(0, 0, z))
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
        return c

    # 바닥 링 두 겹 (원기둥은 각지게 그려져서 64각형 메시로 직접 만든다)
    def ring(name, r_in, r_out, z, n=64):
        pts, idx = [], []
        for i in range(n):
            a = 2 * math.pi * i / n
            pts += [Gf.Vec3f(r_in * math.cos(a), r_in * math.sin(a), z), Gf.Vec3f(r_out * math.cos(a), r_out * math.sin(a), z)]
        for i in range(n):
            j = (i + 1) % n
            idx += [2 * i, 2 * i + 1, 2 * j + 1, 2 * j]
        m = UsdGeom.Mesh.Define(stage, f"{GOAL_MARKER}/{name}")
        m.CreatePointsAttr(pts)
        m.CreateFaceVertexCountsAttr([4] * n)
        m.CreateFaceVertexIndicesAttr(idx)
        m.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(bright)
    ring("Ring", 0.72, 0.85, 0.01)
    # (안쪽 작은 링은 뺐다: 그 자리에 바닥 번호 원이 보이게 — add_goal_pads)
    # 떠 있는 화살표 (아래를 가리킴): 몸통 + 뒤집은 원뿔
    cyl("ArrowShaft", 0.07, 0.55, 2.15, bright)
    head = UsdGeom.Cone.Define(stage, f"{GOAL_MARKER}/ArrowHead")
    head.CreateRadiusAttr(0.22)
    head.CreateHeightAttr(0.4)
    head.CreateAxisAttr("Z")
    hx = UsdGeom.XformCommonAPI(head)
    hx.SetTranslate(Gf.Vec3d(0, 0, 1.68))
    hx.SetRotate(Gf.Vec3f(180, 0, 0))                          # 꼭짓점이 아래로
    UsdShade.MaterialBindingAPI.Apply(head.GetPrim()).Bind(bright)
    # 바닥을 은은하게 비추는 조명 (색은 set_goal_state 가 상태에 맞게 바꾼다)
    light = UsdLux.SphereLight.Define(stage, GOAL_MARKER + "/Glow")
    light.CreateRadiusAttr(0.25)
    light.CreateIntensityAttr(4000.0)
    light.CreateColorAttr(Gf.Vec3f(*GOAL_STATE_COLORS["pending"]))
    UsdGeom.XformCommonAPI(light).SetTranslate(Gf.Vec3d(0, 0, 0.8))
    return root


GOAL_PADS = "/World/GoalPads"
# 7-세그먼트 숫자 (a 위, b 오른쪽 위, c 오른쪽 아래, d 아래, e 왼쪽 아래, f 왼쪽 위, g 가운데)
_DIGIT_SEGS = {"1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc"}


def add_goal_pads(stage=None):
    """목적지 1~4 위치 바닥에 회색 원 + 흰 번호를 놓는다 (위에서 보기·따라가기에서 위치를 알아보기 쉽게).
    번호 윗부분이 +Y(뒤 벽 쪽)를 향해서, 위에서 볼 때와 로봇이 다가갈 때 바로 읽힌다.
    보기용이라 충돌체가 없다. Cosmos 사진을 찍는 동안에는 숨긴다."""
    from pxr import UsdShade
    stage = stage or omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GOAL_PADS):
        stage.RemovePrim(GOAL_PADS)
    UsdGeom.Xform.Define(stage, GOAL_PADS)
    gray = UsdShade.Material.Define(stage, GOAL_PADS + "/Looks/Gray")
    sh = UsdShade.Shader.Define(stage, GOAL_PADS + "/Looks/Gray/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.32, 0.33, 0.35))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    gray.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    white = _glow_material(stage, GOAL_PADS + "/Looks/White", (0.95, 0.95, 0.95), 0.6)
    H, W, t = 0.46, 0.26, 0.065                      # 숫자 높이·폭·획 두께 (m)
    seg = {"a": (-W / 2, W / 2, H / 2 - t, H / 2), "d": (-W / 2, W / 2, -H / 2, -H / 2 + t),
           "g": (-W / 2, W / 2, -t / 2, t / 2),
           "f": (-W / 2, -W / 2 + t, 0, H / 2), "b": (W / 2 - t, W / 2, 0, H / 2),
           "e": (-W / 2, -W / 2 + t, -H / 2, 0), "c": (W / 2 - t, W / 2, -H / 2, 0)}
    for key, (label, (x, y), _attach) in GOALS.items():
        pad = UsdGeom.Xform.Define(stage, f"{GOAL_PADS}/Pad{label}")
        UsdGeom.XformCommonAPI(pad).SetTranslate(Gf.Vec3d(x, y, 0.0))
        # 원판은 매끄럽게 64각형 메시로 (Cylinder 는 각지게 그려졌다)
        _ring_mesh(stage, f"{GOAL_PADS}/Pad{label}/Disc", 0.0, 0.5, 0.006, gray, n=64)
        pts, idx, n = [], [], 0
        segs = [seg[c] for c in _DIGIT_SEGS[label]]
        if label == "1":                              # 1 은 가운데 세로 획 하나
            segs = [(-t / 2, t / 2, -H / 2, H / 2)]
        for x0, x1, y0, y1 in segs:
            pts += [Gf.Vec3f(x0, y0, 0.011), Gf.Vec3f(x1, y0, 0.011), Gf.Vec3f(x1, y1, 0.011), Gf.Vec3f(x0, y1, 0.011)]
            idx += [4 * n, 4 * n + 1, 4 * n + 2, 4 * n + 3]
            n += 1
        num = UsdGeom.Mesh.Define(stage, f"{GOAL_PADS}/Pad{label}/Number")
        num.CreatePointsAttr(pts)
        num.CreateFaceVertexCountsAttr([4] * n)
        num.CreateFaceVertexIndicesAttr(idx)
        num.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(num.GetPrim()).Bind(white)
    return stage.GetPrimAtPath(GOAL_PADS)


def set_goal_state(state="pending"):
    """목적지 표시 색: "pending"(빨강, 아직 안 도착) / "arrived"(초록, 도착). 재질·조명 색만 바꾼다."""
    from pxr import UsdShade
    stage = omni.usd.get_context().get_stage()
    c = GOAL_STATE_COLORS[state]
    sh = UsdShade.Shader(stage.GetPrimAtPath(GOAL_MARKER + "/Looks/Neon/Shader"))
    if sh:
        sh.GetInput("diffuseColor").Set(Gf.Vec3f(*c))
        sh.GetInput("emissiveColor").Set(Gf.Vec3f(*(v * 2.5 for v in c)))
    light = UsdLux.SphereLight(stage.GetPrimAtPath(GOAL_MARKER + "/Glow"))
    if light:
        light.GetColorAttr().Set(Gf.Vec3f(*c))


def display_name(node):
    """지도·진행 기록에 보일 지점 이름. 목적지는 내부 이름(RECEIVE·PACKING·CHARGE·GOAL) 대신 '목적지1'~'목적지4' 로, 나머지(FA·EA…)는 그대로."""
    return f"목적지{GOALS[node][0]}" if node in GOALS else node


def move_goal_marker(goal="GOAL"):
    """네온 목적지 표시를 GOALS[goal] 위치로 옮기고 빨강(아직 도착 전)으로 되돌린다 (없으면 만든다)."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(GOAL_MARKER)
    if not prim:
        prim = add_goal_marker(stage).GetPrim()
    x, y = GOALS[goal][1]
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(x, y, 0.0))
    set_goal_state("pending")


ROBOT_MARKER = ROBOT_PATH + "/chassis_link/RobotMarker"   # 로봇 몸체에 붙여서 같이 움직인다


def _ring_mesh(stage, path, r_in, r_out, z, mat, n=72, a0=0.0, a1=2 * math.pi, plane="XY"):
    """빛나는 링(또는 호) 메시. plane="XY" 는 바닥과 나란히, "YZ" 는 로봇 앞을 향해 세운 면."""
    from pxr import UsdShade
    full = abs(a1 - a0 - 2 * math.pi) < 1e-6
    k = n if full else n + 1
    pts, idx = [], []

    def p(r, a):
        c, s_ = r * math.cos(a), r * math.sin(a)
        return Gf.Vec3f(c, s_, z) if plane == "XY" else Gf.Vec3f(z, c, s_)
    for i in range(k):
        a = a0 + (a1 - a0) * i / n
        pts += [p(r_in, a), p(r_out, a)]
    for i in range(n):
        j = (i + 1) % k
        idx += [2 * i, 2 * i + 1, 2 * j + 1, 2 * j]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([4] * n)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


# 라이다 안전 영역 표시 (실제 산업용 AMR 의 '보호 영역 / 경고 영역' 처럼 바닥에 은은하게)
# 크기는 lidar_drive 의 판단 기준과 같다: 띠 반폭 HALF_W, 멈춤 STOP_DIST, 감속 SLOW_DIST (라이다 위치 기준)
FIELD_HALF_W, FIELD_STOP, FIELD_SLOW = 0.47, 0.95, 2.2   # 폭은 lidar_drive.HALF_W 와 같게
FIELD_COLORS = {"clear": (0.1, 1.0, 0.55),    # 초록: 영역 안에 물체 없음
                "slow": (1.0, 0.75, 0.1),     # 노랑: 경고 영역에 물체 → 감속
                "stop": (1.0, 0.03, 0.03)}    # 빨강: 보호 영역에 물체 → 멈춤


# 각 부분의 빛 세기. 바탕색은 검정이라 조명을 받아 하얗게 뜨지 않고, 이 세기만큼만 빛난다.
# 면을 꽉 채우면 불투명한 판처럼 보여서(RTX 에서 UsdPreviewSurface 반투명이 먹지 않음)
# 영역 안은 가는 줄무늬로 채워 바닥이 비쳐 보이게 한다
FIELD_GLOW = {"Stop": 0.9, "Slow": 0.55, "Edge": 1.3}


def _field_material(stage, path, color, glow):
    from pxr import UsdShade
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(c * glow for c in color)))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _quad(stage, path, x0, x1, y0, y1, z, mat):
    from pxr import UsdShade
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(x0, y0, z), Gf.Vec3f(x1, y0, z), Gf.Vec3f(x1, y1, z), Gf.Vec3f(x0, y1, z)])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


def _stripes(stage, path, x0, x1, y0, y1, z, mat, pitch, width):
    """x0~x1 구간을 가로 줄무늬(폭 width, 간격 pitch)로 채운 메시 하나."""
    from pxr import UsdShade
    pts, idx, n = [], [], 0
    x = x0 + pitch / 2
    while x + width / 2 < x1:
        a, b = x - width / 2, x + width / 2
        pts += [Gf.Vec3f(a, y0, z), Gf.Vec3f(b, y0, z), Gf.Vec3f(b, y1, z), Gf.Vec3f(a, y1, z)]
        idx += [4 * n, 4 * n + 1, 4 * n + 2, 4 * n + 3]
        n += 1
        x += pitch
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([4] * n)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


def _lidar_x(stage):
    """chassis_link 기준 라이다의 앞뒤 위치 (거리 기준점: 앞 RPLidar). 못 찾으면 0.25 m."""
    from pxr import Usd
    ch = stage.GetPrimAtPath(ROBOT_PATH + "/chassis_link")
    for path in (ROBOT_PATH + "/chassis_link/sensors/front_RPLidar/RPLidar_S2E",
                 ROBOT_PATH + "/chassis_link/sensors/front_2d_lidar"):
        p = stage.GetPrimAtPath(path)
        if p and ch:
            t = Usd.TimeCode.Default()
            m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(t) * \
                UsdGeom.Xformable(ch).ComputeLocalToWorldTransform(t).GetInverse()
            return float(m.ExtractTranslation()[0])
    return 0.25


def add_robot_marker(stage=None):
    """로봇 앞 바닥에 라이다 안전 영역을 은은하게 표시한다.
      - 보호 영역(멈춤): 몸체 앞 ~ 라이다에서 0.95 m, 조금 진하게
      - 경고 영역(감속): 0.95 ~ 2.2 m, 더 옅게
    색은 start_marker_anim 이 라이다 상태에 따라 초록 / 노랑(감속) / 빨강(멈춤) 으로 바꾼다.
    보기용이라 충돌체가 없어 주행·라이다에 영향 없음. Cosmos 사진을 찍는 동안에는 숨긴다."""
    stage = stage or omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(ROBOT_MARKER):
        stage.RemovePrim(ROBOT_MARKER)
    root = UsdGeom.Xform.Define(stage, ROBOT_MARKER)
    c = FIELD_COLORS["clear"]
    m_stop = _field_material(stage, ROBOT_MARKER + "/Looks/Stop", c, FIELD_GLOW["Stop"])
    m_slow = _field_material(stage, ROBOT_MARKER + "/Looks/Slow", c, FIELD_GLOW["Slow"])
    m_edge = _field_material(stage, ROBOT_MARKER + "/Looks/Edge", c, FIELD_GLOW["Edge"])
    lx = _lidar_x(stage)
    # chassis 원점은 멈춰 있을 때 바닥 위 약 0.04 m, 주행(Play) 중에는 거의 바닥 높이로 내려앉는다
    # (-0.02 로 두었더니 주행 중 바닥 아래로 묻혀 안 보였다) → 바닥 위 1.5 cm
    z = 0.015
    x0, x1, x2, w = lx + 0.2, lx + FIELD_STOP, lx + FIELD_SLOW, FIELD_HALF_W
    _stripes(stage, ROBOT_MARKER + "/StopZone", x0, x1, -w, w, z, m_stop, pitch=0.07, width=0.018)   # 촘촘하게
    _stripes(stage, ROBOT_MARKER + "/SlowZone", x1, x2, -w, w, z, m_slow, pitch=0.14, width=0.014)   # 성기게
    # 테두리 (얇은 선): 양옆 + 경계선 + 끝선
    t = 0.025
    _quad(stage, ROBOT_MARKER + "/EdgeL", x0, x2, w - t, w, z + 0.001, m_edge)
    _quad(stage, ROBOT_MARKER + "/EdgeR", x0, x2, -w, -w + t, z + 0.001, m_edge)
    _quad(stage, ROBOT_MARKER + "/EdgeStop", x1 - t / 2, x1 + t / 2, -w, w, z + 0.001, m_edge)
    _quad(stage, ROBOT_MARKER + "/EdgeEnd", x2 - t, x2, -w, w, z + 0.001, m_edge)
    return root.GetPrim()


def _playing():
    import omni.timeline
    return omni.timeline.get_timeline_interface().is_playing()


def field_state():
    """지금 라이다 판단 상태: 'clear' / 'slow' / 'stop' (주행 중이 아니면 clear)."""
    import builtins
    nav = getattr(builtins, "_aisle_nav", None)
    d = getattr(nav, "ahead", None)                  # 라이다 주행이 멈춤·감속에 실제로 쓴 거리
    if d is None or not _playing():
        return "clear"
    return "stop" if d < FIELD_STOP else "slow" if d < FIELD_SLOW else "clear"


def start_marker_anim():
    """안전 영역 색을 라이다 상태에 맞게 바꾼다 (상태가 바뀔 때만 USD 를 고침). 한 번만 걸리게 builtins 에 보관."""
    import builtins
    from pxr import UsdShade
    stop_marker_anim()
    last = {"state": None}

    def tick(_e):
        stage = omni.usd.get_context().get_stage()
        if not stage or not stage.GetPrimAtPath(ROBOT_MARKER):
            return
        try:
            st = field_state()
        except Exception:
            st = "clear"
        # 안전 영역은 평소에는 숨기고, 라이다가 경고(감속)·보호(멈춤) 영역에 물체를 잡았을 때만 보인다.
        # (Cosmos 사진을 찍는 동안에는 set_goal_marker_visible 이 억제 표시를 걸어 계속 숨긴다)
        show = st != "clear" and not getattr(builtins, "_amr_marker_suppressed", False)
        img = UsdGeom.Imageable(stage.GetPrimAtPath(ROBOT_MARKER))
        if (img.GetVisibilityAttr().Get() != UsdGeom.Tokens.invisible) != show:      # 실제 상태와 다를 때만 고친다
            img.MakeVisible() if show else img.MakeInvisible()
        if st == last["state"]:
            return
        last["state"] = st
        c = FIELD_COLORS[st]
        for name, glow in FIELD_GLOW.items():
            sh = UsdShade.Shader(stage.GetPrimAtPath(f"{ROBOT_MARKER}/Looks/{name}/Shader"))
            if sh:
                sh.GetInput("emissiveColor").Set(Gf.Vec3f(*(v * glow for v in c)))
    builtins._amr_marker_anim = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        tick, name="amr_marker_anim")


def stop_marker_anim():
    import builtins
    sub = getattr(builtins, "_amr_marker_anim", None)
    if sub is not None:
        try:
            sub.unsubscribe()
        except Exception:
            pass
    builtins._amr_marker_anim = None


def set_goal_marker_visible(visible):
    """Cosmos 용 사진을 찍는 동안 목적지·로봇 표시를 숨겼다가 다시 보이게 한다."""
    import builtins
    stage = omni.usd.get_context().get_stage()
    for path in (GOAL_MARKER, GOAL_PADS):
        prim = stage.GetPrimAtPath(path)
        if prim:
            img = UsdGeom.Imageable(prim)
            img.MakeVisible() if visible else img.MakeInvisible()
    # 로봇 안전 영역: 숨길 때는 바로 숨기고(억제 표시), 다시 보일 때는 라이다 상태에 따라 start_marker_anim 의 tick 이 정한다
    builtins._amr_marker_suppressed = not visible
    prim = stage.GetPrimAtPath(ROBOT_MARKER)
    if prim and not visible:
        UsdGeom.Imageable(prim).MakeInvisible()


def set_robot_pose(x, y, yaw_deg):
    stage = omni.usd.get_context().get_stage()
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT_PATH))
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    t = ops.get("xformOp:translate") or xf.AddTranslateOp()
    o = ops.get("xformOp:orient") or xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    if not ops:
        xf.SetXformOpOrder([t, o])
    t.Set(Gf.Vec3d(x, y, ROBOT_Z))
    q = _yaw_quat(yaw_deg)
    o.Set(q if o.GetPrecision() == UsdGeom.XformOp.PrecisionDouble else Gf.Quatf(q))


def reset_top_view():
    """TopCamera 를 TOP_VIEW (바로 위, 아래를 봄) 로 되돌린다. 마우스로 움직여 둔 것도 원래대로."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(TOP_CAM)
    if not prim:
        top = UsdGeom.Camera.Define(stage, TOP_CAM)
        top.CreateFocalLengthAttr(18.0)
        top.CreateClippingRangeAttr(Gf.Vec2f(0.1, 200.0))
        prim = top.GetPrim()
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()                       # 마우스 조작으로 생긴 회전·이동을 모두 지우고 다시 놓는다
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*TOP_VIEW))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))
    # 지도처럼 보이는 직교 투영. 이때는 높이가 아니라 aperture(0.1 m 단위)가 보이는 폭을 정한다.
    # 가로 60 m → 16:9 화면에서 세로 약 34 m: START(로봇)부터 목적지까지 한 화면에 들어온다
    cam = UsdGeom.Camera(prim)
    cam.GetProjectionAttr().Set(UsdGeom.Tokens.orthographic)
    cam.GetHorizontalApertureAttr().Set(600.0)
    cam.GetVerticalApertureAttr().Set(337.5)
    return prim


def ensure_free_cam():
    """자유 시점 카메라(원근 투영)가 없으면 만든다. 이미 있으면 그대로 둔다 (마우스로 옮겨 둔 위치를 유지).
    처음 위치는 FREE_VIEW_POS / FREE_VIEW_ROT (사용자가 정한 값)."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(FREE_CAM)
    if prim and UsdGeom.Xformable(prim).GetOrderedXformOps():
        return prim                                            # 이미 위치가 정해져 있다 (마우스로 옮긴 값 유지)
    # 없거나, 뷰포트가 위치 없는 빈 카메라를 만들어 둔 경우: 처음 위치로 놓는다
    cam = UsdGeom.Camera.Define(stage, FREE_CAM)
    cam.CreateFocalLengthAttr(18.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(5.0, 500.0))          # 가까운 쪽(x)을 5 m 로: 천장 구조물 등이 가까이 있어도 안 잘리게 사용자가 정한 값
    cam.GetProjectionAttr().Set(UsdGeom.Tokens.perspective)
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*FREE_VIEW_POS))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*FREE_VIEW_ROT))
    return cam.GetPrim()


# PhysX GPU 충돌 검사 버퍼 크기. 기본값(1024)이면 창고 선반·로봇·장애물이 많아
# "increase PxGpuDynamicsMemoryConfig::foundLostAggregatePairsCapacity to 13312" 오류가 나고 충돌을 놓칠 수 있다
PHYSX_GPU_CAPACITY = {"physxScene:gpuFoundLostAggregatePairsCapacity": 65536,
                      "physxScene:gpuTotalAggregatePairsCapacity": 65536}


def tune_physics(stage=None):
    """장면의 모든 PhysicsScene 에 PHYSX_GPU_CAPACITY 를 적용한다 (이미 더 크면 그대로). 타임라인이 멈춘 상태에서 부를 것."""
    from pxr import UsdPhysics
    stage = stage or omni.usd.get_context().get_stage()
    changed = []
    for p in stage.Traverse():
        if not p.IsA(UsdPhysics.Scene):
            continue
        for name, val in PHYSX_GPU_CAPACITY.items():
            a = p.GetAttribute(name)
            if not a:
                a = p.CreateAttribute(name, Sdf.ValueTypeNames.UInt)
            cur = a.Get()
            if cur is None or cur < val:
                a.Set(val)
                changed.append(f"{p.GetPath()}.{name.split(':')[-1]}={val}")
    return changed


SIM_SPEEDS = (1.0, 3.0, 5.0)   # 익스텐션 '배속' 메뉴


def set_sim_speed(factor=1.0):
    """시뮬레이션 배속. 한 프레임에 흐르는 시뮬레이션 시간을 factor 배로 늘린다 (화면 갱신 수는 그대로).
    타임라인은 프레임마다 1/time_codes_per_second 초씩 진행하므로 60/factor 로 바꾼다.
    물리·라이다·주행 타이머(sim_time)가 모두 같이 빨라진다. 렌더 부하 때문에 실제로는 N 배보다 조금 덜 빠르다
    (측정: 2배 설정 → 약 1.7배, 3배 → 약 2.6배).
    Play 중에는 바꾸지 않고 기억만 해 둔다 → 다음에 멈춘 상태에서 apply_pending_speed() 가 적용한다.
    (Play 중에 이 값을 바꾸면 물리 연결이 다시 맺어지며 'Simulation view object is invalidated' 오류가 쏟아졌다)
    결과: "applied" 또는 "pending"."""
    import builtins
    import omni.timeline
    tl = omni.timeline.get_timeline_interface()
    factor = max(0.25, float(factor))
    if not tl.is_stopped():
        builtins._amr_pending_speed = factor
        return "pending"
    builtins._amr_pending_speed = None
    tl.set_time_codes_per_second(60.0 / factor)
    tl.set_target_framerate(60.0)                  # 화면 갱신 목표는 그대로 (이걸 같이 낮추면 배속 효과가 없어진다)
    return "applied"


def apply_pending_speed():
    """주행 중에 고른 배속이 있으면 지금 적용한다 (타임라인이 멈춘 상태에서 부를 것). 적용한 값 또는 None."""
    import builtins
    v = getattr(builtins, "_amr_pending_speed", None)
    if v and set_sim_speed(v) == "applied":
        return v
    return None


def get_sim_speed():
    import omni.timeline
    return round(60.0 / omni.timeline.get_timeline_interface().get_time_codes_per_seconds(), 2)


def reset_robot_links(stage=None):
    """로봇 부품(바퀴·몸체 등 물리 링크)에 남은 위치·회전·속도 값을 지워 에셋 원래 값으로 되돌린다.
    주행(Play) 중 물리가 이 값들을 USD 에 써 넣는데, 그대로 저장되면 다음 Play 때 몸체가 비틀어진 채
    바퀴가 돌고 있는 상태로 시작해 처음에 빙글 돌거나 방향을 못 잡았다. 타임라인이 멈춘 상태에서 부를 것."""
    from pxr import UsdPhysics
    stage = stage or omni.usd.get_context().get_stage()
    names = ("xformOp:translate", "xformOp:orient", "xformOp:rotateXYZ", "xformOp:scale", "xformOpOrder",
             "physics:velocity", "physics:angularVelocity")
    cleared = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH)):
        if p.GetPath() == Sdf.Path(ROBOT_PATH) or not p.HasAPI(UsdPhysics.RigidBodyAPI):
            continue                                  # 로봇 전체 위치(/World/AMR)와 센서·표시 프림은 그대로
        for layer in (stage.GetRootLayer(), stage.GetSessionLayer()):
            spec = layer.GetPrimAtPath(p.GetPath())
            if not spec:
                continue
            for n in names:
                if n in spec.properties:
                    spec.RemoveProperty(spec.properties[n])
                    cleared += 1
    return cleared


def save_scene(path=SCENE_FILE):
    """지금 장면(장애물 없이)을 파일로 저장한다. 장면 구성을 바꾼 뒤 한 번 부르면 된다:
        import amr_scene; amr_scene.build(); amr_scene.save_scene()"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    try:
        import incidents
        incidents.clear_all()
    except Exception:
        pass
    try:
        import dynamic_events
        dynamic_events.remove_all()              # 걸어 다니는 작업자는 장면 파일에 넣지 않는다
    except Exception:
        pass
    reset_robot_links()                          # 주행 중 남은 부품 위치·바퀴 속도는 저장하지 않는다
    set_goal_state("pending")                    # 파일에는 항상 빨강(도착 전)으로 저장
    speed = get_sim_speed()
    set_sim_speed(1.0)                           # 배속 값(timeCodesPerSecond)은 파일에 남기지 않는다
    ok = omni.usd.get_context().save_as_stage(path)
    set_sim_speed(speed)
    print("[amr_scene] saved:", path, ok)
    return ok


async def open_scene_async(path=SCENE_FILE):
    """저장한 장면 파일을 연다. 이미 열려 있으면 그대로 두고, 파일이 없으면 build() 로 만들어 저장한다."""
    import omni.kit.app
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    # Isaac Sim 이 막 켜지는 중이면 기본 빈 장면이 뒤늦게 열리면서 우리가 만든 장면을 덮어쓴다
    # (빈 장면이 파일로 저장됨) → 시작 처리가 끝날 때까지 잠깐 기다린다
    for _ in range(60):
        await app.next_update_async()
    url = (ctx.get_stage_url() or "").replace("\\", "/").lower()
    if url.endswith(path.replace("\\", "/").lower()) and ctx.get_stage().GetPrimAtPath(ROBOT_PATH):
        return "already"
    if os.path.exists(path) and not _scene_file_ok(path):
        os.remove(path)                              # 로봇이 없는 잘못된 파일은 지우고 새로 만든다
    if not os.path.exists(path):
        build()
        for _ in range(30):
            await app.next_update_async()
        if not ctx.get_stage().GetPrimAtPath(ROBOT_PATH):
            raise RuntimeError("장면을 만들지 못했습니다 (로봇 없음)")
        save_scene(path)
        return "built"
    result, err = await ctx.open_stage_async(path)
    if not result:
        raise RuntimeError(f"장면 파일을 열지 못함: {path} ({err})")
    tune_physics()
    reset_robot_links()
    return "opened"


def _scene_file_ok(path):
    """파일 안에 로봇이 들어 있는지 글자로 확인 (Usd.Stage.Open 은 메모리에 열린 레이어를 재사용해 결과가 틀린 적이 있다)."""
    try:
        with open(path, encoding="utf-8", errors="ignore") as f:
            return f'"{ROBOT_PATH.rsplit("/", 1)[-1]}"' in f.read()
    except OSError:
        return False


if __name__ == "__main__":
    build()
