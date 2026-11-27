# 지나온 경로와 격자(경로 지점) 이름을 '위에서 보기' 에 보여 주고, 지도 그림으로 저장한다.
#
#   import route_map
#   route_map.start_trail()          # 주행 시작: 로봇이 지나가는 자리를 기록 (이전 기록은 지운다)
#   route_map.ensure_view()          # 뷰포트 위에 지점 이름·경로를 그리는 화면을 만든다 (위에서 보기 카메라일 때만 보인다)
#   route_map.show_top_view()        # 도착했을 때: 위에서 보기로 바꿔 지나온 경로를 보여 준다
#   await route_map.save_map(폴더)    # 위에서 본 사진 + 경로·지점 이름을 그린 지도 한 장 (route_map.png)
#
# 지점 이름 (amr_scene.graph 의 노드): [열,행] — 열 = 통로 번호 - 1, 행 0 = 통로 앞 복도(Cosmos 판단 지점), 행 1 = 통로 뒤 복도, START 시작점,
#   목적지1~4 (내부 이름은 RECEIVE / PACKING / CHARGE / GOAL). 통로 A~D 는 화면에서 1~4번 통로다 ([0,0] = 1번 통로 앞 복도 지점).
#   '격자 좌표' 가 아니라 로봇이 다니는 길 위의 '이름 붙은 지점' 이다 (4열 x 4행 모양으로 늘어서 있어 격자처럼 보인다).
# 화면에 그리는 것은 뷰포트 위에만 나온다 (장면 파일·Cosmos 사진에는 들어가지 않는다 — bubble.py 와 같은 방식).

import builtins
import math
import os
import time

import omni.kit.app

FRAME_ID = "amr_route_map"
MIN_STEP = 0.2                    # 이만큼(m) 움직일 때마다 경로에 한 점을 찍는다
Z = 0.08                          # 바닥 위로 띄우는 높이 (m)
MAP_RES = (1600, 900)             # 저장하는 지도 크기 (위에서 보기 카메라는 가로 60 m: 1 m = 약 26.7 픽셀)
TRAIL_RGBA = (0.0, 0.9, 1.0, 1.0)
NODE_COLORS = {"start": (0.2, 1.0, 0.45, 1.0), "goal": (1.0, 0.35, 0.8, 1.0), "node": (0.95, 0.95, 0.95, 1.0)}


def _st():
    s = getattr(builtins, "_amr_route_map", None)
    if s is None:
        s = {"sampler": None, "tick": None, "view": None, "root": None, "win": None, "labels": None, "trail": None,
             "pts": [], "t0": None, "built_n": -1, "res_prepared": None}
        builtins._amr_route_map = s
    return s


def trail():
    """지금까지 지나온 자리 [(x, y), ...]."""
    return list(_st()["pts"])


def length_m():
    p = _st()["pts"]
    return sum(math.dist(a, b) for a, b in zip(p, p[1:]))


# ---------------------------------------------------------------- 경로 기록
def _sample(_e=None):
    import omni.timeline
    s = _st()
    if not omni.timeline.get_timeline_interface().is_playing():
        return
    try:
        import lidar_drive
        x, y, _yaw = lidar_drive.pose()
    except Exception:
        return
    if s["t0"] is None:
        s["t0"] = omni.timeline.get_timeline_interface().get_current_time()
    pts = s["pts"]
    if not pts or math.dist(pts[-1], (x, y)) >= MIN_STEP:
        pts.append((x, y))


def start_trail():
    """주행 시작: 이전 경로를 지우고 기록을 시작한다."""
    s = _st()
    s["pts"] = []
    s["t0"] = None
    s["built_n"] = -1
    if s["sampler"] is not None:
        try:
            s["sampler"].unsubscribe()
        except Exception:
            pass
    s["sampler"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        _sample, name="amr_route_map_sampler")


def stop_trail():
    """기록을 멈춘다 (경로는 남겨 둔다: 도착 뒤에도 위에서 보기에서 볼 수 있다)."""
    s = _st()
    if s["sampler"] is not None:
        try:
            s["sampler"].unsubscribe()
        except Exception:
            pass
        s["sampler"] = None


# ---------------------------------------------------------------- 뷰포트 표시
def _top_active():
    """위에서 보기 카메라이고, 이 장면(AMR 로봇과 위에서 보기 카메라가 있는 스테이지)이 열려 있을 때만 True.
    장면을 나가거나 새 스테이지를 열면 카메라 경로 글자는 남아 있어도 False 가 되어 그림이 지워진다."""
    from omni.kit.viewport.utility import get_active_viewport
    import omni.usd
    import amr_scene
    try:
        stage = omni.usd.get_context().get_stage()
        if stage is None or not stage.GetPrimAtPath(amr_scene.TOP_CAM).IsValid() \
                or not stage.GetPrimAtPath(amr_scene.ROBOT_PATH).IsValid():
            return False
        return str(get_active_viewport().camera_path) == amr_scene.TOP_CAM
    except Exception:
        return False


def _label_dy(name, goals):
    """지점 이름을 점 오른쪽의 어느 높이(m)에 쓸지. 뒤 복도 지점과 목적지는 1.1 m 밖에 안 떨어져 있어서
    (위에서 보기에서 약 18 픽셀) 같은 높이에 쓰면 글씨가 겹친다 → 복도 지점([열,행])은 점 아래쪽, 목적지는 위쪽에 쓴다."""
    if name in goals:                       # 목적지 이름 4개는 같은 높이에 쓴다 (통로 간격 4.5 m 라서 이름끼리 겹치지 않는다)
        return 0.35
    if name == "START":
        return 0.0
    return -0.35


def _build_labels():
    from omni.ui import scene as sc
    import omni.ui as ui
    import amr_scene
    s = _st()
    s["labels"].clear()
    nodes, _edges = amr_scene.graph()
    goals = set(amr_scene.GOALS)
    with s["labels"]:
        for name, (x, y) in nodes.items():
            kind = "start" if name == "START" else "goal" if name in goals else "node"
            col = NODE_COLORS[kind]
            with sc.Transform(transform=sc.Matrix44.get_translation_matrix(x, y, Z)):
                sc.Points([[0, 0, 0]], sizes=[9], colors=[col])
            with sc.Transform(transform=sc.Matrix44.get_translation_matrix(x + 0.45, y + _label_dy(name, goals), Z)):
                sc.Label(amr_scene.display_name(name), alignment=ui.Alignment.LEFT_CENTER, color=_abgr(col), size=13)


def _abgr(c):
    r, g, b, a = (int(round(v * 255)) for v in c)
    return (a << 24) | (b << 16) | (g << 8) | r


def _build_trail():
    from omni.ui import scene as sc
    s = _st()
    s["trail"].clear()
    pts = s["pts"]
    s["built_n"] = len(pts)
    if len(pts) < 2:
        return
    with s["trail"]:
        n = len(pts)
        sc.Curve([[x, y, Z] for x, y in pts], thicknesses=[4.0] * n, colors=[TRAIL_RGBA] * n,
                 curve_type=sc.Curve.CurveType.LINEAR)
        with sc.Transform(transform=sc.Matrix44.get_translation_matrix(pts[0][0], pts[0][1], Z + 0.01)):
            sc.Points([[0, 0, 0]], sizes=[14], colors=[NODE_COLORS["start"]])
        with sc.Transform(transform=sc.Matrix44.get_translation_matrix(pts[-1][0], pts[-1][1], Z + 0.01)):
            sc.Points([[0, 0, 0]], sizes=[14], colors=[(1.0, 0.2, 0.2, 1.0)])


def _tick(_e=None):
    s = _st()
    if s["root"] is None:
        return
    try:
        top = _top_active()
        s["root"].visible = top
        if top:
            if not s.get("labels_on"):                      # 위에서 보기가 되면 지점 이름과 점을 그린다
                _build_labels()
                s["labels_on"] = True
            if s["built_n"] != len(s["pts"]):
                _build_trail()
        elif s.get("labels_on") or s["built_n"] != -1:
            # 위에서 보기가 아니면 숨기는 데 그치지 않고 그림을 아예 비운다 (숨김 설정이 먹지 않아도 경로·지점 표시가 안 보이게)
            s["labels"].clear()
            s["trail"].clear()
            s["labels_on"] = False
            s["built_n"] = -1
    except Exception as e:                      # 화면 표시 문제로 주행이 멈추면 안 된다
        s["err"] = repr(e)


def ensure_view():
    """뷰포트 위에 지점 이름·경로를 그리는 SceneView 를 만든다 (한 번만). 위에서 보기 카메라일 때만 보인다."""
    from omni.ui import scene as sc
    from omni.kit.viewport.utility import get_active_viewport_window
    s = _st()
    win = get_active_viewport_window()
    if win is None:
        return False
    if s["view"] is not None and s["win"] is win:
        return True
    _drop_view()
    win.get_frame(FRAME_ID).clear()                              # 이전 상태가 사라져 못 떼어낸 고아 위젯이 있어도 지운다
    with win.get_frame(FRAME_ID):
        view = sc.SceneView()
        with view.scene:
            root = sc.Transform(visible=False)
            with root:
                labels = sc.Transform()
                trail_g = sc.Transform()
    win.viewport_api.add_scene_view(view)
    s.update(view=view, root=root, win=win, labels=labels, trail=trail_g, built_n=-1, labels_on=False)
    if s["tick"] is None:
        s["tick"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            _tick, name="amr_route_map_tick")
    return True


def _drop_view():
    s = _st()
    if s["view"] is not None:
        try:
            s["win"].viewport_api.remove_scene_view(s["view"])
        except Exception:
            pass
        try:
            s["view"].destroy()
        except Exception:
            pass
        try:
            s["win"].get_frame(FRAME_ID).clear()               # 프레임 안에 위젯이 남아 있으면 계속 그려진다: 비운다
        except Exception:
            pass
    s.update(view=None, root=None, win=None, labels=None, trail=None, built_n=-1, labels_on=False)


def stop():
    """구독과 화면을 모두 없앤다 (확장을 끌 때)."""
    s = _st()
    for key in ("sampler", "tick"):
        if s[key] is not None:
            try:
                s[key].unsubscribe()
            except Exception:
                pass
            s[key] = None
    _drop_view()


def show_top_view():
    """위에서 보기로 바꿔 지점 이름과 지나온 경로를 보여 준다 (도착했을 때)."""
    import amr_scene
    from omni.kit.viewport.utility import get_active_viewport
    try:
        import follow_cam
        follow_cam.stop()                        # 따라가기 카메라가 시점을 다시 가져가지 않게
    except Exception:
        pass
    amr_scene.reset_top_view()
    get_active_viewport().camera_path = amr_scene.TOP_CAM
    ensure_view()


# ---------------------------------------------------------------- 지도 그림 저장
def prepare_map():
    """위에서 본 사진용 렌더 프로덕트를 Play 전에 미리 만든다 (Play 중에 만들면 값이 비어 나온다). 주행 시작 때 부를 것."""
    import amr_scene
    import capture
    amr_scene.reset_top_view()
    capture.prepare(amr_scene.TOP_CAM, MAP_RES)
    _st()["res_prepared"] = MAP_RES


def release_map():
    """저장이 끝나면 위에서 본 사진용 렌더 프로덕트를 없앤다 (남겨 두면 주행이 끝난 뒤 매 프레임 렌더링된다)."""
    import amr_scene
    import capture
    if _st()["res_prepared"]:
        capture.drop(amr_scene.TOP_CAM, _st()["res_prepared"])
        _st()["res_prepared"] = None


def _font(size):
    from PIL import ImageFont
    for path in (r"C:\Windows\Fonts\malgun.ttf", r"C:\Windows\Fonts\arial.ttf"):
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    return ImageFont.load_default()


async def save_map(out_dir, name="route_map.png", note=None):
    """위에서 본 사진에 지나온 경로와 지점 이름을 그려 out_dir/name 으로 저장한다. 저장한 경로를 돌려준다 (사진을 못 찍으면 None)."""
    import capture
    import amr_scene
    from PIL import Image, ImageDraw
    res = _st()["res_prepared"] or MAP_RES
    amr_scene.reset_top_view()
    for _ in range(3):
        await omni.kit.app.get_app().next_update_async()
    rgb = await capture._grab(amr_scene.TOP_CAM, res, 6)
    if rgb is None:
        return None
    im = Image.fromarray(rgb).convert("RGB")
    W, H = im.size
    cx, cy = amr_scene.TOP_VIEW[0], amr_scene.TOP_VIEW[1]        # 위에서 보기 카메라의 중심 (직교 투영)
    wm = 60.0                                                    # 가로로 보이는 길이 (m): reset_top_view 의 aperture 600 (0.1 m 단위)
    hm = wm * H / W

    def px(x, y):
        return (W / 2 + (x - cx) / wm * W, H / 2 - (y - cy) / hm * H)

    dr = ImageDraw.Draw(im, "RGBA")
    nodes, _edges = amr_scene.graph()
    goals = set(amr_scene.GOALS)
    f_name, f_small = _font(20), _font(14)
    pts = _st()["pts"]
    if len(pts) >= 2:
        line = [px(x, y) for x, y in pts]
        dr.line(line, fill=(0, 0, 0, 200), width=9, joint="curve")          # 어두운 바탕선 위에 밝은 선: 어느 바닥에서도 보인다
        dr.line(line, fill=(0, 230, 255, 255), width=5, joint="curve")
        x0, y0 = line[0]
        x1, y1 = line[-1]
        dr.ellipse([x0 - 10, y0 - 10, x0 + 10, y0 + 10], fill=(60, 255, 110, 255), outline=(0, 0, 0, 255), width=2)
        dr.ellipse([x1 - 10, y1 - 10, x1 + 10, y1 + 10], fill=(255, 50, 50, 255), outline=(0, 0, 0, 255), width=2)
    for nm, (x, y) in nodes.items():
        u, v = px(x, y)
        col = (60, 255, 110) if nm == "START" else (255, 90, 200) if nm in goals else (255, 255, 255)
        dr.ellipse([u - 5, v - 5, u + 5, v + 5], fill=col + (255,), outline=(0, 0, 0, 255), width=1)
        lu, lv = px(x + 0.45, y + _label_dy(nm, goals))                     # 점 오른쪽: 이름 + (x, y) 좌표 (m) 한 줄
        dn = amr_scene.display_name(nm)
        w_name = dr.textlength(dn, font=f_name)
        dr.text((lu, lv), dn, font=f_name, fill=col + (255,), anchor="lm", stroke_width=3, stroke_fill=(0, 0, 0, 255))
        if nm not in goals:                                                  # 목적지는 이름만 (좌표까지 쓰면 옆 목적지와 겹친다)
            dr.text((lu + w_name + 5, lv + 2), f"({x:.1f}, {y:.1f})", font=f_small, fill=(215, 215, 215, 255), anchor="lm",
                    stroke_width=2, stroke_fill=(0, 0, 0, 255))
    sim_s = None
    if _st()["t0"] is not None:
        from omni.timeline import get_timeline_interface         # (함수 안에서 'import omni.timeline' 을 쓰면 위의 omni.kit.app 이 지역 변수로 바뀌어 오류가 난다)
        sim_s = get_timeline_interface().get_current_time() - _st()["t0"]
    text = note or f"지나온 경로 {length_m():.1f} m" + (f" · {sim_s:.0f} 초(시뮬레이션)" if sim_s else "") + "   (초록 = 출발, 빨강 = 끝)"
    dr.rectangle([10, 10, 10 + dr.textlength(text, font=f_name) + 18, 48], fill=(0, 0, 0, 170))
    dr.text((18, 14), text, font=f_name, fill=(255, 255, 255, 255))
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name)
    im.save(path)
    return path
