# 경로 띠: AMR 이 갈 경로를 바닥 위 반투명 파란 띠로 보여 준다 (자동차 내비게이션 화면처럼).
#   로봇 앞쪽 LOOK m 만 그리고, 멀수록 흐려진다. 모퉁이는 둥글게 돌려서 부드러운 곡선으로 그린다.
#   뷰포트 위에만 그려지므로 장면 파일과 Cosmos 에 보내는 사진에는 들어가지 않는다 (bubble.py 와 같은 방식).
#
#   import path_ribbon
#   path_ribbon.set_path([(x0, y0), (x1, y1), ...])   # 계획된 경로 (지점 좌표, m). 새로 계획할 때마다 부르면 갈아 끼운다
#   path_ribbon.clear()                                # 띠를 지운다 (도착, 정지)
#   path_ribbon.stop()                                 # 구독과 SceneView 를 모두 없앤다
#
# 로봇 위치는 lidar_drive.pose() 를 매 프레임 따라가고, 지나간 부분은 지워진다.

import builtins
import math

import omni.kit.app

LOOK = 9.0              # 로봇 앞 몇 m 까지 그릴지
SKIP = 0.7              # 로봇 중심에서 이만큼 앞에서부터 그린다 (시작은 투명하므로 빈 공간처럼 보이지 않는다)
FADE_IN = 1.6           # 띠 시작에서 이 거리(m)까지 완전 투명 → 진하게 (로봇 몸체 위에 겹쳐도 비치지 않게)
WIDTH = 0.8             # 띠 너비 m
Z = 0.03                # 바닥에서 띄우는 높이 m (바닥과 겹쳐 깜박이지 않게)
CORNER_R = 1.6          # 모퉁이를 둥글게 도는 반지름 m
STEP = 0.25             # 띠를 나누는 간격 m
ALPHA = 0.60            # 로봇 가까운 곳의 불투명도
COLOR = (0.10, 0.50, 1.00)
FRAME_ID = "amr_path_ribbon"


def _st():
    s = getattr(builtins, "_amr_path_ribbon", None)
    if s is None:
        s = {"sub": None, "view": None, "root": None, "win": None,
             "pts": None, "cum": None, "idx": 0, "built": None}
        builtins._amr_path_ribbon = s
    return s


def _smooth(P, r, n=8):
    """꺾이는 지점을 2차 베지에 곡선으로 둥글게 돌린다."""
    pts = [P[0]]
    for i in range(1, len(P) - 1):
        a, b, c = P[i - 1], P[i], P[i + 1]
        la, lc = math.dist(a, b), math.dist(b, c)
        if la < 1e-6 or lc < 1e-6:
            continue
        rr = min(r, la / 2, lc / 2)
        p1 = (b[0] + (a[0] - b[0]) / la * rr, b[1] + (a[1] - b[1]) / la * rr)
        p2 = (b[0] + (c[0] - b[0]) / lc * rr, b[1] + (c[1] - b[1]) / lc * rr)
        for k in range(n + 1):
            t = k / n
            u = 1 - t
            pts.append((u * u * p1[0] + 2 * u * t * b[0] + t * t * p2[0],
                        u * u * p1[1] + 2 * u * t * b[1] + t * t * p2[1]))
    pts.append(P[-1])
    return pts


def _resample(pts, step):
    out, cum = [pts[0]], [0.0]
    carry = 0.0
    for a, b in zip(pts, pts[1:]):
        seg = math.dist(a, b)
        if seg < 1e-9:
            continue
        d = step - carry
        while d <= seg:
            t = d / seg
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            cum.append(cum[-1] + step)
            d += step
        carry = seg - (d - step)
    if math.dist(out[-1], pts[-1]) > 1e-3:
        cum.append(cum[-1] + math.dist(out[-1], pts[-1]))
        out.append(pts[-1])
    return out, cum


def _ensure_view():
    from omni.ui import scene as sc
    from omni.kit.viewport.utility import get_active_viewport_window
    s = _st()
    win = get_active_viewport_window()
    if win is None:
        return False
    if s["view"] is not None and s["win"] is win:
        return True
    _drop_view()
    with win.get_frame(FRAME_ID):
        view = sc.SceneView()
        with view.scene:
            root = sc.Transform()
    win.viewport_api.add_scene_view(view)
    s.update(view=view, root=root, win=win, built=None)
    return True


def ensure_view():
    """SceneView 만 먼저 만들어 둔다 (bubble.py 가 말풍선 앞에 띠 화면이 깔리도록 부른다)."""
    return _ensure_view()


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
    s.update(view=None, root=None, win=None, built=None)


def _build(vis):
    """vis = [(x, y, 시작부터의 거리), ...] 로 띠 메쉬를 다시 만든다."""
    from omni.ui import scene as sc
    s = _st()
    s["root"].clear()
    if len(vis) < 2:
        return
    r, g, b = COLOR
    edge = []                                   # 샘플마다 (왼쪽 점, 오른쪽 점, 불투명도)
    for i, (x, y, d) in enumerate(vis):
        j0, j1 = max(0, i - 1), min(len(vis) - 1, i + 1)
        tx, ty = vis[j1][0] - vis[j0][0], vis[j1][1] - vis[j0][1]
        n = math.hypot(tx, ty) or 1.0
        nx, ny = -ty / n, tx / n
        t = min(1.0, max(0.0, d / FADE_IN))
        fade = t * t * t * (t * (6 * t - 15) + 10)       # smootherstep: 시작 근처는 거의 0 으로 평평하다가 서서히 올라간다
        a = ALPHA * max(0.0, 1.0 - d / LOOK) ** 0.8 * fade   # 로봇 앞머리는 완전 투명, 앞으로 갈수록 진해진다 (몸체를 가리지 않게)
        a = min(1.0, max(0.0, a))
        edge.append(([x + nx * WIDTH / 2, y + ny * WIDTH / 2, Z], [x - nx * WIDTH / 2, y - ny * WIDTH / 2, Z], a))
    # 칸마다 꼭짓점 4개를 따로 둔다 (색 4개 = 인덱스 4개). 꼭짓점을 칸끼리 나눠 쓰면 색 배열이 인덱스보다 짧아져
    # 렌더러가 끝쪽 칸의 색을 배열 밖에서 읽어 빨강·초록·파랑으로 깜박였다.
    positions, colors, counts, indices = [], [], [], []
    for i in range(len(edge) - 1):
        (l0, r0, a0), (l1, r1, a1) = edge[i], edge[i + 1]
        positions += [l0, r0, r1, l1]
        colors += [[r, g, b, a0], [r, g, b, a0], [r, g, b, a1], [r, g, b, a1]]
        counts.append(4)
        indices += [4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3]
    with s["root"]:
        sc.PolygonMesh(positions, colors, counts, indices, wireframe=False)


def _tick(_e=None):
    s = _st()
    if s["view"] is None or not s["pts"]:
        return
    try:
        import lidar_drive
        x, y, yaw = lidar_drive.pose()
    except Exception:
        return
    pts, cum = s["pts"], s["cum"]
    hx, hy = math.cos(yaw), math.sin(yaw)
    lo, hi = max(0, s["idx"] - 4), min(len(pts), s["idx"] + 80)
    idx = min(range(lo, hi), key=lambda i: (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2)
    if math.dist(pts[idx], (x, y)) > 3.0:                    # 경로에서 멀어졌다 → 전체에서 가장 가까운 곳
        idx = min(range(len(pts)), key=lambda i: (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2)
    s["idx"] = idx
    start = cum[idx] + SKIP
    vis = []
    for (px, py), c in zip(pts, cum):
        if c < start:
            continue
        if c > start + LOOK:
            break
        # 로봇 앞쪽에 있는 부분만 그린다. 후진하거나 돌아서 가는 길(경로가 로봇 뒤쪽)을 그리면 띠가 따라가는
        # 카메라 바로 밑·뒤를 지나, 화면 전체가 파랗게 덮였다. 앞쪽이 끊기는 곳에서 멈춘다 (끊긴 둘을 잇지 않는다).
        if (px - x) * hx + (py - y) * hy < 0.3:
            if vis:
                break
            return _show([], s)                  # 시작부터 뒤쪽이면 아직 그릴 것이 없다
        vis.append((px, py, c - start))
    _show(vis, s, idx)


def _show(vis, s, idx=-1):
    key = (idx, len(vis))
    if s["built"] != key:
        _build(vis)
        s["built"] = key


def set_path(points):
    """계획된 경로 [(x, y), ...] 를 받아 띠로 보여 준다. 새로 계획할 때마다 부르면 된다."""
    P = [(float(p[0]), float(p[1])) for p in points]
    if len(P) < 2:
        clear()
        return
    if not _ensure_view():
        return
    s = _st()
    s["pts"], s["cum"] = _resample(_smooth(P, CORNER_R), STEP)
    s["idx"], s["built"] = 0, None
    if s["sub"] is None:
        s["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            _tick, name="amr_path_ribbon")
    _tick()


def clear():
    """띠를 지운다 (구독은 유지)."""
    s = _st()
    s["pts"] = s["cum"] = None
    s["idx"], s["built"] = 0, None
    if s["root"] is not None:
        s["root"].clear()


def stop():
    """구독과 SceneView 를 모두 없앤다."""
    s = _st()
    if s["sub"] is not None:
        try:
            s["sub"].unsubscribe()
        except Exception:
            pass
        s["sub"] = None
    s["pts"] = s["cum"] = None
    _drop_view()
