# Cosmos 말풍선: 뷰포트 안에서 Nova Carter 머리 위를 따라다니며 Cosmos 의 판단과 이유를 보여 준다.
#   통과 가능 = 초록, 통과 불가 = 빨강, 판단 중 = 회색. (뷰포트 위에만 그려지므로 Cosmos 에 보내는 사진에는 안 찍힌다)
#
#   import bubble
#   bubble.thinking("통로2")                                  # 사진을 보내고 답을 기다리는 중
#   bubble.show(True, "통로2 통과 가능", "바닥이 비어 있어 지나갈 수 있습니다")   # 초록
#   bubble.show(False, "통로2 통과 불가", "박스가 통로를 막고 있습니다")          # 빨강
#   bubble.hide() / bubble.stop()                              # 숨기기 / 완전히 끄기
#
# 위치는 lidar_drive.pose() (follow_cam.py 와 같은 로봇 위치)를 매 프레임 따라간다.

import builtins
import math
import time

import omni.kit.app

K = 1.3                 # 글자 크기(size) 대비 화면에 그려지는 실제 크기 비율 (도형 단위와 글자 단위가 달라 맞춘다)
HEIGHT = 0.40           # 로봇 위치(바닥) 위 꼬리 끝(로봇 윗면)까지 m. 꼬리 끝이 로봇 몸체에 닿는 높이
LIFT_PX = 44            # 꼬리 끝(로봇 쪽)에서 말풍선 아래쪽까지의 선 길이 (화면 픽셀). 선을 늘려 말풍선과 로봇을 이어 준다
RADIUS = 34             # 모서리 둥글기 (화면 단위. 말풍선 높이의 절반을 넘으면 알약 모양이 된다)
FONT_TITLE = 20
FONT_BODY = 15
PAD = 12
MAX_CHARS = 26          # 한 줄 글자 수 (한글 기준)
MAX_LINES = 4           # 이유 줄 수 (넘으면 … 로 줄임)
FRAME_ID = "amr_cosmos_bubble"

def _c(r, g, b, a=255):
    return (a << 24) | (b << 16) | (g << 8) | r      # omni.ui.scene 색은 ABGR


# 네온 색 (목적지 표시와 같은 계열: amr_scene.GOAL_COLOR 네온 초록 / 도착 전 빨강). 바탕은 어둡고 테두리와 제목이 빛난다.
NEON = {
    "pass": (26, 255, 140),      # Pass     = 네온 초록 (목적지 표시의 도착 색)
    "block": (255, 40, 40),      # Reroute  = 네온 빨강 (목적지 표시의 도착 전 색)
    "think": (70, 220, 255),     # 판단 중 = 네온 하늘색
}
FILL = (10, 14, 22, 150)         # 바탕색 (빨강, 초록, 파랑, 불투명도 0~255). 불투명도를 낮출수록 뒤가 비친다


def _rgba(c):
    """(r, g, b, a) 0~255 → 메쉬 꼭짓점 색 [r, g, b, a] 0~1."""
    return [c[0] / 255.0, c[1] / 255.0, c[2] / 255.0, c[3] / 255.0]


def _outline(bw, bh, r, n=10):
    """가운데가 (0,0)인 둥근 사각형의 바깥선 점들 (반시계). 모서리마다 n 구간으로 나눈 호."""
    r = max(1.0, min(r, bw / 2, bh / 2))
    pts = []
    for cx, cy, a0 in ((bw / 2 - r, bh / 2 - r, 0.0), (-(bw / 2 - r), bh / 2 - r, math.pi / 2),
                       (-(bw / 2 - r), -(bh / 2 - r), math.pi), (bw / 2 - r, -(bh / 2 - r), 1.5 * math.pi)):
        for k in range(n + 1):
            a = a0 + (math.pi / 2) * k / n
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _fill_mesh(sc, pts, rgba):
    """바깥선 전체를 면 하나(다각형 하나)로 채운다. 삼각형 부채로 여러 면을 이어 붙였더니 면과 면 사이의 가는 선이
    반투명 바탕에서 나비 모양으로 비쳤다. 면이 하나면 안쪽에 이음매가 없다. 색 개수 = 인덱스 개수."""
    sc.PolygonMesh([[p[0], p[1], 0.0] for p in pts], [rgba] * len(pts), [len(pts)], list(range(len(pts))), wireframe=False)


def _ring_mesh(sc, outer, inner, col_outer, col_inner):
    """바깥선과 안쪽선 사이를 채운 고리. 바깥 꼭짓점과 안쪽 꼭짓점에 서로 다른 색을 주어 그라데이션을 만들 수 있다."""
    positions, colors = [], []
    n = len(outer)
    for i in range(n):
        j = (i + 1) % n
        o0, o1, i1, i0 = outer[i], outer[j], inner[j], inner[i]
        positions += [[o0[0], o0[1], 0.0], [o1[0], o1[1], 0.0], [i1[0], i1[1], 0.0], [i0[0], i0[1], 0.0]]
        colors += [col_outer, col_outer, col_inner, col_inner]
    sc.PolygonMesh(positions, colors, [4] * n, list(range(4 * n)), wireframe=False)


def _st():
    s = getattr(builtins, "_amr_bubble", None)
    if s is None:
        s = {"sub": None, "view": None, "root": None, "body": None, "win": None, "content": None, "built": None}
        builtins._amr_bubble = s
    return s


def _wrap(text, n):
    """한글은 폭이 넓어 글자 수로만 줄을 나눈다 (단어 경계 우선)."""
    lines, cur = [], ""
    for word in str(text).split():
        while len(word) > n:                      # 너무 긴 단어는 쪼갠다
            if cur:
                lines.append(cur)
                cur = ""
            lines.append(word[:n])
            word = word[n:]
        if len(cur) + len(word) + (1 if cur else 0) <= n:
            cur = (cur + " " + word).strip()
        else:
            lines.append(cur)
            cur = word
    if cur:
        lines.append(cur)
    return lines


def _text_w(s, size):
    return K * sum(size if ord(c) > 0x2E80 else size * 0.72 for c in s)


def _build(kind, title, body):
    """말풍선 도형을 다시 만든다 (내용이 바뀔 때만)."""
    from omni.ui import scene as sc
    s = _st()
    s["body"].clear()
    paras = [p for p in str(body or "").split("\n") if p.strip()]
    if len(paras) > 1:
        # 문단이 여럿이면 마지막 문단(예: 안전 안내)은 그대로 두고, 앞 문단들을 남은 줄 수에 맞춰 줄인다
        tail = _wrap(paras[-1], MAX_CHARS)[:MAX_LINES - 1]
        lines = _wrap(" ".join(paras[:-1]), MAX_CHARS)
        room = max(1, MAX_LINES - len(tail))
        if len(lines) > room:
            lines = lines[:room]
            lines[-1] = lines[-1][: MAX_CHARS - 1] + "…"
        lines += tail
    else:
        lines = _wrap(body, MAX_CHARS) if body else []
        if len(lines) > MAX_LINES:
            lines = lines[:MAX_LINES]
            lines[-1] = lines[-1][: MAX_CHARS - 1] + "…"
    w = max([_text_w(title, FONT_TITLE)] + [_text_w(l, FONT_BODY) for l in lines]) + 2 * PAD
    lh = K * (FONT_BODY + 4)                        # 본문 한 줄 높이
    th = K * (FONT_TITLE + 4)                       # 제목 줄 높이
    h = PAD * 2 + th + (6 if lines else 0) + len(lines) * lh
    nr, ng, nb = NEON[kind]
    neon = _c(nr, ng, nb)
    with s["body"]:
        # (0,0) = 꼬리 끝(로봇 머리 위 점). 말풍선은 그 위 LIFT_PX 에 앉는다
        cy = LIFT_PX + h / 2
        def at(x, y, z=0.0):
            return sc.Transform(transform=sc.Matrix44.get_translation_matrix(x, y, z))
        sc.Line([0, 0, 0], [0, LIFT_PX, 0], color=neon, thickness=3)
        with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 0, 0.3)):      # 로봇에 닿는 끝에 작은 점
            sc.Arc(4.5, axis=2, begin=0.0, end=2 * math.pi, sector=True, color=neon, wireframe=False)
        # 면은 서로 겹치지 않는 메쉬 세 개로 그린다 (조각을 겹쳐 그리면 반투명이 겹친 곳이 까만 줄로 보였다):
        #   ① 안쪽 바탕(부채꼴 하나) ② 네온 테두리(고리) ③ 바깥 빛 번짐(고리, 안쪽은 진하고 바깥은 투명한 그라데이션)
        # 테두리가 바탕 밑에 깔리지 않으므로 바탕을 반투명하게 해도 안쪽이 네온색으로 물들지 않는다.
        BORDER, GLOW = 4.0, 16.0
        with at(0, cy, 0.0):
            inner = _outline(w, h, RADIUS)
            border_o = _outline(w + BORDER, h + BORDER, RADIUS + BORDER / 2)
            glow_o = _outline(w + BORDER + 2 * GLOW, h + BORDER + 2 * GLOW, RADIUS + BORDER / 2 + GLOW)
            _fill_mesh(sc, inner, _rgba(FILL))
            _ring_mesh(sc, border_o, inner, _rgba((nr, ng, nb, 255)), _rgba((nr, ng, nb, 255)))
            _ring_mesh(sc, glow_o, border_o, _rgba((nr, ng, nb, 0)), _rgba((nr, ng, nb, 120)))
        y = LIFT_PX + h - PAD - th / 2
        with at(0, y, 1.0):
            sc.Label(title, alignment=_align("center"), color=neon, size=FONT_TITLE)
        y -= th / 2 + 6
        for l in lines:
            y -= lh / 2
            with at(0, y, 1.0):
                sc.Label(l, alignment=_align("center"), color=_c(236, 242, 248), size=FONT_BODY)
            y -= lh / 2
    s["built"] = (kind, title, body)


def _align(name):
    import omni.ui as ui
    return getattr(ui.Alignment, name.upper())


def _ensure_view():
    """뷰포트 창에 SceneView 를 붙인다 (한 번만). 이미 있으면 재사용."""
    from omni.ui import scene as sc
    from omni.kit.viewport.utility import get_active_viewport_window
    s = _st()
    win = get_active_viewport_window()
    if win is None:
        return False
    if s["view"] is not None and s["win"] is win:
        return True
    _drop_view()
    try:                                       # 경로 띠(path_ribbon)가 말풍선보다 뒤에 깔리도록 먼저 만들어 둔다
        import path_ribbon
        path_ribbon.ensure_view()
    except Exception:
        pass
    with win.get_frame(FRAME_ID):
        view = sc.SceneView()
        with view.scene:
            root = sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 0, 0), visible=False)
            with root:
                body = sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.SCREEN)
    win.viewport_api.add_scene_view(view)
    s.update(view=view, root=root, body=body, win=win, built=None)
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
    s.update(view=None, root=None, body=None, win=None, built=None)


def _tick(_e=None):
    s = _st()
    s["tick_t"] = time.time()                  # 심장박동: 구독이 실제로 불리는지 확인용
    if s["view"] is None or s["content"] is None:
        return
    from omni.ui import scene as sc
    try:
        import lidar_drive
        x, y, _yaw = lidar_drive.pose()
        kind, title, body = s["content"]
        if s["built"] != s["content"]:
            _build(kind, title, body)
        s["root"].transform = sc.Matrix44.get_translation_matrix(x, y, HEIGHT)
        s["root"].visible = True
        s["err"] = None
    except Exception as e:                     # 조용히 삼키면 말풍선이 멈춘 이유를 알 수 없다 → 마지막 오류를 남긴다
        s["err"] = repr(e)


def _subscribe():
    s = _st()
    if s["sub"] is not None:
        try:
            s["sub"].unsubscribe()
        except Exception:
            pass
    s["tick_t"] = time.time()
    s["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        _tick, name="amr_cosmos_bubble")


def _set(kind, title, body):
    if not _ensure_view():
        return
    s = _st()
    s["content"] = (kind, title, body)
    # 구독이 없거나, 있는데 1초 넘게 한 번도 불리지 않았으면(멈춘 구독) 새로 건다
    if s["sub"] is None or time.time() - s.get("tick_t", 0.0) > 1.0:
        _subscribe()
    _tick()


def thinking(aisle_name):
    _set("think", f"{aisle_name} 판단 중…", "")


def show(passable, title, reason=""):
    _set("pass" if passable else "block", title, reason)


def hide():
    """말풍선을 숨긴다 (구독은 유지, 화면 밖으로)."""
    s = _st()
    s["content"] = None
    if s["root"] is not None:
        s["root"].visible = False
        s["body"].clear()
        s["built"] = None


def stop():
    """구독과 SceneView 를 모두 없앤다."""
    s = _st()
    if s["sub"] is not None:
        try:
            s["sub"].unsubscribe()
        except Exception:
            pass
        s["sub"] = None
    s["content"] = None
    _drop_view()
