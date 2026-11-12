"""Panel: record demonstrations, see how many are collected."""

import asyncio
import random

import omni.ui as ui

WINDOW_TITLE = "AZ Edu Lv2 - Lesson 2"


def _rgb(hex_str, alpha=0xFF):
    r, g, b = int(hex_str[0:2], 16), int(hex_str[2:4], 16), int(hex_str[4:6], 16)
    return (alpha << 24) | (b << 16) | (g << 8) | r


COLOR_TEXT = _rgb("EDEDEA")
COLOR_DIM = _rgb("9A9A94")
COLOR_BORDER = _rgb("38382F")
COLOR_CARD = _rgb("2A2A25")
COLOR_REC = _rgb("E5342A")
COLOR_REC_HOVER = _rgb("C42C23")
COLOR_SECONDARY = _rgb("343430")
COLOR_SECONDARY_HOVER = _rgb("40403A")
COLOR_DISABLED = _rgb("2B2B27")
COLOR_OK = _rgb("5BBF76")
COLOR_WARN = _rgb("E08A2E")
COLOR_GO = _rgb("76B343")  # same green as Lesson 1's 시작 (Play) button
COLOR_GO_HOVER = _rgb("8BC65C")
COLOR_GO_TEXT = _rgb("14150F")
COLOR_STOP = _rgb("C8473F")
COLOR_STOP_HOVER = _rgb("B03C35")
COLOR_RANDOM = _rgb("5DADE2")  # '랜덤' toggle and its note
COLOR_WIDE = _rgb("C39BD3")  # '확장' toggle and its note
COLOR_FREE = _rgb("F0B27A")  # '위치 자유' toggle and its note (trial feature)
COLOR_PAUSE = _rgb("E0A82E")
COLOR_PAUSE_HOVER = _rgb("EBBA52")
COLOR_KEY = _rgb("343430")


def _button_style(bg=COLOR_SECONDARY, hover=COLOR_SECONDARY_HOVER, text=COLOR_TEXT):
    return {
        "Button": {"background_color": bg, "border_width": 0, "border_radius": 5},
        "Button:hovered": {"background_color": hover},
        "Button:disabled": {"background_color": COLOR_DISABLED},
        "Button.Label": {"color": text},
        "Button.Label:disabled": {"color": COLOR_DIM},
    }


# Tables: (header, width; 0 = takes the remaining width). Five rows show, the rest scroll.
DEMO_COLUMNS = (("#", 24), ("녹화 시각", 0), ("세로", 44), ("가로", 44), ("시간", 40), ("이미지", 44), ("결과", 36))
AI_RUN_COLUMNS = (("#", 20), ("세로", 36), ("가로", 36), ("파렛트", 40), ("결과", 50), ("시간", 36), ("경로", 0), ("눈", 54))
DEMO_ROWS_VISIBLE = 5
DEMO_ROW_H = 22
SCROLLBAR_W = 8

# Key display (arrow keys + the two lift keys): key size and the lift keys' width
KEY_SIZE = (36, 26)
LIFT_KEY_W = 80


# Forklift start offset from the scene's own start pose [m], as shown on the panel (sliders; numbers can be
# typed too). The forklift faces the scene's -X (toward the pallet and rack), so 세로 is the scene's X with
# the sign flipped (+ = forward) and 가로 is the scene's Y.
START_OFFSET_RANGE_M = {"x": (-0.2, 1.5), "y": (-2.0, 2.0)}
WIDE_Y_RANGE_M = (-5.0, 5.0)  # '확장' toggle: the wider 가로 range (beyond where the demonstrations started)
START_YAW_RANGE_DEG = (-180.0, 180.0)  # 회전 from the scene's start heading; + turns left (counter-clockwise from above)
# The pallet can be moved straight back (away from the forklift, toward the rack) by up to this much -- never
# toward the forklift, it could end up inside it. The AI drives on up to 1 m past the person's route to get the
# forks all the way in, and the rack is 1 m behind the pallet.
PALLET_BACK_RANGE_M = (0.0, 0.5)
# How the AI judges the distance to the pallet while the forks go in: with YOLO, or not at all (it lifts where
# the person's route ends -- right only while the pallet is where it was during the recordings)
EYES_MODES = (("yolo", "YOLO 사용"), ("none", "YOLO 없음 (녹화 경로 끝에서 들어 올림)"))
PANEL_SIGN = {"x": -1.0, "y": 1.0}  # panel value * sign = offset along the scene axis


def _row_style(margin_height):
    """A button row with `margin_height` (on the row and, inherited, its buttons) instead of the panel's 4:
    the gaps above / below it shrink by 2 * (4 - margin_height)."""
    return {"margin_height": margin_height}


def _row_height(margin_height):
    """The row height that keeps its buttons 22 px tall (as with the panel's usual margins)."""
    return 22 + 4 * margin_height


class ForkliftAIWindow(ui.Window):
    def __init__(self, callbacks, **kwargs):
        kwargs.setdefault("width", 380)
        kwargs.setdefault("height", 640)
        super().__init__(WINDOW_TITLE, **kwargs)
        self._cb = callbacks
        self._offset = {"x": ui.SimpleFloatModel(0.0), "y": ui.SimpleFloatModel(0.0)}
        self._yaw = ui.SimpleFloatModel(0.0)  # 회전 [deg]
        for m in (*self._offset.values(), self._yaw):
            m.add_value_changed_fn(lambda _m: self._cb.get("start_changed", lambda: None)())
        self._random = ui.SimpleBoolModel(False)  # '랜덤': each recording / AI drive starts somewhere random
        self._wide = ui.SimpleBoolModel(False)  # '확장': 가로 range WIDE_Y_RANGE_M
        self._wide.add_value_changed_fn(lambda _m: self._on_wide_changed())
        self._free = ui.SimpleBoolModel(False)  # '위치 자유' (trial): AI drives start wherever the forklift stands
        # '랜덤' and '위치 자유' exclude each other: turning one on turns the other off
        self._random.add_value_changed_fn(lambda m: self._free.set_value(False) if m.get_value_as_bool() else None)
        self._free.add_value_changed_fn(lambda m: self._random.set_value(False) if m.get_value_as_bool() else None)
        self._sliders = {}
        self._pallet_back = ui.SimpleFloatModel(0.0)  # how far back the pallet is moved [m]
        self._pallet_back.add_value_changed_fn(lambda _m: self._cb.get("start_changed", lambda: None)())
        self._eyes_index = 0  # EYES_MODES
        self._reposition = ui.SimpleBoolModel(False)  # '자리 고쳐 잡기': no route -> a short move, then plan again
        self._labels = {}
        self._buttons = {}
        self._keys = {}  # driving key name -> (rectangle, label) of the key display
        self._key_state = {}
        self._demo_rows = None
        self._demos = ([], None)
        self._run_rows = None
        self._runs = ([], [])
        self._clear_armed = False
        self._ai_ready = False
        self._driving = False
        self._pause_state = "idle"  # start button: idle ('AI 운전 시작') / driving ('AI 일시 정지') / paused
        self.frame.set_build_fn(self._build)

    def _note(self, text):
        ui.Label(text, height=0, word_wrap=True, style={"color": COLOR_DIM, "font_size": 12})

    def _build(self):
        self._pause_state = "idle"  # the start button is (re)built as 'AI 운전 시작'
        with self.frame:
            # the panel is taller than its dock slot: scroll it
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(spacing=4, height=0, style={"margin": 4}):
                    ui.Label("시연 녹화", height=0, style={"font_size": 17, "color": COLOR_TEXT})
                    self._note(
                        "녹화를 시작하고, 키보드로 지게차를 운전해 파렛트 구멍에 포크를 넣고 → Lift Up → "
                        "2층 선반에 내려놓고 → 포크를 빼세요. 끝나면 '녹화 끝내기'."
                    )
                    with ui.ZStack(height=0):
                        ui.Rectangle(style={"background_color": COLOR_CARD, "border_radius": 8})
                        # half the usual gaps inside this card (margin 3 on every element + spacing 2), the same 12 px edge
                        with ui.VStack(height=0, style={"margin": 0}):
                            ui.Spacer(height=6)
                            with ui.HStack(height=0, style={"margin": 0}):
                                ui.Spacer(width=6)
                                with ui.VStack(height=0, spacing=2, style={"margin": 3}):
                                    ui.Label("지게차 출발 위치 (녹화 · AI 운전 공통)", height=0, style={"font_size": 13, "color": COLOR_TEXT})
                                    with ui.HStack(height=22, spacing=6):
                                        ui.CheckBox(model=self._random, width=20)
                                        ui.Label("랜덤", width=0, style={"color": COLOR_RANDOM, "font_size": 12})
                                        ui.Spacer(width=18)
                                        ui.CheckBox(model=self._wide, width=20)
                                        ui.Label("확장 (가로 -5 ~ 5 m)", width=0, style={"color": COLOR_WIDE, "font_size": 12})
                                        ui.Spacer(width=18)
                                        ui.CheckBox(model=self._free, width=20)
                                        ui.Label("위치 자유", width=0, style={"color": COLOR_FREE, "font_size": 12})
                                        ui.Spacer()
                                    self._sliders = {}
                                    for key, name in (("y", "가로 (Y)"), ("x", "세로 (X)")):
                                        lo, hi = self._range(key)
                                        with ui.HStack(height=22, spacing=6):
                                            ui.Label(name, width=60, style={"color": COLOR_DIM, "font_size": 12})
                                            self._sliders[key] = ui.FloatSlider(model=self._offset[key], min=lo, max=hi, step=0.05,
                                                                                precision=2)
                                            ui.FloatField(model=self._offset[key], width=60, precision=2)
                                            ui.Label("m", width=12, style={"color": COLOR_DIM, "font_size": 12})
                                    lo, hi = START_YAW_RANGE_DEG
                                    with ui.HStack(height=22, spacing=6):
                                        ui.Label("회전", width=60, style={"color": COLOR_DIM, "font_size": 12})
                                        ui.FloatSlider(model=self._yaw, min=lo, max=hi, step=1.0, precision=0)
                                        ui.FloatField(model=self._yaw, width=60, precision=0)
                                        ui.Label("°", width=12, style={"color": COLOR_DIM, "font_size": 12})
                                    lo, hi = PALLET_BACK_RANGE_M
                                    with ui.HStack(height=22, spacing=6):
                                        ui.Label("파렛트 뒤로", width=60, style={"color": COLOR_DIM, "font_size": 12})
                                        ui.FloatSlider(model=self._pallet_back, min=lo, max=hi, step=0.05, precision=2)
                                        ui.FloatField(model=self._pallet_back, width=60, precision=2)
                                        ui.Label("m", width=12, style={"color": COLOR_DIM, "font_size": 12})
                                    self._note("장면의 기본 출발 위치·방향에서 얼마나 옮기고 돌릴지입니다 (0 = 기본 위치, 회전 + = 왼쪽으로). "
                                               "정지 상태에서 바꾸면 바로 옮겨집니다. "
                                               "'파렛트 뒤로'는 파렛트를 지게차에서 먼 쪽으로 옮깁니다 (AI 운전용). 녹화할 때는 항상 0으로 돌아갑니다 "
                                               "(사람 시연은 같은 파렛트 위치에서 모아야 AI가 배울 수 있습니다).")
                                    for name, color, text in (
                                        ("랜덤", COLOR_RANDOM, "녹화 · AI 운전을 시작할 때마다 범위 안에서 출발 위치·방향을 무작위로 정하고 위 슬라이더에 "
                                                               "표시합니다. AI 운전은 파렛트 위치도 무작위입니다 (녹화는 파렛트 0)."),
                                        ("확장", COLOR_WIDE, "가로 범위를 -5 ~ 5 m로 넓힙니다. 시연을 녹화한 범위(-2 ~ 2 m) 밖이라 AI가 못 갈 수도 "
                                                             "있습니다."),
                                        ("위치 자유", COLOR_FREE, "AI 운전만: 슬라이더 값으로 옮기지 않고, 뷰포트에서 직접 옮기거나 돌려 둔 지금 "
                                                                 "위치·방향에서 출발합니다 (녹화는 그대로 슬라이더 위치)."),
                                    ):
                                        with ui.HStack(height=0, spacing=4):
                                            ui.Label(f"{name} :", width=62, height=0, alignment=ui.Alignment.LEFT_TOP,
                                                     style={"color": color, "font_size": 12})
                                            ui.Label(text, height=0, word_wrap=True, style={"color": COLOR_DIM, "font_size": 12})
                                ui.Spacer(width=6)
                            ui.Spacer(height=6)
                    with ui.ZStack(height=0):
                        ui.Rectangle(style={"background_color": COLOR_CARD, "border_radius": 8})
                        with ui.VStack(height=0, spacing=4, style={"margin": 6}):
                            self._labels["state"] = ui.Label("대기", height=0, alignment=ui.Alignment.CENTER,
                                                             style={"font_size": 20, "color": COLOR_DIM})
                            self._labels["live"] = ui.Label("", height=0, alignment=ui.Alignment.CENTER,
                                                            style={"font_size": 12, "color": COLOR_DIM})
                            with ui.HStack(height=32, spacing=6):
                                self._buttons["start"] = ui.Button("● 녹화 시작", clicked_fn=self._cb["start"],
                                                                   style=_button_style(COLOR_REC, COLOR_REC_HOVER))
                                self._buttons["stop"] = ui.Button("■ 녹화 끝내기", clicked_fn=self._cb["stop"],
                                                                  style=_button_style(), enabled=False)
                    ui.Spacer(height=4)
                    self._note("녹화할 때는 출발 위치를 바꿔 가며 녹화하세요. 실패한 시연은 '마지막 녹화 지우기'로 지우세요.")
                    ui.Label("모인 시연 데이터", height=0, style={"font_size": 14, "color": COLOR_TEXT})
                    ui.Line(height=1, style={"color": COLOR_BORDER})
                    self._labels["summary"] = ui.Label("-", height=0, word_wrap=True, style={"font_size": 13})
                    self._build_demo_table()
                    self._labels["last"] = ui.Label("", height=0, word_wrap=True, visible=False,  # shown with a message
                                                    style={"font_size": 12, "color": COLOR_DIM})
                    # from here to 'AI 운전': half the panel's usual gaps (button rows with less margin, see _row_style)
                    ui.Spacer(height=1, style={"margin": 0})  # a Spacer with the panel's margin is at least 8 px
                    with ui.HStack(height=_row_height(3), spacing=6, style=_row_style(3)):
                        self._buttons["discard"] = ui.Button("마지막 녹화 지우기", clicked_fn=self._cb["discard"], style=_button_style())
                        ui.Button("데이터 폴더 열기", clicked_fn=self._cb["open_folder"], style=_button_style())
                    with ui.HStack(height=_row_height(0), spacing=6, style=_row_style(0)):
                        self._buttons["camera"] = ui.Button("AI 카메라 화면 보기", clicked_fn=self._cb["toggle_camera"], style=_button_style())
                    self._note("AI는 운전석이 아니라 포크 사이에 달린 카메라(포크와 함께 오르내림)로 봅니다. 녹화 이미지도 이 카메라로 찍습니다.")

                    ui.Label("AI 운전", height=0, style={"font_size": 14, "color": COLOR_TEXT})
                    ui.Line(height=1, style={"color": COLOR_BORDER})
                    self._labels["model"] = ui.Label("-", height=0, word_wrap=True, style={"font_size": 12, "color": COLOR_DIM})
                    self._labels["eyes"] = ui.Label("-", height=0, word_wrap=True, style={"font_size": 12, "color": COLOR_DIM})
                    with ui.HStack(height=22, spacing=6):
                        ui.Label("파렛트 거리 판단", width=96, style={"color": COLOR_DIM, "font_size": 12})
                        combo = ui.ComboBox(self._eyes_index, *[label for _, label in EYES_MODES])
                        combo.model.get_item_value_model().add_value_changed_fn(
                            lambda m: setattr(self, "_eyes_index", m.get_value_as_int()))
                    self._note("포크를 파렛트에 넣을 때 '얼마나 남았는지'를 YOLO로 잴지입니다. 'YOLO 없음'은 거리를 재지 않고 "
                               "사람이 녹화한 경로 끝에서 그냥 들어 올립니다 -- 파렛트를 뒤로 옮겨 두고 'YOLO 사용'과 비교해 보세요.")
                    with ui.HStack(height=22, spacing=6):
                        ui.CheckBox(model=self._reposition, width=20)
                        ui.Label("자리 고쳐 잡기", width=0, style={"color": COLOR_TEXT, "font_size": 12})
                    self._note("짧게 생각하고(가까운 녹화만 확인), 바로 갈 경로가 없으면 녹화 출발점 쪽으로 조금 움직인 뒤 그 자리에서 "
                               "다시 생각합니다 (최대 16번). 끄면 처음에 모든 경로를 꼼꼼히 확인하고, 없으면 바로 '경로 없음'으로 "
                               "끝납니다.")
                    # start buttons .. the note under the key display: half the usual gaps
                    ui.Spacer(height=4, style={"margin": 0})
                    with ui.HStack(height=_row_height(0), spacing=6, style=_row_style(0)):
                        self._buttons["ai_start"] = ui.Button(
                            "▶ AI 운전 시작", clicked_fn=self._cb["ai_start"], enabled=self._ai_ready,
                            style=_button_style(COLOR_GO, COLOR_GO_HOVER, COLOR_GO_TEXT))
                        self._buttons["ai_stop"] = ui.Button(
                            "■ AI 멈춤", clicked_fn=self._cb["ai_stop"], enabled=False,
                            style=_button_style(COLOR_STOP, COLOR_STOP_HOVER))
                    self._build_key_display()
                    self._labels["ai"] = ui.Label("", height=0, word_wrap=True, visible=False,  # shown with a message
                                                  style={"font_size": 13})
                    ui.Spacer(height=4, style={"margin": 0})
                    self._note("AI가 학습한 경로(forklift_ai\\models\\paths.json)들을 출발 전에 머릿속으로 연습해 보고, "
                               "가장 정확한 경로를 골라 키를 대신 누릅니다. 출발 위치는 맨 위의 '지게차 출발 위치'를 따릅니다.")
                    ui.Spacer(height=4)
                    ui.Label("AI 운전 기록", height=0, style={"font_size": 14, "color": COLOR_TEXT})
                    ui.Line(height=1, style={"color": COLOR_BORDER})
                    self._labels["runs"] = ui.Label("-", height=0, word_wrap=True, style={"font_size": 13})
                    self._run_row, self._run_rows = self._build_table(AI_RUN_COLUMNS, margin_height=3)
                    with ui.HStack(height=_row_height(0), spacing=6, style=_row_style(0)):  # half the usual gaps
                        self._buttons["run_discard"] = ui.Button("마지막 기록 지우기", clicked_fn=self._cb["run_discard"],
                                                                 style=_button_style())
                        self._buttons["run_clear"] = ui.Button("전체 지우기", clicked_fn=self._on_clear_clicked,
                                                               style=_button_style())
                    self._note("파렛트 칸은 파렛트를 뒤로 옮긴 거리, 경로 칸은 AI가 고른 시연의 번호(위 시연 표의 #)입니다. '변형'은 "
                               "그 시연 경로를 지금 출발 위치에 맞게 고친 것입니다. 결과에 마우스를 올리면 자세한 내용이, #에 올리면 "
                               "시각이 보입니다. 기록 파일: forklift_ai\\ai_runs.csv")
        self._fill_demo_rows()
        self._fill_run_rows()

    def _build_demo_table(self):
        self._demo_row, self._demo_rows = self._build_table(DEMO_COLUMNS)

    def _build_table(self, columns, margin_height=None):
        """A card with a header row and a scrolling body (DEMO_ROWS_VISIBLE rows show); `margin_height` sets
        the card's own (and its background's) top / bottom margin instead of the panel's 4.
        Returns (row(cells, colors=None, tooltips=None), body container).
        (A margin_height inherited from above overrides a widget's own 'margin', so the card's insides set
        both.)"""
        def row(cells, colors=None, tooltips=None, header=False):
            with ui.HStack(height=DEMO_ROW_H, spacing=2, style={"margin": 0, "margin_height": 0}):
                for i, ((_, width), text) in enumerate(zip(columns, cells)):
                    color = COLOR_DIM if header else (colors[i] if colors and colors[i] else COLOR_TEXT)
                    label = ui.Label(text, width=width or ui.Fraction(1), alignment=ui.Alignment.LEFT_CENTER,
                                     elided_text=True, style={"font_size": 12, "color": color})
                    if tooltips and tooltips[i]:
                        label.tooltip = tooltips[i]
                if header:
                    ui.Spacer(width=SCROLLBAR_W)

        with ui.ZStack(height=0, style={} if margin_height is None else {"margin_height": margin_height}):
            ui.Rectangle(style={"background_color": COLOR_CARD, "border_radius": 8})
            with ui.VStack(height=0, spacing=2, style={"margin": 6, "margin_height": 6}):
                row([name for name, _ in columns], header=True)
                ui.Line(height=1, style={"color": COLOR_BORDER, "margin": 0, "margin_height": 0})
                with ui.ScrollingFrame(
                    height=DEMO_ROWS_VISIBLE * DEMO_ROW_H,
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    style={"ScrollingFrame": {"background_color": COLOR_CARD, "scrollbar_size": SCROLLBAR_W},
                           "margin": 0, "margin_height": 0},
                ):
                    body = ui.VStack(height=0, spacing=0)
        return row, body

    def _fill_run_rows(self):
        if self._run_rows is None:
            return
        runs, demo_names = self._runs
        ok = sum(1 for r in runs if r.get("result") == "성공")
        if "runs" in self._labels:
            self._labels["runs"].text = f"AI 운전 {len(runs)}회 (성공 {ok}회)" if runs else "아직 AI 운전 기록이 없습니다."
        self._run_rows.clear()
        number_of = {name: i for i, name in enumerate(demo_names, 1)}
        result_color = {"성공": COLOR_OK, "중단": COLOR_DIM}
        with self._run_rows:
            for number, r in reversed(list(enumerate(runs, 1))):  # newest first
                route = r.get("route") or ""
                base = route.replace(" (변형)", "")
                route_text = (f"#{number_of[base]}" if base in number_of else (base[-6:] if base else "-")) + \
                    (" 변형" if route.endswith("(변형)") else "")
                secs = r.get("seconds")
                try:
                    secs = f"{float(secs):.0f}초" if secs not in (None, "") else "-"
                except ValueError:
                    secs = "-"
                eyes = r.get("eyes") or "-"
                try:
                    if eyes == "YOLO" and r.get("seat_true_m") and r.get("seat_used_m"):
                        eyes = f"YOLO ±{abs(float(r['seat_used_m']) - float(r['seat_true_m'])) * 100:.0f}"
                except ValueError:
                    pass
                result = r.get("result") or "-"
                self._run_row(
                    [str(number), r.get("start_vertical_m") or "-", r.get("start_horizontal_m") or "-",
                     r.get("pallet_back_m") or "0.00", result, secs, route_text, eyes],
                    colors=[None] * 4 + [result_color.get(result, COLOR_WARN)] + [None] * 3,
                    tooltips=[r.get("when") or None] + [None] * 3 + [r.get("detail") or None] + [None, route or None, None])

    def _on_clear_clicked(self):
        """'전체 지우기' asks once more before deleting every AI run record."""
        button = self._buttons.get("run_clear")
        if not self._clear_armed:
            self._clear_armed = True
            if button:
                button.text = "한 번 더 누르면 전체 삭제"

            def disarm():
                self._clear_armed = False
                if self._buttons.get("run_clear"):
                    self._buttons["run_clear"].text = "전체 지우기"

            asyncio.get_event_loop().call_later(4.0, disarm)
            return
        self._clear_armed = False
        if button:
            button.text = "전체 지우기"
        self._cb["run_clear"]()

    def _fill_demo_rows(self):
        if self._demo_rows is None:
            return
        demos, base = self._demos
        self._demo_rows.clear()
        with self._demo_rows:
            if not demos:
                ui.Label("아직 녹화한 시연이 없습니다.", height=DEMO_ROW_H, style={"font_size": 12, "color": COLOR_DIM})
            for number, d in reversed(list(enumerate(demos, 1))):  # newest first
                stamp = d["name"].replace("demo_", "")  # YYYYMMDD_HHMMSS
                when = f"{stamp[4:6]}-{stamp[6:8]} {stamp[9:11]}:{stamp[11:13]}" if len(stamp) >= 13 else stamp
                x, y = d["start"]
                if base is not None and x is not None and y is not None:
                    se, ga = f"{PANEL_SIGN['x'] * (x - base[0]):+.2f}", f"{PANEL_SIGN['y'] * (y - base[1]):+.2f}"
                else:
                    se = ga = "-"
                result = "성공" if d["success"] else "실패"
                self._demo_row([str(number), when, se, ga, f"{d['duration_s']:.0f}초", str(d["images"]), result],
                               colors=[None] * 6 + [COLOR_OK if d["success"] else COLOR_WARN])

    def _build_key_display(self):
        """Which driving keys are held right now (the AI's or the person's): arrow keys + Lift Up / Lift Down."""
        def key(name, text, width=KEY_SIZE[0]):
            with ui.ZStack(width=width, height=KEY_SIZE[1]):
                rect = ui.Rectangle(style={"background_color": COLOR_KEY, "border_radius": 5,
                                           "border_width": 1, "border_color": COLOR_BORDER})
                label = ui.Label(text, alignment=ui.Alignment.CENTER, style={"font_size": 13, "color": COLOR_DIM})
            self._keys[name] = (rect, label)

        # the card's top / bottom margin 3 instead of the panel's 4 (half the gaps around it); its insides set
        # margin_height too, as an inherited one overrides their own 'margin'
        with ui.ZStack(height=0, style={"margin_height": 3}):
            ui.Rectangle(style={"background_color": COLOR_CARD, "border_radius": 8})
            with ui.HStack(height=0, style={"margin": 0, "margin_height": 0}):
                ui.Spacer()
                with ui.VStack(width=0, height=0, spacing=4, style={"margin": 0}):
                    ui.Spacer(height=4)
                    with ui.HStack(width=0, height=0, spacing=4):
                        ui.Spacer(width=KEY_SIZE[0])
                        key("forward", "▲")
                        ui.Spacer(width=KEY_SIZE[0])
                    with ui.HStack(width=0, height=0, spacing=4):
                        key("left", "◀")
                        key("backward", "▼")
                        key("right", "▶")
                    ui.Spacer(height=4)
                ui.Spacer(width=28)
                with ui.VStack(width=0, height=0, spacing=4, style={"margin": 0}):
                    ui.Spacer(height=4)
                    key("lift_up", "Lift Up", LIFT_KEY_W)
                    key("lift_down", "Lift Down", LIFT_KEY_W)
                    ui.Spacer(height=4)
                ui.Spacer()
        self._key_state = {}

    # ---------------------------------------------------------------- api

    def _range(self, key):
        """(lo, hi) of the panel's 세로 ("x") / 가로 ("y") value; '확장' widens 가로."""
        if key == "y" and self._wide.get_value_as_bool():
            return WIDE_Y_RANGE_M
        return START_OFFSET_RANGE_M[key]

    def _on_wide_changed(self):
        lo, hi = self._range("y")
        slider = self._sliders.get("y")
        if slider is not None:
            slider.min, slider.max = lo, hi
        v = self._offset["y"].get_value_as_float()
        if not lo <= v <= hi:  # turning '확장' off with the forklift out wide: bring it back into range
            self._offset["y"].set_value(min(hi, max(lo, v)))

    @property
    def free_start(self):
        """'위치 자유' (trial): AI drives start from wherever the forklift stands now."""
        return self._free.get_value_as_bool()

    @property
    def random_start(self):
        return self._random.get_value_as_bool()

    def randomize_start(self, pallet=False, forklift=True):
        """Pick a random start inside the current ranges (and, with `pallet`, a random pallet position) and
        show it on the sliders."""
        for k in (("y", "x") if forklift else ()):
            lo, hi = self._range(k)
            self._offset[k].set_value(round(random.uniform(lo, hi), 2))
        if forklift:
            self._yaw.set_value(round(random.uniform(*START_YAW_RANGE_DEG)))
        if pallet:
            self._pallet_back.set_value(round(random.uniform(*PALLET_BACK_RANGE_M), 2))

    @property
    def start_offset(self):
        """(dx, dy) along the scene's axes."""
        def axis(k):
            lo, hi = self._range(k)
            return PANEL_SIGN[k] * min(hi, max(lo, self._offset[k].get_value_as_float()))
        return axis("x"), axis("y")

    @property
    def start_yaw(self):
        """회전 [deg] from the scene's start heading (+ = left)."""
        lo, hi = START_YAW_RANGE_DEG
        return min(hi, max(lo, self._yaw.get_value_as_float()))

    def set_start_offset(self, x, y):
        """Set the offset along the scene's axes (the panel shows it with its own signs)."""
        self._offset["x"].set_value(PANEL_SIGN["x"] * float(x))
        self._offset["y"].set_value(PANEL_SIGN["y"] * float(y))

    def set_recording(self, recording):
        if not self._labels:
            return
        self._labels["state"].text = "● 녹화 중" if recording else "대기"
        self._labels["state"].style = {"font_size": 20, "color": COLOR_REC if recording else COLOR_DIM}
        self._buttons["start"].enabled = not recording
        self._buttons["stop"].enabled = recording
        self._buttons["discard"].enabled = not recording
        if not recording:
            self._labels["live"].text = ""

    @property
    def eyes_mode(self):
        """"yolo" / "truth" / "none" (see EYES_MODES)."""
        return EYES_MODES[min(max(self._eyes_index, 0), len(EYES_MODES) - 1)][0]

    @property
    def allow_reposition(self):
        return self._reposition.get_value_as_bool()

    @property
    def use_eyes(self):
        return self.eyes_mode == "yolo"

    @property
    def pallet_back(self):
        lo, hi = PALLET_BACK_RANGE_M
        return min(hi, max(lo, self._pallet_back.get_value_as_float()))

    def set_pallet_back(self, metres):
        self._pallet_back.set_value(float(metres))

    def set_model_info(self, text, eyes_text=None):
        if "model" in self._labels:
            self._labels["model"].text = text
        if eyes_text is not None and "eyes" in self._labels:
            self._labels["eyes"].text = eyes_text

    def set_ai(self, driving, text=None, ok=None):
        self._driving = driving
        if not self._labels:
            return
        # while the AI drives, the start button is its pause / resume button
        self._buttons["ai_start"].enabled = self._pause_state != "idle" or (not driving and self._ai_ready)
        self._buttons["ai_stop"].enabled = driving
        self._buttons["start"].enabled = not driving
        if text is not None:
            self._labels["ai"].text = text
            self._labels["ai"].visible = bool(text)
            color = COLOR_TEXT if ok is None else (COLOR_OK if ok else COLOR_WARN)
            self._labels["ai"].style = {"font_size": 13, "color": color}

    def set_ai_pause_button(self, driving, paused):
        """While the AI drives, the start button becomes 'AI 일시 정지' (or 'AI 다시 시작' while paused)."""
        state = ("paused" if paused else "driving") if driving else "idle"
        button = self._buttons.get("ai_start")
        if button is None or state == self._pause_state:
            return
        self._pause_state = state
        if state == "idle":
            button.text = "▶ AI 운전 시작"
            button.style = _button_style(COLOR_GO, COLOR_GO_HOVER, COLOR_GO_TEXT)
            button.enabled = self._ai_ready and not self._driving
        elif state == "driving":
            button.text = "‖ AI 일시 정지"
            button.style = _button_style(COLOR_PAUSE, COLOR_PAUSE_HOVER, COLOR_GO_TEXT)
            button.enabled = True
        else:
            button.text = "▶ AI 다시 시작"
            button.style = _button_style(COLOR_GO, COLOR_GO_HOVER, COLOR_GO_TEXT)
            button.enabled = True

    def set_ai_ready(self, ready):
        """'AI 운전 시작' can only be pressed once there are learned routes."""
        self._ai_ready = ready
        if "ai_start" in self._buttons:
            self._buttons["ai_start"].enabled = self._pause_state != "idle" or (ready and not self._driving)

    def set_scene_stop(self, enabled):
        """After a finished AI run the scene stays paused; 'AI 멈춤' then stops (resets) it.
        Until then '녹화 시작' stays off, like while the AI drives."""
        if "ai_stop" in self._buttons and not self._driving:
            self._buttons["ai_stop"].enabled = enabled
            self._buttons["start"].enabled = not enabled

    def set_keys(self, held):
        """Light up the driving keys held right now ({key name: bool})."""
        for name, (rect, label) in self._keys.items():
            on = bool(held.get(name))
            if self._key_state.get(name) == on:
                continue
            self._key_state[name] = on
            rect.style = {"background_color": COLOR_GO if on else COLOR_KEY, "border_radius": 5,
                          "border_width": 1, "border_color": COLOR_GO if on else COLOR_BORDER}
            label.style = {"font_size": 13, "color": COLOR_GO_TEXT if on else COLOR_DIM}

    def set_demos(self, demos, base_xy):
        """Fill the recorded-demonstration table (base_xy: the scene's forklift start, for the 세로/가로 columns)."""
        self._demos = (demos, base_xy)
        self._fill_demo_rows()
        self._fill_run_rows()  # route numbers refer to the demonstration table

    def set_ai_runs(self, runs):
        """Fill the AI run table (rows as saved in ai_runs.csv, oldest first)."""
        self._runs = (runs, [d["name"] for d in self._demos[0]])
        self._fill_run_rows()

    def set_camera_button(self, showing_ai):
        if "camera" in self._buttons:
            self._buttons["camera"].text = "운전석 화면으로 돌아가기" if showing_ai else "AI 카메라 화면 보기"

    def set_live(self, text):
        if self._labels:
            self._labels["live"].text = text

    def set_summary(self, count, ok, images, secs):
        if self._labels:
            self._labels["summary"].text = (f"시연 {count}회 (2층에 올리기 성공 {ok}회) · 이미지 {images}장 · "
                                            f"총 {secs / 60:.1f}분")

    def set_last(self, text, ok=True):
        if self._labels:
            self._labels["last"].text = text
            self._labels["last"].visible = bool(text)
            self._labels["last"].style = {"font_size": 12, "color": COLOR_OK if ok else COLOR_WARN}

    def destroy(self):
        self._cb = {}
        self._labels = {}
        self._buttons = {}
        self._keys = {}
        self._sliders = {}
        self._demo_rows = None
        self._run_rows = None
        super().destroy()
