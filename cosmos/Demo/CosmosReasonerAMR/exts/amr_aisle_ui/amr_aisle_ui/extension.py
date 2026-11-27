# ① 통로 판단 테스트 창
#
# 이 창이 하는 일은 버튼으로 기존 모듈을 부르는 것뿐이다 (판단 로직은 프로젝트 폴더의 파일에 있다).
#   (켜질 때)         → scenes/amr_aisle_demo.usda 자동 열기 (없으면 amr_scene.build() 로 만들고 저장)
#   카메라 설정       → 로봇 따라가기(처음 화면) / 위에서 보기(누를 때마다 바로 위 시점으로 되돌림) / 로봇 시점(Cosmos 가 보는 앞 카메라)
#   장애물 배치       → incidents.set_aisle()
#   주행 시작         → run_aisle_demo.run(mode=...)  (막히면 우회)
#                       라이다 주행은 기본, Cosmos 는 맨 위 토글(ON/OFF)로 켜고 끈다 (끄면 라이다만)
#                       목적지: 1~4번 라디오 버튼 (바닥의 회색 원 번호)
#   정확도 시험       → 통로 4개 x 상태 9가지 = 36장을 차례로 판단하고 채점 (박스 계열 + 의미 기반 상황)
#
# 등록: Window → Extensions → ≡ → Settings → Extension Search Paths 에
#       C:/Projects/Edu_Cosmos/DemoProjects/CosmosReasonerAMR/exts 추가 → "AMR Aisle Judge" 켜기

import asyncio
import builtins
import importlib
import json
import linecache
import os
import sys
import time

import numpy as np
import omni.ext
import omni.kit.app
import omni.ui as ui

PROJECT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
# 다시 읽을 프로젝트 모듈 (의존하는 쪽이 뒤에 오도록). 새 모듈을 만들면 여기에 꼭 넣는다
# (lidar_drive 가 빠져 있어 amr_nav 를 고쳐도 라이다 주행은 예전 경로 계산을 계속 쓴 적이 있다)
MODULES = ["amr_config", "cosmos_client", "amr_scene", "capture", "incidents", "dynamic_events", "amr_nav", "rtx_lidar", "lidar_drive",
           "follow_cam", "bubble", "path_ribbon", "route_map", "aisle_judge", "run_aisle_demo", "data_gen"]

DEFAULT_SPEED = 5.0      # 배속 메뉴의 기본값 (amr_scene.SIM_SPEEDS 중 하나)

# 배치 옵션과 이름은 incidents.py 의 장애물 표에서 가져온다 (on_startup 에서 채운다). 작업자는 배치가 아니라
# 주행 중에 걸어 다닌다 (dynamic_events.py) — STATE_KO 에는 있지만 STATES 에는 없다.
STATES = []
STATE_KO = {}
AISLES = ["A", "B", "C", "D"]
IMG_W, IMG_H = 432, 243
ACC_N, ACC_SEED = 24, 1                    # 정확도 시험: 무작위 사진 장수와 seed (고정: 같은 사진으로 프롬프트를 비교할 수 있게)
SAVE_CHECK_TOP = 4                    # '주행 중 데이터 저장' 체크박스 위쪽 빈 칸(픽셀): 키우면 체크박스가 내려가고, 줄이면 올라간다
                                      # (0~4 를 나란히 놓고 비교했더니 4 에서 글씨와 가운데가 맞았다. 4 + 체크박스 18 = 줄 높이 22 라서 이보다 크면 잘린다)
COSMOS_BTN_M = 6                     # Cosmos 토글 글자 둘레 여백 (위·아래·왼쪽·오른쪽 모두 같게)
COSMOS_BTN_H = round(20.7 + 2 * COSMOS_BTN_M)   # 글자 높이 20.7 + 여백 2개
SERVER_CHECK_SEC = 15            # 서버 상태 자동 확인 주기


def normalize_url(text):
    t = text.strip().rstrip("/")
    if not t or not t.isascii() or " " in t:       # 한/영 전환 실수(ㅣㅣㅣ 등)나 공백은 주소가 아니다
        return None
    if "://" not in t:
        t = "http://" + t
    scheme, rest = t.split("://", 1)
    host, _, path = rest.partition("/")
    if ":" not in host.strip("[]").split("]")[-1]:
        host += ":8000"
    if not path.startswith("v1/"):
        path = "v1/chat/completions"
    return f"{scheme}://{host}/{path}"

# 2. 환경 구성의 항목 이름(목적지 선택·장애물 배치·작업자 걸어 다니기·Cosmos 판단 사용)을 같은 목록처럼 보이게 하는 글씨 모양
LIST_LABEL = {"font_size": 18, "color": 0xFFFFFFFF}
LIST_ROW_H = 30                                   # 큰 글씨가 들어가는 줄 높이

GREEN = 0xFF4CC26E
RED = 0xFF4A4AE0
GRAY = 0xFFAAAAAA
YELLOW = 0xFF40C8F0


def _load_modules():
    """프로젝트 모듈을 새로 읽는다 (파일을 고친 뒤 익스텐션을 껐다 켜면 바뀐 코드가 반영됨)."""
    if PROJECT not in sys.path:
        sys.path.insert(0, PROJECT)
    for m in MODULES:
        sys.modules.pop(m, None)
    linecache.clearcache()
    importlib.invalidate_caches()
    return {m: importlib.import_module(m) for m in MODULES}


def url_to_ip(url):
    host = url.split("://", 1)[-1].split("/", 1)[0]
    return host[:-5] if host.endswith(":8000") else host


class AmrAisleExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._m = _load_modules()
        STATES[:] = self._m["incidents"].STATES
        STATE_KO.clear()
        STATE_KO.update(self._m["incidents"].KO)
        self._busy = False
        self._driving = False                # 주행 중인가 (플레이·정지 버튼 표시용)
        self._stopping = False               # 정지를 눌렀고 아직 끝나지 않았는가
        self._stop = False
        self._nav = None
        self._score = []                     # (정답 통과 가능, 판단) 목록
        self._log_lines = []
        self._img = ui.ByteImageProvider()
        self._server_ok = None
        self._build_ui()
        self._log("info", f"프로젝트 폴더: {PROJECT}")
        self._log("info", "Cosmos 서버 주소: " + ("설정됨" if self._m["amr_config"].NIM_URL else "없음 - IP 주소 칸에 입력하세요"))
        self._watch_task = asyncio.ensure_future(self._watch_server())
        self._open_scene()                           # 저장한 장면 파일을 자동으로 연다

    def on_shutdown(self):
        self._stop = True
        for t in getattr(self, "_tasks", []):        # 장면 열기·주행 같은 진행 중 작업도 같이 끝낸다
            t.cancel()                               # (끄는 도중에 남아 있으면 'Task exception was never retrieved' 가 떴다)
        try:
            self._m["follow_cam"].stop()
            self._m["bubble"].stop()
            self._m["path_ribbon"].stop()
            self._m["route_map"].stop()
            self._m["amr_scene"].stop_marker_anim()
            self._m["amr_scene"].set_sim_speed(1.0)
            self._m["dynamic_events"].remove_all()
        except Exception:
            pass
        if getattr(self, "_watch_task", None):
            self._watch_task.cancel()
        if self._nav:
            self._nav.stop_requested = True
        if self._win:
            self._win.destroy()
            self._win = None

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        self._win = ui.Window("AMR 통로 판단 (Cosmos)", width=470, height=900)
        with self._win.frame:
          with ui.VStack(spacing=4):
            # ---- 최상단: Cosmos 서버 상태 (스크롤해도 항상 보임, 15초마다 자동 확인) ----
            with ui.ZStack(height=80):
                ui.Rectangle(style={"background_color": 0xFF2A2A2A, "border_radius": 4})
                with ui.VStack(spacing=4):
                    ui.Spacer(height=2)
                    with ui.HStack(height=24, spacing=4):
                        ui.Spacer(width=6)
                        ui.Label("IP 주소", width=50)
                        self._url_field = ui.StringField()
                        self._url_field.model.set_value(url_to_ip(self._m["amr_config"].NIM_URL))
                        self._url_field.model.add_end_edit_fn(lambda _m: self._on_apply_url())   # Enter 로도 적용
                        ui.Button("적용", width=50, clicked_fn=self._on_apply_url)
                        ui.Spacer(width=4)
                    with ui.HStack(height=24, spacing=6):
                        ui.Spacer(width=6)
                        with ui.VStack(width=14):
                            ui.Spacer()
                            self._server_dot = ui.Circle(radius=6, width=14, height=14,
                                                         style={"background_color": GRAY})
                            ui.Spacer()
                        self._server_lbl = ui.Label("확인 중...", style={"color": GRAY, "font_size": 16})
                        ui.Button("다시 확인", width=80, clicked_fn=self._on_server)
                        ui.Spacer(width=4)
                    with ui.HStack(height=14):
                        ui.Spacer(width=26)
                        self._url_lbl = ui.Label(f"서버 주소: {self._m['amr_config'].NIM_URL}",
                                                 style={"color": GRAY, "font_size": 12})
            with ui.ScrollingFrame():
                with ui.VStack(spacing=6, height=0):
                    with ui.CollapsableFrame("1. 카메라 설정", height=0):
                        with ui.VStack(spacing=4):
                            with ui.HStack(height=26, spacing=4):
                                ui.Button("위에서 보기", clicked_fn=lambda: self._set_cam("top"))
                                ui.Button("로봇 따라가기", clicked_fn=lambda: self._set_cam("follow"))
                            with ui.HStack(height=26, spacing=4):
                                ui.Button("자유 시점", clicked_fn=lambda: self._set_cam("free"))
                                ui.Button("로봇 시점", clicked_fn=lambda: self._set_cam("front"))

                    with ui.CollapsableFrame("2. 환경 구성", height=0):
                        with ui.ZStack():                                                         # 안쪽 + 그 위에 덮는 반투명 막
                            self._env_body = ui.VStack(spacing=4)                         # 주행·작업 중에는 안쪽을 비활성화한다
                            with self._env_body:
                                with ui.HStack(height=LIST_ROW_H, spacing=4):
                                    ui.Label("목적지 선택", width=110, style=LIST_LABEL, alignment=ui.Alignment.LEFT_CENTER)
                                    goals = self._m["amr_scene"].GOALS
                                    self._goal_keys = list(goals)
                                    self._goal_idx = self._goal_keys.index(self._m["amr_scene"].DEFAULT_GOAL)
                                    self._goal_radio = ui.RadioCollection(model=ui.SimpleIntModel(self._goal_idx))
                                    for i, k in enumerate(self._goal_keys):
                                        ui.RadioButton(radio_collection=self._goal_radio, text=f"{goals[k][0]}번", width=64)
                                    ui.Label("  (바닥의 회색 원 번호)", style={"color": GRAY})
                                    self._goal_radio.model.add_value_changed_fn(lambda _m: self._on_goal_changed())
                                with ui.HStack(height=LIST_ROW_H - 2, spacing=4):
                                    ui.Label("장애물 배치", width=0, style=LIST_LABEL, alignment=ui.Alignment.LEFT_CENTER)
                                    ui.Label("  (정답은 채점에만 사용)", width=0, style={"color": GRAY, "font_size": 13})
                                    ui.Spacer()
                                    # 눈에 띄지 않게: 배경 없이 회색 글씨의 작은 버튼
                                    quiet = {"Button": {"background_color": 0x00000000, "color": GRAY, "font_size": 13},
                                             "Button:hovered": {"background_color": 0x33FFFFFF, "color": 0xFFDDDDDD}}
                                    ui.Button("모두 비우기", width=70, height=20, style=quiet, clicked_fn=self._on_clear)
                                    ui.Button("무작위", width=50, height=20, style=quiet, clicked_fn=self._on_random)
                                self._combo = {}
                                self._syncing = False                                     # 코드가 드롭다운 값을 바꿀 때는 콜백을 무시
                                self._placed = {a: 0 for a in AISLES}                     # 마지막으로 장면에 배치한 값 (STATES 번호)
                                with ui.HStack(height=44, spacing=6):                     # 1 2 3 4 통로를 왼쪽 → 오른쪽으로
                                    for a in AISLES:
                                        with ui.VStack(spacing=2):
                                            ui.Label(f"통로{self._m['amr_scene'].aisle_no(a)}", height=18, alignment=ui.Alignment.LEFT_CENTER)
                                            self._combo[a] = ui.ComboBox(0, *[STATE_KO[s] for s in STATES])
                                            # 고를 때마다 바로 그 통로에 배치한다
                                            self._combo[a].model.add_item_changed_fn(
                                                lambda _m, _i, aisle=a: self._on_combo_changed(aisle))
                                # 작업자 3명이 주행 중 걸어 다닌다 (기본 꺼짐, 주행 시작 전에만 바꿀 수 있음)
                                self._workers_state = False
                                with ui.HStack(height=LIST_ROW_H, spacing=6):
                                    self._workers_check = ui.CheckBox(width=20)
                                    ui.Label("작업자 걸어 다니기", alignment=ui.Alignment.LEFT_TOP)
                                self._workers_check.model.add_value_changed_fn(lambda _m: self._on_workers_toggle())
                                # Cosmos 는 이 데모의 주인공이라 ON 이면 통로 입구마다 Cosmos 가 판단, OFF 면 라이다만
                                self._cosmos_state = True
                                with ui.ZStack(height=COSMOS_BTN_H):                      # 주행 중에는 반투명 회색 막으로 덮는다
                                    self._cosmos_row = ui.HStack(height=COSMOS_BTN_H, spacing=12)
                                    with self._cosmos_row:
                                        self._cosmos_lbl = ui.Label("Cosmos 판단 사용", width=0, alignment=ui.Alignment.LEFT_CENTER)
                                        self._cosmos_btn = ui.Button("", width=ui.Pixel(60), height=COSMOS_BTN_H,
                                                                     clicked_fn=self._on_cosmos_toggle)
                                        ui.Spacer()
                                    self._cosmos_cover = ui.Rectangle(style={"background_color": 0xB0303030, "border_radius": 4}, visible=False)
                                self._update_cosmos_ui()
                                with ui.HStack():                                           # 규칙·질문을 'Cosmos 판단 사용' 의 하위 항목으로 한 단계 안쪽에 둔다
                                    ui.Spacer(width=16)
                                    with ui.VStack(spacing=4):
                                        # Cosmos 프롬프트: 장애물별 판단 버튼(막힘/통과) + 프롬프트 글 직접 편집. Cosmos 판단 사용이 ON 일 때만 입력할 수 있다 (주행 중에는 잠김)
                                        self._prompt_frame = ui.VStack(spacing=4)     # 판단 사용이 OFF 면 통째로 숨긴다
                                        with self._prompt_frame:
                                            ui.Label("규칙(System)", height=22, style={"font_size": 16})
                                            with ui.HStack():
                                                ui.Spacer(width=16)                                   # 제목보다 한 단계 안쪽으로 (Tab 들여쓰기)
                                                with ui.VStack(spacing=4):
                                                    with ui.ZStack():
                                                        self._prompt_body = ui.VStack(spacing=4)
                                                        with self._prompt_body:
                                                            aj = self._m["aisle_judge"]
                                                            ui.Label("Cosmos 가 이 장애물을 어떻게 판단할지 설정하세요. (버튼을 누르면 프롬프트가 바뀝니다)",
                                                                     height=18, word_wrap=True, style={"color": GRAY, "font_size": 13})
                                                            self._rule_btns = {}
                                                            with ui.HStack(height=26, spacing=4):
                                                                for k in self._m["incidents"].rule_keys():
                                                                    self._rule_btns[k] = ui.Button("", clicked_fn=lambda key=k: self._on_rule_clicked(key))
                                                            with ui.HStack(height=20, spacing=6):
                                                                self._prompt_count = ui.Label("", width=0, style={"color": GRAY, "font_size": 12})
                                                            self._syncing_prompt = False                                  # 코드가 칸의 글을 바꿀 때는 입력 콜백을 무시
                                                            self._prompt_field = ui.StringField(multiline=True, height=ui.Pixel(115))
                                                            self._prompt_field.model.set_value(aj.PROMPT_TEXT)
                                                            ui.Label(aj.FORMAT_TAIL, word_wrap=True, style={"color": 0xFF8C8C8C, "font_size": 12})        # 고정 형식: 고칠 수 없고 항상 입력 칸 아래에 보인다
                                                        self._prompt_cover = ui.ZStack(visible=False)
                                                        with self._prompt_cover:
                                                            ui.Rectangle(style={"background_color": 0xB0303030, "border_radius": 4})
                                                            self._prompt_cover_lbl = ui.Label("", alignment=ui.Alignment.CENTER,
                                                                                              style={"color": 0xFFE0E0E0, "font_size": 15})
                                        # 질문(User): 사진과 함께 가는 질문. 규칙(System)과 따로 보여 주고 고칠 수 없다
                                        self._question_frame = ui.VStack(spacing=4)
                                        with self._question_frame:
                                            ui.Label("질문(User)", height=22, style={"font_size": 16})
                                            with ui.HStack():
                                                ui.Spacer(width=16)                                   # 제목보다 한 단계 안쪽으로 (Tab 들여쓰기)
                                                ui.Label(aj.USER_TEXT.replace("{aisle}", "<통로 글자>"), word_wrap=True,
                                                         style={"color": 0xFF8C8C8C, "font_size": 12})
                            # 주행 중에는 반투명 회색 막으로 덮어 비활성 상태가 눈에 띄게 한다
                            self._env_cover = ui.ZStack(visible=False)
                            with self._env_cover:
                                ui.Rectangle(style={"background_color": 0xB0303030, "border_radius": 4})
                                ui.Label("주행 중에는 바꿀 수 없습니다", alignment=ui.Alignment.CENTER,
                                         style={"color": 0xFFE0E0E0, "font_size": 15})

                    with ui.CollapsableFrame("3. 실행하기", height=0):
                        with ui.VStack(spacing=4):
                            # 맨 아래: 정확도 시험 · 사진 폴더 열기 (눈에 띄지 않게). 주행·시험 중에도 눌러야(시험 중지) 해서 회색 막 바깥에 둔다
                            quiet = {"Button": {"background_color": 0x00000000, "color": GRAY, "font_size": 13},
                                     "Button:hovered": {"background_color": 0x33FFFFFF, "color": 0xFFDDDDDD},
                                     "Button:disabled": {"background_color": 0x00000000, "color": 0xFF555555}}
                            self._quiet_row = ui.HStack(height=22, spacing=4)         # Cosmos 판단 사용이 OFF 면 함께 숨긴다
                            with self._quiet_row:
                                ui.Spacer()
                                # 데이터 저장 (기본 꺼짐): 켜면 다음 주행부터 통로 입구에서 Cosmos 에 보낸 사진을 박스 라벨 이미지·
                                # Cosmos 응답과 함께 datasets/aisle_drive_<시각>/ 에 저장한다
                                # 글씨 칸 높이를 줄 높이(22)에 고정한다: 지정하지 않으면 칸이 28 로 잡혀 줄 밖으로 넘치고, 글씨가 체크박스보다 아래로 내려간다
                                self._save_label = ui.Label("주행 중 데이터 저장", width=0, height=22, alignment=ui.Alignment.V_CENTER,
                                                            style={"color": GRAY, "font_size": 13})
                                with ui.VStack(width=18):
                                    ui.Spacer(height=SAVE_CHECK_TOP)           # 체크박스를 글씨 높이에 맞춰 아래로 내린다 (픽셀)
                                    self._save_check = ui.CheckBox(width=18, height=18)
                                self._save_check.model.set_value(False)
                                ui.Button("데이터 폴더 열기", width=100, height=20, style=quiet, clicked_fn=self._on_open_folder)
                                self._acc_btn = ui.Button("정확도 시험", width=90, height=20, clicked_fn=self._on_accuracy)
                                self._acc_btn.style = quiet
                            self._prompt_field.model.add_value_changed_fn(lambda _m: self._on_prompt_edited())
                            self._refresh_rule_buttons()
                            self._on_prompt_edited()
                            self._update_prompt_ui()
                            with ui.ZStack(height=26):                                # 주행 중에는 반투명 회색 막으로 덮는다
                                self._speed_row = ui.HStack(height=26, spacing=4)
                                with self._speed_row:
                                    ui.Label("배속", width=60)
                                    speeds = self._m["amr_scene"].SIM_SPEEDS
                                    # 기본 배속 = DEFAULT_SPEED (1배는 시연하기에 너무 느려 5배를 기본으로 한다)
                                    default_idx = speeds.index(DEFAULT_SPEED) if DEFAULT_SPEED in speeds else 0
                                    self._speed_combo = ui.ComboBox(default_idx, *[f"{v:g}배" for v in speeds])
                                    self._speed_combo.model.add_item_changed_fn(lambda *_: self._on_speed_changed())
                                self._speed_cover = ui.Rectangle(style={"background_color": 0xB0303030, "border_radius": 4},
                                                                 visible=False)
                            with ui.HStack(height=38, spacing=6):
                                self._play_btn = ui.Button("", clicked_fn=self._on_drive)
                                self._stop_btn = ui.Button("", width=120, clicked_fn=self._on_stop)
                            self._update_run_buttons()
                            self._busy_lbl = ui.Label("대기 중", style={"color": GRAY})

                    with ui.CollapsableFrame("Cosmos 전송 데이터", height=0):
                        with ui.VStack(spacing=2):
                            ui.ImageWithProvider(self._img, width=IMG_W, height=IMG_H)
                            self._img_lbl = ui.Label("아직 없음", style={"color": GRAY})
                            with ui.CollapsableFrame("Request (Cosmos 에 보낸 JSON)", collapsed=True, height=0):
                                with ui.ScrollingFrame(height=ui.Pixel(240)):
                                    self._req_lbl = ui.Label("아직 없음", word_wrap=True, style={"color": GRAY, "font_size": 12})

                    with ui.CollapsableFrame("판단 결과", height=0):
                        with ui.VStack(spacing=3):
                            self._verdict = ui.Label("-", style={"font_size": 22, "color": GRAY})
                            ui.Label("Response(Cosmos 응답)", height=16, style={"color": GRAY, "font_size": 12})
                            with ui.ScrollingFrame(height=ui.Pixel(170)):
                                self._raw_lbl = ui.Label("", word_wrap=True, style={"color": 0xFFCCCCCC, "font_size": 12})
                            with ui.HStack(height=22):
                                self._score_lbl = ui.Label("채점: 0 / 0")
                                ui.Button("채점 초기화", width=90, clicked_fn=self._reset_score)

                    with ui.CollapsableFrame("진행 기록", height=0):
                        with ui.VStack(spacing=2):
                            with ui.HStack(height=20):
                                ui.Spacer()
                                ui.Button("지우기", width=64, height=20, tooltip="진행 기록을 모두 지웁니다", clicked_fn=self._on_clear_log)
                            with ui.ScrollingFrame(height=220):
                                self._log_stack = ui.VStack(spacing=1, height=0)

    # ------------------------------------------------------------------ 공통
    def _log(self, kind, text):
        colors = {"blocked": RED, "stuck": RED, "error": RED, "arrived": GREEN, "pass": GREEN,
                  "lidar": 0xFFE0A040,
                  "cosmos": YELLOW, "wait": YELLOW, "plan": 0xFFDDDDDD}
        line = f"{time.strftime('%H:%M:%S')} [{kind}] {text}"
        self._log_lines.append((line, colors.get(kind, GRAY)))
        self._log_lines = self._log_lines[-200:]
        self._log_stack.clear()
        with self._log_stack:
            for t, c in reversed(self._log_lines):          # 최신이 위
                ui.Label(t, word_wrap=True, style={"color": c})

    def _update_run_buttons(self):
        """플레이·정지 버튼의 모양을 상태에 맞춘다 (누른 것이 눈에 보이게):
          대기 중  : 플레이 = 선명한 초록(누를 수 있음) / 정지 = 어두운 회색(비활성)
          주행 중  : 플레이 = 어두운 초록 + 밝은 테두리 '플레이 중'(비활성) / 정지 = 선명한 빨강(누를 수 있음)
          정지 누름: 정지 = 어두운 빨강 '정지 중...'(비활성)
          다른 작업(정확도 시험 등): 플레이 = 회색(비활성) / 정지 = 빨강(누를 수 있음)"""
        busy, driving, stopping = self._busy, self._driving, self._stopping
        self._update_prompt_ui()
        if hasattr(self, "_env_body"):
            self._env_body.enabled = not busy                # 플레이(주행) 중에는 목적지·장애물·작업자를 바꿀 수 없다
            self._env_cover.visible = busy                   # 반투명 회색 막으로 덮어 눈에 띄게
        if hasattr(self, "_speed_row"):
            self._speed_row.enabled = not busy               # 배속도 주행 중에는 바꿀 수 없다 (반투명 막으로 덮음)
            self._speed_cover.visible = busy
        if hasattr(self, "_cosmos_row"):
            self._cosmos_row.enabled = not busy              # Cosmos 판단 토글·정확도 시험도 주행 중에는 쓸 수 없다
            self._cosmos_cover.visible = busy
        if hasattr(self, "_acc_btn"):                        # 창을 만드는 중에는 아직 없을 수 있다 (버튼 순서 때문)
            acc = getattr(self, "_accuracy", False)
            if acc:                                          # 시험 중: 같은 버튼으로 중지 (중지 요청 뒤에는 끝날 때까지 비활성)
                self._acc_btn.text = "중지 중..." if stopping else "시험 중지"
                self._acc_btn.enabled = not stopping
            else:
                self._acc_btn.text = "정확도 시험"
                self._acc_btn.enabled = not busy             # 주행 중에는 쓸 수 없다
        font = {"font_size": 17, "border_radius": 6}
        if not busy:
            self._play_btn.text, self._play_btn.enabled = "▶  플레이", True
            self._play_btn.style = {"background_color": 0xFF3A9E2E, "color": 0xFFFFFFFF, **font}
        elif driving:
            self._play_btn.text, self._play_btn.enabled = "▶  플레이 중", False
            self._play_btn.style = {"background_color": 0xFF1F4A1A, "color": 0xFF9FD99A, "border_width": 2,
                                    "border_color": 0xFF4CC26E, **font}
        else:
            self._play_btn.text, self._play_btn.enabled = "▶  플레이", False
            self._play_btn.style = {"background_color": 0xFF3A3A3A, "color": 0xFF808080, **font}
        if not busy:
            self._stop_btn.text, self._stop_btn.enabled = "■  정지", False
            self._stop_btn.style = {"background_color": 0xFF3A3A3A, "color": 0xFF808080, **font}
        elif stopping and not getattr(self, "_accuracy", False):
            self._stop_btn.text, self._stop_btn.enabled = "■  정지 중...", False
            self._stop_btn.style = {"background_color": 0xFF202060, "color": 0xFF9090D0, "border_width": 2,
                                    "border_color": 0xFF4A4AE0, **font}
        elif getattr(self, "_accuracy", False):              # 정확도 시험 중에는 정지 버튼을 못 쓴다 (시험 버튼으로 중지)
            self._stop_btn.text, self._stop_btn.enabled = "■  정지", False
            self._stop_btn.style = {"background_color": 0xFF3A3A3A, "color": 0xFF808080, **font}
        else:
            self._stop_btn.text, self._stop_btn.enabled = "■  정지", True
            self._stop_btn.style = {"background_color": 0xFF3030D0, "color": 0xFFFFFFFF, **font}

    def _set_busy(self, text=None):
        self._busy = text is not None
        if not self._busy:
            self._stopping = False
        self._busy_lbl.text = text or "대기 중"
        self._busy_lbl.style = {"color": YELLOW if text else GRAY}
        if hasattr(self, "_play_btn"):
            self._update_run_buttons()

    def _alive(self):
        """확장이 켜져 있는가. 꺼진 뒤에는 창과 내부 모듈 목록이 사라져 있어서 화면·모듈을 건드리면 오류가 난다."""
        return getattr(self, "_win", None) is not None and hasattr(self, "_m")

    def _run(self, coro_fn, label):
        if self._busy:
            self._log("error", f"다른 작업 중입니다: {self._busy_lbl.text}")
            return
        self._stop = False

        async def wrapper():
            if not self._alive():
                return
            self._set_busy(label)
            try:
                await coro_fn()
            except Exception as e:
                if not self._alive():                # 확장이 꺼지는 중에 난 오류는 조용히 끝낸다
                    return
                import traceback
                self._log("error", f"{type(e).__name__}: {e}")
                print(traceback.format_exc())
            finally:
                if self._alive():
                    self._set_busy(None)
        self._tasks = [t for t in getattr(self, "_tasks", []) if not t.done()]
        self._tasks.append(asyncio.ensure_future(wrapper()))

    def _states_from_ui(self):
        return {a: STATES[self._combo[a].model.get_item_value_model().as_int] for a in AISLES}

    def _show_image(self, rgb, caption):
        from PIL import Image
        small = Image.fromarray(rgb).resize((IMG_W, IMG_H)).convert("RGBA")
        arr = np.asarray(small)
        try:
            self._img.set_data_array(arr, [IMG_W, IMG_H])
        except Exception:
            self._img.set_bytes_data(arr.flatten().tolist(), [IMG_W, IMG_H])
        self._img_lbl.text = caption

    @staticmethod
    def _pretty_raw(raw):
        """Cosmos 원문 답을 읽기 좋게: JSON 이면 들여쓰기하고 \\uXXXX 를 글자로 풀어 보여 준다. 아니면 그대로."""
        text = (raw or "").strip()
        body = text.replace("```json", "").replace("```", "").strip()
        try:
            return json.dumps(json.loads(body), ensure_ascii=False, indent=2)
        except ValueError:
            return text

    def _show_request(self, res=None):
        """Cosmos 에 보낸 요청 JSON (사진 데이터는 크기만)을 Request 칸에 보여 준다.
        판단 결과(res)가 있으면 1차 요청과, 통과였을 때 한 번 더 보낸 확대 재확인 요청을 모두 보여 주고,
        없으면(라이다 막힘 설명 등) 마지막으로 보낸 요청을 보여 준다."""
        def dump(body):
            return json.dumps(body, ensure_ascii=False, indent=2)
        if res and res.get("request"):
            text = "// 1차 요청 (원래 사진)\n" + dump(res["request"])
            if res.get("double_check_request"):
                text += "\n\n// 확대 재확인 요청 (같은 질문, 가운데를 2배로 확대한 사진)\n" + dump(res["double_check_request"])
            self._req_lbl.text = text
            return
        body = getattr(self._m["cosmos_client"].ask, "last_body", None)
        self._req_lbl.text = dump(body) if body else "아직 없음"

    def _on_clear_log(self):
        self._log_lines = []
        self._log_stack.clear()

    def _show_lidar_block(self, aisle, why, raw=None):
        """입구에서는 통과였는데 안에서 라이다에 막힌 경우: 제목에 라이다 막힘, 아래에 Cosmos 가 말해 준 앞의 장면 (채점 안 함)."""
        self._verdict.text = f"통로{self._m['amr_scene'].aisle_no(aisle)} : 라이다 막힘"
        self._verdict.style = {"font_size": 22, "color": RED}
        self._raw_lbl.text = self._pretty_raw(raw) if raw else json.dumps({"reason": why}, ensure_ascii=False, indent=2)
        self._show_request()

    def _show_result(self, aisle, res, state=None):
        ok = res["decision"]
        self._verdict.text = f"통로{self._m['amr_scene'].aisle_no(aisle)} : {'Pass' if ok else 'Reroute'}"
        self._verdict.style = {"font_size": 22, "color": GREEN if ok else RED}
        if state is not None and not res.get("error"):         # 채점: 형식 오류·서버 오류는 Cosmos 가 판단한 것이 아니므로 뺀다
            self._score.append(self._m["incidents"].passable(state) == ok)
            self._score_lbl.text = f"채점: {sum(self._score)} / {len(self._score)}"
        raw1 = self._pretty_raw(res.get("raw")) if res.get("raw") else (res.get("error") or "")
        if res.get("double_check_raw") is not None:                # 통과였을 때 한 번 더 보낸 확대 재확인의 답도 같이 보여 준다
            self._raw_lbl.text = ("// 1차 답\n" + raw1 + "\n\n// 확대 재확인 답\n" + self._pretty_raw(res.get("double_check_raw")))
        else:
            self._raw_lbl.text = raw1
        self._show_request(res)

    def _reset_score(self):
        self._score = []
        self._score_lbl.text = "채점: 0 / 0"

    # ------------------------------------------------------------------ 버튼
    async def _check_server(self):
        """서버 상태를 확인해서 최상단 표시를 바꾼다. 상태가 바뀔 때만 진행 기록에 남긴다."""
        self._server_dot.style = {"background_color": YELLOW}
        t0 = time.time()
        ok, info = await asyncio.get_event_loop().run_in_executor(None, self._m["cosmos_client"].server_ok)
        ms = (time.time() - t0) * 1000
        if ok:
            self._server_lbl.text = f"연결됨  ·  {', '.join(info) or '모델 없음'}  ·  {ms:.0f} ms"
            detail = self._server_lbl.text
        else:
            self._server_lbl.text = "연결 안 됨"
            reason = "연결 거부" if "10061" in str(info) else                      "시간 초과" if "timed out" in str(info).lower() else str(info)[:80]
            detail = f"연결 안 됨 ({reason})"
        color = GREEN if ok else RED
        self._server_dot.style = {"background_color": color}
        self._server_lbl.style = {"color": color, "font_size": 16}
        if ok != self._server_ok:
            self._log("pass" if ok else "error", f"Cosmos 서버: {detail}")
        self._server_ok = ok
        return ok

    async def _watch_server(self):
        while True:
            try:
                await self._check_server()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self._log("error", f"서버 확인 오류: {e}")
            await asyncio.sleep(SERVER_CHECK_SEC)

    def _on_server(self):
        asyncio.ensure_future(self._check_server())

    def _on_apply_url(self):
        url = normalize_url(self._url_field.model.get_value_as_string())
        if not url:
            self._server_dot.style = {"background_color": RED}
            self._server_lbl.text = "IP 형식 오류 (예: xxx.xxx.xx.xx 또는 xxx.xxx.xx.xx:xxxx)"
            self._server_lbl.style = {"color": RED, "font_size": 16}
            self._log("error", "잘못된 IP 형식 → 이전 주소 유지")
            return
        cfg = self._m["amr_config"]
        self._url_field.model.set_value(url_to_ip(url))
        self._url_lbl.text = f"서버 주소: {url}"
        if url == cfg.NIM_URL and self._server_ok is not None:
            self._on_server()
            return
        cfg.NIM_URL = url                       # cosmos_client 는 부를 때마다 config.NIM_URL 을 읽으므로 바로 반영된다
        self._server_ok = None
        self._log("info", "서버 주소 변경")
        self._on_server()

    def _open_scene(self):
        """익스텐션이 켜질 때: 저장한 장면 파일(scenes/amr_aisle_demo.usda)을 열고 '로봇 따라가기' 시점으로 시작한다."""
        async def go():
            sc = self._m["amr_scene"]
            self._m["capture"].reset()              # 장면이 바뀌면 예전 렌더 프로덕트는 쓸 수 없다
            how = await sc.open_scene_async()
            # 장면을 연 직후에는 뷰포트가 카메라를 파일 값으로 다시 맞추므로, 조금 기다렸다가 시점을 바꾼다
            for _ in range(30):
                await omni.kit.app.get_app().next_update_async()
            if not self._alive():
                return
            # 배속은 메뉴에 선택된 값으로 시작한다. (예전에는 무조건 1배로 되돌려서, 메뉴 기본값을 5배로 바꾼 뒤에는
            # '메뉴는 5배인데 실제로는 1배' 인 상태로 시작할 수 있었다)
            sc.set_sim_speed(sc.SIM_SPEEDS[self._speed_combo.model.get_item_value_model().as_int])
            self._m["follow_cam"].start()              # 처음 화면은 '로봇 따라가기'
            sc.start_marker_anim()                       # 로봇 표시(펄스 링·신호 호) 움직임
            msg = {"already": "이미 열려 있음", "opened": "열었습니다", "built": "파일이 없어 새로 만들고 저장했습니다"}[how]
            self._log("info", f"장면 파일 {msg}: {os.path.relpath(sc.SCENE_FILE, PROJECT)}")
        self._run(go, "장면 여는 중")

    async def _ensure_stopped(self):
        """로봇을 코드로 옮기는 일(정확도 시험의 입구 사진)은 물리가 멈춘 상태에서 해야 한다 (라이다 주행 뒤에는 Play 중이다)."""
        import omni.timeline
        tl = omni.timeline.get_timeline_interface()
        if not tl.is_stopped():
            tl.stop()
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()

    def _set_cam(self, which):
        sc, fc = self._m["amr_scene"], self._m["follow_cam"]
        if which == "follow":
            fc.start()                              # 로봇 뒤 위쪽에서 따라감 (주행 중에도 계속)
            return
        fc.stop()
        from omni.kit.viewport.utility import get_active_viewport
        if which == "top":
            sc.reset_top_view()                     # 누를 때마다 창고 바로 위 시점으로 되돌린다 (마우스로 움직일 수는 있음)
        if which == "free":
            sc.ensure_free_cam()                    # 원근 카메라: 이미 있으면 마우스로 옮겨 둔 위치 그대로
            get_active_viewport().camera_path = sc.FREE_CAM
            return
        # 로봇 시점 = Cosmos 가 보는 앞 카메라
        get_active_viewport().camera_path = {"top": sc.TOP_CAM, "front": sc.FRONT_CAM}[which]

    def _set_combo(self, aisle, idx):
        self._syncing = True
        try:
            self._combo[aisle].model.get_item_value_model().set_value(idx)
        finally:
            self._syncing = False

    def _on_combo_changed(self, aisle):
        """드롭다운에서 고르면 그 통로만 바로 배치한다. 주행 중에는 바꿀 수 없다 (원래 값으로 되돌림)."""
        if self._syncing:
            return
        idx = self._combo[aisle].model.get_item_value_model().as_int
        if idx == self._placed[aisle]:
            return
        if self._busy:
            self._set_combo(aisle, self._placed[aisle])
            self._log("error", "주행 중에는 장애물을 바꿀 수 없습니다")
            return
        self._placed[aisle] = idx
        self._m["incidents"].set_aisle(aisle, STATES[idx])
        self._log("info", f"통로{self._m['amr_scene'].aisle_no(aisle)}: {STATE_KO[STATES[idx]]}")

    def _on_place(self):
        """드롭다운 값을 모두 장면에 배치한다 (무작위 버튼용)."""
        st = self._states_from_ui()
        for a, s_ in st.items():
            self._m["incidents"].set_aisle(a, s_)
            self._placed[a] = STATES.index(s_)
        self._log("info", "배치: " + ", ".join(f"통로{self._m['amr_scene'].aisle_no(a)}={STATE_KO[s_]}" for a, s_ in st.items()))

    def _on_clear(self):
        if self._busy:
            self._log("error", "주행 중에는 장애물을 바꿀 수 없습니다")
            return
        for a in AISLES:
            self._set_combo(a, 0)
            self._placed[a] = 0
        self._m["incidents"].clear_all()
        self._log("info", "모든 통로를 비웠습니다")

    def _on_random(self):
        import random
        if self._busy:
            self._log("error", "주행 중에는 장애물을 바꿀 수 없습니다")
            return
        for a in AISLES:
            self._set_combo(a, random.randrange(len(STATES)))
        self._on_place()

    def _on_stop(self):
        if not self._busy or self._stopping:
            return
        self._stopping = True
        self._update_run_buttons()
        self._stop = True
        nav = getattr(builtins, "_aisle_nav", None) or self._nav     # 지금 달리는 쪽에 직접 (예전 값이 남아 있으면 정지가 새 주행에 안 닿았다)
        if nav:
            nav.stop_requested = True
        self._log("info", "정지 요청")

    def _on_speed_changed(self):
        sc = self._m["amr_scene"]
        v = sc.SIM_SPEEDS[self._speed_combo.model.get_item_value_model().as_int]
        if sc.set_sim_speed(v) == "pending":
            self._log("info", f"배속 {v:g}배: 주행 중에는 바꿀 수 없어 다음 주행 시작 때 적용합니다")
        else:
            self._log("info", f"배속: {v:g}배")

    def _rules_text(self):
        """'박스 막힘, 고깔 통과' 처럼 지금의 장애물별 판단을 한 줄로."""
        inc = self._m["incidents"]
        return ", ".join(f"{inc.OBSTACLES[k]['name']} {'막힘' if inc.get_rule(k) == 'block' else '통과'}" for k in inc.rule_keys())

    def _refresh_rule_buttons(self):
        inc = self._m["incidents"]
        for k, btn in self._rule_btns.items():
            blocked = inc.get_rule(k) == "block"
            btn.text = f"{inc.OBSTACLES[k]['name']}: {'막힘' if blocked else '통과'}"
            btn.style = {"background_color": 0xFF4A4A9A if blocked else 0xFF4C8C3A, "color": 0xFFFFFFFF}

    def _sync_prompt_field(self):
        """aisle_judge 의 프롬프트 글을 칸에 다시 채운다 (버튼·초기화처럼 코드가 글을 바꿨을 때)."""
        self._syncing_prompt = True
        try:
            self._prompt_field.model.set_value(self._m["aisle_judge"].PROMPT_TEXT)
        finally:
            self._syncing_prompt = False
        self._on_prompt_edited()

    def _on_rule_clicked(self, key):
        """장애물 하나의 판단(막힘 ↔ 통과)을 바꾼다: 프롬프트의 그 줄과 정답이 함께 바뀐다."""
        if self._busy:
            return
        inc, aj = self._m["incidents"], self._m["aisle_judge"]
        aj.set_prompt_text(self._prompt_field.model.get_value_as_string())          # 직접 고친 글을 먼저 반영
        new = "pass" if inc.get_rule(key) == "block" else "block"
        found = aj.set_rule(key, new)
        self._sync_prompt_field()
        self._refresh_rule_buttons()
        name = inc.OBSTACLES[key]["name"]
        self._log("info", f"{name}: Cosmos 가 {'통과시키도록' if new == 'pass' else '막힘으로 보도록'} 프롬프트와 정답을 바꿨습니다")
        if not found:
            self._log("error", f"프롬프트에서 {name} 줄을 찾지 못해(직접 고친 듯) 새 줄을 끼워 넣었습니다. 기존 줄은 직접 지워 주세요")

    def _on_prompt_edited(self):
        """프롬프트 칸의 글이 바뀔 때: 글자 수 제한을 지키고 aisle_judge 에 알린다 (다음 판단부터 적용)."""
        if getattr(self, "_syncing_prompt", False):
            return
        aj = self._m["aisle_judge"]
        text = self._prompt_field.model.get_value_as_string()
        over = len(text) > aj.PROMPT_MAX_CHARS
        if over:                                                   # 제한을 넘으면 잘라 낸다
            text = text[:aj.PROMPT_MAX_CHARS]
            self._syncing_prompt = True
            try:
                self._prompt_field.model.set_value(text)
            finally:
                self._syncing_prompt = False
        aj.set_prompt_text(text)
        n = len(text)
        self._prompt_count.text = f"{n:,} / {aj.PROMPT_MAX_CHARS:,}자" + ("  (제한을 넘는 글은 잘렸습니다)" if over else "")
        self._prompt_count.style = {"color": RED if over or n >= aj.PROMPT_MAX_CHARS else GRAY, "font_size": 12}

    def _update_prompt_ui(self):
        """Cosmos 판단 사용이 OFF 면 프롬프트 영역을 숨기고, ON 이면 보여 준다. 주행·시험 중에는 보이되 회색 막으로 잠근다."""
        if not hasattr(self, "_prompt_body"):
            return
        self._prompt_frame.visible = bool(self._cosmos_state)
        for name in ("_question_frame", "_quiet_row"):
            if hasattr(self, name):
                getattr(self, name).visible = bool(self._cosmos_state)
        msg = "주행 중에는 바꿀 수 없습니다" if self._busy else ""
        self._prompt_body.enabled = not msg
        self._prompt_cover.visible = bool(msg)
        self._prompt_cover_lbl.text = msg

    def _update_cosmos_ui(self):
        on = self._cosmos_state
        self._cosmos_btn.text = "●  ON" if on else "○  OFF"
        # 글자 둘레 여백을 위아래·좌우 모두 COSMOS_BTN_M 으로: 버튼 크기 = 글자 크기 + 2 × 여백
        # (글자 크기는 font_size 18 에서 측정: "●  ON" 42.0 x 20.7, "○  OFF" 47.3 x 20.7)
        self._cosmos_btn.width = ui.Pixel(round((42.0 if on else 47.3) + 2 * COSMOS_BTN_M))
        self._cosmos_btn.style = {"background_color": 0xFF3A9E2E if on else 0xFF5A5A5A,
                                  "color": 0xFFFFFFFF, "font_size": 18, "border_radius": COSMOS_BTN_H / 2}
        self._cosmos_lbl.style = {"font_size": 18, "color": 0xFFFFFFFF if on else GRAY}
        self._update_prompt_ui()

    def _on_cosmos_toggle(self):
        if self._busy:                               # 주행 중에 바꿔도 이미 시작한 주행에는 적용되지 않으므로 막는다
            self._log("error", "주행 중에는 Cosmos 를 켜고 끌 수 없습니다")
            return
        self._cosmos_state = not self._cosmos_state
        self._update_cosmos_ui()
        self._log("info", "Cosmos 판단 " + ("ON: 통로 입구마다 판단" if self._cosmos_state else "OFF: 라이다만으로 주행"))

    def _on_workers_toggle(self):
        want = self._workers_check.model.get_value_as_bool()
        if want == self._workers_state:
            return
        if self._busy:                               # 이미 시작한 주행에는 적용되지 않으므로 원래 값으로 되돌린다
            self._workers_check.model.set_value(self._workers_state)
            self._log("error", "주행 중에는 작업자를 켜고 끌 수 없습니다")
            return
        self._workers_state = want
        self._log("info", "작업자 " + ("ON: 다음 주행부터 3명이 걸어 다닙니다" if self._workers_state else "OFF: 작업자 없이 주행"))

    def _goal(self):
        return self._goal_keys[self._goal_radio.model.get_value_as_int()]

    def _on_goal_changed(self):
        """목적지를 바꾸면 네온 표시도 그 자리로 옮긴다. 주행 중에는 바꿀 수 없다 (표시만 옮겨지고 로봇은 예전 목적지로 가므로)."""
        idx = self._goal_radio.model.get_value_as_int()
        if idx == self._goal_idx:
            return
        if self._busy:
            self._goal_radio.model.set_value(self._goal_idx)          # 원래 선택으로 되돌린다
            self._log("error", "주행 중에는 목적지를 바꿀 수 없습니다")
            return
        self._goal_idx = idx
        g = self._goal()
        try:
            self._m["amr_scene"].move_goal_marker(g)
        except Exception as e:
            self._log("error", f"목적지 표시 이동 실패: {e}")
            return
        self._log("info", f"목적지: {self._m['amr_scene'].GOALS[g][0]}번")

    def _on_drive(self):
        sc_speeds = self._m["amr_scene"].SIM_SPEEDS
        st = self._states_from_ui()
        rd = self._m["run_aisle_demo"]
        mode, mode_ko = ("lidar", "라이다 + Cosmos") if self._cosmos_state else ("lidar_only", "라이다만")
        goal = self._goal()
        goal_ko = self._m["amr_scene"].GOALS[goal][0]

        def on_image(aisle, rgb):
            self._show_image(rgb, f"통로{self._m['amr_scene'].aisle_no(aisle)} 입구")

        def on_judgement(row):
            pass                                    # 결과 표시는 아래 judge 래핑에서

        async def go():
            self._driving = True
            self._update_run_buttons()
            # 메뉴에 고른 배속을 주행 시작 전에 확실히 적용한다 (타임라인을 멈춘 뒤 몇 프레임 기다려야 적용된다).
            # 전에는 도착 뒤 Play 가 남은 채 배속을 바꿔 '대기' 로만 남고, 시작할 때 prepare() 가 stop() 직후 바로
            # 적용하려다 실패해서 메뉴는 5배인데 실제로는 1배로 달렸다 (측정: 프레임당 16.7 ms = 1배)
            await self._ensure_stopped()
            speed = sc_speeds[self._speed_combo.model.get_item_value_model().as_int]
            self._m["amr_scene"].set_sim_speed(speed)
            self._log("info", f"배속 {speed:g}배로 주행")
            if self._cosmos_state:
                self._log("info", f"Cosmos 프롬프트 {len(self._m['aisle_judge'].PROMPT_TEXT)}자 · 장애물별 판단: {self._rules_text()}")
            # 배치 → 주행. 판단 결과를 화면에 그리려고 judge 를 잠깐 감싼다.
            aj = self._m["aisle_judge"]
            original = aj.judge
            truth = {a: st[a] for a in AISLES}
            loop = asyncio.get_event_loop()          # judge 는 작업 스레드에서 돌므로 메인 루프를 미리 잡아 둔다

            def judge_and_show(b64, aisle, **kw):
                res = original(b64, aisle, **kw)
                eff = (getattr(builtins, "_aisle_demo", None) or {}).get("eff_state", {}).get(aisle, truth[aisle])
                loop.call_soon_threadsafe(self._show_result, aisle, res, eff)
                return res
            aj.judge = judge_and_show
            try:
                builtins._aisle_nav = None
                self._log("info", f"주행 시작 · {mode_ko} · 목적지 {goal_ko}번")
                self._m["amr_scene"].move_goal_marker(goal)
                save_data = self._save_check.model.get_value_as_bool()          # 데이터 저장 체크 (주행 시작 때 값)
                task = asyncio.ensure_future(rd.run(st, goal=goal, on_event=self._log, on_judgement=on_judgement,
                                                    on_image=on_image, mode=mode,
                                                    dynamic=self._workers_state, save_data=save_data,
                                                    on_block=self._show_lidar_block))
                # 주행 중 상태 표시 (라이다가 본 앞 거리)
                while not task.done():
                    await asyncio.sleep(0.3)
                    self._nav = getattr(builtins, "_aisle_nav", None)
                    if self._stop and self._nav is not None:
                        self._nav.stop_requested = True      # 정지는 한 번 누르면 끝까지 유지: 시작 직후 눌러도(주행이 시작 때 값을 지우므로) 다시 건다
                    ahead = getattr(self._nav, "min_ahead", None)
                    if ahead is not None:
                        self._busy_lbl.text = (f"주행 중 · {mode_ko} · 라이다 앞 "
                                               f"{'물체 없음' if ahead == float('inf') else f'{ahead:.1f} m'}")
                status = task.result()
            finally:
                aj.judge = original
                self._driving = False
            if status.get("error"):
                self._log("error", status["error"].strip().splitlines()[-1])
            elif not status.get("ok") and not self._stop:
                self._log("stuck", "목적지에 도착하지 못했습니다")
            if status.get("data_dir") and os.path.isdir(status["data_dir"]):      # 데이터 저장이 켜져 있었다 → '사진 폴더 열기' 가 이 폴더를 연다
                self._last_acc_folder = status["data_dir"]
                self._log("info", f"데이터 저장: {os.path.relpath(status['data_dir'], PROJECT)} "
                                  f"(사진 {len(status.get('judgements', []))}장, 박스 라벨·Cosmos 응답 포함)")
        self._run(go, f"주행 중 · {mode_ko}")

    def _on_open_folder(self):
        """저장된 사진·라벨·결과 표 폴더를 탐색기로 연다 (마지막으로 저장한 폴더: 정확도 시험 또는 데이터 저장 주행, 없으면 datasets 폴더)."""
        folder = getattr(self, "_last_acc_folder", None)
        if not folder or not os.path.isdir(folder):
            folder = os.path.join(PROJECT, "datasets")
        if not os.path.isdir(folder):
            self._log("error", "아직 만든 사진 폴더가 없습니다 (정확도 시험이나 '데이터 저장' 주행을 먼저 실행하세요)")
            return
        try:
            os.startfile(folder)
            self._log("info", f"폴더 열기: {os.path.relpath(folder, PROJECT)}")
        except Exception as ex:
            self._log("error", f"폴더를 열지 못했습니다: {ex}")

    def _on_accuracy(self):
        if getattr(self, "_accuracy", False):                # 시험 중에 다시 누르면 중지
            if not self._stopping:
                self._stopping = True
                self._stop = True
                self._update_run_buttons()
                self._log("info", "정확도 시험 중지 요청 (지금 판단 중인 것까지 끝나면 멈춥니다)")
            return

        async def go():
            self._accuracy = True
            self._update_run_buttons()
            try:
                await body()
            finally:
                self._accuracy = False

        async def body():
            # 예전 정확도 시험은 멈춘 상태에서 바로 찍어 이전 화면·안전 영역 표시가 찍힌 사진이 섞였다
            # (12장 중 최소 2장). 이제 data_gen (Replicator 로 무작위 사진 생성 → 새로고침 후 촬영 → Cosmos 판단) 을 쓴다.
            dg, aj = self._m["data_gen"], self._m["aisle_judge"]
            self._reset_score()
            key = "custom"
            self._log("info", f"정확도 시험: 무작위 사진 {ACC_N}장을 만들어 Cosmos 에 보냅니다 "
                              f"(프롬프트 {len(aj.PROMPT_TEXT)}자 · 장애물별 판단: {self._rules_text()})")

            def on_gen(i, n):
                self._busy_lbl.text = f"정확도 시험 · 사진 만드는 중 {i}/{n}"

            def on_judge(k, i, n):
                self._busy_lbl.text = f"정확도 시험 · Cosmos 판단 중 {i}/{n}"

            folder = await dg.generate(n=ACC_N, seed=ACC_SEED, progress=on_gen, should_stop=lambda: self._stop)
            self._last_acc_folder = folder
            if self._stop:
                self._log("info", "정확도 시험 중단 (사진 만들기)")
                return
            summary = await dg.judge_folder(folder, progress=on_judge, should_stop=lambda: self._stop)
            sm = summary.get(key)
            if not sm:
                self._log("info", "정확도 시험 중단")
                return
            self._score_lbl.text = f"채점: {sm['correct']} / {sm['judged']}"
            rel = os.path.relpath(folder, PROJECT)
            self._log("arrived" if not sm["danger"] else "blocked",
                      f"정확도 {sm['correct']}/{sm['judged']} (오류 {sm['errors']}건 제외) | 틀린 것: {', '.join(sm['wrong']) or '없음'}"
                      f" | 막힌 통로를 통과로 본 것(위험): {sm['danger']} | 평균 응답 {sm['avg_sec']}초")
            self._log("info", f"사진·라벨·결과 표: {rel}")
        self._run(go, "정확도 시험 중")
