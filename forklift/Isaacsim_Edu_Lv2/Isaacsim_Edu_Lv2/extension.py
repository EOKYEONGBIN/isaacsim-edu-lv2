import asyncio

import carb
import carb.settings
import omni.ext
import omni.kit.actions.core
import omni.kit.app
import omni.kit.menu.utils
import omni.ui as ui

from .window import EduLv2Window, WINDOW_TITLE

MENU_GROUP = "Azwell"
LESSON1_MODULE_NAME = "Isaacsim_Edu_Lv2_Lesson1"  # 지게차 시뮬레이션 (Lv1 Lesson 3 복사본)
LESSON2_MODULE_NAME = "Isaacsim_Edu_ForkliftAI"  # 지게차 AI (Lesson 1의 지게차 조작 기능 사용)
ALL_LESSON_MODULES = [
    LESSON1_MODULE_NAME,
    LESSON2_MODULE_NAME,
]
# Lv2 레슨을 시작하면 Lv1 레슨도 끈다. 레슨 씬은 한 번에 하나만 열고,
# Lv1 Lesson 3은 여기 Lesson 1과 같은 지게차 노드 타입을 등록하기 때문.
LV1_LESSON_MODULES = [
    "Isaacsim_Edu_Lv1_Lesson1",
    "Isaacsim_Edu_Lv1_Lesson2",
    "Isaacsim_Edu_Lv1_Lesson3",
    "Isaacsim_Edu_Lv1_Lesson4",
    "Isaacsim_Edu_Lv1_Lesson5",
]
# Lv1 런처도 켜져 있으면 이 창은 그 열에 탭으로 붙는다 (_dock_with_lv1 참고).
LV1_LAUNCHER_MODULE = "Isaacsim_Edu_Lv1"
LV1_WINDOW_TITLE = "AZ Edu Lv1"

# 패널 표시 여부 (재시작 후에도 유지). 기본값은 표시라 매번 Azwell 메뉴로 열 필요가 없고,
# 사용자가 닫으면 다음 실행에도 닫힌 채로 둔다.
SETTING_WINDOW_VISIBLE = "/persistent/exts/Isaacsim_Edu_Lv2/window_visible"

# 현재 활성 레슨. _activate_lesson과 각 레슨의 on_startup에서 기록해, 비정상 종료 후 재실행하면 같은 레슨으로 복귀한다.
# Kit의 "활성 익스텐션 기억"(`user.config.json`의 exts/enabled)과는 별개다. 그 목록은 Extensions 창 체크박스로
# 토글할 때만 갱신되고, 레슨 버튼이 쓰는 set_extension_enabled_immediate() 호출로는 바뀌지 않는다(실측 확인).
# /persistent/ 설정은 레슨 활성화 시점에 바로 기록되므로 정상 종료가 아니어도 남는다.
SETTING_ACTIVE_LESSON = "/persistent/exts/Isaacsim_Edu_Lv2/active_lesson"


# ui.Window에는 "기존 창 묶음 옆에 전체 높이 열을 새로 도킹"하는 API가 없다. dock_in_window/deferred_dock_in은
# 대상 창의 슬롯만 분할하므로, "Stage" 옆에 도킹하면 Property가 기존·새 열 아래에 걸쳐 버린다(실측 확인).
# Stage/Property의 위아래 배치는 그대로 두고, 새 창은 그 묶음 전체의 형제로 한 단계 위에 넣어야 한다.
# 이를 위해 ui.Workspace.dump_workspace()/restore_workspace()(docstring 외 문서 없음)로 도킹 트리를 dict로 받아
# 수정한 뒤 다시 적용한다. 주의: 직접 만든 leaf 항목에 실제 dump에 있는 필드(dock_id, width/height,
# position_x/y, visible)가 빠지면 Stage와 Property가 통째로 도킹 해제됐다(실측 확인).
# 그래서 아래 함수들은 dump_workspace()가 만든 dict를 옮기기만 하고, leaf를 새로 만들지 않는다.
def _find_ancestor_chain(node, title, chain):
    chain = chain + [node]
    if node.get("title") == title:
        return chain
    for child in node.get("children", []):
        found = _find_ancestor_chain(child, title, chain)
        if found is not None:
            return found
    return None


def _first_leaf_width(node):
    """`node` 아래 아무 leaf의 "width"를 반환 (없으면 None).
    새로 분할한 열의 폭을 형제 열에 맞추는 데 쓴다. restore_workspace는 leaf의 width로 분할 비율을 정하므로,
    이전 위치의 오래된 width가 남으면 리로드할 때마다 열이 넓어진다
    (실측: 이 코드의 원본인 Lv1 런처가 조작 없이 ~215 -> ~546px)."""
    if "title" in node:
        return node.get("width")
    for child in node.get("children", []):
        width = _first_leaf_width(child)
        if width is not None:
            return width
    return None


def _extract_group(dump, title):
    """`dump`(dump_workspace() 리스트)에서 leaf `title`을 찾아 제거하고, 다시 도킹할 leaf dict 목록을 반환.
    같은 슬롯에 탭으로 묶인 형제가 있으면 모두 함께 꺼낸다. 실제 앱 실행 시 이 창이 아직 플로팅일 때
    레슨 창이 deferred_dock_in으로 먼저 탭 결합할 수 있는데, `title`만 꺼내면 레슨 탭이 빈 슬롯에 홀로 남아
    고아 열이 생겼다(실측 확인). 최상위 플로팅 항목(Isaac Sim의 숨은 유틸리티 창 등)은 묶지 않는다.
    `title`이 없으면 None."""
    def _search(container):
        for i, item in enumerate(container):
            if item.get("title") == title:
                return container, i
            if "children" in item:
                found = _search(item["children"])
                if found is not None:
                    return found
        return None

    found = _search(dump)
    if found is None:
        return None
    container, index = found
    if container is dump or len(container) == 1:
        return [container.pop(index)]
    group = list(container)
    container.clear()
    return group


def _has_common_dock_ancestor(*titles):
    """`titles`가 모두 트리 루트가 아닌 공통 노드 아래에 도킹돼 있으면 True.
    _dock_full_height_beside와 같은 검사이며 아무것도 옮기지 않는다.
    콜드 부팅 시(같은 세션 리로드와 달리) Stage/Property 창은 서로 도킹되기 전에 먼저 생길 수 있어,
    존재 여부만 폴링하면 _dock_beside_stage가 너무 일찍 실행돼 단순 분할로 빠졌다. 그래서 이것을 폴링한다."""
    dump = ui.Workspace.dump_workspace()
    root = dump[0]
    chains = [_find_ancestor_chain(root, title, []) for title in titles]
    if any(chain is None for chain in chains):
        return False
    common_depth = -1
    for depth, nodes_at_depth in enumerate(zip(*chains)):
        if all(n is nodes_at_depth[0] for n in nodes_at_depth):
            common_depth = depth
        else:
            break
    return common_depth > 0  # depth 0은 트리 루트


def _dock_full_height_beside(new_window_title, *existing_titles):
    """`new_window_title` 창을 `existing_titles`(예: "Stage", "Property")를 모두 포함하는 가장 작은 도킹 그룹 옆에
    전체 높이 열로 도킹한다. 그룹 내부 배치는 그대로 둔다.
    그룹이 예상 형태가 아니거나 새 창이 아직 없으면 아무것도 바꾸지 않고 False (호출 측이 단순 도킹으로 대체)."""
    dump = ui.Workspace.dump_workspace()
    root = dump[0]

    chains = [_find_ancestor_chain(root, title, []) for title in existing_titles]
    if any(chain is None for chain in chains):
        return False

    common = None
    common_depth = -1
    for depth, nodes_at_depth in enumerate(zip(*chains)):
        if all(n is nodes_at_depth[0] for n in nodes_at_depth):
            common, common_depth = nodes_at_depth[0], depth
        else:
            break
    if common is None or common_depth == 0:  # depth 0은 트리 루트
        return False

    parent = chains[0][common_depth - 1]
    index = next((i for i, c in enumerate(parent["children"]) if c is common), None)
    if index is None:
        return False

    new_leaves = _extract_group(dump, new_window_title)
    if not new_leaves:
        return False

    sibling_width = _first_leaf_width(common)
    if sibling_width is not None:
        for leaf in new_leaves:
            leaf["width"] = sibling_width

    # 새 노드 두 개의 dock_id는 Kit이 재구성 시 다시 매겨 실제로는 안 쓰인다(실측 확인).
    # 실제 dump_workspace()의 모든 노드에 있는 필드라 형태를 맞추려고 넣는다.
    wrapper = {
        "dock_id": common.get("dock_id", 0) + 1,
        "position": common.get("position"),
        "children": [
            {**common, "position": "LEFT"},
            {"dock_id": common.get("dock_id", 0) + 2, "position": "RIGHT", "children": new_leaves},
        ],
    }
    parent["children"][index] = wrapper
    ui.Workspace.restore_workspace(dump, True)
    return True


def _shrink_column_width(leaf_title, protected_sibling_title, factor):
    """`leaf_title`의 열 전체를 현재 폭의 `factor` 배로 줄이고, 남는 공간은 `protected_sibling_title`과 함께 쓰는
    2분할의 바깥쪽에 준다(형제 폭은 유지). 예: AZ Edu Lv2를 보호하며 Stage 열을 줄이면 Viewport가 넓어진다.
    set_dock_id_width는 분할 양쪽을 비례로 조정하므로, 한 열만 리사이즈하면 형제 열이 커진다(실측 확인)."""
    leaf = ui.Workspace.get_window(leaf_title)
    sibling = ui.Workspace.get_window(protected_sibling_title)
    if leaf is None or sibling is None:
        return
    leaf_dock, sibling_dock = leaf.dock_id, sibling.dock_id
    parent_dock = ui.Workspace.get_parent_dock_id(leaf_dock)
    if parent_dock == 0:
        return
    new_leaf_width = ui.Workspace.get_dock_id_width(leaf_dock) * factor
    sibling_width = ui.Workspace.get_dock_id_width(sibling_dock)
    ui.Workspace.set_dock_id_width(parent_dock, new_leaf_width + sibling_width)
    ui.Workspace.set_dock_id_width(sibling_dock, sibling_width)


def _shrink_row_height(leaf_title, factor):
    """`leaf_title`의 행을 현재 높이의 `factor` 배로 줄이고, 남는 공간은 분할 상대에게 준다
    (예: Content를 줄이면 위쪽 Viewport가 커짐)."""
    leaf = ui.Workspace.get_window(leaf_title)
    if leaf is None:
        return
    dock_id = leaf.dock_id
    ui.Workspace.set_dock_id_height(dock_id, ui.Workspace.get_dock_id_height(dock_id) * factor)


def _apply_size_preferences():
    """Isaac Sim 기본 레이아웃 위에 강사 요청 비율을 적용한다.
    Stage/Property는 좁게, Content(Console/Robotics Examples와 같은 탭)는 낮게 하고 그만큼 Viewport를 키운다."""
    _shrink_column_width("Stage", WINDOW_TITLE, 0.5)
    _shrink_row_height("Content", 0.5)


class IsaacsimEduLv2Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._shutting_down = False
        settings = carb.settings.get_settings()
        initial_visible = settings.get(SETTING_WINDOW_VISIBLE)
        if initial_visible is None:
            initial_visible = True

        self._window = EduLv2Window(
            visibility_changed_fn=self._on_window_visibility_changed,
            button1_clicked_fn=self._on_button1_clicked,
            button2_clicked_fn=self._on_button2_clicked,
            visible=initial_visible,
        )
        self._menu_helper = omni.kit.menu.utils.MenuHelperExtension()
        self._menu_helper.menu_startup(
            window_name=WINDOW_TITLE,
            menu_desc=WINDOW_TITLE,
            menu_group=MENU_GROUP,
        )
        self._dock_task = asyncio.ensure_future(self._dock_beside_stage())
        self._reclaim_task = asyncio.ensure_future(self._reclaim_stray_lesson_tabs())

        # 마지막 활성 레슨으로 복귀(예: 비정상 종료 후). 이전 버전에서 남은 값처럼 모르는 모듈이면 무시.
        last_lesson = settings.get(SETTING_ACTIVE_LESSON)
        if last_lesson in ALL_LESSON_MODULES:
            self._activate_lesson(last_lesson)

    def on_shutdown(self):
        # 아래 self._window.visible 변경 전에 설정. 그 대입이 _on_window_visibility_changed를 호출하는데,
        # 종료로 인한 숨김이 사용자의 표시 설정을 덮어쓰면 안 된다.
        self._shutting_down = True

        if self._dock_task:
            self._dock_task.cancel()
            self._dock_task = None
        if self._reclaim_task:
            self._reclaim_task.cancel()
            self._reclaim_task = None
        if self._menu_helper:
            self._menu_helper.menu_shutdown()
            self._menu_helper = None
        if self._window:
            # .destroy()는 콜백/순환 참조만 정리하고 창을 숨기지 않는다(omni.ui docstring).
            # 숨기지 않으면 리로드할 때마다 동작하지 않는 이전 창이 화면에 남는다.
            self._window.visible = False
            self._window.destroy()
            self._window = None

    def _on_window_visibility_changed(self, visible):
        if not self._shutting_down:
            carb.settings.get_settings().set(SETTING_WINDOW_VISIBLE, visible)
        if self._menu_helper:
            self._menu_helper.menu_refresh()

    async def _dock_beside_stage(self):
        if omni.kit.app.get_app().get_extension_manager().is_extension_enabled(LV1_LAUNCHER_MODULE):
            if await self._dock_with_lv1():
                return
        # 먼저 Isaac Sim 기본 레이아웃으로 초기화한다. Kit이 실행 간 레이아웃을 자동 복원하는데
        # (경로 불명: `user.config.json`도, omni.kit.quicklayout 저장 파일도, 이 익스텐션 코드도 아님),
        # 이전 세션 상태에 따라 Viewport/Content/Console 크기가 크게 틀어질 수 있다(예: Viewport 높이 몇 px, 실측 확인).
        # 기본 레이아웃을 먼저 불러 모든 교육생이 같은 상태에서 시작하게 한다.
        # 기본 레이아웃은 "AZ Edu Lv2"와 레슨 창을 모르므로 위치는 그대로(이 시점엔 플로팅) 두지만 숨김 처리한다
        # (실측 확인, `QuickLayout.load_file`의 `keep_windows_open=False` 기본값과 같음). `dock_in_window`는 숨겨진 창에서
        # 아무 동작도 하지 않으므로, 다시 표시하지 않으면 이 창(과 함께 붙은 레슨 탭)이 계속 숨은 채 떠 있게 된다.
        registry = omni.kit.actions.core.get_action_registry()
        default_layout_action = registry.get_action("isaacsim.app.setup", "layout_default")
        if default_layout_action is not None:
            default_layout_action.execute()
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()
            if self._window is None:
                return
            self._window.visible = True

        # Stage/Property가 존재하는지가 아니라 서로 도킹됐는지를 폴링한다. 콜드 부팅 경합 대응
        # (_has_common_dock_ancestor docstring 참고). 같은 세션 리로드에서는 발생하지 않는다(실측 확인).
        app = omni.kit.app.get_app()
        for _ in range(60):
            if self._window is None:
                return
            if _has_common_dock_ancestor("Stage", "Property"):
                break
            await app.next_update_async()
        else:
            carb.log_warn("[Isaacsim_Edu_Lv2] Could not dock beside Stage -- target window never appeared")
            return

        # 같은 틱에 막 생성된 창에는 restore_workspace가 성공을 반환하고도 효과가 없을 수 있다(실측 확인:
        # on_startup 직후 창이 플로팅으로 남고 저절로 고쳐지지 않음). 창이 한 번 렌더링된 뒤 다시 호출하면 바로 성공한다.
        # 그래서 첫 시도 전에 한 프레임 쉬고, 반환값이 아닌 실제 도킹 여부를 확인하며 몇 번 재시도한다.
        await app.next_update_async()
        for _ in range(10):
            if self._window is None:
                return
            _dock_full_height_beside(WINDOW_TITLE, "Stage", "Property")
            if self._window.docked:
                break
            await app.next_update_async()
        else:
            # Stage/Property가 예상한 기본 배치(공통 조상을 둔 2분할)가 아니다. 더 시도하면 레이아웃 전체가
            # 깨질 수 있으므로(실측 확인: 창이 모두 도킹 해제됨) 안전한 Stage 옆 단순 분할로 대체한다.
            # Property가 두 열 아래에 걸쳐 보기엔 덜 좋지만 깨지지는 않는다.
            if self._window is not None:
                self._window.dock_in_window("Stage", ui.DockPosition.RIGHT, 0.5)

        # 위 도킹과 같은 이유: 방금 만든 분할을 바로 리사이즈하면 무시될 수 있다.
        await app.next_update_async()
        _apply_size_preferences()

    async def _reclaim_stray_lesson_tabs(self):
        # 레슨의 deferred_dock_in("AZ Edu Lv2", ...)이 이 창이 이동 중일 때(플로팅 상태 또는 막 복원된 직후) 처리되면,
        # 레슨 패널이 탭으로 붙지 못하고 숨김 상태로 혼자 떠 있는 경우가 가끔 있다(실측 확인. 양쪽 코드는 각각 정상).
        # 그래서 시작 후 일정 시간 동안, 제목이 "AZ Edu Lv2 - "로 시작하는 플로팅 항목(dump_workspace()에서 트리 밖
        # 뒷부분)을 찾아 다시 탭으로 붙인다. dock_in_window는 숨겨진 창에서 무시되므로(실측 확인) 표시를 다시 켠다.
        # 레슨의 build_window()도 생성 직후 visible=True로 하므로 레슨 의도와 충돌하지 않는다.
        app = omni.kit.app.get_app()
        prefix = WINDOW_TITLE + " - "
        for _ in range(180):
            if self._window is None:
                return
            for item in ui.Workspace.dump_workspace()[1:]:
                title = item.get("title")
                if title and title.startswith(prefix):
                    try:
                        stray = ui.Workspace.get_window(title)
                        if stray is not None:
                            stray.visible = True
                        # 다시 가져온다: 막 생성된 창의 visible을 바꾸면 이전 핸들이 무효가 될 수 있다(실측 확인).
                        stray = ui.Workspace.get_window(title)
                        if stray is not None:
                            stray.dock_in_window(WINDOW_TITLE, ui.DockPosition.SAME, 1.0)
                    except AttributeError:
                        pass  # 일시적인 핸들 상태, 다음 프레임에 재시도
            await app.next_update_async()

    def _on_button1_clicked(self):
        self._activate_lesson(LESSON1_MODULE_NAME)

    def _on_button2_clicked(self):
        self._activate_lesson(LESSON2_MODULE_NAME)

    async def _dock_with_lv1(self):
        """Lv1 런처도 레이아웃을 초기화하고 Stage 옆에 도킹한다. 둘 다 켜져 있으면 Lv1 도킹을 기다렸다가 그 열에 탭으로 붙는다
        (레이아웃을 한 번 더 초기화하면 Lv1 창이 숨겨짐). 끝내 도킹되지 않으면(예: 창이 닫힘) False -> 기본 루틴 실행."""
        app = omni.kit.app.get_app()
        for _ in range(300):
            if self._window is None:
                return True
            lv1 = ui.Workspace.get_window(LV1_WINDOW_TITLE)
            if lv1 is not None and lv1.visible and lv1.docked:
                break
            await app.next_update_async()
        else:
            return False
        for _ in range(10):  # Lv1의 크기 조정이 끝나길 기다림
            await app.next_update_async()
        for _ in range(10):
            if self._window is None:
                return True
            self._window.visible = True
            self._window.dock_in_window(LV1_WINDOW_TITLE, ui.DockPosition.SAME, 1.0)
            await app.next_update_async()
            if self._window.docked:
                break
        return True

    def _activate_lesson(self, target_module):
        manager = omni.kit.app.get_app().get_extension_manager()
        for module in ALL_LESSON_MODULES:
            if module != target_module:
                manager.set_extension_enabled_immediate(module, False)
        for module in LV1_LESSON_MODULES:
            if manager.is_extension_enabled(module):
                manager.set_extension_enabled_immediate(module, False)
        # 대상 레슨도 이미 켜져 있든 아니든 껐다가 켠다. 켜진 상태에서 set_extension_enabled_immediate(..., True)는
        # 아무 동작도 하지 않아, 같은 레슨 버튼을 다시 눌러도 재시작(씬 리로드, 상태 초기화)이 되지 않기 때문.
        manager.set_extension_enabled_immediate(target_module, False)
        if manager.set_extension_enabled_immediate(target_module, True):
            carb.settings.get_settings().set(SETTING_ACTIVE_LESSON, target_module)
        else:
            carb.log_error(f"[Isaacsim_Edu_Lv2] Could not enable '{target_module}'")
