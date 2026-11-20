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
LESSON1_MODULE_NAME = "Isaacsim_Edu_Lv2_Lesson1"  # 지게차 시뮬레이션 (copy of Lv1's Lesson 3)
LESSON2_MODULE_NAME = "Isaacsim_Edu_ForkliftAI"  # 지게차 AI (runs on Lesson 1's forklift controls)
ALL_LESSON_MODULES = [
    LESSON1_MODULE_NAME,
    LESSON2_MODULE_NAME,
]
# Lv1's lessons are turned off too when a Lv2 lesson starts (only one lesson scene open at a time; Lv1's
# Lesson 3 registers the same forklift node types as Lesson 1 here).
LV1_LESSON_MODULES = [
    "Isaacsim_Edu_Lv1_Lesson1",
    "Isaacsim_Edu_Lv1_Lesson2",
    "Isaacsim_Edu_Lv1_Lesson3",
    "Isaacsim_Edu_Lv1_Lesson4",
    "Isaacsim_Edu_Lv1_Lesson5",
]
# With the Lv1 launcher on as well, this window joins its column as a tab (see _dock_with_lv1).
LV1_LAUNCHER_MODULE = "Isaacsim_Edu_Lv1"
LV1_WINDOW_TITLE = "AZ Edu Lv1"

# Persistent (survives across Isaac Sim restarts) setting for whether the
# panel should be open. Defaults to visible so trainers don't have to
# reopen it via the Azwell menu every launch; if a user explicitly closes
# it, that choice is remembered and it stays closed on the next launch too.
SETTING_WINDOW_VISIBLE = "/persistent/exts/Isaacsim_Edu_Lv2/window_visible"

# Which lesson module is currently active, written every time _activate_lesson
# runs (and mirrored by each lesson's own on_startup -- see their extension.py
# files) so a crashed-and-relaunched Isaac Sim can pick back up in the same
# lesson. NOT the same thing as Kit's own "remember enabled extensions"
# mechanism: that list (visible in user.config.json under exts/enabled) is
# only updated when an extension is toggled through the Extensions *window*
# checkbox -- confirmed live that calling set_extension_enabled_immediate()
# from code, which is how every lesson button here works, never touches it.
# A plain /persistent/ setting, by contrast, is the same proven mechanism
# already used for SETTING_WINDOW_VISIBLE above, written synchronously the
# moment a lesson activates rather than deferred to a clean app exit.
SETTING_ACTIVE_LESSON = "/persistent/exts/Isaacsim_Edu_Lv2/active_lesson"


# ui.Window has no built-in "dock a new full-height column beside an
# existing stack of windows" call: dock_in_window/deferred_dock_in only
# split the target window's own immediate slot, so docking beside "Stage"
# alone (confirmed live) leaves Property spanning underneath both the old
# and new columns instead of keeping its own width -- Stage and Property
# each need to keep their existing top/bottom arrangement untouched, with
# the new window added as a sibling of that whole arrangement, one level up.
# ui.Workspace.dump_workspace()/restore_workspace() (undocumented beyond
# their docstrings) is the only way to reach that: it round-trips the whole
# dock tree as plain dicts, which can be surgically edited before writing
# it back. Confirmed live this is easy to get wrong -- a hand-written leaf
# entry missing fields real dump_workspace() output always has (dock_id,
# width/height, position_x/y, visible) left Stage AND Property undocked
# entirely. The functions below only ever move dicts dump_workspace() itself
# produced, never author new leaf entries from scratch.
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
    """Recursively find any leaf under `node` and return its "width", or
    None. Used to size a freshly-split column to match its sibling rather
    than whatever stale width the moved window last had -- restore_workspace
    derives split proportions from each leaf's declared width, and a stale
    one (e.g. from this same window's last, differently-sized position)
    otherwise makes the split ratio drift wider on every reload (confirmed
    live: repeated reloads without this grew the Lv1 launcher (this code was copied from it) from ~215 to ~546px
    with no user action)."""
    if "title" in node:
        return node.get("width")
    for child in node.get("children", []):
        width = _first_leaf_width(child)
        if width is not None:
            return width
    return None


def _extract_group(dump, title):
    """Find the leaf `title` anywhere in `dump` (a dump_workspace() list),
    remove it, and return the list of leaf dicts to re-dock in its place.
    If it shares a dock slot with sibling tabs, ALL of them are removed and
    returned together, not just `title` alone -- confirmed live this matters
    on a real app launch: a lesson's own window can tab-join this one (via
    its own deferred_dock_in) while this one is still floating, before this
    module's own code gets a chance to dock it anywhere. Extracting only the
    single named leaf left that lesson tab behind, alone, in the vacated
    slot -- a duplicate, orphaned column next to the real one. Floating
    top-level entries (not real tab groups -- e.g. Isaac Sim's various
    hidden utility windows, each its own independent list entry) are never
    swept up this way. None if `title` isn't found at all."""
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
    """True once all of `titles` are already docked together under some
    shared node (deeper than the tree root) -- the same lookup
    _dock_full_height_beside does internally, without moving anything.
    Reported live: on a cold app boot (unlike a same-session extension
    reload) Stage and Property can each already exist as windows
    (ui.Workspace.get_window returns non-None) slightly before whatever
    docks them together relative to each other has actually run, since each
    core window's own extension does that on its own schedule -- polling
    just for existence let _dock_beside_stage run too early and fall back
    to the plainer, uglier split. This is the correct thing to poll for."""
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
    return common_depth > 0  # depth 0 would be the tree root itself


def _dock_full_height_beside(new_window_title, *existing_titles):
    """Dock the window titled `new_window_title` as a new column spanning
    the full height of the smallest existing dock group that already
    contains all of `existing_titles` (e.g. "Stage" and "Property"), leaving
    that group's own internal arrangement untouched. Returns False (no
    change made) if that group can't be found in the expected shape or the
    new window doesn't exist yet, so the caller can fall back to a plainer,
    always-safe docking call instead."""
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
    if common is None or common_depth == 0:  # depth 0 would be the tree root itself
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

    # dock_id on the two new nodes is unused by Kit here (it reassigns real
    # ids on reconstruction regardless -- confirmed live) but every node in
    # a real dump_workspace() has one, so this stays consistent with that
    # shape rather than relying on it being optional.
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
    """Shrink `leaf_title`'s whole dock column to `factor` of its current
    width, handing the freed space to whatever lies OUTSIDE the two-way
    split it shares with `protected_sibling_title` (which keeps its exact
    current width). E.g. shrinking Stage's column while protecting AZ Edu
    Lv2's hands the freed space to Viewport -- Viewport lives outside that
    shared split entirely -- rather than growing AZ Edu Lv2 instead, which
    is what naively resizing just the one column would otherwise do
    (confirmed live: set_dock_id_width scales both sides of a split
    proportionally, it doesn't just take space from one and leave the
    other untouched)."""
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
    """Shrink `leaf_title`'s own dock row to `factor` of its current
    height, handing the freed space to whatever it's split against (e.g.
    Content's row shrinking hands space to Viewport above it)."""
    leaf = ui.Workspace.get_window(leaf_title)
    if leaf is None:
        return
    dock_id = leaf.dock_id
    ui.Workspace.set_dock_id_height(dock_id, ui.Workspace.get_dock_id_height(dock_id) * factor)


def _apply_size_preferences():
    """Trainer-requested proportions layered on top of Isaac Sim's own
    default layout: Stage/Property narrower and Viewport wider by the same
    amount, Content shorter (Console/Robotics Examples share its tab so
    this affects them equally too) and Viewport taller by the same
    amount."""
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

        # Resume whichever lesson was active last time, e.g. after a crash --
        # only if it's still a real, known lesson module (a stale/edited-out
        # value from an older version shouldn't error out).
        last_lesson = settings.get(SETTING_ACTIVE_LESSON)
        if last_lesson in ALL_LESSON_MODULES:
            self._activate_lesson(last_lesson)

    def on_shutdown(self):
        # Set before touching self._window.visible below -- that assignment
        # also fires _on_window_visibility_changed, and a shutdown-triggered
        # hide must NOT overwrite the user's actual last-chosen preference.
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
            # .destroy() alone only "removes callbacks and circular
            # references" (per omni.ui's own docstring) -- it does not hide
            # the window, so without this the old window stays visibly on
            # screen as an inert orphan every time this extension reloads.
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
        # Reset to Isaac Sim's own bundled default layout first. Confirmed
        # live: whatever Kit auto-restores a window layout from between
        # launches (mechanism unclear -- not user.config.json, not
        # omni.kit.quicklayout's own manual save file, and not any of this
        # extension's own code) can leave Viewport, Content, and Console
        # badly mis-sized -- e.g. Viewport squeezed to a few pixels tall --
        # depending on how a previous session happened to be left. Loading
        # the bundled default up front guarantees every trainee's launch
        # starts from the same known-good baseline before this window adds
        # its own column, rather than inheriting whatever a prior session
        # (or this extension's own earlier testing) left behind. This
        # doesn't touch "AZ Edu Lv2" or any lesson window's docked position
        # -- the default layout doesn't know about them, so they're simply
        # left wherever they already are (floating at this point) for the
        # docking below to pick up. It DOES hide them, though (confirmed
        # live): loading a layout that doesn't mention a window marks it
        # invisible, same as `QuickLayout.load_file`'s own
        # `keep_windows_open=False` default, and `dock_in_window` silently
        # no-ops on an invisible window -- so without undoing that here,
        # this window (and, transitively, any lesson tab riding along with
        # it) would otherwise stay stuck floating and hidden forever no
        # matter how many times the docking below retries.
        registry = omni.kit.actions.core.get_action_registry()
        default_layout_action = registry.get_action("isaacsim.app.setup", "layout_default")
        if default_layout_action is not None:
            default_layout_action.execute()
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()
            if self._window is None:
                return
            self._window.visible = True

        # Poll for Stage and Property to already be docked together, not
        # just for each to exist -- see _has_common_dock_ancestor's own
        # docstring for the cold-boot race this guards (confirmed live: a
        # same-session extension reload never hits it, since Stage/Property
        # are long since docked together by then, but a fresh app launch
        # can have this run before they are).
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

        # restore_workspace can silently no-op on a window that was itself
        # only just constructed this same tick -- confirmed live: it can
        # report success immediately after this extension's own on_startup
        # runs (the earliest _has_common_dock_ancestor can already be true),
        # yet leave the window floating, undocked, with no further ticks
        # ever fixing it on their own -- but the exact same call succeeds
        # immediately once called again after any window has had a chance
        # to actually render once. One settling frame before the first
        # attempt, then checking the real result (not just the return
        # value) and retrying a few times, turns that into a non-issue
        # instead of permanently settling for the plainer split below.
        await app.next_update_async()
        for _ in range(10):
            if self._window is None:
                return
            _dock_full_height_beside(WINDOW_TITLE, "Stage", "Property")
            if self._window.docked:
                break
            await app.next_update_async()
        else:
            # Stage/Property aren't in the expected default arrangement (a
            # simple two-way split sharing one ancestor) -- confirmed live
            # this can otherwise leave the whole layout corrupted (windows
            # undocked entirely), so fall back to the one safe, well-defined
            # operation instead of guessing further: a plain split beside
            # Stage only. Less pretty (Property ends up spanning under both
            # columns instead of staying its own width) but never breaks.
            if self._window is not None:
                self._window.dock_in_window("Stage", ui.DockPosition.RIGHT, 0.5)

        # Same settling-frame issue as the docking above -- resizing a dock
        # split immediately after it was just created can silently no-op.
        await app.next_update_async()
        _apply_size_preferences()

    async def _reclaim_stray_lesson_tabs(self):
        # A lesson's own deferred_dock_in("AZ Edu Lv2", ...) call can resolve
        # while this window is mid-move (still floating, or freshly restored
        # into its final spot) -- confirmed live this occasionally leaves
        # the lesson panel behind, floating on its own AND marked invisible,
        # instead of tab-joining, even though each side's own docking code
        # is individually correct in isolation. Self-heals for a while after
        # startup: any window whose title still starts with "AZ Edu Lv2 - "
        # but is still a plain floating entry (dump_workspace()'s flat tail,
        # not nested under the tree at all) gets folded back in directly.
        # dock_in_window silently no-ops on an invisible window (confirmed
        # live) -- the lesson's own build_window() already sets visible=True
        # right after construction, so forcing it back on here is just
        # undoing whatever this race left behind, not fighting anything the
        # lesson intended.
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
                        # Re-fetched, not reused: confirmed live that toggling
                        # visible on a just-created window can invalidate the
                        # handle held from before that toggle.
                        stray = ui.Workspace.get_window(title)
                        if stray is not None:
                            stray.dock_in_window(WINDOW_TITLE, ui.DockPosition.SAME, 1.0)
                    except AttributeError:
                        pass  # transient handle state -- retried next frame
            await app.next_update_async()

    def _on_button1_clicked(self):
        self._activate_lesson(LESSON1_MODULE_NAME)

    def _on_button2_clicked(self):
        self._activate_lesson(LESSON2_MODULE_NAME)

    async def _dock_with_lv1(self):
        """The Lv1 launcher resets the layout and docks itself beside Stage as well. With both on, wait for it
        and join its column as a tab instead of resetting the layout a second time (which would hide it).
        False if it never docks (e.g. its window is closed): then the usual routine runs."""
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
        for _ in range(10):  # let Lv1 finish its own resizing first
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
        # Always disable-then-enable the target too, even if it's already on:
        # set_extension_enabled_immediate(..., True) is a no-op when the
        # extension is already enabled, so re-clicking the same lesson's
        # button would otherwise do nothing at all (no stage reload, no
        # state reset) instead of restarting it cleanly.
        manager.set_extension_enabled_immediate(target_module, False)
        if manager.set_extension_enabled_immediate(target_module, True):
            carb.settings.get_settings().set(SETTING_ACTIVE_LESSON, target_module)
        else:
            carb.log_error(f"[Isaacsim_Edu_Lv2] Could not enable '{target_module}'")
