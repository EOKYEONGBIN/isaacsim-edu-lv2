import omni.ui as ui

WINDOW_TITLE = "AZ Edu Lv2"

DEFAULT_WIDTH = 320
DEFAULT_HEIGHT = 460
BUTTON_HEIGHT = 44
BUTTON_PADDING = 14  # left inset before the "Lesson N" column
LESSON_LABEL_WIDTH = 74  # fixed column width so every subtitle starts at the same x


def _rgb(hex_str, alpha=0xFF):
    """Convert an "RRGGBB" hex string to omni.ui's packed 0xAABBGGRR color int."""
    r, g, b = int(hex_str[0:2], 16), int(hex_str[2:4], 16), int(hex_str[4:6], 16)
    return (alpha << 24) | (b << 16) | (g << 8) | r


COLOR_TEXT_PRIMARY = _rgb("EDEDEA")
COLOR_TEXT_SECONDARY = _rgb("9A9A94")
COLOR_BORDER = _rgb("38382F")

COLOR_BRAND_GREEN = _rgb("76B343")  # AZWELL logo green -- "Lesson N" tag color + hover border glow
COLOR_CARD_BG = _rgb("2A2A25")
COLOR_CARD_BG_HOVER = _rgb("323229")
COLOR_DISABLED_BG = _rgb("242420")
COLOR_DISABLED_TEXT = _rgb("5C5C57")

# Lesson 1 = the forklift driving lesson (a copy of Lv1's Lesson 3), Lesson 2 = the forklift AI practice.
LESSONS = [
    {"key": "lesson1", "title": "Lesson 1", "subtitle": "지게차 시뮬레이션", "enabled": True},
    {"key": "lesson2", "title": "Lesson 2", "subtitle": "지게차 AI", "enabled": True},
]


class EduLv2Window(ui.Window):
    def __init__(
        self,
        visibility_changed_fn=None,
        button1_clicked_fn=None,
        button2_clicked_fn=None,
        **kwargs,
    ):
        kwargs.setdefault("width", DEFAULT_WIDTH)
        kwargs.setdefault("height", DEFAULT_HEIGHT)
        super().__init__(WINDOW_TITLE, **kwargs)
        # Docking (dock_in_window, split beside Stage) happens from
        # extension.py instead -- see its own comment for why.
        self._visibility_changed_fn = visibility_changed_fn
        self._button1_clicked_fn = button1_clicked_fn
        self._button2_clicked_fn = button2_clicked_fn
        self.set_visibility_changed_fn(self._on_visibility_changed)
        self.frame.set_build_fn(self._build_fn)

    def _build_fn(self):
        # spacing=0 + explicit Spacers, height=0 on every Label -- omni.ui
        # gives unset-height widgets a default stretch factor, which is what
        # made the panel grow ugly gaps when the window was resized taller.
        # A single trailing Spacer (no height) collects any leftover room at
        # the bottom instead of splitting it between every row.
        with self.frame:
            with ui.VStack(spacing=0, style={"margin": 4}):
                ui.Label(
                    "Isaac Sim 교육 Lv.2",
                    height=0,
                    style={"font_size": 18, "color": COLOR_TEXT_PRIMARY, "margin": 0, "margin_width": 0},
                )
                ui.Spacer(height=2.5)
                ui.Label(
                    "학습할 레슨을 선택하세요",
                    height=0,
                    style={"font_size": 12, "color": COLOR_TEXT_SECONDARY, "margin": 0, "margin_width": 0},
                )
                ui.Spacer(height=3)
                ui.Line(height=1, style={"color": COLOR_BORDER})
                ui.Spacer(height=3)

                for i, lesson in enumerate(LESSONS):
                    if i > 0:
                        ui.Spacer(height=3)
                    self._build_lesson_button(lesson)
                ui.Spacer()

    def _build_lesson_button(self, lesson):
        # ui.Button always centers its own label and can't hold more than one
        # Label, so a real Button can't give "Lesson N" and the subtitle
        # separate, column-aligned widths. Instead this builds the row by
        # hand -- a background Rectangle plus a Label row on top -- and wires
        # up clicking/hovering directly on the outer ZStack.
        enabled = lesson["enabled"]
        card_bg, card_hover_bg = COLOR_CARD_BG, COLOR_CARD_BG_HOVER
        border, hover_border = COLOR_BORDER, COLOR_BRAND_GREEN
        title_color, subtitle_color = COLOR_BRAND_GREEN, COLOR_TEXT_PRIMARY
        if not enabled:
            card_bg = card_hover_bg = COLOR_DISABLED_BG
            border = hover_border = COLOR_BORDER
            title_color = subtitle_color = COLOR_DISABLED_TEXT

        def _card_style(bg, border_color):
            return {
                "Rectangle": {
                    "background_color": bg,
                    "border_width": 1,
                    "border_color": border_color,
                    "border_radius": 10,
                }
            }

        with ui.ZStack(height=BUTTON_HEIGHT) as row:
            card = ui.Rectangle(style=_card_style(card_bg, border))
            with ui.HStack(spacing=6):
                ui.Spacer(width=BUTTON_PADDING)
                ui.Label(
                    lesson["title"],
                    width=LESSON_LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    style={"color": title_color, "font_size": 14},
                )
                ui.Label(
                    "·",
                    width=14,
                    alignment=ui.Alignment.CENTER,
                    style={"color": COLOR_TEXT_SECONDARY, "font_size": 14},
                )
                ui.Label(
                    lesson["subtitle"],
                    alignment=ui.Alignment.LEFT_CENTER,
                    style={"color": subtitle_color, "font_size": 14},
                )

            if enabled:
                def _on_hovered(hovered, card=card):
                    card.style = _card_style(card_hover_bg if hovered else card_bg, hover_border if hovered else border)

                row.set_mouse_hovered_fn(_on_hovered)
                row.set_mouse_pressed_fn(
                    lambda x, y, button, modifier, k=lesson["key"]: self._on_button_clicked(k) if button == 0 else None
                )

    def _on_button_clicked(self, key):
        if key == "lesson1" and self._button1_clicked_fn:
            self._button1_clicked_fn()
        elif key == "lesson2" and self._button2_clicked_fn:
            self._button2_clicked_fn()

    def _on_visibility_changed(self, visible):
        if self._visibility_changed_fn:
            self._visibility_changed_fn(visible)

    def destroy(self):
        self._visibility_changed_fn = None
        self._button1_clicked_fn = None
        self._button2_clicked_fn = None
        super().destroy()
