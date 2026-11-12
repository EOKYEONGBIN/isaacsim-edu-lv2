import omni.ui as ui

WINDOW_TITLE = "AZ Edu Lv2 - Lesson 1"
PANEL_TITLE = "지게차 시뮬레이션"

DEFAULT_WIDTH = 300
DEFAULT_HEIGHT = 215


def _rgb(hex_str, alpha=0xFF):
    r, g, b = int(hex_str[0:2], 16), int(hex_str[2:4], 16), int(hex_str[4:6], 16)
    return (alpha << 24) | (b << 16) | (g << 8) | r


COLOR_TEXT_PRIMARY = _rgb("EDEDEA")
COLOR_TEXT_SECONDARY = _rgb("9A9A94")
COLOR_BORDER = _rgb("38382F")
COLOR_CARD_BG = _rgb("2A2A25")
COLOR_SECONDARY_BG = _rgb("343430")
COLOR_SECONDARY_BG_HOVER = _rgb("40403A")
COLOR_DISABLED_BG = _rgb("2B2B27")

COLOR_BRAND_GREEN = _rgb("76B343")
COLOR_BRAND_GREEN_HOVER = _rgb("8BC65C")
COLOR_BRAND_GREEN_TEXT = _rgb("14150F")


class ForkliftSimWindow(ui.Window):
    def __init__(self, reset_clicked_fn=None, start_clicked_fn=None, stop_clicked_fn=None, **kwargs):
        kwargs.setdefault("width", DEFAULT_WIDTH)
        kwargs.setdefault("height", DEFAULT_HEIGHT)
        super().__init__(WINDOW_TITLE, **kwargs)
        self._reset_clicked_fn = reset_clicked_fn
        self._start_clicked_fn = start_clicked_fn
        self._stop_clicked_fn = stop_clicked_fn
        self._reset_button = None
        self._running = False
        self.frame.set_build_fn(self._build_fn)

    def _build_fn(self):
        with self.frame:
            with ui.VStack(spacing=0, style={"margin": 4}):
                ui.Label(
                    PANEL_TITLE,
                    height=0,
                    style={"font_size": 16, "color": COLOR_TEXT_PRIMARY, "margin": 0, "margin_width": 0},
                )
                ui.Spacer(height=3)
                ui.Line(height=1, style={"color": COLOR_BORDER})
                ui.Spacer(height=3)

                self._build_control_card()
                ui.Spacer()

    def _build_control_card(self):
        with ui.ZStack(height=0):
            ui.Rectangle(style={"background_color": COLOR_CARD_BG, "border_radius": 10})
            with ui.VStack(spacing=0, style={"margin": 4}):
                ui.Button(
                    "시작",
                    clicked_fn=self._on_start_clicked,
                    height=36,
                    style={
                        "Button": {"background_color": COLOR_BRAND_GREEN, "border_width": 0, "border_radius": 8},
                        "Button:hovered": {"background_color": COLOR_BRAND_GREEN_HOVER},
                        "Button.Label": {"color": COLOR_BRAND_GREEN_TEXT},
                    },
                )
                ui.Spacer(height=4)
                ui.Button(
                    "정지",
                    clicked_fn=self._on_stop_clicked,
                    height=36,
                    style={
                        "Button": {
                            "background_color": COLOR_SECONDARY_BG,
                            "border_width": 1,
                            "border_color": COLOR_SECONDARY_BG,
                            "border_radius": 8,
                        },
                        "Button:hovered": {"background_color": COLOR_SECONDARY_BG_HOVER, "border_color": COLOR_BRAND_GREEN},
                        "Button.Label": {"color": COLOR_TEXT_PRIMARY},
                    },
                )
                ui.Spacer(height=4)
                self._reset_button = ui.Button(
                    "초기화",
                    clicked_fn=self._on_reset_clicked,
                    height=36,
                    style={
                        "Button": {
                            "background_color": COLOR_SECONDARY_BG,
                            "border_width": 1,
                            "border_color": COLOR_SECONDARY_BG,
                            "border_radius": 8,
                        },
                        "Button:hovered": {"background_color": COLOR_SECONDARY_BG_HOVER, "border_color": COLOR_BRAND_GREEN},
                        "Button:disabled": {"background_color": COLOR_DISABLED_BG, "border_color": COLOR_DISABLED_BG},
                        "Button.Label": {"color": COLOR_TEXT_PRIMARY},
                        "Button.Label:disabled": {"color": COLOR_TEXT_SECONDARY},
                    },
                )
                self._reset_button.enabled = not self._running

    def set_running(self, running):
        self._running = running
        if self._reset_button:
            self._reset_button.enabled = not running

    def _on_start_clicked(self):
        if self._start_clicked_fn:
            self._start_clicked_fn()

    def _on_stop_clicked(self):
        if self._stop_clicked_fn:
            self._stop_clicked_fn()

    def _on_reset_clicked(self):
        if self._reset_clicked_fn:
            self._reset_clicked_fn()

    def destroy(self):
        self._reset_clicked_fn = None
        self._start_clicked_fn = None
        self._stop_clicked_fn = None
        self._reset_button = None
        super().destroy()
