from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal
from textual.reactive import reactive

from r2s_gw.widgets.find_dialog import FindDialog

from .log_lines import LogLines


class LogView(Horizontal):
    DEFAULT_CSS = """
    LogLines {
        width: 1fr;
    }
    """

    BINDINGS = [
        Binding("ctrl+l", "toggle('show_line_numbers')", "Line nos.", key_display="^l"),
        Binding("ctrl+f", "show_find_dialog", "Find", key_display="^f"),
        Binding("slash", "show_find_dialog", "Find", key_display="^f", show=False),
    ]

    show_find: reactive[bool] = reactive(False)
    show_line_numbers: reactive[bool] = reactive(False)

    def compose(self) -> ComposeResult:
        yield (
            log_lines := LogLines().data_bind(
                LogView.show_line_numbers, LogView.show_find
            )
        )
        yield FindDialog()

    async def watch_show_find(self, show_find: bool) -> None:
        if not self.is_mounted:
            return
        filter_dialog = self.query_one(FindDialog)
        filter_dialog.set_class(show_find, "visible")
        if show_find:
            filter_dialog.focus_input()
        else:
            self.query_one(LogLines).focus()

    def action_show_find_dialog(self) -> None:
        find_dialog = self.query_one(FindDialog)
        if not self.show_find or not any(
            input.has_focus for input in find_dialog.query("Input")
        ):
            self.show_find = True
            find_dialog.focus_input()
