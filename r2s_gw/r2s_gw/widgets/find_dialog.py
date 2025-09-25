from dataclasses import dataclass

from textual import on
from textual.app import ComposeResult
from textual.binding import Binding
from textual.message import Message
from textual.widget import Widget
from textual.widgets import Input


class FindDialog(Widget):
    DEFAULT_CSS = """
    FindDialog {
        layout: horizontal;
        dock: top;
        padding-top: 1;
        padding-bottom: 1;
        width: 1fr;
        height: auto;
        display: none;

        & #find {
            width: 1fr;
        }
        &.visible {
            display: block;
        }
        Input {
            width: 1fr;
        }
        Input#find-text {
            display: block;
        }
    }
    """

    BINDINGS = [
        Binding("escape", "dismiss_find", "Dismiss", key_display="esc", show=False),
    ]

    DEFAULT_CLASSES = "float"
    BORDER_TITLE = "Find"

    @dataclass
    class Update(Message):
        find: str

    class Dismiss(Message):
        pass

    def compose(self) -> ComposeResult:
        yield Input(placeholder="find", id="find-text")

    def focus_input(self) -> None:
        self.query_one("#find-text").focus()

    def get_value(self) -> str:
        return self.query_one("#find-text", Input).value

    @on(Input.Changed)
    def input_change(self, event: Input.Changed) -> None:
        event.stop()
        self.post_update()

    @on(Input.Submitted)
    def input_submitted(self, event: Input.Changed) -> None:
        event.stop()

    def post_update(self) -> None:
        update = FindDialog.Update(
            find=self.get_value(),
        )
        self.post_message(update)

    def action_dismiss_find(self) -> None:
        self.post_message(FindDialog.Dismiss())
