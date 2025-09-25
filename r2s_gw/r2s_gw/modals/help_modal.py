from rich.text import Text
from textual.app import ComposeResult
from textual.containers import Grid
from textual.screen import ModalScreen
from textual.widgets import Button, DataTable


class HelpModal(ModalScreen):
    help_message = (("Q", "Quit"), ("?", "Help"))

    def compose(self) -> ComposeResult:
        button = Button("OK")
        table = DataTable(
            show_cursor=False,
            zebra_stripes=True,
        )
        table.add_columns(
            Text("Key", "bold", justify="center"),
            Text("Action", "bold", justify="center"),
        )
        for row in self.help_message:
            styled_row = [Text(str(row[0]), justify="center"), Text(row[1])]
            table.add_row(*styled_row)
        yield Grid(table, button, id="help")
        button.focus()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        self.dismiss(None)

    def on_key(self, event) -> None:
        if event.key == "escape" or event.key == "enter":
            self.dismiss(None)
