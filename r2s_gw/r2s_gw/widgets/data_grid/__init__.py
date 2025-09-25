from typing import List

from rich.text import Text
from textual import on, log
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal
from textual.reactive import reactive
from textual.widgets import DataTable

from r2s_gw.widgets.find_dialog import FindDialog


class DataGrid(Horizontal):
    DEFAULT_CSS = """
    DataGrid{
        height: 1fr;
        width: 1fr;
        padding-left: 1;
        padding-right: 1;
        border: $success;
        border-title-align: center;
        border-title-color: $success;

        .datagrid--title {
            color: $success;
            text-style: bold;
        }

        .datagrid--title-filter {
            color: $success;
            text-style: bold;
        }

        .datagrid--title-count {
            color: $success;
            text-style: bold;
        }

        .datagrid--title-search {
            color: $success;
            text-style: bold;
        }

        .datagrid--sorted-column-header {
            color: $success;
            text-style: bold;
        }

        .datagrid--filter-highlight {
            background: $secondary;
            color: auto;
        }

        DataTable {
            height: 1fr;
            width: 1fr;
        }

        DataTable > .datatable--header {
            text-style: bold;
            background: $surface;
            color: white;
        }
    }
    """

    COMPONENT_CLASSES = {
        "datagrid--title",
        "datagrid--title-filter",
        "datagrid--title-count",
        "datagrid--title-search",
        "datagrid--sorted-column-header",
        "datagrid--filter-highlight",
    }

    BINDINGS = [
        Binding("<,left", "dec_sort_key", "Previous Sort"),
        Binding(">,right", "inc_sort_key", "Next Sort"),
        Binding("ctrl+f", "show_find_dialog", "Find", key_display="^f"),
        Binding("slash", "show_find_dialog", "Find", key_display="^f", show=False),
    ]

    show_find: reactive[bool] = reactive(False)

    title: reactive[str] = reactive("")
    filter: reactive[str] = reactive("")
    count: reactive[int] = reactive(0)
    search: reactive[str] = reactive("")
    reverse: bool = False

    id: str = "data_table"

    default_sort_column_id = 0
    sort_column_id = reactive(default_sort_column_id)

    BORDER_TITLE: str = "DataGrid"

    def __init__(self) -> None:
        super().__init__()
        self._composed = False

    def columns(self) -> List[str]:
        """Get the column labels"""
        return []

    def render(self) -> str:
        title_style = self.get_component_rich_style(
            "datagrid--title",
            partial=True,
        )
        title_filter_style = self.get_component_rich_style(
            "datagrid--title-filter",
            partial=True,
        )
        title_count_style = self.get_component_rich_style(
            "datagrid--title-count",
            partial=True,
        )
        title_search_style = self.get_component_rich_style(
            "datagrid--title-search",
            partial=True,
        )

        title = (Text(self.title, style=title_style) + 
            Text(f" ({self.filter}) ", style=title_filter_style) +
            Text(f"[{self.count}]", style=title_count_style))

        if len(self.search):
            title = title + Text(f" <{str(self.search)}>", style=title_search_style)
        self.border_title = title
        return self.border_title

    def compose(self) -> ComposeResult:
        table: DataTable[Text] = DataTable(
            id=self.id,
            header_height=1,
            show_cursor=True,
            zebra_stripes=True,
            cell_padding=5,
            cursor_foreground_priority="renderable"
        )
        table.focus()

        for ii, col in enumerate(self.columns()):
            table.add_column(self.get_heading(ii, col), key=col.lower())

        table.cursor_type = "row"
        self._composed = True
        yield table
        yield FindDialog()


    def get_heading(self, column_idx: int, label: str) -> Text:
        sort_column = (
            self.sort_column_id if self._composed else self.default_sort_column_id
        )
        sort_column_style = self.get_component_rich_style(
            "datagrid--sorted-column-header",
            partial=True,
        )
        log(column_idx, sort_column)
        if column_idx == sort_column:
            return Text(
                label, style=sort_column_style
            )
        else:
            return Text(label)

    async def watch_show_find(self, show_find: bool) -> None:
        if not self.is_mounted:
            return
        filter_dialog = self.query_one(FindDialog)
        filter_dialog.set_class(show_find, "visible")
        if show_find:
            filter_dialog.focus_input()
        else:
            self.query_one(DataTable).focus()

    def action_show_find_dialog(self) -> None:
        find_dialog = self.query_one(FindDialog)
        if not self.show_find or not any(
            input.has_focus for input in find_dialog.query("Input")
        ):
            self.show_find = True
            find_dialog.focus_input()

    @on(FindDialog.Dismiss)
    def dismiss_filter_dialog(self, event: FindDialog.Dismiss) -> None:
        event.stop()
        self.show_find = False

    @on(FindDialog.Update)
    def update_find(self, event: FindDialog.Update) -> None:
        event.stop()
        self.search = event.find

    def action_inc_sort_key(self) -> None:
        table = self.query_one(DataTable)
        new_idx = self.sort_column_id + 1

        cols = self.columns()
        if new_idx >= len(cols):
            new_idx = len(cols) - 1
        if new_idx == self.sort_column_id:
            self.reverse = not self.reverse
        else:
            self.reverse = False
        self.sort_column_id = new_idx

    def action_dec_sort_key(self) -> None:
        table = self.query_one(DataTable)
        new_idx = self.sort_column_id - 1
        cols = self.columns()
        if new_idx < 0:
            new_idx = 0
        if new_idx == self.sort_column_id:
            self.reverse = not self.reverse
        else:
            self.reverse = False
        self.sort_column_id = new_idx

    def watch_sort_column_id(self, sort_column_id: int) -> None:
        table = self.query_one(DataTable)
        for ii, col in enumerate(self.columns()):
            heading = self.get_heading(ii, col)
            log(ii, heading)
            table.ordered_columns[ii].label = heading 

        def sortable(v):
            if isinstance(v, Text):
                return v.plain
            else:
                return v

        table.sort(table.ordered_columns[sort_column_id].key, key=sortable, reverse=self.reverse)