import time
from dataclasses import dataclass
from typing import List, Set

from rich.text import Text


from textual.app import ComposeResult
from textual.binding import Binding
from textual.message import Message
from textual.reactive import reactive
from textual.screen import Screen, ModalScreen
from textual.widget import Widget
from textual.widgets import DataTable, Footer
from textual.containers import Vertical, Horizontal
from textual.widgets import Input, Button, Static

from r2s_gw.watcher import WatcherBase
from r2s_gw.widgets import DataGrid, Header
from r2s_gw.screens.ros2.header import RosHeader
from greenwave_monitor.ui_adaptor import GreenwaveUiAdaptor


@dataclass(frozen=True, eq=False)
class Topic:
    name: str
    msg_type: str
    frequency: str = ""
    expected_hz: float = 0.0
    tolerance: float = 0.0
    status: str = ""


class TopicsFetched(Message):
    def __init__(self, topic_list: List[Topic]) -> None:
        self.topic_list = topic_list
        super().__init__()


class TopicListWatcher(WatcherBase):
    target: Widget

    def __init__(self, node):
        self.node = node
        self.diagnostics_monitor = GreenwaveUiAdaptor(node.node)
        super().__init__()

    def run(self) -> None:
        while not self._exit_event.is_set():
            topics: List[Topic] = []

            topic_names_and_types = self.node.node.get_topic_names_and_types()

            for t in topic_names_and_types:
                topicPubs = self.node.node.get_publishers_info_by_topic(t[0])
                topicSubs = self.node.node.get_subscriptions_info_by_topic(t[0])

                publishers = set()
                subscribers = set()

                for p in topicPubs:
                    if p.node_name[0] == "/":
                        publishers.add(p.node_name)
                    elif p.node_namespace == "/":
                        publishers.add(p.node_namespace + p.node_name)
                    else:
                        publishers.add(p.node_namespace + "/" + p.node_name)

                for s in topicSubs:
                    if s.node_name[0] == "/":
                        subscribers.add(s.node_name)
                    elif s.node_namespace == "/":
                        subscribers.add(s.node_namespace + s.node_name)
                    else:
                        subscribers.add(s.node_namespace + "/" + s.node_name)

                diagnostic = self.diagnostics_monitor.get_topic_diagnostics(t[0])
                freq = diagnostic.pub_rate
                expected_hz, tolerance = self.diagnostics_monitor.get_expected_frequency(t[0])

                topics.append(Topic(
                    name=t[0],
                    msg_type=t[1][0],
                    frequency=freq,
                    status=diagnostic.status,
                    expected_hz=expected_hz,
                    tolerance=tolerance,
                ))

            self.target.post_message(TopicsFetched(topics))
            time.sleep(0.5)


class SetExpectedFrequencyWindow(ModalScreen):
    """A dialog to set monitoring parameters for a topic."""

    CSS = """
    SetExpectedFrequencyWindow {
        align: center middle;
    }

    #dialog {
        width: 38;
        height: auto;
        border: double $primary;
        background: $surface;
        padding: 1 2;
    }

    #error-message {
        color: $error;
        height: auto;
        margin-top: 1;
    }

    #dialog > Horizontal {
        height: auto;
        align: center middle;
        margin-top: 1;
    }

    #dialog > Horizontal > Button {
        width: 50%;
    }
    #clear {
        width: 100%;
    }
    """

    def __init__(
        self,
        topic_name: str,
        diagnostics_monitor: GreenwaveUiAdaptor,
        expected_hz: str = "",
        tolerance: str = ""
    ) -> None:
        self.topic_name = topic_name
        self.diagnostics_monitor = diagnostics_monitor
        self.expected_hz = expected_hz
        self.tolerance = tolerance
        super().__init__()

    def compose(self) -> ComposeResult:
        with Vertical(id="dialog"):
            yield Static(f"Set expected for {self.topic_name}")
            yield Input(
                placeholder="Expected Frequency (Hz)",
                id="frequency",
                value=self.expected_hz,
            )
            yield Input(
                placeholder="Tolerance (%)",
                id="tolerance",
                value=self.tolerance,
            )
            yield Static("", id="error-message")
            with Horizontal():
                yield Button("Confirm", variant="primary", id="confirm", disabled=True)
                yield Button("Cancel", variant="error", id="cancel")
            yield Button("Clear Expected", id="clear", disabled=True)

    def on_mount(self) -> None:
        self._update_ok_button_state()
        self._update_clear_button_state()

    def _update_clear_button_state(self) -> None:
        clear_button = self.query_one("#clear", Button)
        clear_button.disabled = not self.expected_hz

    def _update_ok_button_state(self) -> None:
        """Validate inputs and enable/disable the OK button."""
        frequency_str = self.query_one("#frequency", Input).value
        tolerance_str = self.query_one("#tolerance", Input).value
        error_message = self.query_one("#error-message", Static)
        ok_button = self.query_one("#confirm", Button)

        is_valid = False
        message = ""
        try:
            if frequency_str and tolerance_str:
                float(frequency_str)
                float(tolerance_str)
                is_valid = True
            else:
                message = "Both fields are required."

        except ValueError:
            message = "Inputs must be valid numbers."

        ok_button.disabled = not is_valid
        error_message.update(message)

    def on_input_changed(self, event: Input.Changed) -> None:
        self._update_ok_button_state()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "confirm":
            frequency_str = self.query_one("#frequency", Input).value
            tolerance_str = self.query_one("#tolerance", Input).value

            frequency = float(frequency_str)
            tolerance = float(tolerance_str)

            # Response for SetExpectedFrequency service call
            success, message = self.diagnostics_monitor.set_expected_frequency(
                self.topic_name,
                expected_hz=frequency,
                tolerance_percent=tolerance
            )
            if success:
                self.dismiss()
            else:
                self.query_one("#error-message", Static).update(
                    message or "An unknown error occurred."
                )
        elif event.button.id == "cancel":
            self.dismiss()
        elif event.button.id == "clear":
            self.diagnostics_monitor.set_expected_frequency(self.topic_name, clear=True)
            self.dismiss()


class TopicListGrid(DataGrid):
    BINDINGS = [
        Binding("m", "toggle_monitor", "Monitor Topic"),
        Binding("e", "manage_expected_frequency()", "Set/Clear Expected Frequency"),
    ]

    topics: List[Topic] = []
    title: reactive[str] = reactive("Topics")
    filter_node: reactive[str] = reactive("")

    def on_mount(self):
        self.title = "Topics"
        self.filter = ""

    def watch_filter_node(self) -> None:
        self.filter = self.filter_node if self.filter_node else ""
        self.populate_rows()

    def columns(self):
        return ["Topic Name", "Message Type", "Frequency", "Expected Frequency", "Status"]

    def populate_rows(self):
        table = self.query_one("#data_table", DataTable)
        count = 0

        topics = set([t.name for t in self.topics])
        to_remove = set()
        for row in table.rows:
            if row.value not in topics:
                to_remove.add(row)
        for row in to_remove:
            table.remove_row(row)

        filter_style = self.get_component_rich_style("datagrid--filter-highlight")

        for topic in self.topics:
            prune = False

            # Filter by node if specified
            if self.filter_node:
                if not any(self.filter_node in pub for pub in topic.publishers) and \
                   not any(self.filter_node in sub for sub in topic.subscribers):
                    prune = True

            expected_is_set = topic.expected_hz > 0.0
            row_style = self._get_row_style(topic.status) if expected_is_set else ""

            topic_name = Text(topic.name, style=row_style)
            msg_type = Text(topic.msg_type, style=row_style)

            if self.search:
                name_highlighted = topic_name.highlight_words([self.search], filter_style)
                type_highlighted = msg_type.highlight_words([self.search], filter_style)
                if not name_highlighted and not type_highlighted:
                    prune = True

            if prune:
                if topic.name in table.rows:
                    table.remove_row(topic.name)
            else:
                count += 1

                frequency = Text(topic.frequency, style=row_style)
                expected_hz_text = Text(
                    f"{topic.expected_hz:.2f} Hz ± {topic.tolerance:.0f}%" if expected_is_set else "-",
                    style=row_style)
                status_text = Text(topic.status if expected_is_set else "-", style=row_style)

                if topic.name not in table.rows:
                    table.add_row(
                        topic_name, msg_type, frequency,
                        expected_hz_text, status_text,
                        key=topic.name
                    )
                else:
                    table.update_cell(row_key=topic.name, column_key="topic name", value=topic_name)
                    table.update_cell(row_key=topic.name,
                                      column_key="message type", value=msg_type)
                    table.update_cell(row_key=topic.name, column_key="frequency", value=frequency)
                    table.update_cell(row_key=topic.name, column_key="expected frequency",
                                      value=expected_hz_text, update_width=True)
                    table.update_cell(row_key=topic.name, column_key="status", value=status_text)

        self.count = count

    def _get_row_style(self, status: str) -> str:
        if status == "OK":
            return "green"
        elif status == "WARN" or status == "STALE":
            return "yellow"
        elif status == "ERROR":
            return "red bold"
        return ""

    def on_topics_fetched(self, message: TopicsFetched) -> None:
        self.topics = message.topic_list
        self.populate_rows()

    def action_toggle_monitor(self) -> None:
        table = self.query_one("#data_table", DataTable)
        row_key, _ = table.coordinate_to_cell_key(table.cursor_coordinate)
        if row_key:
            topic_name = str(row_key.value)
            # Find the watcher through parent screen
            if hasattr(
                    self.parent, 'watcher') and hasattr(
                    self.parent.watcher, 'diagnostics_monitor'):
                self.parent.watcher.diagnostics_monitor.toggle_topic_monitoring(topic_name)
                print(f"Toggled monitoring for topic: {topic_name}")

    def action_manage_expected_frequency(self) -> None:
        table = self.query_one("#data_table", DataTable)
        row_key, _ = table.coordinate_to_cell_key(table.cursor_coordinate)
        if not row_key:
            return

        topic_name = str(row_key.value)
        if hasattr(
                self.parent, 'watcher') and hasattr(
                self.parent.watcher, 'diagnostics_monitor'):
            diagnostics_monitor = self.parent.watcher.diagnostics_monitor
        else:
            return

        topic = None
        for monitored_topic in self.topics:
            if monitored_topic.name == topic_name:
                topic = monitored_topic
                break
        if not topic:
            return

        expected_is_set = topic.expected_hz > 0.0
        # Load previous values to dialog if they exist
        current_hz = f"{topic.expected_hz:.2f}" if expected_is_set else ""
        current_tolerance = f"{topic.tolerance:.2f}" if expected_is_set else ""

        self.app.push_screen(
            SetExpectedFrequencyWindow(
                topic_name,
                diagnostics_monitor,
                expected_hz=current_hz,
                tolerance=current_tolerance,
            )
        )


class TopicListScreen(Vertical):
    """A screen to display ROS 2 topics."""

    def __init__(self, node):
        self.node = node
        self.watcher = TopicListWatcher(node)
        super().__init__()

    async def on_mount(self) -> None:
        self.watcher.target = self.query_one(TopicListGrid)
        self.watcher.start()

    def on_unmount(self) -> None:
        self.watcher.close()

    def compose(self) -> ComposeResult:
        yield RosHeader()
        yield TopicListGrid()
        yield Footer()
