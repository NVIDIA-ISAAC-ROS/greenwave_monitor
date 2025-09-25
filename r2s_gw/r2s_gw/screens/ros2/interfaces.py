import time
from dataclasses import dataclass
from typing import List, Set

from rich.text import Text

from textual import log
from textual.app import ComposeResult
from textual.binding import Binding
from textual.message import Message
from textual.reactive import reactive
from textual.screen import Screen
from textual.widget import Widget
from textual.widgets import DataTable, Footer
from textual.containers import Vertical

from r2s_gw.watcher import WatcherBase
from r2s_gw.widgets import DataGrid, Header
from r2s_gw.screens.ros2.header import RosHeader

@dataclass(frozen=True, eq=False)
class Interface:
    name: str
    type: str
    interface: str
    nodes: Set
    hidden: bool = False

class InterfacesFetched(Message):
    def __init__(self, interface_list: List[Interface]) -> None:
        self.interface_list = interface_list
        super().__init__()


class InterfaceListWatcher(WatcherBase):
    target: Widget

    def __init__(self, node):
        self.node = node
        super().__init__()

    def run(self) -> None:
        while not self._exit_event.is_set():
            interfaces: List[Interface] = []

            topic_names_and_types = self.node.node.get_topic_names_and_types()

            for t in topic_names_and_types:
                topicPubs = self.node.node.get_publishers_info_by_topic(t[0])
                topicSubs = self.node.node.get_subscriptions_info_by_topic(t[0])

                nodes = set()
                for p in topicPubs:
                    if p.node_name[0] == "/":
                        nodes.add(p)
                    elif p.node_namespace == "/":
                        nodes.add(p.node_namespace + p.node_name)
                    else:
                        nodes.add(p.node_namespace + "/" + p.node_name)
                for p in topicSubs:
                    if p.node_name[0] == "/":
                        nodes.add(p)
                    elif p.node_namespace == "/":
                        nodes.add(p.node_namespace + p.node_name)
                    else:
                        nodes.add(p.node_namespace + "/" + p.node_name)

                interfaces.append(Interface(
                    name = t[0],
                    type = "topic",
                    interface = t[1][0],
                    nodes=nodes,
                ))

            service_names_and_types = self.node.node.get_service_names_and_types()

            for s in service_names_and_types:
                interfaces.append(Interface(
                    name = s[0],
                    type = "service",
                    interface = s[1][0],
                    nodes={}
                ))

            action_names_and_types = self.node.node.handle.get_action_names_and_types()
            for a in action_names_and_types:
                interfaces.append(Interface(
                    name = a[0],
                    type = "action",
                    interface = a[1][0],
                    nodes={}
                ))

            self.target.post_message(InterfacesFetched(interfaces))
            time.sleep(0.5)


class InterfaceListGrid(DataGrid):
    BINDINGS = [
        Binding("h", "toggle_hidden", "Toggle Hidden"),
        Binding("t", "toggle_type", "Toggle Type")
    ]

    interfaces: List[Interface] = []
    title: reactive[str] = reactive("Interfaces")
    hidden: reactive[str] = reactive("visible")
    type: reactive[str] = reactive("all")
    filter_node: reactive[str] = reactive("")

    def set_filter(self) -> None:
        if self.filter_node:
            self.filter = self.filter_node + "," + self.hidden + "," + self.type
        else:
            self.filter = self.hidden + "," + self.type

    def on_mount(self):
        self.title = "Interfaces"
        self.set_filter()

    def action_toggle_hidden(self) -> None:
        if self.hidden == "all":
            self.hidden = "visible"
        elif self.hidden == "visible":
            self.hidden = "all"
        self.set_filter()
        self.populate_rows()

    def action_toggle_type(self) -> None:
        if self.type == "all":
            self.type = "topic"
        elif self.type == "topic":
            self.type = "service"
        elif self.type == "service":
            self.type = "action"
        elif self.type == "action":
            self.type = "all"
        self.set_filter()
        self.populate_rows()

    def watch_filter_node(self) -> None:
        self.set_filter()
        self.populate_rows()

    def columns(self):
        return ["name", "type", "interface"]

    def populate_rows(self):
        table = self.query_one("#data_table", DataTable)
        count = 0

        nodes = set([n.name for n in self.interfaces])
        to_remove = set()
        for row in table.rows:
            if row.value not in nodes:
                to_remove.add(row)
        for row in to_remove:
            table.remove_row(row)

        filter_style = self.get_component_rich_style("datagrid--filter-highlight")

        for interface in self.interfaces:
            prune = False

            if len(self.filter_node) and self.filter_node not in interface.nodes:
                prune = True

            if not self.hidden == "all" and interface.hidden:
                prune = True

            if not self.type == "all" and self.type != interface.type:
                prune = True

            name = Text(interface.name)
            iface = Text(interface.interface)

            if self.search:
                if (not name.highlight_words([self.search], filter_style) and
                    not iface.highlight_words([self.search], filter_style)):
                    prune = True

            if prune:
                if interface.name in table.rows:
                    table.remove_row(interface.name)
            else:
                count = count + 1

                if interface.name not in table.rows:
                    table.add_row(
                        name, interface.type, iface,
                        key=interface.name
                    )
                else:
                    table.update_cell(row_key=interface.name, column_key="name", value=name)
                    table.update_cell(row_key=interface.name, column_key="interface", value=iface)
        self.count = count

    def on_interfaces_fetched(self, message: InterfacesFetched) -> None:
        self.interfaces = message.interface_list
        message.stop()
        self.populate_rows()

class InterfaceListScreen(Vertical):

    filter_node: reactive[str] = reactive("")

    def __init__(self, node):
        self.watcher = InterfaceListWatcher(node)
        super().__init__()

    async def on_mount(self) -> None:
        self.watcher.target = self.query_one(InterfaceListGrid)
        self.watcher.start()

    def on_unmount(self) -> None:
        self.watcher.close()

    def compose(self) -> ComposeResult:
        yield RosHeader()
        yield InterfaceListGrid()
        yield Footer()
