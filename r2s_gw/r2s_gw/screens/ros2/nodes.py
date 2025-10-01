import time
from dataclasses import dataclass
from typing import List

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

import ros2node.api
import ros2lifecycle.api

@dataclass(frozen=True, eq=False)
class Node:
    namespace: str
    name: str
    full_name: str
    hidden: bool
    subscribers: int
    publishers: int
    service_servers: int
    service_clients: int
    state: str | None = None
    transitions: List[str] = None


class NodesFetched(Message):
    def __init__(self, node_list: List[Node]) -> None:
        self.node_list = node_list
        super().__init__()


class NodeListWatcher(WatcherBase):
    target: Widget

    def __init__(self, node):
        self.node = node
        super().__init__()

    def run(self) -> None:
        while not self._exit_event.is_set():
            nodes: List[Node] = []

            node_names_and_namespaces = self.node.node.get_node_names_and_namespaces()

            lifecycle_node_names = ros2lifecycle.api.get_node_names(
                node=self.node.node, include_hidden_nodes=True
            )
            lc_nodes = [n.full_name for n in lifecycle_node_names]

            lifecycle_node_states = ros2lifecycle.api.call_get_states(
                node=self.node.node, node_names=lc_nodes
            )

            lifecycle_node_transitions = ros2lifecycle.api.call_get_available_transitions(
                node=self.node.node, states=lifecycle_node_states
            )

            for t in node_names_and_namespaces:
                hidden = t[0].startswith(ros2node.api.HIDDEN_NODE_PREFIX)
                full_name = t[1] + ("" if t[1].endswith("/") else "/") + t[0]
                subscribers = ros2node.api.get_subscriber_info(node=self.node.node, remote_node_name=full_name)
                publishers = ros2node.api.get_publisher_info(node=self.node.node, remote_node_name=full_name)

                service_servers = ros2node.api.get_service_server_info(node=self.node.node, remote_node_name=full_name)
                service_clients = ros2node.api.get_service_client_info(node=self.node.node, remote_node_name=full_name)

                state = None
                transitions = []

                if full_name in lc_nodes:
                    state = lifecycle_node_states[full_name]
                    transitions = [
                        t.transition.label.upper()
                        for t in lifecycle_node_transitions[full_name]
                    ]

                nodes.append(
                    Node(
                        name=t[0],
                        namespace=t[1],
                        full_name=full_name,
                        hidden=hidden,
                        subscribers=len(subscribers),
                        publishers=len(publishers),
                        service_servers=len(service_servers),
                        service_clients=len(service_clients),
                        state = state,
                        transitions = []
                    )
                )

            self.target.post_message(NodesFetched(nodes))
            time.sleep(0.5)


class NodeListGrid(DataGrid):
    BINDINGS = [
        Binding("h", "toggle_hidden", "Toggle Hidden")
    ]

    nodes: List[Node] = []
    title: reactive[str] = reactive("Nodes")
    hidden: reactive[str] = reactive("visible")

    def on_mount(self):
        self.title = "Nodes"
        self.filter = self.hidden

    def action_toggle_hidden(self) -> None:
        if self.hidden == "all":
            self.hidden = "visible"
        elif self.hidden == "visible":
            self.hidden = "all"
        self.filter = self.hidden
        self.populate_rows()

    def columns(self):
        return ["Namespace", "Name", #"Full Name", 
                "Subscribers", "Publishers", 
                "Service Servers", "Service Clients",
                "Lifecycle"]

    def populate_rows(self):
        table = self.query_one("#data_table", DataTable)
        count = 0

        nodes = set([n.full_name for n in self.nodes])
        to_remove = set()
        for row in table.rows:
            if row.value not in nodes:
                to_remove.add(row)
        for row in to_remove:
            table.remove_row(row)

        filter_style = self.get_component_rich_style("datagrid--filter-highlight")

        for node in self.nodes:
            prune = False

            if not self.hidden == "all" and node.hidden:
                prune = True

            ns = Text(node.namespace)
            name = Text(node.name)

            if self.search:
                if (not ns.highlight_words([self.search], filter_style) and
                    not name.highlight_words([self.search], filter_style)):
                    prune = True

            if prune:
                if node.full_name in table.rows:
                    table.remove_row(node.full_name)
            else:
                count = count + 1

                if node.full_name not in table.rows:
                    table.add_row(
                        ns, name, #node.full_name, 
                        node.subscribers, node.publishers, 
                        node.service_servers, node.service_clients, 
                        node.state.label if node.state else "",
                        key=node.full_name
                    )
                else:
                    table.update_cell(row_key=node.full_name, column_key="namespace", value=ns)
                    table.update_cell(row_key=node.full_name, column_key="name", value=name)
        self.count = count

    def on_nodes_fetched(self, message: NodesFetched) -> None:
        self.nodes = message.node_list
        message.stop()
        self.populate_rows()


class NodeListScreen(Vertical):

    def __init__(self, node):
        self.watcher = NodeListWatcher(node)
        super().__init__()

    async def on_mount(self) -> None:
        self.watcher.target = self.query_one(NodeListGrid)
        self.watcher.start()

    def on_unmount(self) -> None:
        self.watcher.close()

    def compose(self) -> ComposeResult:
        yield RosHeader()
        yield NodeListGrid()
        yield Footer()
