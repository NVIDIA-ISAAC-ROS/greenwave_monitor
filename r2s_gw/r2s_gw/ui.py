from rich import terminal_theme
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import TabbedContent, TabPane

try:
    from r2s_gw.screens.ros2.get_node import get_node
    from r2s_gw.screens.ros2.nodes import NodeListScreen
    from r2s_gw.screens.ros2.interfaces import InterfaceListScreen
    from r2s_gw.screens.ros2.topics import TopicListScreen
    ROS_AVAILABLE = True
except ImportError as ex:
    ROS_AVAILABLE = False
    ROS_ERROR = ex
    print(ex)


class UI(App):
    BINDINGS = [
        Binding("ctrl+c,q", "quit", "Quit", show=True, key_display="Q"),
    ]
    ansi_colors = True
    node = None

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        if ROS_AVAILABLE:
            self.node = get_node()
            with TabbedContent():
                yield TabPane("Interfaces", InterfaceListScreen(self.node))
                yield TabPane("Nodes", NodeListScreen(self.node))
                yield TabPane("Topics", TopicListScreen(self.node))

    async def on_mount(self) -> None:
        if not ROS_AVAILABLE:
            self.log.warning(ROS_ERROR)
        # self.ansi_theme_dark = terminal_theme.DIMMED_MONOKAI
        self.theme = "nord"

    async def on_unmount(self) -> None:
        if ROS_AVAILABLE and self.node:
            self.node.stop()
