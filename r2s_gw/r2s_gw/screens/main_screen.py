import time

from textual import log
from textual.app import ComposeResult
from textual.screen import Screen

from r2s_gw.watcher import WatcherBase
from r2s_gw.widgets import DataGrid, Header


class NodeWatcher(WatcherBase):
    def run(self) -> None:
        while not self._exit_event.is_set():
            time.sleep(1.0)
            log("Watching")


class MainScreen(Screen):
    CSS = """
    MainScreen {}
    """

    def __init__(self):
        self.watcher = NodeWatcher()
        super().__init__()

    async def on_mount(self) -> None:
        self.watcher.start()

    def on_unmount(self) -> None:
        self.watcher.close()

    def compose(self) -> ComposeResult:
        yield Header()
        yield DataGrid()
