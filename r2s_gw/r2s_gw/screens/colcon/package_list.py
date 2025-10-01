import os
from argparse import ArgumentParser
from dataclasses import dataclass
from pathlib import Path
from typing import List

from colcon_core.location import set_default_config_path  # noqa: E402
from colcon_core.package_selection import add_arguments as add_packages_arguments
from colcon_core.package_selection import get_package_descriptors
from textual import log
from textual.app import ComposeResult
from textual.message import Message
from textual.screen import Screen
from textual.widget import Widget
from textual.widgets import DataTable

from r2s_gw.watcher import WatcherBase
from r2s_gw.widgets import DataGrid, Header


@dataclass(frozen=True, eq=False)
class Package:
    name: str
    path: Path
    type: str
    version: str | None


class PackagesFetched(Message):
    def __init__(self, package_list: List[Package]) -> None:
        self.package_list = package_list
        super().__init__()


class PackageListWatcher(WatcherBase):

    target: Widget | None = None

    def run(self) -> None:
        command_name = "colcon"
        set_default_config_path(
            path=(Path("~") / f".{command_name}").expanduser(),
            env_var=f"{command_name}_HOME".upper(),
        )

        parser = ArgumentParser()
        add_packages_arguments(parser)
        args = parser.parse_args()
        args.base_paths = ["/usr/local/google/home/mjcarroll/workspaces/ros2_rolling"]

        while not self._exit_event.is_set():
            descriptors = get_package_descriptors(args)

            packages: List[Package] = []

            for descrip in descriptors:
                log(descrip)
                vv = None
                if "version" in descrip.metadata:
                    vv = descrip.metadata["version"]
                packages.append(
                    Package(
                        name=descrip.name,
                        path=descrip.path,
                        type=descrip.type,
                        version=vv,
                    )
                )
            if self.target is not None:
                self.target.post_message(PackagesFetched(packages))
            break


class PackageListGrid(DataGrid):
    base_path: str = "/usr/local/google/home/mjcarroll/workspaces/ros2_rolling"

    def columns(self):
        return ["Name", "Version", "Type", "Path"]

    def on_packages_fetched(self, message: PackagesFetched) -> None:
        """Method called to process each fetched snapshot."""
        message.stop()
        table = self.query_one("#data_table", DataTable)

        for package in message.package_list:
            pp = os.path.relpath(package.path, os.path.join(self.base_path, "src"))
            if pp not in table.rows:
                table.add_row(
                    package.name,
                    package.version if package.version else "",
                    package.type,
                    pp,
                    key=pp,
                )


class PackageListScreen(Screen):
    CSS = """
    PackageListScreen {}
    """

    def __init__(self):
        self.watcher = PackageListWatcher()
        super().__init__()

    async def on_mount(self) -> None:
        self.watcher.target = self.query_one(PackageListGrid)
        self.watcher.start()

    def on_unmount(self) -> None:
        self.watcher.close()

    def compose(self) -> ComposeResult:
        yield Header()
        yield PackageListGrid()
