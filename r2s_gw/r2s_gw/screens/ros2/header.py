from r2s_gw.widgets.header import Header

from textual.widgets import Static

from ros2doctor.api import platform

import os
import socket

TITLES = """r2s Version:
ROS Version:
Hostname:
Workspace:
"""

INFO = """0.0.1
{ros_version}
{hostname}
{workspace}"""

class RosHeader(Header):
    def left(self):
        return Static(TITLES, classes="header-box")

    def center(self):
        ros_version = ""
        distro_report = platform.RosdistroReport()
        report = distro_report.report()
        for k, v in report.items:
            if k == "distribution name":
                ros_version = v
                break
        workspace = os.getcwd()
        hostname = socket.gethostname()

        return Static(INFO.format(
            ros_version=ros_version, hostname=hostname, workspace=workspace
        ), classes="header-box")