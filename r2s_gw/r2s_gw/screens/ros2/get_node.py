import os

import rclpy
from rclpy.parameter import Parameter

import threading
import time

from textual import log

DEFAULT_TIMEOUT = 0.5
NODE_NAME_PREFIX = "_r2s_gw"

RCLPY_INIT: bool = False
RCLPY_NODE = None


class NodeWrapper:

    nodes_and_namespaces = []
    topic_names_and_types = []

    def __init__(self, node_name, *args):
        node_name_suffix = getattr(args, "node_name_suffix", "_%d" % os.getpid())
        start_parameter_services = getattr(args, "start_parameter_services", False)
        use_sim_time = getattr(args, "use_sim_time", False)
        start_type_description_service = getattr(
            args, "start_type_description_service", True
        )

        if node_name is None:
            node_name = NODE_NAME_PREFIX + node_name_suffix

        try:
            self.node = rclpy.create_node(
                node_name,
                start_parameter_services=start_parameter_services,
                parameter_overrides=[
                    Parameter("use_sim_time", value=use_sim_time),
                    Parameter(
                        "start_type_description_service",
                        value=start_type_description_service,
                    ),
                ],
                automatically_declare_parameters_from_overrides=True,
            )
        except Exception as e:
            log("Exception caught when creating node:  ", e)
            exit(1)

        self.spinning = True
        self.spinner = threading.Thread(target=self.spin)
        self.spinner.start()

    def stop(self):
        self.spinning = False
        self.spinner.join()

    def __del__(self):
        self.stop()

    def spin(self):
        log("start spinning")
        while self.spinning:
            rclpy.spin_once(self.node, timeout_sec=0.5)


def get_node(*args, node_name=None):
    global RCLPY_INIT, RCLPY_NODE

    if not RCLPY_INIT:
        log("Creating context")
        rclpy.init()
        RCLPY_INIT = True

    if not RCLPY_NODE:
        log("Creating node")
        RCLPY_NODE = NodeWrapper(node_name, *args)

    return RCLPY_NODE
