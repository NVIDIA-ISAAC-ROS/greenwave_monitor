#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

"""
Greenwave monitor diagnostics helpers for UI frontends.

This module contains small data containers plus a monitor class intended to be embedded in
UI processes. The `greenwave_monitor_node` publishes `diagnostic_msgs/DiagnosticArray`
messages on `/diagnostics`. `GreenwaveUiAdaptor` subscribes to that topic and maintains a
thread-safe, easy-to-consume view (`UiDiagnosticData`) per monitored topic, including the
timestamp of the last update for each topic.

In addition to passively subscribing, `GreenwaveUiAdaptor` exposes:
- Parameter-based topic monitoring: start/stop monitoring a topic via the
  `greenwave_diagnostics.<topic>.enabled` parameter (`toggle_topic_monitoring`).
- Parameter-based frequency configuration: set/clear the expected publish rate and tolerance
  for a topic (`set_expected_frequency`) via ROS parameters. Expected rates are also cached
  locally in `expected_frequencies` as `(expected_hz, tolerance_percent)` so UIs can display
  the configured values alongside live diagnostics.
"""

from dataclasses import dataclass
import math
import threading
import time
from typing import Dict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import Parameter, ParameterEvent, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
import rclpy
from rclpy.node import Node

# Parameter name constants
TOPIC_PARAM_PREFIX = 'greenwave_diagnostics.'
FREQ_SUFFIX = '.expected_frequency'
TOL_SUFFIX = '.tolerance'
ENABLED_SUFFIX = '.enabled'
DEFAULT_TOLERANCE_PERCENT = 5.0


def build_full_node_name(node_name: str, node_namespace: str, is_client: bool = False) -> str:
    """Build full ROS node name from name and namespace."""
    join_list = []
    # Strip leading '/' from namespace to avoid double slashes when joining
    if node_namespace and node_namespace != '/':
        join_list.append(node_namespace.lstrip('/'))
    if node_name:
        join_list.append(node_name)
    joined = '/'.join(join_list)
    if not is_client:
        return f'/{joined}'
    return joined


def make_freq_param(topic: str) -> str:
    """Build frequency parameter name for a topic."""
    return f'{TOPIC_PARAM_PREFIX}{topic}{FREQ_SUFFIX}'


def make_tol_param(topic: str) -> str:
    """Build tolerance parameter name for a topic."""
    return f'{TOPIC_PARAM_PREFIX}{topic}{TOL_SUFFIX}'


def _make_param(name: str, value) -> Parameter:
    """Create a Parameter message from a name and Python value (or ParameterValue)."""
    param = Parameter()
    param.name = name
    if isinstance(value, ParameterValue):
        param.value = value
    else:
        param.value = ParameterValue()
        if isinstance(value, str):
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = value
        elif isinstance(value, bool):
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = value
        elif isinstance(value, int):
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = value
        else:
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(value)
    return param


def set_ros_parameters(node: Node, target_node: str, params: dict,
                       timeout_sec: float = 3.0) -> tuple[bool, list[str]]:
    """
    Set one or more parameters on a target node.

    :param node: The ROS node to use for service calls.
    :param target_node: Full name of target node (e.g., '/my_node' or '/ns/my_node').
    :param params: Dict mapping parameter names to values.
    :param timeout_sec: Service call timeout.
    :returns: Tuple of (all_successful, list of failure reasons).
    """
    if '/' not in target_node:
        target_node = f'/{target_node}'
    client = node.create_client(SetParameters, f'{target_node}/set_parameters')
    if not client.wait_for_service(timeout_sec=min(timeout_sec, 5.0)):
        node.destroy_client(client)
        return False, ['Service not available']

    request = SetParameters.Request()
    request.parameters = [_make_param(name, value) for name, value in params.items()]

    try:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        node.destroy_client(client)

        if future.result() is None:
            return False, ['Service call timed out']

        results = future.result().results
        failures = [r.reason for r in results if not r.successful and r.reason]
        return all(r.successful for r in results), failures
    except Exception as e:
        node.destroy_client(client)
        return False, [str(e)]


def get_ros_parameters(node: Node, target_node: str, param_names: list,
                       timeout_sec: float = 5.0) -> dict:
    """
    Get parameters from a target node.

    :param node: The ROS node to use for service calls.
    :param target_node: Full name of target node (e.g., '/my_node' or '/ns/my_node').
    :param param_names: List of parameter names to retrieve.
    :param timeout_sec: Service call timeout.
    :returns: Dict mapping parameter names to their values (float or None if not found/invalid).
    """
    client = node.create_client(GetParameters, f'{target_node}/get_parameters')
    if not client.wait_for_service(timeout_sec=min(timeout_sec, 5.0)):
        node.destroy_client(client)
        return {name: None for name in param_names}

    request = GetParameters.Request()
    request.names = param_names

    try:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        node.destroy_client(client)

        if future.result() is None or not future.result().values:
            return {name: None for name in param_names}

        result = {}
        for name, value in zip(param_names, future.result().values):
            result[name] = param_value_to_python(value)
        return result
    except Exception:
        node.destroy_client(client)
        return {name: None for name in param_names}


def parse_topic_param_name(param_name: str) -> tuple[str, str]:
    """Parse parameter name to extract topic name and field type."""
    if not param_name.startswith(TOPIC_PARAM_PREFIX):
        return ('', '')

    topic_and_field = param_name[len(TOPIC_PARAM_PREFIX):]

    if topic_and_field.endswith(FREQ_SUFFIX):
        return (topic_and_field[:-len(FREQ_SUFFIX)], 'freq')
    if topic_and_field.endswith(TOL_SUFFIX):
        return (topic_and_field[:-len(TOL_SUFFIX)], 'tol')
    if topic_and_field.endswith(ENABLED_SUFFIX):
        return (topic_and_field[:-len(ENABLED_SUFFIX)], 'enabled')

    return ('', '')


def param_value_to_python(value: ParameterValue):
    """Convert a ParameterValue to its Python equivalent."""
    if value.type == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    elif value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    elif value.type == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    elif value.type == ParameterType.PARAMETER_STRING:
        return value.string_value
    return None


_STATUS_LEVEL_MAP = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'WARN',
    DiagnosticStatus.ERROR: 'ERROR',
    DiagnosticStatus.STALE: 'STALE',
}


@dataclass
class UiDiagnosticData:
    """
    UI-ready snapshot of diagnostics for a monitored topic.

    Fields are stored as strings for straightforward rendering in UI components.
    `status` is one of: 'OK' | 'WARN' | 'ERROR' | 'STALE' | 'UNKNOWN' (or '-' if unset).
    `last_update` stores the epoch timestamp when diagnostics were last refreshed.
    """

    pub_rate: str = '-'
    msg_rate: str = '-'
    latency: str = '-'
    status: str = '-'
    last_update: float = 0.0

    @classmethod
    def from_status(cls, status: DiagnosticStatus) -> 'UiDiagnosticData':
        """Create UiDiagnosticData from DiagnosticStatus."""
        data = cls()
        data.status = _STATUS_LEVEL_MAP.get(status.level, 'UNKNOWN')

        for kv in status.values:
            if kv.key == 'frame_rate_node':
                data.pub_rate = kv.value
            elif kv.key == 'frame_rate_msg':
                data.msg_rate = kv.value
            elif kv.key == 'current_delay_from_realtime_ms':
                data.latency = kv.value
        return data


class GreenwaveUiAdaptor:
    """
    Subscribe to `/diagnostics` and manage topic monitoring for UI consumption.

    Designed for UI frontends, this class keeps per-topic `UiDiagnosticData` up to date,
    provides a toggle for monitoring via the `greenwave_diagnostics.<topic>.enabled` parameter,
    and exposes helpers to set/clear expected frequencies via ROS parameters.

    """

    def __init__(self, node: Node, monitor_node_name: str = 'greenwave_monitor'):
        """Initialize the UI adaptor for subscribing to diagnostics and managing topics."""
        self.node = node
        # Just use the bare node name - ROS will expand with the caller's namespace
        self.monitor_node_name = monitor_node_name
        self.data_lock = threading.Lock()
        self.ui_diagnostics: Dict[str, UiDiagnosticData] = {}
        # { topic_name : (expected_hz, tolerance) }
        self.expected_frequencies: Dict[str, tuple[float, float]] = {}
        # { topic_name : node_name } - maps topics to the node that has their params
        self.topic_to_node: Dict[str, str] = {}

        self._setup_ros_components()

    def _setup_ros_components(self):
        """Initialize ROS2 subscriptions, clients, and timers."""
        self.subscription = self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._on_diagnostics,
            100
        )

        self.param_events_subscription = self.node.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self._on_parameter_event,
            10
        )

        # Track pending node queries to prevent garbage collection
        self._pending_node_queries: Dict[str, dict] = {}

        # Query initial parameters after a short delay to let nodes come up
        self._initial_params_timer = self.node.create_timer(
            2.0, self._fetch_initial_parameters_callback)

    def _fetch_initial_parameters_callback(self):
        """Timer callback to fetch initial parameters from all nodes."""
        if self._initial_params_timer is not None:
            self._initial_params_timer.cancel()
            self._initial_params_timer = None

        for node_name, node_namespace in self.node.get_node_names_and_namespaces():
            if node_name == self.node.get_name():
                continue

            full_node_name = build_full_node_name(node_name, node_namespace, is_client=True)
            self._query_node_parameters_async(full_node_name)

    def _query_node_parameters_async(self, full_node_name: str):
        """Start async query for topic parameters on a specific node."""
        list_client = self.node.create_client(
            ListParameters, f'{full_node_name}/list_parameters')

        if not list_client.service_is_ready():
            self.node.destroy_client(list_client)
            return

        query_id = full_node_name
        self._pending_node_queries[query_id] = {
            'node_name': full_node_name,
            'list_client': list_client,
            'get_client': None,
            'param_names': []
        }

        list_request = ListParameters.Request()
        list_request.prefixes = ['greenwave_diagnostics']
        list_request.depth = 10

        list_future = list_client.call_async(list_request)
        list_future.add_done_callback(
            lambda f, qid=query_id: self._on_list_params_response(f, qid))

    def _on_list_params_response(self, future, query_id: str):
        """Handle list_parameters response from a node."""
        try:
            query = self._pending_node_queries.get(query_id)
            if not query or future.result() is None:
                self._cleanup_node_query(query_id)
                return

            param_names = [n for n in future.result().result.names
                           if n.startswith(TOPIC_PARAM_PREFIX)]
            if not param_names:
                self._cleanup_node_query(query_id)
                return

            full_node_name = query['node_name']
            get_client = self.node.create_client(
                GetParameters, f'{full_node_name}/get_parameters')

            if not get_client.service_is_ready():
                self.node.destroy_client(get_client)
                self._cleanup_node_query(query_id)
                return

            query['get_client'] = get_client
            query['param_names'] = param_names

            get_request = GetParameters.Request()
            get_request.names = param_names

            get_future = get_client.call_async(get_request)
            get_future.add_done_callback(
                lambda f, qid=query_id: self._on_get_params_response(f, qid))

        except Exception as e:
            self.node.get_logger().debug(f'Error listing parameters: {e}')
            self._cleanup_node_query(query_id)

    def _on_get_params_response(self, future, query_id: str):
        """Handle get_parameters response from a node."""
        try:
            query = self._pending_node_queries.get(query_id)
            if not query or future.result() is None:
                self._cleanup_node_query(query_id)
                return

            full_node_name = query['node_name']

            # Extract all topic names from param names to build topic -> node mapping
            topics_on_this_node = set()
            for name in query['param_names']:
                topic_name, field = parse_topic_param_name(name)
                if topic_name:
                    topics_on_this_node.add(topic_name)

            topic_configs: Dict[str, Dict[str, float]] = {}
            for name, value in zip(query['param_names'], future.result().values):
                py_value = param_value_to_python(value)
                if not isinstance(py_value, (int, float)):
                    continue
                topic_name, field = parse_topic_param_name(name)
                if not topic_name or not field:
                    continue
                if topic_name not in topic_configs:
                    topic_configs[topic_name] = {}
                topic_configs[topic_name][field] = float(py_value)

            with self.data_lock:
                for topic_name in topics_on_this_node:
                    self.topic_to_node[topic_name] = full_node_name

                for topic_name, config in topic_configs.items():
                    freq = config.get('freq', 0.0)
                    tol = config.get('tol', DEFAULT_TOLERANCE_PERCENT)
                    if freq > 0 and not math.isnan(freq):
                        self.expected_frequencies[topic_name] = (freq, tol)

        except Exception as e:
            self.node.get_logger().debug(f'Error fetching parameters: {e}')
        finally:
            self._cleanup_node_query(query_id)

    def _cleanup_node_query(self, query_id: str):
        """Clean up clients for a completed node query."""
        query = self._pending_node_queries.pop(query_id, None)
        if query:
            if query.get('list_client'):
                self.node.destroy_client(query['list_client'])
            if query.get('get_client'):
                self.node.destroy_client(query['get_client'])

    def _call_service(self, client, request, timeout_sec: float = 3.0):
        """Call a service and return the result, or None on timeout/error."""
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            return future.result()
        except Exception:
            return None

    def _extract_topic_name(self, diagnostic_name: str) -> str:
        """
        Extract topic name from diagnostic status name.

        Handles both formats:
        - NITROS: node_name + namespace + "/" + topic (e.g., "my_node/ns/camera/image")
        - Greenwave: topic_name only (e.g., "/ns/camera/image")

        This is a temporary hack until NITROS migrates to message_diagnostics.hpp.
        """
        # If the name starts with '/', it's already just a topic name (Greenwave format)
        if diagnostic_name.startswith('/'):
            return diagnostic_name

        # NITROS format: node_name + namespace + "/" + topic_name
        # Node names cannot contain '/', so the first '/' marks where namespace+topic begins
        idx = diagnostic_name.find('/')
        if idx >= 0:
            return diagnostic_name[idx:]

        # Fallback: return as-is if no '/' found
        return diagnostic_name

    def _on_diagnostics(self, msg: DiagnosticArray):
        """Process incoming diagnostic messages."""
        with self.data_lock:
            # Update diagnostics
            for status in msg.status:
                ui_data = UiDiagnosticData.from_status(status)
                ui_data.last_update = time.time()
                # Normalize the topic name to handle both NITROS and Greenwave formats
                topic_name = self._extract_topic_name(status.name)
                self.ui_diagnostics[topic_name] = ui_data

    def _on_parameter_event(self, msg: ParameterEvent):
        """Process parameter change events from any node."""
        for param in msg.changed_parameters + msg.new_parameters:
            topic_name, field = parse_topic_param_name(param.name)
            if not topic_name or field == '':
                continue

            with self.data_lock:
                if field == 'enabled' and param in msg.new_parameters:
                    if (param.value.type == ParameterType.PARAMETER_BOOL and
                            param.value.bool_value):
                        # Keep the topic to node map up to date when new enabled parameters appear
                        # This makes it easy to set the right parameter in toggle topic monitoring
                        self.topic_to_node[topic_name] = msg.node
                    continue
                elif field == 'enabled' and param in msg.changed_parameters:
                    if (param.value.type == ParameterType.PARAMETER_BOOL and
                            not param.value.bool_value):
                        self.ui_diagnostics.pop(topic_name, None)
                    continue

                value = param_value_to_python(param.value)
                if not isinstance(value, (int, float)):
                    continue
                value = float(value)

                current = self.expected_frequencies.get(topic_name, (0.0, 0.0))

                if field == 'freq':
                    # Treat NaN or non-positive as "cleared"
                    if value > 0 or math.isnan(value):
                        self.expected_frequencies[topic_name] = (value, current[1])
                elif field == 'tol':
                    if current[0] > 0:  # Only update if frequency is set
                        self.expected_frequencies[topic_name] = (current[0], value)

        for param in msg.deleted_parameters:
            topic_name, field = parse_topic_param_name(param.name)
            if not topic_name or field == '':
                continue

            with self.data_lock:
                if field == 'enabled':
                    # Remove the topic in the map when there is no longer a parameter for it
                    self.topic_to_node.pop(topic_name, None)
                    self.ui_diagnostics.pop(topic_name, None)
                elif field == 'freq':
                    self.expected_frequencies.pop(topic_name, None)
                elif field == 'tol':
                    current = self.expected_frequencies.get(topic_name)
                    if current and current[0] > 0:
                        self.expected_frequencies[topic_name] = (
                            current[0], DEFAULT_TOLERANCE_PERCENT)

    def toggle_topic_monitoring(self, topic_name: str):
        """Toggle monitoring for a topic via enabled parameter."""
        with self.data_lock:
            is_monitoring = topic_name in self.ui_diagnostics
            target_node = self.topic_to_node.get(topic_name, self.monitor_node_name)
        new_enabled = not is_monitoring
        action = 'start' if new_enabled else 'stop'

        enabled_param = f'{TOPIC_PARAM_PREFIX}{topic_name}{ENABLED_SUFFIX}'
        success, failures = set_ros_parameters(
            self.node, target_node, {enabled_param: new_enabled})

        if not success:
            self.node.get_logger().error(
                f'Failed to {action} monitoring: {"; ".join(failures)}')
            return

        with self.data_lock:
            if not new_enabled and topic_name in self.ui_diagnostics:
                self.ui_diagnostics.pop(topic_name, None)

    def set_expected_frequency(self,
                               topic_name: str,
                               expected_hz: float = 0.0,
                               tolerance_percent: float = 0.0,
                               clear: bool = False
                               ) -> tuple[bool, str]:
        """Set or clear the expected frequency for a topic via parameters."""
        with self.data_lock:
            target_node = self.topic_to_node.get(topic_name, self.monitor_node_name)
        action = 'clear' if clear else 'set'

        params = {
            make_freq_param(topic_name): float('nan') if clear else expected_hz,
            make_tol_param(topic_name): DEFAULT_TOLERANCE_PERCENT if clear else tolerance_percent,
        }

        success, failures = set_ros_parameters(self.node, target_node, params)

        if not success:
            return False, f'Failed to {action} expected frequency: {"; ".join(failures)}'

        with self.data_lock:
            if clear:
                self.expected_frequencies.pop(topic_name, None)
            else:
                self.expected_frequencies[topic_name] = (expected_hz, tolerance_percent)

        return True, f'{"Cleared" if clear else "Set"} expected frequency for {topic_name}'

    def get_topic_diagnostics(self, topic_name: str) -> UiDiagnosticData:
        """Get diagnostic data for a topic. Returns default values if topic not found."""
        with self.data_lock:
            return self.ui_diagnostics.get(topic_name, UiDiagnosticData())

    def get_expected_frequency(self, topic_name: str) -> tuple[float, float]:
        """Get monitoring settings for a topic. Returns (0.0, 0.0) if not set."""
        with self.data_lock:
            return self.expected_frequencies.get(topic_name, (0.0, 0.0))

    def get_expected_frequency_str(self, topic_name: str) -> str:
        """Get expected frequency as formatted string with tolerance (e.g., '30.0Hz ±5%')."""
        freq, tol = self.get_expected_frequency(topic_name)
        if freq <= 0.0 or math.isnan(freq):
            return '-'
        if tol > 0.0:
            return f'{freq:.1f}Hz±{tol:.0f}%'
        return f'{freq:.1f}Hz'
