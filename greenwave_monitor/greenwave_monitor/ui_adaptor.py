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

In addition to passively subscribing, `GreenwaveUiAdaptor` exposes clients for two
services on the monitor node:
- ManageTopic: start/stop monitoring a topic (`toggle_topic_monitoring`).
- SetExpectedFrequency: set/clear the expected publish rate and tolerance for a topic
  (`set_expected_frequency`). Expected rates are also cached locally in
  `expected_frequencies` as `(expected_hz, tolerance_percent)` so UIs can display the
  configured values alongside live diagnostics.
"""

from dataclasses import dataclass
from enum import Enum
import threading
import time
from typing import Dict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from greenwave_monitor_interfaces.srv import ManageTopic, SetExpectedFrequency
from rcl_interfaces.msg import ParameterEvent, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters
import rclpy
from rclpy.node import Node

# Parameter name constants
TOPIC_PARAM_PREFIX = 'topics.'
FREQ_SUFFIX = '.expected_frequency'
TOL_SUFFIX = '.tolerance'
DEFAULT_TOLERANCE_PERCENT = 5.0


class TopicParamField(Enum):
    """Type of topic parameter field."""

    NONE = 0
    FREQUENCY = 1
    TOLERANCE = 2


def parse_topic_param_name(param_name: str) -> tuple[str, TopicParamField]:
    """Parse parameter name to extract topic name and field type."""
    if not param_name.startswith(TOPIC_PARAM_PREFIX):
        return ('', TopicParamField.NONE)

    topic_and_field = param_name[len(TOPIC_PARAM_PREFIX):]

    if topic_and_field.endswith(FREQ_SUFFIX):
        topic_name = topic_and_field[:-len(FREQ_SUFFIX)]
        return (topic_name, TopicParamField.FREQUENCY)
    elif topic_and_field.endswith(TOL_SUFFIX):
        topic_name = topic_and_field[:-len(TOL_SUFFIX)]
        return (topic_name, TopicParamField.TOLERANCE)

    return ('', TopicParamField.NONE)


def param_value_to_float(value: ParameterValue) -> float | None:
    """Convert a ParameterValue to float if it's a numeric type."""
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    elif value.type == ParameterType.PARAMETER_INTEGER:
        return float(value.integer_value)
    return None


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
        if status.level == DiagnosticStatus.OK:
            data.status = 'OK'
        elif status.level == DiagnosticStatus.WARN:
            data.status = 'WARN'
        elif status.level == DiagnosticStatus.ERROR:
            data.status = 'ERROR'
        elif status.level == DiagnosticStatus.STALE:
            data.status = 'STALE'
        else:
            data.status = 'UNKNOWN'

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
    provides a toggle for monitoring via `ManageTopic`, and exposes helpers to set/clear
    expected frequencies via `SetExpectedFrequency`. Service names may be discovered
    dynamically or constructed from an optional namespace and node name.

    """

    def __init__(self, node: Node, monitor_node_name: str = 'greenwave_monitor'):
        """Initialize the UI adaptor for subscribing to diagnostics and managing topics."""
        self.node = node
        self.monitor_node_name = monitor_node_name
        self.data_lock = threading.Lock()
        self.ui_diagnostics: Dict[str, UiDiagnosticData] = {}
        # { topic_name : (expected_hz, tolerance) }
        self.expected_frequencies: Dict[str, tuple[float, float]] = {}

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

        manage_service_name = f'{self.monitor_node_name}/manage_topic'
        set_freq_service_name = f'{self.monitor_node_name}/set_expected_frequency'

        self.node.get_logger().info(f'Connecting to monitor service: {manage_service_name}')

        self.manage_topic_client = self.node.create_client(
            ManageTopic,
            manage_service_name
        )

        self.set_expected_frequency_client = self.node.create_client(
            SetExpectedFrequency,
            set_freq_service_name
        )

        # Parameter service clients for querying initial state
        self.list_params_client = self.node.create_client(
            ListParameters,
            f'{self.monitor_node_name}/list_parameters'
        )
        self.get_params_client = self.node.create_client(
            GetParameters,
            f'{self.monitor_node_name}/get_parameters'
        )

        # Query initial parameters after a short delay to let services come up
        self._initial_params_timer = self.node.create_timer(
            0.1, self._fetch_initial_parameters_callback)

    def _fetch_initial_parameters_callback(self):
        """Timer callback to fetch initial parameters (retries until services ready)."""
        # Check if services are available (non-blocking)
        if not self.list_params_client.service_is_ready():
            return  # Timer will retry

        if not self.get_params_client.service_is_ready():
            return  # Timer will retry

        # Cancel timer now that services are ready
        if self._initial_params_timer is not None:
            self._initial_params_timer.cancel()
            self._initial_params_timer = None

        # List all parameters (prefixes filter is unreliable with nested names)
        list_request = ListParameters.Request()
        list_request.prefixes = ['topics']
        list_request.depth = 10

        list_future = self.list_params_client.call_async(list_request)
        list_future.add_done_callback(self._on_list_parameters_response)

    def _on_list_parameters_response(self, future):
        """Handle response from list_parameters service."""
        try:
            if future.result() is None:
                return

            all_param_names = future.result().result.names

            # Filter to only topic parameters (prefixes filter is unreliable)
            param_names = [n for n in all_param_names if n.startswith(TOPIC_PARAM_PREFIX)]

            if not param_names:
                return

            # Store for use in get callback
            self._pending_param_names = param_names

            # Get values for topic parameters only
            get_request = GetParameters.Request()
            get_request.names = param_names

            get_future = self.get_params_client.call_async(get_request)
            get_future.add_done_callback(self._on_get_parameters_response)
        except Exception as e:
            self.node.get_logger().debug(f'Error listing parameters: {e}')

    def _on_get_parameters_response(self, future):
        """Handle response from get_parameters service."""
        try:
            if future.result() is None:
                return

            param_names = getattr(self, '_pending_param_names', [])
            values = future.result().values

            # Parse parameters into expected_frequencies
            topic_configs: Dict[str, Dict[str, float]] = {}

            for name, value in zip(param_names, values):
                numeric_value = param_value_to_float(value)
                if numeric_value is None:
                    continue

                topic_name, field = parse_topic_param_name(name)
                if not topic_name or field == TopicParamField.NONE:
                    continue

                if topic_name not in topic_configs:
                    topic_configs[topic_name] = {}

                if field == TopicParamField.FREQUENCY:
                    topic_configs[topic_name]['freq'] = numeric_value
                elif field == TopicParamField.TOLERANCE:
                    topic_configs[topic_name]['tol'] = numeric_value

            # Update expected_frequencies with valid configs
            with self.data_lock:
                for topic_name, config in topic_configs.items():
                    freq = config.get('freq', 0.0)
                    tol = config.get('tol', DEFAULT_TOLERANCE_PERCENT)
                    if freq > 0:
                        self.expected_frequencies[topic_name] = (freq, tol)

        except Exception as e:
            self.node.get_logger().debug(f'Error fetching parameters: {e}')

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
        """Process parameter change events from the monitor node."""
        # Only process events from the monitor node
        if self.monitor_node_name not in msg.node:
            return

        # Process changed and new parameters
        for param in msg.changed_parameters + msg.new_parameters:
            value = param_value_to_float(param.value)
            if value is None:
                continue

            topic_name, field = parse_topic_param_name(param.name)
            if not topic_name or field == TopicParamField.NONE:
                continue

            with self.data_lock:
                current = self.expected_frequencies.get(topic_name, (0.0, 0.0))

                if field == TopicParamField.FREQUENCY:
                    if value > 0:
                        self.expected_frequencies[topic_name] = (value, current[1])
                    elif topic_name in self.expected_frequencies:
                        del self.expected_frequencies[topic_name]

                elif field == TopicParamField.TOLERANCE:
                    if current[0] > 0:  # Only update if frequency is set
                        self.expected_frequencies[topic_name] = (current[0], value)

    def toggle_topic_monitoring(self, topic_name: str):
        """Toggle monitoring for a topic."""
        if not self.manage_topic_client.wait_for_service(timeout_sec=1.0):
            return

        request = ManageTopic.Request()
        request.topic_name = topic_name

        with self.data_lock:
            request.add_topic = topic_name not in self.ui_diagnostics

        try:
            # Use asynchronous service call to prevent deadlock
            future = self.manage_topic_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)

            if future.result() is None:
                action = 'start' if request.add_topic else 'stop'
                self.node.get_logger().error(
                    f'Failed to {action} monitoring: Service call timed out')
                return

            response = future.result()

            with self.data_lock:
                if not response.success:
                    action = 'start' if request.add_topic else 'stop'
                    self.node.get_logger().error(
                        f'Failed to {action} monitoring: {response.message}')
                    return

                if not request.add_topic and topic_name in self.ui_diagnostics:
                    del self.ui_diagnostics[topic_name]
                    if topic_name in self.expected_frequencies:
                        del self.expected_frequencies[topic_name]

        except Exception as e:
            action = 'start' if request.add_topic else 'stop'
            self.node.get_logger().error(f'Failed to {action} monitoring: {e}')

    def set_expected_frequency(self,
                               topic_name: str,
                               expected_hz: float = 0.0,
                               tolerance_percent: float = 0.0,
                               clear: bool = False
                               ) -> tuple[bool, str]:
        """Set or clear the expected frequency for a topic."""
        if not self.set_expected_frequency_client.wait_for_service(timeout_sec=1.0):
            return False, 'Could not connect to set_expected_frequency service.'

        request = SetExpectedFrequency.Request()
        request.topic_name = topic_name
        request.expected_hz = expected_hz
        request.tolerance_percent = tolerance_percent
        request.clear_expected = clear
        request.add_topic_if_missing = True

        # Use asynchronous service call to prevent deadlock
        try:
            future = self.set_expected_frequency_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)

            if future.result() is None:
                action = 'clear' if clear else 'set'
                error_msg = f'Failed to {action} expected frequency: Service call timed out'
                self.node.get_logger().error(error_msg)
                return False, error_msg

            response = future.result()

            if not response.success:
                action = 'clear' if clear else 'set'
                self.node.get_logger().error(
                    f'Failed to {action} expected frequency: {response.message}')
                return False, response.message
            else:
                with self.data_lock:
                    if clear:
                        self.expected_frequencies.pop(topic_name, None)
                    else:
                        self.expected_frequencies[topic_name] = (expected_hz, tolerance_percent)
                return True, response.message
        except Exception as e:
            action = 'clear' if clear else 'set'
            error_msg = f'Failed to {action} expected frequency: {e}'
            self.node.get_logger().error(error_msg)
            return False, error_msg

    def get_topic_diagnostics(self, topic_name: str) -> UiDiagnosticData:
        """Get diagnostic data for a topic. Returns default values if topic not found."""
        with self.data_lock:
            return self.ui_diagnostics.get(topic_name, UiDiagnosticData())

    def get_expected_frequency(self, topic_name: str) -> tuple[float, float]:
        """Get monitoring settings for a topic. Returns (0.0, 0.0) if not set."""
        with self.data_lock:
            return self.expected_frequencies.get(topic_name, (0.0, 0.0))
