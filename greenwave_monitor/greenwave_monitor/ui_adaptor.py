#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import threading
import time
from typing import Dict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from greenwave_monitor_interfaces.srv import ManageTopic, SetExpectedFrequency
import rclpy
from rclpy.node import Node


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
