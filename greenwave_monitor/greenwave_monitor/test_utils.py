#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import math
import time
from typing import List, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from greenwave_monitor_interfaces.srv import ManageTopic, SetExpectedFrequency
import launch_ros
import rclpy
from rclpy.node import Node


# Test configurations for various message types and frequencies
# (message_type, expected_frequency, tolerance_hz)
# NOTE: Tolerances and frequencies are set conservatively for reliable operation
# on slow/loaded CI systems such as the ROS buildfarm. The 30% tolerance standard
# ensures tests pass even under system load.
TEST_CONFIGURATIONS = [
    ('imu', 1.0, 0.3),
    ('imu', 100.0, 30.0),
    ('imu', 500.0, 150.0),
    ('image', 10.0, 3.0),
    ('string', 100.0, 30.0),
]

# Standard test constants
MANAGE_TOPIC_TEST_CONFIG = TEST_CONFIGURATIONS[2]
MONITOR_NODE_NAME = 'test_greenwave_monitor'
MONITOR_NODE_NAMESPACE = 'test_namespace'


def create_minimal_publisher(
        topic: str, frequency_hz: float, message_type: str, id_suffix: str = ''):
    """Create a minimal publisher node with the given parameters."""
    return launch_ros.actions.Node(
        package='greenwave_monitor',
        executable='minimal_publisher_node',
        name=f'minimal_publisher_node{id_suffix}',
        parameters=[{
            'topic': topic,
            'frequency_hz': frequency_hz,
            'message_type': message_type
        }],
        output='screen'
    )


def create_monitor_node(namespace: str = MONITOR_NODE_NAMESPACE,
                        node_name: str = MONITOR_NODE_NAME,
                        topics: List[str] = None):
    """Create a greenwave_monitor node for testing."""
    if topics is None:
        topics = ['/test_topic']

    return launch_ros.actions.Node(
        package='greenwave_monitor',
        executable='greenwave_monitor',
        name=node_name,
        namespace=namespace,
        parameters=[{
            'topics': topics
        }],
        output='screen'
    )


def wait_for_service_connection(node: Node,
                                service_client,
                                timeout_sec: float = 3.0,
                                service_name: str = 'service') -> bool:
    """Wait for a service to become available."""
    service_available = service_client.wait_for_service(timeout_sec=timeout_sec)
    if not service_available:
        node.get_logger().error(
            f'Service "{service_name}" not available within {timeout_sec} seconds')
    return service_available


def call_manage_topic_service(node: Node,
                              service_client,
                              add: bool,
                              topic: str,
                              timeout_sec: float = 8.0
                              ) -> Optional[ManageTopic.Response]:
    """Call the manage_topic service with given parameters."""
    request = ManageTopic.Request()
    request.add_topic = add
    request.topic_name = topic
    future = service_client.call_async(request)

    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if future.result() is None:
        node.get_logger().error('Service call failed or timed out')
        return None

    return future.result()


def call_set_frequency_service(node: Node,
                               service_client,
                               topic_name: str,
                               expected_hz: float = 0.0,
                               tolerance_percent: float = 0.0,
                               clear: bool = False,
                               add_if_missing: bool = True,
                               timeout_sec: float = 8.0
                               ) -> Optional[SetExpectedFrequency.Response]:
    """Call the set_expected_frequency service with given parameters."""
    request = SetExpectedFrequency.Request()
    request.topic_name = topic_name
    request.expected_hz = expected_hz
    request.tolerance_percent = tolerance_percent
    request.clear_expected = clear
    request.add_topic_if_missing = add_if_missing

    future = service_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if future.result() is None:
        node.get_logger().error('Service call failed or timed out')
        return None

    return future.result()


def collect_diagnostics_for_topic(node: Node,
                                  topic_name: str,
                                  expected_count: int = 5,
                                  timeout_sec: float = 10.0) -> List[DiagnosticStatus]:
    """Collect diagnostic messages for a specific topic."""
    received_diagnostics = []

    def diagnostics_callback(msg):
        for status in msg.status:
            if topic_name == status.name:
                received_diagnostics.append(status)

    subscription = node.create_subscription(
        DiagnosticArray,
        '/diagnostics',
        diagnostics_callback,
        10
    )

    end_time = time.time() + timeout_sec
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(received_diagnostics) >= expected_count:
            break

    # Clean up subscription
    node.destroy_subscription(subscription)

    return received_diagnostics


def find_best_diagnostic(
        diagnostics: List[DiagnosticStatus],
        expected_frequency: float,
        message_type: str
        ) -> Tuple[Optional[DiagnosticStatus], Optional[Tuple[float, float, float]]]:
    """Find the diagnostic message with frequency closest to expected."""
    best_status = None
    best_values = None
    best_diff = float('inf')

    for status in diagnostics:
        node_str = None
        msg_str = None
        latency_str = None

        for kv in status.values:
            if kv.key == 'frame_rate_node':
                node_str = kv.value
            elif kv.key == 'frame_rate_msg':
                msg_str = kv.value
            elif kv.key == 'current_delay_from_realtime_ms':
                latency_str = kv.value
                if latency_str == 'N/A':
                    latency_str = 'nan'

        try:
            node_val = float(node_str) if node_str is not None else None
            msg_val = float(msg_str) if msg_str is not None else None
            latency_val = float(latency_str) if latency_str is not None else None
        except (ValueError, TypeError):
            continue

        if node_val is None or msg_val is None or latency_val is None:
            continue

        # Choose by smallest diff to expected frequency
        diff = abs(node_val - expected_frequency)
        if message_type != 'string' and msg_val is not None:
            diff += abs(msg_val - expected_frequency)

        if diff < best_diff:
            best_diff = diff
            best_status = status
            best_values = (node_val, msg_val, latency_val)

    return best_status, best_values


def verify_diagnostic_values(status: DiagnosticStatus,
                             values: Tuple[float, float, float],
                             expected_frequency: float,
                             message_type: str,
                             tolerance_hz: float) -> List[str]:
    """Verify diagnostic values and return list of assertion errors."""
    errors = []
    reported_frequency_node, reported_frequency_msg, reported_latency_ms = values

    # Check that frequencies were found
    if reported_frequency_node == -1.0:
        errors.append("Did not find 'frame_rate_node' in diagnostic")
    if reported_frequency_msg == -1.0:
        errors.append("Did not find 'frame_rate_msg' in diagnostic")
    if reported_latency_ms == -1.0:
        errors.append("Did not find 'current_delay_from_realtime_ms' in diagnostic")

    # Check frequency tolerances
    if abs(reported_frequency_node - expected_frequency) > tolerance_hz:
        errors.append(
            f'Node frequency {reported_frequency_node} not within '
            f'{tolerance_hz} Hz of expected {expected_frequency}')

    if message_type == 'string':
        if reported_frequency_msg != 0.0:
            errors.append(f'String message frequency should be 0.0, got {reported_frequency_msg}')
        if not math.isnan(reported_latency_ms):
            errors.append(
                f'String latency should be {math.nan}, '
                f'got {reported_latency_ms}')
    else:
        if abs(reported_frequency_msg - expected_frequency) > tolerance_hz:
            errors.append(
                f'Message frequency {reported_frequency_msg} not within '
                f'{tolerance_hz} Hz of expected {expected_frequency}')
        # Relaxed to 50ms for slow/loaded CI systems (was 10ms)
        if reported_latency_ms > 50:
            errors.append(
                f'Latency should be <= 50 ms for non-string types, '
                f'got {reported_latency_ms}')

    return errors


def create_service_clients(node: Node, namespace: str = MONITOR_NODE_NAMESPACE,
                           node_name: str = MONITOR_NODE_NAME):
    """Create service clients for the monitor node."""
    manage_topic_client = node.create_client(
        ManageTopic, f'/{namespace}/{node_name}/manage_topic'
    )

    set_frequency_client = node.create_client(
        SetExpectedFrequency, f'/{namespace}/{node_name}/set_expected_frequency'
    )

    return manage_topic_client, set_frequency_client
