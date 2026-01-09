#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from abc import ABC
import math
import time
from typing import Any, List, Optional, Tuple
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from greenwave_monitor.ui_adaptor import (
    build_full_node_name,
    ENABLED_SUFFIX,
    FREQ_SUFFIX,
    get_ros_parameters,
    set_ros_parameters,
    TOL_SUFFIX,
    TOPIC_PARAM_PREFIX,
)
import launch_ros
from rcl_interfaces.msg import ParameterType, ParameterValue
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
MANAGE_TOPIC_TEST_CONFIG = TEST_CONFIGURATIONS[1]  # 100Hz imu
MONITOR_NODE_NAME = 'test_greenwave_monitor'
MONITOR_NODE_NAMESPACE = 'test_namespace'


def make_enabled_param(topic: str) -> str:
    """Build enabled parameter name for a topic."""
    return f'{TOPIC_PARAM_PREFIX}{topic}{ENABLED_SUFFIX}'


def set_parameter(test_node: Node, param_name: str, value,
                  node_name: str = MONITOR_NODE_NAME,
                  node_namespace: str = MONITOR_NODE_NAMESPACE,
                  timeout_sec: float = 10.0) -> bool:
    """Set a parameter on a node using rclpy service client."""
    full_node_name = build_full_node_name(node_name, node_namespace)
    success, _ = set_ros_parameters(test_node, full_node_name, {param_name: value}, timeout_sec)
    return success


def get_parameter(test_node: Node, param_name: str,
                  node_name: str = MONITOR_NODE_NAME,
                  node_namespace: str = MONITOR_NODE_NAMESPACE) -> Tuple[bool, Any]:
    """Get a parameter from a node using rclpy service client."""
    full_node_name = build_full_node_name(node_name, node_namespace)
    result = get_ros_parameters(test_node, full_node_name, [param_name])
    value = result.get(param_name)
    return (value is not None, value)


def delete_parameter(test_node: Node, param_name: str,
                     node_name: str = MONITOR_NODE_NAME,
                     node_namespace: str = MONITOR_NODE_NAMESPACE,
                     timeout_sec: float = 10.0) -> bool:
    """Delete a parameter from a node using rclpy service client."""
    full_node_name = build_full_node_name(node_name, node_namespace)
    not_set = ParameterValue(type=ParameterType.PARAMETER_NOT_SET)
    success, _ = set_ros_parameters(test_node, full_node_name, {param_name: not_set}, timeout_sec)
    return success


def create_minimal_publisher(
        topic: str, frequency_hz: float, message_type: str, id_suffix: str = '',
        enable_diagnostics: bool = True):
    """Create a minimal publisher node with the given parameters."""
    return launch_ros.actions.Node(
        package='greenwave_monitor',
        executable='minimal_publisher_node',
        name=f'minimal_publisher_node{id_suffix}',
        parameters=[{
            'topic': topic,
            'frequency_hz': frequency_hz,
            'message_type': message_type,
            'enable_greenwave_diagnostics': enable_diagnostics
        }],
        output='screen'
    )


def create_monitor_node(namespace: str = MONITOR_NODE_NAMESPACE,
                        node_name: str = MONITOR_NODE_NAME,
                        topics: List[str] = None,
                        topic_configs: dict = None):
    """Create a greenwave_monitor node for testing."""
    params = {}

    # Only add topics param if explicitly provided or no topic_configs
    if topics is not None:
        if not topics:
            topics = ['']
        params['topics'] = topics
    elif not topic_configs:
        params['topics'] = ['/test_topic']

    if topic_configs:
        for topic, config in topic_configs.items():
            if 'expected_frequency' in config:
                params[f'{TOPIC_PARAM_PREFIX}{topic}{FREQ_SUFFIX}'] = float(
                    config['expected_frequency'])
            if 'tolerance' in config:
                params[f'{TOPIC_PARAM_PREFIX}{topic}{TOL_SUFFIX}'] = float(config['tolerance'])

    return launch_ros.actions.Node(
        package='greenwave_monitor',
        executable='greenwave_monitor',
        name=node_name,
        namespace=namespace,
        parameters=[params],
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


class RosNodeTestCase(unittest.TestCase, ABC):
    """
    Abstract base class for ROS 2 launch tests that need a test node.

    Subclasses must define the TEST_NODE_NAME class attribute to specify
    the unique name for the test node.
    """

    TEST_NODE_NAME: str = None

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 and create test node."""
        if cls.TEST_NODE_NAME is None:
            raise ValueError(
                f'{cls.__name__} must define TEST_NODE_NAME class attribute')
        rclpy.init()
        cls.test_node = Node(cls.TEST_NODE_NAME, namespace=MONITOR_NODE_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS 2."""
        cls.test_node.destroy_node()
        rclpy.shutdown()
