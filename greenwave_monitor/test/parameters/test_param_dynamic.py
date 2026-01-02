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

"""Test: dynamic parameter changes via ros2 param set."""

import subprocess
import time
import unittest

from greenwave_monitor.test_utils import (
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    create_monitor_node,
    MONITOR_NODE_NAME,
    MONITOR_NODE_NAMESPACE
)
from greenwave_monitor.ui_adaptor import (
    FREQ_SUFFIX,
    TOL_SUFFIX,
    TOPIC_PARAM_PREFIX,
)
import launch
import launch_testing
import pytest
import rclpy
from rclpy.node import Node


TEST_TOPIC = '/dynamic_param_topic'
TEST_FREQUENCY = 30.0


def run_ros2_param_set(node_name: str, param_name: str, value: float) -> bool:
    """Run ros2 param set command and return success status."""
    full_node_name = f'/{MONITOR_NODE_NAMESPACE}/{node_name}'
    cmd = ['ros2', 'param', 'set', full_node_name, param_name, str(value)]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10.0)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def run_ros2_param_get(node_name: str, param_name: str) -> tuple[bool, float | None]:
    """Run ros2 param get command and return (success, value)."""
    full_node_name = f'/{MONITOR_NODE_NAMESPACE}/{node_name}'
    cmd = ['ros2', 'param', 'get', full_node_name, param_name]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10.0)
        if result.returncode != 0:
            return False, None
        # Parse output like "Double value is: 30.0" or "Integer value is: 30"
        output = result.stdout.strip()
        if 'value is:' in output:
            value_str = output.split('value is:')[1].strip()
            return True, float(value_str)
        return False, None
    except (subprocess.TimeoutExpired, ValueError):
        return False, None


def make_freq_param(topic: str) -> str:
    """Build frequency parameter name for a topic."""
    return f'{TOPIC_PARAM_PREFIX}{topic}{FREQ_SUFFIX}'


def make_tol_param(topic: str) -> str:
    """Build tolerance parameter name for a topic."""
    return f'{TOPIC_PARAM_PREFIX}{topic}{TOL_SUFFIX}'


@pytest.mark.launch_test
def generate_test_description():
    """Test dynamic parameter changes via ros2 param set."""
    ros2_monitor_node = create_monitor_node(
        namespace=MONITOR_NODE_NAMESPACE,
        node_name=MONITOR_NODE_NAME,
        topics=[TEST_TOPIC],  # Topic exists but no expected frequency
        topic_configs={}
    )

    publisher = create_minimal_publisher(
        TEST_TOPIC, TEST_FREQUENCY, 'imu', '_dynamic'
    )

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            publisher,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


class TestDynamicParameterChanges(unittest.TestCase):
    """Test changing parameters dynamically via ros2 param set."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node."""
        rclpy.init()
        cls.test_node = Node('dynamic_param_test_node', namespace=MONITOR_NODE_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2."""
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def test_set_expected_frequency_via_param(self):
        """Test setting expected frequency via ros2 param set."""
        time.sleep(2.0)

        freq_param = make_freq_param(TEST_TOPIC)
        success = run_ros2_param_set(MONITOR_NODE_NAME, freq_param, TEST_FREQUENCY)
        self.assertTrue(success, f'Failed to set {freq_param}')

        time.sleep(1.0)

        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC, expected_count=3, timeout_sec=10.0
        )

        self.assertGreaterEqual(
            len(received_diagnostics), 3,
            'Expected diagnostics after setting frequency param'
        )

    def test_change_tolerance_via_param(self):
        """Test changing tolerance via ros2 param set."""
        time.sleep(1.0)

        tol_param = make_tol_param(TEST_TOPIC)
        success = run_ros2_param_set(MONITOR_NODE_NAME, tol_param, 20.0)
        self.assertTrue(success, f'Failed to set {tol_param}')

        time.sleep(0.5)

        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC, expected_count=2, timeout_sec=5.0
        )

        self.assertGreaterEqual(
            len(received_diagnostics), 2,
            'Topic should still be monitored after tolerance change'
        )

    def test_verify_params_with_get(self):
        """Test that ros2 param get returns the values we set."""
        time.sleep(1.0)

        # Set specific values
        freq_param = make_freq_param(TEST_TOPIC)
        tol_param = make_tol_param(TEST_TOPIC)
        expected_freq = 42.5
        expected_tol = 15.0

        success = run_ros2_param_set(MONITOR_NODE_NAME, freq_param, expected_freq)
        self.assertTrue(success, f'Failed to set {freq_param}')

        success = run_ros2_param_set(MONITOR_NODE_NAME, tol_param, expected_tol)
        self.assertTrue(success, f'Failed to set {tol_param}')

        time.sleep(0.5)

        # Verify with ros2 param get
        success, actual_freq = run_ros2_param_get(MONITOR_NODE_NAME, freq_param)
        self.assertTrue(success, f'Failed to get {freq_param}')
        self.assertAlmostEqual(
            actual_freq, expected_freq, places=1,
            msg=f'Frequency mismatch: expected {expected_freq}, got {actual_freq}'
        )

        success, actual_tol = run_ros2_param_get(MONITOR_NODE_NAME, tol_param)
        self.assertTrue(success, f'Failed to get {tol_param}')
        self.assertAlmostEqual(
            actual_tol, expected_tol, places=1,
            msg=f'Tolerance mismatch: expected {expected_tol}, got {actual_tol}'
        )


if __name__ == '__main__':
    unittest.main()
