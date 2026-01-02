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

"""Tests for parameter-based topic configuration - both frequency and tolerance."""

import time
import unittest

from greenwave_monitor.test_utils import (
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    create_monitor_node,
    find_best_diagnostic,
    MONITOR_NODE_NAME,
    MONITOR_NODE_NAMESPACE,
)
import launch
import launch_testing
from launch_testing import post_shutdown_test
from launch_testing.asserts import assertExitCodes
import pytest
import rclpy
from rclpy.node import Node


TEST_TOPIC = '/param_test_topic'
TEST_FREQUENCY = 50.0
TEST_TOLERANCE = 10.0


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description with both frequency and tolerance set."""
    topic_configs = {
        TEST_TOPIC: {
            'expected_frequency': TEST_FREQUENCY,
            'tolerance': TEST_TOLERANCE
        }
    }

    ros2_monitor_node = create_monitor_node(
        topic_configs=topic_configs
    )

    publisher = create_minimal_publisher(
        TEST_TOPIC, TEST_FREQUENCY, 'imu', '_param_test'
    )

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            publisher,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


@post_shutdown_test()
class TestTopicParametersPostShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node."""
        rclpy.init()
        cls.test_node = Node('shutdown_test_node', namespace=MONITOR_NODE_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2."""
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def test_node_shutdown(self, proc_info):
        """Test that the node shuts down correctly."""
        available_nodes = self.test_node.get_node_names()
        self.assertNotIn(MONITOR_NODE_NAME, available_nodes)
        assertExitCodes(proc_info, allowable_exit_codes=[0])


class TestTopicParameters(unittest.TestCase):
    """Tests for parameter-based topic configuration."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node."""
        rclpy.init()
        cls.test_node = Node('topic_params_test_node', namespace=MONITOR_NODE_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2."""
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def test_topic_configured_via_parameters(self):
        """Test that topic is monitored when configured via parameters."""
        time.sleep(2.0)

        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC, expected_count=3, timeout_sec=10.0
        )

        self.assertGreaterEqual(
            len(received_diagnostics), 3,
            f'Expected at least 3 diagnostics for {TEST_TOPIC}, got {len(received_diagnostics)}'
        )
        best_status, best_values = find_best_diagnostic(
            received_diagnostics, TEST_FREQUENCY, 'imu'
        )
        self.assertIsNotNone(
            best_status,
            'Should have received diagnostics with valid frame_rate_node'
        )
        frame_rate_node = best_values[0]
        tolerance = TEST_FREQUENCY * 0.5
        self.assertAlmostEqual(
            frame_rate_node, TEST_FREQUENCY, delta=tolerance,
            msg=f'Frame rate {frame_rate_node} not within {tolerance} of {TEST_FREQUENCY}'
        )


if __name__ == '__main__':
    unittest.main()
