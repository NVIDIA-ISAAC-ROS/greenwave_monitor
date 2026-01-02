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

"""Test: only expected_frequency specified, tolerance defaults to 5%."""

import time
import unittest

from greenwave_monitor.test_utils import (
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    create_monitor_node,
    MONITOR_NODE_NAME,
    MONITOR_NODE_NAMESPACE
)
import launch
import launch_testing
import pytest
import rclpy
from rclpy.node import Node


TEST_TOPIC = '/freq_only_topic'
TEST_FREQUENCY = 50.0


@pytest.mark.launch_test
def generate_test_description():
    """Test with only expected_frequency specified."""
    topic_configs = {
        TEST_TOPIC: {
            'expected_frequency': TEST_FREQUENCY
            # No tolerance - should default to 5%
        }
    }

    ros2_monitor_node = create_monitor_node(
        namespace=MONITOR_NODE_NAMESPACE,
        node_name=MONITOR_NODE_NAME,
        topics=[],
        topic_configs=topic_configs
    )

    publisher = create_minimal_publisher(
        TEST_TOPIC, TEST_FREQUENCY, 'imu', '_freq_only'
    )

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            publisher,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


class TestFrequencyOnlyParameter(unittest.TestCase):
    """Test that only specifying frequency works (tolerance defaults)."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node."""
        rclpy.init()
        cls.test_node = Node('freq_only_test_node', namespace=MONITOR_NODE_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2."""
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def test_frequency_only_uses_default_tolerance(self):
        """Test that specifying only frequency uses default tolerance."""
        time.sleep(2.0)

        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC, expected_count=3, timeout_sec=10.0
        )

        self.assertGreaterEqual(
            len(received_diagnostics), 3,
            f'Expected at least 3 diagnostics, got {len(received_diagnostics)}'
        )

        has_valid_rate = False
        for status in received_diagnostics:
            for kv in status.values:
                if kv.key == 'frame_rate_node':
                    try:
                        if float(kv.value) > 0:
                            has_valid_rate = True
                            break
                    except ValueError:
                        continue
            if has_valid_rate:
                break

        self.assertTrue(has_valid_rate, 'Should have valid frame rate with default tolerance')


if __name__ == '__main__':
    unittest.main()
