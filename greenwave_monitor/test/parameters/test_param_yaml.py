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

"""Test: load topic configuration from YAML parameter file."""

import os
import tempfile
import time
import unittest

from greenwave_monitor.test_utils import (
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    find_best_diagnostic,
    MONITOR_NODE_NAME,
    MONITOR_NODE_NAMESPACE,
    RosNodeTestCase,
)
import launch
import launch_ros.actions
import launch_testing
import pytest


YAML_TOPIC = '/yaml_config_topic'
NESTED_TOPIC = '/nested_yaml_topic'
TEST_FREQUENCY = 50.0
NESTED_FREQUENCY = 25.0
TEST_TOLERANCE = 10.0


@pytest.mark.launch_test
def generate_test_description():
    """Test loading parameters from a YAML file."""
    # Write YAML manually - demonstrates both flat dotted keys and nested dict formats
    # Use full namespace path for node parameters
    yaml_content = (
        f'/{MONITOR_NODE_NAMESPACE}/{MONITOR_NODE_NAME}:\n'
        f'  ros__parameters:\n'
        f'    # Flat dotted key format (requires quotes)\n'
        f'    "topics.{YAML_TOPIC}.expected_frequency": {TEST_FREQUENCY}\n'
        f'    "topics.{YAML_TOPIC}.tolerance": {TEST_TOLERANCE}\n'
        f'    # Nested dictionary format\n'
        f'    topics:\n'
        f'      {NESTED_TOPIC}:\n'
        f'        expected_frequency: {NESTED_FREQUENCY}\n'
        f'        tolerance: {TEST_TOLERANCE}\n'
    )

    yaml_dir = tempfile.mkdtemp()
    yaml_path = os.path.join(yaml_dir, 'test_params.yaml')
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)

    ros2_monitor_node = launch_ros.actions.Node(
        package='greenwave_monitor',
        executable='greenwave_monitor',
        name=MONITOR_NODE_NAME,
        namespace=MONITOR_NODE_NAMESPACE,
        parameters=[yaml_path],
        output='screen'
    )

    publisher = create_minimal_publisher(
        YAML_TOPIC, TEST_FREQUENCY, 'imu', '_yaml'
    )

    nested_publisher = create_minimal_publisher(
        NESTED_TOPIC, NESTED_FREQUENCY, 'imu', '_nested_yaml'
    )

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            publisher,
            nested_publisher,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


class TestYamlParameterFile(RosNodeTestCase):
    """Test loading topic configuration from YAML parameter file."""

    TEST_NODE_NAME = 'yaml_test_node'

    def test_topic_configured_via_yaml(self):
        """Test that topic is monitored when configured via YAML file."""
        time.sleep(2.0)

        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, YAML_TOPIC, expected_count=3, timeout_sec=10.0
        )

        self.assertGreaterEqual(
            len(received_diagnostics), 3,
            'Expected diagnostics from YAML-configured topic'
        )
        best_status, _ = find_best_diagnostic(
            received_diagnostics, TEST_FREQUENCY, 'imu'
        )
        self.assertIsNotNone(
            best_status,
            'Should have valid frame rate from YAML config'
        )

    def test_nested_dict_topic_configured_via_yaml(self):
        """Test that topic configured via nested YAML dict is monitored."""
        time.sleep(2.0)

        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, NESTED_TOPIC, expected_count=3, timeout_sec=10.0
        )

        self.assertGreaterEqual(
            len(received_diagnostics), 3,
            'Expected diagnostics from nested YAML-configured topic'
        )
        best_status, _ = find_best_diagnostic(
            received_diagnostics, NESTED_FREQUENCY, 'imu'
        )
        self.assertIsNotNone(
            best_status,
            'Should have valid frame rate from nested YAML config'
        )


if __name__ == '__main__':
    unittest.main()
