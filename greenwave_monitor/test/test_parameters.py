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

"""Consolidated tests for parameter-based topic configuration."""

import os
import tempfile
import time
import unittest

from greenwave_monitor.test_utils import (
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    delete_parameter,
    find_best_diagnostic,
    get_parameter,
    make_enabled_param,
    MONITOR_NODE_NAME,
    MONITOR_NODE_NAMESPACE,
    RosNodeTestCase,
    set_parameter,
)
from greenwave_monitor.ui_adaptor import (
    ENABLED_SUFFIX,
    make_freq_param,
    make_tol_param,
    TOPIC_PARAM_PREFIX,
)
import launch
import launch_ros.actions
import launch_testing
from launch_testing import post_shutdown_test
from launch_testing.asserts import assertExitCodes
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy


# Topic configurations for all tests
ENABLED_FALSE_TOPIC = '/enabled_false_topic'
TOL_ONLY_TOPIC = '/tol_only_topic'
FREQ_ONLY_TOPIC = '/freq_only_topic'
FULL_CONFIG_TOPIC = '/full_config_topic'
DYNAMIC_TOPIC = '/dynamic_param_topic'
DYNAMIC_SET_PARAMS_TOPIC = '/dynamic_param_topic_set_params'
DYNAMIC_DELETE_TOPIC = '/dynamic_param_topic_delete_param'
NONEXISTENT_TOPIC = '/topic_that_does_not_exist'
MULTI_TOPIC_1 = '/multi_topic_1'
MULTI_TOPIC_2 = '/multi_topic_2'
MULTI_TOPIC_3 = '/multi_topic_3'
MULTI_LIST_TOPIC_1 = '/multi_topic_list_1'
MULTI_LIST_TOPIC_2 = '/multi_topic_list_2'
YAML_TOPIC = '/yaml_config_topic'
NESTED_YAML_TOPIC = '/nested_yaml_topic'
ENABLE_EXISTING_TOPIC = '/enable_existing_test_topic'
NEW_DYNAMIC_TOPIC = '/new_dynamic_topic'

# Test frequencies and tolerances
STD_FREQUENCY = 50.0
STD_TOLERANCE = 10.0
MULTI_FREQ_1 = 10.0
MULTI_FREQ_2 = 25.0
MULTI_FREQ_3 = 50.0
MULTI_TOLERANCE = 20.0
MULTI_LIST_FREQ = 30.0
YAML_FREQUENCY = 50.0
YAML_TOLERANCE = 10.0
NESTED_FREQUENCY = 25.0
DYNAMIC_FREQUENCY = 30.0
DYNAMIC_TOLERANCE = 20.0

# Publisher node names
PUBLISHER_DYNAMIC = 'minimal_publisher_node_dynamic'
PUBLISHER_SET_PARAMS = 'minimal_publisher_node_set_params'
PUBLISHER_ENABLE_TEST = 'minimal_publisher_node_enable_test'


def _make_yaml_file():
    """Create a temp YAML file for parameter loading test."""
    yaml_content = (
        f'/{MONITOR_NODE_NAMESPACE}/{MONITOR_NODE_NAME}:\n'
        f'  ros__parameters:\n'
        f'    "greenwave_diagnostics.{YAML_TOPIC}.expected_frequency": {YAML_FREQUENCY}\n'
        f'    "greenwave_diagnostics.{YAML_TOPIC}.tolerance": {YAML_TOLERANCE}\n'
        f'    greenwave_diagnostics:\n'
        f'      {NESTED_YAML_TOPIC}:\n'
        f'        expected_frequency: {NESTED_FREQUENCY}\n'
        f'        tolerance: {YAML_TOLERANCE}\n'
    )
    yaml_dir = tempfile.mkdtemp()
    yaml_path = os.path.join(yaml_dir, 'test_params.yaml')
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    return yaml_path


@pytest.mark.launch_test
def generate_test_description():
    """Generate comprehensive launch for all parameter tests."""
    yaml_path = _make_yaml_file()

    # Build comprehensive parameter set for monitor node
    params = {
        # enabled=false test
        make_freq_param(ENABLED_FALSE_TOPIC): STD_FREQUENCY,
        make_enabled_param(ENABLED_FALSE_TOPIC): False,
        # tolerance only test
        make_tol_param(TOL_ONLY_TOPIC): 15.0,
        # frequency only test
        make_freq_param(FREQ_ONLY_TOPIC): STD_FREQUENCY,
        # full config test
        make_freq_param(FULL_CONFIG_TOPIC): STD_FREQUENCY,
        make_tol_param(FULL_CONFIG_TOPIC): STD_TOLERANCE,
        # multiple topics test
        make_freq_param(MULTI_TOPIC_1): MULTI_FREQ_1,
        make_tol_param(MULTI_TOPIC_1): MULTI_TOLERANCE,
        make_freq_param(MULTI_TOPIC_2): MULTI_FREQ_2,
        make_tol_param(MULTI_TOPIC_2): MULTI_TOLERANCE,
        make_freq_param(MULTI_TOPIC_3): MULTI_FREQ_3,
        make_tol_param(MULTI_TOPIC_3): MULTI_TOLERANCE,
        # Topics list (no expected frequency)
        'topics': [MULTI_LIST_TOPIC_1, MULTI_LIST_TOPIC_2, ENABLE_EXISTING_TOPIC],
    }

    monitor_node = launch_ros.actions.Node(
        package='greenwave_monitor',
        executable='greenwave_monitor',
        name=MONITOR_NODE_NAME,
        namespace=MONITOR_NODE_NAMESPACE,
        parameters=[params, yaml_path],
        output='screen'
    )

    publishers = [
        create_minimal_publisher(
            ENABLED_FALSE_TOPIC, STD_FREQUENCY, 'imu', '_enabled_false',
            enable_diagnostics=False),
        create_minimal_publisher(
            TOL_ONLY_TOPIC, STD_FREQUENCY, 'imu', '_tol_only',
            enable_diagnostics=False),
        create_minimal_publisher(
            FREQ_ONLY_TOPIC, STD_FREQUENCY, 'imu', '_freq_only'),
        create_minimal_publisher(
            FULL_CONFIG_TOPIC, STD_FREQUENCY, 'imu', '_full_config'),
        create_minimal_publisher(
            DYNAMIC_TOPIC, DYNAMIC_FREQUENCY, 'imu', '_dynamic'),
        create_minimal_publisher(
            DYNAMIC_SET_PARAMS_TOPIC, DYNAMIC_FREQUENCY, 'imu', '_set_params',
            enable_diagnostics=True),
        create_minimal_publisher(
            DYNAMIC_DELETE_TOPIC, DYNAMIC_FREQUENCY, 'imu', '_delete_param',
            enable_diagnostics=False),
        create_minimal_publisher(
            MULTI_TOPIC_1, MULTI_FREQ_1, 'imu', '_multi_1'),
        create_minimal_publisher(
            MULTI_TOPIC_2, MULTI_FREQ_2, 'imu', '_multi_2'),
        create_minimal_publisher(
            MULTI_TOPIC_3, MULTI_FREQ_3, 'imu', '_multi_3'),
        create_minimal_publisher(
            MULTI_LIST_TOPIC_1, MULTI_LIST_FREQ, 'imu', '_list_1'),
        create_minimal_publisher(
            MULTI_LIST_TOPIC_2, MULTI_LIST_FREQ, 'imu', '_list_2'),
        create_minimal_publisher(
            YAML_TOPIC, YAML_FREQUENCY, 'imu', '_yaml'),
        create_minimal_publisher(
            NESTED_YAML_TOPIC, NESTED_FREQUENCY, 'imu', '_nested_yaml'),
        create_minimal_publisher(
            ENABLE_EXISTING_TOPIC, STD_FREQUENCY, 'imu', '_enable_test'),
        create_minimal_publisher(
            NEW_DYNAMIC_TOPIC, STD_FREQUENCY, 'imu', '_new_dynamic',
            enable_diagnostics=False),
    ]

    return (
        launch.LaunchDescription([monitor_node] + publishers + [
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


@post_shutdown_test()
class TestPostShutdown(RosNodeTestCase):
    """Post-shutdown tests."""

    TEST_NODE_NAME = 'shutdown_test_node'

    def test_node_shutdown(self, proc_info):
        """Test that the node shuts down correctly."""
        available_nodes = self.test_node.get_node_names()
        self.assertNotIn(MONITOR_NODE_NAME, available_nodes)
        assertExitCodes(proc_info, allowable_exit_codes=[0])


class TestStartupBehavior(RosNodeTestCase):
    """Tests for parameter behavior at node startup."""

    TEST_NODE_NAME = 'startup_behavior_test_node'

    def test_enabled_false_does_not_monitor(self):
        """Test that enabled=false prevents topic monitoring."""
        time.sleep(2.0)
        diagnostics = collect_diagnostics_for_topic(
            self.test_node, ENABLED_FALSE_TOPIC, expected_count=1, timeout_sec=3.0)
        self.assertEqual(
            len(diagnostics), 0,
            f'Should not monitor topic with enabled=false, got {len(diagnostics)}')

    def test_tolerance_only_starts_monitoring(self):
        """Test that specifying only tolerance starts monitoring."""
        time.sleep(2.0)
        diagnostics = collect_diagnostics_for_topic(
            self.test_node, TOL_ONLY_TOPIC, expected_count=3, timeout_sec=5.0)
        self.assertGreaterEqual(
            len(diagnostics), 3,
            f'Should monitor when tolerance is set, got {len(diagnostics)}')

    def test_frequency_only_uses_default_tolerance(self):
        """Test that specifying only frequency uses default tolerance."""
        time.sleep(2.0)
        diagnostics = collect_diagnostics_for_topic(
            self.test_node, FREQ_ONLY_TOPIC, expected_count=3, timeout_sec=10.0)
        self.assertGreaterEqual(len(diagnostics), 3)
        best_status, _ = find_best_diagnostic(diagnostics, STD_FREQUENCY, 'imu')
        self.assertIsNotNone(best_status, 'Should have valid frame rate')

    def test_full_config_monitors_correctly(self):
        """Test topic with both frequency and tolerance configured."""
        time.sleep(2.0)
        diagnostics = collect_diagnostics_for_topic(
            self.test_node, FULL_CONFIG_TOPIC, expected_count=3, timeout_sec=10.0)
        self.assertGreaterEqual(len(diagnostics), 3)
        best_status, best_values = find_best_diagnostic(
            diagnostics, STD_FREQUENCY, 'imu')
        self.assertIsNotNone(best_status)
        frame_rate = best_values[0]
        tolerance = STD_FREQUENCY * STD_TOLERANCE / 100.0
        self.assertAlmostEqual(frame_rate, STD_FREQUENCY, delta=tolerance)


class TestMultipleTopics(RosNodeTestCase):
    """Tests for multiple topic configuration."""

    TEST_NODE_NAME = 'multiple_topics_test_node'

    def test_all_configured_topics_monitored(self):
        """Test that all configured topics are monitored."""
        time.sleep(2.0)
        topics_to_check = [
            (MULTI_TOPIC_1, MULTI_FREQ_1),
            (MULTI_TOPIC_2, MULTI_FREQ_2),
            (MULTI_TOPIC_3, MULTI_FREQ_3),
        ]

        for topic, expected_freq in topics_to_check:
            with self.subTest(topic=topic):
                diagnostics = collect_diagnostics_for_topic(
                    self.test_node, topic, expected_count=3, timeout_sec=10.0)
                self.assertGreaterEqual(len(diagnostics), 3)
                best_status, best_values = find_best_diagnostic(
                    diagnostics, expected_freq, 'imu')
                self.assertIsNotNone(best_status)
                tolerance_hz = expected_freq * MULTI_TOLERANCE / 100.0
                self.assertAlmostEqual(
                    best_values[0], expected_freq, delta=tolerance_hz)

    def test_topics_list_monitored_without_expected_frequency(self):
        """Test topics in list are monitored but show no expected frequency."""
        time.sleep(2.0)
        for topic in [MULTI_LIST_TOPIC_1, MULTI_LIST_TOPIC_2]:
            with self.subTest(topic=topic):
                diagnostics = collect_diagnostics_for_topic(
                    self.test_node, topic, expected_count=3, timeout_sec=10.0)
                self.assertGreaterEqual(len(diagnostics), 3)
                last_diag = diagnostics[-1]
                expected_freq_value = None
                frame_rate_value = None
                for kv in last_diag.values:
                    if kv.key == 'expected_frequency':
                        expected_freq_value = float(kv.value)
                    elif kv.key == 'frame_rate_node':
                        frame_rate_value = float(kv.value)
                self.assertTrue(
                    expected_freq_value is None or expected_freq_value == 0.0)
                self.assertIsNotNone(frame_rate_value)
                self.assertGreater(frame_rate_value, 0.0)


class TestYamlConfiguration(RosNodeTestCase):
    """Tests for YAML parameter file configuration."""

    TEST_NODE_NAME = 'yaml_test_node'

    def test_topic_configured_via_yaml(self):
        """Test that topic configured via YAML file is monitored."""
        time.sleep(2.0)
        diagnostics = collect_diagnostics_for_topic(
            self.test_node, YAML_TOPIC, expected_count=3, timeout_sec=10.0)
        self.assertGreaterEqual(len(diagnostics), 3)
        best_status, _ = find_best_diagnostic(
            diagnostics, YAML_FREQUENCY, 'imu')
        self.assertIsNotNone(best_status)

    def test_nested_dict_topic_configured_via_yaml(self):
        """Test topic configured via nested YAML dict."""
        time.sleep(2.0)
        diagnostics = collect_diagnostics_for_topic(
            self.test_node, NESTED_YAML_TOPIC, expected_count=3, timeout_sec=10.0)
        self.assertGreaterEqual(len(diagnostics), 3)
        best_status, _ = find_best_diagnostic(
            diagnostics, NESTED_FREQUENCY, 'imu')
        self.assertIsNotNone(best_status)


class TestDynamicParameters(RosNodeTestCase):
    """Tests for dynamic parameter changes."""

    TEST_NODE_NAME = 'dynamic_param_test_node'

    def test_set_parameters_dynamically(self):
        """Test setting frequency and tolerance parameters dynamically."""
        time.sleep(2.0)
        freq_param = make_freq_param(DYNAMIC_SET_PARAMS_TOPIC)
        tol_param = make_tol_param(DYNAMIC_SET_PARAMS_TOPIC)

        # Verify diagnostics are published
        diagnostics = collect_diagnostics_for_topic(
            self.test_node, DYNAMIC_SET_PARAMS_TOPIC, expected_count=3, timeout_sec=5.0)
        self.assertGreaterEqual(len(diagnostics), 3)

        # Set parameters on publisher node
        success = set_parameter(
            self.test_node, freq_param, DYNAMIC_FREQUENCY,
            node_name=PUBLISHER_SET_PARAMS, node_namespace='')
        self.assertTrue(success, f'Failed to set {freq_param}')

        success = set_parameter(
            self.test_node, tol_param, DYNAMIC_TOLERANCE,
            node_name=PUBLISHER_SET_PARAMS, node_namespace='')
        self.assertTrue(success, f'Failed to set {tol_param}')

        # Verify parameters were set
        success, actual_freq = get_parameter(
            self.test_node, freq_param,
            node_name=PUBLISHER_SET_PARAMS, node_namespace='')
        self.assertTrue(success)
        self.assertAlmostEqual(actual_freq, DYNAMIC_FREQUENCY, places=1)

        # Update to mismatched frequency - should cause error diagnostics
        mismatched_frequency = 1.0
        success = set_parameter(
            self.test_node, freq_param, mismatched_frequency,
            node_name=PUBLISHER_SET_PARAMS, node_namespace='')
        self.assertTrue(success)

        time.sleep(2.0)
        diagnostics = collect_diagnostics_for_topic(
            self.test_node, DYNAMIC_SET_PARAMS_TOPIC, expected_count=3, timeout_sec=10.0)
        self.assertGreaterEqual(len(diagnostics), 3)
        has_non_ok = any(ord(d.level) != 0 for d in diagnostics)
        self.assertTrue(has_non_ok, 'Expected non-OK due to frequency mismatch')

    def test_add_new_topic_via_frequency_param(self):
        """Test that frequency param can be set for new topic."""
        time.sleep(2.0)
        freq_param = make_freq_param(NEW_DYNAMIC_TOPIC)
        success = set_parameter(self.test_node, freq_param, STD_FREQUENCY)
        self.assertTrue(success, f'Failed to set {freq_param}')

        success, value = get_parameter(self.test_node, freq_param)
        self.assertTrue(success)
        self.assertAlmostEqual(value, STD_FREQUENCY, places=1)

    def test_set_frequency_for_nonexistent_topic(self):
        """Test setting frequency for a topic that doesn't exist."""
        time.sleep(1.0)
        freq_param = make_freq_param(NONEXISTENT_TOPIC)
        success = set_parameter(self.test_node, freq_param, STD_FREQUENCY)
        self.assertTrue(success)

        success, actual_freq = get_parameter(self.test_node, freq_param)
        self.assertTrue(success)
        self.assertAlmostEqual(actual_freq, STD_FREQUENCY, places=1)

        diagnostics = collect_diagnostics_for_topic(
            self.test_node, NONEXISTENT_TOPIC, expected_count=1, timeout_sec=3.0)
        self.assertEqual(len(diagnostics), 0)

    def test_non_numeric_parameter_rejected(self):
        """Test that non-numeric parameter values are rejected."""
        time.sleep(1.0)
        freq_param = make_freq_param(DYNAMIC_TOPIC)
        success = set_parameter(
            self.test_node, freq_param, 'not_a_number',
            node_name=PUBLISHER_DYNAMIC, node_namespace='')
        self.assertFalse(success)

        tol_param = make_tol_param(DYNAMIC_TOPIC)
        success = set_parameter(
            self.test_node, tol_param, 'invalid',
            node_name=PUBLISHER_DYNAMIC, node_namespace='')
        self.assertFalse(success)

    def test_non_positive_frequency_rejected(self):
        """Test that non-positive frequency values are rejected."""
        time.sleep(1.0)
        freq_param = make_freq_param(DYNAMIC_TOPIC)

        success = set_parameter(
            self.test_node, freq_param, 0.0,
            node_name=PUBLISHER_DYNAMIC, node_namespace='')
        self.assertFalse(success, 'Zero frequency should be rejected')

        success = set_parameter(
            self.test_node, freq_param, -10.0,
            node_name=PUBLISHER_DYNAMIC, node_namespace='')
        self.assertFalse(success, 'Negative frequency should be rejected')

    def test_negative_tolerance_rejected(self):
        """Test that negative tolerance values are rejected."""
        time.sleep(1.0)
        tol_param = make_tol_param(DYNAMIC_TOPIC)
        success = set_parameter(
            self.test_node, tol_param, -5.0,
            node_name=PUBLISHER_DYNAMIC, node_namespace='')
        self.assertFalse(success)

    def test_delete_parameter_rejected(self):
        """Test that deleting a parameter is rejected."""
        time.sleep(2.0)
        freq_param = make_freq_param(DYNAMIC_TOPIC)
        success = delete_parameter(
            self.test_node, freq_param,
            node_name=PUBLISHER_DYNAMIC, node_namespace='')
        self.assertFalse(success)


class TestEnableExistingNode(RosNodeTestCase):
    """Tests for enabling/disabling monitoring via enabled parameter."""

    TEST_NODE_NAME = 'enable_existing_test_node'

    def test_enabled_parameter_toggles_monitoring(self):
        """Test that setting enabled parameter toggles monitoring."""
        time.sleep(3.0)
        publisher_full_name = f'/{PUBLISHER_ENABLE_TEST}'
        enabled_param_name = f'{TOPIC_PARAM_PREFIX}{ENABLE_EXISTING_TOPIC}{ENABLED_SUFFIX}'

        get_params_client = self.test_node.create_client(
            GetParameters, f'{publisher_full_name}/get_parameters')
        self.assertTrue(
            get_params_client.wait_for_service(timeout_sec=5.0),
            'Get parameters service not available')

        # Verify publisher has enabled parameter
        request = GetParameters.Request()
        request.names = [enabled_param_name]
        future = get_params_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result())
        self.assertTrue(len(future.result().values) > 0)

        # Disable monitoring via enabled parameter
        success = set_parameter(
            self.test_node, enabled_param_name, False)
        self.assertTrue(success, 'Failed to disable monitoring')

        time.sleep(0.5)

        # Verify enabled=false on monitor node
        success, value = get_parameter(self.test_node, enabled_param_name)
        self.assertTrue(success)
        self.assertFalse(value)

        # Re-enable monitoring via enabled parameter
        success = set_parameter(
            self.test_node, enabled_param_name, True)
        self.assertTrue(success, 'Failed to enable monitoring')

        time.sleep(0.5)

        # Verify enabled=true
        success, value = get_parameter(self.test_node, enabled_param_name)
        self.assertTrue(success)
        self.assertTrue(value)

        self.test_node.destroy_client(get_params_client)


if __name__ == '__main__':
    unittest.main()
