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

"""Test: dynamic parameter changes via ros2 param set."""

import time
import unittest

from greenwave_monitor.test_utils import (
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    create_monitor_node,
    delete_parameter,
    get_parameter,
    make_freq_param,
    make_tol_param,
    RosNodeTestCase,
    set_parameter,
)
import launch
import launch_testing
import pytest


TEST_TOPIC = '/dynamic_param_topic'
TEST_TOPIC_SET_PARAMS = '/dynamic_param_topic_set_params'
TEST_TOPIC_DELETE_PARAM = '/dynamic_param_topic_delete_param'
TEST_FREQUENCY = 30.0
TEST_TOLERANCE = 20.0
NONEXISTENT_TOPIC = '/topic_that_does_not_exist'
# Publisher node names (those with GreenwaveDiagnostics)
PUBLISHER_NODE_NAME = 'minimal_publisher_node_dynamic'
PUBLISHER_SET_PARAMS_NODE = 'minimal_publisher_node_set_params'


@pytest.mark.launch_test
def generate_test_description():
    """Test dynamic parameter changes via ros2 param set."""
    ros2_monitor_node = create_monitor_node()

    publisher = create_minimal_publisher(
        TEST_TOPIC, TEST_FREQUENCY, 'imu', '_dynamic'
    )

    publisher_set_params = create_minimal_publisher(
        TEST_TOPIC_SET_PARAMS, TEST_FREQUENCY, 'imu', '_set_params',
        enable_diagnostics=True
    )

    publisher_delete_param = create_minimal_publisher(
        TEST_TOPIC_DELETE_PARAM, TEST_FREQUENCY, 'imu', '_delete_param',
        enable_diagnostics=False
    )

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            publisher,
            publisher_set_params,
            publisher_delete_param,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


class TestDynamicParameterChanges(RosNodeTestCase):
    """Test changing parameters dynamically via ros2 param set."""

    TEST_NODE_NAME = 'dynamic_param_test_node'

    def test_set_parameters(self):
        """Test setting frequency and tolerance parameters dynamically."""
        time.sleep(2.0)

        freq_param = make_freq_param(TEST_TOPIC_SET_PARAMS)
        tol_param = make_tol_param(TEST_TOPIC_SET_PARAMS)

        # 1. Verify diagnostics are being published (publisher has diagnostics enabled)
        initial_diagnostics = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC_SET_PARAMS, expected_count=3, timeout_sec=5.0
        )
        self.assertGreaterEqual(
            len(initial_diagnostics), 3,
            f'{TEST_TOPIC_SET_PARAMS} should have diagnostics from publisher'
        )

        # 2. Set frequency and tolerance on the publisher node
        success = set_parameter(
            self.test_node, freq_param, TEST_FREQUENCY,
            node_name=PUBLISHER_SET_PARAMS_NODE, node_namespace='')
        self.assertTrue(success, f'Failed to set {freq_param}')

        success = set_parameter(
            self.test_node, tol_param, TEST_TOLERANCE,
            node_name=PUBLISHER_SET_PARAMS_NODE, node_namespace='')
        self.assertTrue(success, f'Failed to set {tol_param}')

        # Verify parameters were set
        success, actual_freq = get_parameter(
            self.test_node, freq_param,
            node_name=PUBLISHER_SET_PARAMS_NODE, node_namespace='')
        self.assertTrue(success, f'Failed to get {freq_param}')
        self.assertAlmostEqual(
            actual_freq, TEST_FREQUENCY, places=1,
            msg=f'Frequency mismatch: expected {TEST_FREQUENCY}, got {actual_freq}'
        )

        # 3. Update expected frequency to mismatched value - should cause error
        # Publisher is still at 30 Hz, tolerance is 20%, but we set expected to 1 Hz
        mismatched_frequency = 1.0
        success = set_parameter(
            self.test_node, freq_param, mismatched_frequency,
            node_name=PUBLISHER_SET_PARAMS_NODE, node_namespace='')
        self.assertTrue(success, f'Failed to update {freq_param}')

        time.sleep(2.0)
        diagnostics_mismatched = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC_SET_PARAMS, expected_count=3, timeout_sec=10.0
        )
        self.assertGreaterEqual(
            len(diagnostics_mismatched), 3,
            'Should still receive diagnostics after frequency update'
        )

        # Verify diagnostics show non-OK status due to frequency mismatch
        has_non_ok = any(
            ord(d.level) != 0 for d in diagnostics_mismatched
        )
        self.assertTrue(
            has_non_ok,
            'Expected non-OK diagnostics when actual frequency (30 Hz) '
            'does not match expected (1 Hz)'
        )

    def test_set_frequency_for_nonexistent_topic(self):
        """Test setting expected frequency for a topic that does not exist."""
        time.sleep(1.0)

        freq_param = make_freq_param(NONEXISTENT_TOPIC)
        success = set_parameter(self.test_node, freq_param, TEST_FREQUENCY)
        self.assertTrue(success, f'Failed to set {freq_param}')

        # Verify parameter was set
        success, actual_freq = get_parameter(self.test_node, freq_param)
        self.assertTrue(success, f'Failed to get {freq_param}')
        self.assertAlmostEqual(
            actual_freq, TEST_FREQUENCY, places=1,
            msg=f'Frequency mismatch: expected {TEST_FREQUENCY}, got {actual_freq}'
        )

        # Topic should not appear in diagnostics since it doesn't exist
        diagnostics = collect_diagnostics_for_topic(
            self.test_node, NONEXISTENT_TOPIC, expected_count=1, timeout_sec=3.0
        )
        self.assertEqual(
            len(diagnostics), 0,
            f'{NONEXISTENT_TOPIC} should not appear in diagnostics'
        )

    def test_non_numeric_parameter_rejected(self):
        """Test that non-numeric parameter values are rejected."""
        time.sleep(1.0)

        # Target the publisher node which has GreenwaveDiagnostics
        freq_param = make_freq_param(TEST_TOPIC)
        success = set_parameter(
            self.test_node, freq_param, 'not_a_number',
            node_name=PUBLISHER_NODE_NAME, node_namespace='')
        self.assertFalse(success, 'Non-numeric frequency parameter should be rejected')

        tol_param = make_tol_param(TEST_TOPIC)
        success = set_parameter(
            self.test_node, tol_param, 'invalid',
            node_name=PUBLISHER_NODE_NAME, node_namespace='')
        self.assertFalse(success, 'Non-numeric tolerance parameter should be rejected')

    def test_non_positive_frequency_rejected(self):
        """Test that non-positive frequency values are rejected."""
        time.sleep(1.0)

        # Target the publisher node which has GreenwaveDiagnostics
        freq_param = make_freq_param(TEST_TOPIC)

        # Test zero frequency
        success = set_parameter(
            self.test_node, freq_param, 0.0,
            node_name=PUBLISHER_NODE_NAME, node_namespace='')
        self.assertFalse(success, 'Zero frequency should be rejected')

        # Test negative frequency
        success = set_parameter(
            self.test_node, freq_param, -10.0,
            node_name=PUBLISHER_NODE_NAME, node_namespace='')
        self.assertFalse(success, 'Negative frequency should be rejected')

    def test_negative_tolerance_rejected(self):
        """Test that negative tolerance values are rejected."""
        time.sleep(1.0)

        # Target the publisher node which has GreenwaveDiagnostics
        tol_param = make_tol_param(TEST_TOPIC)
        success = set_parameter(
            self.test_node, tol_param, -5.0,
            node_name=PUBLISHER_NODE_NAME, node_namespace='')
        self.assertFalse(success, 'Negative tolerance should be rejected')

    def test_delete_parameter_rejected(self):
        """Test that deleting a parameter is rejected."""
        time.sleep(2.0)

        # Target the publisher node which has GreenwaveDiagnostics
        freq_param = make_freq_param(TEST_TOPIC)

        # Attempt to delete the frequency parameter - should be rejected
        success = delete_parameter(
            self.test_node, freq_param,
            node_name=PUBLISHER_NODE_NAME, node_namespace='')
        self.assertFalse(success, 'Parameter deletion should be rejected')


if __name__ == '__main__':
    unittest.main()
