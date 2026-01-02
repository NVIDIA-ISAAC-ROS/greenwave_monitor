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

import time

from greenwave_monitor.test_utils import (
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    create_monitor_node,
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
TEST_FREQUENCY = 30.0
TEST_TOLERANCE = 20.0
NONEXISTENT_TOPIC = '/topic_that_does_not_exist'


@pytest.mark.launch_test
def generate_test_description():
    """Test dynamic parameter changes via ros2 param set."""
    ros2_monitor_node = create_monitor_node()

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


class TestDynamicParameterChanges(RosNodeTestCase):
    """Test changing parameters dynamically via ros2 param set."""

    TEST_NODE_NAME = 'dynamic_param_test_node'

    def test_set_parameters(self):
        """Test setting frequency and tolerance parameters in sequence."""
        time.sleep(2.0)

        freq_param = make_freq_param(TEST_TOPIC)
        tol_param = make_tol_param(TEST_TOPIC)

        # 1. Verify topic is not monitored initially
        initial_diagnostics = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC, expected_count=1, timeout_sec=2.0
        )
        self.assertEqual(
            len(initial_diagnostics), 0,
            f'{TEST_TOPIC} should not be monitored initially'
        )

        # 2. Set tolerance before frequency - topic should remain unmonitored
        success = set_parameter(self.test_node, tol_param, TEST_TOLERANCE)
        self.assertTrue(success, f'Failed to set {tol_param}')

        success, actual_tol = get_parameter(self.test_node, tol_param)
        self.assertTrue(success, f'Failed to get {tol_param}')
        self.assertAlmostEqual(
            actual_tol, TEST_TOLERANCE, places=1,
            msg=f'Tolerance mismatch: expected {TEST_TOLERANCE}, got {actual_tol}'
        )

        time.sleep(1.0)
        diagnostics_after_tol = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC, expected_count=1, timeout_sec=2.0
        )
        self.assertEqual(
            len(diagnostics_after_tol), 0,
            f'{TEST_TOPIC} should remain unmonitored after setting only tolerance'
        )

        # 3. Set frequency - topic should become monitored
        success = set_parameter(self.test_node, freq_param, TEST_FREQUENCY)
        self.assertTrue(success, f'Failed to set {freq_param}')

        success, actual_freq = get_parameter(self.test_node, freq_param)
        self.assertTrue(success, f'Failed to get {freq_param}')
        self.assertAlmostEqual(
            actual_freq, TEST_FREQUENCY, places=1,
            msg=f'Frequency mismatch: expected {TEST_FREQUENCY}, got {actual_freq}'
        )

        time.sleep(1.0)
        diagnostics_after_freq = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC, expected_count=3, timeout_sec=10.0
        )
        self.assertGreaterEqual(
            len(diagnostics_after_freq), 3,
            'Expected diagnostics after setting frequency param'
        )

        # 4. Set tolerance to 0.0 - should cause diagnostics to show error
        success = set_parameter(self.test_node, tol_param, 0.0)
        self.assertTrue(success, f'Failed to set {tol_param} to 0.0')

        success, actual_tol = get_parameter(self.test_node, tol_param)
        self.assertTrue(success, f'Failed to get {tol_param}')
        self.assertAlmostEqual(
            actual_tol, 0.0, places=1,
            msg=f'Tolerance mismatch: expected 0.0, got {actual_tol}'
        )

        time.sleep(2.0)
        diagnostics_with_zero_tol = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC, expected_count=2, timeout_sec=5.0
        )
        self.assertGreaterEqual(
            len(diagnostics_with_zero_tol), 2,
            'Topic should still be monitored with zero tolerance'
        )

        # Check that at least one diagnostic has ERROR level (frequency outside 0% tolerance)
        has_error = any(
            d.level != 0 for d in diagnostics_with_zero_tol
        )
        self.assertTrue(
            has_error,
            'Expected ERROR diagnostics with 0% tolerance'
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


if __name__ == '__main__':
    unittest.main()
