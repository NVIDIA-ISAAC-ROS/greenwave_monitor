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


@pytest.mark.launch_test
def generate_test_description():
    """Test dynamic parameter changes via ros2 param set."""
    ros2_monitor_node = create_monitor_node()

    publisher = create_minimal_publisher(
        TEST_TOPIC, TEST_FREQUENCY, 'imu', '_dynamic'
    )

    publisher_set_params = create_minimal_publisher(
        TEST_TOPIC_SET_PARAMS, TEST_FREQUENCY, 'imu', '_set_params'
    )

    publisher_delete_param = create_minimal_publisher(
        TEST_TOPIC_DELETE_PARAM, TEST_FREQUENCY, 'imu', '_delete_param'
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
        """Test setting frequency and tolerance parameters in sequence."""
        time.sleep(2.0)

        freq_param = make_freq_param(TEST_TOPIC_SET_PARAMS)
        tol_param = make_tol_param(TEST_TOPIC_SET_PARAMS)

        # 1. Verify topic is not monitored initially
        initial_diagnostics = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC_SET_PARAMS, expected_count=1, timeout_sec=2.0
        )
        self.assertEqual(
            len(initial_diagnostics), 0,
            f'{TEST_TOPIC_SET_PARAMS} should not be monitored initially'
        )

        # 2. Set tolerance before frequency - should succeed but not start monitoring
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
            self.test_node, TEST_TOPIC_SET_PARAMS, expected_count=1, timeout_sec=2.0
        )
        self.assertEqual(
            len(diagnostics_after_tol), 0,
            f'{TEST_TOPIC_SET_PARAMS} should remain unmonitored after setting only tolerance'
        )

        # 3. Set frequency - topic should have frequency checking enabled
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
            self.test_node, TEST_TOPIC_SET_PARAMS, expected_count=3, timeout_sec=10.0
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
            self.test_node, TEST_TOPIC_SET_PARAMS, expected_count=2, timeout_sec=5.0
        )
        self.assertGreaterEqual(
            len(diagnostics_with_zero_tol), 2,
            'Topic should still be monitored with zero tolerance'
        )

        # Check that at least one diagnostic has ERROR level (frequency outside 0% tolerance)
        has_error = any(
            ord(d.level) != 0 for d in diagnostics_with_zero_tol
        )
        self.assertTrue(
            has_error,
            'Expected ERROR diagnostics with 0% tolerance'
        )

        # Reset tolerance to 10% - should no longer error
        success = set_parameter(self.test_node, tol_param, 10.0)
        self.assertTrue(success, f'Failed to reset {tol_param}')

        # Wait for diagnostics to stabilize after tolerance change
        time.sleep(3.0)
        diagnostics_after_reset = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC_SET_PARAMS, expected_count=3, timeout_sec=10.0
        )
        self.assertGreaterEqual(
            len(diagnostics_after_reset), 3,
            'Expected diagnostics after resetting tolerance'
        )

        # Verify most recent diagnostic is OK after resetting tolerance
        last_diagnostic = diagnostics_after_reset[-1]
        self.assertEqual(
            ord(last_diagnostic.level), 0,
            'Expected OK diagnostic after resetting tolerance to 10%'
        )

        # 5. Update expected frequency to mismatched value - should cause error
        # Publisher is still at 30 Hz, tolerance is 10%, but we set expected to 1 Hz
        mismatched_frequency = 1.0
        success = set_parameter(self.test_node, freq_param, mismatched_frequency)
        self.assertTrue(success, f'Failed to update {freq_param}')

        success, actual_freq = get_parameter(self.test_node, freq_param)
        self.assertTrue(success, f'Failed to get updated {freq_param}')
        self.assertAlmostEqual(
            actual_freq, mismatched_frequency, places=1,
            msg=f'Frequency mismatch: expected {mismatched_frequency}, got {actual_freq}'
        )

        time.sleep(2.0)
        diagnostics_mismatched = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC_SET_PARAMS, expected_count=3, timeout_sec=10.0
        )
        self.assertGreaterEqual(
            len(diagnostics_mismatched), 3,
            'Should still receive diagnostics after frequency update'
        )

        # Verify diagnostics show error due to frequency mismatch
        has_error = any(
            ord(d.level) != 0 for d in diagnostics_mismatched
        )
        self.assertTrue(
            has_error,
            'Expected ERROR diagnostics when actual frequency (30 Hz) '
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

        freq_param = make_freq_param(TEST_TOPIC)
        success = set_parameter(self.test_node, freq_param, 'not_a_number')
        self.assertFalse(success, 'Non-numeric frequency parameter should be rejected')

        tol_param = make_tol_param(TEST_TOPIC)
        success = set_parameter(self.test_node, tol_param, 'invalid')
        self.assertFalse(success, 'Non-numeric tolerance parameter should be rejected')

    def test_non_positive_frequency_rejected(self):
        """Test that non-positive frequency values are rejected."""
        time.sleep(1.0)

        freq_param = make_freq_param(TEST_TOPIC)

        # Test zero frequency
        success = set_parameter(self.test_node, freq_param, 0.0)
        self.assertFalse(success, 'Zero frequency should be rejected')

        # Test negative frequency
        success = set_parameter(self.test_node, freq_param, -10.0)
        self.assertFalse(success, 'Negative frequency should be rejected')

    def test_negative_tolerance_rejected(self):
        """Test that negative tolerance values are rejected."""
        time.sleep(1.0)

        tol_param = make_tol_param(TEST_TOPIC)
        success = set_parameter(self.test_node, tol_param, -5.0)
        self.assertFalse(success, 'Negative tolerance should be rejected')

    def test_delete_parameter_clears_error(self):
        """Test that deleting a parameter clears the error state."""
        time.sleep(2.0)

        freq_param = make_freq_param(TEST_TOPIC_DELETE_PARAM)
        tol_param = make_tol_param(TEST_TOPIC_DELETE_PARAM)

        # Set up monitoring with correct frequency (publisher is at 30 Hz)
        success = set_parameter(self.test_node, freq_param, TEST_FREQUENCY)
        self.assertTrue(success, f'Failed to set {freq_param}')

        success = set_parameter(self.test_node, tol_param, 10.0)
        self.assertTrue(success, f'Failed to set {tol_param}')

        time.sleep(2.0)

        # Verify initial diagnostics are OK
        diagnostics = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC_DELETE_PARAM, expected_count=3, timeout_sec=10.0
        )
        self.assertGreaterEqual(len(diagnostics), 1, 'Should have diagnostics')
        self.assertEqual(
            ord(diagnostics[-1].level), 0,
            'Initial diagnostics should be OK'
        )

        # Set mismatched frequency to cause error (expect 1 Hz but publisher is 30 Hz)
        success = set_parameter(self.test_node, freq_param, 1.0)
        self.assertTrue(success, 'Failed to set mismatched frequency')

        time.sleep(2.0)

        # Verify diagnostics show error
        diagnostics_error = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC_DELETE_PARAM, expected_count=3, timeout_sec=10.0
        )
        self.assertGreaterEqual(len(diagnostics_error), 1, 'Should have diagnostics')
        has_error = any(ord(d.level) != 0 for d in diagnostics_error)
        self.assertTrue(has_error, 'Should have error diagnostics with mismatched frequency')

        # Delete the frequency parameter to clear expected frequency
        success = delete_parameter(self.test_node, freq_param)
        self.assertTrue(success, f'Failed to delete {freq_param}')

        time.sleep(2.0)

        # Verify diagnostics are OK again (no expected frequency = no error)
        diagnostics_after_delete = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC_DELETE_PARAM, expected_count=3, timeout_sec=10.0
        )
        self.assertGreaterEqual(
            len(diagnostics_after_delete), 1, 'Should have diagnostics after delete'
        )
        self.assertEqual(
            ord(diagnostics_after_delete[-1].level), 0,
            'Diagnostics should be OK after deleting frequency parameter'
        )


if __name__ == '__main__':
    unittest.main()
