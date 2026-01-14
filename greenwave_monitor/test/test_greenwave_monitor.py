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


import os
import tempfile
import time
import unittest

from greenwave_monitor.test_utils import (
    call_manage_topic_service,
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    create_monitor_node,
    create_service_clients,
    find_best_diagnostic,
    MANAGE_TOPIC_TEST_CONFIG,
    MONITOR_NODE_NAME,
    MONITOR_NODE_NAMESPACE,
    TEST_CONFIGURATIONS,
    verify_diagnostic_values,
    wait_for_service_connection
)
import launch
import launch_testing
from launch_testing import post_shutdown_test
from launch_testing.asserts import assertExitCodes
import pytest
import rclpy
from rclpy.node import Node


# Temp directory auto-cleans when garbage collected or process exits
_temp_dir = tempfile.TemporaryDirectory()

TEST_TOPIC = '/test_topic'
TEST_TOPIC1 = '/test_topic1'
TEST_TOPIC2 = '/test_topic2'

VALID_PARAMS_TOPIC = '/test_topic_valid_parameters'
VALID_PARAMS_EXPECTED_FREQUENCY = 100.0
VALID_PARAMS_TOLERANCE = 10.0

INVALID_PARAMS_TOPIC = '/test_topic_invalid_parameters'
INVALID_PARAMS_EXPECTED_FREQUENCY = -10.0
INVALID_PARAMS_TOLERANCE = -10.0

INTEGER_PARAMS_TOPIC = '/test_topic_integer_params'
INTEGER_PARAMS_EXPECTED_FREQUENCY = 50
INTEGER_PARAMS_TOLERANCE = 5

NONEXISTENT_TOPIC = '/test_topic_nonexistent'
NONEXISTENT_EXPECTED_FREQUENCY = 10.0
NONEXISTENT_TOLERANCE = 1.0


def create_test_yaml_config():
    """Create a temporary YAML config file for testing parameter loading."""
    # Use /** wildcard to match any namespace/node name
    yaml_content = f"""\
/**:
  ros__parameters:
    greenwave_diagnostics:
      {VALID_PARAMS_TOPIC}:
        expected_frequency: {VALID_PARAMS_EXPECTED_FREQUENCY}
        tolerance: {VALID_PARAMS_TOLERANCE}
      {INVALID_PARAMS_TOPIC}:
        expected_frequency: {INVALID_PARAMS_EXPECTED_FREQUENCY}
        tolerance: {INVALID_PARAMS_TOLERANCE}
      {INTEGER_PARAMS_TOPIC}:
        expected_frequency: {INTEGER_PARAMS_EXPECTED_FREQUENCY}
        tolerance: {INTEGER_PARAMS_TOLERANCE}
      {NONEXISTENT_TOPIC}:
        expected_frequency: {NONEXISTENT_EXPECTED_FREQUENCY}
        tolerance: {NONEXISTENT_TOLERANCE}
"""
    filepath = os.path.join(_temp_dir.name, 'test_config.yaml')
    with open(filepath, 'w') as f:
        f.write(yaml_content)
    return filepath


@pytest.mark.launch_test
@launch_testing.parametrize('message_type, expected_frequency, tolerance_hz', TEST_CONFIGURATIONS)
def generate_test_description(message_type, expected_frequency, tolerance_hz):
    """Generate launch description for greenwave monitor tests."""
    # Create temporary YAML config for testing parameter loading
    yaml_config_file = create_test_yaml_config()

    # Launch the greenwave_monitor
    ros2_monitor_node = create_monitor_node(
        namespace=MONITOR_NODE_NAMESPACE,
        node_name=MONITOR_NODE_NAME,
        parameters=[yaml_config_file, {'topics': ['/test_topic']}]
    )

    # Create publishers
    publishers = [
        # Main test topic publisher with parametrized frequency
        create_minimal_publisher(TEST_TOPIC, expected_frequency, message_type),
        # Additional publishers for topic management tests
        create_minimal_publisher(TEST_TOPIC1, expected_frequency, message_type, '1'),
        create_minimal_publisher(TEST_TOPIC2, expected_frequency, message_type, '2'),
        # Publisher for YAML configuration tests
        create_minimal_publisher(
            VALID_PARAMS_TOPIC,
            expected_frequency, message_type, '_valid_expected_frequency'),
        create_minimal_publisher(
            INTEGER_PARAMS_TOPIC,
            expected_frequency, message_type, '_integer_params'),
        create_minimal_publisher(
            INVALID_PARAMS_TOPIC,
            expected_frequency, message_type, '_invalid_expected_frequency')
    ]

    context = {
        'expected_frequency': expected_frequency,
        'message_type': message_type,
        'tolerance_hz': tolerance_hz,
    }

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            *publishers,  # Unpack all publishers into the launch description
            launch_testing.actions.ReadyToTest()
        ]), context
    )


@post_shutdown_test()
class TestGreenwaveMonitorPostShutdown(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node."""
        rclpy.init()
        cls.test_node = Node('shutdown_test_node')

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


class TestGreenwaveMonitor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node."""
        rclpy.init()
        cls.test_node = Node('test_node')

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2."""
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def check_node_launches_successfully(self):
        """Test that the node launches without errors."""
        # Create a service client to check if the node is ready
        # Service discovery is more reliable than node discovery in launch_testing
        manage_client, set_freq_client = create_service_clients(
            self.test_node, MONITOR_NODE_NAMESPACE, MONITOR_NODE_NAME
        )
        service_available = wait_for_service_connection(
            self.test_node, manage_client, timeout_sec=10.0,
            service_name=f'/{MONITOR_NODE_NAMESPACE}/{MONITOR_NODE_NAME}/manage_topic'
        )
        self.assertTrue(
            service_available,
            f'Service "/{MONITOR_NODE_NAMESPACE}/{MONITOR_NODE_NAME}/manage_topic" '
            'not available within timeout')
        return manage_client

    def verify_diagnostics(self, topic_name, expected_frequency, message_type, tolerance_hz):
        """Verify diagnostics for a given topic."""
        # Collect diagnostic messages using shared utility
        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, topic_name, expected_count=5, timeout_sec=10.0
        )

        # We expect the monitor to be publishing diagnostics
        self.assertGreaterEqual(len(received_diagnostics), 5,
                                'Did not receive enough diagnostics messages')

        # Find the best diagnostic message using shared utility
        best_status, best_values = find_best_diagnostic(
            received_diagnostics, expected_frequency, message_type
        )

        self.assertIsNotNone(best_status, 'Did not find a diagnostic with all required values')
        self.assertEqual(topic_name, best_status.name)

        # Verify diagnostic values using shared utility
        errors = verify_diagnostic_values(
            best_status, best_values, expected_frequency, message_type, tolerance_hz
        )

        # Assert no errors occurred
        if errors:
            self.fail(f"Diagnostic verification failed: {'; '.join(errors)}")

    def test_frequency_monitoring(self, expected_frequency, message_type, tolerance_hz):
        """Test that the monitor node correctly tracks different frequencies."""
        # This test runs for all configurations to verify frequency monitoring
        self.check_node_launches_successfully()
        self.verify_diagnostics(TEST_TOPIC, expected_frequency, message_type, tolerance_hz)

    def call_manage_topic(self, add, topic, service_client):
        """Service call helper."""
        response = call_manage_topic_service(
            self.test_node, service_client, add, topic, timeout_sec=8.0
        )
        self.assertIsNotNone(response, 'Service call failed or timed out')
        return response

    def test_manage_one_topic(self, expected_frequency, message_type, tolerance_hz):
        """Test that add_topic() and remove_topic() work correctly for one topic."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running manage topic tests once')

        service_client = self.check_node_launches_successfully()

        # 1. Remove an existing topic – should succeed on first attempt.
        response = self.call_manage_topic(
            add=False, topic=TEST_TOPIC, service_client=service_client)
        self.assertTrue(response.success)

        # 2. Removing the same topic again should fail because it no longer exists.
        response = self.call_manage_topic(
            add=False, topic=TEST_TOPIC, service_client=service_client)
        self.assertFalse(response.success)

        # 3. Add the topic back – should succeed now.
        response = self.call_manage_topic(
            add=True, topic=TEST_TOPIC, service_client=service_client)
        self.assertTrue(response.success)

        # Verify diagnostics after adding the topic back
        self.verify_diagnostics(TEST_TOPIC, expected_frequency, message_type, tolerance_hz)

        # 4. Adding the same topic again should fail because it's already monitored.
        response = self.call_manage_topic(
            add=True, topic=TEST_TOPIC, service_client=service_client)
        self.assertFalse(response.success)

    def test_manage_multiple_topics(self, expected_frequency, message_type, tolerance_hz):
        """Test that add_topic() and remove_topic() work correctly for multiple topics."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running manage topic tests once for 30 hz images')

        service_client = self.check_node_launches_successfully()

        # Allow some time for topic discovery
        end_time = time.time() + 1.0
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Try to add a non-existent topic - should fail
        response = self.call_manage_topic(
            add=True, topic=NONEXISTENT_TOPIC, service_client=service_client)
        self.assertFalse(response.success)

        # 1. Add first topic – should succeed.
        response = self.call_manage_topic(
            add=True, topic=TEST_TOPIC1, service_client=service_client)
        self.assertTrue(response.success)

        # Verify diagnostics after adding the first topic
        self.verify_diagnostics(TEST_TOPIC1, expected_frequency, message_type, tolerance_hz)

        # 2. Add second topic – should succeed.
        response = self.call_manage_topic(
            add=True, topic=TEST_TOPIC2, service_client=service_client)
        self.assertTrue(response.success)

        # Verify diagnostics after adding the second topic
        self.verify_diagnostics(TEST_TOPIC2, expected_frequency, message_type, tolerance_hz)

        # 3. Remove first topic – should succeed.
        response = self.call_manage_topic(
            add=False, topic=TEST_TOPIC1, service_client=service_client)
        self.assertTrue(response.success)

    def test_yaml_parameter_loading(self, expected_frequency, message_type, tolerance_hz):
        """Test that topics defined in YAML config are automatically monitored."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running YAML parameter tests once')

        self.check_node_launches_successfully()

        # Collect diagnostics - topic should already be monitored from YAML
        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, VALID_PARAMS_TOPIC, expected_count=3, timeout_sec=10.0
        )

        self.assertGreaterEqual(
            len(received_diagnostics), 1,
            f'Topic {VALID_PARAMS_TOPIC} from YAML should be auto-monitored'
        )

        # Verify the expected_frequency from YAML (100.0) is applied
        best_status, _ = find_best_diagnostic(
            received_diagnostics, VALID_PARAMS_EXPECTED_FREQUENCY, message_type
        )
        self.assertIsNotNone(best_status, f'Did not find diagnostics for {VALID_PARAMS_TOPIC}')

        # Extract values from diagnostic status
        diag_values = {kv.key: kv.value for kv in best_status.values}

        # Check that expected_frequency was set from YAML
        self.assertIn('expected_frequency', diag_values)
        self.assertAlmostEqual(
            float(diag_values['expected_frequency']), VALID_PARAMS_EXPECTED_FREQUENCY, places=1,
            msg='Expected frequency from YAML should be 100.0'
        )

        # Verify the tolerance from YAML (10.0) is applied
        self.assertIn('tolerance', diag_values)
        self.assertAlmostEqual(
            float(diag_values['tolerance']), VALID_PARAMS_TOLERANCE, places=1,
            msg='Tolerance from YAML should be 10.0'
        )

        # Verify integer parameters are handled correctly
        int_diagnostics = collect_diagnostics_for_topic(
            self.test_node, INTEGER_PARAMS_TOPIC, expected_count=3, timeout_sec=10.0
        )
        self.assertGreaterEqual(
            len(int_diagnostics), 1,
            f'Topic {INTEGER_PARAMS_TOPIC} with integer params should be monitored'
        )

        int_status, _ = find_best_diagnostic(
            int_diagnostics, INTEGER_PARAMS_EXPECTED_FREQUENCY, message_type)
        self.assertIsNotNone(int_status, f'Did not find diagnostics for {INTEGER_PARAMS_TOPIC}')

        int_values = {kv.key: kv.value for kv in int_status.values}

        # Check integer expected_frequency (50) is properly converted
        self.assertIn('expected_frequency', int_values)
        self.assertAlmostEqual(
            float(int_values['expected_frequency']), INTEGER_PARAMS_EXPECTED_FREQUENCY, places=1,
            msg='Integer expected frequency from YAML should be 50'
        )

        # Check integer tolerance (5) is properly converted
        self.assertIn('tolerance', int_values)
        self.assertAlmostEqual(
            float(int_values['tolerance']), INTEGER_PARAMS_TOLERANCE, places=1,
            msg='Integer tolerance from YAML should be 5'
        )

        # Verify topic with invalid (negative) params is monitored but with 0 values
        invalid_diagnostics = collect_diagnostics_for_topic(
            self.test_node, INVALID_PARAMS_TOPIC, expected_count=3, timeout_sec=10.0
        )
        self.assertGreaterEqual(
            len(invalid_diagnostics), 1,
            f'Topic {INVALID_PARAMS_TOPIC} should still be monitored'
        )

        invalid_status, _ = find_best_diagnostic(
            invalid_diagnostics, INVALID_PARAMS_EXPECTED_FREQUENCY, message_type)
        self.assertIsNotNone(
            invalid_status, f'Did not find diagnostics for {INVALID_PARAMS_TOPIC}')

        invalid_values = {kv.key: kv.value for kv in invalid_status.values}

        # Invalid (negative) expected_frequency should report as 0
        self.assertIn('expected_frequency', invalid_values)
        self.assertAlmostEqual(
            float(invalid_values['expected_frequency']), 0.0, places=1,
            msg='Invalid expected frequency should report as 0'
        )

        # Invalid (negative) tolerance should report as 0
        self.assertIn('tolerance', invalid_values)
        self.assertAlmostEqual(
            float(invalid_values['tolerance']), 0.0, places=1,
            msg='Invalid tolerance should report as 0'
        )

        # Verify topic with nonexistent parameters is not monitored
        nonexistent_diagnostics = collect_diagnostics_for_topic(
            self.test_node, NONEXISTENT_TOPIC, expected_count=3, timeout_sec=10.0
        )
        self.assertEqual(
            len(nonexistent_diagnostics), 0,
            f'Topic {NONEXISTENT_TOPIC} should not be monitored'
        )


if __name__ == '__main__':
    unittest.main()
