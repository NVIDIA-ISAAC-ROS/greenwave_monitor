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


@pytest.mark.launch_test
@launch_testing.parametrize('message_type, expected_frequency, tolerance_hz', TEST_CONFIGURATIONS)
def generate_test_description(message_type, expected_frequency, tolerance_hz):
    """Generate launch description for greenwave monitor tests."""
    # Launch the greenwave_monitor
    ros2_monitor_node = create_monitor_node(
        namespace=MONITOR_NODE_NAMESPACE,
        node_name=MONITOR_NODE_NAME,
        topics=['/test_topic']
    )

    # Create publishers
    publishers = [
        # Main test topic publisher with parametrized frequency
        create_minimal_publisher('/test_topic', expected_frequency, message_type),
        # Additional publishers for topic management tests
        create_minimal_publisher('/test_topic1', expected_frequency, message_type, '1'),
        create_minimal_publisher('/test_topic2', expected_frequency, message_type, '2')
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
            self.test_node, manage_client, timeout_sec=3.0,
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
        self.verify_diagnostics('/test_topic', expected_frequency, message_type, tolerance_hz)

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

        TEST_TOPIC = '/test_topic'

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

        TEST_TOPIC1 = '/test_topic1'
        TEST_TOPIC2 = '/test_topic2'

        # Allow some time for topic discovery
        end_time = time.time() + 1.0
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Try to add a non-existent topic - should fail
        nonexistent_topic = '/test/nonexistent_topic'
        response = self.call_manage_topic(
            add=True, topic=nonexistent_topic, service_client=service_client)
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


if __name__ == '__main__':
    unittest.main()
