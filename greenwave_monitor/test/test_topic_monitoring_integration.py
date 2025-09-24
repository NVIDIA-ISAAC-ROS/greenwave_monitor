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

import threading
import time
import unittest

from diagnostic_msgs.msg import DiagnosticStatus
from greenwave_monitor.test_utils import (
    create_minimal_publisher,
    create_monitor_node,
    MANAGE_TOPIC_TEST_CONFIG,
    MONITOR_NODE_NAME,
    MONITOR_NODE_NAMESPACE,
    TEST_CONFIGURATIONS
)
from greenwave_monitor.ui_adaptor import GreenwaveUiAdaptor, UiDiagnosticData
from greenwave_monitor_interfaces.srv import ManageTopic
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
    """Generate launch description for topic monitoring tests."""
    # Launch the greenwave_monitor
    ros2_monitor_node = create_monitor_node(
        node_name=MONITOR_NODE_NAME,
        topics=['/test_topic']
    )

    # Create publishers for testing
    publishers = [
        # Main test topic publisher with parametrized frequency
        create_minimal_publisher('/test_topic', expected_frequency, message_type),
        # Additional publishers for topic management tests
        create_minimal_publisher('/test_topic1', expected_frequency, message_type, '1'),
        create_minimal_publisher('/test_topic2', expected_frequency, message_type, '2'),
        # Publisher for service discovery tests
        create_minimal_publisher('/discovery_test_topic', 50.0, 'imu', '_discovery')
    ]

    context = {
        'expected_frequency': expected_frequency,
        'message_type': message_type,
        'tolerance_hz': tolerance_hz,
    }

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            *publishers,
            launch_testing.actions.ReadyToTest()
        ]), context
    )


@post_shutdown_test()
class TestTopicMonitoringPostShutdown(unittest.TestCase):
    """Post-shutdown tests for topic monitoring."""

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


class TestTopicMonitoringIntegration(unittest.TestCase):
    """Integration tests for topic monitoring functionality."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node."""
        rclpy.init()
        cls.test_node = Node('topic_monitoring_test_node', namespace=MONITOR_NODE_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2."""
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        """Set up for each test."""
        # Create a fresh GreenwaveUiAdaptor instance for each test with proper namespace
        self.diagnostics_monitor = GreenwaveUiAdaptor(
            self.test_node,
            monitor_node_name=MONITOR_NODE_NAME
        )

        # Allow time for service discovery in test environment (reduced from 2.0s)
        time.sleep(1.0)

    def tearDown(self):
        """Clean up after each test."""
        # Clean up the monitor instance
        if hasattr(self, 'diagnostics_monitor'):
            # Clean up ROS components
            try:
                self.test_node.destroy_subscription(self.diagnostics_monitor.subscription)
                self.test_node.destroy_client(self.diagnostics_monitor.manage_topic_client)
                self.test_node.destroy_client(
                    self.diagnostics_monitor.set_expected_frequency_client)
            except Exception:
                pass  # Ignore cleanup errors

    def test_service_discovery_default_namespace(
            self, expected_frequency, message_type, tolerance_hz):
        """Test service discovery with default namespace."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running service discovery tests once')

        # The monitor should discover the services automatically
        self.assertIsNotNone(self.diagnostics_monitor.manage_topic_client)
        self.assertIsNotNone(self.diagnostics_monitor.set_expected_frequency_client)

        # Verify services are available
        manage_available = self.diagnostics_monitor.manage_topic_client.wait_for_service(
            timeout_sec=5.0)
        set_freq_available = (
            self.diagnostics_monitor.set_expected_frequency_client
            .wait_for_service(timeout_sec=5.0))

        self.assertTrue(manage_available, 'ManageTopic service should be available')
        self.assertTrue(set_freq_available, 'SetExpectedFrequency service should be available')

    def test_diagnostic_data_conversion(self, expected_frequency, message_type, tolerance_hz):
        """Test conversion from DiagnosticStatus to UiDiagnosticData."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running diagnostic conversion tests once')

        # Create a mock DiagnosticStatus
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = '/test_topic'
        status.message = 'Test message'

        # Add key-value pairs
        from diagnostic_msgs.msg import KeyValue
        status.values = [
            KeyValue(key='frame_rate_node', value='100.5'),
            KeyValue(key='frame_rate_msg', value='99.8'),
            KeyValue(key='current_delay_from_realtime_ms', value='5.2')
        ]

        # Convert to UI data
        ui_data = UiDiagnosticData.from_status(status)

        # Verify conversion
        self.assertEqual(ui_data.status, 'OK')
        self.assertEqual(ui_data.pub_rate, '100.5')
        self.assertEqual(ui_data.msg_rate, '99.8')
        self.assertEqual(ui_data.latency, '5.2')

    def test_diagnostic_data_conversion_different_levels(
            self, expected_frequency, message_type, tolerance_hz):
        """Test diagnostic status level conversion."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running diagnostic conversion tests once')

        status_levels = [
            (DiagnosticStatus.OK, 'OK'),
            (DiagnosticStatus.WARN, 'WARN'),
            (DiagnosticStatus.ERROR, 'ERROR'),
            (DiagnosticStatus.STALE, 'STALE'),
            (bytes([99]), 'UNKNOWN')  # Unknown level as bytes
        ]

        for level, expected_str in status_levels:
            with self.subTest(level=level):
                status = DiagnosticStatus()
                status.level = level
                status.name = '/test_topic'

                ui_data = UiDiagnosticData.from_status(status)
                self.assertEqual(ui_data.status, expected_str)

    def test_toggle_topic_monitoring_add_remove(
            self, expected_frequency, message_type, tolerance_hz):
        """Test adding and removing topics from monitoring."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running topic toggle tests once')

        test_topic = '/test_topic1'

        # Initially, topic should not be in diagnostics
        self.assertNotIn(test_topic, self.diagnostics_monitor.ui_diagnostics)

        # Add topic
        self.diagnostics_monitor.toggle_topic_monitoring(test_topic)

        # Wait for diagnostic data to arrive and be processed
        max_wait_time = 5.0
        start_time = time.time()
        topic_data = None

        while time.time() - start_time < max_wait_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            topic_data = self.diagnostics_monitor.get_topic_diagnostics(test_topic)
            if topic_data.status != '-':
                break
            time.sleep(0.1)

        # Topic should now have diagnostic data
        self.assertIsNotNone(topic_data)
        self.assertNotEqual(topic_data.status, '-',
                            f'Should have received diagnostic data after {max_wait_time}s')

        # Remove topic
        self.diagnostics_monitor.toggle_topic_monitoring(test_topic)

        # Topic should be removed from diagnostics
        self.assertNotIn(test_topic, self.diagnostics_monitor.ui_diagnostics)

    def test_set_expected_frequency_operations(
            self, expected_frequency, message_type, tolerance_hz):
        """Test setting and clearing expected frequencies."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running frequency setting tests once')

        test_topic = '/test_topic2'
        test_freq = 42.5
        test_tolerance = 15.0

        # Initially no expected frequency
        freq, tolerance = self.diagnostics_monitor.get_expected_frequency(test_topic)
        self.assertEqual((freq, tolerance), (0.0, 0.0))

        # Set expected frequency
        success, message = self.diagnostics_monitor.set_expected_frequency(
            test_topic, test_freq, test_tolerance
        )
        self.assertTrue(success, f'Failed to set frequency: {message}')

        # Check that frequency was stored locally
        freq, tolerance = self.diagnostics_monitor.get_expected_frequency(test_topic)
        self.assertEqual((freq, tolerance), (test_freq, test_tolerance))

        # Clear expected frequency
        success, message = self.diagnostics_monitor.set_expected_frequency(
            test_topic, clear=True
        )
        self.assertTrue(success, f'Failed to clear frequency: {message}')

        # Should be back to defaults
        freq, tolerance = self.diagnostics_monitor.get_expected_frequency(test_topic)
        self.assertEqual((freq, tolerance), (0.0, 0.0))

    def test_diagnostic_data_thread_safety(self, expected_frequency, message_type, tolerance_hz):
        """Test thread safety of diagnostic data updates."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running thread safety tests once')

        test_topic = '/test_topic'
        update_count = 0
        error_occurred = False

        def update_thread():
            nonlocal update_count, error_occurred
            try:
                for _ in range(50):
                    # Simulate concurrent diagnostic updates
                    data = self.diagnostics_monitor.get_topic_diagnostics(test_topic)
                    if data.status != '-':
                        update_count += 1
                    time.sleep(0.01)
            except Exception:
                error_occurred = True

        def spin_thread():
            nonlocal error_occurred
            try:
                for _ in range(100):
                    rclpy.spin_once(self.test_node, timeout_sec=0.01)
                    time.sleep(0.005)
            except Exception:
                error_occurred = True

        # Start concurrent threads
        threads = [
            threading.Thread(target=update_thread),
            threading.Thread(target=spin_thread)
        ]

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()

        # Should not have encountered any thread safety issues
        self.assertFalse(error_occurred, 'Thread safety error occurred')
        self.assertGreater(update_count, 0, 'Should have received some diagnostic updates')

    def test_get_topic_diagnostics_nonexistent_topic(
            self, expected_frequency, message_type, tolerance_hz):
        """Test getting diagnostics for non-existent topic."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running diagnostic retrieval tests once')

        # Request diagnostics for non-monitored topic
        data = self.diagnostics_monitor.get_topic_diagnostics('/nonexistent_topic')

        # Should return default values
        expected_default = UiDiagnosticData()
        self.assertEqual(data.pub_rate, expected_default.pub_rate)
        self.assertEqual(data.msg_rate, expected_default.msg_rate)
        self.assertEqual(data.latency, expected_default.latency)
        self.assertEqual(data.status, expected_default.status)

    def test_diagnostics_callback_processing(self, expected_frequency, message_type, tolerance_hz):
        """Test that diagnostic callbacks are processed correctly."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running callback processing tests once')

        test_topic = '/test_topic'

        # Spin to receive diagnostics
        for _ in range(20):  # Give enough time to receive diagnostics
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Should have received diagnostics for the test topic
        topic_data = self.diagnostics_monitor.get_topic_diagnostics(test_topic)

        # Should have received real diagnostic data (not default values)
        self.assertNotEqual(topic_data.status, '-')
        self.assertNotEqual(topic_data.pub_rate, '-')

        # Check that timestamp was updated recently
        self.assertGreater(topic_data.last_update, time.time() - 10.0)

    def test_service_timeout_handling(self, expected_frequency, message_type, tolerance_hz):
        """Test service call timeout handling."""
        if (message_type, expected_frequency, tolerance_hz) != MANAGE_TOPIC_TEST_CONFIG:
            self.skipTest('Only running timeout handling tests once')

        # Create a client to a non-existent service
        fake_client = self.test_node.create_client(ManageTopic, '/nonexistent_service')

        # Replace the real client temporarily
        original_client = self.diagnostics_monitor.manage_topic_client
        self.diagnostics_monitor.manage_topic_client = fake_client

        try:
            # This should handle the service not being available gracefully
            self.diagnostics_monitor.toggle_topic_monitoring('/some_topic')
            # Should not crash or raise exceptions
        finally:
            # Restore original client
            self.diagnostics_monitor.manage_topic_client = original_client
            self.test_node.destroy_client(fake_client)


if __name__ == '__main__':
    unittest.main()
