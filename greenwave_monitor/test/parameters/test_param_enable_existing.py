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

"""Test: add_topic enables existing node's parameter instead of creating local diagnostics."""

import time
import unittest

from greenwave_monitor.test_utils import (
    call_manage_topic_service,
    create_minimal_publisher,
    create_monitor_node,
    create_service_clients,
    RosNodeTestCase,
    wait_for_service_connection,
)
from greenwave_monitor.ui_adaptor import ENABLED_SUFFIX, TOPIC_PARAM_PREFIX
import launch
import launch_testing
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy


TEST_TOPIC = '/enable_existing_test_topic'
TEST_FREQUENCY = 50.0
PUBLISHER_NODE_NAME = 'minimal_publisher_node_enable_test'


def make_enabled_param(topic: str) -> str:
    """Build enabled parameter name for a topic."""
    return f'{TOPIC_PARAM_PREFIX}{topic}{ENABLED_SUFFIX}'


@pytest.mark.launch_test
def generate_test_description():
    """Launch monitor and publisher nodes for testing."""
    ros2_monitor_node = create_monitor_node(topics=[''])

    publisher = create_minimal_publisher(
        TEST_TOPIC, TEST_FREQUENCY, 'imu', '_enable_test'
    )

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            publisher,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


class TestEnableExistingNodeParameter(RosNodeTestCase):
    """Test that add_topic enables parameter on existing nodes."""

    TEST_NODE_NAME = 'enable_existing_test_node'

    def test_add_topic_enables_existing_node_parameter(self):
        """Test that adding a topic with an existing publisher sets the enabled parameter."""
        time.sleep(3.0)

        publisher_full_name = f'/{PUBLISHER_NODE_NAME}'

        # Create a client to get parameters from the publisher node
        get_params_client = self.test_node.create_client(
            GetParameters, f'{publisher_full_name}/get_parameters'
        )
        self.assertTrue(
            get_params_client.wait_for_service(timeout_sec=5.0),
            'Get parameters service not available on publisher node'
        )

        enabled_param_name = make_enabled_param(TEST_TOPIC)

        # Verify the publisher node has the enabled parameter
        request = GetParameters.Request()
        request.names = [enabled_param_name]
        future = get_params_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'Failed to get parameters from publisher')
        self.assertTrue(
            len(future.result().values) > 0,
            f'Publisher node should have parameter {enabled_param_name}'
        )

        # Create service client for manage_topic
        manage_topic_client = create_service_clients(self.test_node)
        self.assertTrue(
            wait_for_service_connection(
                self.test_node, manage_topic_client,
                timeout_sec=5.0, service_name='manage_topic'
            ),
            'ManageTopic service not available'
        )

        # Call add_topic for the test topic
        response = call_manage_topic_service(
            self.test_node, manage_topic_client, add=True, topic=TEST_TOPIC
        )
        self.assertIsNotNone(response, 'ManageTopic service call failed')
        self.assertTrue(response.success, f'Failed to add topic: {response.message}')

        # Verify the response message indicates it enabled monitoring on existing node
        self.assertIn(
            'Enabled monitoring on existing node',
            response.message,
            f'Expected message about enabling existing node, got: {response.message}'
        )

        # Verify the enabled parameter is now true on the publisher node
        request = GetParameters.Request()
        request.names = [enabled_param_name]
        future = get_params_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'Failed to get parameters after add_topic')
        self.assertTrue(
            len(future.result().values) > 0,
            'Parameter should still exist after add_topic'
        )
        self.assertTrue(
            future.result().values[0].bool_value,
            'Enabled parameter should be true after add_topic'
        )

        # Cleanup
        self.test_node.destroy_client(get_params_client)
        self.test_node.destroy_client(manage_topic_client)


if __name__ == '__main__':
    unittest.main()
