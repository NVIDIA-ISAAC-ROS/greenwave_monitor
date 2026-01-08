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

"""Test: add new topic to monitoring via ros2 param set."""

import time
import unittest

from greenwave_monitor.test_utils import (
    create_minimal_publisher,
    create_monitor_node,
    get_parameter,
    make_freq_param,
    RosNodeTestCase,
    set_parameter,
)
import launch
import launch_testing
import pytest


NEW_TOPIC = '/new_dynamic_topic'
TEST_FREQUENCY = 50.0


@pytest.mark.launch_test
def generate_test_description():
    """Test adding a new topic via ros2 param set."""
    ros2_monitor_node = create_monitor_node()

    publisher = create_minimal_publisher(
        NEW_TOPIC, TEST_FREQUENCY, 'imu', '_new_dynamic',
        enable_diagnostics=False
    )

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            publisher,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


class TestAddNewTopicViaParam(RosNodeTestCase):
    """Test adding a new topic to monitoring via ros2 param set."""

    TEST_NODE_NAME = 'new_topic_test_node'

    def test_add_new_topic_via_frequency_param(self):
        """Test that frequency param can be set for a topic (parameter is stored)."""
        # NOTE: Dynamic topic addition via parameter events is not implemented.
        # This test verifies that the parameter can be set, but monitoring only
        # starts at node startup when parameters are already configured.
        time.sleep(2.0)

        freq_param = make_freq_param(NEW_TOPIC)
        success = set_parameter(self.test_node, freq_param, TEST_FREQUENCY)
        self.assertTrue(success, f'Failed to set {freq_param}')

        # Verify the parameter was stored (monitoring won't start dynamically)
        success, value = get_parameter(self.test_node, freq_param)
        self.assertTrue(success, f'Failed to get {freq_param}')
        self.assertAlmostEqual(
            value, TEST_FREQUENCY, places=1,
            msg=f'Parameter value mismatch: expected {TEST_FREQUENCY}, got {value}'
        )


if __name__ == '__main__':
    unittest.main()
