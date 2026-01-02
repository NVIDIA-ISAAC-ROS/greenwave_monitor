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

"""Test: add new topic to monitoring via ros2 param set."""

import time

from greenwave_monitor.test_utils import (
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    create_monitor_node,
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
        NEW_TOPIC, TEST_FREQUENCY, 'imu', '_new_dynamic'
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
        """Test that setting frequency param for new topic starts monitoring."""
        time.sleep(2.0)

        initial_diagnostics = collect_diagnostics_for_topic(
            self.test_node, NEW_TOPIC, expected_count=1, timeout_sec=2.0
        )
        self.assertEqual(
            len(initial_diagnostics), 0,
            'Topic should not be monitored initially'
        )

        freq_param = make_freq_param(NEW_TOPIC)
        success = set_parameter(self.test_node, freq_param, TEST_FREQUENCY)
        self.assertTrue(success, f'Failed to set {freq_param}')

        time.sleep(2.0)

        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, NEW_TOPIC, expected_count=3, timeout_sec=10.0
        )

        self.assertGreaterEqual(
            len(received_diagnostics), 3,
            'Should monitor new topic after setting frequency param'
        )


if __name__ == '__main__':
    unittest.main()
