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

"""Test: only tolerance specified - should NOT start monitoring."""

import time
import unittest

from greenwave_monitor.test_utils import (
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    make_tol_param,
    MONITOR_NODE_NAME,
    MONITOR_NODE_NAMESPACE,
    RosNodeTestCase,
)
import launch
import launch_ros.actions
import launch_testing
import pytest


TEST_TOPIC = '/tol_only_topic'
TEST_FREQUENCY = 50.0


@pytest.mark.launch_test
def generate_test_description():
    """Test with only tolerance specified (should not monitor)."""
    params = {
        make_tol_param(TEST_TOPIC): 15.0
    }

    ros2_monitor_node = launch_ros.actions.Node(
        package='greenwave_monitor',
        executable='greenwave_monitor',
        name=MONITOR_NODE_NAME,
        namespace=MONITOR_NODE_NAMESPACE,
        parameters=[params],
        output='screen'
    )

    publisher = create_minimal_publisher(
        TEST_TOPIC, TEST_FREQUENCY, 'imu', '_tol_only'
    )

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            publisher,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


class TestToleranceOnlyParameter(RosNodeTestCase):
    """Test that only specifying tolerance does NOT start monitoring."""

    TEST_NODE_NAME = 'tol_only_test_node'

    def test_tolerance_only_does_not_monitor(self):
        """Test that specifying only tolerance does not start monitoring."""
        time.sleep(2.0)

        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC, expected_count=1, timeout_sec=3.0
        )

        self.assertEqual(
            len(received_diagnostics), 0,
            f'Should not monitor topic with only tolerance set, got {len(received_diagnostics)}'
        )


if __name__ == '__main__':
    unittest.main()
