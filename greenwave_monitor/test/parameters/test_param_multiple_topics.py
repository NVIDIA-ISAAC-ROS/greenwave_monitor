#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test: multiple topics configured via parameters at startup."""

import time
import unittest

from greenwave_monitor.test_utils import (
    collect_diagnostics_for_topic,
    create_minimal_publisher,
    create_monitor_node,
    find_best_diagnostic,
    RosNodeTestCase,
)
import launch
import launch_testing
import pytest


TOPIC_1 = '/multi_topic_1'
TOPIC_2 = '/multi_topic_2'
TOPIC_3 = '/multi_topic_3'
FREQUENCY_1 = 10.0
FREQUENCY_2 = 25.0
FREQUENCY_3 = 50.0
TOLERANCE = 20.0


@pytest.mark.launch_test
def generate_test_description():
    """Test multiple topics configured via parameters."""
    topic_configs = {
        TOPIC_1: {
            'expected_frequency': FREQUENCY_1,
            'tolerance': TOLERANCE
        },
        TOPIC_2: {
            'expected_frequency': FREQUENCY_2,
            'tolerance': TOLERANCE
        },
        TOPIC_3: {
            'expected_frequency': FREQUENCY_3,
            'tolerance': TOLERANCE
        }
    }

    ros2_monitor_node = create_monitor_node(topic_configs=topic_configs)

    publisher_1 = create_minimal_publisher(TOPIC_1, FREQUENCY_1, 'imu', '_multi_1')
    publisher_2 = create_minimal_publisher(TOPIC_2, FREQUENCY_2, 'imu', '_multi_2')
    publisher_3 = create_minimal_publisher(TOPIC_3, FREQUENCY_3, 'imu', '_multi_3')

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            publisher_1,
            publisher_2,
            publisher_3,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


class TestMultipleTopicsViaParameters(RosNodeTestCase):
    """Test that multiple topics can be configured via parameters."""

    TEST_NODE_NAME = 'multiple_topics_test_node'

    def test_all_topics_monitored(self):
        """Test that all configured topics are monitored."""
        time.sleep(2.0)

        topics_to_check = [
            (TOPIC_1, FREQUENCY_1),
            (TOPIC_2, FREQUENCY_2),
            (TOPIC_3, FREQUENCY_3),
        ]

        for topic, expected_freq in topics_to_check:
            with self.subTest(topic=topic):
                diagnostics = collect_diagnostics_for_topic(
                    self.test_node, topic, expected_count=3, timeout_sec=10.0
                )
                self.assertGreaterEqual(
                    len(diagnostics), 3,
                    f'Expected at least 3 diagnostics for {topic}'
                )

                best_status, best_values = find_best_diagnostic(
                    diagnostics, expected_freq, 'imu'
                )
                self.assertIsNotNone(
                    best_status,
                    f'Should have valid diagnostics for {topic}'
                )

                frame_rate = best_values[0]
                tolerance_hz = expected_freq * TOLERANCE / 100.0
                self.assertAlmostEqual(
                    frame_rate, expected_freq, delta=tolerance_hz,
                    msg=f'{topic}: frame rate {frame_rate} not within '
                        f'{tolerance_hz} of expected {expected_freq}'
                )


if __name__ == '__main__':
    unittest.main()
