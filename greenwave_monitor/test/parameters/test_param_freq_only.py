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

"""Test: only expected_frequency specified, tolerance defaults to 5%."""

import time

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


TEST_TOPIC = '/freq_only_topic'
TEST_FREQUENCY = 50.0


@pytest.mark.launch_test
def generate_test_description():
    """Test with only expected_frequency specified."""
    topic_configs = {
        TEST_TOPIC: {
            'expected_frequency': TEST_FREQUENCY
            # No tolerance - should default to 5%
        }
    }

    ros2_monitor_node = create_monitor_node(
        topic_configs=topic_configs
    )

    publisher = create_minimal_publisher(
        TEST_TOPIC, TEST_FREQUENCY, 'imu', '_freq_only'
    )

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            publisher,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


class TestFrequencyOnlyParameter(RosNodeTestCase):
    """Test that only specifying frequency works (tolerance defaults)."""

    TEST_NODE_NAME = 'freq_only_test_node'

    def test_frequency_only_uses_default_tolerance(self):
        """Test that specifying only frequency uses default tolerance."""
        time.sleep(2.0)

        received_diagnostics = collect_diagnostics_for_topic(
            self.test_node, TEST_TOPIC, expected_count=3, timeout_sec=10.0
        )

        self.assertGreaterEqual(
            len(received_diagnostics), 3,
            f'Expected at least 3 diagnostics, got {len(received_diagnostics)}'
        )
        best_status, _ = find_best_diagnostic(
            received_diagnostics, TEST_FREQUENCY, 'imu'
        )
        self.assertIsNotNone(
            best_status,
            'Should have valid frame rate with default tolerance'
        )


if __name__ == '__main__':
    unittest.main()
