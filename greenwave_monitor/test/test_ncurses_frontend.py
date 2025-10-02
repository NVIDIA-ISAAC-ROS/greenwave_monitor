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

"""Smoke tests for ncurses frontend to ensure it starts and exits cleanly."""

import os
import signal
import subprocess
import time
import unittest

from greenwave_monitor.test_utils import (
    create_minimal_publisher,
    create_monitor_node,
    MONITOR_NODE_NAME
)
import launch
import launch_testing
from launch_testing import post_shutdown_test
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for ncurses frontend smoke test."""
    # Launch the greenwave_monitor node
    ros2_monitor_node = create_monitor_node(
        node_name=MONITOR_NODE_NAME,
        topics=['/test_topic']
    )

    # Create a test publisher
    test_publisher = create_minimal_publisher('/test_topic', 30.0, 'image')

    return (
        launch.LaunchDescription([
            ros2_monitor_node,
            test_publisher,
            launch_testing.actions.ReadyToTest()
        ]), {}
    )


class TestNcursesFrontendSmokeTest(unittest.TestCase):
    """Smoke tests for ncurses frontend."""

    def test_ncurses_frontend_startup_and_shutdown(self):
        """Test that ncurses frontend starts and exits cleanly without crashing."""
        # Set environment to use a fake terminal for ncurses
        env = os.environ.copy()
        env['TERM'] = 'xterm'

        # Start the ncurses frontend in a subprocess
        proc = subprocess.Popen(
            ['python3', '-m', 'greenwave_monitor.ncurses_frontend'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env
        )

        try:
            # Give it time to initialize
            time.sleep(2.0)

            # Check if process is still running (didn't crash on startup)
            poll_result = proc.poll()
            self.assertIsNone(poll_result, 'Process should still be running after startup')

            # Send 'q' to quit
            if proc.stdin:
                proc.stdin.write(b'q')
                proc.stdin.flush()

            # Wait for clean exit with timeout
            try:
                return_code = proc.wait(timeout=5.0)
                # Should exit cleanly with code 0 (not 11 for SIGSEGV or -6 for SIGABRT)
                self.assertEqual(
                    return_code, 0,
                    f'Process should exit cleanly with code 0, got {return_code}'
                )
            except subprocess.TimeoutExpired:
                self.fail('Process did not exit within timeout after quit command')

        finally:
            # Ensure cleanup
            if proc.poll() is None:
                proc.send_signal(signal.SIGTERM)
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()

    def test_ncurses_frontend_signal_handling(self):
        """Test that ncurses frontend handles SIGINT and SIGTERM gracefully."""
        env = os.environ.copy()
        env['TERM'] = 'xterm'

        for sig in [signal.SIGINT, signal.SIGTERM]:
            with self.subTest(signal=sig):
                proc = subprocess.Popen(
                    ['python3', '-m', 'greenwave_monitor.ncurses_frontend'],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    env=env
                )

                try:
                    # Give it time to initialize
                    time.sleep(2.0)

                    # Send signal
                    proc.send_signal(sig)

                    # Wait for clean exit
                    try:
                        return_code = proc.wait(timeout=5.0)
                        # Should exit cleanly (0 or terminated by signal)
                        # SIGINT/SIGTERM result in negative return codes on Unix
                        self.assertIn(
                            return_code, [0, -sig, 128 + sig],
                            f'Process should handle {sig} gracefully, got {return_code}'
                        )
                    except subprocess.TimeoutExpired:
                        self.fail(f'Process did not exit within timeout after {sig}')

                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.wait()


@post_shutdown_test()
class TestNcursesFrontendPostShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_monitor_shutdown(self, proc_info):
        """Test that all processes shut down correctly."""
        assertExitCodes(proc_info, allowable_exit_codes=[0])


if __name__ == '__main__':
    unittest.main()
