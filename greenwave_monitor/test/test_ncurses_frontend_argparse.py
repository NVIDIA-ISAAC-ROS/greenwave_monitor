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
"""Tests for ncurses frontend argument parsing."""

from greenwave_monitor.ncurses_frontend import parse_args


class TestParseArgs:
    """Test argument parsing for ncurses frontend."""

    def test_default_hide_unmonitored_false(self):
        """Test that hide_unmonitored defaults to False."""
        parsed_args, _ = parse_args([])
        assert parsed_args.hide_unmonitored is False

    def test_hide_unmonitored_long_flag(self):
        """Test --hide-unmonitored flag enables hide_unmonitored."""
        parsed_args, _ = parse_args(['--hide-unmonitored'])
        assert parsed_args.hide_unmonitored is True

    def test_ros_args_passthrough(self):
        """Test that ROS arguments are passed through."""
        parsed_args, ros_args = parse_args(
            ['--hide-unmonitored', '--ros-args', '-r', '__node:=my_node'])
        assert parsed_args.hide_unmonitored is True
        assert '--ros-args' in ros_args
        assert '-r' in ros_args
        assert '__node:=my_node' in ros_args
