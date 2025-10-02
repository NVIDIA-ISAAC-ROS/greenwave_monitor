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

"""
Ncurses-based frontend for Greenwave Monitor.

This module provides a terminal-based interface for monitoring ROS2 topics
with real-time diagnostics including publication rates, latency, and status.
"""

import curses
import signal
import threading
from threading import Lock
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from .ui_adaptor import GreenwaveUiAdaptor


class GreenwaveNcursesFrontend(Node):
    """Ncurses frontend for Greenwave Monitor."""

    def __init__(self):
        """Initialize the ncurses frontend node."""
        super().__init__('greenwave_ncurses_frontend')

        self.running = True
        self.selected_row = 0
        self.scroll_offset = 0
        self.ui_adaptor: Optional[GreenwaveUiAdaptor] = None

        # Topic management
        self.topics_lock = Lock()
        self.all_topics = set()  # Set of all available topic names
        self.visible_topics = []  # Filtered list of topics to display

        # UI state
        self.input_mode = None
        self.input_buffer = ''
        self.status_message = ''
        self.status_timeout = 0
        self.show_only_monitored = False

        # Initialize UI adaptor
        self.ui_adaptor = GreenwaveUiAdaptor(self)

        # Timer to periodically update the topic list
        self.topic_update_timer = self.create_timer(5.0, self.update_topic_list)

        # Initial topic list update
        self.update_topic_list()

    def update_topic_list(self):
        """Periodically update the list of all available topics."""
        try:
            topic_names_and_types = self.get_topic_names_and_types()

            # Filter out nitros topics
            topic_names_and_types = [
                (topic_name, topic_type)
                for topic_name, topic_type in topic_names_and_types
                if (not topic_name.endswith('/nitros')) and ('/nitros/' not in topic_name)
            ]

            topic_set = {topic_name for topic_name, topic_type in topic_names_and_types}

            with self.topics_lock:
                # Update topics
                self.all_topics = topic_set

                # Update visible topics with filtering
                self.update_visible_topics()

        except Exception as e:
            self.get_logger().warning(f'Error updating topic list: {e}')

    def update_visible_topics(self):
        """Update the visible topics list based on current filters."""
        all_topic_names = list(self.all_topics)

        if self.show_only_monitored and self.ui_adaptor:
            # Filter to only show topics that have diagnostic data (are being monitored)
            filtered_topics = []
            for topic_name in all_topic_names:
                diag = self.ui_adaptor.get_topic_diagnostics(topic_name)
                if diag.status != '-':  # Topic is monitored
                    filtered_topics.append(topic_name)
            self.visible_topics = filtered_topics
        else:
            # Show all topics
            self.visible_topics = all_topic_names

        self.visible_topics.sort()

    def toggle_topic_monitoring(self, topic_name: str):
        """Toggle monitoring for a topic."""
        if self.ui_adaptor:
            self.ui_adaptor.toggle_topic_monitoring(topic_name)
            self.show_status(f'Toggling monitoring for {topic_name}')
        else:
            self.show_status('UI adaptor not available')

    def show_status(self, message: str):
        """Show a status message for 3 seconds."""
        self.status_message = message
        self.status_timeout = time.time() + 3.0


def curses_main(stdscr, node):
    """Run the main curses UI loop for displaying topics and diagnostics."""
    stdscr.nodelay(True)
    curses.curs_set(0)
    stdscr.keypad(True)
    stdscr.leaveok(True)

    start_idx = 0
    key = -1
    selected_row = 0
    redraw_interval = 0.1
    last_redraw = 0
    status_message = ''
    status_timeout = 0
    input_mode = None
    input_buffer = ''

    # Initialize color pairs
    curses.start_color()
    COLOR_OK = 1
    COLOR_UNMONITORED = 2
    COLOR_BUTTON_ADD = 3
    COLOR_ERROR = 4
    COLOR_WARN = 5
    COLOR_STATUS_MSG = 6
    COLOR_STALE = 7

    curses.init_pair(COLOR_OK, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(COLOR_UNMONITORED, curses.COLOR_WHITE, curses.COLOR_BLACK)
    curses.init_pair(COLOR_BUTTON_ADD, curses.COLOR_BLACK, curses.COLOR_GREEN)
    curses.init_pair(COLOR_ERROR, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(COLOR_WARN, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(COLOR_STATUS_MSG, curses.COLOR_BLUE, curses.COLOR_BLACK)
    curses.init_pair(COLOR_STALE, curses.COLOR_CYAN, curses.COLOR_BLACK)

    while rclpy.ok() and node.running:
        current_time = time.time()

        # Always check for input
        try:
            key = stdscr.getch()
        except Exception:
            key = -1

        # Only redraw if we got input or enough time has passed
        should_redraw = (key != -1) or (current_time - last_redraw >= redraw_interval)

        if not should_redraw:
            time.sleep(0.01)
            continue

        last_redraw = current_time
        stdscr.erase()
        height, width = stdscr.getmaxyx()

        # Calculate column widths
        MAX_NAME_WIDTH = 60
        FRAME_RATE_WIDTH = 16
        REALTIME_DELAY_WIDTH = 16
        STATUS_WIDTH = 18
        BUTTON_WIDTH = 10

        total_width_needed = (
            MAX_NAME_WIDTH + 2 * FRAME_RATE_WIDTH + REALTIME_DELAY_WIDTH +
            STATUS_WIDTH + BUTTON_WIDTH + 5)
        if total_width_needed > width:
            scaling_factor = width / total_width_needed
            MAX_NAME_WIDTH = int(MAX_NAME_WIDTH * scaling_factor)
            FRAME_RATE_WIDTH = int(FRAME_RATE_WIDTH * scaling_factor)
            REALTIME_DELAY_WIDTH = int(REALTIME_DELAY_WIDTH * scaling_factor)
            STATUS_WIDTH = int(STATUS_WIDTH * scaling_factor)

        # Draw header
        header = (f'{"Topic Name":<{MAX_NAME_WIDTH}} {"Status":<{STATUS_WIDTH}} '
                  f'{"Pub Rate (Hz)":<{FRAME_RATE_WIDTH}} '
                  f'{"Latency (ms)":<{REALTIME_DELAY_WIDTH}} {"Expected Hz":<12}')
        separator_width = min(
            width - BUTTON_WIDTH - 2,
            MAX_NAME_WIDTH + FRAME_RATE_WIDTH + REALTIME_DELAY_WIDTH + STATUS_WIDTH + 12 + 4)
        separator = '-' * separator_width

        try:
            stdscr.addstr(0, 0, header[:width - BUTTON_WIDTH - 3])
            stdscr.addstr(1, 0, separator)
        except curses.error:
            pass

        # Handle key presses
        if input_mode:
            # Handle input mode
            if key == 27:  # ESC
                input_mode = None
                input_buffer = ''
            elif key == 10 or key == 13:  # Enter
                if input_mode == 'frequency' and 0 <= selected_row < len(node.visible_topics):
                    topic_name = node.visible_topics[selected_row]
                    try:
                        parts = input_buffer.strip().split()
                        if len(parts) >= 1:
                            hz = float(parts[0])
                            tolerance = float(parts[1]) if len(parts) > 1 else 5.0
                            success, msg = node.ui_adaptor.set_expected_frequency(
                                topic_name, hz, tolerance)
                            status_message = f'Set frequency for {topic_name}: {hz}Hz'
                            if not success:
                                status_message = f'Error: {msg}'
                        else:
                            status_message = 'Invalid input format'
                    except ValueError:
                        status_message = 'Invalid frequency values'
                    status_timeout = current_time + 3.0
                input_mode = None
                input_buffer = ''
            elif key == curses.KEY_BACKSPACE or key == 127:
                input_buffer = input_buffer[:-1]
            elif 32 <= key <= 126:  # Printable characters
                input_buffer += chr(key)
        else:
            # Normal navigation mode
            if key == curses.KEY_UP:
                selected_row = max(0, selected_row - 1)
                if selected_row < start_idx:
                    start_idx = selected_row
            elif key == curses.KEY_DOWN:
                if len(node.visible_topics) > 0:
                    selected_row = min(len(node.visible_topics) - 1, selected_row + 1)
                if selected_row >= start_idx + (height - 5):
                    start_idx = min(
                        len(node.visible_topics) - (height - 5),
                        selected_row - (height - 5) + 1)
            elif key == curses.KEY_PPAGE:  # Page Up
                visible_height = height - 5
                start_idx = max(0, start_idx - visible_height)
                selected_row = max(0, selected_row - visible_height)
            elif key == curses.KEY_NPAGE:  # Page Down
                visible_height = height - 5
                if len(node.visible_topics) > 0:
                    start_idx = min(len(node.visible_topics) - visible_height,
                                    start_idx + visible_height)
                    selected_row = min(len(node.visible_topics) - 1, selected_row + visible_height)
            elif key == ord('q') or key == ord('Q'):
                node.running = False
                break
            elif key == ord('\n') or key == ord(' '):
                if 0 <= selected_row < len(node.visible_topics):
                    topic_name = node.visible_topics[selected_row]
                    node.toggle_topic_monitoring(topic_name)
                    status_message = f'Toggled monitoring for {topic_name}'
                    status_timeout = current_time + 3.0
            elif key == ord('f') or key == ord('F'):
                if 0 <= selected_row < len(node.visible_topics):
                    input_mode = 'frequency'
                    input_buffer = ''
            elif key == ord('c') or key == ord('C'):
                if 0 <= selected_row < len(node.visible_topics):
                    topic_name = node.visible_topics[selected_row]
                    success, msg = node.ui_adaptor.set_expected_frequency(
                        topic_name, clear=True)
                    status_message = f'Cleared frequency for {topic_name}'
                    if not success:
                        status_message = f'Error: {msg}'
                    status_timeout = current_time + 3.0
            elif key == ord('h') or key == ord('H'):
                node.show_only_monitored = not node.show_only_monitored
                with node.topics_lock:
                    node.update_visible_topics()
                mode_text = 'monitored only' if node.show_only_monitored else 'all topics'
                status_message = f'Showing {mode_text}'
                status_timeout = current_time + 3.0

        # Get data safely
        with node.topics_lock:
            visible_topics = node.visible_topics.copy()

        # Ensure indices are within valid range
        if len(visible_topics) == 0:
            selected_row = 0
            start_idx = 0
        else:
            selected_row = min(selected_row, len(visible_topics) - 1)
            start_idx = min(start_idx, max(0, len(visible_topics) - (height - 5)))
            start_idx = max(0, start_idx)

        visible_height = height - 5
        visible_topics_slice = visible_topics[start_idx:start_idx + visible_height]

        # Draw visible topics
        for idx, topic_name in enumerate(visible_topics_slice):
            actual_idx = idx + start_idx

            # Check monitoring status via ui_adaptor
            is_monitored = False
            status_display = ''
            frame_rate_node = 'N/A'.ljust(FRAME_RATE_WIDTH)
            current_delay_from_realtime_ms = 'N/A'.ljust(REALTIME_DELAY_WIDTH)
            expected_freq_display = '-'.ljust(12)

            if node.ui_adaptor:
                diag = node.ui_adaptor.get_topic_diagnostics(topic_name)
                if diag.status != '-':
                    is_monitored = True
                    status_display = diag.status  # Use actual diagnostic status
                    frame_rate_node = (diag.pub_rate.ljust(FRAME_RATE_WIDTH)
                                       if diag.pub_rate != '-' else 'N/A'.ljust(FRAME_RATE_WIDTH))
                    current_delay_from_realtime_ms = (
                        diag.latency.ljust(REALTIME_DELAY_WIDTH)
                        if diag.latency != '-' else 'N/A'.ljust(REALTIME_DELAY_WIDTH))

                # Get expected frequency
                expected_hz, tolerance = node.ui_adaptor.get_expected_frequency(topic_name)
                if expected_hz > 0:
                    expected_freq_display = f'{expected_hz:.1f}Hz'.ljust(12)

            # Color coding based on status
            if is_monitored:
                if status_display == 'OK':
                    color_pair = curses.color_pair(COLOR_OK)
                elif status_display == 'WARN':
                    color_pair = curses.color_pair(COLOR_WARN)
                elif status_display == 'STALE':
                    color_pair = curses.color_pair(COLOR_STALE)
                elif status_display == 'ERROR':
                    color_pair = curses.color_pair(COLOR_ERROR)
                else:
                    color_pair = curses.color_pair(COLOR_OK)  # Default green for monitored
            else:
                color_pair = curses.color_pair(COLOR_UNMONITORED)

            status_display = status_display.ljust(STATUS_WIDTH)

            # Format topic name with truncation
            if len(topic_name) > MAX_NAME_WIDTH:
                name_display = topic_name[:MAX_NAME_WIDTH-3] + '...'
            else:
                name_display = topic_name.ljust(MAX_NAME_WIDTH)

            # Build display line
            line = f'{name_display} {status_display} {frame_rate_node}'
            line += f' {current_delay_from_realtime_ms} {expected_freq_display}'
            line = line[:width - BUTTON_WIDTH - 3]

            try:
                is_selected = (actual_idx == selected_row)
                stdscr.addstr(idx + 2, 0, line,
                              curses.A_REVERSE if is_selected else color_pair)

                # Draw button
                button_text = ' [Remove] ' if is_monitored else ' [Add] '
                button_x = width - len(button_text) - 1

                if is_selected:
                    button_color = (curses.color_pair(COLOR_ERROR) if is_monitored
                                    else curses.color_pair(COLOR_BUTTON_ADD))
                else:
                    button_color = curses.color_pair(COLOR_WARN)
                stdscr.addstr(idx + 2, button_x, button_text, button_color)

            except curses.error:
                pass

        # Add scroll indicators
        try:
            if start_idx > 0:
                stdscr.addstr(2, width - BUTTON_WIDTH - 3, '↑')
            if start_idx + visible_height < len(visible_topics):
                stdscr.addstr(min(height - 3, 2 + visible_height),
                              width - BUTTON_WIDTH - 3, '↓')
        except curses.error:
            pass

        # Status message
        if current_time < status_timeout:
            try:
                stdscr.addstr(height - 3, 0, status_message[:width-1],
                              curses.color_pair(COLOR_STATUS_MSG))
            except curses.error:
                pass

        # Show node status message
        if current_time < node.status_timeout:
            try:
                stdscr.addstr(height - 3, 0, node.status_message[:width-1],
                              curses.color_pair(COLOR_STATUS_MSG))
            except curses.error:
                pass

        # Input prompt (if in input mode)
        if input_mode:
            try:
                prompt = f'Set frequency: {input_buffer}'
                stdscr.addstr(height - 3, 0, prompt[:width-1],
                              curses.color_pair(COLOR_STATUS_MSG))
                # Position cursor after the input
                cursor_x = len(prompt)
                if cursor_x < width - 1:
                    stdscr.move(height - 3, cursor_x)
                    curses.curs_set(1)  # Show cursor
            except curses.error:
                pass
        else:
            curses.curs_set(0)  # Hide cursor when not in input mode

        # Footer
        num_shown = min(start_idx + len(visible_topics_slice), len(visible_topics))
        if input_mode:
            status_line = ("Format: Hz [tolerance%] - Examples: '30' (30Hz±5% default) "
                           "or '30 10' (30Hz±10%) - ESC=cancel, Enter=confirm")
        else:
            mode_text = 'monitored only' if node.show_only_monitored else 'all topics'
            status_line = (
                f'Showing {start_idx + 1} - {num_shown} of {len(visible_topics)} '
                f'topics ({mode_text}). Enter=toggle, f=set freq, c=clear freq, '
                f'h=hide unmonitored, q=quit')

        try:
            stdscr.addstr(height - 2, 0, status_line[:width - 1])
        except curses.error:
            pass

        stdscr.refresh()


def main(args=None):
    """Entry point for the ncurses frontend application."""
    rclpy.init(args=args)
    node = GreenwaveNcursesFrontend()
    thread = None

    def signal_handler(signum, frame):
        """Handle shutdown signals gracefully."""
        node.running = False

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=False)
        thread.start()
        curses.wrapper(curses_main, node)
    except KeyboardInterrupt:
        node.running = False
    finally:
        node.running = False
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
