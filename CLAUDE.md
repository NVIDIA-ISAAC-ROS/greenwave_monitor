# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Greenwave Monitor is a ROS 2 diagnostic tool for runtime monitoring of topic frame rates and latency. The project consists of two packages:

1. **greenwave_monitor** - Core C++ monitoring node, ncurses dashboard frontend, and header-only library
2. **greenwave_monitor_interfaces** - ROS service definitions (ManageTopic, SetExpectedFrequency)

## Build Commands

```bash
# Build all packages
cd <ros_workspace>
colcon build --packages-up-to greenwave_monitor

# Build individual packages
colcon build --packages-select greenwave_monitor
colcon build --packages-select greenwave_monitor_interfaces
```

## Testing

```bash
# Run all tests for greenwave_monitor
colcon test --packages-select greenwave_monitor
colcon test-result --verbose

# Run specific test types
colcon test --packages-select greenwave_monitor --pytest-args -k test_greenwave_monitor
colcon test --packages-select greenwave_monitor --pytest-args -k test_topic_monitoring_integration

# C++ unit tests (gtest) are run automatically with colcon test
# - test_message_diagnostics.cpp - Tests MessageDiagnostics header-only library
# - test_minimal_publisher.cpp - Tests example publisher node

# Python linting tests
colcon test --packages-select greenwave_monitor --pytest-args -k test_flake8
colcon test --packages-select greenwave_monitor --pytest-args -k test_pep257
colcon test --packages-select greenwave_monitor --pytest-args -k test_copyright
```

## Architecture

### Core Components

**GreenwaveMonitor Node** (`greenwave_monitor.cpp`):
- Subscribes to topics using generic subscriptions (rclcpp::GenericSubscription)
- Extracts timestamps from serialized messages for latency calculation
- Uses MessageDiagnostics to track frame rates and latency per topic
- Publishes diagnostics to `/diagnostics` topic (diagnostic_msgs/DiagnosticArray)
- Exposes services at `~/manage_topic` and `~/set_expected_frequency`
- Timer callback (1Hz) triggers diagnostics publishing for all monitored topics

**MessageDiagnostics Library** (`message_diagnostics.hpp`):
- Header-only C++ library for calculating and publishing diagnostics
- Tracks frame rate using both node clock and message timestamps
- Uses rolling window filtering (default 300 messages) for smoothing
- Calculates jitter, dropped frames, and latency metrics
- Can be integrated directly into custom nodes to avoid subscription overhead
- Thread-safe with mutex protection for updateDiagnostics() and publishDiagnostics()

**UI Adaptor**:
- `ncurses_frontend.py` - Lightweight terminal dashboard
- Subscribes to `/diagnostics` topic and calls services to manage monitoring
- `ui_adaptor.py` provides GreenwaveUiAdaptor class for building custom UIs

Third-party UIs (like r2s_gw, available as a separate package) can integrate by subscribing to the `/diagnostics` topic and using the ManageTopic and SetExpectedFrequency services.

### Diagnostics Message Format

Greenwave follows Isaac ROS NITROS diagnostic conventions with these KeyValue fields:
- `frame_rate_node` - Hz calculated from node clock
- `frame_rate_msg` - Hz calculated from message timestamps
- `current_delay_from_realtime_ms` - Latency (or "N/A" if unavailable)
- `num_jitter_outliers_node/msg` - Count of deadline misses
- `max_abs_jitter_node/msg` - Maximum jitter observed (µs)
- `mean_abs_jitter_node/msg` - Average jitter (µs)
- `total_dropped_frames` - Total frames dropped

This format allows NITROS-enabled nodes to be monitored without greenwave_monitor subscribing (when `ENABLE_GLOBAL_NITROS_DIAGNOSTICS=1` is set).

### Latency Calculation

Latency is computed as: `current_time - message.header.stamp`

**Important constraints:**
- Message type must have a `std_msgs/Header` field
- Message type must be registered in `known_header_types` map in greenwave_monitor.cpp:has_header_from_type()
- Header timestamp must use epoch time (not boottime)

If conditions aren't met, latency shows as "N/A". Commonly supported types include sensor_msgs/Image, sensor_msgs/CameraInfo, etc.

**To add support for new message types:**
1. Edit `greenwave_monitor.cpp::has_header_from_type()`
2. Add entry to `known_header_types` map with type name and header offset
3. Example: `{"sensor_msgs/msg/Image", {0, true}}`

## Service API

**ManageTopic.srv** - Add/remove topics from monitoring:
```
string topic_name
bool add_topic  # true=add, false=remove
---
bool success
string message
```

**SetExpectedFrequency.srv** - Configure expected rate and tolerances:
```
string topic_name
float64 expected_hz
float64 tolerance_percent  # e.g., 5.0 = 5%
bool clear_expected        # reset to no expectation
bool add_topic_if_missing  # auto-add topic if not monitored
---
bool success
string message
```

## Contributing

All commits must be signed off using `git commit -s` to certify the Developer Certificate of Origin (DCO). This indicates you have rights to submit the contribution under the Apache 2.0 license.

CI automatically runs on PRs. For local testing across ROS distributions, use:
```bash
scripts/docker-test.sh
```

## Code Style

- C++17 standard (enforced in CMakeLists.txt)
- Python code must pass flake8, pep257, and copyright tests
- Use ament_lint_auto for automatic style checking
- Copyright header required: Apache 2.0 license with NVIDIA CORPORATION & AFFILIATES
