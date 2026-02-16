# Greenwave Monitor Design And Implementation Guide

This page explains how greenwave monitor is built, and how to inline
`greenwave_diagnostics` directly into your own ROS 2 node.

## Design Goals

- Provide low-overhead runtime topic diagnostics (rate, latency, jitter signals).
- Support both deployment styles:
  - centralized monitor node (`greenwave_monitor`)
  - inline diagnostics in producer/consumer nodes (`greenwave_diagnostics.hpp`)
- Publish standard `diagnostic_msgs/DiagnosticArray` on `/diagnostics` so multiple frontends can consume it.

## Overall Design

### 1) Central monitor node (`greenwave_monitor`)

Implementation: `greenwave_monitor/src/greenwave_monitor.cpp`

- Starts a generic subscription per monitored topic.
- Reads topic list from:
  - `gw_monitored_topics`
  - `gw_frequency_monitored_topics.<topic>.expected_frequency`
  - `gw_frequency_monitored_topics.<topic>.tolerance`
- Resolves topic type dynamically and uses serialized subscriptions.
- Extracts message timestamps for known header-bearing types.
- Maintains one `GreenwaveDiagnostics` instance per monitored topic.
- Publishes diagnostics once per second in `timer_callback()`.

Runtime services:

- `/greenwave_monitor/manage_topic` (`ManageTopic.srv`)
- `/greenwave_monitor/set_expected_frequency` (`SetExpectedFrequency.srv`)

### Service API and CLI Usage

The monitor exposes two runtime services for dynamic configuration:

#### `ManageTopic`

Service type: `greenwave_monitor_interfaces/srv/ManageTopic`

Request fields:

- `topic_name` (string)
- `add_topic` (bool, `true` = add, `false` = remove)

Response fields:

- `success` (bool)
- `message` (string)

Examples:

```bash
# Add a topic
ros2 service call /greenwave_monitor/manage_topic \
  greenwave_monitor_interfaces/srv/ManageTopic \
  "{topic_name: '/topic2', add_topic: true}"

# Remove a topic
ros2 service call /greenwave_monitor/manage_topic \
  greenwave_monitor_interfaces/srv/ManageTopic \
  "{topic_name: '/topic2', add_topic: false}"
```

#### `SetExpectedFrequency`

Service type: `greenwave_monitor_interfaces/srv/SetExpectedFrequency`

Request fields:

- `topic_name` (string)
- `expected_hz` (float64)
- `tolerance_percent` (float64, e.g. `5.0` = 5%)
- `clear_expected` (bool)
- `add_topic_if_missing` (bool)

Response fields:

- `success` (bool)
- `message` (string)

Examples:

```bash
# Set expected frequency/tolerance
ros2 service call /greenwave_monitor/set_expected_frequency \
  greenwave_monitor_interfaces/srv/SetExpectedFrequency \
  "{topic_name: '/topic2', expected_hz: 30.0, tolerance_percent: 10.0, add_topic_if_missing: true}"

# Clear expected frequency
ros2 service call /greenwave_monitor/set_expected_frequency \
  greenwave_monitor_interfaces/srv/SetExpectedFrequency \
  "{topic_name: '/topic2', clear_expected: true}"
```

Note: topic names should include the leading slash (`/topic2`).

### 2) Header-only diagnostics core (`greenwave_diagnostics`)

Implementation: `greenwave_monitor/include/greenwave_diagnostics.hpp`

`GreenwaveDiagnostics` tracks:

- node-time interarrival rate (`frame_rate_node`)
- message-time interarrival rate (`frame_rate_msg`)
- current delay from realtime (`current_delay_from_realtime_ms`)
- jitter/outlier counters and summary stats
- status transitions (`OK`, `ERROR`, `STALE`) for missed timing expectations

This class is thread-safe internally (`std::mutex` guards update/publish paths).

### 3) UI adapters

- `greenwave_monitor/greenwave_monitor/ui_adaptor.py` subscribes to `/diagnostics`.
- Frontends render values from key/value pairs in each `DiagnosticStatus`.
- Keep the published keys stable if you add custom producers, since UIs parse specific keys.

## Inline Integration In Your Own Node

Reference pattern: `greenwave_monitor/src/minimal_publisher_node.cpp`

### Step 1: Add the include and member

```cpp
#include "greenwave_diagnostics.hpp"

std::unique_ptr<greenwave_diagnostics::GreenwaveDiagnostics> gw_diag_;
```

### Step 2: Construct diagnostics in your node constructor

```cpp
greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
config.enable_all_topic_diagnostics = true;

gw_diag_ = std::make_unique<greenwave_diagnostics::GreenwaveDiagnostics>(
  *this, "/my_topic", config);
```

Optional expected-rate checks (enables jitter/deadline logic):

```cpp
gw_diag_->setExpectedDt(/*expected_hz=*/30.0, /*tolerance_percent=*/10.0);
```

### Step 3: Update diagnostics per message event

Call `updateDiagnostics(msg_timestamp_ns)` whenever you publish or consume.

```cpp
const auto stamp_ns = msg.header.stamp.sec * 1000000000ULL + msg.header.stamp.nanosec;
gw_diag_->updateDiagnostics(stamp_ns);
```

If your message has no header, pass node time:

```cpp
gw_diag_->updateDiagnostics(this->now().nanoseconds());
```

### Step 4: Publish diagnostics periodically

`updateDiagnostics()` does not publish by itself.
Call `publishDiagnostics()` from a timer (for example, 1 Hz):

```cpp
diag_timer_ = this->create_wall_timer(
  std::chrono::seconds(1),
  [this]() { gw_diag_->publishDiagnostics(); });
```

## Diagnostics Contract (Important For UI Compatibility)

Dashboards expect specific keys inside `DiagnosticStatus.values`, including:

- `frame_rate_node`
- `frame_rate_msg`
- `current_delay_from_realtime_ms`
- `expected_frequency`
- `tolerance`

These are already emitted by `GreenwaveDiagnostics::publishDiagnostics()`. If you write your own publisher for `/diagnostics`, keep this schema compatible.

## Implementation Notes And Pitfalls

- Message timestamp should be epoch time for latency to be meaningful.
- The central monitor only parses headers for types listed in
  `GreenwaveMonitor::has_header_from_type()`; unknown types fall back to no-header behavior.
- `publishDiagnostics()` marks status as `STALE` if no fresh `updateDiagnostics()` happened since the previous publish.
- `setExpectedDt()` requires `expected_hz > 0`; zero disables useful timing checks.

## Where To Look In Code

- Core monitor node: `greenwave_monitor/src/greenwave_monitor.cpp`
- Diagnostics API: `greenwave_monitor/include/greenwave_diagnostics.hpp`
- Inline example node: `greenwave_monitor/src/minimal_publisher_node.cpp`
- Service definitions:
  - `greenwave_monitor_interfaces/srv/ManageTopic.srv`
  - `greenwave_monitor_interfaces/srv/SetExpectedFrequency.srv`
