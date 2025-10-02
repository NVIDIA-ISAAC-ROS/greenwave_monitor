# Services

The Greenwave Monitor provides two services. The `ManageTopic` service dynamically adds or removes topics from monitoring. The `SetExpectedFrequency` service dynamically sets or clears expected frequencies for a specified topic, which enables additional diagnostic values and statuses.

## Manage Topic

The monitor node exposes a `/greenwave_monitor/manage_topic` service that follows the `greenwave_monitor_interfaces/srv/ManageTopic` service definition.

**Usage Examples**

To add a topic to the monitoring list:
```bash
ros2 service call /greenwave_monitor/manage_topic greenwave_monitor_interfaces/srv/ManageTopic "{topic_name: '/topic2', add_topic: true}"
```

To remove a topic from the monitoring list:
```bash
ros2 service call /greenwave_monitor/manage_topic greenwave_monitor_interfaces/srv/ManageTopic "{topic_name: '/topic2', add_topic: false}"
```

## Set Expected Frequency

The monitor node exposes a `/greenwave_monitor/set_expected_frequency` service that follows the `greenwave_monitor_interfaces/srv/SetExpectedFrequency` service definition.

**Usage Examples**

To set the expected frequency for a topic:
```bash
ros2 service call /greenwave_monitor/set_expected_frequency greenwave_monitor_interfaces/srv/SetExpectedFrequency "{topic_name: '/topic2', expected_hz: <float>, tolerance_percent: <float>, add_topic_if_missing: true}"
```

To clear the expected frequency for a topic:
```bash
ros2 service call /greenwave_monitor/set_expected_frequency greenwave_monitor_interfaces/srv/SetExpectedFrequency "{topic_name: '/topic2', clear_expected: true}"
```

Note: The topic name must include the leading slash (e.g., '/topic2' not 'topic2').
