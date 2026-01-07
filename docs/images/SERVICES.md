# Services and Parameters

The Greenwave Monitor provides a `ManageTopic` service to dynamically add or remove topics from monitoring. Expected frequencies are configured via ROS parameters, which enables additional diagnostic values and statuses.

## Manage Topic Service

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

## Expected Frequency Parameters

Expected frequencies are configured via ROS parameters with the following naming convention:
- `topics.<topic_name>.expected_frequency` - Expected publish rate in Hz
- `topics.<topic_name>.tolerance` - Tolerance percentage (default: 5.0%)

**Usage Examples**

To set the expected frequency for a topic:
```bash
ros2 param set /greenwave_monitor topics./topic2.expected_frequency 30.0
ros2 param set /greenwave_monitor topics./topic2.tolerance 10.0
```

To clear the expected frequency for a topic:
```bash
ros2 param delete /greenwave_monitor topics./topic2.expected_frequency
```

Parameters can also be set at launch time:
```bash
ros2 run greenwave_monitor greenwave_monitor --ros-args -p topics./topic2.expected_frequency:=30.0 -p topics./topic2.tolerance:=10.0
```

Note: The topic name must include the leading slash (e.g., '/topic2' not 'topic2').
