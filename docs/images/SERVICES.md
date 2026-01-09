# Parameters

The Greenwave Monitor uses ROS parameters for all topic configuration, including enabling/disabling monitoring and setting expected frequencies.

## Topic Monitoring Control

Topics can be enabled or disabled via the `greenwave_diagnostics.<topic>.enabled` parameter.

**Usage Examples**

To enable monitoring for a topic:
```bash
ros2 param set /greenwave_monitor greenwave_diagnostics./topic2.enabled true
```

To disable monitoring for a topic:
```bash
ros2 param set /greenwave_monitor greenwave_diagnostics./topic2.enabled false
```

## Expected Frequency Parameters

Expected frequencies are configured via ROS parameters with the following naming convention:
- `greenwave_diagnostics.<topic>.expected_frequency` - Expected publish rate in Hz
- `greenwave_diagnostics.<topic>.tolerance` - Tolerance percentage (default: 5.0%)

**Usage Examples**

To set the expected frequency for a topic:
```bash
ros2 param set /greenwave_monitor greenwave_diagnostics./topic2.expected_frequency 30.0
ros2 param set /greenwave_monitor greenwave_diagnostics./topic2.tolerance 10.0
```

Parameters can also be set at launch time via YAML:
```yaml
greenwave_monitor:
  ros__parameters:
    greenwave_diagnostics./topic2.expected_frequency: 30.0
    greenwave_diagnostics./topic2.tolerance: 10.0
    greenwave_diagnostics./topic2.enabled: true
```

Or via command line:
```bash
ros2 run greenwave_monitor greenwave_monitor --ros-args \
  -p greenwave_diagnostics./topic2.expected_frequency:=30.0 \
  -p greenwave_diagnostics./topic2.tolerance:=10.0
```

Note: The topic name must include the leading slash (e.g., '/topic2' not 'topic2').
