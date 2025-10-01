# Greenwave Monitor

A high-performance diagnostic tool for ROS 2 that provides real-time monitoring of topic frame rates and latency metrics. The monitor node is like a more performant `ros2 topic hz` with ROS2 Diagnostics output, and services to manage topics. The dashboard is a terminal-based interface that provides a real-time view of the monitoring data, as well as displaying diagnostic data published by sensor drivers.

The diagnostics messages follow conventions from [Isaac ROS NITROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros), that means configured NITROS nodes can be monitored with the same tool without additional subscriber overhead. For example the drivers from [Isaac ROS NOVA](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nova) can be monitored with the same tool (also, did you know you can enable all NITROS nodes to publish diagnostics? see the bottom of [this](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/isaac_ros_nitros/index.html) page).

Finally, we provide monitoring code as a standalone C++ header, which can be integrated into your own nodes.

## Dashboard Interface

The Greenwave Monitor ships with a fork of [r2s](https://github.com/mjcarroll/r2s) to provide an intuitive terminal interface that displays real-time statistics for all monitored topics. Click around with your mouse and tab key, and see the bottom bar with keyboard shortcuts.

![r2s_gw + Greenwave Monitor Dashboard](docs/images/greenwave_r2s_dashboard.png)

## Key Features

- **High Performance**: Up to 10x more CPU efficient than the built-in `ros2 topic hz` command
- **Interactive Dashboard**: Real-time visualization of monitoring data with an easy-to-use terminal interface
- **Dynamic Topic Management**: Add or remove topics from monitoring without restarting
- **Topic Status Indicators**: Color-coded topic status to easily catch topics that are not working as expected
- **Universal Compatibility**: Displays diagnostics from any source that publishes a compatible `/diagnostics` topic, including NVIDIA Nova sensors and other hardware drivers

## Installation
TODO

```bash
cd ros_ws/src
git clone https://gitlab-master.nvidia.com/sgillen/ros2_monitor_node.git
cd ..
colcon build --symlink-install --packages-up-to r2s_gw
pip3 install -r requirements.txt --break-system-packages
source install/setup.sh
```

## Usage

### Monitor Dashboard (Recommended)

The easiest way to use Greenwave Monitor is with the all-in-one dashboard, which provides visibility into all diagnostics, including those from drivers that publish their own diagnostic data:

```bash
ros2 run greenwave_monitor r2s_gw_dashboard
```

This command:
- Starts the greenwave monitor node in the background
- Launches the r2s_gw TUI frontend in the foreground
- Automatically handles cleanup when you exit

#### Dashboard Options

```bash
# Launch with demo publisher nodes
ros2 run greenwave_monitor r2s_gw_dashboard --demo

# Enable logging to a directory
ros2 run greenwave_monitor r2s_gw_dashboard --log-dir /path/to/logs
```

### Manual Launch (ros2 topic hz mode)

If you want to use the tool as C++ based ros2 topic hz, you can do so with the following:

```bash
ros2 launch greenwave_monitor hz.launch.py topics:='["/topic1", "/topic2"]'
```

## Services

The Greenwave Monitor provides two services. The `ManageTopic` service dynamically adds or removes topics from monitoring. The `SetExpectedFrequency` service dynamically sets or clears expected frequencies for a specified topic, which enables additional diagnostic values and statuses.

### Manage Topic

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

### Set Expected Frequency

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

## r2s_gw Dashboard

The r2s_gw dashboard displays real-time information about ROS 2 interfaces, nodes, and topics within your ROS environment. The dashboard can be navigated using key bindings or using a mouse. The full list of available key bindings for each tab can be found in the dashboard footer.
