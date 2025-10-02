# Greenwave Monitor

A high-performance diagnostic tool for ROS 2 that provides real-time monitoring of topic frame rates and latency metrics. The monitor node is like a more performant `ros2 topic hz` with ROS2 Diagnostics output, and services to manage topics. The dashboard is a terminal-based interface that provides a real-time view of the monitoring data, as well as displaying diagnostic data published by sensor drivers.

The diagnostics messages follow conventions from [Isaac ROS NITROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros), that means configured NITROS nodes can be monitored with the same tool without additional subscriber overhead. For example the drivers from [Isaac ROS NOVA](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nova) can be monitored with the same tool (also, did you know you can enable all NITROS nodes to publish diagnostics? see the bottom of [this](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/isaac_ros_nitros/index.html) page).

Finally, we provide monitoring code as a standalone C++ header, which can be integrated into your own nodes.

## Dashboard Interfaces

The Greenwave Monitor provides two terminal-based dashboard options:

### r2s_gw Dashboard (Rich TUI)
Ships with a fork of [r2s](https://github.com/mjcarroll/r2s) to provide an intuitive terminal interface that displays real-time statistics for all monitored topics. Click around with your mouse and tab key, and see the bottom bar with keyboard shortcuts.

![r2s_gw + Greenwave Monitor Dashboard](docs/images/greenwave_r2s_dashboard.png)

### ncurses Dashboard (Lightweight)
A lightweight ncurses-based interface focused specifically on topic monitoring with keyboard navigation. Features color-coded status indicators, interactive frequency management, and filtering options.

## Key Features

- **High Performance**: Up to 10x more CPU efficient than the built-in `ros2 topic hz` command
- **Multiple Dashboard Options**: Choose between rich TUI (r2s_gw) or lightweight ncurses interface
- **Real-time Monitoring**: Live visualization of topic rates, latency, and status with color-coded indicators
- **Interactive Management**: Add/remove topics and set expected frequencies directly from the interface
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

### Monitor Dashboards

The easiest way to use Greenwave Monitor is with one of the all-in-one dashboard commands, which provide visibility into all diagnostics, including those from drivers that publish their own diagnostic data.

#### r2s_gw Dashboard (Rich TUI)

```bash
ros2 run greenwave_monitor r2s_gw_dashboard
```

Features:
- Mouse and keyboard navigation
- Tabbed interface (Topics, Nodes, Interfaces)
- Rich text rendering and charts
- Comprehensive ROS 2 system overview

#### ncurses Dashboard (Lightweight)

```bash
ros2 run greenwave_monitor ncurses_dashboard
```

Features:
- Keyboard-only navigation (↑/↓ arrows, Enter, etc.)
- Real-time topic monitoring with color-coded status
- Interactive frequency management (`f` to set, `c` to clear)
- Topic filtering (`h` to hide unmonitored topics)
- Minimal resource usage

##### ncurses Dashboard Controls

- **↑/↓ arrows**: Navigate topics
- **Enter/Space**: Toggle monitoring for selected topic
- **f**: Set expected frequency for selected topic
- **c**: Clear expected frequency for selected topic
- **h**: Toggle between showing all topics vs monitored only
- **q**: Quit

Both commands:
- Start the greenwave monitor node in the background
- Launch the respective frontend in the foreground
- Automatically handle cleanup when you exit

#### Dashboard Options

```bash
# Launch with demo publisher nodes
ros2 run greenwave_monitor r2s_gw_dashboard --demo
ros2 run greenwave_monitor ncurses_dashboard --demo

# Enable logging to a directory
ros2 run greenwave_monitor r2s_gw_dashboard --log-dir /path/to/logs
ros2 run greenwave_monitor ncurses_dashboard --log-dir /path/to/logs
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
